"""PyTorch DreamerV3 policy from the deployment handoff (deploy/onboard.py)."""
from __future__ import annotations

import json
import pathlib

import numpy as np
import torch
from torch.nn import functional as F


def _silu(x):
    return F.silu(x)


def _sig(x):
    return torch.sigmoid(x)


def _rms(x, s):
    return x / torch.sqrt(x.square().mean(-1, keepdim=True) + 1e-4) * s


def _symlog(x):
    return torch.sign(x) * torch.log1p(torch.abs(x))


def _symexp(x):
    return torch.sign(x) * (torch.exp(torch.abs(x)) - 1.0)


class Policy:
    """DreamerV3 policy step (encoder -> RSSM -> actor, + decoded gate position), float32 PyTorch.
    Same maths as Agent.policy(mode="eval"); export_policy.py proves it against JAX."""

    def __init__(self, bundle, seed=0, device=None):
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        bundle = pathlib.Path(bundle)
        self.meta = json.loads((bundle / "policy_meta.json").read_text())
        with np.load(bundle / "policy_weights.npz") as z:
            self.w = {k.replace("|", "/"): torch.as_tensor(z[k], dtype=torch.float32, device=self.device) for k in z.files}
        d = self.meta["dims"]
        self.D, self.S, self.C, self.G = d["deter"], d["stoch"], d["classes"], d["blocks"]
        self.unimix = d["unimix"]
        self.n_cnn, self.n_enc, self.n_pol, self.n_dec = len(d["mults"]), d["enc_layers"], d["pol_layers"], d["dec_layers"]
        self.vec_keys, self.vec_dims = self.meta["enc_vec_keys"], self.meta["enc_vec_dims"]
        self.rng = np.random.default_rng(seed)
        self._conv_weights = [
            self.w[f"enc/cnn{i}/kernel"].permute(3, 2, 0, 1).contiguous()
            for i in range(self.n_cnn)
        ]
        self.reset()

    def reset(self, batch=1):
        self.deter = torch.zeros((batch, self.D), device=self.device)
        self.stoch = torch.zeros((batch, self.S * self.C), device=self.device)
        self.prev = torch.zeros((batch, 4), device=self.device)
        self._tok = None

    def _lin(self, x, n):
        return x @ self.w[f"{n}/kernel"] + self.w[f"{n}/bias"]

    def _dense(self, x, n, norm):
        return _silu(_rms(self._lin(x, n), self.w[f"{norm}/scale"]))

    def _block(self, x, n):
        k = self.w[f"{n}/kernel"]
        y = torch.einsum("bki,kio->bko", x.reshape(x.shape[0], k.shape[0], -1), k)
        return y.reshape(x.shape[0], -1) + self.w[f"{n}/bias"]

    def encode_mask(self, mask):
        x = torch.as_tensor(mask, dtype=torch.float32, device=self.device)
        x = x.reshape(-1, 1, 64, 64) / 255.0 - 0.5
        for i, kernel in enumerate(self._conv_weights):
            x = F.conv2d(x, kernel, self.w[f"enc/cnn{i}/bias"],
                         padding=(kernel.shape[2] // 2, kernel.shape[3] // 2))
            x = F.max_pool2d(x, 2)
            x = _silu(_rms(x.permute(0, 2, 3, 1), self.w[f"enc/cnn{i}norm/scale"]))
            x = x.permute(0, 3, 1, 2)
        return x.permute(0, 2, 3, 1).reshape(x.shape[0], -1)

    @torch.inference_mode()
    def step(self, mask, vec, is_first, mask_id=None, gumbel=None):
        """mask (64,64) or (B,64,64) uint8; vec (32,) or (B,32) in meta["enc_vec_keys"] order
        (acc, att, flight_plan, rates).  Returns action (B,4) and decoded gate position p_g (B,3)."""
        vec = torch.as_tensor(vec, dtype=torch.float32, device=self.device).reshape(-1, sum(self.vec_dims))
        b = vec.shape[0]
        if self.deter.shape[0] != b:
            self.reset(b)
        keep = (~torch.as_tensor(is_first, dtype=torch.bool, device=self.device).reshape(-1))[:, None]
        deter, stoch, prev = self.deter * keep, self.stoch * keep, self.prev * keep
        # encoder (the CNN is skipped while the camera frame is the same)
        if mask_id is not None and self._tok is not None and self._tok[0] == mask_id:
            img = self._tok[1]
        else:
            img = self.encode_mask(mask)
            self._tok = (mask_id, img) if mask_id is not None else None
        if gumbel is None:
            gumbel = -np.log(-np.log(self.rng.uniform(1e-9, 1.0 - 1e-9, (b, self.S, self.C))))
        noise = torch.as_tensor(gumbel, device=self.device)
        action, p_g, self.deter, self.stoch = self.forward_state(img, vec, deter, stoch, prev, noise)
        self.prev = action
        return action.cpu().numpy(), p_g.cpu().numpy()

    def forward_state(self, img, vec, deter, stoch, prev, noise):
        b = vec.shape[0]
        v = _symlog(vec) if self.meta["enc_symlog"] else vec
        for i in range(self.n_enc):
            v = self._dense(v, f"enc/mlp{i}", f"enc/mlp{i}norm")
        tokens = torch.cat([v, img], -1)
        # RSSM core (block GRU)
        a = prev / prev.abs().clamp_min(1.0)
        x = torch.cat([self._dense(deter, "dyn/dynin0", "dyn/dynin0norm"),
                            self._dense(stoch, "dyn/dynin1", "dyn/dynin1norm"),
                            self._dense(a, "dyn/dynin2", "dyn/dynin2norm")], -1)
        x = torch.cat([deter.reshape(b, self.G, -1), x[:, None, :].expand(-1, self.G, -1)], -1).reshape(b, -1)
        x = _silu(_rms(self._block(x, "dyn/dynhid0"), self.w["dyn/dynhid0norm/scale"]))
        r, c, u = self._block(x, "dyn/dyngru").reshape(b, self.G, -1).chunk(3, dim=-1)
        cand = torch.tanh(_sig(r.reshape(b, -1)) * c.reshape(b, -1))
        upd = _sig(u.reshape(b, -1) - 1.0)
        deter = upd * cand + (1.0 - upd) * deter
        # posterior: sample the latent (as in training and evaluation)
        x = self._dense(torch.cat([deter, tokens], -1), "dyn/obs0", "dyn/obs0norm")
        logit = self._lin(x, "dyn/obslogit").reshape(b, self.S, self.C)
        p = (1.0 - self.unimix) * logit.softmax(dim=-1) + self.unimix / self.C
        idx = (p.log() + noise).argmax(dim=-1, keepdim=True)
        stoch = torch.zeros_like(p).scatter_(-1, idx, 1.0)
        stoch = stoch.reshape(b, -1)
        # actor: deterministic action
        x = torch.cat([deter, stoch], -1)
        for i in range(self.n_pol):
            x = self._dense(x, f"pol/mlp/linear{i}", f"pol/mlp/norm{i}")
        action = torch.tanh(self._lin(x, "pol/head/action/mean"))
        # decoder: where the world model thinks the drone is, in the current gate's frame
        x = torch.cat([stoch, deter], -1)
        for i in range(self.n_dec):
            x = self._dense(x, f"dec/mlp/linear{i}", f"dec/mlp/norm{i}")
        p_g = self._lin(x, "dec/vec/info_p_g/pred")
        p_g = _symexp(p_g) if self.meta["dec_symlog"] else p_g
        return action, p_g, deter, stoch
