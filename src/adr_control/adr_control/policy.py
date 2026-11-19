"""NumPy DreamerV3 policy from the deployment handoff (deploy/onboard.py)."""
from __future__ import annotations

import json
import pathlib

import numpy as np


def _silu(x):
    return x / (1.0 + np.exp(-x))


def _sig(x):
    return 1.0 / (1.0 + np.exp(-x))


def _rms(x, s):
    return x / np.sqrt(np.mean(x * x, -1, keepdims=True) + 1e-4) * s


def _symlog(x):
    return np.sign(x) * np.log1p(np.abs(x))


def _symexp(x):
    return np.sign(x) * np.expm1(np.abs(x))


class Policy:
    """DreamerV3 policy step (encoder -> RSSM -> actor, + decoded gate position), float32 numpy.
    Same maths as Agent.policy(mode="eval"); export_policy.py proves it against JAX."""

    def __init__(self, bundle, seed=0):
        bundle = pathlib.Path(bundle)
        self.meta = json.loads((bundle / "policy_meta.json").read_text())
        with np.load(bundle / "policy_weights.npz") as z:
            self.w = {k.replace("|", "/"): z[k].astype(np.float32) for k in z.files}
        d = self.meta["dims"]
        self.D, self.S, self.C, self.G = d["deter"], d["stoch"], d["classes"], d["blocks"]
        self.unimix = d["unimix"]
        self.n_cnn, self.n_enc, self.n_pol, self.n_dec = len(d["mults"]), d["enc_layers"], d["pol_layers"], d["dec_layers"]
        self.vec_keys, self.vec_dims = self.meta["enc_vec_keys"], self.meta["enc_vec_dims"]
        self.rng = np.random.default_rng(seed)
        self._ck = [self.w[f"enc/cnn{i}/kernel"].reshape(-1, self.w[f"enc/cnn{i}/kernel"].shape[-1])
                    for i in range(self.n_cnn)]
        self.reset()

    def reset(self, batch=1):
        self.deter = np.zeros((batch, self.D), np.float32)
        self.stoch = np.zeros((batch, self.S * self.C), np.float32)
        self.prev = np.zeros((batch, 4), np.float32)
        self._tok = None

    def _lin(self, x, n):
        return x @ self.w[f"{n}/kernel"] + self.w[f"{n}/bias"]

    def _dense(self, x, n, norm):
        return _silu(_rms(self._lin(x, n), self.w[f"{norm}/scale"]))

    def _block(self, x, n):
        k = self.w[f"{n}/kernel"]
        y = np.einsum("bki,kio->bko", x.reshape(x.shape[0], k.shape[0], -1), k)
        return y.reshape(x.shape[0], -1) + self.w[f"{n}/bias"]

    def encode_mask(self, mask):
        x = np.asarray(mask, np.float32).reshape(-1, 64, 64, 1) / 255.0 - 0.5
        for i in range(self.n_cnn):
            kh, kw, _, cout = self.w[f"enc/cnn{i}/kernel"].shape
            b, h, w, c = x.shape
            xp = np.pad(x, ((0, 0), (kh // 2,) * 2, (kw // 2,) * 2, (0, 0)))
            cols = np.empty((b, h, w, kh, kw, c), np.float32)
            for dy in range(kh):
                for dx in range(kw):
                    cols[:, :, :, dy, dx, :] = xp[:, dy:dy + h, dx:dx + w, :]
            x = (cols.reshape(b * h * w, -1) @ self._ck[i] + self.w[f"enc/cnn{i}/bias"]).reshape(b, h, w, cout)
            x = x.reshape(b, h // 2, 2, w // 2, 2, cout).max((2, 4))
            x = _silu(_rms(x, self.w[f"enc/cnn{i}norm/scale"]))
        return x.reshape(x.shape[0], -1)

    def step(self, mask, vec, is_first, mask_id=None, gumbel=None):
        """mask (64,64) or (B,64,64) uint8; vec (32,) or (B,32) in meta["enc_vec_keys"] order
        (acc, att, flight_plan, rates).  Returns action (B,4) and decoded gate position p_g (B,3)."""
        vec = np.asarray(vec, np.float32).reshape(-1, sum(self.vec_dims))
        b = vec.shape[0]
        if self.deter.shape[0] != b:
            self.reset(b)
        keep = (~np.asarray(is_first, bool).reshape(-1))[:, None].astype(np.float32)
        deter, stoch, prev = self.deter * keep, self.stoch * keep, self.prev * keep
        # encoder (the CNN is skipped while the camera frame is the same)
        if mask_id is not None and self._tok is not None and self._tok[0] == mask_id:
            img = self._tok[1]
        else:
            img = self.encode_mask(mask)
            self._tok = (mask_id, img) if mask_id is not None else None
        v = _symlog(vec) if self.meta["enc_symlog"] else vec
        for i in range(self.n_enc):
            v = self._dense(v, f"enc/mlp{i}", f"enc/mlp{i}norm")
        tokens = np.concatenate([v, img], -1)
        # RSSM core (block GRU)
        a = prev / np.maximum(1.0, np.abs(prev))
        x = np.concatenate([self._dense(deter, "dyn/dynin0", "dyn/dynin0norm"),
                            self._dense(stoch, "dyn/dynin1", "dyn/dynin1norm"),
                            self._dense(a, "dyn/dynin2", "dyn/dynin2norm")], -1)
        x = np.concatenate([deter.reshape(b, self.G, -1), x[:, None, :].repeat(self.G, 1)], -1).reshape(b, -1)
        x = _silu(_rms(self._block(x, "dyn/dynhid0"), self.w["dyn/dynhid0norm/scale"]))
        r, c, u = np.split(self._block(x, "dyn/dyngru").reshape(b, self.G, -1), 3, -1)
        cand = np.tanh(_sig(r.reshape(b, -1)) * c.reshape(b, -1))
        upd = _sig(u.reshape(b, -1) - 1.0)
        deter = upd * cand + (1.0 - upd) * deter
        # posterior: sample the latent (as in training and evaluation)
        x = self._dense(np.concatenate([deter, tokens], -1), "dyn/obs0", "dyn/obs0norm")
        logit = self._lin(x, "dyn/obslogit").reshape(b, self.S, self.C)
        self.last_logit = logit
        p = np.exp(logit - logit.max(-1, keepdims=True))
        p = (1.0 - self.unimix) * p / p.sum(-1, keepdims=True) + self.unimix / self.C
        if gumbel is None:
            gumbel = -np.log(-np.log(self.rng.uniform(1e-9, 1.0 - 1e-9, p.shape)))
        idx = np.argmax(np.log(p) + gumbel, -1)
        stoch = np.zeros((b, self.S, self.C), np.float32)
        np.put_along_axis(stoch, idx[..., None], 1.0, -1)
        stoch = stoch.reshape(b, -1)
        # actor: deterministic action
        x = np.concatenate([deter, stoch], -1)
        for i in range(self.n_pol):
            x = self._dense(x, f"pol/mlp/linear{i}", f"pol/mlp/norm{i}")
        action = np.tanh(self._lin(x, "pol/head/action/mean"))
        # decoder: where the world model thinks the drone is, in the current gate's frame
        x = np.concatenate([stoch, deter], -1)
        for i in range(self.n_dec):
            x = self._dense(x, f"dec/mlp/linear{i}", f"dec/mlp/norm{i}")
        p_g = self._lin(x, "dec/vec/info_p_g/pred")
        p_g = _symexp(p_g) if self.meta["dec_symlog"] else p_g
        self.deter, self.stoch, self.prev = deter, stoch, action
        return action, p_g


# =============================================================================== flight plan
class FlightPlan:
    """Which gate is next.  It is a policy INPUT (24 numbers of track geometry), not a pass/fail
    check: the index moves on when the decoded position along the gate normal x_g > -0.15 m
    (SkyDreamer II-H), re-arms once x_g < -0.5 m, at most once per 0.25 s."""

    def __init__(self, meta):
        plan = meta["flight_plan"]
        self.table = np.asarray(plan["table"], np.float32)
        self.start_index = plan.get("start_index", 0)
        if (self.table.ndim != 2 or not np.isfinite(self.table).all()
                or not isinstance(self.start_index, int)
                or not 0 <= self.start_index < len(self.table)):
            raise ValueError("Invalid flight-plan table or start_index")
        self.pass_x = float(plan.get("pass_x", -0.15))
        self.rearm_x = float(plan.get("rearm_x", -0.5))
        self.min_gap = float(plan.get("min_gap_s", 0.25))
        if (not np.isfinite([self.pass_x, self.rearm_x, self.min_gap]).all()
                or self.rearm_x >= self.pass_x or self.min_gap < 0):
            raise ValueError("Invalid flight-plan transition thresholds")
        self.reset()

    def reset(self):
        self.index, self.passed, self.armed, self.since = self.start_index, 0, True, 1e9

    @property
    def vector(self):
        return self.table[self.index]

    def update(self, x_g, dt):
        self.since += dt
        if not self.armed and x_g < self.rearm_x:
            self.armed = True
        if self.armed and x_g > self.pass_x and self.since >= self.min_gap:
            self.index = (self.index + 1) % len(self.table)
            self.passed += 1
            self.armed, self.since = False, 0.0
            return True
        return False
