"""320x240 undistort path. The ONE place that reads configs/gatenet_K.yaml.

    original 1640x1232 -> 320x240 (INTER_AREA / nvvidconv)
                       -> cv2.remap(map from source_K + dist  ->  target_K)
                       -> 240x320 pinhole image = GateNet input

Labels go through the same downscale and the same map with nearest sampling.
"""

from __future__ import annotations

import os

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(ROOT, "configs", "gatenet_K.yaml")
TOL = 1e-3


def _kmat(fx, fy, cx, cy) -> np.ndarray:
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


def load_calibration(path: str) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    with open(path) as f:
        y = yaml.safe_load(f)
    c = y["camera"]["calibration"]
    K = np.array(c["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    d = np.array(c["distortion_coefficients"]["data"], dtype=np.float64)
    return K, d, (int(c["image_width"]), int(c["image_height"]))


def scale_K(K: np.ndarray, src_wh, dst_wh) -> np.ndarray:
    """Rescale with the half-pixel convention: (u + 0.5) * s - 0.5."""
    sx, sy = dst_wh[0] / src_wh[0], dst_wh[1] / src_wh[1]
    out = K.copy()
    out[0, 0] *= sx
    out[1, 1] *= sy
    out[0, 2] = (K[0, 2] + 0.5) * sx - 0.5
    out[1, 2] = (K[1, 2] + 0.5) * sy - 0.5
    return out


class Camera320:
    def __init__(self, config: str = CONFIG, check: bool = True):
        with open(config) as f:
            self.cfg = yaml.safe_load(f)
        base = os.path.dirname(os.path.abspath(config))
        self.calib_path = os.path.join(base, self.cfg["calibration_file"])
        Kc, self.dist, wh = load_calibration(self.calib_path)
        if tuple(wh) != tuple(self.cfg["calibration_size"]):
            raise ValueError(f"calibration size {wh} != {self.cfg['calibration_size']}")
        self.K_calib, self.calib_wh = Kc, tuple(wh)
        t = self.cfg["target_K"]
        self.size = tuple(t["size"])                    # (320, 240)
        self.K_target = _kmat(t["fx"], t["fy"], t["cx"], t["cy"])
        self.K_source = scale_K(Kc, wh, self.size)
        if check:
            self.verify()

    # ------------------------------------------------------------------ #
    def verify(self) -> dict:
        e = self.cfg["source_K_320x240_expected"]
        got = dict(fx=self.K_source[0, 0], fy=self.K_source[1, 1],
                   cx=self.K_source[0, 2], cy=self.K_source[1, 2])
        bad = {k: (got[k], e[k]) for k in e if abs(got[k] - e[k]) > TOL}
        if bad:
            raise ValueError(f"source K mismatch (got, expected): {bad}")
        p = self.cfg["policy_K_64_expected"]
        K64 = self.policy_K()
        got64 = dict(fx=K64[0, 0], fy=K64[1, 1], cx_sim=K64[0, 2] + 0.5,
                     cy_sim=K64[1, 2] + 0.5)
        bad = {k: (got64[k], p[k]) for k in got64 if abs(got64[k] - p[k]) > TOL}
        if bad:
            raise ValueError(f"64x64 K mismatch (got, expected): {bad}")
        return {"source": got, "policy64": got64}

    def policy_K(self) -> np.ndarray:
        return scale_K(self.K_target, self.size, (64, 64))

    # ------------------------------------------------------------------ #
    def maps(self, scale: int = 1) -> tuple[np.ndarray, np.ndarray]:
        """float32 maps for a (320*scale)x(240*scale) target from a same-scale source."""
        w, h = self.size[0] * scale, self.size[1] * scale
        Ks = scale_K(self.K_calib, self.calib_wh, (w, h))
        Kt = scale_K(self.K_target, self.size, (w, h))
        return cv2.initUndistortRectifyMap(Ks, self.dist, None, Kt, (w, h),
                                           cv2.CV_32FC1)

    def margins(self, scale: int = 1) -> dict:
        """How far inside the source image the map stays (px, source scale)."""
        mx, my = self.maps(scale)
        w, h = self.size[0] * scale, self.size[1] * scale
        return {"left": float(mx.min()), "right": float((w - 1) - mx.max()),
                "top": float(my.min()), "bottom": float((h - 1) - my.max())}

    # ------------------------------------------------------------------ #
    def downscale(self, img: np.ndarray, scale: int = 1) -> np.ndarray:
        w, h = self.size[0] * scale, self.size[1] * scale
        return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)

    def undistort_image(self, small: np.ndarray, maps=None) -> np.ndarray:
        mx, my = maps if maps is not None else self.maps()
        return cv2.remap(small, mx, my, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def undistort_label(self, full_mask01: np.ndarray, scale: int = 1, maps=None
                        ) -> np.ndarray:
        """Full-res {0,1} mask -> same INTER_AREA downscale (>=0.5) -> nearest remap."""
        small = self.downscale(full_mask01.astype(np.float32), scale)
        small = (small >= 0.5).astype(np.uint8)
        mx, my = maps if maps is not None else self.maps(scale)
        return cv2.remap(small, mx, my, cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def frame_to_input(self, full_bgr: np.ndarray, maps=None) -> np.ndarray:
        """File-input path: original -> INTER_AREA 320x240 -> remap. uint8 BGR 240x320."""
        return self.undistort_image(self.downscale(full_bgr), maps)


def to_64(prob_or_mask: np.ndarray) -> np.ndarray:
    """240x320 {0,1}/prob -> 64x64 uint8 {0,255} (INTER_AREA then 0.5)."""
    s = cv2.resize(prob_or_mask.astype(np.float32), (64, 64), interpolation=cv2.INTER_AREA)
    return np.where(s >= 0.5, 255, 0).astype(np.uint8)


__all__ = ["Camera320", "load_calibration", "scale_K", "to_64", "CONFIG"]
