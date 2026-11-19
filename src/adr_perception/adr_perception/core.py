"""ROS-free processing core of gatenet_node.

Everything here runs without rclpy so it can be tested on a Mac / plain Python.
The image path itself is NOT reimplemented: every stage is the function from
handoff_boseong_v2/gatenet_rt/pipeline.py (copied unchanged into gatenet_rt/).

    frame (BGR, 1640x1232 or 320x240)
      -> pipeline.scale_cpu      (INTER_AREA to 320x240; 320x240 passes through)
      -> pipeline.Undistort      (remap with configs/remap_320x240.npz)
      -> pipeline.preprocess     (RGB float /255, 1x3x240x320)
      -> backend                 (pipeline.TrtBackend, else pipeline.OnnxBackend)
      -> pipeline.postprocess    (sigmoid >= 0.5 -> 240x320 {0,1})
      -> pipeline.to64           (INTER_AREA 64x64 -> >= 0.5 -> {0,255})
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass

import numpy as np

from .gatenet_rt import pipeline as P
from .gatenet_rt.camera320 import Camera320

SENSOR_WH = (P.SENSOR_W, P.SENSOR_H)
NET_WH = (P.NET_W, P.NET_H)


# --------------------------------------------------------------------------
# remap: npz must equal what gatenet_K.yaml + calibration give
# --------------------------------------------------------------------------
def check_remap(remap_file: str, gatenet_K_file: str, tol: float = 1e-3) -> dict:
    """Camera320(gatenet_K_file) re-verifies every K (raises on mismatch) and
    rebuilds the 320x240 map; the npz must match it. Returns a small report."""
    cam = Camera320(gatenet_K_file)
    mx, my = cam.maps(1)
    with np.load(remap_file, allow_pickle=False) as z:
        d = float(max(np.abs(z["map_x"] - mx).max(), np.abs(z["map_y"] - my).max()))
    if d > tol:
        raise RuntimeError(f"{remap_file} does not match {gatenet_K_file} "
                           f"(max map diff {d:.4g} px); regenerate the remap with the selected calibration")
    return {"calib_file": cam.calib_path, "target_K": cam.cfg["target_K"],
            "max_map_diff_px": d, "margins_px": cam.margins(1)}


# --------------------------------------------------------------------------
# backend: TensorRT engine first, else ONNX Runtime
# --------------------------------------------------------------------------
def make_backend(engine_file: str | None, onnx_file: str | None, log=print):
    """Create in the thread that will run inference (pycuda.autoinit binds the
    CUDA context to the creating thread)."""
    if engine_file and os.path.isfile(engine_file):
        try:
            b = P.TrtBackend(engine_file)
            log(f"backend: {b.info}")
            return b
        except Exception as e:      # tensorrt / pycuda missing, wrong TRT version ...
            log(f"TensorRT engine not usable ({type(e).__name__}: {e}); falling back to ONNX Runtime")
    elif engine_file:
        log(f"no TensorRT engine at {engine_file}; using ONNX Runtime")
    if not onnx_file or not os.path.isfile(onnx_file):
        raise FileNotFoundError(f"ONNX model not found: {onnx_file}")
    b = P.OnnxBackend(onnx_file)
    log(f"backend: {b.info}")
    return b


# --------------------------------------------------------------------------
# one frame
# --------------------------------------------------------------------------
@dataclass
class Result:
    mask64: np.ndarray       # 64x64 uint8 {0,255}
    mask_full: np.ndarray    # 240x320 uint8 {0,255}, undistorted (gatenet_K) frame
    small: np.ndarray        # 240x320 BGR as it entered undistort (= recorded frame)
    rectified: np.ndarray    # 240x320 BGR in the same pixel frame as mask_full
    infer_ms: float          # scale..to64 wall time


class GateNetCore:
    def __init__(self, remap_file: str, backend):
        self.undistort = P.Undistort(remap_file)
        self.backend = backend

    def process(self, frame_bgr: np.ndarray) -> Result:
        t0 = time.perf_counter()
        small = P.scale_cpu(frame_bgr)
        rectified = self.undistort(small)
        m01 = P.postprocess(self.backend(P.preprocess(rectified)))
        m64 = P.to64(m01)
        return Result(m64, (m01 * 255).astype(np.uint8), small, rectified,
                      (time.perf_counter() - t0) * 1e3)


def input_size_ok(h: int, w: int) -> bool:
    """The calibration is the full-FOV 1640x1232 mode; anything else 4:3 is
    only valid if it is that mode scaled (e.g. 320x240 from nvvidconv)."""
    return (w, h) in (SENSOR_WH, NET_WH)


# --------------------------------------------------------------------------
# keep only the newest frame
# --------------------------------------------------------------------------
class LatestSlot:
    """Single-slot mailbox: put() overwrites, get() waits. Frames that arrive
    while the worker is busy are dropped (counted), only the newest survives."""

    def __init__(self):
        self._cv = threading.Condition()
        self._item = None
        self.dropped = 0
        self.closed = False

    def put(self, item):
        with self._cv:
            if self._item is not None:
                self.dropped += 1
            self._item = item
            self._cv.notify()

    def get(self, timeout: float = 0.5):
        with self._cv:
            if self._item is None and not self.closed:
                self._cv.wait(timeout)
            item, self._item = self._item, None
            return item

    def close(self):
        with self._cv:
            self.closed = True
            self._cv.notify_all()
