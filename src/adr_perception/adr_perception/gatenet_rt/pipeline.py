"""GateNet flight path v2 (320x240, undistort first), stage by stage.

  1 capture     nvarguscamerasrc 1640x1232@30 (full-FOV 2x2 binned mode)
  2 scale       nvvidconv on the ISP -> 320x240   |  file input: cv2 INTER_AREA
  3 undistort   ONE cv2.remap (linear) with the precomputed map
                (320x240 source K + distortion  ->  configs/gatenet_K.yaml target K)
  4 preprocess  BGR uint8 -> RGB float32 /255 -> 1x3x240x320
  5 gatenet     torch | onnx | trt, highest-resolution LOGIT only
  6 postprocess sigmoid >= 0.5  ->  240x320 {0,1}
  7 to64        INTER_AREA 64x64 -> >= 0.5 -> uint8 {0,255}
                (4:3 stretched to square, no blank rows: fx 52.015, fy 69.254)

Every stage is a function/object so bench.py can time each one.
"""

from __future__ import annotations

import csv
import os
import queue
import threading
import time

import cv2
import numpy as np

NET_W, NET_H = 320, 240
SENSOR_W, SENSOR_H = 1640, 1232
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_REMAP = os.path.join(HERE, "configs", "remap_320x240.npz")


# --------------------------------------------------------------------------
# 1-2 capture / scale
# --------------------------------------------------------------------------
def gst_pipeline(sensor_id: int = 0, exposure_ms: float | None = None,
                 hw_scale: bool = True, fps: int = 30, interp: int | None = None) -> str:
    """`exposure_ms` locks exposure AND gain (gain alone would keep moving).

    `interp` = nvvidconv interpolation-method (0 nearest, 1 bilinear, 2 5-tap,
    3 10-tap, 4 smart, 5 nicest). None leaves the plugin default.
    """
    src = f"nvarguscamerasrc sensor-id={sensor_id}"
    if exposure_ms is not None:
        ns = int(exposure_ms * 1e6)
        src += (f' exposuretimerange="{ns} {ns}"'
                ' gainrange="1 1" ispdigitalgainrange="1 1" aelock=true')
    caps = (f"video/x-raw(memory:NVMM),width={SENSOR_W},height={SENSOR_H},"
            f"framerate={fps}/1,format=NV12")
    im = f" interpolation-method={interp}" if interp is not None else ""
    if hw_scale:
        conv = f"nvvidconv{im} ! video/x-raw,width={NET_W},height={NET_H},format=BGRx"
    else:
        conv = "nvvidconv ! video/x-raw,format=BGRx"
    return (f"{src} ! {caps} ! {conv} ! videoconvert ! video/x-raw,format=BGR "
            f"! appsink drop=1 max-buffers=1 sync=false")


class Camera:
    """Returns (frame_bgr, (t_read_start, t_read_end), sensor_ts_s_or_None)."""

    def __init__(self, sensor_id=0, exposure_ms=None, hw_scale=True, fps=30, interp=None):
        self.hw_scale = hw_scale
        self.pipeline = gst_pipeline(sensor_id, exposure_ms, hw_scale, fps, interp)
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("could not open nvarguscamerasrc (CSI camera present? "
                               "OpenCV built with GStreamer?)\n" + self.pipeline)

    def read(self):
        t0 = time.perf_counter()
        ok, frame = self.cap.read()
        t1 = time.perf_counter()
        if not ok:
            raise RuntimeError("camera read failed")
        ts = self.cap.get(cv2.CAP_PROP_POS_MSEC)   # usually 0 on the Argus path
        return frame, (t0, t1), (ts / 1000.0 if ts and ts > 0 else None)

    def release(self):
        self.cap.release()


def scale_cpu(frame_bgr: np.ndarray) -> np.ndarray:
    """File/full-frame input: 1640x1232 -> 320x240 INTER_AREA (= training)."""
    if frame_bgr.shape[:2] == (NET_H, NET_W):
        return frame_bgr
    return cv2.resize(frame_bgr, (NET_W, NET_H), interpolation=cv2.INTER_AREA)


# --------------------------------------------------------------------------
# 3 undistort
# --------------------------------------------------------------------------
class Undistort:
    def __init__(self, path: str = DEFAULT_REMAP):
        z = np.load(path, allow_pickle=False)
        self.map_x = z["map_x"].astype(np.float32)
        self.map_y = z["map_y"].astype(np.float32)
        self.meta = {k: z[k].tolist() for k in z.files if k not in ("map_x", "map_y")}

    def __call__(self, small_bgr: np.ndarray) -> np.ndarray:
        return cv2.remap(small_bgr, self.map_x, self.map_y, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)


# --------------------------------------------------------------------------
# 4 preprocess
# --------------------------------------------------------------------------
def preprocess(img_bgr: np.ndarray) -> np.ndarray:
    rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return np.ascontiguousarray(rgb.transpose(2, 0, 1))[None]


# --------------------------------------------------------------------------
# 5 backends: (1,3,240,320) float32 -> (240,320) float32 logits
# --------------------------------------------------------------------------
class TorchBackend:
    def __init__(self, ckpt: str, device: str = "cuda", fp16: bool = False):
        import torch
        from models.gatenet import build_model
        self.torch = torch
        ck = torch.load(ckpt, map_location="cpu", weights_only=False)
        self.net = build_model(width_factor=ck.get("width_factor", 4.0))
        self.net.load_state_dict(ck["model"])
        self.net.eval().to(device)
        self.device, self.fp16 = device, fp16
        if fp16:
            self.net.half()
        self.info = f"torch {torch.__version__} {device}{' fp16' if fp16 else ''} epoch {ck.get('epoch')}"

    def __call__(self, x):
        t = self.torch.from_numpy(x).to(self.device)
        if self.fp16:
            t = t.half()
        with self.torch.no_grad():
            z = self.net(t, return_logits=True)[-1]
        return z.float().cpu().numpy()[0, 0]     # the copy back forces sync


class OnnxBackend:
    def __init__(self, path: str, providers=None):
        import onnxruntime as ort
        if providers is None:
            avail = ort.get_available_providers()
            providers = [p for p in ("CUDAExecutionProvider", "CPUExecutionProvider")
                         if p in avail]
        self.sess = ort.InferenceSession(path, providers=providers)
        self.iname = self.sess.get_inputs()[0].name
        self.info = f"onnxruntime {ort.__version__} {self.sess.get_providers()}"

    def __call__(self, x):
        return self.sess.run(None, {self.iname: x})[0][0, 0]


class TrtBackend:
    """Engine from trtexec (README). TensorRT >= 8.5 tensor API + pycuda."""

    def __init__(self, engine_path: str):
        import tensorrt as trt
        import pycuda.autoinit  # noqa: F401
        import pycuda.driver as cuda
        self.cuda = cuda
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        self.inp = next(n for n in names
                        if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
        self.outp = next(n for n in names
                         if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT)
        self.oshape = tuple(self.ctx.get_tensor_shape(self.outp))
        self.d_in = cuda.mem_alloc(1 * 3 * NET_H * NET_W * 4)
        self.d_out = cuda.mem_alloc(int(np.prod(self.oshape)) * 4)
        self.h_out = np.empty(self.oshape, np.float32)
        self.stream = cuda.Stream()
        self.info = f"tensorrt {trt.__version__} {engine_path}"

    def __call__(self, x):
        self.cuda.memcpy_htod_async(self.d_in, np.ascontiguousarray(x, np.float32), self.stream)
        self.ctx.set_tensor_address(self.inp, int(self.d_in))
        self.ctx.set_tensor_address(self.outp, int(self.d_out))
        self.ctx.execute_async_v3(self.stream.handle)
        self.cuda.memcpy_dtoh_async(self.h_out, self.d_out, self.stream)
        self.stream.synchronize()
        return self.h_out[0, 0]


def make_backend(kind, ckpt, onnx, engine, device, fp16=False):
    if kind == "torch":
        return TorchBackend(ckpt, device, fp16)
    if kind == "onnx":
        return OnnxBackend(onnx)
    if kind == "trt":
        if not engine:
            raise SystemExit("--backend trt needs --engine <.plan>")
        return TrtBackend(engine)
    raise SystemExit(kind)


# --------------------------------------------------------------------------
# 6-7 postprocess / to64
# --------------------------------------------------------------------------
def postprocess(logits: np.ndarray, threshold: float = 0.5) -> np.ndarray:
    """logits -> {0,1} uint8. sigmoid(z) >= 0.5  <=>  z >= 0."""
    prob = 1.0 / (1.0 + np.exp(-logits.astype(np.float32)))
    return (prob >= threshold).astype(np.uint8)


def to64(mask01: np.ndarray, threshold: float = 0.5) -> np.ndarray:
    s = cv2.resize(mask01.astype(np.float32), (64, 64), interpolation=cv2.INTER_AREA)
    return np.where(s >= threshold, 255, 0).astype(np.uint8)


def frame_to_64(frame_bgr, undistort: Undistort, backend, hw_scaled=False):
    """Whole path for one frame (file input when hw_scaled=False)."""
    small = frame_bgr if hw_scaled else scale_cpu(frame_bgr)
    u = undistort(small)
    m = postprocess(backend(preprocess(u)))
    return to64(m), m, u


# --------------------------------------------------------------------------
# recording
# --------------------------------------------------------------------------
class Recorder:
    """Background writer: raw frame + 64x64 mask + timestamps.

    frames/<idx>.jpg  the frame exactly as it entered stage 3 or, with
                      --record-full, the 1640x1232 original
    masks64/<idx>.png the 64x64 {0,255} policy mask
    index.csv         idx, t_mono_s (read end), t_wall_s, sensor_ts_s, files
    Writes happen on a thread with a bounded queue; a full queue DROPS the
    record (counted in dropped) rather than stalling the flight loop.
    """

    def __init__(self, out_dir: str, jpeg_quality: int = 95, max_queue: int = 64):
        self.out = out_dir
        os.makedirs(os.path.join(out_dir, "frames"), exist_ok=True)
        os.makedirs(os.path.join(out_dir, "masks64"), exist_ok=True)
        self.q: queue.Queue = queue.Queue(maxsize=max_queue)
        self.q_jpeg = int(jpeg_quality)
        self.dropped = 0
        self.f = open(os.path.join(out_dir, "index.csv"), "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["idx", "t_mono_s", "t_wall_s", "sensor_ts_s", "frame", "mask64",
                         "frame_w", "frame_h"])
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def put(self, idx, frame, mask64, t_mono, sensor_ts):
        try:
            self.q.put_nowait((idx, frame.copy(), mask64.copy(), t_mono, time.time(), sensor_ts))
        except queue.Full:
            self.dropped += 1

    def _run(self):
        while True:
            item = self.q.get()
            if item is None:
                break
            idx, fr, m, tm, tw, ts = item
            fn = f"frames/{idx:07d}.jpg"
            mn = f"masks64/{idx:07d}.png"
            cv2.imwrite(os.path.join(self.out, fn), fr, [cv2.IMWRITE_JPEG_QUALITY, self.q_jpeg])
            cv2.imwrite(os.path.join(self.out, mn), m)
            self.w.writerow([idx, f"{tm:.6f}", f"{tw:.6f}", "" if ts is None else f"{ts:.6f}",
                             fn, mn, fr.shape[1], fr.shape[0]])

    def close(self):
        self.q.put(None)
        self.th.join()
        self.f.close()
        return self.dropped
