"""Convert ROS Image pixel buffers into OpenCV BGR frames."""
import cv2
import numpy as np


def image_to_bgr(encoding: str, height: int, width: int, step: int, data) -> np.ndarray:
    enc = encoding.lower()
    ch = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}.get(enc)
    if ch is None:
        raise ValueError(f"unsupported encoding {encoding!r} (bgr8/rgb8/bgra8/rgba8/mono8)")
    if height <= 0 or width <= 0 or step < width * ch or len(data) != height * step:
        raise ValueError("invalid Image dimensions, row stride or buffer length")
    buf = np.frombuffer(data, np.uint8).reshape(height, step)[:, :width * ch]
    img = buf.reshape(height, width, ch) if ch > 1 else buf.reshape(height, width)
    if enc == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if enc == "mono8":
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return np.ascontiguousarray(img)
