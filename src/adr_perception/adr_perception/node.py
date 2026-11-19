"""Subscribe to the latest ROS camera frame and publish GateNet observations."""
from __future__ import annotations

from array import array
import os
from pathlib import Path
import tempfile
import threading
import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from adr_camera.image import image_to_bgr
from . import core
from .gatenet_rt.pipeline import Recorder

try:
    CONFIGS = Path(get_package_share_directory("adr_perception")) / "configs"
except PackageNotFoundError:
    CONFIGS = Path(__file__).resolve().parents[1] / "configs"

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def resolve(value):
    return str(CONFIGS / Path(value).expanduser()) if value else ""


def image_message(pixels, header, encoding):
    height, width = pixels.shape[:2]
    return Image(header=header, height=height, width=width, encoding=encoding,
                 step=pixels.nbytes // height, data=array("B", pixels.tobytes()))


class GateNetNode(Node):
    def __init__(self):
        super().__init__("gatenet_node")
        defaults = {
            "image_topic": "/adr/camera/image_raw",
            "engine_file": "gatenet_240x320_fp16.plan",
            "onnx_file": "gatenet_240x320.onnx",
            "remap_file": "remap_320x240.npz",
            "gatenet_K_file": "gatenet_K.yaml",
            "calib_file": "calib_1640x1232_v2.yaml",
            "publish_mask_full": False,
            "publish_rectified": False,
            "record": False,
            "record_dir": "~/adr_gatenet_records",
        }
        self.p = {name: self.declare_parameter(name, value).value
                  for name, value in defaults.items()}
        p = self.p
        for name in ("engine_file", "onnx_file", "remap_file", "gatenet_K_file", "calib_file"):
            p[name] = resolve(p[name])
        report = core.check_remap(p["remap_file"], p["gatenet_K_file"])
        if os.path.realpath(report["calib_file"]) != os.path.realpath(p["calib_file"]):
            raise ValueError("calib_file must match the calibration named by gatenet_K_file")
        self.get_logger().info(f"GateNet remap max error {report['max_map_diff_px']:.2e} px")

        self.pub_mask = self.create_publisher(Image, "/adr/perception/mask", QOS)
        self.pub_lat = self.create_publisher(Float32, "/adr/perception/latency_ms", QOS)
        self.pub_full = (self.create_publisher(Image, "/adr/perception/mask_full", QOS)
                         if p["publish_mask_full"] else None)
        self.pub_rectified = (self.create_publisher(Image, "/adr/perception/image_rect", QOS)
                              if p["publish_rectified"] else None)
        self.slot = core.LatestSlot()
        self.stop = threading.Event()
        self.worker_error = None
        self.recorder = None
        if p["record"]:
            root = Path(p["record_dir"]).expanduser()
            root.mkdir(parents=True, exist_ok=True)
            directory = tempfile.mkdtemp(prefix=time.strftime("%Y%m%d_%H%M%S_"), dir=root)
            self.recorder = Recorder(directory)
            self.get_logger().info(f"Recording GateNet input and masks to {directory}")

        self.create_subscription(Image, p["image_topic"], self.slot.put, QOS)
        self.create_timer(0.25, self.check_worker)
        self.worker = threading.Thread(target=self.run_worker, daemon=True)
        self.worker.start()

    def check_worker(self):
        if self.worker_error is not None:
            raise RuntimeError("GateNet worker failed") from self.worker_error

    def run_worker(self):
        try:
            self.work()
        except Exception as exc:
            self.worker_error = exc

    def work(self):
        # TensorRT's CUDA context belongs to the inference thread.
        p = self.p
        backend = core.make_backend(p["engine_file"], p["onnx_file"], self.get_logger().info)
        net = core.GateNetCore(p["remap_file"], backend)
        index = count = 0
        started = time.monotonic()
        while not self.stop.is_set() and rclpy.ok():
            msg = self.slot.get()
            if self.stop.is_set() or msg is None:
                continue
            if not core.input_size_ok(msg.height, msg.width):
                self.get_logger().error(
                    f"Expected a 1640x1232 or 320x240 camera frame, got {msg.width}x{msg.height}",
                    throttle_duration_sec=5.0)
                continue
            try:
                frame = image_to_bgr(msg.encoding, msg.height, msg.width, msg.step, msg.data)
            except ValueError as exc:
                self.get_logger().error(str(exc), throttle_duration_sec=5.0)
                continue
            result = net.process(frame)
            stamp = msg.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            latency = (self.get_clock().now().nanoseconds - stamp_ns) / 1e6
            self.pub_mask.publish(image_message(result.mask64, msg.header, "mono8"))
            self.pub_lat.publish(Float32(data=float(latency)))
            if self.pub_full is not None:
                self.pub_full.publish(image_message(result.mask_full, msg.header, "mono8"))
            if self.pub_rectified is not None:
                self.pub_rectified.publish(image_message(result.rectified, msg.header, "bgr8"))
            now = time.monotonic()
            if self.recorder is not None:
                self.recorder.put(index, result.small, result.mask64, now, stamp_ns / 1e9)
            index += 1
            count += 1
            if now - started >= 5.0:
                self.get_logger().info(
                    f"{count / (now - started):.1f} Hz, latency {latency:.1f} ms, "
                    f"infer {result.infer_ms:.1f} ms, dropped {self.slot.dropped}")
                count, started = 0, now

    def destroy_node(self):
        self.stop.set()
        self.slot.close()
        self.worker.join()
        if self.recorder is not None:
            dropped = self.recorder.close()
            self.get_logger().info(f"GateNet recorder closed ({dropped} records dropped)")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GateNetNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
