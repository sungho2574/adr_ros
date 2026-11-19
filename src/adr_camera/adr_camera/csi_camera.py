#!/usr/bin/env python3
"""Publish Jetson IMX219 frames and camera intrinsics via Argus and GStreamer."""
from array import array
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
gi.require_version('GstVideo', '1.0')
from gi.repository import Gst, GstVideo  # noqa: E402  (gi needs require_version first)

# IMX219 sensor modes exposed by the nv_imx219 driver, as (width, height, fps).
# Picking a listed pair keeps Argus from renegotiating to a different mode.
SENSOR_MODES = ((3280, 2464, 21), (3280, 1848, 28), (1920, 1080, 30),
                (1640, 1232, 30), (1280, 720, 60))


def video_info(caps):
    """GstVideo.VideoInfo from caps, across the two PyGObject spellings."""
    factory = getattr(GstVideo.VideoInfo, 'new_from_caps', None)
    if factory is not None:
        return factory(caps)
    info = GstVideo.VideoInfo()
    info.from_caps(caps)
    return info


def load_camera_info(path, width, height, frame_id, logger, *, calibration_json='', capture_size=None):
    """Read runtime calibration and scale intrinsics to the published dimensions."""
    info = CameraInfo(width=width, height=height, distortion_model='plumb_bob')
    info.header.frame_id = frame_id
    if not path and not calibration_json:
        logger.warn('No calibration set: publishing uncalibrated CameraInfo.')
        return info
    try:
        if calibration_json:
            import json
            data = json.loads(calibration_json)
        else:
            import yaml
            with open(path) as handle:
                data = yaml.safe_load(handle) or {}
        if 'camera' in data:
            data = data['camera']['calibration']
        k = list(data['camera_matrix']['data'])
        d = list(data['distortion_coefficients']['data'])
        if len(k) != 9:
            raise ValueError('camera_matrix must hold 9 values')
        file_w = int(data.get('image_width', width))
        file_h = int(data.get('image_height', height))
        if calibration_json or capture_size:
            from math import isfinite
            capture_width, capture_height = capture_size or (width, height)
            if (file_w, file_h) != (capture_width, capture_height):
                raise ValueError(f'Expected {capture_width}x{capture_height} calibration, '
                                 f'got {file_w}x{file_h}')
            if not all(isfinite(float(v)) for v in k + d) or float(k[0]) <= 0 or float(k[4]) <= 0:
                raise ValueError('Calibration must have finite coefficients and positive focal lengths')
        sx, sy = width / file_w, height / file_h
        if (file_w, file_h) != (width, height):
            k = [k[0]*sx, k[1]*sx, (k[2] + 0.5)*sx - 0.5,
                 k[3]*sy, k[4]*sy, (k[5] + 0.5)*sy - 0.5,
                 k[6], k[7], k[8]]
        info.k = [float(v) for v in k]
        info.d = [float(v) for v in d]
        info.distortion_model = str(data.get('distortion_model', 'plumb_bob'))
        info.r = [float(v) for v in data.get('rectification_matrix', {}).get(
            'data', [1., 0., 0., 0., 1., 0., 0., 0., 1.])]
        projection = data.get('projection_matrix', {}).get('data')
        info.p = ([float(v)*sx + (sx - 1.0)/2 if i == 2 else
                   float(v)*sy + (sy - 1.0)/2 if i == 6 else
                   float(v) * (sx if i < 4 else sy if i < 8 else 1.0)
                   for i, v in enumerate(projection)] if projection else
                  [info.k[0], info.k[1], info.k[2], 0.,
                   info.k[3], info.k[4], info.k[5], 0.,
                   info.k[6], info.k[7], info.k[8], 0.])
        logger.info(f'Loaded camera intrinsics from {path or "real flight profile"}')
    except (OSError, ValueError, TypeError, KeyError) as exc:
        raise ValueError(f'Invalid camera calibration {path or "in launch config"}: {exc}') from exc
    return info


class CsiCamera(Node):
    def __init__(self, name='csi_camera', **kwargs):
        super().__init__(name, **kwargs)
        defaults = {
            'sensor_id': 0, 'width': 1640, 'height': 1232, 'fps': 30.0,
            'sensor_mode': 3,           # -1 lets Argus choose from the requested caps
            'output_width': 0, 'output_height': 0,  # 0 keeps the capture dimension
            'flip_method': 0,            # nvvidconv: 0 none, 2 180deg, 4 horiz, 6 vert
            'encoding': 'bgr8',          # bgr8 or mono8
            'image_topic': '/adr/camera/image_raw',
            'camera_info_topic': '/adr/camera/camera_info',
            'frame_id': 'camera_optical_frame',
            'camera_info_url': '',
            'calibration_json': '',
            'wbmode': 1,                 # Argus auto white balance
            'exposure_time_range': '',   # e.g. '5000000 20000000' (ns), '' = Argus auto
            'gain_range': '',            # e.g. '1 10'
            'isp_digital_gain_range': '',
            'ae_lock': False,
            'interpolation_method': 1,   # nvvidconv bilinear
        }
        cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}

        self.width, self.height = int(cfg['width']), int(cfg['height'])
        self.fps = float(cfg['fps'])
        self.encoding = str(cfg['encoding'])
        self.output_width = cfg['output_width'] or self.width
        self.output_height = cfg['output_height'] or self.height
        if self.width <= 0 or self.height <= 0 or not 0 < self.fps < float("inf"):
            raise ValueError('Invalid width, height or fps')
        # Argus mode 3 scales to 320x240 using the handoff's nearest whole-row size.
        expected_height = self.output_width * self.height / self.width
        if (not 0 < self.output_width <= self.width or not 0 < self.output_height <= self.height
                or abs(self.output_height - expected_height) > 0.5):
            raise ValueError('Camera output must preserve the capture aspect ratio to the nearest row')
        if self.encoding not in ('bgr8', 'mono8'):
            raise ValueError("encoding must be 'bgr8' or 'mono8'")
        if cfg['sensor_mode'] < -1:
            raise ValueError('sensor_mode must be -1 (auto) or a nonnegative Argus mode index')
        if not 0 <= cfg['interpolation_method'] <= 5:
            raise ValueError('interpolation_method must be between 0 and 5')
        if (self.width, self.height, round(self.fps)) not in SENSOR_MODES:
            self.get_logger().warn(
                f'{self.width}x{self.height}@{self.fps:g} is outside the built-in IMX219 mode list '
                f'{SENSOR_MODES}. Argus will scale or renegotiate, which costs '
                'frame rate and may change the field of view.')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(Image, str(cfg['image_topic']), qos)
        self.info_pub = self.create_publisher(CameraInfo, str(cfg['camera_info_topic']), qos)
        self.frame_id = str(cfg['frame_id'])
        self.info = load_camera_info(
            str(cfg['camera_info_url']), self.output_width, self.output_height,
            self.frame_id, self.get_logger(), calibration_json=cfg['calibration_json'],
            capture_size=(self.width, self.height))

        Gst.init(None)
        self.pipeline = Gst.parse_launch(self.build_pipeline(cfg))
        self.sink = self.pipeline.get_by_name('sink')
        self.running = True
        self.failed = False
        self.warned_pts = False
        self.frames = 0
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.pipeline.set_state(Gst.State.NULL)
            raise RuntimeError(
                'Could not start the Argus pipeline. Another process may hold the '
                'camera (only one Argus session is allowed), or nvarguscamerasrc is '
                'missing (install nvidia-l4t-gstreamer).')
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()
        self.create_timer(5.0, self.report)
        self.create_timer(0.5, self.check_capture)
        self.get_logger().info(
            f'Capturing {self.width}x{self.height}@{self.fps:g}, '
            f'publishing {self.output_width}x{self.output_height} {self.encoding} '
            f'-> {cfg["image_topic"]}')

    def build_pipeline(self, cfg):
        src = f'nvarguscamerasrc sensor-id={int(cfg["sensor_id"])} wbmode={int(cfg["wbmode"])}'
        if cfg['sensor_mode'] >= 0:
            src += f' sensor-mode={cfg["sensor_mode"]}'
        if cfg['exposure_time_range']:
            src += f' exposuretimerange="{cfg["exposure_time_range"]}"'
        if cfg['gain_range']:
            src += f' gainrange="{cfg["gain_range"]}"'
        if cfg['isp_digital_gain_range']:
            src += f' ispdigitalgainrange="{cfg["isp_digital_gain_range"]}"'
        if cfg['ae_lock']:
            src += ' aelock=true'
        out = 'GRAY8' if self.encoding == 'mono8' else 'BGR'
        return (
            f'{src} ! '
            f'video/x-raw(memory:NVMM),width={self.width},height={self.height},'
            f'framerate={round(self.fps)}/1,format=NV12 ! '
            f'nvvidconv flip-method={cfg["flip_method"]} '
            f'interpolation-method={cfg["interpolation_method"]} ! '
            f'video/x-raw,width={self.output_width},height={self.output_height},format=BGRx ! '
            'videoconvert ! '
            f'video/x-raw,format={out} ! '
            'appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false')

    def capture_loop(self):
        channels = 1 if self.encoding == 'mono8' else 3
        while self.running:
            sample = self.sink.emit('pull-sample')
            if sample is None:                     # pipeline stopped or EOS
                if self.running:
                    self.failed = True
                break
            try:
                self.publish(sample, channels)
            except Exception as exc:               # noqa: BLE001 - keep streaming
                self.get_logger().error(f'Dropping frame: {exc}',
                                        throttle_duration_sec=5.0)

    def publish(self, sample, channels):
        info = video_info(sample.get_caps())
        width, height, stride = info.width, info.height, info.stride[0]
        if (width, height) != (self.info.width, self.info.height):
            raise RuntimeError(f'Unexpected frame size {width}x{height}')
        row = width * channels

        buffer = sample.get_buffer()
        ok, mapped = buffer.map(Gst.MapFlags.READ)
        if not ok:
            raise RuntimeError('Could not map the GStreamer buffer')
        try:
            raw = bytes(mapped.data)
        finally:
            buffer.unmap(mapped)
        if len(raw) < stride * height:
            raise RuntimeError(f'Short frame: {len(raw)} bytes for {height}x{stride}')
        # GStreamer pads each row up to a 4-byte boundary; ROS Image wants the
        # stride it is told about, so hand over tightly packed rows.
        payload = (raw[:row * height] if stride == row else
                   b''.join(raw[y * stride:y * stride + row] for y in range(height)))

        stamp = self.capture_stamp(sample, buffer)
        message = Image(height=height, width=width, encoding=self.encoding,
                        is_bigendian=0, step=row, data=array('B', payload))
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        self.image_pub.publish(message)
        self.info.header.stamp = stamp
        self.info_pub.publish(self.info)
        self.frames += 1

    def capture_stamp(self, sample, buffer):
        clock = self.pipeline.get_clock()
        if buffer.pts != Gst.CLOCK_TIME_NONE and clock is not None:
            monotonic_now = time.monotonic_ns()
            segment = sample.get_segment()
            if segment is not None and abs(clock.get_time() - monotonic_now) < 50_000_000:
                running = segment.to_running_time(Gst.Format.TIME, buffer.pts)
                base = self.pipeline.get_base_time()
                if running != Gst.CLOCK_TIME_NONE and base != Gst.CLOCK_TIME_NONE:
                    stamp_ns = base + running + time.time_ns() - time.monotonic_ns()
                    return Time(nanoseconds=stamp_ns).to_msg()
        if not self.warned_pts:
            self.get_logger().warn('Camera PTS unavailable. Using frame arrival timestamps.')
            self.warned_pts = True
        return self.get_clock().now().to_msg()

    def check_capture(self):
        event = self.pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
        if event is not None and event.type == Gst.MessageType.ERROR:
            error, details = event.parse_error()
            raise RuntimeError(f'Camera pipeline failed: {error}: {details}')
        if self.failed or event is not None:
            raise RuntimeError('Camera stream ended unexpectedly.')

    def report(self):
        self.get_logger().info(f'{self.frames / 5.0:.1f} fps published',
                               throttle_duration_sec=30.0)
        self.frames = 0

    def stop(self):
        self.running = False
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)


def main():
    rclpy.init()
    node = CsiCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
