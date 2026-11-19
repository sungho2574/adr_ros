"""Render the GateNet mask and flight battery readout for local viewing and streaming."""
from array import array
from collections import OrderedDict
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, CompressedImage, Image
from std_msgs.msg import String

from adr_camera.image import image_to_bgr

IMAGE_QOS = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)


def battery_values(voltage, remaining, connected):
    return {
        'connected': bool(connected),
        'voltage_v': float(voltage) if connected and math.isfinite(voltage) and voltage > 0 else None,
        'remaining_pct': float(100 * remaining) if connected and 0 <= remaining <= 1 else None,
    }


def render(frame, mask, battery, max_width):
    if frame.shape[:2] != mask.shape:
        raise ValueError('Rectified frame and GateNet mask sizes differ')
    canvas = frame.copy()
    gate = mask != 0
    canvas[gate] = (canvas[gate].astype(np.uint16) + (40, 230, 50)) // 2

    if max_width:
        h, w = canvas.shape[:2]
        interpolation = cv2.INTER_AREA if max_width < w else cv2.INTER_LINEAR
        canvas = cv2.resize(canvas, (max_width, round(h * max_width / w)),
                            interpolation=interpolation)
    h, w = canvas.shape[:2]
    voltage, percent = battery['voltage_v'], battery['remaining_pct']
    volts = '-- V' if voltage is None else f'{voltage:.2f} V'
    remaining = '-- %' if percent is None else f'{percent:.0f} %'
    text = f'{volts}     {remaining}'
    scale = max(0.4, w / 800)
    thickness = max(1, round(w / 425))
    font = cv2.FONT_HERSHEY_SIMPLEX
    (width, _), _ = cv2.getTextSize(text, font, scale, thickness)
    origin = ((w - width) // 2, h - max(9, round(h * 0.05)))
    cv2.putText(canvas, text, origin, font, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(canvas, text, origin, font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return canvas


class Monitor(Node):
    def __init__(self):
        super().__init__('racing_monitor')
        defaults = {
            'fps': 30.0, 'jpeg_quality': 70, 'output_width': 640, 'stale_after': 2.0,
            'battery_source': 'ros', 'battery_topic': '',
        }
        cfg = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        fps = cfg['fps']
        self.quality = cfg['jpeg_quality']
        self.output_width = cfg['output_width']
        self.stale_after = cfg['stale_after']
        if (not math.isfinite(fps) or fps <= 0 or not 1 <= self.quality <= 100
                or (self.output_width != 0 and self.output_width < 160)
                or not math.isfinite(self.stale_after) or self.stale_after <= 0):
            raise ValueError('Invalid monitor fps, JPEG quality, output width or stale timeout')

        self.pending = (OrderedDict(), OrderedDict())
        self.latest = None
        self.latest_stamp = None
        self.battery = battery_values(0.0, -1.0, False)
        self.battery_at = None
        self.raw_pub = self.create_publisher(Image, '/adr/monitor/image', IMAGE_QOS)
        self.jpeg_pub = self.create_publisher(CompressedImage, '/adr/monitor/image/compressed', IMAGE_QOS)
        self.status_pub = self.create_publisher(String, '/adr/monitor/status', IMAGE_QOS)
        self.create_subscription(Image, '/adr/perception/image_rect',
                                 lambda msg: self.receive_image(msg, 0), IMAGE_QOS)
        self.create_subscription(Image, '/adr/perception/mask_full',
                                 lambda msg: self.receive_image(msg, 1), IMAGE_QOS)
        topic = cfg['battery_topic']
        if cfg['battery_source'] == 'ros':
            self.create_subscription(BatteryState, topic or '/adr/telemetry/battery',
                                     self.receive_ros_battery, IMAGE_QOS)
        elif cfg['battery_source'] == 'px4':
            from px4_msgs.msg import BatteryStatus
            version = getattr(BatteryStatus, 'MESSAGE_VERSION', 0)
            default_topic = '/fmu/out/battery_status' + (f'_v{version}' if version else '')
            self.create_subscription(BatteryStatus, topic or default_topic,
                                     self.receive_px4_battery, IMAGE_QOS)
        else:
            raise ValueError('battery_source must be ros or px4')
        self.create_timer(1.0 / fps, self.tick)

    def receive_image(self, msg, index):
        stamp = msg.header.stamp
        key = (stamp.sec, stamp.nanosec, msg.header.frame_id)
        if self.latest_stamp is not None and key <= self.latest_stamp:
            return
        own, peer = self.pending[index], self.pending[1 - index]
        matched = peer.pop(key, None)
        if matched is None:
            own[key] = msg
            if len(own) > IMAGE_QOS.depth:
                own.popitem(last=False)
            return
        self.latest = (msg, matched) if index == 0 else (matched, msg)
        self.latest_stamp = key

    def receive_ros_battery(self, msg):
        self.battery = battery_values(msg.voltage, msg.percentage, msg.present)
        self.battery_at = time.monotonic()

    def receive_px4_battery(self, msg):
        self.battery = battery_values(msg.voltage_v, msg.remaining, msg.connected)
        self.battery_at = time.monotonic()

    def tick(self):
        age = None if self.battery_at is None else time.monotonic() - self.battery_at
        health = 'missing' if age is None else ('stale' if age > self.stale_after else 'ok')
        battery = self.battery if health == 'ok' else battery_values(0.0, -1.0, False)
        if self.status_pub.get_subscription_count():
            state = {'battery': {'health': health, 'value': battery}}
            self.status_pub.publish(String(data=json.dumps(state, allow_nan=False)))

        pair, self.latest = self.latest, None
        raw = self.raw_pub.get_subscription_count()
        jpeg = self.jpeg_pub.get_subscription_count()
        if pair is None or not (raw or jpeg):
            return
        msg, mask = pair
        try:
            frame = image_to_bgr(msg.encoding, msg.height, msg.width, msg.step, msg.data)
            if mask.encoding != 'mono8' or mask.step < mask.width:
                raise ValueError('Invalid GateNet mask encoding or stride')
            pixels = np.frombuffer(mask.data, np.uint8).reshape(mask.height, mask.step)[:, :mask.width]
            canvas = render(frame, pixels, battery, self.output_width)
        except (ValueError, cv2.error) as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        h, w = canvas.shape[:2]
        if raw:
            self.raw_pub.publish(Image(header=msg.header, height=h, width=w, encoding='bgr8',
                                       step=w * 3, data=array('B', canvas.tobytes())))
        if jpeg:
            ok, encoded = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ok:
                self.jpeg_pub.publish(CompressedImage(header=msg.header, format='bgr8; jpeg compressed bgr8',
                                                      data=array('B', encoded.tobytes())))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Monitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
