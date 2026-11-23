"""Render the GateNet mask and flight telemetry for local viewing and streaming."""
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
from sensor_msgs.msg import BatteryState, CompressedImage, Image, Imu
from std_msgs.msg import String

from adr_camera.image import image_to_bgr

IMAGE_QOS = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
WHITE = (255, 255, 255)
GREEN = (0, 220, 0)
RED = (0, 0, 255)
ORANGE = (0, 165, 255)
GRAY = (160, 160, 160)
MODE_COLORS = {'AUTO': GREEN, 'ARMED': (0, 165, 255), 'DISARMED': (160, 160, 160),
               'NO STATUS': RED}


def battery_values(voltage, remaining, connected):
    return {
        'connected': bool(connected),
        'voltage_v': float(voltage) if connected and math.isfinite(voltage) and voltage > 0 else None,
        'remaining_pct': float(100 * remaining) if connected and 0 <= remaining <= 1 else None,
    }


def attitude_degrees(orientation):
    q = orientation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(norm) or norm < 1e-6:
        return None
    x, y, z, w = (value / norm for value in (q.x, q.y, q.z, q.w))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    # Display Betaflight's clockwise heading, converted back from ROS ENU yaw.
    heading = round(90 - math.degrees(yaw), 1) % 360
    return {'roll_deg': math.degrees(roll), 'pitch_deg': math.degrees(pitch), 'yaw_deg': heading}


def flight_mode(armed, msp_override):
    if armed is True:
        return 'AUTO' if msp_override is True else 'ARMED'
    return 'DISARMED' if armed is False else 'NO STATUS'


def render(frame, mask, battery, flight, max_width, control=None):
    if frame.shape[:2] != mask.shape:
        raise ValueError('Rectified frame and GateNet mask sizes differ')
    canvas = frame.copy()
    gate = mask != 0
    canvas[gate] = (canvas[gate].astype(np.uint16) + (40, 230, 50)) // 2

    if max_width and max_width != canvas.shape[1]:
        h, w = canvas.shape[:2]
        interpolation = cv2.INTER_AREA if max_width < w else cv2.INTER_LINEAR
        canvas = cv2.resize(canvas, (max_width, round(h * max_width / w)),
                            interpolation=interpolation)
    h, w = canvas.shape[:2]
    color = MODE_COLORS[flight['mode']]
    border = max(3, round(w * 0.008))
    canvas[:border, :] = color
    canvas[-border:, :] = color
    canvas[:, :border] = color
    canvas[:, -border:] = color

    unit = min(w / 640, h / 480)
    margin = max(border + 4, round(15 * unit))
    scale = max(0.48, unit)
    top = margin + round(25 * scale)
    line_height = max(15, round(24 * scale))
    font = cv2.FONT_HERSHEY_SIMPLEX
    split = 2 * w // 5
    columns = {
        'left': (margin, split - 3),
        'right': (split + 3, w - margin),
        'center': (margin, w - margin),
    }

    def draw(text, y, color=WHITE, *, align='left', size=1.0):
        left, right = columns[align]
        text_scale = scale * size
        thickness = max(1, round(2 * text_scale))
        width = cv2.getTextSize(text, font, text_scale, thickness)[0][0]
        available = right - left - 4  # Keep space for the text outline.
        if width > available:
            text_scale *= (available - thickness) / (width - thickness)
            thickness = max(1, round(2 * text_scale))
            width = cv2.getTextSize(text, font, text_scale, thickness)[0][0]
        x = {'left': left, 'right': right - width, 'center': (left + right - width) // 2}[align]
        outline = thickness + max(2, round(3 * text_scale))
        cv2.putText(canvas, text, (x, y), font, text_scale, (0, 0, 0), outline, cv2.LINE_AA)
        cv2.putText(canvas, text, (x, y), font, text_scale, color, thickness, cv2.LINE_AA)

    draw(flight['mode'].removesuffix('ED'), top, WHITE if flight['mode'] == 'DISARMED' else color)
    connected = flight['connected']
    draw('FC CONNECTED' if connected else 'FC DISCONNECTED', top,
         GREEN if connected else RED, align='right', size=0.65)
    angles = flight['attitude']
    for index, axis in enumerate(('roll', 'pitch', 'yaw'), start=1):
        value = angles[f'{axis}_deg'] if angles is not None else None
        degrees = f'{value:+.1f} deg' if value is not None else '--'
        draw(f'{axis.upper()}  {degrees}', top + index * line_height, align='right', size=0.60)

    if control is not None:
        rc_state = flight.get('rc_state')
        rc_label = {
            'active': 'RC ACTIVE', 'disabled': 'RC OFF',
            'restart_required': 'RESTART REQUIRED',
            'waiting_for_action': 'RC WAIT ACTION',
            'waiting_for_disarm': 'RC WAIT DISARM',
            'ready': 'RC READY', 'unavailable': 'RC UNAVAILABLE',
        }.get(rc_state, 'RC NO STATUS')
        rc_color = {
            'active': GREEN, 'restart_required': RED,
            'ready': ORANGE, 'waiting_for_action': ORANGE, 'waiting_for_disarm': ORANGE,
        }.get(rc_state, GRAY)
        policy_state = control.get('state')
        policy_label = {
            'stopped_stale_mask': 'MASK STALE',
            'stopped_stale_imu': 'IMU STALE',
            'stale_mask': 'MASK STALE',
            'stale_imu': 'IMU STALE',
            'running': f"POLICY {control.get('step_hz', 0):.0f} Hz",
        }.get(policy_state, policy_state.replace('_', ' ').upper() if policy_state else 'POLICY NO STATUS')
        policy_color = WHITE
        if policy_state is None:
            policy_color = GRAY
        elif policy_state.startswith('waiting_'):
            policy_color = ORANGE
        elif policy_state.startswith(('stale_', 'stopped_')):
            policy_color = RED
        mask_age = control.get('mask_age_ms')
        imu_age = control.get('imu_age_ms')
        lines = (
            (rc_label, rc_color),
            (policy_label, policy_color),
            (f'MASK {mask_age:.0f} ms' if mask_age is not None else 'MASK -- ms', WHITE),
            (f'IMU {imu_age:.0f} ms' if imu_age is not None else 'IMU -- ms', WHITE),
        )
        control_top = top + 4 * line_height + margin
        for index, (label, text_color) in enumerate(lines):
            draw(label, control_top + index * line_height, text_color, align='right', size=0.65)

    voltage, percent = battery['voltage_v'], battery['remaining_pct']
    volts = '-- V' if voltage is None else f'{voltage:.2f} V'
    remaining = '-- %' if percent is None else f'{percent:.0f} %'
    draw(f'{volts}     {remaining}', h - margin, align='center', size=0.75)
    return canvas


class Monitor(Node):
    def __init__(self):
        super().__init__('racing_monitor')
        defaults = {
            'fps': 30.0, 'jpeg_quality': 70, 'output_width': 320, 'stale_after': 2.0,
            'battery_source': 'ros', 'battery_topic': '', 'fc_stale_after': 0.5,
        }
        cfg = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        fps = cfg['fps']
        self.quality = cfg['jpeg_quality']
        self.output_width = cfg['output_width']
        self.stale_after = cfg['stale_after']
        self.fc_stale_after = cfg['fc_stale_after']
        if (not math.isfinite(fps) or fps <= 0 or not 1 <= self.quality <= 100
                or (self.output_width != 0 and self.output_width < 160)
                or any(not math.isfinite(seconds) or seconds <= 0
                       for seconds in (self.stale_after, self.fc_stale_after))):
            raise ValueError('Invalid monitor fps, JPEG quality, output width or stale timeout')

        self.pending = (OrderedDict(), OrderedDict())
        self.latest = None
        self.latest_stamp = None
        self.battery = battery_values(0.0, -1.0, False)
        self.battery_at = None
        self.fc = self.fc_at = None
        self.control = self.control_at = None
        self.attitude = self.attitude_at = None
        self.raw_pub = self.create_publisher(Image, '/adr/monitor/image', IMAGE_QOS)
        self.jpeg_pub = self.create_publisher(CompressedImage, '/adr/monitor/image/compressed', IMAGE_QOS)
        self.status_pub = self.create_publisher(String, '/adr/monitor/status', IMAGE_QOS)
        self.create_subscription(Image, '/adr/perception/image_rect',
                                 lambda msg: self.receive_image(msg, 0), IMAGE_QOS)
        self.create_subscription(Image, '/adr/perception/mask_full',
                                 lambda msg: self.receive_image(msg, 1), IMAGE_QOS)
        self.create_subscription(String, '/adr/telemetry/fc_status', self.receive_fc, IMAGE_QOS)
        self.create_subscription(String, '/adr/control/status', self.receive_control, IMAGE_QOS)
        self.create_subscription(Imu, '/adr/telemetry/imu', self.receive_attitude, IMAGE_QOS)
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

    def receive_fc(self, msg):
        try:
            value = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(value, dict) or not isinstance(value.get('connected'), bool):
            return
        self.fc = value
        self.fc_at = time.monotonic()
        if not value['connected']:
            self.attitude = None
            self.attitude_at = None

    def receive_control(self, msg):
        try:
            value = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(value, dict) or not isinstance(value.get('state'), str):
            return
        for key in ('step_hz', 'mask_age_ms', 'imu_age_ms'):
            number = value.get(key)
            if isinstance(number, (int, float)) and math.isfinite(number):
                continue
            value[key] = 0 if key == 'step_hz' else None
        self.control = value
        self.control_at = time.monotonic()

    def receive_attitude(self, msg):
        self.attitude = (attitude_degrees(msg.orientation)
                         if msg.orientation_covariance[0] != -1 else None)
        self.attitude_at = time.monotonic()

    def flight_status(self, now):
        connected = (self.fc_at is not None and now - self.fc_at <= self.fc_stale_after
                     and self.fc['connected'])
        mode = 'NO STATUS'
        if connected:
            rc_state = self.fc.get('rc_state', 'disabled')
            auto = self.fc.get('msp_override') if rc_state == 'disabled' else rc_state == 'active'
            mode = flight_mode(self.fc.get('armed'), auto)
        fresh_angles = self.attitude_at is not None and now - self.attitude_at <= self.fc_stale_after
        return {'connected': connected, 'mode': mode,
                'rc_state': rc_state if connected else None,
                'attitude': self.attitude if connected and fresh_angles else None}

    def tick(self):
        now = time.monotonic()
        age = None if self.battery_at is None else now - self.battery_at
        health = 'missing' if age is None else ('stale' if age > self.stale_after else 'ok')
        battery = self.battery if health == 'ok' else battery_values(0.0, -1.0, False)
        flight = self.flight_status(now)
        if self.status_pub.get_subscription_count():
            state = {'battery': {'health': health, 'value': battery}, 'flight': flight}
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
            control = self.control
            if self.control_at is not None and now - self.control_at > self.stale_after:
                control = {}
            canvas = render(frame, pixels, battery, flight, self.output_width, control)
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
