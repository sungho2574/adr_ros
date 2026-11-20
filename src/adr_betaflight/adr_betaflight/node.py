"""Publish Betaflight IMU, battery and FC status via ROS 2."""
import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import serial
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String

from .msp import (Client, Command, RejectedCommand, decode_battery, decode_battery_config,
                  decode_flight_mode, decode_imu)

STANDARD_GRAVITY = 9.80665
LINK_ERRORS = (OSError, ValueError, RejectedCommand)


class MspTelemetry(Node):
    def __init__(self):
        super().__init__('msp_telemetry')
        self.device = self.declare_parameter('port', '').value
        self.baud = self.declare_parameter('baud', 115200).value
        imu_hz = self.declare_parameter('imu_hz', 100.0).value
        battery_hz = self.declare_parameter('battery_hz', 2.0).value
        acc_lsb_per_g = self.declare_parameter('acc_lsb_per_g', 512.0).value
        self.timeout = self.declare_parameter('response_timeout_s', 0.025).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        if not self.device or self.baud <= 0:
            raise ValueError('Set port to the Jetson UART device and baud to the Betaflight MSP speed')
        if any(not math.isfinite(value) or value <= 0
               for value in (imu_hz, battery_hz, acc_lsb_per_g, self.timeout)):
            raise ValueError('MSP rates, accelerometer scale and timeout must be finite and positive')
        if battery_hz > imu_hz:
            raise ValueError('battery_hz must not exceed imu_hz')

        self.acc_scale = STANDARD_GRAVITY / acc_lsb_per_g
        self.battery_period = 1 / battery_hz
        self.client = None
        self.battery_config = None
        self.box_ids = None
        self.next_battery = self.next_status = self.retry_at = 0.0
        self.first_imu = True

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_pub = self.create_publisher(Imu, '/adr/telemetry/imu', qos)
        self.battery_pub = self.create_publisher(BatteryState, '/adr/telemetry/battery', qos)
        self.fc_pub = self.create_publisher(String, '/adr/telemetry/fc_status', qos)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1 / imu_hz, self.tick, clock=self.steady_clock)
        self.get_logger().info(f'Polling {self.device} at {self.baud} baud')

    def connect(self):
        port = serial.Serial(self.device, self.baud, timeout=0, write_timeout=self.timeout,
                             exclusive=True)
        self.client = Client(port, self.timeout)
        port.reset_input_buffer()
        if self.client.read(Command.FC_VARIANT) != b'BTFL':
            raise ValueError('MSP device did not identify itself as Betaflight')

        self.battery_config = None
        try:
            self.battery_config = decode_battery_config(self.client.read(Command.BATTERY_CONFIG))
        except (ValueError, RejectedCommand) as exc:
            self.warn(f'Battery configuration unavailable: {exc}')
        self.box_ids = None
        try:
            self.box_ids = self.client.read_box_ids()
        except (ValueError, RejectedCommand) as exc:
            self.warn(f'Flight mode IDs unavailable: {exc}')
        self.next_battery = self.next_status = 0.0
        self.first_imu = True
        self.get_logger().info('Betaflight MSP connected')

    def tick(self):
        now = time.monotonic()
        try:
            if self.client is None:
                if now >= self.retry_at:
                    self.connect()
                return

            poll_battery = now >= self.next_battery
            poll_status = now >= self.next_status
            commands = [Command.RAW_IMU, Command.ATTITUDE]
            if poll_battery:
                commands.append(Command.BATTERY_STATE)
            if poll_status and self.box_ids is not None:
                commands.append(Command.STATUS)
            replies = dict(zip(commands, self.client.read_batch(*commands)))
            stamp = self.get_clock().now().to_msg()
            self.publish_imu(decode_imu(replies[Command.RAW_IMU], replies[Command.ATTITUDE]), stamp)
            if poll_status:
                self.next_status = now + 0.1
                self.publish_flight_mode(replies.get(Command.STATUS))
            if poll_battery:
                self.next_battery = now + self.battery_period
                try:
                    sample = decode_battery(replies[Command.BATTERY_STATE], self.battery_config)
                except ValueError as exc:
                    self.warn(str(exc))
                else:
                    self.publish_battery(sample, stamp)
        except LINK_ERRORS as exc:
            self.disconnect()
            self.publish_fc_status(False)
            self.retry_at = time.monotonic() + 1.0
            self.warn(f'MSP link {self.device}: {exc}')

    def publish_flight_mode(self, payload):
        armed = override = None
        if payload is not None:
            try:
                armed, override = decode_flight_mode(payload, self.box_ids)
            except ValueError as exc:
                self.warn(str(exc))
        self.publish_fc_status(True, armed, override)

    def publish_fc_status(self, connected, armed=None, msp_override=None):
        self.fc_pub.publish(String(data=json.dumps({
            'connected': connected, 'armed': armed, 'msp_override': msp_override,
        })))

    def publish_imu(self, sample, stamp):
        if not any(sample.acceleration_raw):
            self.warn('Zero Betaflight accelerometer sample. Check ACC availability and calibration.')
            return
        if self.first_imu:
            self.get_logger().info(f'First MSP accelerometer sample (LSB): {sample.acceleration_raw}')
            self.first_imu = False

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = sample.orientation
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample.angular_velocity
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
            value * self.acc_scale for value in sample.acceleration_raw)
        self.imu_pub.publish(msg)

    def publish_battery(self, sample, stamp):
        msg = BatteryState(
            present=sample.cells > 0, voltage=sample.voltage, current=sample.current,
            temperature=math.nan, charge=sample.charge, capacity=math.nan,
            design_capacity=sample.design_capacity, percentage=sample.percentage,
            cell_voltage=[math.nan] * sample.cells, cell_temperature=[math.nan] * sample.cells,
        )
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.battery_pub.publish(msg)

    def warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def disconnect(self):
        client, self.client = self.client, None
        if client is not None:
            try:
                client.port.close()
            except OSError as exc:
                self.warn(f'Error closing MSP UART: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MspTelemetry()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.disconnect()
            node.destroy_node()
        rclpy.try_shutdown()
