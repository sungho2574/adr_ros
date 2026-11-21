"""Bridge Betaflight telemetry and pilot-authorized RC commands over one UART."""
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

from .msp import (Client, Command, RejectedCommand, check_override_config, decode_battery,
                  decode_battery_config, decode_fc_version, decode_flight_mode, decode_imu, decode_rc)
from .override import Override

STANDARD_GRAVITY = 9.80665
LINK_ERRORS = (OSError, ValueError, RejectedCommand)


def mode_status(mode):
    return {
        'armed': mode.armed if mode is not None else None,
        'msp_override': mode.override if mode is not None else None,
        'angle': mode.angle if mode is not None else None,
        'failsafe': mode.failsafe if mode is not None else None,
        'rx_loss': mode.rx_loss if mode is not None else None,
        'failsafe_switch': mode.failsafe_switch if mode is not None else None,
    }


class MspTelemetry(Node):
    def __init__(self):
        super().__init__('msp_telemetry')
        self.device = self.declare_parameter('port', '').value
        self.baud = self.declare_parameter('baud', 115200).value
        imu_hz = self.declare_parameter('imu_hz', 100.0).value
        battery_hz = self.declare_parameter('battery_hz', 2.0).value
        rc_hz = self.declare_parameter('rc_hz', 25.0).value
        acc_lsb_per_g = self.declare_parameter('acc_lsb_per_g', 512.0).value
        self.timeout = self.declare_parameter('response_timeout_s', 0.025).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.enable_rc = self.declare_parameter('enable_rc', False).value
        if not self.device or self.baud <= 0:
            raise ValueError('Set port to the Jetson UART device and baud to the Betaflight MSP speed')
        if any(not math.isfinite(value) or value <= 0
               for value in (imu_hz, battery_hz, acc_lsb_per_g, self.timeout)):
            raise ValueError('MSP rates, accelerometer scale and timeout must be finite and positive')
        if battery_hz > imu_hz:
            raise ValueError('battery_hz must not exceed imu_hz')
        if not 0 <= rc_hz <= imu_hz:
            raise ValueError('rc_hz must be between zero and imu_hz')

        self.acc_scale = STANDARD_GRAVITY / acc_lsb_per_g
        self.battery_period = 1 / battery_hz
        self.rc_period = 1 / rc_hz if rc_hz else None
        self.client = None
        self.fc_version = None
        self.battery_config = None
        self.box_ids = None
        self.rc_available = False
        self.override = Override()
        self.next_battery = self.next_status = self.retry_at = 0.0
        self.next_rc = 0.0
        self.first_imu = True
        self.last_arming_disable_mask = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_pub = self.create_publisher(Imu, '/adr/telemetry/imu', qos)
        self.battery_pub = self.create_publisher(BatteryState, '/adr/telemetry/battery', qos)
        self.fc_pub = self.create_publisher(String, '/adr/telemetry/fc_status', qos)
        if self.rc_period is not None:
            self.rc_pub = self.create_publisher(String, '/adr/telemetry/rc', qos)
        if self.enable_rc:
            self.create_subscription(String, '/adr/control/rc_action', self.receive_action, qos)
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

        self.fc_version = None
        try:
            self.fc_version = decode_fc_version(self.client.read(Command.FC_VERSION))
        except (ValueError, RejectedCommand, TimeoutError) as exc:
            self.warn(f'Betaflight version unavailable: {exc}')

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
        if self.enable_rc:
            try:
                check_override_config(self.client, self.box_ids, self.fc_version)
            except (ValueError, RejectedCommand) as exc:
                self.get_logger().error(f'MSP RC disabled: {exc}')
            else:
                self.rc_available = True
        self.next_battery = self.next_status = self.next_rc = 0.0
        self.first_imu = True
        self.get_logger().info('Betaflight MSP connected')

    def receive_action(self, msg):
        self.override.receive(msg.data, time.monotonic_ns())

    def tick(self):
        now = time.monotonic()
        try:
            if self.client is None:
                if now >= self.retry_at:
                    self.connect()
                return

            poll_battery = now >= self.next_battery
            poll_status = now >= self.next_status
            poll_rc = self.rc_period is not None and now >= self.next_rc
            commands = [Command.RAW_IMU, Command.ATTITUDE]
            if poll_battery:
                commands.append(Command.BATTERY_STATE)
            if poll_status or poll_rc or self.rc_available:
                commands.append(Command.STATUS)
            if poll_rc:
                commands.append(Command.RC)
            replies = dict(zip(commands, self.client.read_batch(*commands)))
            stamp = self.get_clock().now().to_msg()
            self.publish_imu(decode_imu(replies[Command.RAW_IMU], replies[Command.ATTITUDE]), stamp)
            mode = None
            if Command.STATUS in replies:
                try:
                    mode = decode_flight_mode(replies[Command.STATUS], self.box_ids, self.fc_version)
                except ValueError as exc:
                    if self.rc_available:
                        raise
                    self.warn(str(exc))
            previous_rc = self.override.state
            if self.rc_available:
                self.override.tick(self.client, mode, time.monotonic_ns())
            arming_changed = mode is not None and mode.arming_disable_mask != self.last_arming_disable_mask
            if poll_status or arming_changed or previous_rc != self.override.state:
                self.next_status = now + 0.1
                self.publish_fc_status(True, mode)
            if previous_rc != self.override.state and self.override.state == 'restart_required':
                self.get_logger().error('Policy RC stopped. Return to manual and restart on the ground.')
            if poll_rc:
                self.next_rc += self.rc_period
                if self.next_rc <= now:
                    self.next_rc = now + self.rc_period
                try:
                    channels = decode_rc(replies[Command.RC])
                except ValueError as exc:
                    self.warn(str(exc))
                else:
                    self.publish_rc(channels, stamp, mode)
            if poll_battery:
                self.next_battery = now + self.battery_period
                try:
                    sample = decode_battery(replies[Command.BATTERY_STATE], self.battery_config)
                except ValueError as exc:
                    self.warn(str(exc))
                else:
                    self.publish_battery(sample, stamp)
        except LINK_ERRORS as exc:
            self.override.link_lost()
            self.disconnect()
            self.publish_fc_status(False)
            self.retry_at = time.monotonic() + 1.0
            self.warn(f'MSP link {self.device}: {exc}')

    def publish_fc_status(self, connected, mode=None):
        rc_state = 'disabled'
        if self.enable_rc:
            if self.rc_available or self.override.state == 'restart_required':
                rc_state = self.override.state
            else:
                rc_state = 'unavailable'
        version = '.'.join(map(str, self.fc_version)) if connected and self.fc_version else None
        self.last_arming_disable_mask = mode.arming_disable_mask if mode is not None else None
        self.fc_pub.publish(String(data=json.dumps({
            'connected': connected,
            'fc_version': version,
            **mode_status(mode),
            'arming_disable_flags': mode.arming_disable_flags if mode is not None else None,
            'arming_disable_mask': self.last_arming_disable_mask,
            'rc_session': self.override.session,
            'rc_state': rc_state,
            'stamp_ns': time.monotonic_ns(),
        })))

    def publish_rc(self, channels, stamp, mode):
        roll, pitch, yaw, throttle, *aux = channels
        self.rc_pub.publish(String(data=json.dumps({
            'stamp_ns': stamp.sec * 1_000_000_000 + stamp.nanosec,
            'roll_us': roll, 'pitch_us': pitch, 'yaw_us': yaw, 'throttle_us': throttle,
            'aux_us': aux,
            **mode_status(mode),
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
        self.rc_available = False
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
            if node.rc_available and node.override.session is not None:
                try:
                    node.client.release_rc()
                except LINK_ERRORS as exc:
                    node.warn(f'Error releasing MSP override: {exc}')
            node.disconnect()
            node.destroy_node()
        rclpy.try_shutdown()
