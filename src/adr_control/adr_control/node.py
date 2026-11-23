"""Run the racing policy from ROS observations and publish its normalized actions."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32MultiArray, String

from .flight_plan import FlightPlan

PERIOD_S = 0.01
MASK_SHAPE = (64, 64)
ACTION_ORDER = ('throttle', 'roll', 'pitch', 'yaw_rate')
FLU_TO_FRD = np.array([1, -1, -1], dtype=np.float32)
STANDARD_GRAVITY = 9.80665
FC_STATUS_TIMEOUT_NS = 250_000_000
QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def decode_mask(msg):
    if ((msg.height, msg.width) != MASK_SHAPE or msg.encoding != 'mono8'
            or msg.step < msg.width or len(msg.data) != msg.height * msg.step):
        raise ValueError('Expected a 64x64 mono8 mask with valid rows')
    mask = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
    if np.any((mask != 0) & (mask != 255)):
        raise ValueError('GateNet mask pixels must be 0 or 255')
    return mask.copy()


def imu_observation(msg):
    """Convert a body FLU IMU sample in ROS units into the policy's FRD inputs."""
    covariance = (msg.orientation_covariance, msg.angular_velocity_covariance,
                  msg.linear_acceleration_covariance)
    if any(values[0] == -1 for values in covariance):
        raise ValueError('Policy requires IMU orientation, angular velocity and acceleration')
    q = msg.orientation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError('Invalid IMU orientation quaternion')
    x, y, z, w = (value / norm for value in (q.x, q.y, q.z, q.w))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = -math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    rate = msg.angular_velocity
    accel = msg.linear_acceleration
    values = np.array([rate.x, rate.y, rate.z, accel.x, accel.y, accel.z, roll, pitch],
                      dtype=np.float32)
    if not np.isfinite(values).all():
        raise ValueError('Non-finite IMU observation')
    return {
        'rates': values[:3] * FLU_TO_FRD,
        'acc': values[3:6] * FLU_TO_FRD / STANDARD_GRAVITY,
        'att': values[6:],
    }


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    received_ns: int
    value: object

    def age(self, ros_now_ns, steady_now_ns):
        capture_age = ros_now_ns - self.stamp_ns
        if capture_age < 0:
            return None
        return max(capture_age, steady_now_ns - self.received_ns) / 1e9


class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')
        bundle_dir = self.declare_parameter('bundle_dir', '').value
        self.bench_imu = self.declare_parameter('bench_imu', False).value
        self.enable_rc = self.declare_parameter('enable_rc', False).value
        if self.enable_rc and self.bench_imu:
            raise ValueError('RC override requires measured IMU observations')
        self.mask_timeout = self.declare_parameter('mask_timeout_s', 0.15).value
        self.imu_timeout = self.declare_parameter('imu_timeout_s', 0.05).value
        if any(not math.isfinite(value) or value <= 0
               for value in (self.mask_timeout, self.imu_timeout)):
            raise ValueError('Observation timeouts must be finite and positive')
        if not bundle_dir:
            raise ValueError('Set bundle_dir to an exported deploy bundle')
        bundle = Path(bundle_dir).expanduser()
        for name in ('policy_meta.json', 'policy_weights.npz'):
            if not (bundle / name).is_file():
                raise FileNotFoundError(f'Missing policy bundle file: {bundle / name}')

        engine_dir = self.declare_parameter('engine_dir', '').value
        backend = self.declare_parameter('backend', 'tensorrt').value
        if backend == 'tensorrt':
            from adr_control.trt_policy import TensorRTPolicy
            self.policy = TensorRTPolicy(bundle, Path(engine_dir).expanduser() if engine_dir else bundle)
        elif backend == 'torch':
            import torch
            from adr_control.policy import Policy

            torch.set_num_threads(1)
            self.policy = Policy(bundle)
        else:
            raise ValueError(f'Unknown policy backend: {backend}')
        meta = self.policy.meta
        self.plan = FlightPlan(meta)
        if (meta['control']['freq_hz'] != 100 or tuple(meta['control']['action']) != ACTION_ORDER
                or tuple(meta['dims']['image']) != (*MASK_SHAPE, 1)):
            raise ValueError('Expected a 100 Hz deploy bundle with 64x64x1 masks and the standard action order')
        if self.enable_rc and (meta['control']['angle_limit_deg'] != 55.0
                               or not math.isclose(meta['control']['yaw_rate_max_dps'], 229.183118, abs_tol=0.5)):
            raise ValueError('RC override expects a policy trained for 55° ANGLE and 229°/s full-stick yaw')
        vector_sizes = {'rates': 3, 'acc': 3, 'att': 2, 'flight_plan': len(self.plan.vector)}
        if (set(self.policy.vec_keys) != vector_sizes.keys()
                or len(self.policy.vec_keys) != len(vector_sizes)
                or self.policy.vec_dims != [vector_sizes[key] for key in self.policy.vec_keys]):
            raise ValueError('Bundle observation dimensions must match rates, acc, att and flight_plan')

        self.mask = self.imu = None
        self.first = True
        self.state = 'waiting_for_mask'
        self.stopped = False
        self.last_step = None
        self.steps = self.overruns = 0
        self.last_report_steps = 0
        self.last_report_ns = time.monotonic_ns()
        self.action_pub = self.create_publisher(Float32MultiArray, '/adr/control/action', QOS)
        self.status_pub = self.create_publisher(String, '/adr/control/status', QOS)
        self.fc_session = self.session = None
        self.fc_stamp_ns = 0
        if self.enable_rc:
            self.state = 'waiting_for_override'
            self.rc_pub = self.create_publisher(String, '/adr/control/rc_action', QOS)
            self.create_subscription(String, '/adr/telemetry/fc_status', self.receive_fc, QOS)
        self.create_subscription(Image, '/adr/perception/mask', self.receive_mask, QOS)
        if self.bench_imu:
            self.bench_observation = {
                'rates': np.zeros(3, np.float32), 'acc': np.array([0, 0, -1], np.float32),
                'att': np.zeros(2, np.float32),
            }
            self.get_logger().info('Bench input: using a stationary, level IMU observation')
        else:
            self.create_subscription(Imu, '/adr/telemetry/imu', self.receive_imu, QOS)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(PERIOD_S, self.tick, clock=self.steady_clock)
        self.create_timer(1.0, self.report, clock=self.steady_clock)
        self.get_logger().info(f'100 Hz policy loaded on {self.policy.device} from {bundle}')

    def receive_mask(self, msg):
        self.mask = self.receive(msg, decode_mask, self.mask, self.mask_timeout)

    def receive_imu(self, msg):
        self.imu = self.receive(msg, imu_observation, self.imu, self.imu_timeout)

    def receive_fc(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                return
            stamp = status['stamp_ns']
            if (type(stamp) is not int or stamp <= self.fc_stamp_ns
                    or not 0 <= time.monotonic_ns() - stamp <= FC_STATUS_TIMEOUT_NS):
                return
        except (ValueError, KeyError):
            return
        self.fc_stamp_ns = stamp
        session = status.get('rc_session')
        self.fc_session = session if status.get('connected') is True and isinstance(session, str) else None

    def receive(self, msg, decode, previous, timeout):
        arrival = time.monotonic_ns()
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now = self.get_clock().now().nanoseconds
        if stamp_ns <= 0 or not 0 <= now - stamp_ns <= timeout * 1e9:
            return previous
        if previous is not None and stamp_ns <= previous.stamp_ns <= now:
            return previous
        try:
            return Sample(stamp_ns, arrival, decode(msg))
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return previous

    def reset_policy(self):
        self.policy.reset()
        self.plan.reset()
        self.first = True
        self.last_step = None

    def wait_for_input(self, state):
        if self.state == 'running' and not self.bench_imu:
            self.stopped = True
            state = f'stopped_{state}'
        if not self.first:
            self.reset_policy()
        self.state = state

    def input_ages(self, steady_now):
        ros_now = self.get_clock().now().nanoseconds
        return (self.mask.age(ros_now, steady_now) if self.mask is not None else None,
                self.imu.age(ros_now, steady_now) if self.imu is not None else None)

    def tick(self):
        if self.stopped:
            return
        started = time.monotonic_ns()
        if self.enable_rc:
            if self.fc_session is not None and started - self.fc_stamp_ns > FC_STATUS_TIMEOUT_NS:
                self.wait_for_input('stale_fc_status')
                return
            if self.fc_session is None:
                if self.session is not None:
                    self.reset_policy()
                    self.session = None
                self.state = 'waiting_for_override'
                return
            if self.session != self.fc_session:
                self.reset_policy()
                self.session = self.fc_session
                self.state = 'waiting_for_inputs'
        mask_age, imu_age = self.input_ages(started)
        if mask_age is None or mask_age > self.mask_timeout:
            self.wait_for_input('waiting_for_mask' if self.mask is None else 'stale_mask')
            return
        if not self.bench_imu and (imu_age is None or imu_age > self.imu_timeout):
            self.wait_for_input('waiting_for_imu' if self.imu is None else 'stale_imu')
            return

        obs = self.bench_observation if self.bench_imu else self.imu.value
        parts = dict(obs, flight_plan=self.plan.vector)
        vec = np.concatenate([parts[key] for key in self.policy.vec_keys])
        gate_index = self.plan.index
        action, gate_position = self.policy.step(
            self.mask.value, vec, self.first, mask_id=self.mask.stamp_ns)
        action, gate_position = action[0], gate_position[0]
        if not np.isfinite(action).all() or not np.isfinite(gate_position).all():
            raise RuntimeError('Policy returned a non-finite action or gate position')
        self.first = False
        finished = time.monotonic_ns()
        step_s = (finished - started) / 1e9
        self.overruns += step_s >= PERIOD_S
        mask_age, imu_age = self.input_ages(finished)
        if mask_age is None or mask_age > self.mask_timeout:
            self.wait_for_input('stale_mask')
            return
        if not self.bench_imu and (imu_age is None or imu_age > self.imu_timeout):
            self.wait_for_input('stale_imu')
            return
        passed = self.plan.update(float(gate_position[0]), PERIOD_S)
        self.steps += 1
        self.state = 'running'
        values = action.tolist()
        self.action_pub.publish(Float32MultiArray(data=values))
        if self.enable_rc:
            self.rc_pub.publish(String(data=json.dumps({
                'session': self.session, 'stamp_ns': finished, 'action': values,
            }, allow_nan=False)))
        self.last_step = {
            'step_ms': step_s * 1000,
            'gate_index': gate_index,
            'next_gate_index': self.plan.index,
            'gates_passed': self.plan.passed,
            'passed': passed,
            'gate_position_m': gate_position.tolist(),
        }

    def report(self):
        now = time.monotonic_ns()
        rate = 1e9 * (self.steps - self.last_report_steps) / (now - self.last_report_ns)
        self.last_report_ns, self.last_report_steps = now, self.steps
        mask_age, imu_age = self.input_ages(now)
        status = {
            'state': self.state,
            'bench_imu': self.bench_imu,
            'rc_enabled': self.enable_rc,
            'step_hz': rate,
            'steps': self.steps,
            'overruns': int(self.overruns),
            'mask_age_ms': None if mask_age is None else 1000 * mask_age,
            'imu_age_ms': None if imu_age is None else 1000 * imu_age,
        }
        if self.last_step is not None:
            status.update(self.last_step)
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = PolicyNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
