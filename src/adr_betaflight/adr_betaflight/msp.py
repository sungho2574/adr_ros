"""MSP v1 transport for Betaflight telemetry and RC override."""
from dataclasses import dataclass
from enum import IntEnum
import math
import select
import struct
import time


class Command(IntEnum):
    FC_VARIANT = 2
    FC_VERSION = 3
    BATTERY_CONFIG = 32
    FEATURE_CONFIG = 36
    RX_MAP = 64
    STATUS = 101
    RAW_IMU = 102
    ATTITUDE = 108
    BOX_IDS = 119
    BATTERY_STATE = 130
    SET_RAW_RC = 200
    MULTIPLE_MSP = 230


class RejectedCommand(RuntimeError):
    """Betaflight returned an MSP error reply."""


def checksum(data):
    value = 0
    for byte in data:
        value ^= byte
    return value


def extract_reply(buffer):
    """Return the first valid reply from a byte stream, leaving incomplete frames buffered."""
    while True:
        start = buffer.find(b'$M')
        if start < 0:
            del buffer[:-1]
            return None
        del buffer[:start]
        if len(buffer) < 6:
            return None
        if buffer[2] not in b'<>!':
            del buffer[0]
            continue
        end = 6 + buffer[3]
        if len(buffer) < end:
            return None
        if checksum(buffer[3:end - 1]) != buffer[end - 1]:
            del buffer[0]
            continue
        direction, command = chr(buffer[2]), buffer[4]
        payload = bytes(buffer[5:end - 1])
        del buffer[:end]
        if direction != '<':
            return direction, command, payload


class Client:
    """Exchange one request at a time. The caller reopens the port after a timeout."""

    def __init__(self, port, timeout):
        self.port = port
        self.timeout = timeout

    def read(self, command):
        command = Command(command)
        if command is Command.MULTIPLE_MSP:
            raise ValueError('Use read_batch for MSP_MULTIPLE_MSP')
        if command is Command.SET_RAW_RC:
            raise ValueError('Use write_rc or release_rc for MSP_SET_RAW_RC')
        return self._request(command, b'')

    def read_box_ids(self):
        """Read the permanent mode IDs in the order used by MSP_STATUS."""
        ids = bytearray()
        for page in range(4):
            part = self._request(Command.BOX_IDS, bytes((page,)))
            if len(part) > 32:
                raise ValueError('MSP_BOXIDS returned an oversized page')
            ids.extend(part)
            if len(part) < 32:
                break
        if 0 not in ids or len(ids) != len(set(ids)):
            raise ValueError('Invalid Betaflight mode IDs')
        return bytes(ids)

    def read_batch(self, *commands):
        commands = tuple(Command(cmd) for cmd in commands)
        if not commands or {Command.MULTIPLE_MSP, Command.SET_RAW_RC}.intersection(commands):
            raise ValueError('Expected a nonempty list of MSP telemetry commands')
        payload = self._request(Command.MULTIPLE_MSP, bytes(commands))
        replies = []
        offset = 0
        for _ in commands:
            if offset == len(payload):
                raise ValueError('MSP_MULTIPLE_MSP omitted a reply')
            end = offset + 1 + payload[offset]
            if end > len(payload):
                raise ValueError('Truncated MSP_MULTIPLE_MSP reply')
            replies.append(payload[offset + 1:end])
            offset = end
        if offset != len(payload):
            raise ValueError('Unexpected data after MSP_MULTIPLE_MSP replies')
        return replies

    def write_rc(self, channels):
        self._write_rc(struct.pack('<4H', *channels))

    def release_rc(self):
        # Betaflight 2026.6+ falls back to the receiver for omitted channels.
        self._write_rc(b'')

    def _write_rc(self, payload):
        if self._request(Command.SET_RAW_RC, payload):
            raise ValueError('Unexpected MSP_SET_RAW_RC reply payload')

    def _request(self, command, payload):
        body = bytes((len(payload), command)) + payload
        frame = b'$M<' + body + bytes((checksum(body),))
        deadline = time.monotonic() + self.timeout
        if self.port.write(frame) != len(frame):
            raise OSError(f'Short MSP write for {command.name}')

        buffer = bytearray()
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.port], [], [], remaining)[0]:
                raise TimeoutError(f'No MSP reply for {command.name}')
            buffer.extend(self.port.read(min(self.port.in_waiting or 1, 256)))
            while (reply := extract_reply(buffer)) is not None:
                direction, received, data = reply
                if received == command:
                    if direction == '!':
                        raise RejectedCommand(f'Betaflight rejected {command.name}')
                    return data


@dataclass(frozen=True)
class ImuSample:
    acceleration_raw: tuple[int, int, int]
    angular_velocity: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


def decode_imu(raw, attitude):
    if len(raw) != 18 or len(attitude) != 6:
        raise ValueError('Invalid Betaflight RAW_IMU or ATTITUDE reply length')
    ax, ay, az, gx, gy, gz = struct.unpack_from('<6h', raw)
    roll, pitch, heading = struct.unpack('<3h', attitude)
    angular_velocity = tuple(math.radians(value) for value in (gx, gy, gz))

    # MSP uses body FLU and clockwise heading. ROS ENU yaw increases counterclockwise from east.
    h_roll = math.radians(roll / 10) / 2
    h_pitch = math.radians(pitch / 10) / 2
    h_yaw = math.radians(90 - heading) / 2
    sr, cr = math.sin(h_roll), math.cos(h_roll)
    sp, cp = math.sin(h_pitch), math.cos(h_pitch)
    sy, cy = math.sin(h_yaw), math.cos(h_yaw)
    orientation = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    return ImuSample((ax, ay, az), angular_velocity, orientation)


@dataclass(frozen=True)
class FlightMode:
    armed: bool
    override: bool
    angle: bool
    failsafe: bool


def decode_flight_mode(payload, box_ids):
    if len(payload) < 16:
        raise ValueError('Truncated MSP_STATUS reply')
    extra_bytes = payload[15] & 0x0F
    end = 16 + extra_bytes
    if len(payload) < end or len(box_ids) > 8 * (4 + extra_bytes):
        raise ValueError('Truncated MSP_STATUS mode flags')
    modes = int.from_bytes(payload[6:10] + payload[16:end], 'little')
    # MSP packs only boxes available on the FC, in the same order as MSP_BOXIDS.
    def active(box_id):
        return box_id in box_ids and bool(modes & (1 << box_ids.index(box_id)))

    return FlightMode(armed=active(0), override=active(50), angle=active(1), failsafe=active(27))


def check_override_config(client, box_ids):
    version = client.read(Command.FC_VERSION)
    if len(version) < 3 or not 1 <= version[1] <= 12 or tuple(version[:3]) < (26, 6, 0):
        raise ValueError('RC override requires Betaflight 2026.6 or newer for receiver fallback')
    features = client.read(Command.FEATURE_CONFIG)
    rx_msp = 1 << 14
    physical_rx = (1 << 0) | (1 << 3) | (1 << 13) | (1 << 25)  # PPM, serial, PWM, SPI
    enabled = int.from_bytes(features, 'little')
    if len(features) != 4 or enabled & rx_msp or not enabled & physical_rx:
        raise ValueError('RC override requires a physical receiver and feature RX_MSP disabled')
    rx_map = client.read(Command.RX_MAP)
    # MSP_RX_MAP stores wire positions in internal roll, pitch, yaw, throttle order.
    if rx_map[:4] != bytes((0, 1, 3, 2)):
        raise ValueError('RC override requires the receiver channel map AETR')
    if box_ids is None or 1 not in box_ids or 50 not in box_ids:
        raise ValueError('RC override requires ANGLE and MSP OVERRIDE modes')


@dataclass(frozen=True)
class BatteryConfig:
    current_meter: bool
    min_cell_v: float
    max_cell_v: float


def decode_battery_config(payload):
    if len(payload) == 7:
        min_cell_v, max_cell_v = (value / 10 for value in payload[:2])
    elif len(payload) >= 13:
        min_cell_v, max_cell_v = (value / 100 for value in struct.unpack_from('<2H', payload, 7))
    else:
        raise ValueError('Invalid Betaflight BATTERY_CONFIG reply length')
    return BatteryConfig(bool(payload[6]), min_cell_v, max_cell_v)


@dataclass(frozen=True)
class BatterySample:
    cells: int
    voltage: float
    current: float
    design_capacity: float
    charge: float
    percentage: float


def decode_battery(payload, config):
    if len(payload) != 9 and len(payload) < 11:
        raise ValueError('Invalid Betaflight BATTERY_STATE reply length')
    cells, capacity, decivolts, drawn, centiamps, _ = struct.unpack_from('<BHBHhB', payload)
    voltage = struct.unpack_from('<H', payload, 9)[0] / 100 if len(payload) >= 11 else decivolts / 10
    design_capacity = capacity / 1000 if capacity else math.nan
    current = charge = percentage = math.nan
    if cells and config is not None:
        if config.current_meter:
            current = -centiamps / 100
        if capacity and config.current_meter:
            charge = max(0.0, (capacity - drawn) / 1000)
            percentage = charge / design_capacity
        elif voltage > 0 and 0 < config.min_cell_v < config.max_cell_v:
            fraction = (voltage / cells - config.min_cell_v) / (config.max_cell_v - config.min_cell_v)
            percentage = min(1.0, max(0.0, fraction))
    return BatterySample(cells, voltage, current, design_capacity, charge, percentage)
