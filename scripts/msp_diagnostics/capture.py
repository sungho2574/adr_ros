"""Record the actual ADR MSP client and its serial reads without changing deadlines."""
import atexit
import datetime
import json
import os
import platform
import queue
import sys
import threading
import time
from types import SimpleNamespace


class Capture:
    def __init__(self, directory, **metadata):
        self.path = directory / f'events-{os.getpid()}.jsonl'
        self.output = self.path.open('x', encoding='utf-8')
        self.events = queue.Queue(maxsize=8192)
        self.dropped = 0
        self.writer = threading.Thread(target=self.write_events, daemon=True)
        self.writer.start()
        self.emit('session', utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  system=platform.platform(), **metadata)
        atexit.register(self.close)

    def emit(self, kind, **fields):
        try:
            self.events.put_nowait(dict(event=kind, time_ns=time.monotonic_ns(), **fields))
        except queue.Full:
            self.dropped += 1

    def write_events(self):
        try:
            while True:
                try:
                    event = self.events.get(timeout=0.5)
                except queue.Empty:
                    self.output.flush()
                    continue
                self.output.write(json.dumps(event, separators=(',', ':')) + '\n')
                if event['event'] == 'capture_end':
                    break
        except OSError as exc:
            print(f'MSP capture write failed: {exc}', file=sys.stderr, flush=True)

    def close(self):
        if self.output.closed:
            return
        end = dict(event='capture_end', time_ns=time.monotonic_ns(), dropped=self.dropped)
        while self.writer.is_alive():
            try:
                self.events.put(end, timeout=0.1)
                break
            except queue.Full:
                continue
        self.writer.join()
        self.output.close()


class SerialCapture:
    def __init__(self, port, capture):
        self.port = port
        self.capture = capture
        self.request = None
        self.port_id = time.monotonic_ns()
        capture.emit('port_open', port_id=self.port_id, port=port.port, baud=port.baudrate)

    def __getattr__(self, name):
        return getattr(self.port, name)

    def write(self, data):
        item = self.request
        if item is not None:
            item['tx_hex'] = data.hex()
            item['write_start_ns'] = time.monotonic_ns()
        try:
            return self.port.write(data)
        finally:
            if item is not None:
                item['write_end_ns'] = time.monotonic_ns()

    def read(self, size):
        data = self.port.read(size)
        if data and self.request is not None:
            self.request['rx'].append([time.monotonic_ns(), data.hex()])
        return data

    def close(self):
        try:
            return self.port.close()
        finally:
            self.capture.emit('port_close', port_id=self.port_id)


def traced_client(msp, capture):
    class TracedClient(msp.Client):
        def __init__(self, port, timeout):
            super().__init__(SerialCapture(port, capture), timeout)
            self.last_request_ns = None

        def _request(self, command, payload):
            started = time.monotonic_ns()
            self.last_request_ns = started
            item = dict(port_id=self.port.port_id, started_ns=started, command=command.name,
                        command_id=int(command), timeout_ms=self.timeout * 1000, rx=[])
            if command is msp.Command.MULTIPLE_MSP:
                item['children'] = [msp.Command(value).name for value in payload]
            self.port.request = item
            try:
                answer = super()._request(command, payload)
            except Exception as exc:
                item.update(error_type=type(exc).__name__, error=str(exc))
                raise
            else:
                item['response_bytes'] = len(answer)
                return answer
            finally:
                item['finished_ns'] = time.monotonic_ns()
                self.port.request = None
                capture.emit('request', **item)

        def read_batch(self, *commands):
            previous = self.last_request_ns
            try:
                return super().read_batch(*commands)
            except ValueError as exc:
                capture.emit('batch_error', port_id=self.port.port_id,
                             started_ns=self.last_request_ns if self.last_request_ns != previous else None,
                             children=[msp.Command(value).name for value in commands], error=str(exc))
                raise

    return TracedClient


def time_tick(capture, started_ns, previous_ns, period_ns=None):
    finished = time.monotonic_ns()
    capture.emit('tick', started_ns=started_ns, duration_ns=finished - started_ns,
                 gap_ns=None if previous_ns is None else started_ns - previous_ns,
                 period_ns=period_ns)


def trace_ros_node(node, msp, capture):
    node.Client = traced_client(msp, capture)
    original_init = node.MspTelemetry.__init__
    original_tick = node.MspTelemetry.tick
    original_imu = node.MspTelemetry.publish_imu
    original_warn = node.MspTelemetry.warn

    def init(self, *args, **kwargs):
        original_init(self, *args, **kwargs)
        settings = {key: self.get_parameter(key).value for key in (
            'port', 'baud', 'imu_hz', 'battery_hz', 'rc_hz', 'response_timeout_s', 'enable_rc')}
        self._diag_period_ns = round(1e9 / settings['imu_hz'])
        capture.emit('node_settings', **settings)
        publisher = self.fc_pub

        def publish_status(message):
            publisher.publish(message)
            capture.emit('fc_status', **json.loads(message.data))

        self.fc_pub = SimpleNamespace(publish=publish_status)

    def tick(self):
        started = time.monotonic_ns()
        previous = getattr(self, '_diag_previous_tick', None)
        self._diag_previous_tick = started
        try:
            return original_tick(self)
        finally:
            time_tick(capture, started, previous, self._diag_period_ns)

    def imu(self, sample, stamp):
        result = original_imu(self, sample, stamp)
        if any(sample.acceleration_raw):
            capture.emit('imu_publish', stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec)
        return result

    def warn(self, message):
        capture.emit('warning', message=message)
        return original_warn(self, message)

    node.MspTelemetry.__init__ = init
    node.MspTelemetry.tick = tick
    node.MspTelemetry.publish_imu = imu
    node.MspTelemetry.warn = warn
