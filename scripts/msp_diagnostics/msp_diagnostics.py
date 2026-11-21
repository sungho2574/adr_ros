#!/usr/bin/env python3
"""Run the read-only MSP probe, capture a normal ADR launch, or analyze captures."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from capture import Capture, time_tick, traced_client
from report import event_paths, summarize


SCRIPT = Path(__file__).resolve().parent
REPO = SCRIPT.parents[1]


def output_directory(args):
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    out = args.out or REPO / 'flight_logs/msp_diagnostics' / f'{stamp}-{args.mode}'
    out = out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)
    return out


def probe(args, out):
    import serial

    sys.path.insert(0, str(REPO / 'src/adr_betaflight'))
    from adr_betaflight import msp

    options = {key: value for key, value in vars(args).items() if key != 'out'}
    capture = Capture(out, **options)
    client_class = traced_client(msp, capture)
    cmd = msp.Command
    rates = {cmd.BATTERY_STATE: args.battery_hz, cmd.STATUS: args.status_hz, cmd.RC: args.rc_hz}
    intervals = {command: round(1e9 / hz) for command, hz in rates.items() if hz > 0}
    due = dict.fromkeys(intervals, 0)
    period = round(1e9 / args.hz)
    end_ns = time.monotonic_ns() + round(args.seconds * 1e9)
    previous = None
    client = None
    timeout = args.timeout_ms / 1000
    try:
        while (started := time.monotonic_ns()) < end_ns:
            if client is None:
                try:
                    port = serial.Serial(args.port, args.baud, timeout=0,
                                         write_timeout=timeout, exclusive=True)
                except OSError as exc:
                    capture.emit('open_error', message=str(exc))
                    print(exc, file=sys.stderr)
                    return 1
                client = client_class(port, timeout)
                port.reset_input_buffer()

            if args.profile == 'variant':
                commands = [cmd.FC_VARIANT]
            else:
                commands = [cmd.RAW_IMU, cmd.ATTITUDE]
                if args.profile == 'full':
                    poll_rc = cmd.RC in due and started >= due[cmd.RC]
                    for command, interval in intervals.items():
                        if started >= due[command]:
                            commands.append(command)
                            missed = (started - due[command]) // interval
                            due[command] += (missed + 1) * interval
                        elif command is cmd.STATUS and poll_rc:
                            commands.append(command)

            try:
                if args.profile == 'variant' or args.individual:
                    replies = [client.read(command) for command in commands]
                else:
                    replies = client.read_batch(*commands)
                sizes = dict(zip(commands, replies))
                if cmd.FC_VARIANT in sizes and sizes[cmd.FC_VARIANT] != b'BTFL':
                    raise ValueError(f'Unexpected FC variant {sizes[cmd.FC_VARIANT]!r}')
                if cmd.RAW_IMU in sizes:
                    msp.decode_imu(sizes[cmd.RAW_IMU], sizes[cmd.ATTITUDE])
                if cmd.STATUS in sizes:
                    msp.decode_flight_mode(sizes[cmd.STATUS], None)
                if cmd.RC in sizes:
                    msp.decode_rc(sizes[cmd.RC])
                if cmd.BATTERY_STATE in sizes:
                    msp.decode_battery(sizes[cmd.BATTERY_STATE], None)
            except (OSError, ValueError, msp.RejectedCommand) as exc:
                capture.emit('warning', message=str(exc))
                client.port.close()
                client = None
            finally:
                time_tick(capture, started, previous, period)
                previous = started

            wake_ns = time.monotonic_ns() + 1_000_000_000 if client is None else started + period
            remaining = (min(wake_ns, end_ns) - time.monotonic_ns()) / 1e9
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        return 130
    finally:
        if client is not None:
            client.port.close()
        capture.close()
    return 0


def live(args, out):
    env = dict(os.environ, ADR_MSP_CAPTURE_DIR=str(out))
    env['PYTHONPATH'] = os.pathsep.join(filter(None, (str(SCRIPT / 'hook'), str(SCRIPT),
                                                     env.get('PYTHONPATH'))))
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    process = subprocess.Popen(command, env=env, start_new_session=True)
    try:
        return process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        return 130


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('expected a finite positive number')
    return number


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError('expected a finite nonnegative number')
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    poll = sub.add_parser('probe', help='issue only read requests on the UART')
    poll.add_argument('--port', default='/dev/ttyTHS1')
    poll.add_argument('--baud', type=int, required=True)
    poll.add_argument('--profile', choices=('variant', 'imu', 'full'), default='imu')
    poll.add_argument('--individual', action='store_true', help='send each read as a separate MSP request')
    poll.add_argument('--hz', type=positive, default=25.0)
    poll.add_argument('--status-hz', type=nonnegative, default=10.0)
    poll.add_argument('--rc-hz', type=nonnegative, default=25.0)
    poll.add_argument('--battery-hz', type=nonnegative, default=2.0)
    poll.add_argument('--seconds', type=positive, default=180.0)
    poll.add_argument('--timeout-ms', type=positive, default=25.0)

    run = sub.add_parser('live', help='capture the MSP client inside a normal ros2 launch or run')
    run.add_argument('command', nargs=argparse.REMAINDER)

    for command in (poll, run):
        command.add_argument('--fc-serial-hz', type=positive,
                             help='record the FC serial_update_rate_hz for this run')

    report = sub.add_parser('analyze', help='recreate the reports from saved JSONL files')
    report.add_argument('inputs', nargs='+')
    for command in (poll, run, report):
        command.add_argument('--out', type=Path)
        command.add_argument('--deadline-ms', type=positive, default=25.0,
                             help='flag a valid response as late above this threshold')

    args = parser.parse_args()
    if args.mode == 'probe' and args.baud <= 0:
        poll.error('--baud must be positive')
    if args.mode == 'live' and args.command in ([], ['--']):
        run.error('pass the launch command after --')
    if args.mode == 'analyze':
        paths = event_paths(args.inputs)
        if not paths:
            report.error('no events-*.jsonl files found')
        summarize(paths, args.out or paths[0].parent, args.deadline_ms)
        return 0

    out = output_directory(args)
    metadata = {key: value for key, value in vars(args).items() if key not in ('out', 'command')}
    (out / 'run.json').write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    print(f'MSP diagnostics output: {out}', flush=True)
    status = probe(args, out) if args.mode == 'probe' else live(args, out)
    paths = event_paths([out])
    if not paths:
        print('No capture from msp_telemetry. Check the sourced ROS workspace and the launch output.',
              file=sys.stderr)
        return status or 1
    summarize(paths, out, args.deadline_ms)
    return status


if __name__ == '__main__':
    raise SystemExit(main())
