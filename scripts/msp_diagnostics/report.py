"""Summarize captured transactions and independently inspect the received bytes."""
from bisect import bisect_left
from collections import Counter, defaultdict
import csv
from itertools import accumulate
import json
from pathlib import Path
from statistics import quantiles


def latency_ms(later, earlier):
    if later is None or earlier is None:
        return None
    return round((later - earlier) / 1e6, 3)


def timing(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    p = quantiles(values, n=100, method='inclusive') if len(values) > 1 else values * 99
    return {key: round(value, 3) for key, value in
            zip(('p50', 'p95', 'p99', 'max'), (p[49], p[94], p[98], max(values)))}


def inspect_rx(chunks, command_id):
    data = [bytes.fromhex(raw) for _, raw in chunks]
    rx = b''.join(data)
    offsets = list(accumulate(map(len, data)))
    stats = dict(rx_bytes=len(rx), bad_checksum=0, incomplete_frames=0,
                 unexpected_frames=0, echoed_frames=0, unframed_bytes=0)
    response = None
    cursor = 0
    while cursor < len(rx):
        start = rx.find(b'$M', cursor)
        if start == -1:
            partial_header = int(rx.endswith(b'$'))
            stats['unframed_bytes'] += len(rx) - cursor - partial_header
            stats['incomplete_frames'] += partial_header
            break
        stats['unframed_bytes'] += start - cursor
        cursor = start + 2
        if start + 6 > len(rx):
            stats['incomplete_frames'] += 1
            break
        if rx[start + 2] not in b'<>!':
            stats['unframed_bytes'] += 2
            continue
        end = start + 6 + rx[start + 3]
        if end > len(rx):
            stats['incomplete_frames'] += 1
            continue
        xor = 0
        for value in rx[start + 3:end]:
            xor ^= value
        if xor:
            stats['bad_checksum'] += 1
            continue
        cursor = end
        if rx[start + 2] == ord('<'):
            stats['echoed_frames'] += 1
        elif rx[start + 4] != command_id:
            stats['unexpected_frames'] += 1
        elif response is None:
            response = chunks[bisect_left(offsets, end)][0]
    return stats, response


def request_row(event, port, deadline_ms):
    received, seen_ns = inspect_rx(event['rx'], event['command_id'])
    first_ns = event['rx'][0][0] if event['rx'] else None
    started_ns = event['started_ns']
    first_ms = latency_ms(first_ns, started_ns)
    valid_ms = latency_ms(seen_ns, started_ns)
    error = event.get('error_type')
    if error is None:
        late = seen_ns is not None and seen_ns - started_ns > deadline_ms * 1e6
        outcome = 'late' if late else 'ok'
    elif error != 'TimeoutError':
        outcome = error
    elif seen_ns is not None:
        outcome = 'valid_frame_not_accepted'
    elif not received['rx_bytes']:
        outcome = 'no_rx_bytes'
    elif received['bad_checksum']:
        outcome = 'checksum_error'
    elif received['incomplete_frames']:
        outcome = 'incomplete_frame'
    else:
        outcome = 'no_matching_frame'
    children = '+'.join(event.get('children', ()))
    command = event['command'] + (f'[{children}]' if children else '')
    baud = port.get('baud')
    tx_bytes = len(event.get('tx_hex', '')) // 2
    wire_ms = round((tx_bytes + received['rx_bytes']) * 10000 / baud, 3) if baud else None
    return dict(port_id=event['port_id'], baud=baud, command=command, outcome=outcome,
                started_ns=started_ns, timeout_ms=event['timeout_ms'],
                write_ms=latency_ms(event.get('write_end_ns'), event.get('write_start_ns')),
                first_rx_ms=first_ms, frame_ms=valid_ms,
                total_ms=latency_ms(event['finished_ns'], started_ns),
                wire_ms=wire_ms, tx_bytes=tx_bytes, **received, error=event.get('error', ''))


def rate(stamps):
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return None
    return round((len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0]), 3)


def summarize(paths, out, deadline_ms):
    ports, groups = {}, defaultdict(list)
    by_transaction = {}
    kinds, warnings, batch_errors = Counter(), Counter(), Counter()
    sessions, settings, ticks, imu, connection_changes, rows = [], [], [], [], [], []
    dropped = invalid_lines = 0
    previous_status = {}
    reopens = 0
    for path in paths:
        opened = False
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    invalid_lines += 1
                    continue
                kind = event['event']
                kinds[kind] += 1
                if kind == 'session':
                    sessions.append(event)
                elif kind == 'node_settings':
                    settings.append(event)
                elif kind == 'port_open':
                    ports[event['port_id']] = event
                    reopens += int(opened)
                    opened = True
                elif kind == 'request':
                    row = request_row(event, ports.get(event['port_id'], {}), deadline_ms)
                    groups[(row['baud'], row['command'])].append(row)
                    rows.append(row)
                    by_transaction[(row['port_id'], row['started_ns'])] = row
                elif kind == 'tick':
                    ticks.append(event)
                elif kind == 'imu_publish':
                    imu.append(event['time_ns'])
                elif kind == 'fc_status':
                    state = event['connected'], event['rc_state']
                    if previous_status.get(path) != state:
                        connection_changes.append(event)
                        previous_status[path] = state
                elif kind in ('warning', 'open_error'):
                    warnings[event['message']] += 1
                elif kind == 'batch_error':
                    batch_errors[event['error']] += 1
                    row = by_transaction.get((event['port_id'], event.get('started_ns')))
                    if row is not None:
                        row.update(outcome='batch_error', error=event['error'])
                elif kind == 'capture_end':
                    dropped += event['dropped']

    summary = {
        'nominal_deadline_ms': deadline_ms, 'events': kinds, 'sessions': sessions,
        'ports': list(ports.values()), 'reopens': reopens,
        'node_settings': settings, 'connection_changes': connection_changes,
        'warnings': warnings, 'batch_errors': batch_errors, 'dropped_events': dropped,
        'invalid_json_lines': invalid_lines, 'incomplete_captures': len(sessions) - kinds['capture_end'],
        'tick_hz': rate(sorted(t['started_ns'] for t in ticks)), 'imu_publish_hz': rate(sorted(imu)),
        'imu_publish_gap_ms': timing(latency_ms(b, a) for a, b in zip(imu, imu[1:]) if b >= a),
        'tick_duration_ms': timing(t['duration_ns'] / 1e6 for t in ticks),
        'tick_gap_ms': timing(t['gap_ns'] / 1e6 for t in ticks if t['gap_ns'] is not None),
        'tick_over_budget': sum(t['duration_ns'] > t['period_ns'] for t in ticks
                                if t['period_ns'] is not None),
        'requests': [
            dict(baud=baud, command=command, count=len(items),
                 outcomes=Counter(row['outcome'] for row in items),
                 first_rx_ms=timing(row['first_rx_ms'] for row in items),
                 frame_ms=timing(row['frame_ms'] for row in items),
                 total_ms=timing(row['total_ms'] for row in items),
                 wire_ms=timing(row['wire_ms'] for row in items),
                 tx_bytes=sum(row['tx_bytes'] for row in items),
                 rx_bytes=sum(row['rx_bytes'] for row in items),
                 bad_checksum=sum(row['bad_checksum'] for row in items))
            for (baud, command), items in groups.items()
        ],
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    if rows:
        with (out / 'requests.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    print(f'{len(rows)} requests, {kinds["port_open"]} UART opens, report: {out / "summary.json"}')


def event_paths(inputs):
    files = []
    for value in inputs:
        path = Path(value)
        files.extend(sorted(path.glob('events-*.jsonl')) if path.is_dir() else [path])
    return list(dict.fromkeys(files))
