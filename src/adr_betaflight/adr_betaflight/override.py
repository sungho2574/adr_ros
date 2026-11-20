"""Pilot handoff and freshness checks for MSP stick commands."""
import json
from uuid import uuid4


ACTION_TIMEOUT_NS = 50_000_000
HANDOFF_TIMEOUT_NS = 250_000_000


def rc_from_action(action):
    """Map the onboard policy's throttle, roll, pitch and yaw to AETR pulses."""
    if (not isinstance(action, (tuple, list)) or len(action) != 4
            or any(type(value) not in (int, float) or not -1 <= value <= 1 for value in action)):
        raise ValueError('Expected four normalized policy actions')
    throttle, roll, pitch, yaw = action
    return (round(1500 + 500 * roll), round(1500 - 500 * pitch),
            round(1000 + 500 * (throttle + 1)), round(1500 + 500 * yaw))


class Override:
    def __init__(self):
        self.session = None
        self.state = 'waiting_for_disarm'
        self.started_ns = 0
        self.last_sent_ns = None
        self.last_stamp_ns = 0
        self.pending = None

    def receive(self, data, now_ns):
        if self.session is None:
            return
        try:
            command = json.loads(data)
            if not isinstance(command, dict) or command.get('session') != self.session:
                return
            stamp = command.get('stamp_ns')
            if (type(stamp) is not int or stamp <= self.last_stamp_ns
                    or not self.started_ns <= stamp <= now_ns
                    or now_ns - stamp > ACTION_TIMEOUT_NS):
                return
            channels = rc_from_action(command['action'])
        except (ValueError, TypeError, KeyError):
            return
        self.last_stamp_ns = stamp
        self.pending = stamp, channels

    def stop(self, state):
        self.session = None
        self.pending = None
        self.last_sent_ns = None
        self.state = state

    def link_lost(self):
        if self.session is not None or self.state == 'restart_required':
            self.stop('restart_required')
        else:
            self.stop('waiting_for_disarm')

    def tick(self, client, mode, now_ns):
        if self.state == 'restart_required':
            return
        if self.session is None:
            if not mode.armed:
                self.state = 'waiting_for_disarm' if mode.override or mode.failsafe else 'ready'
            elif self.state == 'ready' and mode.override:
                if mode.angle and not mode.failsafe:
                    self.session = uuid4().hex
                    self.started_ns = now_ns
                    self.last_stamp_ns = 0
                    self.state = 'waiting_for_action'
                else:
                    self.state = 'waiting_for_disarm'
            return

        if mode.failsafe or not (mode.armed and mode.angle and mode.override):
            client.release_rc()
            self.stop('restart_required' if mode.failsafe else 'waiting_for_disarm')
            return

        expired = (now_ns - self.started_ns > HANDOFF_TIMEOUT_NS if self.last_sent_ns is None
                   else now_ns - self.last_sent_ns > ACTION_TIMEOUT_NS)
        if expired:
            client.release_rc()
            self.stop('restart_required')
            return

        if self.pending is not None:
            stamp, channels = self.pending
            self.pending = None
            if now_ns - stamp <= ACTION_TIMEOUT_NS:
                client.write_rc(channels)
                self.last_sent_ns = now_ns
                self.state = 'active'
