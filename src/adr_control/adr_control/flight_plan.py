"""Track geometry and gate transitions for the policy."""

import numpy as np


class FlightPlan:
    """Which gate is next.  It is a policy INPUT (24 numbers of track geometry), not a pass/fail
    check: the index moves on when the decoded position along the gate normal x_g > -0.15 m
    (SkyDreamer II-H), re-arms once x_g < -0.5 m, at most once per 0.25 s."""

    def __init__(self, meta):
        plan = meta["flight_plan"]
        self.table = np.asarray(plan["table"], np.float32)
        self.start_index = plan.get("start_index", 0)
        if (self.table.ndim != 2 or not np.isfinite(self.table).all()
                or not isinstance(self.start_index, int)
                or not 0 <= self.start_index < len(self.table)):
            raise ValueError("Invalid flight-plan table or start_index")
        self.pass_x = float(plan.get("pass_x", -0.15))
        self.rearm_x = float(plan.get("rearm_x", -0.5))
        self.min_gap = float(plan.get("min_gap_s", 0.25))
        if (not np.isfinite([self.pass_x, self.rearm_x, self.min_gap]).all()
                or self.rearm_x >= self.pass_x or self.min_gap < 0):
            raise ValueError("Invalid flight-plan transition thresholds")
        self.reset()

    def reset(self):
        self.index, self.passed, self.armed, self.since = self.start_index, 0, True, 1e9

    @property
    def vector(self):
        return self.table[self.index]

    def update(self, x_g, dt):
        self.since += dt
        if not self.armed and x_g < self.rearm_x:
            self.armed = True
        if self.armed and x_g > self.pass_x and self.since >= self.min_gap:
            self.index = (self.index + 1) % len(self.table)
            self.passed += 1
            self.armed, self.since = False, 0.0
            return True
        return False
