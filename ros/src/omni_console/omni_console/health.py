"""The robot's health over time (the console's Health tab), ROS-free: a 1 Hz history of the last 30 minutes, lifetime
counters, and maintenance reminders.

History: pack voltage and estimated charge, the robot computer's load, temperature and GPU, each drive motor's
winding and driver temperature (MCU INFO, protocol 3), wheel errors. Counters (health.json, kept across restarts):
distance driven (from the odometry), hours with the software running, throws seen, catches, balls fired. A reminder
comes due when its counter passes the interval since it was last marked done. The intervals are estimates (est):
nothing has worn on a real robot yet.
"""

from __future__ import annotations

import math
from collections import deque
from pathlib import Path

HISTORY_S = 1800
# item: (counter, interval, what to do)  (est: guesses until the robot has worn something)
MAINTENANCE = {
    "rollers": ("km", 20.0, "Check the omni wheels' TPU rollers for flat spots and the roller pins for play"),
    "pods": ("hours", 25.0, "Check the suspension pods' screws and the shocks' preload"),
    "flywheels": ("fires", 2000.0, "Check the flywheel tyres for wear and the flywheels for balance"),
    "feed": ("fires", 2000.0, "Clean the feed wheels and the chute; check the feed-exit beam"),
    "battery": ("hours", 50.0, "Check the pack's balance (cell voltages) and its connector"),
}
COUNTERS = ("km", "hours", "throws", "catches", "fires")


_GPU = []                          # the load file, found once ([None]: there is none)


def gpu_load() -> float | None:
    """The Jetson's GPU load, % (its devfreq 'load' file, 0..1000; VERIFY the path on the Orin), or None."""
    if not _GPU:
        found = None
        for pattern in ("sys/devices/platform/*gpu*/load", "sys/devices/gpu.0/load", "sys/devices/platform/gpu.0/load"):
            found = found or next(iter(sorted(Path("/").glob(pattern))), None)
        _GPU.append(found)
    if _GPU[0] is None:
        return None
    try:
        return round(int(_GPU[0].read_text().strip()) / 10.0, 1)
    except (OSError, ValueError):
        return None


class Health:
    def __init__(self, saved: dict | None = None):
        saved = saved or {}
        self.counters = {k: float((saved.get("counters") or {}).get(k, 0.0)) for k in COUNTERS}
        self.done_at = {k: float((saved.get("done_at") or {}).get(k, 0.0)) for k in MAINTENANCE}
        self.hist: deque = deque(maxlen=HISTORY_S)
        self.last_xy = None
        self.changed = False

    def sample(self, t: float, mcu: dict | None, info: dict | None, sys: dict | None, odom_xy=None, dt: float = 1.0,
               running: bool = True):
        """One second. odom_xy: the odometry position (for the distance driven)."""
        if running:
            self.counters["hours"] += dt / 3600.0
        if odom_xy is not None:
            if self.last_xy is not None:
                d = math.hypot(odom_xy[0] - self.last_xy[0], odom_xy[1] - self.last_xy[1])
                if d < 5.0:                                  # a jump is an odometry reset, not driving
                    self.counters["km"] += d / 1000.0
            self.last_xy = odom_xy
        row = dict(t=round(t, 1), v=mcu.get("v_bus") if mcu else None, batt=mcu.get("battery_pct") if mcu else None,
                   load=sys.get("load") if sys else None, temp=sys.get("temp_c") if sys else None, gpu=gpu_load(),
                   motor=list(info.get("motor_temp") or []) if info else [],
                   driver=list(info.get("driver_temp") or []) if info else [],
                   werr=sum(1 for e in (mcu.get("dm_err") or []) if e not in (0, 1)) if mcu else 0)
        self.hist.append(row)
        self.changed = True
        return row

    def count(self, key: str, n: float = 1.0):
        self.counters[key] += n
        self.changed = True

    def due(self) -> list:
        out = []
        for item, (ctr, every, what) in MAINTENANCE.items():
            since = self.counters[ctr] - self.done_at[item]
            out.append(dict(item=item, what=what, counter=ctr, every=every, since=round(since, 2),
                            due=since >= every, left=round(every - since, 2)))
        return out

    def done(self, item: str):
        if item not in MAINTENANCE:
            raise ValueError(f"no maintenance item '{item}'")
        self.done_at[item] = self.counters[MAINTENANCE[item][0]]
        self.changed = True

    def saved(self) -> dict:
        return dict(counters={k: round(v, 4) for k, v in self.counters.items()}, done_at=dict(self.done_at))

    def report(self, step: int = 1, since: float = -math.inf) -> dict:
        rows = [r for r in self.hist if r["t"] > since][::max(1, step)]
        return dict(counters={k: round(v, 3) for k, v in self.counters.items()}, maintenance=self.due(), history=rows)
