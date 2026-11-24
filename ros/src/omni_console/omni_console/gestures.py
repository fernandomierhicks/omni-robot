"""The base's feelings, ROS-free: short wheel gestures (a happy wiggle, a startled recoil) as the base's twist at any
time. omni_face asks for one (the console's `express` verb, expression.REACTIONS); missions.py plays it from
drive_output only while the robot rests in MANUAL with nobody at the stick (2 s), nothing else driving and no fault;
any stick input, mission or stop ends it at once.

Every gesture ends where it began, facing the same way: each axis moves in whole periods of a sine (or an out-and-back
of equal areas), so nothing accumulates. The size (0-1) scales the amplitude: the console's emotion.base level / 10
times the moment's intensity; the largest (size 1) strays at most MAX_REACH and turns at most MAX_TURN_DEG. Speeds
and accelerations stay inside the firmware's adult profile at size 1 (the kid profile's caps soften them further).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

MAX_REACH = 0.06                  # m: the farthest a gesture strays (size 1)
MAX_TURN_DEG = 14.0               # deg: the most it turns away (size 1)
GAP = 6.0                         # s between two base gestures


@dataclass(frozen=True)
class Gesture:
    name: str
    seconds: float
    axis: str                     # vx | vy | wz
    shape: str                    # sine (whole periods) | out_back (out in t_out, back in the rest) | hold (out, hold, back)
    amp: float                    # the peak excursion at size 1: m, or deg for wz
    periods: int = 1
    t_out: float = 0.0            # out_back / hold: the time going out (and for hold, the hold too)


GESTURES = {
    # happy: it shakes its wheels (a quick yaw wiggle)
    "wiggle": Gesture("wiggle", 2.4, "wz", "sine", MAX_TURN_DEG, periods=3),     # 1.9 rad/s, 15 rad/s² at size 1
    # excited: a sideways shimmy
    "shimmy": Gesture("shimmy", 1.6, "vy", "sine", 0.03, periods=4),
    # proud: a little forward bounce, twice
    "bounce": Gesture("bounce", 1.2, "vx", "sine", 0.025, periods=2),
    # surprised: a hop toward it and back
    "hop": Gesture("hop", 0.6, "vx", "sine", 0.03, periods=1),
    # startled: back quickly, then forward slowly to where it was
    "recoil": Gesture("recoil", 1.2, "vx", "out_back", -MAX_REACH, t_out=0.3),
    # curious: lean in, look, come back
    "lean": Gesture("lean", 2.4, "vx", "hold", 0.04, t_out=0.8),
    # frustrated: a sharp yaw jerk, once
    "huff": Gesture("huff", 0.6, "wz", "sine", 7.0, periods=1),
    # worried: shrink back a little, hold, return
    "shrink": Gesture("shrink", 2.4, "vx", "hold", -0.035, t_out=0.8),
    # sad: sag back slowly and return
    "sag": Gesture("sag", 2.6, "vx", "hold", -0.02, t_out=1.0),
    # relieved: a tiny forward settle, like a sigh
    "settle": Gesture("settle", 1.4, "vx", "out_back", 0.012, t_out=0.7),
}


def _rate(g: Gesture, t: float) -> float:
    """The excursion's rate (m/s or deg/s) at size 1, t in [0, seconds]."""
    T = g.seconds
    if t < 0.0 or t > T:
        return 0.0
    if g.shape == "sine":                       # x(t) = A sin(2 pi k t / T): whole periods, back to 0
        w = 2 * math.pi * g.periods / T
        return g.amp * w * math.cos(w * t)
    if g.shape == "out_back":                   # x rises to A over t_out (half a cosine), falls back over the rest
        a, b = g.t_out, T - g.t_out
        if t < a:
            return g.amp * math.pi / (2 * a) * math.sin(math.pi * t / a)
        return -g.amp * math.pi / (2 * b) * math.sin(math.pi * (t - a) / b)
    if g.shape == "hold":                       # out over t_out, hold, back over t_out
        a = g.t_out
        if t < a:
            return g.amp * math.pi / (2 * a) * math.sin(math.pi * t / a)
        if t > T - a:
            return -g.amp * math.pi / (2 * a) * math.sin(math.pi * (t - (T - a)) / a)
        return 0.0
    return 0.0


class Playing:
    """A gesture being played: twist(now) -> (vx, vy, wz) in m/s, rad/s (body frame), None once it's over."""

    def __init__(self, name: str, size: float, t0: float):
        if name not in GESTURES:
            raise ValueError(f"gesture: one of {', '.join(GESTURES)}")
        self.g = GESTURES[name]
        self.size = max(0.0, min(1.0, float(size)))
        self.t0 = t0

    @property
    def name(self) -> str:
        return self.g.name

    def done(self, now: float) -> bool:
        return now - self.t0 > self.g.seconds

    def twist(self, now: float):
        t = now - self.t0
        if t > self.g.seconds:
            return None
        r = _rate(self.g, t) * self.size
        if self.g.axis == "wz":
            return (0.0, 0.0, math.radians(r))
        return (r, 0.0, 0.0) if self.g.axis == "vx" else (0.0, r, 0.0)


def size_for(level: float, intensity: float) -> float:
    """A gesture's size from the console's base level (0-10) and the moment's intensity (0-1)."""
    return max(0.0, min(1.0, level / 10.0)) * (0.6 + 0.4 * max(0.0, min(1.0, intensity)))
