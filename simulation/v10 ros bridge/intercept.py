"""Moving-catch intercept planner.

The ball's crossing of the rim plane fixes where (p_c) and when (t_c) the funnel
must meet it. The stop controller brakes to rest at p_c; when there is too
little time, a moving catch arrives at p_c *on time* but still moving, at the
slowest speed that makes it possible: accelerate fully, then brake fully, and
pass through p_c at t_c at speed v_f. Executed by the same braking-curve goto
with an end speed (omni_control.goto_step(v_end=v_f)), and replanned at 20 Hz:
early, the next replan lowers v_f (to 0: a plain stop); late, it raises it.
So stop-and-catch is just the v_f = 0 case.

Clear-path gate (moving catches only): the path, the short coast after the catch
and the gentle braking with the ball aboard must keep the robot's outline a
margin away from the arena walls and from people. If v_f fails it, v_f is capped
at the fastest speed that passes: the base may arrive late, but it stays clear.

Two earlier versions lost to the plain stop controller and are documented in the
README: chasing a virtual target that slides at the meeting velocity (when it
started behind the robot, the robot waited for it and then couldn't catch it),
and constant-acceleration guidance (it arrives at twice the average speed, past
the speed cap and with too little room to stop).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class SafetyZone:
    half_x: float = 2.5                    # arena half sizes (m), centered on the origin
    half_y: float = 2.0
    people: list = field(default_factory=list)   # [(x, y, radius)]
    robot_radius: float = 0.21             # bumper outline
    margin: float = 0.20                   # required clearance for a moving catch

    def clear_point(self, q) -> bool:
        lim = self.robot_radius + self.margin
        if abs(q[0]) > self.half_x - lim or abs(q[1]) > self.half_y - lim:
            return False
        return all(math.hypot(q[0] - x, q[1] - y) >= r + lim for x, y, r in self.people)


def stop_point(p_c, v_f, coast: float, a_ball: float):
    sp = float(np.linalg.norm(v_f))
    if sp < 1e-9:
        return np.asarray(p_c, float)
    return np.asarray(p_c, float) + np.asarray(v_f, float) * (coast + sp / (2 * a_ball))


def path_clear(zone: SafetyZone, p0, p_c, v_f, coast: float, a_ball: float, step: float = 0.05) -> bool:
    """The (straight-line approximation of the) path to the meeting point and the stop after it."""
    p0, p_c = np.asarray(p0, float), np.asarray(p_c, float)
    q_end = stop_point(p_c, v_f, coast, a_ball)
    for a, b in ((p0, p_c), (p_c, q_end)):
        n = max(2, int(math.ceil(np.linalg.norm(b - a) / step)) + 1)
        for k in range(n):
            if not zone.clear_point(a + (b - a) * k / (n - 1)):
                return False
    return True


def min_time(u0: float, d: float, v_f: float, a: float, v_max: float) -> float:
    """Shortest time to cover d along a line, starting at speed u0 and ending at v_f (|accel| <= a)."""
    if d <= 0:
        return 0.0
    vp = math.sqrt(a * d + (u0 * u0 + v_f * v_f) / 2)
    if vp < max(u0, v_f):                 # can't shed (or gain) the speed in the distance: one-sided ramp
        return 2 * d / (u0 + v_f) if u0 + v_f > 1e-9 else float("inf")
    if vp <= v_max:
        return (vp - u0) / a + (vp - v_f) / a
    d_ramp = (2 * v_max ** 2 - u0 ** 2 - v_f ** 2) / (2 * a)
    return (v_max - u0) / a + (v_max - v_f) / a + (d - d_ramp) / v_max


def min_arrival_speed(u0: float, d: float, T: float, a: float, v_max: float) -> float:
    """Slowest end speed that still covers d within T (bisection); v_max if even that is too slow."""
    if min_time(u0, d, 0.0, a, v_max) <= T:
        return 0.0
    if min_time(u0, d, v_max, a, v_max) > T:
        return v_max
    lo, hi = 0.0, v_max
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if min_time(u0, d, mid, a, v_max) <= T else (mid, hi)
    return hi


@dataclass
class InterceptChoice:
    v_f: float             # end speed at the meeting point (0 = stop-and-catch)
    direction: np.ndarray  # unit direction of travel at the meeting point
    feasible: bool         # reachable on time within the limits
    gated: bool            # the clear-path gate capped v_f


def choose_intercept(p0, v0, p_c, T: float, a: float, v_max: float, zone: SafetyZone, coast: float,
                     a_ball: float, moving: bool = True) -> InterceptChoice:
    p0, p_c = np.asarray(p0, float), np.asarray(p_c, float)
    dvec = p_c - p0
    d = float(np.linalg.norm(dvec))
    if d < 0.01 or T <= 0.02:
        return InterceptChoice(0.0, np.array([1.0, 0.0]), True, False)
    dh = dvec / d
    u0 = max(0.0, float(np.asarray(v0, float) @ dh))
    v_f = min_arrival_speed(u0, d, T, a, v_max)
    feasible = min_time(u0, d, v_f, a, v_max) <= T
    if v_f <= 0.0:
        return InterceptChoice(0.0, dh, feasible, False)
    if not moving:
        return InterceptChoice(0.0, dh, False, False)
    if path_clear(zone, p0, p_c, v_f * dh, coast, a_ball):
        return InterceptChoice(v_f, dh, feasible, False)
    lo, hi = 0.0, v_f                                 # fastest end speed the gate allows
    for _ in range(20):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if path_clear(zone, p0, p_c, mid * dh, coast, a_ball) else (lo, mid)
    return InterceptChoice(lo, dh, False, True)
