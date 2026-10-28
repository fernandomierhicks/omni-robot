"""Joint-space motions for the arm (design.md 11): a list of waypoints played as smooth segments, each timed by its
slowest joint at a fraction of the STS3215's speed, sampled every cycle into goals for the servos. Straight lines of
the tool (down to a grasp, up from it) are waypoints from the IK every few mm.

Every segment is checked against the robot's keep-out volumes (kinematics.clear) before it plays; plan() goes through
a "high" waypoint when the straight segment isn't clear. That is all the planning this arm needs: its work happens in
front of the robot, on the floor or over the bin, and the robot's outline doesn't move.
"""

from __future__ import annotations

import math

import numpy as np

from omni_arm.kinematics import ArmKinematics

SPEED_FRAC = 0.5               # of the servos' no-load speed (4.7 rad/s): the motions' joint speed cap
MIN_SEGMENT = 0.25             # s
JOG_MAX = math.radians(30.0)   # the largest jog of one joint at a time (the console's "by hand")


def smooth(s: float) -> float:
    """Quintic 0..1 with zero speed and acceleration at both ends."""
    s = min(1.0, max(0.0, s))
    return s * s * s * (10 + s * (-15 + 6 * s))


class Motion:
    """Waypoints q_0 (where the arm is) .. q_n, each segment timed so no joint passes speed * 1.875 (the quintic's
    peak over the mean)."""

    def __init__(self, waypoints, speed: np.ndarray, t0: float, min_segment: float = MIN_SEGMENT):
        self.q = [np.asarray(w, float) for w in waypoints]
        self.t = [t0]
        for a, b in zip(self.q[:-1], self.q[1:]):
            dur = max(min_segment, float(np.max(np.abs(b - a) / speed)) * 1.875)
            self.t.append(self.t[-1] + dur)

    @property
    def t_end(self) -> float:
        return self.t[-1]

    def sample(self, t: float) -> np.ndarray:
        if t >= self.t[-1]:
            return self.q[-1].copy()
        k = max(0, int(np.searchsorted(self.t, t, side="right")) - 1)
        s = smooth((t - self.t[k]) / (self.t[k + 1] - self.t[k]))
        return self.q[k] + s * (self.q[k + 1] - self.q[k])

    def done(self, t: float) -> bool:
        return t >= self.t[-1]


def speeds(kin: ArmKinematics, frac: float = SPEED_FRAC) -> np.ndarray:
    return np.full(6, kin.cfg.servos["speed"] * frac)


def line(kin: ArmKinematics, q0, p1, approach, yaw=None, step: float = 0.005):
    """Waypoints moving the tool in a straight line from where q0 has it to p1 (the fingers along approach), or None
    if some point has no IK solution. The gripper keeps q0's."""
    p0 = kin.tool(q0)[:3, 3]
    n = max(1, int(math.ceil(np.linalg.norm(np.asarray(p1) - p0) / step)))
    out, q = [], np.asarray(q0, float).copy()
    for s in np.linspace(0, 1, n + 1)[1:]:
        qq = kin.ik(p0 + s * (np.asarray(p1) - p0), approach, yaw, seed=q)
        if qq is None:
            return None
        qq[5] = q0[5]
        out.append(qq)
        q = qq
    return out


def plan(kin: ArmKinematics, q0, q1, via=None, **clear_kw):
    """[q0, (waypoints,) q1] with every segment clear, or None. Tries the straight segment, then through one of via
    (a pose, or a list of them: "ready", "high"), then through two of them in either order."""
    from itertools import permutations
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    vias = [] if via is None else ([via] if np.ndim(via) == 1 else list(via))
    vias = [np.asarray(v, float).copy() for v in vias]
    for v in vias:
        v[5] = q0[5]
    for path in [[]] + [[v] for v in vias] + [list(p) for p in permutations(vias, 2)]:
        pts = [q0] + path + [q1]
        if all(kin.path_clear(a, b, **clear_kw)[0] for a, b in zip(pts[:-1], pts[1:])):
            return pts
    return None


def jog(kin: ArmKinematics, q_start, joint: int, delta: float, base=None):
    """One joint moved by delta (at most JOG_MAX, kept in its range) from base (default q_start, where the arm is
    sent now; a jog still playing passes its end so clicks add up): (target, "") with the move q_start -> target
    clear, else (None, why). An arm that starts in the bin's box (at the drop) may move in it; one that starts
    touching something (dropped limp, pushed) may step to any clear pose, to get out."""
    q0 = np.asarray(q_start, float)
    q1 = np.asarray(q0 if base is None else base, float).copy()
    d = max(-JOG_MAX, min(JOG_MAX, float(delta)))
    q1[joint] = float(np.clip(q1[joint] + d, kin.lo[joint], kin.hi[joint]))
    if np.allclose(q1, q0, atol=1e-6):
        return None, "at the end of its range"
    kw = dict(allow_bin=not kin.clear(q0)[0] and kin.clear(q0, allow_bin=True)[0])
    ok, why = kin.path_clear(q0, q1, **kw) if kin.clear(q0, **kw)[0] else kin.clear(q1, **kw)
    return (q1, "") if ok else (None, f"it would hit the {why}")
