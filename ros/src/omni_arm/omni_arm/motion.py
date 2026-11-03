"""Joint-space motions for the arm (design.md 11): a list of waypoints played as smooth segments, each timed by its
slowest joint at a fraction of the STS3215's speed, sampled every cycle into goals for the servos. Straight lines of
the tool (down to a grasp, up from it) are waypoints from the IK every few mm.

Every segment is checked against the robot's keep-out volumes (kinematics.clear) before it plays; plan() goes through
a "high" waypoint when the straight segment isn't clear. That is all the planning this arm needs: its work happens in
front of the robot, on the floor or over the bin, and the robot's outline doesn't move. No unchecked motion plays:
plan_checked() refuses (with why), except one step out of a pose that already touches something; check_trajectory()
refuses an external trajectory whole rather than clipping it.
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


class Linear:
    """Straight lines between waypoints at given times (t[0] is the start), held at the last: a FollowJointTrajectory
    goal's points. omni_cpp's arm_driver plays the same."""

    def __init__(self, times, waypoints):
        self.t = [float(x) for x in times]
        self.q = [np.asarray(w, float) for w in waypoints]

    @property
    def t_end(self) -> float:
        return self.t[-1]

    def done(self, t: float) -> bool:
        return t >= self.t[-1]

    def sample(self, t: float) -> np.ndarray:
        if t >= self.t[-1]:
            return self.q[-1].copy()
        k = max(i for i in range(len(self.t)) if self.t[i] <= t) if t >= self.t[0] else 0
        a, b = (self.t[k], self.q[k]), (self.t[min(k + 1, len(self.t) - 1)], self.q[min(k + 1, len(self.t) - 1)])
        f = max(0.0, (t - a[0]) / max(1e-6, b[0] - a[0]))
        return a[1] + f * (b[1] - a[1])


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


def plan_checked(kin: ArmKinematics, q0, q1, via=None, **clear_kw):
    """(path, why): plan()'s, or None and why there's no checked way. The one exception is a start that touches
    something (dropped limp, pushed), as jog's: its first step goes out to a via that is clear, the rest is checked."""
    path = plan(kin, q0, q1, via, **clear_kw)
    if path is not None:
        return path, ""
    ok, why = kin.clear(q0, **clear_kw)
    if ok:
        return None, "no clear path from here"
    q0 = np.asarray(q0, float)
    for v in ([] if via is None else ([via] if np.ndim(via) == 1 else list(via))):
        v = np.asarray(v, float).copy()
        v[5] = q0[5]
        if kin.clear(v, **clear_kw)[0]:
            rest = plan(kin, v, q1, via, **clear_kw)
            if rest is not None:
                return [q0] + rest, f"recovering: it touches the {why}"
    return None, f"it touches the {why} and no clear way out"


def check_trajectory(kin: ArmKinematics, q0, joint_names, points, max_speed: float | None = None):
    """A FollowJointTrajectory goal checked before it plays, from q0 (where the arm is commanded): (times, waypoints)
    starting (0, q0), or (None, why). points: (t from start, positions in joint_names' order). Nothing is clipped: a
    goal outside a range, faster than the servos (max_speed rad/s, default their no-load speed) or into the robot's
    keep-outs is refused whole."""
    names = list(joint_names)
    if not names or len(set(names)) != len(names) or not set(names) <= set(kin.cfg.joints):
        return None, f"joint names {names}"
    idx = [kin.cfg.joints.index(n) for n in names]
    vmax = kin.cfg.servos["speed"] if max_speed is None else max_speed
    q0 = np.asarray(q0, float)
    times, wps = [0.0], [q0.copy()]
    for k, (t, pos) in enumerate(points):
        if len(pos) != len(names) or not all(math.isfinite(float(x)) for x in (t, *pos)):
            return None, f"point {k}: {len(pos)} positions for {len(names)} joints, or not finite"
        q = wps[-1].copy()
        q[idx] = np.asarray(pos, float)
        if np.any(q < kin.lo - 1e-6) or np.any(q > kin.hi + 1e-6):
            return None, f"point {k}: outside a joint's range"
        dt, dq = float(t) - times[-1], float(np.max(np.abs(q - wps[-1])))
        if dt <= 0.0:
            if dt == 0.0 and dq < 1e-3:                    # a repeated point (MoveIt's start state): nothing to play
                continue
            return None, f"point {k}: time {float(t):.3f} s doesn't follow {times[-1]:.3f} s"
        if dq / dt > vmax:
            return None, f"point {k}: {dq / dt:.2f} rad/s, over {vmax:.2f}"
        times.append(float(t))
        wps.append(q)
    kw = dict(allow_bin=not kin.clear(q0)[0] and kin.clear(q0, allow_bin=True)[0])     # from the bin: as jog
    for k, (a, b) in enumerate(zip(wps[:-1], wps[1:])):
        ok, why = kin.path_clear(a, b, **kw)
        if not ok:
            return None, f"segment {k}: it would hit the {why}"
    return times, wps


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
