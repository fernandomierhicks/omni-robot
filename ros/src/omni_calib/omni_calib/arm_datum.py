"""Zeroing the arm against touch blocks on the robot (design.md 4.5; a proposal, Nov 20, tried in simulation first):
the fixed fingertip touches each block's faces, slowly and softly, from several poses; where the joints stopped says
where the fingertip was, and the blocks' drawn places turn that into the joints' true zeros and sag. Removable: with no
blocks listed nothing here runs.

  Datum                 a block: centre and half sizes in base_link (the drawing's)
  plan(kin, datums)     touches: a face, the fingertip's approach, a start 15 mm off and an end 6 mm past, by IK
  TouchOff              the procedure (ArmLoadCal's actions: move, line with "soft"); a touch is seen when the joints
                        fall behind the goal (the encoder stops while the goal goes on)
  fit(kin, model, ...)  least squares on the touches (and the ruler's heights if given): zeros of all five joints, the
                        sag of lift / elbow / wrist flex, priors as arm_load's ruler fit

The fingertip is the fixed finger's 4 mm sphere (arm_so101.FINGERTIPS), the gripper open so the jaw stays clear.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

from omni_calib.arm_load import JointLoad, LoadModel, N5, _tip_height, true_angles

SPHERE = np.array([-0.0135, 0.0, -0.100])      # the fixed fingertip's sphere centre, the gripper's frame
SPHERE_R = 0.004
OPEN = 0.6                                       # rad: the gripper during a touch (the jaw's tip away)
STANDOFF, PAST = 0.015, 0.006                    # m: the start off the face, the end past it
SPEED = 0.003                                    # m/s along the approach
DEV = 0.004                                      # rad: the joints this far behind the goal (over the line's own lag)


@dataclass
class Datum:
    name: str
    centre: np.ndarray                           # base_link
    half: np.ndarray


def _sphere(kin, q) -> np.ndarray:
    T = kin.joint_frames(q)[1]["arm_gripper"]
    return T[:3, :3] @ SPHERE + T[:3, 3]


def plan(kin, datums, seed: int = 0) -> list:
    """Touches on each block within what a five-joint arm can do (its fingers stay in the vertical plane through the
    pan axis), each a slow straight move of the fingertip into a face along the face's normal:
      top     fingers straight down at three wrist rolls, and tilted toward / away from the arm (heights: lift, elbow,
              wrist flex, and roll through the sphere sitting off the finger's axis)
      sides   fingers down, the tip 5 mm under the top edge moved sideways into each vertical face (the faces across
              the arm's radius see pan; along it, the reach)
    Each kept when the IK reaches both ends and the path is clear of the robot (the blocks aren't in its keep-out)."""
    rng = np.random.default_rng(seed + 33)
    out = []
    o = kin.joint_frames(np.zeros(6))[0][0][0]          # the pan axis's point
    down = np.array([0.0, 0.0, -1.0])
    for d in datums:
        c, h = np.asarray(d.centre, float), np.asarray(d.half, float)
        r = c[:2] - o[:2]
        r = np.r_[r / np.linalg.norm(r), 0.0]          # radial, away from the arm
        top = c + [0, 0, h[2]]
        cases = [(np.array([0, 0, 1.0]), top, down, roll) for roll in (-0.8, 0.0, 0.8)]
        cases += [(np.array([0, 0, 1.0]), top, np.r_[math.sin(th) * r[:2], -math.cos(th)], 0.0)
                  for th in (-0.35, 0.35, 0.6)]
        for axis in (0, 1):
            for sgn in (1.0, -1.0):
                n = np.zeros(3)
                n[axis] = sgn
                pt = c + n * h
                pt[2] = c[2] + h[2] - 0.005
                cases += [(n, pt, down, roll) for roll in (0.0, 0.7)]
        for n, pt, f, roll in cases:
            ends = []
            for off in (STANDOFF, -PAST):
                q = _ik_sphere(kin, pt + n * (SPHERE_R + off), f / np.linalg.norm(f), None, roll)
                if q is None:
                    break
                ends.append(q)
            if len(ends) == 2 and kin.clear(ends[0])[0] and kin.path_clear(ends[0], ends[1])[0]:
                out.append(dict(datum=d.name, normal=n, point=pt, q0=ends[0], q1=ends[1]))
    rng.shuffle(out)
    return out


def _ik_sphere(kin, cen, a, yaw, roll=0.0):
    """Joint angles that put the fingertip's sphere centre at cen, the fingers along a, the wrist's roll at roll (the
    sphere sits ~6 mm off the tool point, partly sideways: corrected by FK twice)."""
    tool = cen - a * 0.010
    q = np.array([0, 0, 0, 0, roll, 0.0])
    for _ in range(3):
        q = kin.ik(tool, a, yaw=yaw, tol_pos=0.003, seed=q)
        if q is None:
            return None
        tool = tool + (cen - _sphere(kin, q))
    q[5] = OPEN
    return q if np.linalg.norm(_sphere(kin, q) - cen) < 0.001 else None


class TouchOff:
    """step(t, obs) -> actions, as omni_calib.arm_load.ArmLoadCal: move to each touch's start, settle, a slow straight
    line toward the face ("soft": the caller lowers the torque limit), the touch when the joints fall behind, back.
    obs needs q_enc, q, goal (6 each)."""

    def __init__(self, touches, log=None):
        self.touches = touches
        self.k = -1
        self.phase = "start"
        self.t0 = 0.0
        self.base = []
        self.prev = None
        self.hist: list = []
        self.hits: list = []                     # [(touch, q_enc at the touch)]
        self.missed: list = []
        self.done = False
        self.log = log or (lambda s: None)

    def _next(self, t):
        self.k += 1
        self.phase, self.t0, self.base, self.prev, self.hist = "start", t, [], None, []

    def step(self, t, obs) -> list:
        if self.done:
            return []
        if self.k < 0:
            self._next(t)
        if self.k >= len(self.touches):
            self.done = True
            return [dict(op="done")]
        tc = self.touches[self.k]
        if self.phase == "start":
            self.phase = "to_start"
            return [dict(op="move", q=tc["q0"], frac=0.3)]
        if self.phase == "to_start" and not obs["moving"]:
            self.phase, self.t0 = "settle", t
        if self.phase == "settle" and t - self.t0 > 0.6:
            dist = STANDOFF + PAST
            self.phase, self.t0 = "line", t
            self.dur = dist / SPEED
            return [dict(op="line", q0=tc["q0"], q1=tc["q1"], duration=self.dur, soft=True)]
        if self.phase == "line":
            dev = float(np.max(np.abs(np.asarray(obs["goal"][:N5]) - np.asarray(obs["q"][:N5]))))
            s = (t - self.t0) / self.dur
            if s < 0.25:                             # before the face: the line's own lag
                self.base.append(dev)
            elif self.base and dev > max(self.base) + DEV and self.hist:
                # the touch: the last sample before the joints began to fall behind (later ones are pressed)
                lim = max(self.base) + DEV / 4
                k = len(self.hist) - 1
                while k > 0 and self.hist[k][0] > lim:
                    k -= 1
                self.hits.append((tc, np.asarray(self.hist[k][1]["q_enc"], float)))
                self.log(f"touch {self.k + 1}/{len(self.touches)} {tc['datum']}: at {s:.2f} of the line")
                self.phase = "back"
                return [dict(op="hold"), dict(op="move", q=tc["q0"], frac=0.2)]
            self.prev = obs
            self.hist.append((dev, obs))
            if s > 1.05:
                self.missed.append(tc)
                self.log(f"touch {self.k + 1}: no contact")
                self.phase = "back"
                return [dict(op="move", q=tc["q0"], frac=0.2)]
        if self.phase == "back" and not obs["moving"]:
            self._next(t)
        return []


def _with(m: LoadModel, x, joints=(0, 1, 2, 3, 4), sag=(1, 2, 3)) -> LoadModel:
    mm = LoadModel([JointLoad(**asdict(j)) for j in m.joints], m.links, m.use_current, m.source, x[-1])
    for k, j in enumerate(joints):
        mm.joints[j].zero = x[k]
    for k, j in enumerate(sag):
        mm.joints[j].compliance = x[len(joints) + k]
    return mm


def fit(kin, model: LoadModel, hits, datums, ruler=None, weights=None, g=(0.0, 0.0, -9.81), free=None) -> LoadModel:
    """The zeros (pan, lift, elbow, wrist flex, roll), the sag (lift, elbow, wrist flex) and the floor's height (only with
    the ruler's records) from the touches: each says the fingertip's sphere was its radius off the face. free: the
    indices of x (0-4 zeros, 5-7 sag, 8 the floor) the fit may move; the rest stay at the model's (default all)."""
    from scipy.optimize import least_squares
    dmap = {d.name: d for d in datums}
    ruler = [r for r in (ruler or []) if r.kind == "ruler"]

    def res(x):
        mm = _with(model, x)
        out = []
        for tc, q_enc in hits:
            q = true_angles(mm, kin, q_enc, g)
            out.append((tc["normal"] @ (_sphere(kin, q) - tc["point"]) - SPHERE_R) * 1000 / 0.3)   # mm / 0.3
        for r in ruler:
            out.append((_tip_height(kin, mm, r.q, r.g, weights or {}, r.weight) * 1000 - r.value) / 1.0)
        out += list(x[:5] / math.radians(3.0)) + list(x[5:8] / math.radians(2.0)) + [x[8] / 0.006]
        return np.array(out)

    x0 = np.r_[[j.zero for j in model.joints], [model.joints[j].compliance for j in (1, 2, 3)], model.mount_dz]
    # sag can't be negative; near the arm's base gravity hardly loads the joints, so the touches alone barely see it
    lo = np.r_[[-0.2] * 5, [0.0] * 3, -0.02]
    hi = np.r_[[0.2] * 5, [0.06] * 3, 0.02]
    x0 = np.clip(x0, lo + 1e-9, hi - 1e-9)
    idx = np.arange(9) if free is None else np.asarray(free)

    def res_free(xf):
        x = x0.copy()
        x[idx] = xf
        return res(x)

    xf = least_squares(res_free, x0[idx], bounds=(lo[idx], hi[idx]), loss="soft_l1", f_scale=2.0).x
    x = x0.copy()
    x[idx] = xf
    if not ruler:
        x[8] = model.mount_dz                        # the floor isn't seen without the ruler
    mm = _with(model, x)
    mm.source = model.source + " + datums"
    del dmap
    return mm
