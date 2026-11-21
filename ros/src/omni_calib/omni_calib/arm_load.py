"""Item 10: the arm's load (design.md 4.5): what each servo's telemetry says about the torque on its joint, the links'
masses, the joints' true zeros and their sag past the encoders, fitted from a procedure the robot runs on itself with a
person's help (two weights, a kitchen scale, calipers, a ruler; a luggage scale for a check). The arm driver
(omni_cpp arm_load.hpp) uses the result to tell an outside force from gravity and friction (the strain detector and its
back-off reflex) and to put the joints where it means to.

  LoadModel, estimate()   omni_cpp's load model in Python (the same json; omni_cpp/test checks the two agree)
  plan()                  the steps: holds approached from both sides, constant-speed sweeps, holds with each weight
                          hanging from the fixed finger, fingertip heights for the ruler, the luggage-scale check
  ArmLoadCal              the procedure as the robot runs it (arm_brain's "cal_load", the console's Calibrate tab):
                          step(t, obs) -> actions (moves, prompts), answer(value) for a person's steps, then the fit
  fit()                   least squares with bounds and priors on the official masses: each joint's servo model
                          (duty x supply, back-EMF, rotor inertia, offset, friction moving and still) and the links'
                          masses and centres; the current model where the servos report it; then the zeros, the sag
                          and the mount's height from the ruler; held-out holds and leave-one-out heights check it

The model, per joint (the five that place the gripper), with duty d, supply V, speed w, the goal's acceleration a:
  motor tm = A d V / (1 + alpha (T - 25)) - E w - J a - o        (current: Ki (i - io) - J a - o)
  moving in direction s: tm - s (F + mu |g|) + g + outside = 0
  still after moving in direction s: tm - s (Fs + mus |g|) + g + outside = 0 on average; the driver allows the whole
  band +-(Fs + mus |g|) at rest (the gear's friction holds anywhere in it)
  true angle = encoder + zero + compliance x g (the sag the encoder doesn't see); g = gravity's torque on the joint.

No servo has been measured yet (Nov 20): the procedure, the limits and the people's errors are est; its numbers come
from simulation/v11 sil's detailed arm (arm_model "detailed", studies/arm_load_cal.py).
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field

import numpy as np

N5 = 5
V_EPS = 0.02                        # rad/s: a goal slower is still (omni_cpp LOAD_V_EPS)
W_EPS = 0.30                        # rad/s: a measured speed past this moves the joint (LOAD_W_EPS)
G0 = (0.0, 0.0, -9.81)
FIELDS = ("A", "E", "J", "o", "F", "mu", "Fs", "mus", "Ki", "io", "alpha", "zero", "compliance", "thr", "thr_move")
# the fixed finger's very end in the gripper's frame (arm_so101.FINGERTIPS: its 4 mm sphere's centre, z -0.100)
TIP = np.array([-0.0135, 0.0, -0.104])
TIP_R = 0.004


# -- the model (omni_cpp/arm_load.hpp) --------------------------------------------------------------------------------

@dataclass
class JointLoad:
    A: float = 0.0
    E: float = 0.0
    J: float = 0.0
    o: float = 0.0
    F: float = 0.1
    mu: float = 0.5
    Fs: float = 0.1
    mus: float = 0.5
    Ki: float = 0.0
    io: float = 0.0
    alpha: float = 0.0039
    zero: float = 0.0
    compliance: float = 0.0
    thr: float = 0.35
    thr_move: float = 0.7


@dataclass
class LoadModel:
    joints: list
    links: list                      # [{body, mass, com}]
    use_current: bool = False
    source: str = "nominal"
    mount_dz: float = 0.0            # m: the ruler's floor against base_link's (reported, not used by the driver)

    @staticmethod
    def nominal(cfg) -> "LoadModel":
        s = cfg.servos
        stall, speed = float(s.get("stall_torque", 2.942)), float(s.get("speed", 4.7))
        js = [JointLoad(A=stall / 12.0, E=stall / speed, F=0.08, mu=0.6, Fs=0.08, mus=0.6, thr=0.45, thr_move=0.9)
              for _ in range(N5)]
        return LoadModel(js, [dict(body=l["body"], mass=l["mass"], com=np.asarray(l["com"], float))
                              for l in cfg.inertials])

    @staticmethod
    def load(path, cfg) -> "LoadModel":
        m = LoadModel.nominal(cfg)
        if not path:
            return m
        raw = json.loads(open(path).read()) if not isinstance(path, dict) else path
        m.source = raw.get("source", str(path))
        m.use_current = bool(raw.get("use_current", False))
        for i, name in enumerate(cfg.joints[:N5]):
            for k, v in raw.get("joints", {}).get(name, {}).items():
                if k in FIELDS:
                    setattr(m.joints[i], k, float(v))
        if "links" in raw:
            m.links = [dict(body=l["body"], mass=float(l["mass"]), com=np.asarray(l["com"], float)) for l in raw["links"]]
        m.mount_dz = float(raw.get("mount_dz", 0.0))
        return m

    def to_json(self, cfg, **extra) -> dict:
        return dict(model="omni_arm_load/1", source=self.source, use_current=self.use_current, mount_dz=self.mount_dz,
                    joints={n: asdict(j) for n, j in zip(cfg.joints[:N5], self.joints)},
                    links=[dict(body=l["body"], mass=float(l["mass"]), com=[float(v) for v in l["com"]])
                           for l in self.links], **extra)


def estimate(m: LoadModel, kin, q, w, duty, volts, temp, current, goal_vel, goal_acc, g=G0) -> dict:
    """omni_cpp's estimate(): {grav, motor, ext, band, moving} (6 each; the jaw's 0)."""
    grav = kin.gravity(q, g, m.links)
    inertia = kin.inertia_torque(q, goal_acc, m.links)
    out = dict(grav=grav, motor=np.zeros(6), ext=np.zeros(6), band=np.zeros(6), moving=np.zeros(6, bool),
               inertia=inertia)
    for i in range(N5):
        L = m.joints[i]
        v = goal_vel[i] if abs(goal_vel[i]) > V_EPS else w[i] if abs(w[i]) > W_EPS else 0.0
        if m.use_current and L.Ki != 0:
            tm = L.Ki * (current[i] - L.io)
        else:
            tm = L.A * duty[i] * volts[i] / (1.0 + L.alpha * (temp[i] - 25.0)) - L.E * v
        tm -= L.J * goal_acc[i] + L.o
        c = -grav[i] - tm + inertia[i]
        out["motor"][i] = tm
        if v != 0:
            out["moving"][i] = True
            out["ext"][i] = c + math.copysign(L.F + L.mu * abs(grav[i]), v)
        else:
            fb = L.Fs + L.mus * abs(grav[i])
            out["band"][i] = fb
            out["ext"][i] = math.copysign(max(0.0, abs(c) - fb), c) if c else 0.0
    return out


def true_angles(m: LoadModel, kin, q_enc, g=G0) -> np.ndarray:
    q = np.asarray(q_enc, float).copy()
    q[:N5] += [j.zero for j in m.joints]
    c = np.array([j.compliance for j in m.joints])
    if not c.any():
        return q
    base = q.copy()
    for _ in range(2):
        q[:N5] = base[:N5] + c * kin.gravity(q, g, m.links)[:N5]
    return q


def encoder_goals(m: LoadModel, kin, q_true, g=G0) -> np.ndarray:
    q = np.asarray(q_true, float).copy()
    gr = kin.gravity(q, g, m.links)
    for i, j in enumerate(m.joints):
        q[i] = q[i] - j.zero - j.compliance * gr[i]
    return q


def tool_force(kin, q, ext, p_gripper=None, damping: float = 1e-4) -> np.ndarray:
    J = kin.jacobian(q, p_gripper)
    return np.linalg.solve(J @ J.T + damping * np.eye(3), J @ np.asarray(ext, float)[:N5])


# -- the procedure ------------------------------------------------------------------------------------------------------

DELTA = math.radians(4.0)            # each hold approached from this far on every joint, from one side then the other
SETTLE, SETTLE_WEIGHT, RECORD = 1.0, 1.6, 0.6     # s: the weight's string swings a while (est)
SWEEP_SPEEDS = (0.25, 0.6)           # rad/s
CLOSED = -0.17                       # the gripper during the calibration


def hang_point(dist_mm: float) -> np.ndarray:
    """The string's point in the gripper's frame: dist_mm (the calipers) up the fixed finger from its end."""
    return TIP + np.array([0.0, 0.0, dist_mm / 1000.0])


def _weight_torques(kin, q, kg, point, g) -> np.ndarray:
    return kin.point_torque(q, kg * np.asarray(g, float), point)


def _tip_base(kin, q) -> np.ndarray:
    T = kin.joint_frames(q)[1]["arm_gripper"]
    return T[:3, :3] @ TIP + T[:3, 3]


def plan(kin, seed: int = 0, quick: bool = False) -> list:
    """The steps, in order: dicts with "op" hold | sweep | prompt | ruler | pull and their poses (checked clear of the
    robot; loads under 1.1 N m on every joint with the heavier weight, est 0.32 kg, so the servos don't heat)."""
    rng = np.random.default_rng(seed + 7100)
    lo, hi = kin.lo, kin.hi
    nom = LoadModel.nominal(kin.cfg)

    def ok_pose(q, w_kg=0.0, out_front=False, z_min=0.12):
        if not kin.clear(q)[0]:
            return False
        tip = _tip_base(kin, q)
        if tip[2] < z_min:
            return False
        gr = np.abs(kin.gravity(q, G0, nom.links)[:N5] + _weight_torques(kin, q, w_kg, hang_point(12.0), G0)[:N5])
        if gr.max() > 1.1:
            return False
        if out_front:                                    # the weight hangs clear of the robot, in front
            P = kin.joint_frames(q)[1]["arm_gripper"]
            hp = P[:3, :3] @ hang_point(12.0) + P[:3, 3]
            if hp[0] < 0.30 or hp[2] < 0.20 or abs(hp[1]) > 0.2:
                return False
        return True

    def draw(n, **kw):
        box = np.radians([[-55, 55], [-75, 35], [-70, 85], [-85, 85], [-90, 90]])
        cands = []
        for _ in range(6000):
            q = np.r_[rng.uniform(box[:, 0], box[:, 1]), CLOSED]
            q[:5] = np.clip(q[:5], lo[:5], hi[:5])
            if ok_pose(q, **kw):
                cands.append(q)
            if len(cands) >= 40 * n:
                break
        if len(cands) < n:
            raise RuntimeError(f"only {len(cands)} clear poses for {n} ({kw})")
        # spread them: farthest-point in the gravity torques' space and the joints'
        C = np.array(cands)
        feat = np.c_[np.array([kin.gravity(q, G0, nom.links)[:N5] for q in C]) * 2.0, C[:, :5]]
        pick = [int(rng.integers(len(C)))]
        d = np.linalg.norm(feat - feat[pick[0]], axis=1)
        while len(pick) < n:
            k = int(np.argmax(d))
            pick.append(k)
            d = np.minimum(d, np.linalg.norm(feat - feat[k], axis=1))
        return [C[k] for k in pick]

    n_free, n_w, n_ruler = (8, 5, 6) if quick else (24, 10, 8)
    steps = [dict(op="prompt", kind="confirm", key="start", text=(
        "Put the robot on a flat floor, 60 cm clear in front of the arm and nothing round it. The arm moves by itself "
        f"for about {4 if quick else 9} minutes and stops for you a few times: two weights to weigh and hang, heights "
        "to read with a ruler, one pull with the luggage scale. Have ready two weights of about 150 g and 300 g on a "
        "string loop, the kitchen scale, the calipers, a ruler and the luggage scale. Continue when ready."))]
    free = draw(n_free)
    held_out = set(range(0, n_free, 3))               # every third pose is kept for the check
    for k, q in enumerate(free):
        for d in (1, -1):
            steps.append(dict(op="hold", q=q, dir=d, weight=None, holdout=k in held_out))
    # constant-speed sweeps: every joint both ways at two speeds, from a pose in the middle of its free range
    mid = np.array([0.0, -0.4, 0.6, 0.3, 0.0, CLOSED])
    spans = {0: (-0.6, 0.6), 1: (-0.9, 0.2), 2: (-0.2, 1.2), 3: (-0.9, 0.9), 4: (-1.2, 1.2)}
    for j, (a, b) in spans.items():
        q0, q1 = mid.copy(), mid.copy()
        q0[j], q1[j] = a, b
        if not (kin.path_clear(q0, q1)[0] and kin.clear(q0)[0]):
            continue
        for v in SWEEP_SPEEDS[:1] if quick else SWEEP_SPEEDS:
            steps.append(dict(op="sweep", joint=j, q0=q0, q1=q1, speed=v))
            steps.append(dict(op="sweep", joint=j, q0=q1, q1=q0, speed=v))
    for key, label, kg in (("A", "the lighter weight (about 150 g)", 0.15), ("B", "the heavier weight (about 300 g)", 0.32)):
        steps += [dict(op="present"),
                  dict(op="prompt", kind="weigh", key=key, unit="g", text=f"Weigh {label} with its string on the kitchen "
                                                                       "scale and enter its mass."),
                  dict(op="prompt", kind="hang", key=key, unit="mm", text=(
                      "Loop its string over the fixed finger (the jaw that doesn't move) and let it hang. With the "
                      "calipers, measure from the very end of that finger to the middle of the string, and enter it."))]
        wposes = draw(n_w, w_kg=kg, out_front=True)
        for k, q in enumerate(wposes):
            for d in (1, -1):
                steps.append(dict(op="hold", q=q, dir=d, weight=key, holdout=k % 4 == 1))
        if key == "B":                                # the heavier weight's sag on the ruler
            steps += [dict(op="ruler", q=q, weight="B")
                      for q in _ruler_poses(kin, rng, 3 if quick else 4, (0.20, 0.30))]
        steps.append(dict(op="prompt", kind="unhang", key=key, text="Take the weight off the finger, then continue."))
    steps += [dict(op="ruler", q=q, weight=None) for q in _ruler_poses(kin, rng, n_ruler)]
    pq = _pull_pose(kin)
    if pq is not None:
        steps += [dict(op="pull", q=pq)]
    return steps


def _ruler_poses(kin, rng, n, heights=(0.015, 0.11)) -> list:
    """Fingertip poses over the floor in front of the bumper, fingers down or tilted, heights (m) up; with a weight
    hanging, high enough that it clears the floor."""
    out, tries = [], 0
    while len(out) < n and tries < 600:
        tries += 1
        x, y, h = rng.uniform(0.30, 0.42), rng.uniform(-0.12, 0.12), rng.uniform(*heights)
        tilt, az = math.radians(rng.uniform(0, 35)), rng.uniform(0, 2 * math.pi)
        a = np.array([math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az), -math.cos(tilt)])
        # the tool point is 6 mm above the fingertip's end along the fingers
        p = np.array([x, y, h + TIP_R]) - 0.006 * a
        q = kin.ik(p, a, yaw=rng.uniform(-1.0, 1.0))
        if q is None:
            continue
        q[5] = CLOSED
        if kin.clear(q, floor_margin=0.004)[0]:
            out.append(q)
    return out


def _pull_pose(kin):
    q = kin.ik(np.array([0.36, 0.0, 0.22]), np.array([0.0, 0.0, -1.0]), yaw=0.0)
    if q is None:
        return None
    q[5] = CLOSED
    return q if kin.clear(q)[0] else None


@dataclass
class Record:
    """One held pose (averaged) or one sweep sample."""
    kind: str                          # hold | sweep | ruler | pull
    q: np.ndarray                      # the encoders' angles
    duty: np.ndarray
    volts: np.ndarray
    temp: np.ndarray
    current: np.ndarray
    w: np.ndarray
    goal_vel: np.ndarray
    goal_acc: np.ndarray
    g: np.ndarray
    dir: np.ndarray                    # each joint's last motion (+1 / -1), or the sweep's direction
    weight: str | None = None
    holdout: bool = False
    value: float | None = None         # ruler: the height read (mm); pull: the scale (kg)
    n: int = 1


class ArmLoadCal:
    """The procedure: step(t, obs) every driver cycle; it returns actions for the caller to carry out:
      {"op": "move", "q": (6,), "frac": f}          plan a checked motion there (motion.plan_checked) and play it,
                                                    unguarded; then obs["moving"] is True until it has played
      {"op": "line", "q0", "q1", "duration": s}     a straight joint line at constant speed (a linear MOVE)
      {"op": "prompt", ...}                         show it; answer(value) when the person is done
      {"op": "done", "result": {...}}               finished (fit() ran; "result" None if cancelled or failed)
    obs: t, q_enc (6), duty, volts, temp, current, w (6 each), goal_vel, goal_acc (6), g (3, base_link), moving,
    ok (the servos healthy). refused(why) when a move couldn't be planned: that step is skipped."""

    def __init__(self, kin, seed: int = 0, quick: bool = False, steps: list | None = None, log=None):
        self.kin = kin
        self.steps = steps if steps is not None else plan(kin, seed, quick)
        self.k = -1
        self.phase = "idle"
        self.t_phase = 0.0
        self.buf: list = []
        self.records: list = []
        self.weights: dict = {}            # key -> {"kg", "point"}
        self.hung = None
        self.prompt = None
        self.answer_value = None
        self.result = None
        self.done = False
        self.why = ""
        self.last_q = None
        self.events: list = []
        self.log = log or (lambda s: None)

    # -- the console's view -----------------------------------------------------------------------------------------
    def status(self) -> dict:
        n = len(self.steps)
        st = self.steps[self.k] if 0 <= self.k < n else {}
        return dict(step=self.k + 1, steps=n, phase=self.phase, op=st.get("op"), prompt=self.prompt,
                    weight=self.hung, records=len(self.records), done=self.done, why=self.why,
                    result=None if self.result is None else self.result.get("summary"))

    def answer(self, value) -> tuple:
        if self.prompt is None:
            return False, "nothing is waiting for an answer"
        kind = self.prompt["kind"]
        if kind in ("weigh", "hang", "ruler", "pull"):
            try:
                v = float(value)
            except (TypeError, ValueError):
                return False, f"a number in {self.prompt.get('unit')}"
            lim = dict(weigh=(20, 1000), hang=(0, 60), ruler=(0, 400), pull=(0.05, 5.0))[kind]
            if not lim[0] <= v <= lim[1]:
                return False, f"{v:g} {self.prompt.get('unit')} is out of range {lim}"
            self.answer_value = v
        else:
            self.answer_value = True
        return True, "ok"

    def cancel(self, why: str = "cancelled"):
        self.done, self.why, self.phase, self.prompt = True, why, "done", None

    def refused(self, why: str):
        self._event(f"step {self.k + 1} skipped: {why}")
        self._next(self.t_phase)

    # -- the sequence -------------------------------------------------------------------------------------------------
    def _event(self, s):
        self.events.append(s)
        self.log(s)

    def _set(self, phase, t):
        self.phase, self.t_phase = phase, t

    def _next(self, t):
        self.k += 1
        self.buf = []
        self.prompt = None
        self.answer_value = None
        self._set("start", t)

    def step(self, t: float, obs: dict) -> list:
        if self.done:
            return []
        self._record_motion(obs)
        if self.k < 0:
            self._next(t)
        if not obs.get("ok", True) and self.phase not in ("prompt",):
            self.cancel("the servos reported a fault")
            self._event("stopped: a servo fault")
            return [dict(op="done", result=None)]
        if self.k >= len(self.steps):
            self.phase = "fit"
            try:
                self.result = fit(self.kin, self.records, self.weights)
            except Exception as e:          # noqa: BLE001  (a fit that fails ends the run, with why)
                self.result = None
                self.why = f"fit failed: {e}"
            self.done, self.phase = True, "done"
            return [dict(op="done", result=self.result)]
        st = self.steps[self.k]
        op = st["op"]
        if op == "prompt":
            return self._prompt_step(t, st)
        if op == "present":                       # the gripper where a person can reach it, fingers down
            return self._present(t, obs)
        if op in ("hold", "ruler", "pull"):
            return self._hold_step(t, obs, st)
        if op == "sweep":
            return self._sweep_step(t, obs, st)
        self._next(t)
        return []

    def _record_motion(self, obs):
        """Every other cycle of the moves between the holds (quintic: they accelerate): the rotors' inertia and the
        detector's thresholds on motions like the tidy's. A joint whose goal is still has no direction (left out)."""
        self._mk = getattr(self, "_mk", 0) + 1
        if not obs.get("moving") or self.phase == "sweep" or self._mk % 2:
            return
        gv = np.asarray(obs["goal_vel"], float)
        if np.abs(gv[:N5]).max() < V_EPS:
            return
        d = np.where(np.abs(gv) > V_EPS, np.sign(gv), 0.0)
        self.records.append(_average([obs], "move", d, self.hung, False, None))

    def _prompt_step(self, t, st) -> list:
        if self.phase == "start":
            self.prompt = {k: v for k, v in st.items() if k != "op"}
            self._set("prompt", t)
            return [dict(op="prompt", **self.prompt)]
        if self.answer_value is None:
            return []
        kind, key, v = st["kind"], st.get("key"), self.answer_value
        if kind == "weigh":
            self.weights.setdefault(key, {})["kg"] = v / 1000.0
        elif kind == "hang":
            self.weights.setdefault(key, {})["point"] = hang_point(v)
            self.hung = key
        elif kind == "unhang":
            self.hung = None
        self._event(f"{kind} {key or ''}: {v}")
        self._next(t)
        return []

    def _present(self, t, obs) -> list:
        if self.phase == "start":
            q = self.kin.ik(np.array([0.34, 0.0, 0.30]), np.array([0.0, 0.0, -1.0]), yaw=0.0)
            if q is None:
                self._next(t)
                return []
            q[5] = CLOSED
            self._set("moving", t)
            return [dict(op="move", q=q, frac=0.3)]
        if self.phase == "moving" and not obs["moving"]:
            self._next(t)
        return []

    def _hold_step(self, t, obs, st) -> list:
        q = np.asarray(st["q"], float)
        if self.phase == "start":
            if st["op"] in ("hold",):
                qa = q.copy()
                qa[:N5] -= st["dir"] * DELTA
                self._set("approach", t)
                return [dict(op="move", q=qa, frac=0.3)]
            self._set("final", t)
            return [dict(op="move", q=q, frac=0.3)]
        if self.phase == "approach" and not obs["moving"]:
            self._set("final", t)
            return [dict(op="move", q=q, frac=0.08)]
        if self.phase == "final" and not obs["moving"]:
            self._set("settle", t)
            return []
        settle = SETTLE_WEIGHT if self.hung else SETTLE
        if self.phase == "settle" and t - self.t_phase >= settle:
            if st["op"] == "ruler":
                self.prompt = dict(kind="ruler", unit="mm", key=f"ruler{self.k}", text=(
                    "Stand the ruler upright on the floor beside the fixed finger's end and read how high its lowest "
                    "point is above the floor. Enter it in mm."))
                self._set("prompt", t)
                return [dict(op="prompt", **self.prompt)]
            if st["op"] == "pull":
                self.prompt = dict(kind="pull", unit="kg", key="pull", text=(
                    "Hook the luggage scale on the fixed finger and pull it straight ahead of the robot, level, until it "
                    "reads about 0.5 kg; hold it steady and enter the reading (kg)."))
                self._set("prompt", t)
                return [dict(op="prompt", **self.prompt)]
            self._set("record", t)
        if self.phase == "prompt" and self.answer_value is not None:
            self.prompt = None
            self._set("record", t)
            self.buf = []
            return []
        if self.phase == "record":
            self.buf.append(obs)
            if t - self.t_phase >= RECORD:
                d = np.full(6, float(st.get("dir", 0)))
                rec = _average(self.buf, st["op"], d, self.hung if st["op"] != "pull" else self.hung,
                               st.get("holdout", False), self.answer_value)
                self.records.append(rec)
                if st["op"] == "pull":
                    self.prompt = dict(kind="release", key="release", text="Let go of the luggage scale, then continue.")
                    self._set("released", t)
                    return [dict(op="prompt", **self.prompt)]
                self._next(t)
        if self.phase == "released" and self.answer_value is True:
            self._next(t)
        return []

    def _sweep_step(self, t, obs, st) -> list:
        j, q0, q1, v = st["joint"], np.asarray(st["q0"], float), np.asarray(st["q1"], float), st["speed"]
        dur = abs(q1[j] - q0[j]) / v
        if self.phase == "start":
            self._set("to_start", t)
            return [dict(op="move", q=q0, frac=0.3)]
        if self.phase == "to_start" and not obs["moving"]:
            self._set("sweep", t)
            self.buf = []
            return [dict(op="line", q0=q0, q1=q1, duration=dur)]
        if self.phase == "sweep":
            s = (t - self.t_phase) / dur
            if 0.2 < s < 0.85:                    # the steady middle (the driver's straight line: constant speed)
                self.buf.append(obs)
            if s >= 1.0 and not obs["moving"]:
                d = np.zeros(6)
                d[j] = math.copysign(1.0, q1[j] - q0[j])
                for k, o in enumerate(self.buf):
                    if k % 2 == 0:
                        self.records.append(_average([o], "sweep", d, self.hung, False, None))
                self._next(t)
        return []


def _average(buf, kind, d, weight, holdout, value) -> Record:
    def m(k, n=6):
        return np.mean([np.asarray(o[k], float)[:n] for o in buf], axis=0)
    return Record(kind=kind, q=m("q_enc"), duty=m("duty"), volts=m("volts"), temp=m("temp"), current=m("current"),
                  w=m("w"), goal_vel=m("goal_vel"), goal_acc=m("goal_acc"), g=m("g", 3), dir=d, weight=weight,
                  holdout=holdout, value=value, n=len(buf))


# -- the fit ------------------------------------------------------------------------------------------------------------

LINK_SIGMA = (0.15, 0.004)            # the official model's masses: +-15 %, centres +-4 mm (prior)
SAME_SERVO = (0.03, 0.10)             # pan and roll: their gain and back-EMF held this near the loaded joints'
                                      # (relative): the same servo model. At 10 % the moves' inertia, which the soft
                                      # servo answers late, pulled pan's gain to a ninth (Nov 20, detailed sim arm)
SIGMA_TAU = 0.03                      # N m: a still torque's noise (est)


def _regressor(kin, q, g, links):
    """Y (5 x 4 per link): gravity torque = Y . [m, m cx, m cy, m cz] per link."""
    frames, T = kin.joint_frames(q)
    last = kin._moved_by()
    g = np.asarray(g, float)
    Y = np.zeros((N5, 4 * len(links)))
    for k, l in enumerate(links):
        Tk = T[l["body"]]
        R, t = Tk[:3, :3], Tk[:3, 3]
        for j in range(min(last[l["body"]], N5 - 1) + 1):
            o, z = frames[j]
            Y[j, 4 * k] = np.cross(t - o, g) @ z
            Y[j, 4 * k + 1:4 * k + 4] = R.T @ np.cross(g, z)
    return Y


def _theta(links) -> np.ndarray:
    return np.concatenate([[l["mass"], *(l["mass"] * np.asarray(l["com"], float))] for l in links])


def _links(theta, names) -> list:
    out = []
    for k, b in enumerate(names):
        m = max(theta[4 * k], 1e-4)
        out.append(dict(body=b, mass=float(m), com=theta[4 * k + 1:4 * k + 4] / m))
    return out


def _weight_tau(kin, r: Record, weights, true_q=None):
    if r.weight is None or r.weight not in weights:
        return np.zeros(N5)
    w = weights[r.weight]
    return _weight_torques(kin, r.q if true_q is None else true_q, w["kg"], w["point"], r.g)[:N5]


def _dvp(r: Record, alpha=0.0039) -> np.ndarray:
    return r.duty[:N5] * r.volts[:N5] / (1.0 + alpha * (r.temp[:N5] - 25.0))


def fit_torques(kin, records, weights, qs=None, nominal: LoadModel | None = None, use="duty", same_servo=None) -> tuple:
    """The joints' servo models and the links' masses: linear least squares with bounds and priors (Huber by
    reweighting). qs: the true angles of each record (default its encoders'). same_servo: (A, E) the joints gravity
    loads found; pan (its axis vertical: gravity never loads it, so its gain, friction and back-EMF scale together)
    and roll (hardly loaded) are held near them, the same servo model. Returns (LoadModel, stats)."""
    from scipy.optimize import lsq_linear
    nom = nominal or LoadModel.nominal(kin.cfg)
    names = [l["body"] for l in nom.links]
    th0 = _theta(nom.links)
    nl = len(th0)
    P = 8                                      # per joint: A, E, J, o, F, mu, Fs, mus
    nx = P * N5 + nl
    rows, rhs, wts = [], [], []
    use_rec = [r for r in records if r.kind in ("hold", "sweep", "move") and not r.holdout]
    for k, r in enumerate(use_rec):
        q = r.q if qs is None else qs[id(r)]
        Y = _regressor(kin, q, r.g, nom.links)
        gnom = np.abs(Y @ th0 + _weight_tau(kin, r, weights, q))
        # the links' inertia (point masses, the official masses): known, to the right-hand side
        inert = kin.inertia_torque(q, r.goal_acc, nom.links)[:N5] if r.kind != "hold" else np.zeros(N5)
        dv = _dvp(r) if use == "duty" else r.current[:N5]
        for j in range(N5):
            row = np.zeros(nx)
            b = P * j
            row[b] = dv[j]
            v = r.goal_vel[j] if abs(r.goal_vel[j]) > V_EPS else r.w[j] if abs(r.w[j]) > W_EPS else 0.0
            row[b + 1] = -v if use == "duty" else 0.0
            row[b + 2] = -r.goal_acc[j]
            row[b + 3] = -1.0
            s = r.dir[j]
            if r.kind in ("sweep", "move"):
                if s == 0:                     # a joint not moving: no friction direction known
                    continue
                row[b + 4], row[b + 5] = -s, -s * gnom[j]
            else:
                row[b + 6], row[b + 7] = -s, -s * gnom[j]
            row[P * N5:] = Y[j]
            rows.append(row)
            rhs.append(-_weight_tau(kin, r, weights, q)[j] + inert[j])
            wts.append(1.0 if r.kind == "hold" else 0.35 if r.kind == "sweep" else 0.2)
    A_nom = nom.joints[0].A
    E_nom = nom.joints[0].E
    pri = [(b, v, s) for j in range(N5) for b, v, s in (
        (P * j, A_nom, 0.6 * A_nom), (P * j + 1, E_nom, 2 * E_nom), (P * j + 2, 0.0, 0.05), (P * j + 3, 0.0, 0.2),
        (P * j + 4, 0.05, 0.2), (P * j + 5, 0.3, 1.0), (P * j + 6, 0.05, 0.2), (P * j + 7, 0.3, 1.0))]
    if same_servo is not None:
        a_s, e_s = same_servo
        for j in (0, 4):
            pri = [(b, v, s) for b, v, s in pri if b not in (P * j, P * j + 1)]
            pri += [(P * j, a_s, SAME_SERVO[0] * a_s), (P * j + 1, e_s, SAME_SERVO[1] * e_s)]
    for k in range(nl):
        s = LINK_SIGMA[0] * max(th0[4 * (k // 4)], 0.02) if k % 4 == 0 else LINK_SIGMA[1] * max(th0[4 * (k // 4)], 0.02)
        pri.append((P * N5 + k, th0[k], s))
    if use != "duty":                          # the current model: Ki in A's place (stall torque / stall current), E unused
        ki = float(kin.cfg.servos.get("stall_torque", 2.942)) / 2.7
        pri = [(b, ki, 1.0) if (b < P * N5 and b % P == 0) else (b, v, s) for b, v, s in pri]
    R = np.array(rows)
    y = np.array(rhs)
    w = np.array(wts)
    lo = np.full(nx, -np.inf)
    hi = np.full(nx, np.inf)
    for j in range(N5):
        lo[P * j] = 1e-3
        lo[P * j + 1] = 0.0
        lo[P * j + 2] = 0.0
        lo[P * j + 4:P * j + 8] = 0.0
    for k in range(0, nl, 4):
        lo[P * N5 + k] = 1e-3
    hub = np.ones(len(y))
    x = None
    for _ in range(4):
        Wr = (w * hub / SIGMA_TAU)[:, None]
        M = np.vstack([R * Wr] + [np.eye(nx)[b] / s for b, v, s in pri])
        t = np.concatenate([y * Wr[:, 0], [v / s for b, v, s in pri]])
        x = lsq_linear(M, t, bounds=(lo, hi), lsmr_tol="auto", max_iter=5000).x
        res = (R @ x - y) / SIGMA_TAU
        hub = np.where(np.abs(res) <= 3.0, 1.0, 3.0 / np.maximum(np.abs(res), 1e-9))   # Huber at 3 sigma
    joints = []
    for j in range(N5):
        b = P * j
        if use == "duty":
            joints.append(JointLoad(A=x[b], E=x[b + 1], J=x[b + 2], o=x[b + 3], F=x[b + 4], mu=x[b + 5],
                                    Fs=x[b + 6], mus=x[b + 7]))
        else:
            joints.append(JointLoad(Ki=x[b], J=x[b + 2], o=x[b + 3], F=x[b + 4], mu=x[b + 5], Fs=x[b + 6],
                                    mus=x[b + 7]))
    m = LoadModel(joints, _links(x[P * N5:], names), use_current=use != "duty", source="arm_load fit")
    resid = (R @ x - y)
    return m, dict(rows=len(y), rms_nm=float(np.sqrt(np.mean(resid ** 2))))


def _tip_height(kin, m: LoadModel, q_enc, g, weights, weight) -> float:
    """The fixed finger's end over the floor as the model has it (m): its true angles, gravity's direction."""
    q = true_angles(m, kin, q_enc, g)
    if weight is not None and weight in weights and any(j.compliance for j in m.joints):
        # the weight's torque sags the joints too
        w = weights[weight]
        extra = _weight_torques(kin, q, w["kg"], w["point"], g)
        q[:N5] += np.array([j.compliance for j in m.joints]) * extra[:N5]
    p = _tip_base(kin, q)
    gh = np.asarray(g, float) / np.linalg.norm(g)
    return float(-gh @ p - TIP_R + m.mount_dz)


RULER_JOINTS = (1, 2, 3)               # lift, elbow, wrist flex: the ones heights see


def fit_ruler(kin, m: LoadModel, records, weights, skip: int | None = None) -> tuple:
    """The joints' zeros and sag (lift, elbow, wrist flex) and the floor's height against base_link, from the
    ruler's readings (priors: zero 3 deg, sag 2 deg/N m, floor 6 mm). Returns (model, x)."""
    from scipy.optimize import least_squares
    rs = [r for k, r in enumerate(x for x in records if x.kind == "ruler") if k != skip]
    if len(rs) < 4:
        return m, None

    def apply(x):
        mm = LoadModel([JointLoad(**asdict(j)) for j in m.joints], m.links, m.use_current, m.source, x[6])
        for k, j in enumerate(RULER_JOINTS):
            mm.joints[j].zero = x[k]
            mm.joints[j].compliance = x[3 + k]
        return mm

    def res(x):
        mm = apply(x)
        r = [(_tip_height(kin, mm, r.q, r.g, weights, r.weight) * 1000 - r.value) / 1.0 for r in rs]
        pri = list(x[:3] / math.radians(3.0)) + list(x[3:6] / math.radians(2.0)) + [x[6] / 0.006]
        return np.array(r + pri)

    x = least_squares(res, np.zeros(7), loss="soft_l1", f_scale=2.0).x
    return apply(x), x


def _static_errors(kin, m: LoadModel, records, weights, true_weights=None) -> dict:
    """On the held-out holds: the outside torque from the model at the approach direction (the friction's side known)
    against the truth (zero, or the weight's torque); the weight's vertical force at its point."""
    errs, ferr = [], []
    tw = true_weights or weights
    for r in records:
        if r.kind != "hold" or not r.holdout:
            continue
        q = true_angles(m, kin, r.q, r.g)
        e = estimate(m, kin, q, r.w, r.duty, r.volts, r.temp, r.current, np.zeros(6), np.zeros(6), r.g)
        truth = _weight_tau(kin, r, tw, q)
        # the gear's friction grows with the whole load it carries, the weight's too (the fit's own regressor)
        fb = np.array([j.Fs + j.mus * abs(gv + wv) for j, gv, wv in zip(m.joints, e["grav"][:N5], truth)])
        ext = -e["grav"][:N5] - e["motor"][:N5] + r.dir[:N5] * fb
        errs.append(ext - truth)
        if r.weight is not None and r.weight in tw:
            J = kin.jacobian(q, tw[r.weight]["point"])
            f = np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), J @ ext)
            ferr.append(f[2] - (-9.81 * tw[r.weight]["kg"]))
    E = np.array(errs) if errs else np.zeros((1, N5))
    return dict(per_joint_rms=[float(v) for v in np.sqrt(np.mean(E ** 2, axis=0))],
                static_rms_nm=float(np.sqrt(np.mean(E ** 2))),
                weight_err_n=float(np.sqrt(np.mean(np.square(ferr)))) if ferr else float("nan"),
                n=len(errs))


def fit(kin, records, weights, nominal: LoadModel | None = None) -> dict:
    """The whole fit: torques (duty model) at the encoders' angles, the ruler's zeros and sag, the torques again at the
    true angles; the current model if the servos report it; thresholds; held-out checks. Returns the result: the
    driver's json (value), the metrics, a summary."""
    nom = nominal or LoadModel.nominal(kin.cfg)
    m, st = fit_torques(kin, records, weights, nominal=nom)
    loaded = (1, 2, 3)                                   # lift, elbow, wrist flex: gravity fixes their gain
    same = (float(np.mean([m.joints[j].A for j in loaded])), float(np.mean([m.joints[j].E for j in loaded])))
    m, st = fit_torques(kin, records, weights, nominal=nom, same_servo=same)
    m, x = fit_ruler(kin, m, records, weights)
    qs = {id(r): true_angles(m, kin, r.q, r.g) for r in records}
    m2, st = fit_torques(kin, records, weights, qs=qs, nominal=nom, same_servo=same)
    for j, j0 in zip(m2.joints, m.joints):
        j.zero, j.compliance = j0.zero, j0.compliance
    m2.mount_dz = m.mount_dz
    m = m2
    stats = dict(torque=st)
    # the current model, where the servos report one: kept if it does better on the held-out holds
    have_i = any(np.abs(r.current).max() > 1e-6 for r in records)
    if have_i:
        mc, sti = fit_torques(kin, records, weights, qs=qs, nominal=nom, use="current")
        for j, jd in zip(mc.joints, m.joints):
            j.A, j.E, j.zero, j.compliance = jd.A, jd.E, jd.zero, jd.compliance
        mc.mount_dz, mc.links = m.mount_dz, m.links
        e_d = _static_errors(kin, m, records, weights)["static_rms_nm"]
        e_c = _static_errors(kin, mc, records, weights)["static_rms_nm"]
        stats["current"] = dict(**sti, holdout_duty=e_d, holdout_current=e_c)
        if e_c < e_d:
            m = mc
    # the checks on the fitted model; then the driver's copy: the band still widened by the held-out scatter (the gear's
    # friction holds anywhere in it), thresholds from the sweeps' scatter
    held = _static_errors(kin, m, records, weights)
    nheld = _static_errors(kin, nom, records, weights)
    ruler = _ruler_check(kin, m, nom, records, weights)
    sweep_res = _sweep_residuals(kin, m, records, weights)
    m = LoadModel([JointLoad(**asdict(j)) for j in m.joints], m.links, m.use_current, "arm_load", m.mount_dz)
    for i, j in enumerate(m.joints):
        sig_s = held["per_joint_rms"][i]
        j.Fs = j.Fs + 2.0 * sig_s
        j.mus = j.mus * 1.15
        j.thr = float(max(0.10, 3.0 * sig_s))                 # still: the held-out scatter
        j.thr_move = float(max(0.15, 5.0 * sweep_res[i]))      # moving: the sweeps' and moves' scatter
    metrics = dict(static_rms_nm=held["static_rms_nm"], weight_err_n=held["weight_err_n"],
                   ruler_rms_mm=ruler["loo_rms_mm"], per_joint_rms_nm=held["per_joint_rms"],
                   nominal_static_rms_nm=nheld["static_rms_nm"], nominal_weight_err_n=nheld["weight_err_n"],
                   nominal_ruler_rms_mm=ruler["nominal_rms_mm"], sweep_rms_nm=[float(v) for v in sweep_res],
                   records=len(records), use_current=m.use_current, pull=_pull_check(kin, m, nom, records))
    value = m.to_json(kin.cfg, fit=stats)
    summary = (f"still torque {held['static_rms_nm']:.3f} N m rms (nominal {nheld['static_rms_nm']:.3f}), weight "
               f"{held['weight_err_n']:.2f} N (nominal {nheld['weight_err_n']:.2f}), heights "
               f"{ruler['loo_rms_mm']:.1f} mm (nominal {ruler['nominal_rms_mm']:.1f})"
               + (", current model" if m.use_current else ""))
    return dict(value=value, metrics=metrics, summary=summary, model=m)


def _sweep_residuals(kin, m: LoadModel, records, weights) -> np.ndarray:
    """Each joint's outside-torque scatter while moving (the sweeps and the quintic moves): the detector's noise."""
    out = [[] for _ in range(N5)]
    for r in records:
        if r.kind not in ("sweep", "move"):
            continue
        q = true_angles(m, kin, r.q, r.g)
        e = estimate(m, kin, q, r.w, r.duty, r.volts, r.temp, r.current, r.goal_vel, r.goal_acc, r.g)
        wt = _weight_tau(kin, r, weights, q)
        for j in range(N5):
            if r.dir[j] != 0:
                out[j].append(e["ext"][j] - wt[j])
    return np.array([float(np.sqrt(np.mean(np.square(v)))) if v else 0.05 for v in out])


def _ruler_check(kin, m, nom, records, weights) -> dict:
    rs = [r for r in records if r.kind == "ruler"]
    if len(rs) < 4:
        return dict(loo_rms_mm=float("nan"), nominal_rms_mm=float("nan"))
    loo = []
    base = LoadModel([JointLoad(**{**asdict(j), "zero": 0.0, "compliance": 0.0}) for j in m.joints], m.links,
                     m.use_current, m.source, 0.0)
    for k, r in enumerate(rs):
        mk, _ = fit_ruler(kin, base, records, weights, skip=k)
        loo.append(_tip_height(kin, mk, r.q, r.g, weights, r.weight) * 1000 - r.value)
    nomh = [_tip_height(kin, nom, r.q, r.g, weights, r.weight) * 1000 - r.value for r in rs]
    return dict(loo_rms_mm=float(np.sqrt(np.mean(np.square(loo)))),
                nominal_rms_mm=float(np.sqrt(np.mean(np.square(nomh)))))


def _pull_check(kin, m, nom, records) -> dict | None:
    rs = [r for r in records if r.kind == "pull" and r.value]
    if not rs:
        return None
    r = rs[-1]
    out = {}
    for name, mm in (("calibrated", m), ("nominal", nom)):
        q = true_angles(mm, kin, r.q, r.g)
        e = estimate(mm, kin, q, r.w, r.duty, r.volts, r.temp, r.current, np.zeros(6), np.zeros(6), r.g)
        c = -e["grav"][:N5] - e["motor"][:N5]                # the band's middle: a still pull's best guess
        out[name] = [float(v) for v in tool_force(kin, q, c, TIP)]
    out["scale_n"] = float(r.value * 9.81)
    return out
