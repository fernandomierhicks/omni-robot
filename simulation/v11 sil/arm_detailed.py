"""The SO-101 in detail (params.arm_model = "detailed", design.md 4.5): what the robot's arm calibration has to find, as a
truth drawn per seed. arm_so101.ArmServos (the default, "simple") stays the arm of every earlier result.

Each of the six joints, from the motor to the link (all est, no STS3215 measured yet, Nov 20):

  motor     a DC motor behind the H-bridge: i = (duty V - Ke w) / R(T), torque Ke i (output-referred: Ke 2.0 V s/rad,
            R 4.44 ohm with the bridge, so 12 V stalls at 2.7 A and 2.94 N m through the gearbox as the reseller says);
            the winding heats (R +0.39 %/C, Ke -0.12 %/C), the case thermistor follows it slowly
  rotor     its inertia through the 1:345 gearbox (J_r ~0.02 kg m2 at the output), integrated here every physics step
  gearbox   Coulomb + viscous friction, stiction, and load-dependent losses: 45 % of the motor's torque when it drives
            (efficiency 0.55), 55 % of the load's when the load drives it (0.45: hard to back-drive); backlash ~0.2 deg;
            the train itself stiff (3000 N m/rad)
  encoder   on the output shaft (servo_emulator: 12 bit, once- and twice-per-turn errors, noise, the hand calibration's
            offset)
  horn+link the horn's spline, the printed brackets and links as one torsion spring per joint (35-90 N m/rad), past
            the encoder: the sag under gravity it never sees
  link      MuJoCo's joint: the drivetrain's torque reaches it through a position actuator (kp = the gear and the horn in
            series, kv their damping) whose set point is the rotor's angle outside the backlash

The links' masses are the official model's x N(1, 7 %), their centres moved N(0, 3 mm), plus the servo leads; the robot
only knows the official model. The supply is a 12 V converter (+-1 %) through 0.2 ohm of leads to the six servos.
Servo firmware, sensors and the bus are servo_emulator.py's.

  arm = DetailedArm(model, data, p, "stow", seed=3)      # robot.py makes it for params.arm_model "detailed"
  bus = servo_emulator.ServoBus(arm.servos)               # the robot's driver talks to this
  arm.hang(0.2, (x, y, z))                                # a weight on a string from a point in the gripper's frame
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

import arm_so101 as A
from servo_emulator import FW_DT, TICK, SensorTruth, Sts3215

T_AMB = 24.0
G = 9.81
V_SET, R_LEADS, I_IDLE = 12.0, 0.20, 0.03     # V, ohm, A per servo (its electronics)

# per joint (pan, lift, elbow, wrist flex, wrist roll, gripper): horn + link stiffness N m/rad, backlash deg, the link's
# inertia about the joint for the drivetrain's damping (kg m2, est)
K_FLEX = (90.0, 70.0, 60.0, 45.0, 35.0, 60.0)
# (the play the loop lives with: at 0.4-0.5 deg every unloaded joint hunts at full duty, the encoder can't see the
# motor cross the gap; at ~0.2 deg it holds still at LeRobot's P 16 and jitters a little at the factory P 32)
BACKLASH_DEG = (0.18, 0.20, 0.20, 0.18, 0.16, 0.22)
I_LINK = (6e-3, 6e-3, 2e-3, 3e-4, 1.5e-4, 6e-5)
K_GEAR = 3000.0                                # N m/rad: the metal gear train and the output shaft (est). It must be
                                               # well over the servo's own loop stiffness (~1000 N m/rad, the official
                                               # model's kp): the encoder sits past it, so a soft train hunts
ZETA = 0.12                                    # the drivetrain's damping ratio
LEAD_KG = {"arm_upper_arm": 0.006, "arm_lower_arm": 0.006, "arm_wrist": 0.004}   # the servo leads riding each link
LINKS = ("arm_shoulder", "arm_upper_arm", "arm_lower_arm", "arm_wrist", "arm_gripper", "arm_jaw")


@dataclass
class ServoTruth:
    """One servo's motor and gearbox as built (est)."""
    R0: float = 4.44           # ohm at 25 C (winding + bridge)
    Ke: float = 2.0            # V s/rad = N m/A, output-referred
    J_r: float = 0.02          # kg m2: the rotor through the gearbox
    Fc: float = 0.03           # N m: Coulomb friction at the output
    Fs: float = 0.045          # N m: static
    b: float = 0.02            # N m s/rad
    eta_f: float = 0.55        # the gearbox driven by the motor
    eta_b: float = 0.45        # driven by the load
    backlash: float = math.radians(0.4)
    k_flex: float = 60.0
    k_gear: float = K_GEAR
    c: float = 0.1             # N m s/rad: the drivetrain's damping
    R_wc: float = 4.0          # K/W winding -> case
    C_w: float = 3.0           # J/K
    R_ca: float = 18.0         # K/W case -> air
    C_c: float = 25.0

    @property
    def k_s(self) -> float:
        return 1.0 / (1.0 / self.k_gear + 1.0 / self.k_flex)

    @staticmethod
    def draw(rng: np.random.Generator, j: int) -> "ServoTruth":
        fc = abs(rng.normal(0.03, 0.008))
        k_flex = K_FLEX[j] * rng.normal(1.0, 0.12)
        s = ServoTruth(R0=rng.normal(4.44, 0.15), Ke=rng.normal(2.0, 0.06), J_r=rng.normal(0.02, 0.002), Fc=fc,
                       Fs=fc * rng.uniform(1.3, 1.7), b=abs(rng.normal(0.02, 0.005)), eta_f=rng.normal(0.55, 0.03),
                       eta_b=rng.normal(0.45, 0.03), backlash=math.radians(BACKLASH_DEG[j] * rng.uniform(0.7, 1.3)),
                       k_flex=k_flex, k_gear=K_GEAR * rng.normal(1.0, 0.1), R_ca=rng.normal(18.0, 1.5))
        s.c = 2 * ZETA * math.sqrt(s.k_s * I_LINK[j])
        return s


class DetailedArm:
    """The six servos' physics under servo_emulator's firmware; robot.py steps it before each physics step (as
    arm_so101.ArmServos)."""

    def __init__(self, model, data, p, pose="stow", seed: int = 0, joint_err_deg: float = 1.0):
        self.m, self.d, self.p = model, data, p
        rng = np.random.default_rng(seed + 4242)
        self.a = np.array([model.actuator(A.name(j)).id for j in A.JOINTS])
        self.qadr = np.array([model.joint(A.name(j)).qposadr[0] for j in A.JOINTS])
        self.vadr = np.array([model.joint(A.name(j)).dofadr[0] for j in A.JOINTS])
        self.lo, self.hi = A.joint_ranges().T
        self.truth = [ServoTruth.draw(rng, j) for j in range(6)]
        self.mass_truth = self._perturb_masses(rng)
        q0 = np.clip(np.asarray(A.POSES[pose] if isinstance(pose, str) else pose, float), self.lo, self.hi)
        data.qpos[self.qadr] = q0
        data.qvel[self.vadr] = 0.0
        self.th_m = q0.copy()                    # the rotors (output-referred angles)
        self.w_m = np.zeros(6)
        self.i = np.zeros(6)                     # motor currents
        self.duty = np.zeros(6)
        self.T_w = np.full(6, T_AMB)
        self.T_c = np.full(6, T_AMB)
        self.v_set = V_SET * rng.normal(1.0, 0.01)
        self.v = self.v_set
        srng = np.random.default_rng(seed + 977)
        self.servos = {}
        for j in range(6):
            sens = SensorTruth.draw(srng, joint_err_deg if j < 5 else 0.0)
            self.servos[j + 1] = Sts3215(j + 1, sens, np.random.default_rng(seed * 31 + j), angle0=float(q0[j]))
        self._sv = [self.servos[j + 1] for j in range(6)]
        self.t = 0.0
        self.t_fw = 0.0
        self.hung = None                         # (kg, point in the gripper's frame): a weight on a string
        self.pull = None                         # (force N world (3,), point in the gripper's frame)
        self.g_body = model.body("arm_gripper").id
        for k, j in enumerate(self.a):           # the set point is the rotor's: no range, no force limit
            model.actuator_ctrllimited[j] = 0
            model.actuator_forcelimited[j] = 0
        self._set_drive()

    # -- the truth ------------------------------------------------------------------------------------------------
    def _perturb_masses(self, rng) -> dict:
        m = self.m
        out = {}
        for name in LINKS:
            b = m.body(name).id
            m0, c0 = float(m.body_mass[b]), m.body_ipos[b].copy()
            k = rng.normal(1.0, 0.07)
            mass, com = m0 * k, c0 + rng.normal(0.0, 0.003, 3)
            lead = LEAD_KG.get(name, 0.0)
            if lead:                                         # a lead along the link: at its centre's place (est)
                com = (mass * com + lead * c0) / (mass + lead)
                mass += lead
            m.body_mass[b] = mass
            m.body_ipos[b] = com
            m.body_inertia[b] *= mass / m0
            out[name] = dict(mass=mass, com=com.tolist(), nominal_mass=m0, nominal_com=c0.tolist())
        return out

    # -- arm_so101.ArmServos' interface ---------------------------------------------------------------------------
    @property
    def target(self) -> np.ndarray:
        return np.array([(sv.get("goal_position") - 2048) * TICK for sv in self._sv])

    def set_pose(self, q, snap: bool = False):
        q = np.clip(np.asarray(q, float), self.lo, self.hi)
        for j, sv in enumerate(self._sv):
            sv._put("goal_position", int(round(2048 + q[j] / TICK)))
        if snap:
            self.d.qpos[self.qadr] = q
            self.d.qvel[self.vadr] = 0.0
            self.th_m[:] = q
            self.w_m[:] = 0.0

    def q(self) -> np.ndarray:
        return self.d.qpos[self.qadr].copy()

    def settled(self, tol=math.radians(1.0)) -> bool:
        return bool(np.all(np.abs(self.q() - self.target) < tol) and np.all(np.abs(self.w_m) < 0.05))

    # -- the physics ------------------------------------------------------------------------------------------------
    def shaft_angles(self) -> np.ndarray:
        """The output shafts' true angles (where the encoders are): the link plus the horn's and the link's twist."""
        tau = self.d.actuator_force[self.a]
        return self.q() + tau / np.array([s.k_flex for s in self.truth])

    def _set_drive(self):
        m, d = self.m, self.d
        q = d.qpos[self.qadr]
        for j, s in enumerate(self.truth):
            a = self.a[j]
            gap = self.th_m[j] - q[j]
            half = s.backlash / 2
            if abs(gap) > half:                              # the teeth touch: the gear and the horn in series
                kp, kv = s.k_s, s.c
                d.ctrl[a] = self.th_m[j] - math.copysign(half, gap) + kv / kp * self.w_m[j]
            else:                                            # inside the backlash: nothing reaches the link
                kp = kv = 0.0
                d.ctrl[a] = q[j]
            m.actuator_gainprm[a, 0] = kp
            m.actuator_biasprm[a, 1] = -kp
            m.actuator_biasprm[a, 2] = -kv

    def step(self, dt: float):
        d = self.d
        self.t += dt
        tau = d.actuator_force[self.a].copy()               # what the drivetrain put on each link last step
        if self.t >= self.t_fw - 1e-12:                      # the servos' firmware tick
            self.t_fw += FW_DT
            th_o = self.shaft_angles()
            for j, sv in enumerate(self._sv):
                self.duty[j] = sv.update(self.t, float(th_o[j]), float(self.i[j]), self.v, float(self.T_c[j]))
        for j, s in enumerate(self.truth):
            sv = self._sv[j]
            R = s.R0 * (1 + 0.0039 * (self.T_w[j] - 25.0))
            Ke = s.Ke * (1 - 0.0012 * (self.T_w[j] - 25.0))
            on = sv.get("torque_enable") == 1 and not sv.unloaded
            # duty 0 with the torque on: the bridge brakes (both low sides on); torque off: it coasts
            i = (self.duty[j] * self.v - Ke * self.w_m[j]) / R if on else 0.0
            self.i[j] = i
            te = Ke * i
            net = te - tau[j]
            w = self.w_m[j]
            load_f = (1 - s.eta_f) * abs(te) if te * w > 0 else (1 - s.eta_b) * abs(tau[j])
            if abs(w) < 1e-4:
                f_static = s.Fs + max((1 - s.eta_f) * abs(te), (1 - s.eta_b) * abs(tau[j]))
                if abs(net) <= f_static:                     # stuck
                    w_new = 0.0
                else:
                    w_new = w + dt / s.J_r * (net - math.copysign(s.Fc + load_f, net))
            else:
                f = s.Fc + s.b * abs(w) + load_f
                w_new = w + dt / s.J_r * (net - math.copysign(f, w))
                if w_new * w < 0:                            # friction stops it, it doesn't turn it round
                    w_new = 0.0
            self.w_m[j] = w_new
            self.th_m[j] += w_new * dt
            # heat: the winding, the case, the air
            p = i * i * R
            self.T_w[j] += dt / s.C_w * (p - (self.T_w[j] - self.T_c[j]) / s.R_wc)
            self.T_c[j] += dt / s.C_c * ((self.T_w[j] - self.T_c[j]) / s.R_wc - (self.T_c[j] - T_AMB) / s.R_ca)
        i_sup = float(np.sum(np.maximum(0.0, self.duty * self.i))) + 6 * I_IDLE
        self.v = self.v_set - R_LEADS * i_sup
        self._set_drive()
        self._external()

    def bus_current(self, v_bus: float) -> float:
        """The pack's current for the 12 V rail (DC-DC 90 %)."""
        i_sup = float(np.sum(np.maximum(0.0, self.duty * self.i))) + 6 * I_IDLE
        return i_sup * self.v / v_bus / 0.9

    # -- what a person does to it -----------------------------------------------------------------------------------
    def hang(self, kg: float, point):
        """A weight of kg on a string from point (the gripper's frame, m): its weight pulls straight down there."""
        self.hung = (float(kg), np.asarray(point, float))

    def unhang(self):
        self.hung = None

    def push(self, force_world, point):
        """A steady hand force (N, world) at point (the gripper's frame); None to let go."""
        self.pull = None if force_world is None else (np.asarray(force_world, float), np.asarray(point, float))

    def _external(self):
        d, b = self.d, self.g_body
        d.xfrc_applied[b] = 0.0
        for f, pt in ((None, None) if self.hung is None else (np.array([0, 0, -G * self.hung[0]]), self.hung[1]),
                      (None, None) if self.pull is None else self.pull):
            if f is None:
                continue
            P = self.point_world(pt)
            d.xfrc_applied[b, :3] += f
            d.xfrc_applied[b, 3:] += np.cross(P - d.xipos[b], f)

    def point_world(self, pt) -> np.ndarray:
        b = self.g_body
        return self.d.xpos[b] + self.d.xmat[b].reshape(3, 3) @ np.asarray(pt, float)

    def tip_height(self) -> float:
        """The fixed fingertip's lowest point over the world floor (z 0), m: what a ruler on the floor reads."""
        gid = self.m.geom("arm_tip0").id
        return float(self.d.geom_xpos[gid][2] - self.m.geom_size[gid][0])

    def summary(self) -> dict:
        return dict(truth=[s.__dict__ for s in self.truth], masses=self.mass_truth,
                    sensors=[sv.s.__dict__ for sv in self._sv], v_set=self.v_set)


def tip_point_gripper() -> np.ndarray:
    """The fixed fingertip sphere's centre in the gripper's frame (arm_so101.FINGERTIPS)."""
    return np.array(A.FINGERTIPS[0][1], float)


def model_check(seed: int = 0) -> dict:
    """Quick physics check: the arm at stow, held 2 s: how far it sagged past its encoders."""
    from robot_export import arm_params
    from robot_model import build_model
    p = arm_params(arm_model="detailed")
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    arm = DetailedArm(m, d, p, A.start_pose(p), seed=seed)
    for _ in range(4000):
        arm.step(m.opt.timestep)
        mujoco.mj_step(m, d)
    return dict(q=arm.q().tolist(), shaft=arm.shaft_angles().tolist(), duty=arm.duty.tolist(), i=arm.i.tolist())
