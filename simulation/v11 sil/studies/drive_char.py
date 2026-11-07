"""Drive-motor characterization of the v10 arm robot (Nov 1 2026; design.md 4.7, plan: is the AK40-10 KV170 right for this
robot, and do cheaper motors do as well?).

The robot: robot_export.arm_params() (cad/v10's coarse model: 140 mm 2x6x36 printed wheels with plain rollers on M3 pins,
hinged sprung forks, the SO-101 stowed, OVONIC 6S 5200), explicit rollers, at 10 / 15 / 20 kg (the extra mass at the
CAD's CG) and 20 kg with the CG 40 mm higher ("20hi"). Drive motors from catalog.DRIVE_MOTORS: "AK40-10" is fitted to the
maker's 24 V curve (motor_fit.py); the "-ds" entries are datasheet fits by one rule (catalog.datasheet_fit), the AK40-10
included as "AK40-10-ds", so every candidate compares on equal terms.

Physics-capability parts (the Python controller with no software caps: no speed or yaw-rate cap, the wheel-speed ceiling
at the live bus voltage, no acceleration table; each test sets the acceleration it asks for):

  C  top speed per body direction; steady cruise power; rest-to-rest straights of 2 / 5 / 10 / 15.2 m (15.2 m = 50 ft,
     the house's longest straight) at speed caps and accelerations
  A  clean acceleration and braking per direction (from rest, and at speed), reversals, direction changes at speed,
     unramped panic stops
  R  spin in place (top yaw rate), translate-while-turning envelope
  S  square steps and triangular bumps (height ladder per direction and approach speed, creep included), and restart
     from rest against a step's edge
  F  house transitions (thresholds, rug edges, carpet trim, a 20 mm rug) per direction and speed
  I  slopes by tilted gravity (start uphill from rest, hold for 30 s, downhill braking, top speed uphill) on floor and
     carpet; real ramps (floor.Incline) for the foot and the crest
  U  one-sided bumps (one pair of wheels), a wheel set low or high (floor dip / build error) under hard acceleration
  T  thermal: sustained climbing, start-stop bursts, spinning, pushing against an edge it cannot climb (stall)
  P  suspension sensitivity (pod rate x damping x preload, a near-rigid reference) on a scenario subset

Every run records why it stopped being clean (Mon / classify): torque (a wheel at its current limit), rpm (at its back-EMF
limit), thermal (the driver's derate), supply (bus under 21 V), grip (the base slides: odometry vs truth), tipping (a wheel
off the floor > 50 ms, or flipped), grounding (belly or hull on an obstacle or the floor).

  python drive_char.py PART [--motors AK40-10,DM-2325-ds] [--masses 10,20] [--surfaces floor,carpet] [--socs 1.0,0.2]
                            [--smoke] [--workers N]
  python drive_char.py report [PART]
Results: results/motor_char/<PART>.jsonl, one line per job, appended as jobs finish (a rerun skips the jobs already
there, so an interrupted run resumes).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, replace

import mujoco
import numpy as np

from common import RESULTS, tilt_deg  # noqa: E402
import robot_export as E  # noqa: E402
from catalog import DRIVE_MOTORS, Thermal  # noqa: E402
from floor import Carpet, Incline, Threshold  # noqa: E402
import floor as floor_mod  # noqa: E402
from floor_wheels import OBSTACLES  # noqa: E402
from omni_control import wrap_angle, world_to_body  # noqa: E402
from robot import Robot  # noqa: E402
from robot_model import build_model  # noqa: E402

OUT = RESULTS / "motor_char"
OUT.mkdir(parents=True, exist_ok=True)
WORKERS = min(27, (os.cpu_count() or 4) - 1)

# ---------------------------------------------------------------------------------------------------------------------
# configurations

ALT_MOTORS = ("AK40-10", "AK40-10-ds", "DM-2325-ds", "DM-2325-low-ds", "DM-J3507-ds", "GIM4305-10-ds", "GIM3510-8-ds",
              "EduLite05-ds", "M2006-ds")
MASSES = {"10": (10.0, 0.0), "15": (15.0, 0.0), "20": (20.0, 0.0), "20hi": (20.0, 0.04)}
V_STOP = 21.0                         # the firmware's provisional low-battery stop (6S, 3.5 V per cell)

# the AK40-10's thermal resistances x 0.5 and x 1.5 (both nodes): the (est) thermal model's sensitivity
for _tag, _k in (("th0.5", 0.5), ("th1.5", 1.5)):
    _m = DRIVE_MOTORS["AK40-10"]
    _t = _m.thermal
    DRIVE_MOTORS[f"AK40-10@{_tag}"] = replace(_m, thermal=Thermal(_t.c_winding, _t.c_case, _t.r_wc * _k, _t.r_ca * _k,
                                                                    _t.t_max))


@dataclass(frozen=True)
class Cfg:
    motor: str = "AK40-10"
    mass: str = "10"
    soc: float = 1.0
    surface: str = "floor"            # "floor" (hard, the wheel's mu), "dusty" (mu 0.6), "carpet" (whole floor)
    springs: str = "matched"          # "matched": pod rate and preload scale with the sprung mass (the same sag as the
                                      # 10 kg ARM); "fixed": ARM's 3000 N/m / 2 N at any mass
    pod_rate: float | None = None     # P: explicit overrides (N/m at the wheel, damping ratio, preload N)
    pod_zeta: float | None = None
    pod_preload: float | None = None
    loop: float = 1.0                 # the drives' speed-loop gains x this (kp 2, ki 40 at 1; the CubeMars loop's own
                                      # gains are unknown): the wheels' ripple and internal fight follow it
    pivot_z: float | None = None      # m: the fork pivot's height over the axle (cad/v10: 0 since Nov 2, 0.043 before); the pivot stays 80.6
                                      # mm from the axle. 0 = level with the axle: no torque jacking (drive torque and
                                      # traction cancel at the hinge)
    hands: str | None = None          # pod_hands override: "diagonal" (cad/v10) or "same" (every pod the same hand)
    bleed: bool = False               # drain the four speed loops' integrators' null-space part (their internal fight)
                                      # with a 50 ms time constant: a firmware option this study tests, not built


CARPET_FLOOR = Carpet(-40.0, -15.0, 40.0, 15.0, thickness=0.008, mu=0.85, crr=0.06, softness=0.012)
SPRUNG_10 = None                      # the 10 kg robot's sprung mass (set on first use)


def physics_limits(accel: float = 3.0, jerk: float = 0.0) -> dict:
    return dict(limits="physics", base_max_speed=math.inf, base_max_accel=accel, accel_table=None, base_max_jerk=jerk,
                base_max_yaw_rate=math.inf, base_max_yaw_accel=100.0, brake_margin=0.9, wheel_speed_frac=0.97,
                live_wheel_limit=True, terrain_assist=False, slow_zones=())


def params(cfg: Cfg, features=(), accel: float = 3.0, jerk: float = 0.0, **kw):
    global SPRUNG_10
    kg, dz = MASSES[cfg.mass]
    feats = tuple(features)
    if cfg.surface == "carpet":
        feats = (CARPET_FLOOR,) + feats
    extra = dict(physics_limits(accel, jerk), drive_motor=cfg.motor, battery_soc=cfg.soc, area_size=(90.0, 34.0),
                 home_xy=(0.0, 0.0), floor_features=feats)
    if cfg.surface == "dusty":
        extra["wheel_mu"] = 0.6
    if cfg.loop != 1.0:
        extra.update(drive_kp=2.0 * cfg.loop, drive_ki=40.0 * cfg.loop)
    extra.update(kw)
    p = E.arm_params(kg, dz, **extra)
    if SPRUNG_10 is None:
        SPRUNG_10 = E.arm_params(10.0).pod_sprung_mass
    pod = {}
    if cfg.springs == "matched":
        k = p.pod_sprung_mass / SPRUNG_10
        pod = dict(pod_rate=p.pod_rate * k, pod_preload=p.pod_preload * k)
    if cfg.pod_rate is not None:
        pod["pod_rate"] = cfg.pod_rate
    if cfg.pod_zeta is not None:
        pod["pod_damping_ratio"] = cfg.pod_zeta
    if cfg.pod_preload is not None:
        pod["pod_preload"] = cfg.pod_preload
    if cfg.hands is not None:
        pod["pod_hands"] = cfg.hands
    if cfg.pivot_z is not None:
        rho = math.hypot(*p.pod_pivot)
        pod["pod_pivot"] = (math.sqrt(rho * rho - cfg.pivot_z ** 2), cfg.pivot_z)
    return p.with_(**pod) if pod else p


def rig(cfg: Cfg, x0=0.0, y0=0.0, yaw=0.0, slope_deg=0.0, uphill=(1.0, 0.0), settle=0.4, features=(), accel=3.0,
        jerk=0.0, robot_cls=Robot, **kw):
    """(m, d, r): the robot at (x0, y0) facing yaw, gravity tilted so the floor is a slope_deg slope rising toward the
    world direction `uphill`, settled on its wheels."""
    p = params(cfg, features, accel, jerk, **kw)
    m = build_model(p)
    if slope_deg:
        s, c = math.sin(math.radians(slope_deg)), math.cos(math.radians(slope_deg))
        u = np.asarray(uphill, float) / np.linalg.norm(uphill)
        m.opt.gravity[:] = (-9.81 * s * u[0], -9.81 * s * u[1], -9.81 * c)
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr] = x0
    d.qpos[adr + 1] = y0
    if cfg.surface == "carpet":
        d.qpos[adr + 2] += CARPET_FLOOR.thickness + 0.002
    d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(m, d)
    r = robot_cls(m, d, p)
    if cfg.bleed and r.nw == 4:
        _add_bleed(r)
    if slope_deg:                       # hold position while it settles on the slope
        for _ in range(int(settle / m.opt.timestep)):
            r.drive_velocity((0.0, 0.0), 0.0, r.pose()[2])
            r.step()
    else:
        for _ in range(int(settle / m.opt.timestep)):
            r.step()
    # the drop onto the wheels at spawn winds the speed loops' integrators against each other (the robot's start, not
    # its driving): start every scenario from rest with them empty
    for drv in r.drives:
        drv.integ = 0.0
    for _ in range(int(0.2 / m.opt.timestep)):
        if slope_deg:
            r.drive_velocity((0.0, 0.0), 0.0, r.pose()[2])
        r.step()
    return m, d, r


def _add_bleed(r, tau=0.05):
    """Wrap r.step: after each step remove a dt/tau share of the drives' integrators' null-space component (the four
    wheels squeezing each other; it moves the base nowhere)."""
    U, S, Vt = np.linalg.svd(np.asarray(r.kin.J).T)
    null = Vt[-1]
    k = min(1.0, r.m.opt.timestep / tau)
    step0 = r.step

    def step():
        step0()
        ig = np.array([dv.integ for dv in r.drives])
        ig -= null * float(null @ ig) * k
        for dv, x in zip(r.drives, ig):
            dv.integ = float(x)
    r.step = step


def unit(deg: float) -> np.ndarray:
    return np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])


_RATIO = {}


def ratio(cfg: Cfg, direction: float) -> float:
    """The base's true speed over the commanded one along a body direction, cruising at 0.6 m/s on cfg's surface
    (the barrel rollers' rolling radius and the rollers' creep: 0.92-0.98 on the 140 mm wheel). A real robot gets this
    from its odometry calibration; the tests command v / ratio and judge the true speed against v. Mirrored: 0-180."""
    dd = abs(((direction + 180.0) % 360.0) - 180.0)
    k = (cfg.motor, cfg.mass, cfg.surface, cfg.springs, cfg.pod_rate, cfg.pod_zeta, cfg.pod_preload, cfg.pivot_z,
         cfg.hands, cfg.loop, cfg.bleed, round(dd, 1))
    if k not in _RATIO:
        cal = replace(cfg, soc=1.0)
        yaw = -math.radians(dd)
        m, d, r = rig(cal, x0=-40.0, yaw=yaw, accel=1.0)
        cmd = lambda t: r.drive_velocity((0.6, 0.0), hold_yaw(r, yaw), r.pose()[2])     # noqa: E731
        mon = Mon(r)
        run_for(r, mon, 1.6, cmd)
        x0, t0 = r.pose()[0], d.time
        run_for(r, mon, 1.5, cmd)
        _RATIO[k] = float((r.pose()[0] - x0) / (d.time - t0) / 0.6)
    return _RATIO[k]


# ---------------------------------------------------------------------------------------------------------------------
# per-step monitor and the failure classifier

THRESH = dict(sat_frac=0.97, sat_ms=40.0, slip=0.05, slip_rel=0.05, slip_wheels=2, lift_n=0.5, lift_ms=50.0,
              tip_deg=35.0, t_derate_margin=10.0, hop_frac=0.15)


class Mon:
    """What limited the robot, step by step: wheel torque at its current / back-EMF / thermal limit, slip, lifted
    wheels, belly / hull contacts, tilt, battery, winding temperatures, and the wheels' internal fight.

    Slip is each wheel's own contact slip along its rolling direction: the hub's velocity along it minus spin x the hub's
    height over its lowest roller contact (the instantaneous rolling radius; the barrel rollers' polygon makes the mean
    ~7% under the nominal radius, which is kinematics, not slip). The base is sliding when >= 2 wheels slip by more than
    5 cm/s + 5% of the speed. The fight is the wheel torque in the X4's null space (torques that cancel in the body's
    force and yaw torque): four speed loops on three degrees of freedom settle against each other's small rolling-radius
    differences."""

    def __init__(self, r, feature_geoms=True):
        self.r = r
        m = r.m
        self.dt = m.opt.timestep
        self.base_dof = m.jnt_dofadr[m.joint("base").id]
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in range(m.ngeom)]
        self.belly = {g for g, n in enumerate(names) if n.startswith("belly")}
        base_body = m.body("base").id
        self.hull = {g for g in range(m.ngeom) if m.geom_bodyid[g] == base_body and m.geom_contype[g]
                     and g not in self.belly}
        self.obst = {g for g, n in enumerate(names) if n.startswith("floorfeat") or n == "floor"}
        self.hub = [m.body(f"hub{i}").id for i in range(1, r.nw + 1)]
        self.roller_wheel = {}
        for g, n in enumerate(names):
            if n.startswith("roller") and "_" in n:
                self.roller_wheel[g] = int(n[6:n.index("_")]) - 1
        U, S, Vt = np.linalg.svd(np.asarray(r.kin.J).T)
        self.null = Vt[-1] if r.nw == 4 else None
        self._vel = np.zeros(6)
        self.susp_q = [m.joint(f"susp{i}").qposadr[0] for i in range(1, r.nw + 1)]
        self.susp_max = [float(m.jnt_range[m.joint(f"susp{i}").id][1]) for i in range(1, r.nw + 1)]
        self.reset()

    def reset(self):
        self.t = 0.0
        self.sat = {"torque": 0.0, "rpm": 0.0, "thermal": 0.0}
        self.sat_run = 0.0
        self.sat_run_max = 0.0
        self.slip_max = self.slip_t = 0.0
        self.fight_sum = self.fight_max = 0.0
        self.track_max = 0.0
        self.lift_run = self.lift_max = self.air_run = self.air_max = 0.0
        self.tilt_max = 0.0
        self.flipped = False
        self.belly = self.belly if isinstance(self.belly, set) else set()
        self.belly_hit = self.hull_hit = False
        self.bottom_ms = 0.0
        self.v_min = math.inf
        self.i_peak = 0.0
        self.e_drive = self.e_total = 0.0
        self.tw_max = 0.0
        self.derate_min = 1.0
        self.tau_peak = 0.0
        self.tau_sq = 0.0
        self.w_peak = 0.0
        self.load_min = math.inf
        self.load_frac_low = 0.0      # time with some wheel under 30% of its static share

    def step(self, v_cmd_world=None):
        r, d, m, dt = self.r, self.r.d, self.r.m, self.dt
        self.t += dt
        v_bus = r.battery.v
        any_sat = False
        for k, drv in enumerate(r.drives):
            a = r.a_wheel[k]
            f = float(d.actuator_force[a])
            lo, hi = m.actuator_forcerange[a]
            w = float(d.qvel[r.j_drive_v[k]])
            self.tau_peak = max(self.tau_peak, abs(f))
            self.tau_sq += f * f * dt / r.nw
            self.w_peak = max(self.w_peak, abs(w))
            side = hi if f > 0 else lo
            if abs(side) > 0.05 and abs(f) >= THRESH["sat_frac"] * abs(side):
                any_sat = True
                i_mot, i_brk, i_cap = drv.limit_currents(w, v_bus)
                i_side = i_mot if f * w >= 0 else i_brk
                if i_side < 0.97 * i_cap:
                    cause = "rpm"
                elif drv.thermal.derate() < 0.999:
                    cause = "thermal"
                else:
                    cause = "torque"
                self.sat[cause] += dt / r.nw
            self.tw_max = max(self.tw_max, drv.thermal.t_w)
            self.derate_min = min(self.derate_min, drv.thermal.derate())
            self.e_drive += drv.i_bus * v_bus * dt
        self.sat_run = self.sat_run + dt if any_sat else 0.0
        self.sat_run_max = max(self.sat_run_max, self.sat_run)
        self.e_total += r.i_total * v_bus * dt
        self.v_min = min(self.v_min, v_bus)
        self.i_peak = max(self.i_peak, r.i_total)
        # per-wheel contact slip
        zc = [math.inf] * r.nw
        for k in range(d.ncon):
            c = d.contact[k]
            w = self.roller_wheel.get(c.geom1, self.roller_wheel.get(c.geom2))
            if w is not None and c.pos[2] < zc[w]:
                zc[w] = c.pos[2]
        speed = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
        tol = THRESH["slip"] + THRESH["slip_rel"] * speed
        n_slip, worst = 0, 0.0
        for k in range(r.nw):
            if zc[k] == math.inf:
                continue
            hb = self.hub[k]
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, hb, self._vel, 0)
            axle = d.xmat[hb].reshape(3, 3)[:, 0]
            t = np.cross(axle, (0.0, 0.0, 1.0))
            nt = float(np.linalg.norm(t))
            if nt < 1e-6:
                continue
            h = float(d.xpos[hb][2] - zc[k])
            sl = abs(float(self._vel[3:] @ t) / nt - float(d.qvel[r.j_drive_v[k]]) * h)
            worst = max(worst, sl)
            n_slip += sl > tol
        self.slip_max = max(self.slip_max, worst)
        if n_slip >= THRESH["slip_wheels"]:
            self.slip_t += dt
        if self.null is not None:
            fight = abs(float(np.array([d.actuator_force[a] for a in r.a_wheel]) @ self.null))
            self.fight_sum += fight * dt
            self.fight_max = max(self.fight_max, fight)
        if v_cmd_world is not None:
            self.track_max = max(self.track_max, float(np.linalg.norm(d.sensor("base_linvel").data[:2] - v_cmd_world)))
        t = tilt_deg(d)
        self.tilt_max = max(self.tilt_max, t)
        if t > 60:
            self.flipped = True
        loads = r.wheel_loads()
        self.load_min = min(self.load_min, float(np.min(loads)))
        share = r.mass * 9.81 / r.nw
        if np.min(loads) < 0.3 * share:
            self.load_frac_low += dt
        self.lift_run = self.lift_run + dt if np.any(loads < THRESH["lift_n"]) else 0.0
        self.lift_max = max(self.lift_max, self.lift_run)
        self.air_run = self.air_run + dt if np.all(loads < THRESH["lift_n"]) else 0.0
        self.air_max = max(self.air_max, self.air_run)
        if any(d.qpos[q] > 0.97 * mx for q, mx in zip(self.susp_q, self.susp_max)):
            self.bottom_ms += dt * 1000
        for k in range(d.ncon):
            c = d.contact[k]
            g1, g2 = c.geom1, c.geom2
            if g1 in self.obst or g2 in self.obst:
                o = g2 if g1 in self.obst else g1
                if o in self.belly:
                    self.belly_hit = True
                elif o in self.hull:
                    self.hull_hit = True

    def summary(self) -> dict:
        n = max(self.t, 1e-9)
        return dict(sat_torque_ms=1000 * self.sat["torque"], sat_rpm_ms=1000 * self.sat["rpm"],
                    sat_thermal_ms=1000 * self.sat["thermal"], sat_run_ms=1000 * self.sat_run_max,
                    slip_max=self.slip_max, slip_ms=1000 * self.slip_t, track_max=self.track_max,
                    fight_mean=self.fight_sum / n, fight_max=self.fight_max,
                    lift_ms=1000 * self.lift_max, air_ms=1000 * self.air_max, tilt_max=self.tilt_max,
                    flipped=self.flipped, belly=self.belly_hit, hull=self.hull_hit, bottom_ms=self.bottom_ms,
                    v_min=self.v_min, i_peak=self.i_peak, e_drive_J=self.e_drive, e_total_J=self.e_total,
                    tw_max=self.tw_max, derate_min=self.derate_min, tau_peak=self.tau_peak,
                    tau_rms=math.sqrt(self.tau_sq / n), w_peak=self.w_peak, load_min=self.load_min,
                    low_load_frac=self.load_frac_low / n, t=self.t)


def classify(s: dict, ok: bool) -> str:
    """The binding constraint of a run (for a failed run: why it failed; for a clean one: what came closest)."""
    if s["flipped"] or s["tilt_max"] > THRESH["tip_deg"]:
        return "tipping"
    if s["belly"] or s["hull"]:
        return "grounding" if not ok else "grounding-touch"
    if s["v_min"] < V_STOP:
        return "supply"
    if s["derate_min"] < 0.999 or s["sat_thermal_ms"] > THRESH["sat_ms"]:
        return "thermal"
    sat_t, sat_r = s["sat_torque_ms"], s["sat_rpm_ms"]
    slip = s["slip_ms"] > THRESH["sat_ms"]
    sat = max(sat_t, sat_r) > THRESH["sat_ms"]
    # a failed run that slid: grip (the wheels had more torque than the floor took, or the torque limit and the grip
    # came together); a clean run: the motor limit it sat on, if any, else the slip it came close with
    if not ok and slip:
        return "grip"
    if sat:
        return "torque" if sat_t >= sat_r else "rpm"
    if slip:
        return "grip"
    if s["lift_ms"] > THRESH["lift_ms"]:
        return "tipping-lift"
    if not ok and s["low_load_frac"] > THRESH["hop_frac"]:
        return "wheel-hop"           # loads bouncing (torque jacking on the forks, wheel hop): traction comes and goes
    return "ok" if ok else "controller"


def run_for(r, mon, seconds, cmd, stop=None):
    """Step `seconds`; cmd(t) issues the command each step (and returns the world velocity it asks for, or None);
    stop(t) ends early when it returns True."""
    dt = r.m.opt.timestep
    t = 0.0
    while t < seconds:
        vc = cmd(t)
        r.step()
        mon.step(vc)
        t += dt
        if stop is not None and stop(t):
            break
    return t


def hold_yaw(r, yaw):
    return float(np.clip(6.0 * wrap_angle(yaw - r.pose()[2]), -20.0, 20.0))


# ---------------------------------------------------------------------------------------------------------------------
# C: speed, cruise, straights

DIRS9 = tuple(float(a) for a in np.arange(0.0, 180.1, 22.5))


def job_topspeed(cfg: Cfg, direction: float):
    """Full speed along a body direction (the heading held): the fastest steady speed and what held it there."""
    u = unit(direction)
    yaw = -math.radians(direction)          # world +x is body `direction`
    m, d, r = rig(cfg, x0=-40.0, yaw=yaw, accel=2.0)
    mon = Mon(r)
    hist = []

    def cmd(t):
        r.drive_velocity((20.0, 0.0), hold_yaw(r, yaw), r.pose()[2])
        return None

    def stop(t):
        v = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
        hist.append(v)
        n = int(0.6 / m.opt.timestep)
        return (len(hist) > 3 * n and hist[-1] - hist[-n] < 0.01) or r.pose()[0] > 38.0
    run_for(r, mon, 30.0, cmd, stop)
    v_ss = float(np.mean(hist[-int(0.3 / m.opt.timestep):]))
    s_run = mon.summary()
    plateau = Mon(r)                  # what holds it at the plateau
    run_for(r, plateau, 0.8, cmd, lambda t: r.pose()[0] > 40.0)
    s = plateau.summary()
    return dict(v_top=v_ss, v_peak=max(hist), cause=classify(s, True), v_bus=r.battery.v, run_slip_ms=s_run["slip_ms"],
                run_lift_ms=s_run["lift_ms"], run_tilt_max=s_run["tilt_max"], **s)


CRUISE_V = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)


def job_cruise(cfg: Cfg, direction: float, v: float):
    """Steady cruise: drive power, current and energy per metre over 3 s at speed."""
    yaw = -math.radians(direction)
    vc = v / ratio(cfg, direction)
    m, d, r = rig(cfg, x0=-40.0, yaw=yaw, accel=1.5)
    mon = Mon(r)
    run_for(r, mon, v / 1.5 + 1.0, lambda t: r.drive_velocity((vc, 0.0), hold_yaw(r, yaw), r.pose()[2]))
    v_true = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
    if v_true < 0.93 * v:
        return dict(reached=False, v_true=v_true, cause=classify(mon.summary(), False))
    mon2 = Mon(r)
    x0 = r.pose()[0]
    run_for(r, mon2, 3.0, lambda t: r.drive_velocity((vc, 0.0), hold_yaw(r, yaw), r.pose()[2]))
    dist = r.pose()[0] - x0
    s = mon2.summary()
    return dict(reached=True, v_true=dist / 3.0, p_drive_W=s["e_drive_J"] / 3.0, p_total_W=s["e_total_J"] / 3.0,
                wh_per_km_drive=s["e_drive_J"] / 3600 / max(dist, 1e-6) * 1000, tau_rms=s["tau_rms"],
                tw_max=s["tw_max"], cause=classify(s, True), **{f"s_{k}": v for k, v in s.items()})


STRAIGHT_L = (2.0, 5.0, 10.0, 15.2)
STRAIGHT_CAPS = (1.0, 1.5, 2.0, 2.5, 3.0, 3.5, math.inf)
STRAIGHT_A = (2.0, 3.0, 4.0, 5.0)


def job_straight(cfg: Cfg, direction: float, L: float, v_cap: float, a: float):
    """Rest to rest over L metres along a body direction at speed cap v_cap and acceleration a (jerk-limited 60 m/s^3):
    the time to settle within 2 cm and 3 cm/s, peak speed, overshoot."""
    yaw = -math.radians(direction)
    m, d, r = rig(cfg, x0=-8.0, yaw=yaw, accel=a, jerk=60.0)
    r.ctl.max_speed = v_cap
    mon = Mon(r)
    target = (-8.0 + L, 0.0, yaw)
    peak = [0.0]
    settled = [None]

    def cmd(t):
        pose = r.pose()
        r.drive_to(pose, target)
        v = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
        peak[0] = max(peak[0], v)
        if np.hypot(pose[0] - target[0], pose[1] - target[1]) < 0.02 and v < 0.03:
            if settled[0] is None:
                settled[0] = t
        else:
            settled[0] = None
        return None
    run_for(r, mon, L / max(min(v_cap, 4.5), 0.5) + 8.0, cmd, lambda t: settled[0] is not None and t - settled[0] > 0.2)
    s = mon.summary()
    over = max(0.0, r.pose()[0] - target[0])
    return dict(t_settle=settled[0], v_peak=peak[0], overshoot=over, cause=classify(s, settled[0] is not None), **s)


# ---------------------------------------------------------------------------------------------------------------------
# A: acceleration, braking, reversals, direction changes, panic stops

def _ramp(cfg, direction, a, v0, v1, jerk=200.0):
    """Body-direction velocity ramp v0 -> v1 at acceleration a (jerk-limited), on the wheel speeds directly (the
    controller's limits out of the way). Clean if the base's own acceleration (its true speed's time from 20% to 80% of
    the change) is at least 90% of a, it ends within 3 cm/s + 3% of v1, without a wheel lifted > 50 ms, heading drift >
    3 deg or a flip. Slip (>= 2 wheels sliding) names the cause of a failure; it doesn't fail a run that accelerates."""
    yaw = -math.radians(direction)
    u = unit(0.0)                         # world +x = body `direction`
    kr = 1.0 / ratio(cfg, direction)      # wheel command per unit of true speed
    m, d, r = rig(cfg, x0=-30.0, yaw=yaw)
    dt = m.opt.timestep
    mon = Mon(r)
    # reach v0 gently first
    if v0 > 0:
        run_for(r, mon, v0 / 1.5 + 1.0, lambda t: r.drive_velocity((v0 * kr, 0.0), hold_yaw(r, yaw), r.pose()[2],
                                                                    a_cap=1.5))
        mon = Mon(r)
    v, acc = v0, 0.0
    sgn = 1.0 if v1 >= v0 else -1.0
    e_track = 0.0
    t_end = None
    t = 0.0
    t_reach = t20 = t80 = None
    tol_v = 0.03 + 0.03 * max(abs(v0), abs(v1))
    yaw0 = r.pose()[2]
    r.ctl.v_cmd = np.array([v0 * kr, 0.0])
    while t_end is None or t < t_end + 0.3 or (t80 is None and t < t_end + 1.5):
        left = abs(v1 - v)
        if acc > 0 and left <= acc * acc / (2 * jerk) + 1e-9:
            acc = max(0.0, acc - jerk * dt)
        else:
            acc = min(a, acc + jerk * dt)
        v = v + sgn * acc * dt
        if (v - v1) * sgn >= 0:
            v = v1
            if t_end is None:
                t_end = t
        vb = world_to_body(v * kr * u, r.pose()[2])
        r.wheel_cmd = r.kin.wheel_speeds([vb[0], vb[1], hold_yaw(r, yaw)])
        r.a_body_cmd = world_to_body(sgn * acc * u, r.pose()[2])
        r.step()
        mon.step(v * u)
        t += dt
        e_track = max(e_track, float(np.linalg.norm(d.sensor("base_linvel").data[:2] - v * u)))
        vt = float(d.sensor("base_linvel").data[:2] @ u)
        frac = (vt - v0) / (v1 - v0)
        if frac >= 0.2 and t20 is None:
            t20 = t
        if frac >= 0.8 and t80 is None:
            t80 = t
        near = float(np.linalg.norm(d.sensor("base_linvel").data[:2] - v1 * u)) < tol_v
        if near and t_reach is None:
            t_reach = t
        elif not near and t_end is not None and t > t_end + 0.15:
            t_reach = None                 # it got there but didn't stay
        if t > 20:
            break
    v_end = d.sensor("base_linvel").data[:2]
    err = float(np.linalg.norm(v_end - v1 * u))
    drift = math.degrees(abs(wrap_angle(r.pose()[2] - yaw0)))
    s = mon.summary()
    a_true = 0.6 * abs(v1 - v0) / (t80 - t20) if t20 is not None and t80 is not None and t80 > t20 else 0.0
    ok = (a_true >= 0.9 * a and err < tol_v and s["lift_ms"] < THRESH["lift_ms"] and drift < 3.0
          and not s["flipped"])
    return ok, dict(e_track=e_track, err=err, drift=drift, a_true=a_true, t_reach=t_reach, **s)


ACCEL_CASES = {"start": (0.0, 1.5), "brake": (1.5, 0.0), "at speed": (1.5, 2.5), "brake fast": (2.5, 0.5)}


def job_accel(cfg: Cfg, direction: float, case: str):
    """Highest clean acceleration (bisection 0.5-14 m/s^2, 8 steps) for a case of ACCEL_CASES; the cause at the first
    level that failed."""
    v0, v1 = ACCEL_CASES[case]
    lo, hi = 0.25, 14.0
    ok_lo, fail = None, None
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        ok, s = _ramp(cfg, direction, mid, v0, v1)
        if ok:
            lo, ok_lo = mid, s
        else:
            hi, fail = mid, s
    return dict(a_max=lo, a_fail=hi, cause=classify(fail, False) if fail else "beyond 14",
                a_true_fail=fail["a_true"] if fail else None,
                at_max={k: ok_lo[k] for k in ("tilt_max", "slip_max", "slip_ms", "tau_peak", "lift_ms", "i_peak",
                                              "v_min", "a_true")}
                if ok_lo else None,
                at_fail={k: fail[k] for k in ("tilt_max", "slip_max", "slip_ms", "sat_torque_ms", "sat_rpm_ms",
                                              "lift_ms", "belly", "flipped")} if fail else None)


def job_reverse(cfg: Cfg, direction: float, v: float, a: float):
    """+v to -v along a body direction through the controller at acceleration a: time, slip, lift, tilt."""
    yaw = -math.radians(direction)
    m, d, r = rig(cfg, x0=-10.0, yaw=yaw, accel=a, jerk=200.0)
    kf, kb = 1.0 / ratio(cfg, direction), 1.0 / ratio(cfg, direction + 180.0)
    mon = Mon(r)
    run_for(r, mon, v / a + 1.0, lambda t: r.drive_velocity((v * kf, 0.0), hold_yaw(r, yaw), r.pose()[2]))
    mon = Mon(r)
    done = [None]

    def cmd(t):
        r.drive_velocity((-v * kb, 0.0), hold_yaw(r, yaw), r.pose()[2])
        vx = float(d.sensor("base_linvel").data[0])
        if done[0] is None and vx < -0.95 * v:
            done[0] = t
        return np.array([float(r.ctl.v_cmd[0]), float(r.ctl.v_cmd[1])]) / kb
    run_for(r, mon, 2 * v / a + 2.0, cmd, lambda t: done[0] is not None and t > done[0] + 0.3)
    s = mon.summary()
    ok = done[0] is not None and s["track_max"] < 0.25 + 0.05 * v and s["lift_ms"] < THRESH["lift_ms"]
    return dict(t_rev=done[0], t_ideal=2 * v / a, ok=ok, cause=classify(s, ok), **s)


def job_turn(cfg: Cfg, v: float, turn_deg: float, a: float):
    """At speed v heading world +x (body forward), the request turns turn_deg to a new world direction at the same speed
    (heading held): time to the new velocity, the path's bulge, slip, lift."""
    m, d, r = rig(cfg, x0=-10.0, yaw=0.0, accel=a, jerk=200.0)
    k0, k1 = 1.0 / ratio(cfg, 0.0), 1.0 / ratio(cfg, turn_deg)
    mon = Mon(r)
    run_for(r, mon, v / a + 1.0, lambda t: r.drive_velocity((v * k0, 0.0), hold_yaw(r, 0.0), r.pose()[2]))
    target = v * unit(turn_deg)
    mon = Mon(r)
    done = [None]

    def cmd(t):
        r.drive_velocity(target * k1, hold_yaw(r, 0.0), r.pose()[2])
        if done[0] is None and np.linalg.norm(d.sensor("base_linvel").data[:2] - target) < 0.05 * v + 0.03:
            done[0] = t
        return r.ctl.v_cmd / k1
    run_for(r, mon, 2 * v / a + 2.0, cmd, lambda t: done[0] is not None and t > done[0] + 0.3)
    s = mon.summary()
    ok = done[0] is not None and s["track_max"] < 0.25 + 0.05 * v and s["lift_ms"] < THRESH["lift_ms"]
    dv = float(np.linalg.norm(target - np.array([v, 0.0])))
    return dict(t_turn=done[0], t_ideal=dv / a, ok=ok, cause=classify(s, ok), **s)


def job_panic(cfg: Cfg, direction: float, v: float):
    """Unramped stop: at speed v the wheel speeds are commanded to zero at once (an e-stop that holds the motors):
    stopping distance, tilt, lift, flip."""
    yaw = -math.radians(direction)
    m, d, r = rig(cfg, x0=-30.0, yaw=yaw, accel=2.0)
    vc = v / ratio(cfg, direction)
    mon = Mon(r)
    run_for(r, mon, v / 2.0 + 1.0, lambda t: r.drive_velocity((vc, 0.0), hold_yaw(r, yaw), r.pose()[2]))
    v_true = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
    if v_true < 0.93 * v:
        return dict(reached=False, v_true=v_true)
    mon = Mon(r)
    x0 = r.pose()[0]
    r.ctl.reset()

    def cmd(t):
        r.wheel_cmd = np.zeros(r.nw)
        r.a_body_cmd = None
    run_for(r, mon, 2.5, cmd)
    s = mon.summary()
    return dict(reached=True, v_true=v_true, stop_m=r.pose()[0] - x0, cause=classify(s, not s["flipped"]), **s)


# ---------------------------------------------------------------------------------------------------------------------
# R: rotation

def job_spin(cfg: Cfg, yaw_accel: float):
    """Spin in place at the most the wheels allow, yaw acceleration yaw_accel: top yaw rate, time to reach 95% of it."""
    m, d, r = rig(cfg, accel=3.0)
    r.ctl.max_yaw_accel = yaw_accel
    mon = Mon(r)
    hist = []
    wz = m.jnt_dofadr[m.joint("base").id] + 5

    def cmd(t):
        r.drive_velocity((0.0, 0.0), 200.0, r.pose()[2])
        hist.append(float(d.qvel[wz]))
    run_for(r, mon, 6.0, cmd)
    w_top = float(np.mean(hist[-int(0.5 / m.opt.timestep):]))
    t95 = next((k * m.opt.timestep for k, w in enumerate(hist) if w > 0.95 * w_top), None)
    s = mon.summary()
    drift = float(np.linalg.norm(r.pose()[:2]))
    return dict(w_top=w_top, t95=t95, drift_m=drift, cause=classify(s, True), **s)


def job_combo(cfg: Cfg, v: float, w: float):
    """Translate at v (world +x) while turning at w: does the base hold both (world velocity within 10%, yaw rate within
    10%) for 3 s, and what limits it."""
    m, d, r = rig(cfg, x0=-30.0, accel=3.0)
    r.ctl.max_yaw_accel = 30.0
    kr = 2.0 / (ratio(cfg, 0.0) + ratio(cfg, 45.0))      # the heading turns through every direction
    mon = Mon(r)
    wz = m.jnt_dofadr[m.joint("base").id] + 5
    run_for(r, mon, max(v / 3.0, w / 30.0) + 1.0, lambda t: r.drive_velocity((v * kr, 0.0), w, r.pose()[2]))
    mon = Mon(r)
    errs, werrs = [], []

    def cmd(t):
        r.drive_velocity((v * kr, 0.0), w, r.pose()[2])
        errs.append(float(np.linalg.norm(d.sensor("base_linvel").data[:2] - np.array([v, 0.0]))))
        werrs.append(abs(float(d.qvel[wz]) - w))
        return np.array([v, 0.0])
    run_for(r, mon, 3.0, cmd)
    e, ew = float(np.percentile(errs, 95)), float(np.percentile(werrs, 95))
    ok = e < 0.1 * v + 0.05 and ew < 0.1 * w + 0.1
    s = mon.summary()
    capped = bool(np.linalg.norm(r.ctl.v_cmd) < 0.98 * v * kr or abs(r.ctl.w_cmd) < 0.98 * w)
    return dict(ok=ok, e_v95=e, e_w95=ew, wheel_capped=capped,
                cause=classify(s, ok) if ok or not capped else "rpm", **s)


# ---------------------------------------------------------------------------------------------------------------------
# S and F: steps, bumps, house transitions

EDGE = 0.6
PLATFORM = 0.8
STEP_LADDER = tuple(float(h) for h in np.arange(5.0, 70.1, 2.5))       # mm (to the wheel's radius)
BUMP_LADDER = (6.0, 10.0, 13.0, 16.0, 19.0, 22.0, 25.0, 28.0, 32.0, 35.0, 38.0, 44.0, 50.0)
TRI_W = 0.076
DIRS13 = tuple(float(a) for a in range(0, 181, 15))
CROSS_SPEEDS = (0.25, 0.5, 1.0, 2.0, 3.0)


def obstacle(name: str, edge_mu: float = 0.85):
    """(features, the far edge's x, where the run ends) for an obstacle name (as base_trade.obstacle, plus a 20 mm rug).
    edge_mu: the rollers' grip on a step (0.85 wood / painted, est; 0.5 a slick edge: the sensitivity)."""
    if name.startswith("step "):
        h = float(name.split()[1]) / 1000
        return (Threshold(EDGE + PLATFORM / 2, 0.0, length=3.0, height=h, top=PLATFORM, ramp=0.0005, mu=edge_mu),), \
            EDGE + PLATFORM, EDGE + PLATFORM + 0.45
    if name.startswith("bump "):
        h = float(name.split()[1]) / 1000
        return (Threshold(EDGE + TRI_W / 2, 0.0, length=3.0, height=h, top=0.002, ramp=TRI_W / 2 - 0.001, mu=0.8),), \
            EDGE + TRI_W, EDGE + TRI_W + 0.55
    if name == "rug 20 mm":
        return (Carpet(EDGE, -1.5, EDGE + 2.0, 1.5, thickness=0.020, mu=0.85, crr=0.08, softness=0.015),), EDGE + 0.05, 1.25
    f = {**OBSTACLES, **floor_mod.HOUSE_OBSTACLES}[name]
    return (f,), EDGE + 0.05, 1.25


HOUSE_NAMES = tuple(OBSTACLES) + tuple(floor_mod.HOUSE_OBSTACLES) + ("rug 20 mm",)


def cross(cfg: Cfg, name: str, direction: float, v: float, a: float = 2.0, x_start: float = -0.15,
          from_rest_at_edge: bool = False, edge_mu: float = 0.85):
    """Drive across an obstacle at v in world +x with the heading held so that this is body direction `direction`."""
    feats, x_far, x_end = obstacle(name, edge_mu)
    yaw = -math.radians(direction)
    if from_rest_at_edge:
        # the robot's leading wheel just touching the edge: the outermost point of the wheels along +x
        reach = max(0.146 * abs(math.cos(math.radians(ang) + yaw)) for ang in (45, 135, 225, 315)) + 0.07
        x_start = EDGE - reach - 0.002
    m, d, r = rig(cfg, x0=x_start, yaw=yaw, features=feats, accel=a, jerk=0.0)
    vc = v / ratio(cfg, direction)
    mon = Mon(r)
    x_max = [-1.0]
    flipped = [False]

    def cmd(t):
        r.drive_velocity((vc, 0.0), hold_yaw(r, yaw), r.pose()[2])
        x_max[0] = max(x_max[0], float(r.pose()[0]))
        if tilt_deg(d) > 45:
            flipped[0] = True

    run_for(r, mon, 1.0 + (x_end - x_start + 0.3) / v * 2.5, cmd, lambda t: r.pose()[0] > x_end or flipped[0])
    crossed = bool(r.pose()[0] > x_end and not flipped[0])
    s = mon.summary()
    where = "across" if crossed else ("flipped" if flipped[0] else "before the edge" if x_max[0] < EDGE + 0.05
                                      else "climbing" if x_max[0] < EDGE + 0.25 else "on it" if x_max[0] < x_far + 0.2
                                      else "coming off")
    return crossed, dict(where=where, x_max=x_max[0], cause=classify(s, crossed), **s)


def job_ladder(cfg: Cfg, kind: str, direction: float, v: float, rest: bool = False, edge_mu: float = 0.85):
    """Heights up a ladder (steps or bumps) until two in a row fail: the tallest crossed and why the first failure
    failed. Steps go coarse to fine: 5 mm rungs, then the 2.5 mm between the best and the first failure."""
    ladder = STEP_LADDER[::2] if kind == "step" else BUMP_LADDER
    best, fails, rows, first_fail = 0.0, 0, [], None

    def trial(h):
        ok, s = cross(cfg, f"{kind} {h:g} mm", direction, v, from_rest_at_edge=rest, edge_mu=edge_mu)
        rows.append(dict(h=h, ok=ok, where=s["where"], cause=s["cause"], tilt=s["tilt_max"], lift_ms=s["lift_ms"],
                         belly=s["belly"], hull=s["hull"], tau_peak=s["tau_peak"], slip_ms=s["slip_ms"]))
        return ok
    for h in ladder:
        if trial(h):
            best, fails = h, 0
        else:
            fails += 1
            if first_fail is None:
                first_fail = rows[-1]
            if fails >= 2:
                break
    if kind == "step" and first_fail is not None and first_fail["h"] - best > 2.6:
        if trial(best + 2.5):
            best += 2.5
        else:
            first_fail = rows[-1]
    return dict(max_mm=best, first_fail=first_fail, rows=rows)


def job_house(cfg: Cfg, name: str, direction: float, v: float):
    ok, s = cross(cfg, name, direction, v)
    return dict(crossed=ok, **s)


# ---------------------------------------------------------------------------------------------------------------------
# I: slopes (tilted gravity) and ramps (Incline)

SLOPE_LADDER = tuple(float(a) for a in range(2, 41, 2))
SLOPE_DIRS = (0.0, 45.0, 90.0)
SLOPE_TESTS = ("start", "hold", "down_brake", "vmax_up")


def slope_trial(cfg: Cfg, angle: float, direction: float, test: str):
    """One slope trial; world +x is uphill and body `direction`. start: from rest up to 1 m/s at 1 m/s^2, held 2 s;
    hold: zero command for 30 s (creep, heat); down_brake: down at 1 m/s then a 1.5 m/s^2 stop; vmax_up: the top speed
    uphill. Returns (ok, metrics)."""
    yaw = -math.radians(direction)
    m, d, r = rig(cfg, x0=-30.0 if test != "down_brake" else 30.0, yaw=yaw, slope_deg=angle, accel=1.0, jerk=0.0)
    mon = Mon(r)
    x0 = r.pose()[0]
    kr = 1.0 / ratio(cfg, direction)
    if test == "start":
        run_for(r, mon, 3.0, lambda t: r.drive_velocity((kr, 0.0), hold_yaw(r, yaw), r.pose()[2]))
        v = float(d.sensor("base_linvel").data[0])
        s = mon.summary()
        # climbing: up to >= 90% of 1 m/s in 3 s and held (creep under the climbing load is physics, not failure)
        ok = v >= 0.9 and s["lift_ms"] < THRESH["lift_ms"] and not s["flipped"]
        mon2 = Mon(r)
        x1 = r.pose()[0]
        run_for(r, mon2, 2.0, lambda t: r.drive_velocity((kr, 0.0), hold_yaw(r, yaw), r.pose()[2]))
        s2 = mon2.summary()
        v = (r.pose()[0] - x1) / 2.0
        ok = ok and v >= 0.9 and s2["slip_ms"] < 200.0 and not s2["flipped"]
        s = {**s, "v_end": v, "p_climb_W": s2["e_drive_J"] / 2.0, "tw_max": max(s["tw_max"], s2["tw_max"])}
    elif test == "hold":
        run_for(r, mon, 30.0, lambda t: r.drive_velocity((0.0, 0.0), hold_yaw(r, yaw), r.pose()[2]))
        creep = float(np.linalg.norm(r.pose()[:2] - np.array([x0, 0.0])))
        s = mon.summary()
        ok = creep < 0.05 and not s["flipped"] and s["derate_min"] > 0.999
        s = {**s, "creep_m": creep}
    elif test == "down_brake":
        r.ctl.max_accel = 1.5
        kd = 1.0 / ratio(cfg, direction + 180.0)
        run_for(r, mon, 2.0, lambda t: r.drive_velocity((-kd, 0.0), hold_yaw(r, yaw), r.pose()[2], a_cap=1.0))
        v0 = float(d.sensor("base_linvel").data[0])
        xa = r.pose()[0]
        mon = Mon(r)
        run_for(r, mon, 4.0, lambda t: r.drive_velocity((0.0, 0.0), hold_yaw(r, yaw), r.pose()[2], a_cap=1.5))
        dist = xa - r.pose()[0]
        s = mon.summary()
        ideal = v0 * v0 / (2 * 1.5)
        v_end = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
        ok = v_end < 0.03 and dist < ideal * 1.3 + 0.05 and not s["flipped"] and s["lift_ms"] < THRESH["lift_ms"]
        s = {**s, "v0": v0, "stop_m": dist, "ideal_m": ideal, "v_end": v_end}
    else:   # vmax_up
        r.ctl.max_accel = 1.5
        hist = []

        def cmd(t):
            r.drive_velocity((20.0, 0.0), hold_yaw(r, yaw), r.pose()[2])
            hist.append(float(d.sensor("base_linvel").data[0]))
        run_for(r, mon, 9.0, cmd)
        v = float(np.mean(hist[-int(0.5 / m.opt.timestep):]))
        s = mon.summary()
        ok = v > 0.5 and not s["flipped"]
        s = {**s, "v_top": v}
    return ok, {**s, "cause": classify(s, ok)}


def job_slope(cfg: Cfg, direction: float, test: str):
    best, fails, rows, first_fail = 0.0, 0, [], None
    for ang in SLOPE_LADDER:
        ok, s = slope_trial(cfg, ang, direction, test)
        keep = ("cause", "tilt_max", "slip_ms", "sat_torque_ms", "sat_rpm_ms", "lift_ms", "tw_max", "tau_peak",
                "creep_m", "stop_m", "ideal_m", "v_top", "v_end", "p_climb_W", "i_peak", "v_min")
        rows.append(dict(angle=ang, ok=ok, **{k: s[k] for k in keep if k in s}))
        if ok:
            best, fails = ang, 0
        else:
            fails += 1
            first_fail = first_fail or rows[-1]
            if fails >= 2:
                break
    return dict(max_deg=best, first_fail=first_fail, rows=rows)


RAMP_ANGLES = (5.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0)


def job_ramp(cfg: Cfg, angle: float, direction: float, v: float):
    """Over a real ramp (floor.Incline, 120 mm up, 1 m top, the same angle down): the foot and the crest."""
    inc = Incline(EDGE, 0.0, angle_up=angle, height=0.12, top=1.0, angle_down=angle, length=3.0, mu=0.85)
    up, dn = inc.runs
    x_end = EDGE + up + 1.0 + dn + 0.45
    yaw = -math.radians(direction)
    m, d, r = rig(cfg, x0=-0.3, yaw=yaw, features=(inc,), accel=2.0)
    mon = Mon(r)
    run_for(r, mon, 2.0 + (x_end + 0.3) / v * 2.5, lambda t: r.drive_velocity((v, 0.0), hold_yaw(r, yaw), r.pose()[2]),
            lambda t: r.pose()[0] > x_end or mon.flipped)
    ok = bool(r.pose()[0] > x_end and not mon.flipped)
    s = mon.summary()
    return dict(crossed=ok, x_max=float(r.pose()[0]), cause=classify(s, ok), **s)


# ---------------------------------------------------------------------------------------------------------------------
# U: uneven loading

def job_side_bump(cfg: Cfg, h_mm: float, v: float):
    """A bump under the left wheels only (front-left, then rear-left): load loss, lift, heading kick, bottoming."""
    yw = 0.146 * math.sin(math.radians(45))
    f = Threshold(EDGE + TRI_W / 2, yw, length=0.12, height=h_mm / 1000, top=0.002, ramp=TRI_W / 2 - 0.001, mu=0.8)
    m, d, r = rig(cfg, x0=-0.3, features=(f,), accel=2.0)
    mon = Mon(r)
    x_end = EDGE + 0.6
    run_for(r, mon, 2.0 + x_end / v * 2.5, lambda t: r.drive_velocity((v, 0.0), hold_yaw(r, 0.0), r.pose()[2]),
            lambda t: r.pose()[0] > x_end)
    s = mon.summary()
    kick = abs(math.degrees(wrap_angle(r.pose()[2])))
    lateral = abs(float(r.pose()[1]))
    ok = r.pose()[0] > x_end and not s["flipped"]
    return dict(crossed=bool(ok), yaw_kick_deg=kick, lateral_m=lateral, cause=classify(s, ok), **s)


def job_wheel_offset(cfg: Cfg, wheel: int, dz_mm: float, direction: float, a: float):
    """One wheel's floor dz_mm lower (+) or higher (-) (a floor dip or a build error): static loads, then a hard
    acceleration and braking along a body direction."""
    off = [0.0] * 4
    off[wheel] = dz_mm / 1000
    yaw = -math.radians(direction)
    m, d, r = rig(cfg, x0=-10.0, yaw=yaw, accel=a, jerk=200.0, wheel_z_offset=tuple(off))
    loads0 = r.wheel_loads().tolist()
    mon = Mon(r)
    run_for(r, mon, 1.5 / a + 0.5, lambda t: r.drive_velocity((1.5, 0.0), hold_yaw(r, yaw), r.pose()[2]))
    run_for(r, mon, 1.5 / a + 1.0, lambda t: r.drive_velocity((0.0, 0.0), hold_yaw(r, yaw), r.pose()[2]))
    s = mon.summary()
    ok = s["slip_ms"] < THRESH["sat_ms"] and s["lift_ms"] < THRESH["lift_ms"]
    return dict(static_loads=loads0, ok=ok, cause=classify(s, ok), **s)


# ---------------------------------------------------------------------------------------------------------------------
# T: thermal

def job_thermal(cfg: Cfg, scenario: str, level: float, seconds: float = 60.0):
    """Sustained duty: climb (a level-deg slope at 0.5 m/s), bursts (rest - 2 m/s - rest at level m/s^2, repeated),
    spin (level rad/s in place), push (against a 60 mm step at level m/s, it can't climb: stalled). Winding peak, time
    to the derate's onset, the derate."""
    t_derate = [None]
    if scenario == "climb":
        m, d, r = rig(cfg, x0=-40.0, slope_deg=level, accel=1.0)
        cmd = lambda t: r.drive_velocity((0.5, 0.0), hold_yaw(r, 0.0), r.pose()[2])       # noqa: E731
    elif scenario == "bursts":
        m, d, r = rig(cfg, x0=-3.0, accel=level, jerk=0.0)
        period = 2 * 2.0 / level + 1.0

        def cmd(t):
            ph = t % period
            sgn = 1.0 if int(t // period) % 2 == 0 else -1.0
            r.drive_velocity((sgn * 2.0 if ph < period / 2 else 0.0, 0.0), hold_yaw(r, 0.0), r.pose()[2])
    elif scenario == "spin":
        m, d, r = rig(cfg, accel=3.0)
        r.ctl.max_yaw_accel = 30.0
        cmd = lambda t: r.drive_velocity((0.0, 0.0), level, r.pose()[2])       # noqa: E731
    elif scenario == "push":
        feats = (Threshold(EDGE + PLATFORM / 2, 0.0, length=3.0, height=0.06, top=PLATFORM, ramp=0.0005, mu=0.85),)
        m, d, r = rig(cfg, x0=EDGE - 0.32, features=feats, accel=2.0)
        cmd = lambda t: r.drive_velocity((level, 0.0), hold_yaw(r, 0.0), r.pose()[2])     # noqa: E731
    else:
        raise ValueError(scenario)
    mon = Mon(r)
    tw = []

    def c(t):
        cmd(t)
        if t_derate[0] is None and max(dv.thermal.t_w for dv in r.drives) >= r.drives[0].s.thermal.t_max - 10.0:
            t_derate[0] = t
        if int(t * 1000) % 1000 == 0:
            tw.append(round(max(dv.thermal.t_w for dv in r.drives), 2))
    run_for(r, mon, seconds, c)
    s = mon.summary()
    # the steady state the winding heads for at this duty (two-node model: losses x (r_wc + r_ca) over ambient)
    p_loss = float(np.mean([dv.p_loss for dv in r.drives]))
    th = r.drives[0].s.thermal
    return dict(t_derate=t_derate[0], tw_trace=tw, p_loss_end=p_loss, tw_ss_est=25.0 + p_loss * (th.r_wc + th.r_ca),
                cause=classify(s, t_derate[0] is None), **s)


# ---------------------------------------------------------------------------------------------------------------------
# P: suspension sensitivity

POD_RATES = (1500.0, 2500.0, 4000.0, 6000.0, 9000.0)
POD_ZETAS = (0.15, 0.3, 0.6, 1.0)
POD_PRELOADS = ("low", "mid")
RIGID_RATE = 100000.0
PIVOT_ZS = (0.043, 0.03, 0.02, 0.01, 0.0)      # m over the axle (cad/v10: 0.043)


def job_susp(cfg: Cfg, scenario: str):
    """One suspension scenario for a pod setting (cfg.pod_*): steps / rug / side bump / panic stop / bumpy cruise."""
    if scenario.startswith("step"):
        _, h, v, dirn = scenario.split()
        ok, s = cross(cfg, f"step {h} mm", float(dirn), float(v))
        return dict(ok=ok, **s)
    if scenario.startswith("rug"):
        _, v, dirn = scenario.split()
        ok, s = cross(cfg, "plush rug 16 mm", float(dirn), float(v))
        return dict(ok=ok, **s)
    if scenario.startswith("side"):
        _, h, v = scenario.split()
        return job_side_bump(cfg, float(h), float(v))
    if scenario.startswith("accel"):
        _, case, dirn = scenario.split()
        return job_accel(cfg, float(dirn), case)
    if scenario.startswith("panic"):
        _, v = scenario.split()
        return job_panic(cfg, 0.0, float(v))
    if scenario.startswith("bumpy"):
        _, v = scenario.split()
        rng = np.random.default_rng(7)
        feats = tuple(Threshold(0.5 + 0.7 * k, float(rng.uniform(-0.3, 0.3)), yaw_deg=float(rng.uniform(-40, 40)),
                                length=0.5, height=float(rng.uniform(0.003, 0.009)), top=0.01, ramp=0.015, mu=0.8)
                      for k in range(14))
        m, d, r = rig(cfg, x0=-0.5, features=feats, accel=2.0)
        mon = Mon(r)
        run_for(r, mon, 1.0 + 10.5 / float(v) * 1.5, lambda t: r.drive_velocity((float(v), 0.0), hold_yaw(r, 0.0),
                                                                                r.pose()[2]),
                lambda t: r.pose()[0] > 10.3)
        s = mon.summary()
        ok = r.pose()[0] > 10.3 and not s["flipped"]
        return dict(ok=bool(ok), lateral_m=abs(float(r.pose()[1])), cause=classify(s, ok), **s)
    raise ValueError(scenario)


SUSP_SCENARIOS = ("accel start 0", "accel start 90", "accel brake 0", "step 12 1 0", "step 20 1 0", "step 20 2 0", "step 30 1 0", "step 20 1 45", "step 20 1 90",
                  "rug 2 45", "side 20 1", "side 30 2", "panic 2", "bumpy 1", "bumpy 2")


# ---------------------------------------------------------------------------------------------------------------------
# parts: job lists

def _cfgs(a):
    out = []
    for mo in a.motors:
        for ms in a.masses:
            for su in a.surfaces:
                for so in a.socs:
                    for lp in a.loops:
                        out.append(Cfg(motor=mo, mass=ms, soc=so, surface=su, loop=lp))
    return out


COARSE_DIRS = (0.0, 45.0, 90.0, 135.0, 180.0)


def jobs_for(part: str, a) -> list:
    """--coarse (phase 4, the alternatives): five directions, fewer speeds."""
    J = []
    cfgs = _cfgs(a)
    dirs9 = COARSE_DIRS if a.coarse else DIRS9
    dirs13 = COARSE_DIRS if a.coarse else DIRS13
    cross_speeds = (0.25, 1.0, 2.0) if a.coarse else CROSS_SPEEDS
    if part == "C":
        for c in cfgs:
            J += [("topspeed", c, (dr,)) for dr in dirs9]
            J += [("cruise", c, (dr, v)) for dr in (0.0, 45.0) for v in CRUISE_V]
            if c.soc == a.socs[0]:
                if a.coarse:
                    J += [("straight", c, (0.0, L, cap, acc)) for L in (5.0, 15.2) for cap in (2.0, 3.0, math.inf)
                          for acc in (3.0, 5.0)]
                else:
                    J += [("straight", c, (0.0, L, cap, acc)) for L in STRAIGHT_L for cap in STRAIGHT_CAPS
                          for acc in STRAIGHT_A]
    elif part == "A":
        for c in cfgs:
            J += [("accel", c, (dr, case)) for dr in dirs9 for case in ACCEL_CASES]
            J += [("reverse", c, (dr, v, acc)) for dr in (0.0, 45.0, 90.0) for v in (1.0, 2.0, 3.0) for acc in (3.0, 5.0)]
            J += [("turn", c, (v, td, acc)) for v in (1.0, 2.0, 3.0) for td in (45.0, 90.0, 135.0) for acc in (3.0, 5.0)]
            J += [("panic", c, (dr, v)) for dr in (0.0, 45.0, 90.0, 180.0) for v in (1.0, 2.0, 3.0)]
    elif part == "R":
        for c in cfgs:
            J += [("spin", c, (ya,)) for ya in (10.0, 30.0, 100.0)]
            J += [("combo", c, (v, w)) for v in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0) for w in (1.0, 2.0, 4.0, 6.0, 9.0, 12.0)]
    elif part == "S":
        for c in cfgs:
            if c.surface != "floor":
                continue
            J += [("ladder", c, ("step", dr, v)) for dr in dirs13 for v in cross_speeds]
            J += [("ladder", c, ("bump", dr, v)) for dr in dirs13 for v in ((1.0, 2.0) if a.coarse else (0.5, 1.0, 2.0, 3.0))]
            J += [("ladder", c, ("step", dr, 0.5, True)) for dr in dirs13]
            if c.mass in ("10", "20") and c.soc == a.socs[0]:    # a slick edge (mu 0.5): the edge-grip sensitivity
                J += [("ladder", c, ("step", dr, v, False, 0.5)) for dr in (0.0, 45.0, 90.0) for v in (0.25, 1.0)]
    elif part == "F":
        for c in cfgs:
            if c.surface != "floor":
                continue
            J += [("house", c, (n, dr, v)) for n in HOUSE_NAMES for dr in dirs13
                  for v in ((1.0, 2.0) if a.coarse else (0.5, 1.0, 2.0, 3.0))]
    elif part == "I":
        for c in cfgs:
            if c.surface == "dusty":
                continue
            J += [("slope", c, (dr, t)) for dr in SLOPE_DIRS for t in SLOPE_TESTS]
            if c.surface == "floor":
                J += [("ramp", c, (ang, dr, v)) for ang in RAMP_ANGLES for dr in SLOPE_DIRS for v in (0.5, 1.0, 2.0)]
    elif part == "U":
        for c in cfgs:
            if c.surface != "floor":
                continue
            J += [("side_bump", c, (h, v)) for h in (10.0, 20.0, 30.0) for v in (0.5, 1.0, 2.0, 3.0)]
            J += [("wheel_offset", c, (w, dz, dr, acc)) for w in (0,) for dz in (-10.0, -5.0, 5.0, 10.0, 15.0)
                  for dr in (0.0, 45.0, 90.0) for acc in (3.0, 5.0)]
    elif part == "T":
        motors = list(a.motors)
        if "AK40-10" in motors and not a.no_thermal_variants:
            motors += ["AK40-10@th0.5", "AK40-10@th1.5"]
        for mo in motors:
            for ms in a.masses:
                for lp in a.loops:
                    c = Cfg(motor=mo, mass=ms, surface="carpet", loop=lp)
                    J += [("thermal", c, ("climb", lv, 60.0)) for lv in (6.0, 10.0, 14.0, 18.0)]
                    J += [("thermal", c, ("bursts", lv, 60.0)) for lv in (3.0, 5.0)]
                    J += [("thermal", c, ("spin", lv, 60.0)) for lv in (6.0, 10.0)]
                    J += [("thermal", replace(c, surface="floor"), ("push", lv, 30.0)) for lv in (0.3, 1.0)]
    elif part == "P":
        for ms in a.masses:
            base = Cfg(motor=a.motors[0], mass=ms, springs="fixed")
            sprung = E.arm_params(*MASSES[ms]).pod_sprung_mass
            settings = []
            for k in POD_RATES:
                for z in POD_ZETAS:
                    for pl in POD_PRELOADS:
                        # preload "low" 2 N, "mid": a third of the sprung weight per wheel (the spring starts a third
                        # loaded at full droop)
                        pre = 2.0 if pl == "low" else sprung * 9.81 / 4 / 3
                        settings.append(replace(base, pod_rate=k, pod_zeta=z, pod_preload=round(pre, 2)))
            settings.append(replace(base, pod_rate=RIGID_RATE, pod_zeta=0.6, pod_preload=2.0))
            # the fork's pivot height (torque jacking): at the matched springs
            settings += [replace(base, springs="matched", pivot_z=pz) for pz in PIVOT_ZS]
            J += [("susp", c, (sc,)) for c in settings for sc in SUSP_SCENARIOS]
    else:
        raise ValueError(part)
    return J


FUNCS = dict(topspeed=job_topspeed, cruise=job_cruise, straight=job_straight, accel=job_accel, reverse=job_reverse,
             turn=job_turn, panic=job_panic, spin=job_spin, combo=job_combo, ladder=job_ladder, house=job_house,
             slope=job_slope, ramp=job_ramp, side_bump=job_side_bump, wheel_offset=job_wheel_offset,
             thermal=job_thermal, susp=job_susp)


def key(job) -> str:
    kind, cfg, args = job
    return json.dumps([kind, asdict(cfg), list(args)], default=float)


def run_job(job):
    kind, cfg, args = job
    w0 = time.perf_counter()
    try:
        res = FUNCS[kind](cfg, *args)
        err = None
    except Exception:
        res, err = None, traceback.format_exc()
    return dict(key=key(job), kind=kind, cfg=asdict(cfg), args=list(args), res=res, err=err,
                wall_s=time.perf_counter() - w0)


def _clean(x):
    if isinstance(x, dict):
        return {k: _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, (np.floating, float)):
        return None if not math.isfinite(float(x)) else float(x)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, np.bool_):
        return bool(x)
    return x


def load(part: str) -> list:
    path = OUT / f"{part}.jsonl"
    if not path.exists():
        return []
    return [json.loads(ln) for ln in path.read_text().splitlines() if ln.strip()]


def run_part(part: str, a):
    jobs = jobs_for(part, a)
    if a.smoke:
        seen, pick = set(), []
        for j in jobs:
            if j[0] not in seen:
                seen.add(j[0])
                pick.append(j)
        jobs = pick
    path = OUT / (f"{part}.jsonl" if not a.smoke else f"smoke_{part}.jsonl")
    done = set()
    if path.exists() and not a.smoke:
        done = {json.loads(ln)["key"] for ln in path.read_text().splitlines() if ln.strip()
                and json.loads(ln).get("err") is None}
    todo = [j for j in jobs if key(j) not in done]
    print(f"part {part}: {len(jobs)} jobs, {len(jobs) - len(todo)} already done, {len(todo)} to run on "
          f"{a.workers} workers", flush=True)
    if not todo:
        return
    w0 = time.perf_counter()
    n_err = 0
    with open(path, "a" if not a.smoke else "w") as f, ProcessPoolExecutor(a.workers) as ex:
        futs = [ex.submit(run_job, j) for j in todo]
        for k, fu in enumerate(as_completed(futs), 1):
            row = _clean(fu.result())
            if row["err"]:
                n_err += 1
                print(f"  ERROR {row['kind']} {row['args']}: {row['err'].strip().splitlines()[-1]}", flush=True)
            f.write(json.dumps(row) + "\n")
            f.flush()
            if k % max(1, len(todo) // 40) == 0 or k == len(todo):
                el = time.perf_counter() - w0
                print(f"  {k}/{len(todo)}  {el / 60:.1f} min elapsed, ~{el / k * (len(todo) - k) / 60:.1f} min left, "
                      f"{n_err} errors", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("part")
    ap.add_argument("--motors", default="AK40-10")
    ap.add_argument("--masses", default="10,15,20,20hi")
    ap.add_argument("--surfaces", default="floor,carpet")
    ap.add_argument("--socs", default="1.0,0.2")
    ap.add_argument("--workers", type=int, default=WORKERS)
    ap.add_argument("--smoke", action="store_true", help="one job of each kind, to results/motor_char/smoke_<PART>.jsonl")
    ap.add_argument("--no-thermal-variants", action="store_true")
    ap.add_argument("--coarse", action="store_true", help="phase 4: five directions, fewer speeds (S, F, A, C)")
    ap.add_argument("--loops", default="1.0", help="speed-loop gain factors (Cfg.loop), e.g. 1.0,0.25")
    ap.add_argument("--only", default="", help="report: these parts only (e.g. CA)")
    a = ap.parse_args()
    a.motors = tuple(a.motors.split(","))
    a.masses = tuple(a.masses.split(","))
    a.surfaces = tuple(a.surfaces.split(","))
    a.socs = tuple(float(x) for x in a.socs.split(","))
    a.loops = tuple(float(x) for x in a.loops.split(","))
    for mo in a.motors:
        if mo not in DRIVE_MOTORS:
            sys.exit(f"unknown motor {mo}")
    if a.part == "report":
        import drive_char_report
        drive_char_report.main(list(a.only) if a.only else list("CARSFIUTP") + ["M", "E"])
        return
    for part in a.part:
        run_part(part, a)


if __name__ == "__main__":
    main()
