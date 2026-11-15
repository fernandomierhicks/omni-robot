"""Bumps without bumper switches (Nov 9 2026, Fernando: "do number 1 in sim only first to see how good we could detect
bumps"). Sim only; no firmware or CAD change.

The question: before drawing bumper switches into cad/v13, how well can the arm robot tell it has touched something
from what its Teensy already reads?
  * the BNO085: game rotation vector, gyro and accelerometer at 400 Hz (sensors.py's noise model);
  * the four AK40-10s' status broadcasts at 500 Hz, as the firmware decodes them: speed (10 ERPM steps) and q-axis
    current (10 mA steps; current noise added in the analysis, est);
  * its own wheel-speed commands (the ramped body velocity it is executing).

The robot is the arm robot (robot_export.arm_d435_params: CAD mass 9.63 kg, the SO-101 stowed) with explicit rollers,
running the C++ firmware in the loop (SilRobot, the "arm" variant) and DRIVE_VEL_BODY commands over the emulated USB
link, adult profile. Each run logs those MCU-side signals plus the truth (contact forces between the robot and the
obstacle, where on the robot, the pose). Positive runs keep commanding the same velocity for 1.5 s after the contact (a
robot computer that hasn't noticed); detection is scored afterwards on the logs, with the observer below run over them
tick by tick as the MCU would.

Obstacles (each face square to the path unless stated; the robot travels in body direction theta, yaw held):
  wall    rigid wall (0.6 m), head-on, or angled 45 deg (glancing)
  post    rigid 50 mm post (a table or chair leg) at a lateral offset: the contact bearing is off the direction of travel
  sofa    a sprung face (1.5 kN/m over 80 mm, then hard; est): a sofa front / cushion
  leg     a sprung 100 mm post (4 kN/m over 50 mm, est): a person's or pet's leg
  object  free things on the floor (mu 0.45 on the floor, est): a shoe 0.4 kg, toy box 1.5 kg, moving box 3 kg, a
          heavy toy chest 8 kg, a play ball 0.3 kg
  table   a table top's edge at z 0.42-0.45 m: above the lidar's plane and the D435's view, it meets the face / speaker
  push    the robot standing still, pushed at its bumper: a lean (25 N or 10 N, ramped) or a kick (60 N for 40 ms)
Negatives (no contact; false alarms):
  free    hard-floor driving: 1 and 2 m/s legs and stops in eight directions, snap reversals, spins, spin + translate,
          tidy-style creeping
  floor   every transition of floor_wheels.OBSTACLES and floor.HOUSE_OBSTACLES at 0.3 / 0.8 / 1.5 m/s, 0 / 45 / 90 deg
  arm     the SO-101 moving (stow -> high -> predrop -> drop -> stow) with the base still, and driving with it high

Detectors (ContactObserver; the same code would port to omni_fw):
  imu     the IMU's horizontal acceleration less the commanded one (the firmware's ramped velocity, lagged by the speed
          loop): the "can the IMU detect a bump" question
  motor   the wheel forces from the currents less what the commanded acceleration needs (no IMU)
  fused   an external-force observer: F_ext = M a_imu - J^T (tau - I_w dw/dt - friction(w)); M, I_w and friction are
          fitted on the hard-floor free-driving runs (as a calibration drive would on the robot)
  Each fires on a fast tier (|signal| low-passed at 20 Hz over a threshold for 5 ms) or a slow tier (3 Hz, 60 ms).
  Thresholds: the largest value any negative run reached (each tier, each signal) x MARGIN. The contact's bearing is
  the direction the external force comes from (a round rim: a frictionless contact force points at the centre).

  python bump_detect.py run [GROUP ...] [--workers N]     simulate (logs to OMNI_BUMP_LOGS, default ~/sim_data/bump_detect)
  python bump_detect.py analyze [--noise A]               fit, thresholds, scores -> results/bump_detect.json + .md
  python bump_detect.py closed [--workers N]              stop on detection vs an ideal switch vs no stop
  python bump_detect.py one NAME                          one case, printed
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
import time
import traceback
import zlib
from dataclasses import asdict, dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from common import RESULTS, pool  # noqa: E402  (puts the sim on the path)
import floor as floor_mod  # noqa: E402
import robot_export as E  # noqa: E402
from floor_wheels import OBSTACLES  # noqa: E402
from robot_model import build_xml  # noqa: E402
from sil_link import SimLink  # noqa: E402  (puts ros/src on the path)
from sil_robot import SilRobot  # noqa: E402
from omni_mcu import protocol as PR  # noqa: E402

LOGS = Path(os.environ.get("OMNI_BUMP_LOGS", Path.home() / "sim_data" / "bump_detect"))
OUT_JSON = RESULTS / "bump_detect.json"
OUT_MD = RESULTS / "bump_detect_2026-11-09.md"

R_BUMP = 0.225                 # m: the bumper's outer radius (sim ring r 0.211 + its 14 mm capsule; cad/v13 225.15)
SETTLE = 0.6                   # s standing before the first command
AFTER = 1.5                    # s commanding on after the first contact (positives)
FLOOR_MU = 0.45                # the study's floor_mu: what a free object slides on (est, cardboard / shoe on wood)
POSITIVE = ("wall", "post", "sofa", "leg", "object", "table", "push")
NEGATIVE = ("free", "floor", "arm")
OBJECTS = {  # name: (size x (along travel), y, z m, kg, kind)
    "shoe": ((0.10, 0.28, 0.11), 0.4, "box"),          # lying across the path
    "toy box": ((0.30, 0.40, 0.30), 1.5, "box"),
    "moving box": ((0.40, 0.55, 0.40), 3.0, "box"),
    "toy chest": ((0.40, 0.70, 0.45), 8.0, "box"),
    "play ball": ((0.22, 0.22, 0.22), 0.3, "sphere"),
}
FLOORS = {**OBSTACLES, **floor_mod.HOUSE_OBSTACLES}


# -- cases ------------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Case:
    name: str
    group: str
    v: float = 0.0                 # m/s
    theta: float = 0.0             # deg: direction of travel in the body frame (0 = forward)
    offset: float = 0.0            # post / leg: lateral offset (m); wall: its angle (deg); push: bearing (deg)
    obj: str = ""                  # object, floor feature, push kind or script name
    seed: int = 0

    @property
    def positive(self) -> bool:
        return self.group in POSITIVE


def cases() -> list:
    out = []
    for th in (0.0, 45.0, 90.0, 180.0):
        for v in (0.1, 0.2, 0.3, 0.5, 0.8):
            out.append(Case(f"wall v{v} th{th:g}", "wall", v, th))
    for th in (0.0, 90.0):
        for v in (0.2, 0.5, 0.8):
            out.append(Case(f"wall45 v{v} th{th:g}", "wall", v, th, offset=45.0))
    for th in (0.0, 90.0):
        for b in (0.0, 0.12, 0.18):
            for v in (0.2, 0.5, 0.8):
                out.append(Case(f"post b{b} v{v} th{th:g}", "post", v, th, offset=b))
    for th in (0.0, 45.0, 90.0, 180.0):
        for v in (0.2, 0.5, 0.8):
            out.append(Case(f"sofa v{v} th{th:g}", "sofa", v, th))
    for th in (0.0, 90.0):
        for b in (0.0, 0.12):
            for v in (0.2, 0.5, 0.8):
                out.append(Case(f"leg b{b} v{v} th{th:g}", "leg", v, th, offset=b))
    for o in OBJECTS:
        for th in (0.0, 90.0):
            for v in (0.2, 0.5, 0.8):
                out.append(Case(f"object {o} v{v} th{th:g}", "object", v, th, obj=o))
    for th in (0.0, 180.0):
        for v in (0.2, 0.5, 0.8):
            out.append(Case(f"table v{v} th{th:g}", "table", v, th))
    for kind in ("lean25", "lean10", "kick60"):
        for b in (0.0, 90.0, 225.0):
            out.append(Case(f"push {kind} b{b:g}", "push", 0.0, 0.0, offset=b, obj=kind))
    for s in SCRIPTS:
        out.append(Case(f"free {s}", "free", obj=s))
    for f in FLOORS:
        for th in (0.0, 45.0, 90.0):
            for v in (0.3, 0.8, 1.5):
                out.append(Case(f"floor {f} v{v} th{th:g}", "floor", v, th, obj=f))
    out.append(Case("arm moves", "arm", obj="moves"))
    out.append(Case("arm high drive", "arm", obj="high drive"))
    return out


def _seg(dur, v=0.0, th=0.0, wz=0.0, hold=True):
    return (dur, v * math.cos(math.radians(th)), v * math.sin(math.radians(th)), wz, hold)


SCRIPTS = {
    "legs 1 m/s a": sum(([_seg(1.2, 1.0, th), _seg(0.6), _seg(1.2, 1.0, th + 180), _seg(0.6)] for th in (0, 45)), []),
    "legs 1 m/s b": sum(([_seg(1.2, 1.0, th), _seg(0.6), _seg(1.2, 1.0, th + 180), _seg(0.6)] for th in (90, 135)), []),
    "legs 2 m/s": sum(([_seg(1.6, 2.0, th), _seg(1.2), _seg(1.6, 2.0, th + 180), _seg(1.2)] for th in (0, 90)), []),
    "reversals": sum(([_seg(1.0, 0.8, th), _seg(1.0, 0.8, th + 180)] for th in (0, 45, 90)), []) + [_seg(1.0)],
    "spins": [_seg(1.5, wz=3.0, hold=False), _seg(1.5, wz=-3.0, hold=False), _seg(0.6),
              _seg(2.0, 0.5, 0, wz=2.0, hold=False), _seg(1.0)],
    "creep": sum(([_seg(0.6, 0.15, th), _seg(0.4)] for th in range(0, 360, 45)), [])
    + sum(([_seg(0.5, 0.3, th), _seg(0.5)] for th in range(0, 360, 90)), []),
}


# -- the world --------------------------------------------------------------------------------------------------------

def _f(*xs) -> str:
    return " ".join(f"{x:.6g}" for x in xs)


def _yawq(a_deg: float) -> str:
    a = math.radians(a_deg) / 2
    return _f(math.cos(a), 0, 0, math.sin(a))


def gap_for(c: Case) -> float:
    """m between the bumper and the obstacle at the start: room to reach the speed and hold it ~0.4 s."""
    return 0.12 + c.v * c.v / 2.5 + 0.4 * c.v


def obstacle_xml(c: Case) -> str:
    """World geoms / bodies named obst_* (the truth counts contacts with these). The robot starts at the origin,
    travelling along world +x."""
    x0 = R_BUMP + gap_for(c)                                # the face the bumper meets
    rigid = 'contype="1" conaffinity="1" solref="0.005 1" friction="0.5 0.005 0.0001" rgba="0.75 0.7 0.6 1"'
    if c.group == "wall":
        a = c.offset
        xl = R_BUMP / math.cos(math.radians(a)) + gap_for(c) if a else x0
        n = np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])
        ctr = np.array([xl, 0.0]) + n * 0.05
        return f'<geom name="obst_wall" type="box" size="0.05 1.6 0.3" pos="{_f(*ctr, 0.3)}" quat="{_yawq(a)}" {rigid}/>'
    if c.group == "post":
        r = 0.025
        return (f'<geom name="obst_post" type="cylinder" size="{_f(r, 0.35)}" pos="{_f(x0 + r, c.offset, 0.35)}" '
                f'{rigid}/>')
    if c.group in ("sofa", "leg"):
        if c.group == "sofa":
            k, travel, mass, geom = 1500.0, 0.08, 2.0, f'type="box" size="0.15 0.8 0.2" pos="0.15 0 0.22"'
            body = (x0, 0.0)
        else:
            k, travel, mass, geom = 4000.0, 0.05, 3.0, 'type="cylinder" size="0.05 0.3" pos="0 0 0.3"'
            body = (x0 + 0.05, c.offset)
        damp = 2 * 0.4 * math.sqrt(k * mass)
        return (f'<body name="obst_{c.group}" pos="{_f(*body, 0)}"><joint name="obst_slide" type="slide" axis="1 0 0" '
                f'stiffness="{k:g}" damping="{damp:.4g}" range="0 {travel:g}" solreflimit="0.004 1"/>'
                f'<geom name="obst_{c.group}" {geom} mass="{mass:g}" contype="1" conaffinity="1" '
                f'friction="0.8 0.005 0.0001" rgba="0.4 0.45 0.6 1"/></body>')
    if c.group == "object":
        (sx, sy, sz), kg, kind = OBJECTS[c.obj]
        g = (f'type="box" size="{_f(sx / 2, sy / 2, sz / 2)}"' if kind == "box" else f'type="sphere" size="{sx / 2:.4g}"')
        return (f'<body name="obst_obj" pos="{_f(x0 + sx / 2, 0, sz / 2 + 0.0005)}"><freejoint name="obst_free"/>'
                f'<geom name="obst_obj" {g} mass="{kg:g}" contype="1" conaffinity="1" condim="4" '
                f'friction="0.3 0.005 0.0001" solref="0.004 1" rgba="0.8 0.55 0.3 1"/></body>')
    if c.group == "table":
        # the first tall colliding part the edge meets, along the travel: forward the face (body x 0.158); backward the
        # edge passes over the bin (top z 0.25) and meets the uprights' rear faces (body x 0.096: -0.096 along travel)
        lead = 0.158 if c.theta == 0.0 else -0.096
        xe = lead + gap_for(c)
        return f'<geom name="obst_table" type="box" size="0.45 0.7 0.015" pos="{_f(xe + 0.45, 0, 0.435)}" {rigid}/>'
    return ""


def params_for(c: Case):
    feats = ()
    if c.group == "floor":
        feats = (FLOORS[c.obj],)
    return E.arm_d435_params(wheel_model="rollers", area_size=(12.0, 12.0), home_xy=(0.0, 0.0), wall_height=0.6,
                             floor_features=feats, floor_mu=FLOOR_MU)


def start_for(c: Case) -> tuple:
    """(x, y) of the robot's centre at the start."""
    if c.group == "floor":
        f = FLOORS[c.obj]
        edge = f.x if hasattr(f, "x") else f.x0
        return (edge - R_BUMP - gap_for(c), 0.0)
    if c.group == "free":
        return (0.0, 0.0)
    return (0.0, 0.0)


def build(c: Case):
    p = params_for(c)
    xml = build_xml(p)
    extra = obstacle_xml(c)
    if extra:
        xml = xml.replace("</worldbody>", extra + "</worldbody>", 1)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    x, y = start_for(c)
    yaw = -math.radians(c.theta)                           # body direction theta = world +x
    d.qpos[adr:adr + 2] = (x, y)
    d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(m, d)
    return p, m, d


# -- the run ----------------------------------------------------------------------------------------------------------

class Truth:
    """Contact forces on the robot from obst_* geoms (body frame), where (bearing in the body frame), at every call."""

    def __init__(self, m, d):
        self.m, self.d = m, d
        base = m.body("base").id
        sub = set()
        for b in range(m.nbody):
            bb = b
            while bb > 0:
                if bb == base:
                    sub.add(b)
                    break
                bb = m.body_parentid[bb]
        self.robot = np.array([m.geom_bodyid[g] in sub for g in range(m.ngeom)])
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "" for g in range(m.ngeom)]
        self.obst = np.array([n.startswith("obst_") for n in names])
        self.f6 = np.zeros(6)
        self.base = base
        self.obj_body = next((b for b in range(m.nbody) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "")
                              .startswith("obst_")), None)
        self.obj0 = None if self.obj_body is None else d.xpos[self.obj_body].copy()

    def sample(self, yaw: float):
        m, d = self.m, self.d
        F = np.zeros(3)
        pts, wts = [], []
        for i in range(d.ncon):
            con = d.contact[i]
            g1, g2 = con.geom1, con.geom2
            if self.robot[g1] and self.obst[g2]:
                sgn = -1.0
            elif self.robot[g2] and self.obst[g1]:
                sgn = 1.0
            else:
                continue
            mujoco.mj_contactForce(m, d, i, self.f6)
            fw = con.frame.reshape(3, 3).T @ self.f6[:3]
            F += sgn * fw
            pts.append(con.pos.copy())
            wts.append(abs(self.f6[0]))
        c, s = math.cos(yaw), math.sin(yaw)
        fb = np.array([c * F[0] + s * F[1], -s * F[0] + c * F[1], F[2]])
        bearing = math.nan
        z = math.nan
        if pts and sum(wts) > 0:
            P = np.average(np.array(pts), axis=0, weights=np.array(wts)) - d.xpos[self.base]
            bearing = math.degrees(math.atan2(-s * P[0] + c * P[1], c * P[0] + s * P[1]))
            z = float(np.average(np.array(pts), axis=0, weights=np.array(wts))[2])
        moved = 0.0 if self.obj_body is None else float(np.linalg.norm(d.xpos[self.obj_body][:2] - self.obj0[:2]))
        return fb, bearing, z, moved


def run_case(c: Case, closed: str = "", cal: dict | None = None, save: bool = True) -> dict:
    """Simulate one case. closed: "" (open loop: keep commanding), "switch" (an ideal bumper switch: stop 2 ms after
    the first contact), "fused" (stop when the fused observer fires; needs cal), "none" (never stop, as "" but scored
    as a closed run)."""
    p, m, d = build(c)
    r = SilRobot(m, d, p, seed=c.seed)
    r.arm.set_pose(E.ARM_POSES["stow"], snap=True)
    mujoco.mj_forward(m, d)
    link = SimLink(r, seed=c.seed)
    truth = Truth(m, d)
    kt = r.dm[0].kt
    mot_log = []

    def wrap(k, mot):
        orig = mot.status

        def status(t, pos, vel, tau, t_coil):
            fr = orig(t, pos, vel, tau, t_coil)
            if fr is not None:
                _, sp, cur, _, _ = struct.unpack(">hhhbB", bytes(fr.data[:8]))
                mot_log.append((t, k, sp * 10.0 / mot.k, cur / 100.0))
            return fr
        mot.status = status
    for k, mot in enumerate(r.dm):
        wrap(k, mot)

    J = np.asarray(r.kin.J, dtype=float)
    obs = ContactObserver(J, kt, cal) if closed == "fused" else None
    cur_rng = np.random.default_rng(zlib.crc32(c.name.encode()))
    cur_off = cur_rng.normal(0, 0.03, 4)
    imu_log, cmd_log, tru_log = [], [], []
    script = None
    if c.group == "free":
        script = SCRIPTS[c.obj]
    elif c.group == "arm" and c.obj == "high drive":
        script = sum(([_seg(0.8, 0.5, th), _seg(0.8)] for th in (0, 90, 180, 270)), [])
    t_end_script = SETTLE + (sum(s[0] for s in script) if script else 0.0)
    if c.group in ("wall", "post", "sofa", "leg", "object", "table"):
        dist = gap_for(c) + (0.4 if c.group == "post" else 0.1)
        t_max = SETTLE + dist / c.v + c.v / 2.0 + AFTER + 0.5
    elif c.group == "floor":
        t_max = SETTLE + (gap_for(c) + R_BUMP * 2 + 0.25) / c.v + c.v / 2.0 + 1.5
    elif c.group == "push":
        t_max = SETTLE + 2.5
    elif c.group == "arm" and c.obj == "moves":
        t_max = SETTLE + 9.0
    else:
        t_max = t_end_script + 0.5
    arm_seq = [(SETTLE + 0.2, "high"), (SETTLE + 2.2, "predrop"), (SETTLE + 4.2, "drop"), (SETTLE + 6.0, "high"),
               (SETTLE + 7.5, "stow")] if c.obj == "moves" else []
    if c.obj == "high drive":
        r.arm.set_pose(E.ARM_POSES["high"], snap=True)
        mujoco.mj_forward(m, d)
    floor_stop_x = math.inf
    if c.group == "floor":
        f = FLOORS[c.obj]
        floor_stop_x = (f.x if hasattr(f, "x") else f.x0) + R_BUMP + 0.25
    dt = m.opt.timestep
    t_cmd, t_ms = 0.0, 0.0
    t_contact = None
    t_stop = None
    last_imu_t = -1.0
    stop = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=0.0, b=0.0, c=0.0, flags=PR.CMD_HOLD_YAW)
    push_pt = None
    if c.group == "push":
        b = math.radians(c.offset)
        push_pt = np.array([R_BUMP * math.cos(b), R_BUMP * math.sin(b), 0.07])
    n_mot = 0
    t_wall = time.time()
    mot_state = [(0.0, 0.0)] * 4
    imu_state = None
    try:
        while d.time < t_max:
            t = d.time
            link.poll(t)
            if t >= t_cmd - 1e-9:
                t_cmd = t + 0.01
                if t_stop is not None and t >= t_stop:
                    cmd = stop
                elif t < SETTLE:
                    cmd = stop
                elif script is not None:
                    tt, cmd = SETTLE, stop
                    for dur, vx, vy, wz, hold in script:
                        if t < tt + dur:
                            cmd = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=vx, b=vy, c=wz,
                                             flags=PR.CMD_HOLD_YAW if hold else 0)
                            break
                        tt += dur
                elif c.group in ("push", "arm"):
                    cmd = stop
                else:
                    th = math.radians(c.theta)
                    cmd = PR.Command(drive_mode=PR.DRIVE_VEL_BODY, a=c.v * math.cos(th), b=c.v * math.sin(th),
                                     c=0.0, flags=PR.CMD_HOLD_YAW)
                    if c.group == "floor" and d.qpos[m.joint("base").qposadr[0]] > floor_stop_x:
                        cmd = stop                          # across it (a threshold) or on it (a rug): stop there
                link.send(cmd, t)
            for ta, pose in arm_seq:
                if abs(t - ta) < dt / 2:
                    r.arm.set_pose(E.ARM_POSES[pose])
            if push_pt is not None:
                d.xfrc_applied[truth.base] = 0.0
                ts = t - (SETTLE + 0.3)
                mag = 0.0
                if c.obj.startswith("lean"):
                    F0 = float(c.obj[4:])
                    mag = F0 * min(1.0, ts / 0.3) if 0.0 <= ts < 1.3 else 0.0
                elif c.obj.startswith("kick"):
                    mag = float(c.obj[4:]) if 0.0 <= ts < 0.04 else 0.0
                if mag > 0:
                    R = d.xmat[truth.base].reshape(3, 3)
                    pw = d.xpos[truth.base] + R @ push_pt
                    fw = -mag * (R @ np.array([push_pt[0], push_pt[1], 0.0])) / R_BUMP      # toward the centre
                    d.xfrc_applied[truth.base, :3] = fw
                    d.xfrc_applied[truth.base, 3:] = np.cross(pw - d.xipos[truth.base], fw)
            r.step()
            rd = r.readings
            if rd is not None and rd.imu3 is not None and rd.imu3[0] != last_imu_t:
                ti, q, g, a = rd.imu3
                last_imu_t = ti
                imu_log.append((ti, *q, *g, *a))
                imu_state = (np.asarray(q, float), np.asarray(g, float), np.asarray(a, float))
            if d.time >= t_ms - 1e-9:                      # the MCU's 1 kHz tick
                t_ms = d.time + 0.001
                pose = r.pose()
                yaw = float(pose[2])
                fb, bearing, zc, moved = truth.sample(yaw)
                if c.group == "push":
                    fx = d.xfrc_applied[truth.base, :3]
                    cy, sy = math.cos(yaw), math.sin(yaw)
                    fb = np.array([cy * fx[0] + sy * fx[1], -sy * fx[0] + cy * fx[1], fx[2]])
                    bearing = c.offset if np.linalg.norm(fx) > 0 else math.nan
                    zc = 0.07 if np.linalg.norm(fx) > 0 else math.nan
                v = d.sensor("base_linvel").data
                cy, sy = math.cos(yaw), math.sin(yaw)
                wcmd = np.asarray(r.wheel_cmd, float) if getattr(r, "wheel_cmd", None) is not None else np.zeros(4)
                cmd_log.append((d.time, *wcmd))
                R = d.xmat[truth.base].reshape(3, 3)
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
                tru_log.append((d.time, pose[0], pose[1], yaw, cy * v[0] + sy * v[1], -sy * v[0] + cy * v[1],
                                *fb, bearing, zc, moved, tilt))
                fmag = float(np.linalg.norm(fb[:2]))
                if t_contact is None and fmag > 0.5:
                    t_contact = d.time
                    if closed == "switch":
                        t_stop = d.time + 0.002            # an ideal switch: 2 ms debounce
                        t_cmd = min(t_cmd, t_stop)
                    elif closed == "" and c.positive and c.group != "push":
                        t_max = min(t_max, d.time + AFTER)
                if obs is not None and t_stop is None:
                    for (_, k, w, i) in mot_log[n_mot:]:          # the analysis' current noise (est), online too
                        mot_state[k] = (w, round((i + cur_rng.normal(0, 0.05) + cur_off[k]) * 100) / 100)
                    n_mot = len(mot_log)
                    if imu_state is not None:
                        out = obs.tick(imu_state, np.array([w for w, _ in mot_state]),
                                       np.array([i for _, i in mot_state]), wcmd)
                        if out["fire"] and d.time > SETTLE + 0.05:
                            t_stop = d.time + 0.0005       # the MCU's own decision: the stop goes out at once
                            t_cmd = min(t_cmd, t_stop)
                if tilt > 35.0:
                    break                                  # tipped
    finally:
        r.close()
    log = dict(imu=np.array(imu_log, np.float32), mot=np.array(mot_log, np.float32), cmd=np.array(cmd_log, np.float32),
               tru=np.array(tru_log, np.float32), J=J, kt=np.array([kt]))
    if save:
        LOGS.mkdir(parents=True, exist_ok=True)
        fn = LOGS / f"{_slug(c.name)}{'__' + closed if closed else ''}.npz"
        np.savez_compressed(fn, **log)
    res = dict(case=asdict(c), closed=closed, t_contact=t_contact, t_stop=t_stop, wall_s=round(time.time() - t_wall, 1),
               sim_s=round(d.time, 3))
    if closed:
        res.update(closed_metrics(c, log, t_contact))
    return res


def _slug(s: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-." else "_" for ch in s)


def closed_metrics(c: Case, log: dict, t_contact) -> dict:
    """Peak contact force, how far the robot went on after the first contact, and how far the obstacle moved."""
    tru = log["tru"].astype(float)
    if t_contact is None or not len(tru):
        return dict(peak_N=None, travel_after_m=None, obj_moved_m=None)
    k0 = int(np.searchsorted(tru[:, 0], t_contact))
    F = np.hypot(tru[k0:, 6], tru[k0:, 7])
    travel = float(np.max(tru[k0:, 1]) - tru[k0, 1])              # travel is along world +x
    return dict(peak_N=round(float(np.max(F)), 1), travel_after_m=round(travel, 4),
                obj_moved_m=round(float(np.max(tru[k0:, 11])), 4), end_N=round(float(F[-1]), 1),
                tilt_max=round(float(np.max(tru[:, 12])), 1))


# -- the detector -----------------------------------------------------------------------------------------------------

def _quat_rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


G = 9.81
FC_FAST, FC_SLOW = 20.0, 3.0           # Hz
N_FAST, N_SLOW = 5, 60                 # ms the fast / slow tier must stay over its threshold
HELD_TAU, HELD_FRAC, N_HELD = 0.3, 0.5, 100     # held back: velocity blend (s), allowance (of the speed), ms over
TIERS = {"fused": (("f", N_FAST), ("s", N_SLOW)), "imu": (("f", N_FAST), ("s", N_SLOW)),
         "motor": (("f", N_FAST), ("s", N_SLOW)), "held": (("h", N_HELD),)}
GATE_R = 0.22                          # m: "crossing a mapped floor edge" while the centre is this near one


def _alpha(fc, dt=0.001):
    tau = 1.0 / (2 * math.pi * fc)
    return dt / (tau + dt)


class ContactObserver:
    """Runs at the MCU's 1 kHz tick on the latest IMU sample (q wxyz, gyro, accel incl. gravity, IMU frame = body frame)
    and the latest motor status (wheel speeds rad/s, currents A) and its own wheel-speed commands (rad/s).

    cal: M (kg), Iw (kg m^2 per wheel at the wheel), fc (N m Coulomb at the wheel), b (N m s), lag (s: the speed loop
    behind the command), thresholds {detector: (fast N or m/s^2, slow ...)}. Without cal the signals are raw parts
    (for the fit)."""

    def __init__(self, J, kt, cal: dict | None = None):
        self.J = np.asarray(J, float)
        self.Jt = self.J.T
        self.Jp = np.linalg.pinv(self.J)
        self.kt = float(kt)
        self.cal = cal or {}
        self.af, self.as_ = _alpha(FC_FAST), _alpha(FC_SLOW)
        self.ad = _alpha(50.0)
        self.w_lp = None
        self.vcmd_lp = None
        self.acmd_lag = np.zeros(2)
        self.st = {}
        self.cnt = {}

    def _lp(self, key, x, a):
        y = self.st.get(key)
        y = np.array(x, float) if y is None else y + a * (np.asarray(x, float) - y)
        self.st[key] = y
        return y

    def parts(self, imu, w, cur, wcmd):
        """The raw (unfiltered) ingredients at one tick."""
        q, g, acc = imu
        Rm = _quat_rot(q)                                   # IMU -> level frame (its yaw is irrelevant here)
        a_lin = acc - Rm.T @ np.array([0.0, 0.0, G])        # specific force less gravity, IMU frame
        # wheel accelerations: a dirty derivative of the reported speeds (50 Hz)
        if self.w_lp is None:
            self.w_lp = np.array(w, float)
        w_prev = self.w_lp.copy()
        self.w_lp = self.w_lp + self.ad * (np.asarray(w, float) - self.w_lp)
        dw = (self.w_lp - w_prev) / 0.001
        v_cmd = self.Jp @ np.asarray(wcmd, float)
        if self.vcmd_lp is None:
            self.vcmd_lp = v_cmd.copy()
        v_prev = self.vcmd_lp.copy()
        self.vcmd_lp = self.vcmd_lp + self.ad * (v_cmd - self.vcmd_lp)
        a_cmd = (self.vcmd_lp - v_prev) / 0.001
        wz = float(g[2])
        a_cmd_xy = a_cmd[:2] + wz * np.array([-v_cmd[1], v_cmd[0]])     # body-frame acceleration of the commanded motion
        tau = self.kt * np.asarray(cur, float)
        return dict(a=a_lin[:2], az=a_lin[2], gxy=float(math.hypot(g[0], g[1])), Fm=(self.Jt @ tau)[:2],
                    Fdw=(self.Jt @ dw)[:2], Fsg=(self.Jt @ np.tanh(np.asarray(w, float) / 0.5))[:2],
                    Fv=(self.Jt @ np.asarray(w, float))[:2], a_cmd=a_cmd_xy, v_cmd=v_cmd[:2], wz=wz,
                    vo=(self.Jp @ np.asarray(w, float))[:2],
                    verr=(self.Jp @ (np.asarray(wcmd, float) - np.asarray(w, float)))[:2])

    def held(self, P) -> float:
        """Held back: how much slower than commanded the body is going (m/s), less HELD_FRAC of the speed. The body's
        velocity is the IMU integrated and pulled toward the wheel odometry with HELD_TAU (a wheel spinning against a
        wall keeps its odometry going; the IMU says the body stopped); the command lags by the speed loop's lag."""
        if not hasattr(self, "v_est"):
            self.v_est, self.v_ref = P["vo"].copy(), P["v_cmd"].copy()
        v = self.v_est
        self.v_est = v + 0.001 * (P["a"] - P["wz"] * np.array([-v[1], v[0]])) + (0.001 / HELD_TAU) * (P["vo"] - v)
        lag = self.cal.get("lag", 0.02)
        self.v_ref = self.v_ref + (0.001 / (lag + 0.001)) * (P["v_cmd"] - self.v_ref)
        sp = float(np.hypot(*self.v_ref))
        if sp < 0.05:
            return -1.0
        return sp - float(self.v_ref @ self.v_est) / sp - HELD_FRAC * sp

    def tick(self, imu, w, cur, wcmd) -> dict:
        P = self.parts(imu, w, cur, wcmd)
        cal = self.cal
        lag = cal.get("lag", 0.02)
        self.acmd_lag = self.acmd_lag + (0.001 / (lag + 0.001)) * (P["a_cmd"] - self.acmd_lag)
        M, Iw, fc, b = cal.get("M", 9.63), cal.get("Iw", 0.0), cal.get("fc", 0.0), cal.get("b", 0.0)
        Fw = P["Fm"] - Iw * P["Fdw"] - fc * P["Fsg"] - b * P["Fv"]        # what the wheels push the body with
        sig = dict(fused=M * P["a"] - Fw,                                  # N: the external force
                   imu=P["a"] - self.acmd_lag,                             # m/s^2: unexpected acceleration
                   motor=M * self.acmd_lag - Fw)                           # N: unexpected wheel force
        out = dict(parts=P)
        for k, s in sig.items():
            out[k + "_f"] = self._lp(k + "_f", s, self.af)
            out[k + "_s"] = self._lp(k + "_s", s, self.as_)
        out["held_h"] = np.array([self.held(P)])
        out["gxy"] = self._lp("gxy", P["gxy"], self.af)
        fire = False
        gated = bool(cal.get("gate_now", False))
        for k in cal.get("online", ()):
            th = cal.get("thresholds", {}).get(k)
            for (tier, n), thr in zip(TIERS[k], th or ()):
                key = k + tier
                over = not gated and _mag(out[f"{k}_{tier}"]) > thr
                self.cnt[key] = self.cnt.get(key, 0) + 1 if over else 0
                fire |= self.cnt[key] >= n
        out["fire"] = fire
        return out


def _mag(x) -> float:
    x = np.asarray(x, float)
    return float(x[0]) if x.shape == (1,) else float(math.hypot(x[0], x[1]))


def replay(log: dict, cal: dict | None) -> dict:
    """Run the observer over a log at 1 kHz with sample-and-hold inputs, as on the MCU. Returns arrays."""
    imu, mot, cmd = (log[k].astype(float) for k in ("imu", "mot", "cmd"))
    obs = ContactObserver(log["J"], float(log["kt"][0]), cal)
    w, cur = np.zeros(4), np.zeros(4)
    ki = km = 0
    rows = {k: [] for k in ("t", "fused_f", "fused_s", "imu_f", "imu_s", "motor_f", "motor_s", "held_h", "gxy", "a",
                            "Fm", "Fdw", "Fsg", "Fv", "a_cmd", "verr")}
    imu_s = None
    for row in cmd:
        t = row[0]
        while ki < len(imu) and imu[ki, 0] <= t + 1e-9:
            imu_s = (imu[ki, 1:5], imu[ki, 5:8], imu[ki, 8:11])
            ki += 1
        while km < len(mot) and mot[km, 0] <= t + 1e-9:
            k = int(mot[km, 1])
            w[k], cur[k] = mot[km, 2], mot[km, 3]
            km += 1
        if imu_s is None:
            continue
        o = obs.tick(imu_s, w, cur, row[1:5])
        rows["t"].append(t)
        for k in ("fused_f", "fused_s", "imu_f", "imu_s", "motor_f", "motor_s", "held_h", "gxy"):
            rows[k].append(o[k])
        for k in ("a", "Fm", "Fdw", "Fsg", "Fv", "a_cmd", "verr"):
            rows[k].append(o["parts"][k])
    return {k: np.array(v) for k, v in rows.items()}


# -- analysis ---------------------------------------------------------------------------------------------------------

def load(c: Case, closed: str = "") -> dict | None:
    fn = LOGS / f"{_slug(c.name)}{'__' + closed if closed else ''}.npz"
    if not fn.exists():
        return None
    z = np.load(fn)
    return {k: z[k] for k in z.files}


def add_current_noise(log: dict, sigma: float, bias: float, seed: int) -> dict:
    """The AK40-10's reported current with white noise sigma (A rms) and a per-motor offset (A); est, VERIFY."""
    if sigma <= 0 and bias <= 0:
        return log
    rng = np.random.default_rng(seed)
    mot = log["mot"].astype(float).copy()
    off = rng.normal(0, bias, 4)
    mot[:, 3] += rng.normal(0, sigma, len(mot)) + off[mot[:, 1].astype(int)]
    mot[:, 3] = np.round(mot[:, 3] * 100) / 100
    return dict(log, mot=mot)


def fit_cal(logs: list) -> dict:
    """M, Iw, fc, b by least squares on the hard-floor free-driving runs (their fast-filtered parts), and the command
    lag that best matches the IMU's acceleration."""
    A, y = [], []
    lag_rows = []
    for log in logs:
        rp = replay(log, None)
        a, Fm, Fdw, Fsg, Fv = (rp[k] for k in ("a", "Fm", "Fdw", "Fsg", "Fv"))
        # filter all parts alike (20 Hz) before the fit
        def lp(x):
            y_ = np.empty_like(x)
            s = x[0].copy()
            al = _alpha(FC_FAST)
            for i in range(len(x)):
                s = s + al * (x[i] - s)
                y_[i] = s
            return y_
        a, Fm, Fdw, Fsg, Fv = (lp(x) for x in (a, Fm, Fdw, Fsg, Fv))
        for ax in range(2):
            A.append(np.column_stack([a[:, ax], Fdw[:, ax], Fsg[:, ax], Fv[:, ax]]))
            y.append(Fm[:, ax])
        lag_rows.append((rp["a"], rp["a_cmd"]))
    A, y = np.vstack(A), np.concatenate(y)
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    M, Iw, fc, b = (float(x) for x in coef)
    best = None
    for lag in (0.005, 0.01, 0.02, 0.03, 0.05, 0.08):
        err = 0.0
        for a, ac in lag_rows:
            al = 0.001 / (lag + 0.001)
            s = np.zeros(2)
            lagged = np.empty_like(ac)
            for i in range(len(ac)):
                s = s + al * (ac[i] - s)
                lagged[i] = s
            err += float(np.sum((a - lagged) ** 2))
        if best is None or err < best[1]:
            best = (lag, err)
    resid = y - A @ coef
    return dict(M=round(M, 3), Iw=round(Iw, 5), fc=round(fc, 4), b=round(b, 5), lag=best[0],
                fit_rms_N=round(float(np.sqrt(np.mean(resid ** 2))), 2))


DETS = ("fused", "imu", "motor", "held", "fused+held")
MARGIN = 1.25


def _sig(rp, k, tier):
    x = np.asarray(rp[f"{k}_{tier}"], float)
    return x[:, 0] if x.shape[1] == 1 else np.hypot(x[:, 0], x[:, 1])


def gate_mask(c: Case, log: dict) -> np.ndarray:
    """True where the robot is crossing a mapped floor edge (its centre within GATE_R of one): what the map and the
    height layer would tell the MCU. From the truth pose (localization ~5 cm in the house runs)."""
    tru = log["tru"].astype(float)
    m = np.zeros(len(tru), bool)
    if c.group != "floor":
        return m
    for e in floor_mod.edges(params_for(c)):
        m |= floor_mod.seg_dist(tru[:, 1:3], e) < GATE_R
    return m


def _on_grid(rp, mask_tru, tru_t):
    """The truth-rate mask on the observer's ticks."""
    if mask_tru is None:
        return np.zeros(len(rp["t"]), bool)
    i = np.clip(np.searchsorted(tru_t, rp["t"]), 0, len(tru_t) - 1)
    return mask_tru[i]


def peaks(rp: dict, t_from: float, gated=None) -> dict:
    """Per detector and tier, the largest value held for the tier's persistence (the threshold that would just fire)."""
    from numpy.lib.stride_tricks import sliding_window_view
    m = rp["t"] >= t_from
    out = {}
    for k in DETS[:4]:
        for tier, n in TIERS[k]:
            x = _sig(rp, k, tier).copy()
            x[~m] = -1e9
            if gated is not None:
                x[gated] = -1e9
            out[k + tier] = float(np.max(np.min(sliding_window_view(x, n), axis=1))) if len(x) >= n else 0.0
    return out


def first_fire(rp: dict, k: str, th, t_from: float, gated=None):
    """(time, tier) the detector k first fires at or after t_from, else (None, None)."""
    if k == "fused+held":
        a, b = first_fire(rp, "fused", th, t_from, gated), first_fire(rp, "held", th, t_from, gated)
        cands = [x for x in (a, b) if x[0] is not None]
        return min(cands, key=lambda x: x[0]) if cands else (None, None)
    th = th[k]
    best = (None, None)
    for (tier, n), thr in zip(TIERS[k], th):
        x = (_sig(rp, k, tier) > thr) & (rp["t"] >= t_from)
        if gated is not None:
            x &= ~gated
        run = 0
        for i, v in enumerate(x):
            run = run + 1 if v else 0
            if run >= n:
                if best[0] is None or rp["t"][i] < best[0]:
                    best = (float(rp["t"][i]), tier)
                break
    return best


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def analyze(noise: float = 0.05, bias: float = 0.03, margin: float = MARGIN, gate: bool = False, logs=None) -> dict:
    """gate: suppress detection while crossing a mapped floor edge (thresholds then come from the rest)."""
    all_cases = cases()
    if logs is None:
        logs = {}
        for c in all_cases:
            lg = load(c)
            if lg is not None:
                logs[c.name] = add_current_noise(lg, noise, bias, zlib.crc32(c.name.encode()))
    missing = [c.name for c in all_cases if c.name not in logs]
    cal = fit_cal([logs[c.name] for c in all_cases if c.group == "free" and c.name in logs])
    rps = {name: replay(lg, cal) for name, lg in logs.items()}
    neg_peaks = {}
    for c in all_cases:
        if c.name in rps and not c.positive:
            g = _on_grid(rps[c.name], gate_mask(c, logs[c.name]), logs[c.name]["tru"][:, 0]) if gate else None
            neg_peaks[c.name] = peaks(rps[c.name], SETTLE + 0.05, g)
    thr = {k: tuple(round(margin * max(p[k + t] for p in neg_peaks.values()), 4) for t, _ in TIERS[k])
           for k in DETS[:4]}
    worst_neg = {k + t: max(neg_peaks, key=lambda n: neg_peaks[n][k + t]) for k in DETS[:4] for t, _ in TIERS[k]}
    rows = []
    for c in all_cases:
        if c.name not in rps or not c.positive:
            continue
        rp, tru = rps[c.name], logs[c.name]["tru"].astype(float)
        F = np.hypot(tru[:, 6], tru[:, 7])
        if not (F > 0.5).any():
            rows.append(dict(case=c.name, group=c.group, contact=False))
            continue
        k0 = int(np.argmax(F > 0.5))
        t0 = float(tru[k0, 0])
        row = dict(case=c.name, group=c.group, v=c.v, theta=c.theta, obj=c.obj, offset=c.offset, contact=True,
                   t0=t0, bearing_true=round(float(tru[min(k0 + 5, len(tru) - 1), 9]), 1),
                   z_contact=round(float(tru[k0, 10]), 3), peak_N=round(float(F.max()), 1),
                   obj_moved=round(float(tru[-1, 11]), 3), tilt_max=round(float(np.max(tru[:, 12])), 1))
        for k in DETS:
            pre = first_fire(rp, k, thr, SETTLE + 0.05)
            if pre[0] is not None and pre[0] < t0 - 0.02:
                row[k + "_early"] = round(pre[0] - t0, 3)
            td, tier = first_fire(rp, k, thr, t0 - 0.02)
            if td is None or td > t0 + AFTER:
                row[k] = None
                continue
            kd = min(int(np.searchsorted(tru[:, 0], td)), len(tru) - 1)
            ir = min(int(np.searchsorted(rp["t"], td)) + 10, len(rp["t"]) - 1)
            # the bearing: where the external force comes from (the fused observer's, whichever detector fired)
            Fs = rp["fused_f" if tier == "f" else "fused_s"][ir] if k != "imu" else rp[f"imu_{tier}"][ir]
            b_est = math.degrees(math.atan2(-Fs[1], -Fs[0]))
            bt = tru[kd, 9] if not math.isnan(tru[kd, 9]) else row["bearing_true"]
            push = float(np.hypot(*(tru[kd, 1:3] - tru[k0, 1:3])))
            row[k] = dict(latency_ms=round((td - t0) * 1000, 1), tier=tier, force_N=round(float(F[kd]), 1),
                          push_mm=round(push * 1000, 1), bearing_err=round(abs(_wrap(b_est - bt)), 1))
        rows.append(row)
    return dict(cal=cal, noise_A=noise, bias_A=bias, margin=margin, gate=gate, thresholds=thr, worst_negative=worst_neg,
                neg_peaks=neg_peaks, rows=rows, summary=summarize(rows), missing=missing)


def summarize(rows: list) -> dict:
    out = {}
    groups = sorted({r["group"] for r in rows})
    for k in DETS:
        per = {}
        for g in groups + ["all"]:
            rr = [r for r in rows if r.get("contact") and (g == "all" or r["group"] == g)]
            hit = [r[k] for r in rr if r.get(k)]
            per[g] = dict(n=len(rr), detected=len(hit),
                          latency_ms_median=_med([h["latency_ms"] for h in hit]),
                          push_mm_median=_med([h["push_mm"] for h in hit]),
                          force_N_median=_med([h["force_N"] for h in hit]),
                          bearing_err_median=_med([h["bearing_err"] for h in hit]),
                          bearing_err_p90=_pct([h["bearing_err"] for h in hit], 90),
                          early=sum(1 for r in rr if r.get(k + "_early") is not None))
        out[k] = per
    return out


def _med(x):
    return round(float(np.median(x)), 1) if x else None


def _pct(x, q):
    return round(float(np.percentile(x, q)), 1) if x else None


GROUP_NAMES = dict(wall="wall (rigid)", post="post 50 mm (rigid)", sofa="sofa (sprung)", leg="leg (sprung)",
                   object="free objects", table="table edge 0.42 m", push="pushed standing", all="all")


def _det_table(o: dict, keys) -> list:
    L = ["| Obstacle | " + " | ".join(keys) + " |", "| --- |" + " --- |" * len(keys)]
    for g in o["summary"][keys[0]]:
        cells = []
        for k in keys:
            s = o["summary"][k][g]
            cells.append(f"{s['detected']}/{s['n']}" + (f", {s['latency_ms_median']:.0f} ms" if s["detected"] else ""))
        L.append(f"| {GROUP_NAMES.get(g, g)} | " + " | ".join(cells) + " |")
    return L


def findings(res: dict) -> list:
    o, og = res["open"], res["open_gated"]
    s, sg = o["summary"], og["summary"]

    def n(sm, k, g="all"):
        return f"{sm[k][g]['detected']}/{sm[k][g]['n']}"
    rigid = sum(sg["fused"][g]["detected"] for g in ("wall", "post", "sofa", "leg", "table"))
    rigid_n = sum(sg["fused"][g]["n"] for g in ("wall", "post", "sofa", "leg", "table"))
    early = sum(sg["fused"][g]["early"] for g in sg["fused"] if g != "all")
    thr = og["thresholds"]["fused"]
    L = ["## Findings", "",
         f"1. **Floor transitions are the problem, not sensor noise.** A wheel meeting a rug edge or a threshold is an "
         f"external force like any bump: a 12 mm square rug edge at 1.5 m/s reads {o['thresholds']['fused'][0] / o['margin']:.0f} N "
         f"and {o['thresholds']['imu'][0] / o['margin']:.0f} m/s² for a moment, more than a wall at 0.5 m/s. With thresholds "
         f"that must stay quiet over every transition, the fused observer finds {n(s, 'fused')} contacts and the IMU "
         f"{n(s, 'imu')}. Hard-floor driving alone is quiet (≤ {og['neg_peaks']['free legs 2 m/s']['fusedf']:.0f} N, "
         f"≤ {og['neg_peaks']['free legs 2 m/s']['imuf']:.1f} m/s²); current noise up to 0.3 A rms changes nothing "
         "(the sweep below).",
         f"2. **With the floor edges known** (detection paused only while crossing a mapped edge), the fused force observer "
         f"(IMU + currents + wheel speeds) finds {rigid}/{rigid_n} contacts with fixed, sprung or high obstacles (walls, "
         f"posts, sofa, leg, a table edge) and {n(sg, 'fused')} overall, with {early} alarms before a contact; rigid "
         f"hits in ~{sg['fused']['wall']['latency_ms_median']:.0f} ms after ~1-3 mm, sprung ones in "
         f"~{sg['fused']['sofa']['latency_ms_median']:.0f} ms. Its floor is ~{thr[1]:.0f} N held (rug driving sets it) or "
         f"~{thr[0]:.0f} N of impact.",
         f"3. **The IMU alone** catches only the hard, quick hits ({n(sg, 'imu')} with edges known, {n(s, 'imu')} without): "
         "once the robot is pinned, measured and commanded accelerations are both zero. The currents carry the rest.",
         f"4. **Missed: light things and gentle touches.** Free objects {n(sg, 'fused', 'object')}: a shoe (0.4 kg), a "
         "play ball (all runs), a 1.5 kg toy box (5 of 6) and a 3 kg box at 0.2 m/s are pushed on (0.3-1.9 m) without a "
         "trace: sliding them takes 2-13 N, under the floor. A 10 N lean on the standing robot and most short kicks are "
         f"missed too (pushes {n(sg, 'fused', 'push')}; 25 N leans found in ~0.35 s). These are exactly what a bumper "
         "switch (1-5 N) would catch: in the closed loop an ideal switch stops the shoe after 19-130 mm.",
         f"5. **Direction:** where the observer fires, the force's direction gives the contact bearing within "
         f"{sg['fused']['all']['bearing_err_median']:.0f}° (median), {sg['fused']['all']['bearing_err_p90']:.0f}° (p90): "
         "finer than four quadrants. Except the table edge: hit at the head, the robot tips back and the estimate "
         "reads ~180° off.",
         "6. **A table edge at 0.42 m tips the robot.** Neither the lidar nor the D435 sees it; driving on, the head is "
         "held while the wheels push the base under: 28-34° of tilt. A stop on detection holds it to 6-23° (an ideal "
         "switch on the head 2-18°): a tilt stop and the arm-robot's height in the costmap matter more here than rim "
         "switches, which would not touch it.",
         "7. **Stopping doesn't soften the hit.** The first impact's peak is the same with a switch, the observer or no "
         "stop (it is over in milliseconds); detection limits only what follows. The firmware's ramped stop alone runs "
         "on ~19 / 64 / 130 mm at 0.2 / 0.5 / 0.8 m/s, so a switch's few mm of travel only report the contact.",
         "",
         "Limits: the sim's contacts, rugs and rollers are models (no tile grout, cables, deep pile, a rug that moved off "
         "the map); thresholds are 1.25 x the worst of ~100 no-contact runs, not a measured false-alarm rate; the gate "
         "uses the true pose and a perfect edge map; the contact peaks come from stiff MuJoCo contacts. On the robot the "
         "thresholds must be re-tuned on its floors.", ""]
    return L


def report(res: dict) -> str:
    o, og = res["open"], res["open_gated"]
    cal = og["cal"]
    L = ["# Bumps without bumper switches: sim study (Nov 9, 2026)", "",
         "Generated by `studies/bump_detect.py report` from `results/bump_detect.json` (the module docstring has the "
         "method). **Simulation only**: MuJoCo contacts, explicit rollers, the AK40-10 and BNO085 models; current noise "
         "and every obstacle's stiffness and friction are estimates. Robot: `arm_d435_params()` (9.63 kg, SO-101 "
         "stowed), the C++ firmware in the loop (`arm` variant, adult profile), DRIVE_VEL_BODY, yaw held. Positive "
         "runs keep driving 1.5 s after contact unless stated.", "",
         "**Decision (Fernando, Nov 9): not implemented.** No software contact sensing (it triggers on rugs and "
         "thresholds) and no rim bumper switches (too complex for the CAD). design.md 4.13.", ""]
    L += findings(res)
    L += ["## Calibration", "",
         f"Fitted on the hard-floor free-driving runs: M {cal['M']} kg (true 9.63), wheel inertia {cal['Iw']} kg m², "
         f"Coulomb {cal['fc']} N m, viscous {cal['b']} N m s per wheel, speed-loop lag {cal['lag'] * 1000:.0f} ms; "
         f"residual {cal['fit_rms_N']} N rms. Current noise {og['noise_A']} A rms + {og['bias_A']} A per-motor offset "
         "(est).", "",
         "## Thresholds: the largest value a no-contact run held, x %.2f" % og["margin"], "",
         "| Detector, tier | Anywhere | set by | Floor edges known | set by |", "| --- | --- | --- | --- | --- |"]
    unit = dict(fused="N", motor="N", imu="m/s²", held="m/s")
    for k in DETS[:4]:
        for j, (t, n) in enumerate(TIERS[k]):
            L.append(f"| {k} {t} ({n} ms) | {o['thresholds'][k][j]:.3g} {unit[k]} | {o['worst_negative'][k + t]} | "
                     f"{og['thresholds'][k][j]:.3g} {unit[k]} | {og['worst_negative'][k + t]} |")
    L += ["", "Free driving on hard floor alone (1-2 m/s legs, reversals, spins, creeping, the arm moving) peaks at: "
          + ", ".join(f"{k} {v:.3g}" for k, v in og["neg_peaks"]["free legs 2 m/s"].items()) + " (the 2 m/s legs).", ""]
    L += ["## Detected / contacts, median latency", "", "**Anywhere (no floor context)**", ""] + _det_table(o, DETS)
    L += ["", "**Floor edges known** (detection paused while the centre is within %.2f m of a mapped edge)" % GATE_R, ""]
    L += _det_table(og, DETS)
    L += ["", "## Floor edges known, fused observer: details", "",
          "| Obstacle | Detected | Latency median (ms) | Pushed before detection, median (mm) | Bearing error median / "
          "p90 (deg) |", "| --- | --- | --- | --- | --- |"]
    for g, s in og["summary"]["fused"].items():
        L.append(f"| {GROUP_NAMES.get(g, g)} | {s['detected']}/{s['n']} | {s['latency_ms_median']} | {s['push_mm_median']} "
                 f"| {s['bearing_err_median']} / {s['bearing_err_p90']} |")
    L += ["", "Misses:", ""]
    miss = [r for r in og["rows"] if r.get("contact") and not r.get("fused")]
    L += [f"- {r['case']}: object moved {r['obj_moved']} m" for r in miss] or ["- none"]
    tab = [r for r in og["rows"] if r["group"] == "table"]
    L += ["", "Table edge (z 0.42 m): the robot's largest tilt in each run, driving on 1.5 s: " +
          ", ".join(f"{r['case']} {r['tilt_max']} deg" for r in tab), ""]
    if "noise_sweep" in res:
        L += ["## Sensitivity to the current noise (floor edges known, all positives)", "",
              "| Current noise / offset (A) | fused thresholds fast / slow (N) | fused | fused+held |",
              "| --- | --- | --- | --- |"]
        for row in res["noise_sweep"]:
            f, fh = row["all"]["fused"], row["all"]["fused+held"]
            L.append(f"| {row['noise']} / {row['bias']} | {row['thr'][0]:.0f} / {row['thr'][1]:.1f} | "
                     f"{f['detected']}/{f['n']} | {fh['detected']}/{fh['n']} |")
        L.append("")
    if "closed" in res:
        L += ["## Closed loop: the firmware's ramped stop on detection", "",
              "Switch = an ideal bumper switch (stop 2 ms after the first contact, any force); fused = the fused + held "
              "observer online (floor-edge thresholds, the same current noise); none = drives on. Peak contact force "
              "(N) / travel after the first contact (mm) / the obstacle's displacement (mm).", "",
              "| Case | Switch | Observer | No stop |", "| --- | --- | --- | --- |"]
        by = {m: {r["case"]["name"]: r for r in rr if "error" not in r} for m, rr in res["closed"].items()}
        for name in CLOSED_SET:
            cells = []
            for mode in ("switch", "fused", "none"):
                r = by.get(mode, {}).get(name)
                if not r or r.get("peak_N") is None:
                    cells.append("-")
                    continue
                cells.append(f"{r['peak_N']:.0f} / {1000 * r['travel_after_m']:.0f} / {1000 * r['obj_moved_m']:.0f}"
                             + (f" (tilt {r['tilt_max']:.0f}°)" if r.get("tilt_max", 0) > 10 else ""))
            L.append(f"| {name} | " + " | ".join(cells) + " |")
    return "\n".join(L) + "\n"


# -- driver -----------------------------------------------------------------------------------------------------------

def _job(args):
    name, closed, cal = args
    c = next(cc for cc in cases() if cc.name == name)
    try:
        return run_case(c, closed, cal)
    except Exception:
        return dict(case=asdict(c), closed=closed, error=traceback.format_exc())


def run_all(groups, workers, closed="", cal=None, names=None, skip_done=True):
    todo = [c for c in cases() if (not groups or c.group in groups) and (names is None or c.name in names)]
    if skip_done and not closed:
        todo = [c for c in todo if load(c) is None]
    print(f"{len(todo)} runs, {workers} workers -> {LOGS}", flush=True)
    res = []
    t0 = time.time()
    with pool(workers) as ex:
        for i, r in enumerate(ex.map(_job, [(c.name, closed, cal) for c in todo])):
            res.append(r)
            if "error" in r:
                print("ERROR", r["case"]["name"], r["error"].splitlines()[-1], flush=True)
            if (i + 1) % 10 == 0 or i + 1 == len(todo):
                print(f"  {i + 1}/{len(todo)}  {time.time() - t0:.0f} s", flush=True)
    return res


CLOSED_SET = [f"wall v{v} th0" for v in (0.2, 0.5, 0.8)] + [f"post b0.12 v{v} th0" for v in (0.2, 0.5, 0.8)] + \
             [f"sofa v{v} th0" for v in (0.2, 0.5, 0.8)] + [f"leg b0.0 v{v} th0" for v in (0.2, 0.5, 0.8)] + \
             [f"object {o} v{v} th0" for o in ("shoe", "toy box", "moving box", "toy chest") for v in (0.2, 0.5, 0.8)] + \
             [f"table v{v} th0" for v in (0.2, 0.5, 0.8)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("run", "analyze", "sweep", "closed", "report", "one", "list"))
    ap.add_argument("rest", nargs="*")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 4) - 4))
    ap.add_argument("--noise", type=float, default=0.05)
    ap.add_argument("--bias", type=float, default=0.03)
    ap.add_argument("--margin", type=float, default=MARGIN)
    ap.add_argument("--redo", action="store_true")
    a = ap.parse_args()
    if a.cmd == "list":
        for c in cases():
            print(c.name)
        return
    if a.cmd == "one":
        c = next(cc for cc in cases() if cc.name == " ".join(a.rest))
        print(json.dumps(run_case(c), indent=1))
        return
    if a.cmd == "run":
        res = run_all(set(a.rest), a.workers, skip_done=not a.redo)
        errs = [r for r in res if "error" in r]
        print(f"done: {len(res) - len(errs)} ok, {len(errs)} errors")
        return
    if a.cmd == "analyze":
        prev = json.loads(OUT_JSON.read_text()) if OUT_JSON.exists() else {}
        for key, gate in (("open", False), ("open_gated", True)):
            out = analyze(a.noise, a.bias, a.margin, gate=gate)
            prev[key] = out
            print(f"== {key}: cal {out['cal']}")
            print("thresholds", out["thresholds"])
            print("set by", out["worst_negative"])
            _print_summary(out)
        OUT_JSON.write_text(json.dumps(prev, indent=1, default=float))
        return
    if a.cmd == "sweep":
        prev = json.loads(OUT_JSON.read_text())
        rows = []
        for noise, bias in ((0.0, 0.0), (0.05, 0.03), (0.15, 0.1), (0.3, 0.2)):
            o = analyze(noise, bias, a.margin, gate=True)
            rows.append(dict(noise=noise, bias=bias, thr=o["thresholds"]["fused"],
                             all={k: o["summary"][k]["all"] for k in ("fused", "held", "fused+held")}))
            print(rows[-1], flush=True)
        prev["noise_sweep"] = rows
        OUT_JSON.write_text(json.dumps(prev, indent=1, default=float))
        return
    if a.cmd == "report":
        OUT_MD.write_text(report(json.loads(OUT_JSON.read_text())), encoding="utf-8")
        print(OUT_MD)
        return
    if a.cmd == "closed":
        prev = json.loads(OUT_JSON.read_text())
        o = prev["open_gated"]
        cal = dict(o["cal"], thresholds={k: tuple(v) for k, v in o["thresholds"].items()}, online=("fused", "held"))
        out = {}
        for mode in ("switch", "fused", "none"):
            out[mode] = run_all(set(), a.workers, closed=mode, cal=cal, names=set(CLOSED_SET), skip_done=False)
        prev["closed"] = out
        OUT_JSON.write_text(json.dumps(prev, indent=1, default=float))
        return


def _print_summary(out):
    for k in DETS:
        print(k)
        for g, s in out["summary"][k].items():
            print(f"  {g:7s} {s['detected']}/{s['n']}  lat {s['latency_ms_median']} ms  push {s['push_mm_median']} mm"
                  f"  F {s['force_N_median']} N  bearing err med {s['bearing_err_median']} p90 {s['bearing_err_p90']}"
                  f"  early {s['early']}")

if __name__ == "__main__":
    main()
