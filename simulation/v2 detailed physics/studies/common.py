"""Shared scenario helpers for the component studies."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from kiwi_control import wrap_angle  # noqa: E402
from params import SimParams  # noqa: E402
from robot import Robot  # noqa: E402
from robot_model import build_model  # noqa: E402

RESULTS = HERE / "results"
RESULTS.mkdir(exist_ok=True)


def make_robot(p: SimParams):
    """Robot at the area center facing +x, settled on its wheels."""
    p = p.with_(home_xy=(0.0, 0.0))
    m = build_model(p)
    d = mujoco.MjData(m)
    adr = m.joint("kiwi").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    # park the ball out of the way
    bq = m.joint("ball").qposadr[0]
    d.qpos[bq:bq + 3] = [-2.2, 1.7, p.ball_spec.radius]
    mujoco.mj_forward(m, d)
    r = Robot(m, d, p)
    for _ in range(int(0.2 / m.opt.timestep)):
        r.step()
    return m, d, r


def body_velocity(d, yaw):
    v = d.sensor("base_linvel").data
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])


def accel_run(p: SimParams, direction_deg: float, accel: float, v_final: float = 1.0):
    """Constant-acceleration open-loop ramp in a body direction (heading held by
    the wheel kinematics only). Returns tracking error at the end of the ramp
    (fraction of commanded speed), heading drift (deg) and peak bus current."""
    m, d, r = make_robot(p)
    u = np.array([math.cos(math.radians(direction_deg)), math.sin(math.radians(direction_deg))])
    t_ramp = v_final / accel
    dt = m.opt.timestep
    yaw0 = r.pose()[2]
    i_peak = 0.0
    tilt = 0.0
    n = int(round((t_ramp + 0.05) / dt))
    for k in range(n):
        v = min(v_final, accel * (k + 1) * dt)
        r.wheel_cmd = r.kin.wheel_speeds([*(v * u), 0.0])
        r.step()
        i_peak = max(i_peak, r.i_total)
        q = d.sensor("base_quat").data
        tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[1] ** 2 + q[2] ** 2))))))
    yaw = r.pose()[2]
    vb = body_velocity(d, yaw)
    # commanded velocity is in the body frame; compare in the body frame
    err = float(np.linalg.norm(vb - v_final * u) / v_final)
    drift = math.degrees(abs(wrap_angle(yaw - yaw0)))
    accel_run.last_tilt = tilt
    return err, drift, i_peak


def max_clean_accel(p: SimParams, direction_deg: float, lo=1.0, hi=9.0, iters=6, err_tol=0.05, drift_tol=2.0,
                    tilt_tol=2.0):
    """Highest acceleration that tracks within err_tol, with < drift_tol deg heading
    drift and < tilt_tol deg body tilt (a wheel lifting means the robot is tipping)."""
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        err, drift, _ = accel_run(p, direction_deg, mid)
        if err < err_tol and drift < drift_tol and accel_run.last_tilt < tilt_tol:
            lo = mid
        else:
            hi = mid
    return lo


def analytic_max_accel(p: SimParams, direction_deg: float, cg_height: float, cg_xy=(0.0, 0.0)) -> float:
    """Quasi-static traction / tip limit for a kiwi base.

    Wheel i at angle th_i, radius L, CG at (c, h):
      normal load N_i = mg/3 + (2/3)(m g / L)(c . r_i) - (2/3)(m h / L)(a . r_i)
      traction     f_i = (2/3) m (a . t_i)
    Slip when |f_i| > mu N_i; tip when N_i < 0.
    """
    L, g, mu = p.wheel_base_radius, 9.81, p.mu
    u = np.array([math.cos(math.radians(direction_deg)), math.sin(math.radians(direction_deg))])
    c = np.asarray(cg_xy, dtype=float)
    best = float("inf")
    for th in np.radians(p.wheel_angles_deg):
        r_hat = np.array([math.cos(th), math.sin(th)])
        t_hat = np.array([-math.sin(th), math.cos(th)])
        n0 = 1 / 3 + 2 / 3 * float(c @ r_hat) / L          # static load share (x mg)
        lt = 2 / 3 * cg_height / L * float(u @ r_hat)       # load lost per unit a/g
        ft = 2 / 3 * abs(float(u @ t_hat))                  # traction needed per unit a/g
        den = ft + mu * lt
        if den > 1e-9:
            best = min(best, mu * n0 * g / den)
        if lt > 1e-9:
            best = min(best, n0 * g / lt)                   # this wheel unloads completely: tip
    return best


def cg_height(p: SimParams, with_xy: bool = False):
    """(CG height, mass) of the robot, plus the CG's (x, y) offset in the robot frame if asked."""
    m = build_model(p.with_(home_xy=(0.0, 0.0)))
    d = mujoco.MjData(m)
    adr = m.joint("kiwi").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    kiwi = m.body("kiwi").id
    c = d.subtree_com[kiwi] - d.xpos[kiwi]
    if with_xy:
        return float(c[2]), float(m.body_subtreemass[kiwi]), (float(c[0]), float(c[1]))
    return float(c[2]), float(m.body_subtreemass[kiwi])


def goto_run(p: SimParams, target_xy, accel: float, timeout: float = 2.0, robot=None):
    """Point-to-point move with the base controller; returns time to settle within 1 cm."""
    m, d, r = robot or make_robot(p)
    r.ctl.max_accel = accel
    start = r.pose()
    tgt = np.array([start[0] + target_xy[0], start[1] + target_xy[1], start[2]])
    t0 = d.time
    settled = None
    overshoot = 0.0
    d0 = float(np.hypot(*target_xy))
    while d.time - t0 < timeout:
        pose = r.pose()
        r.drive_to(pose, tgt)
        r.step()
        e = float(np.hypot(tgt[0] - pose[0], tgt[1] - pose[1]))
        prog = float(np.dot(pose[:2] - start[:2], np.array(target_xy) / d0))
        overshoot = max(overshoot, prog - d0)
        speed = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
        if e < 0.01 and speed < 0.03:
            settled = settled or d.time - t0
        else:
            settled = None
    return settled, overshoot, (m, d, r)


def save(name: str, data):
    (RESULTS / f"{name}.json").write_text(json.dumps(data, indent=1, default=float))
