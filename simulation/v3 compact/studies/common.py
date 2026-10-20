"""Shared scenario helpers for the v3 studies."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from omni_control import wrap_angle  # noqa: E402
from params import SimParams  # noqa: E402
from robot import Robot  # noqa: E402
from robot_model import build_model, geometry, mass_properties  # noqa: E402
from traction import Base  # noqa: E402

RESULTS = HERE / "results"
RESULTS.mkdir(exist_ok=True)


def make_robot(p: SimParams, settle: float = 0.3):
    """Robot at the area center facing +x, settled on its wheels."""
    p = p.with_(home_xy=(0.0, 0.0))
    m = build_model(p)
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    bq = m.joint("ball").qposadr[0]
    d.qpos[bq:bq + 3] = [-2.2, 1.7, p.ball_spec.radius]
    mujoco.mj_forward(m, d)
    r = Robot(m, d, p)
    for _ in range(int(settle / m.opt.timestep)):
        r.step()
    return m, d, r


def body_velocity(d, yaw):
    v = d.sensor("base_linvel").data
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])


def tilt_deg(d) -> float:
    q = d.sensor("base_quat").data
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[1] ** 2 + q[2] ** 2)))))


def accel_run(p: SimParams, direction_deg: float, accel: float, v_final: float = 1.0, jerk: float = 40.0,
              robot=None):
    """Open-loop velocity ramp in a body direction: acceleration rises at `jerk`, holds at `accel`.
    Returns a dict: tracking error at the end (fraction of the commanded speed), heading drift (deg),
    peak body tilt (deg), smallest wheel load (N, 0 = a wheel lifted) and peak bus current."""
    m, d, r = robot or make_robot(p)
    u = np.array([math.cos(math.radians(direction_deg)), math.sin(math.radians(direction_deg))])
    dt = m.opt.timestep
    yaw0 = r.pose()[2]
    v = a = 0.0
    i_peak = tilt = 0.0
    n_min = float("inf")
    lifted_ms = 0.0
    e_track = 0.0
    t = 0.0
    t_end = None
    while t_end is None or t < t_end + 0.05:
        # jerk-limited ramp to accel, hold, then ramp down so v lands on v_final
        v_left = v_final - v
        if a > 0 and v_left <= a * a / (2 * jerk) + 1e-9:
            a = max(0.0, a - jerk * dt)
        else:
            a = min(accel, a + jerk * dt) if jerk > 0 else accel
        v = min(v_final, v + a * dt)
        if v >= v_final and t_end is None:
            t_end = t
        r.a_body_cmd = a * u
        r.wheel_cmd = r.kin.wheel_speeds([*(v * u), 0.0])
        r.step()
        t += dt
        # base velocity vs command during the ramp: with 4+ wheels one wheel can slip while the
        # others keep tracking, so the end-of-ramp error alone misses slip
        e_track = max(e_track, float(np.linalg.norm(body_velocity(d, r.pose()[2]) - v * u)))
        i_peak = max(i_peak, r.i_total)
        tilt = max(tilt, tilt_deg(d))
        loads = r.wheel_loads()
        n_min = min(n_min, float(np.min(loads)))
        if np.min(loads) < 0.5:
            lifted_ms += dt * 1000
    yaw = r.pose()[2]
    vb = body_velocity(d, yaw)
    err = float(np.linalg.norm(vb - v_final * u) / v_final)
    drift = math.degrees(abs(wrap_angle(yaw - yaw0)))
    return dict(err=err, e_track=e_track, drift=drift, tilt=tilt, n_min=n_min, lifted_ms=lifted_ms, i_peak=i_peak)


def max_clean_accel(p: SimParams, direction_deg: float, lo=1.0, hi=10.0, iters=7, err_tol=0.05, track_tol=0.05,
                    drift_tol=2.0, tilt_tol=2.0, jerk=40.0):
    """Highest acceleration with no base slip (velocity within track_tol m/s of the command
    throughout the ramp, and within err_tol of it at the end), < drift_tol deg heading drift and
    < tilt_tol deg body tilt. Also reports what failed first just above it."""
    why = "none"
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        r = accel_run(p, direction_deg, mid, jerk=jerk)
        ok = r["err"] < err_tol and r["e_track"] < track_tol and r["drift"] < drift_tol and r["tilt"] < tilt_tol
        if ok:
            lo = mid
        else:
            hi = mid
            why = "tip" if r["tilt"] >= tilt_tol else "slip"
    return lo, why


def traction_base(p: SimParams, tilt: float = 0.0, **kw) -> Base:
    """The analytic model with this robot's mass properties."""
    m = build_model(p.with_(home_xy=(0.0, 0.0)))
    mass, cg = mass_properties(m, p, tilt)
    return Base(p.wheel_angles_deg, p.wheel_base_radius, float(cg[2]), float(cg[0]), float(cg[1]), p.mu, **kw)


def save(name: str, data):
    (RESULTS / f"{name}.json").write_text(json.dumps(data, indent=1, default=float))
