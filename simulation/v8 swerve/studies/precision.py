"""Precision on a flat floor (design.md 26.5): what the omni X4 and the swerve give SLAM and a future arm.

Ground truth here is the sim's pose; the robot's own estimate is wheel odometry (each base's kinematics integrated
from its measured wheel speeds, and steering angles for the swerve). Floor: uniform, mu 0.95 (est). Firmware limits.

  O  odometry drift: a 10-waypoint path (random within 2 m, headings changing by up to 90 deg), driven to each
     waypoint with the firmware's planner (accelerations up to the table), 3 seeds. Position error / distance and
     heading error at the end, plus the path's worst lateral error from the ideal kinematics.
  H  holding against an arm: holding a pose, a 20 N horizontal push at 0.35 m height for 1 s (a ~2 kg payload swung or
     a tool pushing a table), in 4 directions, then a 1 N m yaw torque; deflection, and the error once released.
  M  small moves: 5 / 10 / 20 mm steps in 4 directions; final error and settle time.

  python precision.py              # all
Writes results/precision.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from common import RESULTS, save, tilt_deg  # noqa: E402
from home_floor import FWP, params  # noqa: E402
from launcher_bench import Bench  # noqa: E402
from omni_control import wrap_angle  # noqa: E402

CONFIGS = ("v6", "v7 on DM-2325", "swerve3 v2, r 170")


def _bench(cfg):
    b = Bench(params(cfg, area_size=(14.0, 14.0), **FWP))
    b.run(0.4)
    return b


def _odo_step(r, odo, dt):
    tw = r.kin.body_twist(r.wheel_speeds())
    c, s = math.cos(odo[2]), math.sin(odo[2])
    odo += dt * np.array([c * tw[0] - s * tw[1], s * tw[0] + c * tw[1], tw[2]])


def odo_job(args):
    cfg, seed = args
    b = _bench(cfg)
    r, d = b.r, b.d
    dt = b.dt
    rng = np.random.default_rng(seed)
    p0 = r.pose().copy()
    odo = p0.copy()
    wps = [p0 + np.array([*rng.uniform(-1.0, 1.0, 2), rng.uniform(-1.5, 1.5)]) for _ in range(10)]
    dist, last = 0.0, p0[:2].copy()
    for wp in wps:
        t0 = d.time
        while d.time - t0 < 4.0:
            r.drive_to(r.pose(), wp)
            r.step()
            _odo_step(r, odo, dt)
            pose = r.pose()
            dist += float(np.linalg.norm(pose[:2] - last))
            last = pose[:2].copy()
            if np.linalg.norm(pose[:2] - wp[:2]) < 0.005 and abs(wrap_angle(pose[2] - wp[2])) < 0.01 and \
                    np.linalg.norm(d.sensor("base_linvel").data[:2]) < 0.02:
                break
    pose = r.pose()
    e = float(np.linalg.norm(odo[:2] - pose[:2]))
    return dict(config=cfg, kind="odo", seed=seed, dist=dist, err=e, err_pct=100 * e / dist,
                heading=math.degrees(abs(wrap_angle(odo[2] - pose[2]))))


def hold_job(args):
    """Hold a pose while a 20 N push acts at 0.35 m height (direction in the body frame), then a 1 N m yaw torque."""
    cfg, direction = args
    b = _bench(cfg)
    r, d, m = b.r, b.d, b.m
    target = r.pose().copy()
    bid = m.body("base").id
    out = dict(config=cfg, kind="hold", direction=direction)
    yaw = target[2]
    a = math.radians(direction) + yaw
    f = np.array([20 * math.cos(a), 20 * math.sin(a), 0.0])
    defl = rot = 0.0
    for phase, dur in (("push", 1.0), ("release", 1.0), ("twist", 1.0), ("release2", 1.0)):
        t0 = d.time
        while d.time - t0 < dur:
            d.xfrc_applied[bid] = 0.0
            if phase == "push":
                pos = d.xpos[bid] + np.array([0, 0, 0.35])
                mujoco_force(m, d, f, pos, bid)
            elif phase == "twist":
                d.xfrc_applied[bid, 5] = 1.0
            r.drive_to(r.pose(), target)
            r.step()
            pose = r.pose()
            if phase == "push":
                defl = max(defl, float(np.linalg.norm(pose[:2] - target[:2])))
            if phase == "twist":
                rot = max(rot, abs(math.degrees(wrap_angle(pose[2] - target[2]))))
        if phase == "release":
            out["after_push_mm"] = 1000 * float(np.linalg.norm(r.pose()[:2] - target[:2]))
    d.xfrc_applied[bid] = 0.0
    out.update(push_mm=1000 * defl, twist_deg=rot, after_twist_deg=abs(math.degrees(wrap_angle(r.pose()[2] - target[2]))),
               tilt=tilt_deg(d))
    return out


def mujoco_force(m, d, f, pos, bid):
    """A force f at world point pos on body bid, as xfrc_applied (force + the moment about the body's CoM)."""
    com = d.xipos[bid]
    d.xfrc_applied[bid, :3] = f
    d.xfrc_applied[bid, 3:] = np.cross(pos - com, f)


def micro_job(args):
    cfg, direction, step = args
    b = _bench(cfg)
    r, d = b.r, b.d
    p0 = r.pose().copy()
    u = np.array([math.cos(math.radians(direction)), math.sin(math.radians(direction))])
    target = np.array([*(p0[:2] + step * u), p0[2]])
    t0, t_ok = d.time, None
    while d.time - t0 < 2.0:
        r.drive_to(r.pose(), target)
        r.step()
        ok = np.linalg.norm(r.pose()[:2] - target[:2]) < 0.001
        t_ok = (t_ok or d.time - t0) if ok else None
    return dict(config=cfg, kind="micro", direction=direction, step=step,
                err_mm=1000 * float(np.linalg.norm(r.pose()[:2] - target[:2])), t=t_ok if t_ok is not None else np.nan)


def main(cfgs=CONFIGS):
    jobs_o = [(c, s) for c in cfgs for s in range(3)]
    jobs_h = [(c, a) for c in cfgs for a in (0.0, 45.0, 90.0, 180.0)]
    jobs_m = [(c, a, st) for c in cfgs for a in (0.0, 45.0, 90.0, 135.0) for st in (0.005, 0.01, 0.02)]
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        o = list(pool.map(odo_job, jobs_o, chunksize=1))
        h = list(pool.map(hold_job, jobs_h, chunksize=1))
        mm = list(pool.map(micro_job, jobs_m, chunksize=1))
    save("precision", dict(odo=o, hold=h, micro=mm))
    report(o, h, mm)


def report(o, h, mm):
    print("Flat floor. Odometry over a 10-waypoint path (3 seeds): position error % of distance, heading error. Holding "
          "a pose against a 20 N push at 0.35 m (4 directions): worst deflection, error after release; 1 N m yaw "
          "torque: rotation. Small moves 5 / 10 / 20 mm: worst final error, median time to within 1 mm")
    for c in dict.fromkeys(x["config"] for x in o):
        oo = [x for x in o if x["config"] == c]
        hh = [x for x in h if x["config"] == c]
        m_ = [x for x in mm if x["config"] == c]
        print(f"  {c:20s} odo {np.mean([x['err_pct'] for x in oo]):.2f}% (worst {max(x['err_pct'] for x in oo):.2f}%), "
              f"heading {max(x['heading'] for x in oo):.1f} deg over {np.mean([x['dist'] for x in oo]):.1f} m | "
              f"push {max(x['push_mm'] for x in hh):.1f} mm, after {max(x['after_push_mm'] for x in hh):.1f} mm; "
              f"twist {max(x['twist_deg'] for x in hh):.2f} deg | small moves "
              + " ".join(f"{1000 * st:.0f}mm: {max(x['err_mm'] for x in m_ if x['step'] == st):.2f} mm "
                         f"{np.nanmedian([x['t'] for x in m_ if x['step'] == st]):.2f} s" for st in (0.005, 0.01, 0.02)))


if __name__ == "__main__":
    main(tuple(sys.argv[1:]) or CONFIGS)
