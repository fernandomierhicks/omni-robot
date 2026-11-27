"""What square step can the arm robot always climb? (Nov 26). The arm robot as baselined (robot_export.arm_d435_params:
cad/v10's coarse model with the D435 on the riser, the AK40-10s, 140 mm v13 wheels, the pods, the CAD's mass) drives
up a square step onto a 0.8 m platform and down its far side, explicit rollers, the Python base controller at the
firmware's adult limits and no terrain assist (an edge no map knows: the worst case).

Swept: step height 10-70 mm, approach speed 0.15 / 0.4 / 1.0 m/s, body direction of travel 0-180 deg by 30 (the robot
is near mirror-symmetric left / right), and the edge's angle to the path 0 / 30 / 60 deg (square-on, then oblique: the
wheels meet it one at a time). Scored as base_trade.cross_job: across (up and down again), where it stopped, peak
tilt, airborne, flipped. "Always climbs" = every direction and edge angle crossed at that speed and at every lower
height.

  python step_climb.py               run (all cores), write results/step_climb.json, print the table
  python step_climb.py report        the table from the saved results
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import RESULTS, save, tilt_deg  # noqa: E402
from floor import Threshold  # noqa: E402
from launcher_bench import Bench  # noqa: E402
from omni_control import wrap_angle  # noqa: E402

OUT = RESULTS / "step_climb.json"
WORKERS = max(1, min(24, (os.cpu_count() or 4) - 2))
HEIGHTS_MM = (10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70)
SPEEDS = (0.15, 0.4, 1.0)
DIRS = (0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0)
EDGE_ANGLES = (0.0, 30.0, 60.0)
PLATFORM = 0.8                  # m deep (along its normal)
NEAR = 0.75                     # m: the step's near edge on the robot's path (it starts at x = -0.15)


def params(features):
    from robot_export import arm_d435_params
    return arm_d435_params(floor_features=features, floor_objects=(), wheel_model="rollers", limits="firmware",
                           base_max_speed=2.0, floor_patches=(), area_size=(8.0, 8.0))


def cross_job(args):
    h_mm, v, direction, angle = args
    c = math.cos(math.radians(angle))
    half = PLATFORM / 2 / c                       # the platform's half depth along the path (world x)
    cx = NEAR + half
    step = Threshold(cx, 0.0, yaw_deg=angle, length=3.5, height=h_mm / 1000, top=PLATFORM, ramp=0.0005, mu=0.85)
    x_far, x_end = cx + half, cx + half + 0.45
    b = Bench(params((step,)))
    m, d, r = b.m, b.d, b.r
    yaw = -math.radians(direction)                # driving along world +x is body direction `direction`
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    d.qpos[adr] = -0.15
    mujoco.mj_forward(m, d)
    b.run(0.3)
    dt = m.opt.timestep
    tilt = air = air_run = 0.0
    flipped = False
    t0, x_max = d.time, -1.0
    t_lim = 1.0 + (x_end + 0.15) / v * 2.5
    while d.time - t0 < t_lim:
        pose = r.pose()
        w = float(np.clip(4.0 * wrap_angle(yaw - pose[2]), -2.0, 2.0))
        r.drive_velocity((v, 0.0), w, pose[2], a_cap=2.0)
        r.step()
        x_max = max(x_max, float(pose[0]))
        if pose[0] > 0.2:
            t = tilt_deg(d)
            tilt = max(tilt, t)
            if t > 45:
                flipped = True
                break
            air_run = air_run + dt if np.all(r.wheel_loads() < 0.5) else 0.0
            air = max(air, air_run)
        if pose[0] > x_end:
            break
    pose = r.pose()
    crossed = bool(pose[0] > x_end and not flipped)
    where = "across" if crossed else ("flipped" if flipped else
                                      "before the edge" if x_max < NEAR - 0.15 else
                                      "climbing" if x_max < NEAR + 0.2 else
                                      "on it" if x_max < x_far + 0.15 else "coming off")
    return dict(height_mm=h_mm, speed=v, direction=direction, edge_angle=angle, crossed=crossed, where=where,
                x_max=round(x_max, 3), tilt=round(tilt, 1), airborne_ms=round(1000 * air), sim_s=round(d.time - t0, 2))


def report(rows):
    print("Square step up onto a 0.8 m platform and off it (arm robot, rollers, no terrain assist): crossed of "
          f"{len(DIRS)} body directions x {len(EDGE_ANGLES)} edge angles, peak tilt, how the rest stopped")
    for v in SPEEDS:
        print(f"  at {v:.2f} m/s")
        always, broken = 0, False
        for h in HEIGHTS_MM:
            rr = [x for x in rows if x["speed"] == v and x["height_mm"] == h]
            if not rr:
                continue
            n = sum(x["crossed"] for x in rr)
            fails = {}
            for x in rr:
                if not x["crossed"]:
                    fails[x["where"]] = fails.get(x["where"], []) + [f"{int(x['direction'])}/{int(x['edge_angle'])}"]
            st = "; ".join(f"{w}: {', '.join(v_)}" for w, v_ in fails.items())
            if n == len(rr) and not broken:
                always = h
            else:
                broken = True
            print(f"    {h:3d} mm  {n:2d}/{len(rr)}  tilt {max(x['tilt'] for x in rr):4.1f} deg  "
                  f"air {max(x['airborne_ms'] for x in rr):3d} ms  {('fails (body/edge deg): ' + st) if st else ''}")
        print(f"    -> always crosses up to {always} mm at {v:.2f} m/s")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        report(json.loads(OUT.read_text())["rows"])
        return
    jobs = [(h, v, a, e) for h in HEIGHTS_MM for v in SPEEDS for a in DIRS for e in EDGE_ANGLES]
    w0 = time.perf_counter()
    with ProcessPoolExecutor(WORKERS) as pool:
        rows = list(pool.map(cross_job, jobs, chunksize=1))
    print(f"{len(rows)} crossings in {time.perf_counter() - w0:.0f} s on {WORKERS} workers")
    save("step_climb", dict(rows=rows, heights_mm=HEIGHTS_MM, speeds=SPEEDS, dirs=DIRS, edge_angles=EDGE_ANGLES))
    report(rows)


if __name__ == "__main__":
    main()
