"""Catch-module rig for long, shallow throws: a static robot, the ball arriving fast and flat.

The 6-7 m throws of shallow_sweep.py reach the rim at 6.5-8 m/s, 27-71 deg from vertical (median 45). With exact
ball data the v5 robot is always in place, yet a third of them bounce out. This rig isolates the catch module:

  - the ball crosses the rim-center plane at speed V, THETA deg from vertical, coming from the front (the robot faces
    the incoming ball), at a horizontal offset from the rim center (positioning + prediction error);
  - the module is tilted to min(gain * theta, cap) before the ball arrives, and (dump) swung back upright
    dump_delay after the ball passes the rim, as the game does;
  - caught = the break-beam sees the ball 0.9 s later.

Configurations are (name, SimParams kwargs, gain, cap deg). Writes results/catch_rig.json.
  python catch_rig.py                 # the configurations in CONFIGS
  python catch_rig.py trace NAME      # print where each missed ball went, for one configuration
"""

from __future__ import annotations

import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from common import SimParams, save  # noqa: E402
from launcher_bench import Bench  # noqa: E402

SPEEDS = (6.5, 8.0)
ANGLES = (30.0, 40.0, 50.0, 60.0)
OFFSETS = [(0.0, 0.0)] + [(r * math.cos(a), r * math.sin(a)) for r in (0.04, 0.08)
                          for a in np.radians(np.arange(0, 360, 60))]

V5 = dict(funnel="cone55", feed="kicker", ball="foam-pickleball", pocket_pad=False, feed_motor="ideal",
          contact_model="elastic")      # rounds 1-2 rows: ideal feed motor, elastic contacts
RL = dict(V5, feed="roller", pocket_pad=True)      # v6 round 1 (round 2's chute: "v6 chute pocket")
PAD = dict(pocket_pad=True)
NET = dict(funnel="net")
IX = dict(hopper="indexer")
CONFIGS = [
    ("v5 module (plunger), tilt 60% capped 15", V5, 0.6, 15.0),
    ("v5 module, capped 25", V5, 0.6, 25.0),
    ("v5 + padded pocket, capped 15", {**V5, **PAD}, 0.6, 15.0),
    ("roller feed, capped 15", {**RL, "pocket_pad": False}, 0.6, 15.0),
    ("roller feed, capped 35", {**RL, "pocket_pad": False}, 0.6, 35.0),
    ("v6: roller + padded pocket, capped 35", {**RL, **PAD}, 0.6, 35.0),
    ("v6, 80% capped 40", {**RL, **PAD}, 0.8, 40.0),
    ("v6, 40% capped 35", {**RL, **PAD}, 0.4, 35.0),
    ("v6 + slack-net model, capped 35", {**RL, **PAD, **NET}, 0.6, 35.0),
    ("v6 chute pocket, capped 35", {**RL, **PAD, "feed": "chute"}, 0.6, 35.0),
    ("v6 + indexer, capped 35", {**RL, **PAD, **IX}, 0.6, 35.0),
    ("v6 + indexer, net roll 4 mm, capped 35", {**RL, **PAD, **IX, "funnel_roll": 0.004}, 0.6, 35.0),
    ("v6 round 3: chute, GM3506 feed, viscoelastic", dict(feed="chute", pocket_pad=True), 0.6, 35.0),
]


def trial(b: Bench, v: float, theta_deg: float, gain: float, cap: float, offset, dump_delay=0.03, trace=False):
    r, d, g = b.r, b.d, b.r.g
    th = math.radians(theta_deg)
    lo, hi = b.p.tilt_limits
    tilt = float(np.clip(gain * th, 0.0, min(hi, math.radians(cap))))
    b.set_tilt(math.degrees(tilt))
    x_rc, z_rc = g.rim_center_robot(tilt)
    bpos = d.sensor("base_pos").data
    target = np.array([bpos[0] + x_rc + offset[0], bpos[1] + offset[1], bpos[2] + z_rc])
    v_t = v * np.array([-math.sin(th), 0.0, -math.cos(th)])
    T = 0.06
    gz = np.array([0, 0, 9.81])
    p0 = target - v_t * T - 0.5 * gz * T * T
    v0 = v_t + gz * T
    d.qpos[b.bq:b.bq + 7] = [*p0, 1, 0, 0, 0]
    d.qvel[b.bv:b.bv + 6] = [*v0, 0, 0, 0]
    t_in, path = None, []
    t0 = d.time
    while d.time - t0 < 0.9:
        r.step()
        bp = d.sensor("ball_pos").data
        if t_in is None and bp[2] < r.funnel_center_world()[2]:
            t_in = d.time
        if t_in is not None and d.time - t_in > dump_delay:
            r.tilt_target = 0.0
        if trace and int(round((d.time - t0) / b.dt)) % 20 == 0:
            # ball in the module frame (x forward, z up the funnel axis, origin at the module base)
            mb = r.module
            q = d.xmat[mb].reshape(3, 3).T @ (bp - d.xpos[mb])
            path.append((round(d.time - t0, 3), *np.round(q * 1000).astype(int),
                         round(float(np.linalg.norm(d.sensor("ball_vel").data)), 2)))
    held = r.ball_aboard()
    d.qpos[b.bq:b.bq + 7] = [-2.2, 1.7, 0.037, 1, 0, 0, 0]
    d.qvel[b.bv:b.bv + 6] = 0
    r.gate_open = False
    b.run(0.05)
    return (bool(held), path) if trace else bool(held)


def run_config(cfg) -> dict:
    name, pkw, gain, cap = cfg
    t0 = time.perf_counter()
    b = Bench(SimParams(**pkw))
    res = {f"{v:g}/{th:g}": [trial(b, v, th, gain, cap, off) for off in OFFSETS] for v in SPEEDS for th in ANGLES}
    by_angle = {th: float(np.mean([x for v in SPEEDS for x in res[f"{v:g}/{th:g}"]])) for th in ANGLES}
    by_speed = {v: float(np.mean([x for th in ANGLES for x in res[f"{v:g}/{th:g}"]])) for v in SPEEDS}
    by_off = {}
    for rad in (0.0, 0.04, 0.08):
        idx = [i for i, o in enumerate(OFFSETS) if abs(math.hypot(*o) - rad) < 1e-6]
        by_off[rad] = float(np.mean([res[k][i] for k in res for i in idx]))
    g = b.r.g
    return dict(name=name, pkw=pkw, gain=gain, cap=cap, held=res, by_angle=by_angle, by_speed=by_speed,
                by_offset=by_off, overall=float(np.mean([x for v in res.values() for x in v])),
                rim_height=float(g.rim_center_robot(0.0)[1]), seconds=time.perf_counter() - t0)


def report(rows):
    for row in rows:
        print(f"{row['name']:48s} caught {100 * row['overall']:5.1f}% | by angle " +
              " ".join(f"{k:g}:{100 * v:3.0f}%" for k, v in row["by_angle"].items()) + " | by speed " +
              " ".join(f"{k:g}:{100 * v:3.0f}%" for k, v in row["by_speed"].items()) + " | by offset " +
              " ".join(f"{100 * k:g}cm:{100 * v:3.0f}%" for k, v in row["by_offset"].items()) +
              f" | rim {1000 * row['rim_height']:.0f} mm ({row['seconds']:.0f} s)", flush=True)


def main(configs=None):
    configs = configs or CONFIGS
    with ProcessPoolExecutor(min(len(configs), os.cpu_count() or 4)) as pool:
        rows = list(pool.map(run_config, configs))
    report(rows)
    save("catch_rig", dict(speeds=SPEEDS, angles=ANGLES, offsets=OFFSETS, configs=rows))
    return rows


def trace(name, speeds=SPEEDS, angles=ANGLES):
    cfg = next(c for c in CONFIGS if c[0] == name)
    _, pkw, gain, cap = cfg
    b = Bench(SimParams(**pkw))
    for v in speeds:
        for th in angles:
            for off in OFFSETS[:4]:
                held, path = trial(b, v, th, gain, cap, off, trace=True)
                if not held:
                    print(f"--- MISS v {v} theta {th} offset {np.round(np.array(off) * 100).astype(int)} cm: "
                          "t, x, y, z (mm, module frame), |v|")
                    print("    " + " ".join(str(p_) for p_ in path[::2]))


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "trace":
        trace(sys.argv[2])
    else:
        main()
