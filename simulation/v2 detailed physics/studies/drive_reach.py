"""Reach study: where the drive options actually differ.

drive_components.py shows every feasible combination catches 20/20 inside the
nominal 0.45 m zone. Here the zone is stretched to 0.65 m (the design doc's ~1 m
catch circle, minus margin) and moves are 0.6 m in the robot's worst traction
direction, so top speed and acceleration both matter.

Writes results/drive_reach.json.
"""

from __future__ import annotations

import math

import numpy as np

from common import SimParams, analytic_max_accel, cg_height, goto_run, save
from drive_components import catch_rate, top_speed

COMBOS = [("AK40-10", "goBILDA-96"), ("AK40-10", "Nexus-100"), ("AK40-10", "Nexus-127"),
          ("AK45-10", "goBILDA-96"), ("DM-J4310", "goBILDA-96"), ("GIM6010-8", "Nexus-100"),
          ("AK60-6", "Nexus-127")]


def main():
    rows = []
    for mk, wk in COMBOS:
        p = SimParams(drive_motor=mk, wheel=wk)
        h, mass, cxy = cg_height(p, with_xy=True)
        dirs = range(0, 360, 5)
        a_dir = [analytic_max_accel(p, a, h, cxy) for a in dirs]
        worst = list(dirs)[int(np.argmin(a_dir))]
        a = 0.85 * min(a_dir)
        v_cap = min(1.8, 0.95 * top_speed(p, 22.0))
        p_run = p.with_(base_max_speed=v_cap)
        u = (0.6 * math.cos(math.radians(worst)), 0.6 * math.sin(math.radians(worst)))
        t_worst, _, _ = goto_run(p_run, u, a, timeout=3.0)
        t_fwd, _, _ = goto_run(p_run, (0.6, 0.0), a, timeout=3.0)
        c = catch_rate(p_run, a, tosses=30, zone=0.65, seed=21)
        rows.append(dict(motor=mk, wheel=wk, accel=a, v_cap=v_cap, worst_dir=worst,
                         t_move_06_worst=t_worst, t_move_06_fwd=t_fwd, **{f"catch_{k}": v for k, v in c.items()}))
        print(f"{mk:10s} {wk:11s} a {a:.2f} vcap {v_cap:.2f}: 0.6 m move {t_fwd or float('nan'):.2f} s fwd / "
              f"{t_worst or float('nan'):.2f} s worst ({worst} deg); catch {c['caught']}/{c['tosses']} in 0.65 m zone", flush=True)
    sweep = []
    p = SimParams()
    for a in (1.5, 2.0, 2.5, 3.0, 3.5):
        c = catch_rate(p.with_(base_max_speed=1.8), a, tosses=30, zone=0.65, seed=21)
        sweep.append(dict(accel=a, **c))
        print(f"baseline, accel limit {a}: {c['caught']}/30", flush=True)
    save("drive_reach", dict(combos=rows, accel_sweep=sweep))


if __name__ == "__main__":
    main()
