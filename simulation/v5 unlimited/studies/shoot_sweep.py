"""Hopper mode accuracy: shooting at targets standing still vs on the move (shoot_game.py).

A 10 x 8 m gym, the magazine tube loaded with 6 balls, 6 targets per run (floor
bins at 0.15 m or marks at 1.2 m, 2-6 m away), 8 random layouts per robot:
  stop             - stop, face the target, fire once still
  move 0.5..3 m/s  - drive a 5 x 4 m loop at that speed and fire on the move
Range bias is learned from the logged hits as the run goes (as for return throws).
Writes results/shoot_sweep.json.
"""

from __future__ import annotations

import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from common import save  # noqa: F401  (sets up the import path)
import shoot_game  # noqa: E402

ROBOTS = {"stop": dict(mode="stop"), **{f"move {v:g} m/s": dict(mode="move", route_speed=v) for v in (0.5, 1.0, 2.0, 3.0)}}
LAYOUTS = 8


def targets(seed):
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < 6:
        x, y = rng.uniform(-4.7, 4.7), rng.uniform(-3.7, 3.7)
        if 2.0 < np.hypot(x, y) < 6.0:
            out.append((float(x), float(y), float(rng.choice([0.15, 1.2]))))
    return tuple(out)


def run_one(args):
    name, seed = args
    g = shoot_game.run(n=6, verbose=False, targets=targets(seed), **ROBOTS[name])
    return dict(robot=name, seed=seed, t_end=g.d.time,
                shots=[dict(miss=s.miss, range_err=s.range_err, cross_err=s.cross_err, distance=s.distance,
                            base_speed=s.base_speed, exit_speed=s.exit_speed, t=s.t_fire) for s in g.shots])


def main():
    jobs = [(n, s) for n in ROBOTS for s in range(LAYOUTS)]
    with ProcessPoolExecutor(min(26, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(run_one, jobs))
    save("shoot_sweep", rows)
    print("Hopper shooting, 6 balls at 6 targets 2-6 m away, 8 layouts per robot:")
    for name in ROBOTS:
        sh = [s for r in rows if r["robot"] == name for s in r["shots"]]
        miss = np.array([s["miss"] for s in sh])
        ok = np.isfinite(miss)
        span = [r["shots"][-1]["t"] - r["shots"][0]["t"] for r in rows if r["robot"] == name and len(r["shots"]) > 1]
        print(f"  {name:12s}: {ok.sum()}/{len(sh)} shots scored; miss median {100 * np.median(miss[ok]):.0f} cm, "
              f"90% {100 * np.percentile(miss[ok], 90):.0f} cm; base speed at firing median "
              f"{np.median([s['base_speed'] for s in sh]):.1f} m/s; 6 shots in {np.median(span):.1f} s "
              f"(median); |range| / |lateral| error median {100 * np.nanmedian(np.abs([s['range_err'] for s in sh])):.0f} / "
              f"{100 * np.nanmedian(np.abs([s['cross_err'] for s in sh])):.0f} cm")


if __name__ == "__main__":
    main()
