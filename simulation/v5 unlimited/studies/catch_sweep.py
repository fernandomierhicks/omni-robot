"""v5: what removing the firmware limits buys in the catch game, with the detailed vision model.

Grid (as v3): thrower distance (hand to the robot's waiting spot at x = 1 m) 1.5 /
2.25 / 3.0 m, apex 0.5-3 m above a 1.0 m release, landing 0.3 / 0.6 / 1.0 m from
the waiting spot in a random direction. 6 tosses per cell, the same throws for
every robot, in the 5 x 4 m arena.

Robots:
  firmware, stop, 1.4 m/s          - v3b's limits (85% traction table capped at 4.5, 40 m/s^3), stop-and-catch
  firmware, moving, 2 m/s, gate    - v3's proposed adult mode (20 cm clear-path gate, 3 m/s^2 braking with the ball)
  physics, stop                    - no firmware limits (3.4 m/s, full traction table, no jerk limit), stop-and-catch
  physics, moving                  - no firmware limits, moving catches; only rule: don't hit the netting or the thrower
  physics, moving, exact state     - the same with the exact ball state after 0.25 s (v3's old assumption)
  physics, moving, fast vision     - 200 fps, 20 ms latency, 90 deg lenses (vision_sweep.py's best)
All with perception="vision" (vision.py) unless noted.
Writes results/catch_sweep.json and .png.
"""

from __future__ import annotations

import itertools
import os
from concurrent.futures import ProcessPoolExecutor

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from common import RESULTS, SimParams, save  # noqa: E402
from catch_game import CatchGame, GameConfig  # noqa: E402
from robot_model import build_model  # noqa: E402

DISTANCES = (1.5, 2.25, 3.0)
APEXES = (0.5, 1.0, 2.0, 3.0)
OFFSETS = (0.3, 0.6, 1.0)
TOSSES = 6
HOME = (1.0, 0.0)
ROBOTS = {   # name: (SimParams kwargs, GameConfig kwargs)
    "firmware, stop, 1.4 m/s": (dict(limits="firmware"), dict(limits="firmware", catch_mode="stop", perception="vision")),
    "firmware, moving, 2 m/s, gate": (dict(limits="firmware", base_max_speed=2.0),
                                      dict(limits="firmware", catch_mode="moving", perception="vision")),
    "physics, stop": (dict(), dict(catch_mode="stop", perception="vision")),
    "physics, moving": (dict(), dict(catch_mode="moving", perception="vision")),
    "physics, moving, exact state after 0.25 s": (dict(), dict(catch_mode="moving", perception="truth")),
    "physics, moving, fast vision": (dict(camera_fovy_deg=90.0), dict(catch_mode="moving", perception="vision",
                                                                      vision_cfg="fast")),
}


def run_cell(args):
    robot, dist, apex, off, seed = args
    pkw, gkw = ROBOTS[robot]
    gkw = dict(gkw)
    if gkw.get("vision_cfg") == "fast":
        from vision import VisionConfig
        gkw["vision_cfg"] = VisionConfig(fps=200.0, latency_mean=0.020, latency_jitter=0.005)
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), **pkw)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(catch_zone_radius=off, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
                     wait_between=0.8, **gkw)
    game = CatchGame(m, d, p, cfg=cfg, seed=seed, verbose=False)
    while len(game.results) < TOSSES:
        game.step()
    rim_margin = p.funnel_rim_radius - p.ball_spec.radius
    out = []
    for r in game.results:
        why = "caught" if r.caught else ("reach" if not np.isfinite(r.pos_error) or r.pos_error > rim_margin else "bounce")
        out.append(dict(caught=bool(r.caught), why=why, pos_error=r.pos_error, catch_speed=r.catch_speed,
                        moving=bool(r.moving), gated=bool(r.gated), land_sigma=r.land_sigma, moved=r.moved))
    return dict(robot=robot, distance=dist, apex=apex, offset=off, tosses=out)


def _sel(rows, **kw):
    return [t for r in rows if all(r[k] == v for k, v in kw.items()) for t in r["tosses"]]


def report(rows):
    for rb in ROBOTS:
        sel = _sel(rows, robot=rb)
        k = sum(t["caught"] for t in sel)
        whys = {w: sum(t["why"] == w for t in sel) for w in ("reach", "bounce")}
        caught_off = [t["pos_error"] for t in sel if t["caught"]]
        mv = [t for t in sel if t["moving"]]
        print(f"\n== {rb}: {k}/{len(sel)} = {100 * k / len(sel):.0f}% caught; misses {whys}; funnel offset of catches "
              f"median {100 * np.nanmedian(caught_off):.1f} cm; moving catches {len(mv)} "
              f"({sum(t['caught'] for t in mv)} caught, speed up to {max([t['catch_speed'] for t in mv] or [0]):.1f} m/s); "
              f"gated {sum(t['gated'] for t in sel)}"
              + (f"; landing 1-sigma at commit median {100 * np.nanmedian([t['land_sigma'] for t in sel]):.1f} cm"
                 if "exact" not in rb else ""))
        for off in OFFSETS:
            print(f"  landing {off:.1f} m away (rows: thrower distance, columns: apex)   " +
                  "".join(f"{a:>7.1f}" for a in APEXES))
            for dist in DISTANCES:
                cells = [_sel(rows, robot=rb, distance=dist, apex=a, offset=off) for a in APEXES]
                print(f"     {dist:4.2f} m" + " " * 42 + "".join(f"{100 * np.mean([t['caught'] for t in c]):6.0f}%" for c in cells))


def plot(rows):
    robots = list(ROBOTS)
    fig, axes = plt.subplots(len(robots), len(OFFSETS), figsize=(10, 2.6 * len(robots)), squeeze=False)
    for i, rb in enumerate(robots):
        for j, off in enumerate(OFFSETS):
            grid = np.array([[np.mean([t["caught"] for t in _sel(rows, robot=rb, distance=dd, apex=a, offset=off)])
                              for a in APEXES] for dd in DISTANCES])
            ax = axes[i][j]
            ax.imshow(grid, vmin=0, vmax=1, cmap="RdYlGn", origin="lower", aspect="auto")
            for (y, x), v in np.ndenumerate(grid):
                ax.text(x, y, f"{100 * v:.0f}", ha="center", va="center", fontsize=8)
            ax.set_xticks(range(len(APEXES)), [f"{a:g}" for a in APEXES])
            ax.set_yticks(range(len(DISTANCES)), [f"{dd:g}" for dd in DISTANCES])
            ax.set_title(f"{rb}: landing {off:g} m away", fontsize=8)
            if i == len(robots) - 1:
                ax.set_xlabel("apex above release (m)")
            if j == 0:
                ax.set_ylabel("thrower distance (m)")
    fig.suptitle("Catch rate (%) in the 5 x 4 m arena, 6 tosses per cell", fontsize=10)
    fig.tight_layout()
    fig.savefig(RESULTS / "catch_sweep.png", dpi=110)


def main():
    import json
    import sys
    only = sys.argv[1:]                    # e.g. "moving, model": rerun these robots, keep the others' saved rows
    todo = [rb for rb in ROBOTS if not only or rb in only]
    cells = [(rb, dist, apex, off, j) for rb in todo
             for j, (dist, apex, off) in enumerate(itertools.product(DISTANCES, APEXES, OFFSETS))]
    with ProcessPoolExecutor(min(26, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(run_cell, cells, chunksize=1))
    if only and (RESULTS / "catch_sweep.json").exists():
        old = json.loads((RESULTS / "catch_sweep.json").read_text())["rows"]
        rows = [r for r in old if r["robot"] not in todo] + rows
        rows.sort(key=lambda r: list(ROBOTS).index(r["robot"]))
    save("catch_sweep", dict(distances=DISTANCES, apexes=APEXES, offsets=OFFSETS, tosses=TOSSES, rows=rows))
    report(rows)
    plot(rows)


if __name__ == "__main__":
    main()
