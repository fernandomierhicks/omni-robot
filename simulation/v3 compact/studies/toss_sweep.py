"""Toss sensitivity: how far from the baseline throw does catching hold up?

Grid: thrower distance (hand to the robot's waiting spot) x apex above the
release (0.5 m = a flat line drive ... 3 m = a high lob) x landing offset from
the waiting spot (exactly that far, random direction). Release height 1.0 m.
Perception is still ground truth after 0.25 s, so this maps the robot's own
limits: reach in the available time, funnel acceptance at the arrival angle and
speed, tilt range.

Robots: v3b as first reported (catch tilt up to 25 deg, brakes as soon as the
ball is in); the same with the funnel held upright; v3b with the catch fixes
(tilt capped at 15 deg, keeps coasting 0.2 s after the ball enters); and those
plus the direction-aware acceleration limit (params.ACCEL_TABLE_V3B, 3.0-4.5
m/s^2 instead of a flat 2.5).

Each failure is classified: "reach" (the funnel center was > 12.8 cm from the
ball at the rim: robot not under it in time) or "bounce" (it was under, but the
ball didn't stay). Writes results/toss_sweep.json and .png.
"""

from __future__ import annotations

import itertools
import math
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

DISTANCES = (1.5, 2.5, 3.5, 4.5)
APEXES = (0.5, 1.0, 2.0, 3.0)
OFFSETS = (0.3, 0.6, 1.0)
TOSSES = 6
HOME = (1.5, 0.0)
from params import ACCEL_TABLE_V3B  # noqa: E402

ROBOTS = {
    "v3b as first reported": (dict(accel_table=None), dict(coast_after_catch=0.0, max_catch_tilt_deg=25.0)),
    "v3b, funnel upright": (dict(tilt=False, accel_table=None), dict(align_tilt=False)),
    "v3b + catch fixes": (dict(accel_table=None), dict()),
    "v3b + catch fixes + direction-aware accel": (dict(accel_table=ACCEL_TABLE_V3B), dict()),
}


def run_cell(args):
    robot, dist, apex, off, seed = args
    pkw, gkw = ROBOTS[robot]
    p = SimParams(area_size=(8.0, 6.0), home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), **pkw)
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
        out.append(dict(caught=bool(r.caught), why=why, pos_error=r.pos_error, arrival_deg=r.arrival_deg,
                        arrival_speed=r.arrival_speed, flight_time=r.flight_time, tilt_deg=r.tilt_deg, moved=r.moved))
    return dict(robot=robot, distance=dist, apex=apex, offset=off, tosses=out)


def main():
    # same seed per cell for every robot, so they see the same throws
    cells = [(rb, dist, apex, off, j) for rb in ROBOTS
             for j, (dist, apex, off) in enumerate(itertools.product(DISTANCES, APEXES, OFFSETS))]
    with ProcessPoolExecutor(min(26, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(run_cell, cells, chunksize=2))
    save("toss_sweep", dict(distances=DISTANCES, apexes=APEXES, offsets=OFFSETS, tosses=TOSSES, rows=rows))
    report(rows)
    plot(rows)


def _rate(rows, **kw):
    sel = [t for r in rows if all(r[k] == v for k, v in kw.items()) for t in r["tosses"]]
    return float(np.mean([t["caught"] for t in sel])) if sel else float("nan"), sel


def report(rows):
    for rb in ROBOTS:
        tot, sel = _rate(rows, robot=rb)
        whys = {w: sum(t["why"] == w for t in sel) for w in ("reach", "bounce")}
        print(f"\n== {rb}: {100 * tot:.0f}% caught over the grid ({len(sel)} tosses; misses: {whys})")
        for off in OFFSETS:
            print(f"  landing {off:.1f} m from the waiting spot   (rows: thrower distance, columns: apex above release)")
            print("          " + "".join(f"{a:>8.1f}" for a in APEXES))
            for dist in DISTANCES:
                cells = [_rate(rows, robot=rb, distance=dist, apex=a, offset=off)[0] for a in APEXES]
                print(f"   {dist:4.1f} m " + "".join(f"{100 * c:7.0f}%" for c in cells))
    # what the throws look like at the rim (from the trajectory itself, landing at the waiting spot)
    from ball_flight import BallFlight
    p = SimParams()
    b = p.ball_spec
    fl = BallFlight(b.mass, b.radius, b.drag_cd, p.air_density, p.air_viscosity)
    rim = 0.39
    print("\nThe throws at the rim (angle from vertical / speed / time from release; rows: distance, columns: apex):")
    print("          " + "".join(f"{a:>20.1f}" for a in APEXES))
    for dist in DISTANCES:
        cells = []
        for a in APEXES:
            hand = np.array([0.0, 0.0, 1.0])
            v0 = fl.solve_toss(hand, [dist, 0.0, rim], a)
            _, v, t = fl.crossing_state(hand, v0, rim)
            cells.append(f"{math.degrees(math.atan2(math.hypot(v[0], v[1]), -v[2])):3.0f} deg {np.linalg.norm(v):3.1f} m/s {t:3.2f} s")
        print(f"   {dist:4.1f} m " + "".join(f"{c:>20s}" for c in cells))
    print("Bounce-outs by cell (robot under the ball but it didn't stay):")
    for rb in ROBOTS:
        cells = [(r["distance"], r["apex"], r["offset"], sum(t["why"] == "bounce" for t in r["tosses"]))
                 for r in rows if r["robot"] == rb]
        print(f"  {rb}: " + ", ".join(f"{d:g} m/apex {a:g}/off {o:g}: {n}" for d, a, o, n in cells if n))


def plot(rows):
    fig, axes = plt.subplots(len(ROBOTS), len(OFFSETS), figsize=(11, 3.2 * len(ROBOTS)), squeeze=False)
    for i, rb in enumerate(ROBOTS):
        for j, off in enumerate(OFFSETS):
            grid = np.array([[_rate(rows, robot=rb, distance=dist, apex=a, offset=off)[0] for a in APEXES]
                             for dist in DISTANCES])
            ax = axes[i][j]
            ax.imshow(grid, vmin=0, vmax=1, cmap="RdYlGn", origin="lower", aspect="auto")
            for (y, x), v in np.ndenumerate(grid):
                ax.text(x, y, f"{100 * v:.0f}", ha="center", va="center", fontsize=8)
            ax.set_xticks(range(len(APEXES)), [f"{a:g}" for a in APEXES])
            ax.set_yticks(range(len(DISTANCES)), [f"{d:g}" for d in DISTANCES])
            ax.set_title(f"{rb}: landing {off:g} m away", fontsize=9)
            if i == len(ROBOTS) - 1:
                ax.set_xlabel("apex above release (m)")
            if j == 0:
                ax.set_ylabel("thrower distance (m)")
    fig.suptitle("Catch rate (%) by throw (6 tosses per cell, perfect perception)", fontsize=10)
    fig.tight_layout()
    fig.savefig(RESULTS / "toss_sweep.png", dpi=120)


if __name__ == "__main__":
    main()
