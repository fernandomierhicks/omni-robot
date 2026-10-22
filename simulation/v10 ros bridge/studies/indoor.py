"""Indoor play: a standard one-storey ceiling over the 6-7 m shallow throws, both ways.

A ceiling at 2.44 m (8 ft) or 2.7 m (9 ft) is in the scene: balls that reach it bounce off. The thrower's lobs are
limited to what clears it (apex above the 1 m release up to 1.25 m in an 8 ft room, 1.5 m at 9 ft), with the human
scatter of shallow_sweep.py. The robot (v6, firmware adult mode, expected cameras) catches and throws back to chest
height. Return planners:
  "ceiling-aware": GameConfig.ceiling = the room; the steepest elevation from 50 deg down to 30 whose apex stays
                   0.25 m below the ceiling;
  "fixed 50 deg":  v5/v6's lob, which peaks ~3 m high at 7 m.
3 distances x the apexes x 2 seeds x 6 tosses per robot and room. Writes results/indoor.json.
"""

from __future__ import annotations

import itertools
import os
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import SimParams, save  # noqa: E402
from catch_game import GameConfig  # noqa: E402
from robot_model import build_model  # noqa: E402
from shallow_sweep import AREA, EXPECTED, FW, FWG, HOME, SPIN, V6G, V6M, HumanToss  # noqa: E402

ROOMS = {"8 ft (2.44 m)": (2.44, (0.75, 1.0, 1.25)), "9 ft (2.7 m)": (2.70, (0.75, 1.0, 1.25, 1.5))}
DISTANCES = (6.0, 6.5, 7.0)
SEEDS, TOSSES = 2, 6
PLANNERS = {"ceiling-aware": True, "fixed 50 deg": False}


def run_cell(args):
    room, planner, dist, apex, seed = args
    hc, _ = ROOMS[room]
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, ceiling_height=hc,
                  **{**V6M, **FW, "feed": "chute"})
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(apex, apex), return_throws=True,
                               perception="vision", vision_cfg=EXPECTED, toss_spin=SPIN,
                               ceiling=hc if PLANNERS[planner] else None), **FWG, **V6G})
    game = HumanToss(m, d, p, cfg=cfg, seed=seed, verbose=False, dist=dist)
    while len(game.results) < TOSSES or game.state != "wait":
        game.step()
        if d.time > 200:
            break
    rows = []
    for r in game.results:
        rows.append(dict(caught=bool(r.caught), launched=bool(r.launched), toss_top=r.toss_top, ret_err=r.return_error,
                         ret_top=r.return_top, elev=r.return_elev, speed=r.return_speed, rpm=r.return_rpm,
                         arr_speed=r.return_arrival_speed, pos_error=r.pos_error))
    return dict(room=room, planner=planner, distance=dist, apex=apex, seed=seed, ceiling=hc, tosses=rows)


def report(out):
    print("Indoor: v6 robot, firmware adult mode, expected cameras; thrower 6-7 m away; balls bounce off the ceiling")
    for room in ROOMS:
        hc = ROOMS[room][0]
        for pl in PLANNERS:
            ts = [t for r in out if r["room"] == room and r["planner"] == pl for t in r["tosses"]]
            if not ts:
                continue
            hit_in = sum(t["toss_top"] >= hc - 0.003 for t in ts)
            rt = [t for t in ts if t["launched"]]
            hit_out = sum(np.isfinite(t["ret_top"]) and t["ret_top"] >= hc - 0.003 for t in rt)
            err = np.array([t["ret_err"] for t in rt], float)
            ok = np.isfinite(err)
            print(f"  {room}, {pl:13s}: caught {100 * np.mean([t['caught'] for t in ts]):3.0f}% of {len(ts)} "
                  f"({hit_in} incoming hit the ceiling) | {len(rt)} returns: {hit_out} hit the ceiling, miss median "
                  f"{100 * np.median(err[ok]):.0f} cm, 90% {100 * np.percentile(err[ok], 90):.0f} cm | elevation "
                  f"{np.nanmin([t['elev'] for t in rt]):.0f}-{np.nanmax([t['elev'] for t in rt]):.0f} deg, highest ball top "
                  f"{np.nanmax([t['ret_top'] for t in rt]):.2f} m, exit {np.nanmedian([t['speed'] for t in rt]):.1f} m/s "
                  f"(max {np.nanmax([t['speed'] for t in rt]):.1f}), flywheels up to {np.nanmax([t['rpm'] for t in rt]):.0f} rpm, "
                  f"arrives at the hands {np.nanmedian([t['arr_speed'] for t in rt]):.1f} m/s")


def main():
    cells = [(room, pl, dd, a, 200 * s + 10 * DISTANCES.index(dd) + ROOMS[room][1].index(a))
             for room in ROOMS for pl in PLANNERS for dd in DISTANCES for a in ROOMS[room][1] for s in range(SEEDS)]
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        out = list(pool.map(run_cell, cells, chunksize=1))
    save("indoor", out)
    report(out)


if __name__ == "__main__":
    main()
