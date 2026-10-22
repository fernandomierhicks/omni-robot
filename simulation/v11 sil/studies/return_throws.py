"""Return throws to a thrower 6-7 m away: does the roller feed throw back as accurately as v5's plunger?

Full catch game (expected cameras, firmware adult mode, the 6.5 m shallow throws of shallow_sweep.py at a 1.5 m apex),
with return throws: the robot catches, tilts to the 50 deg return elevation, sets the flywheels from the launcher
calibration and fires; the range bias is learned from the logged landings (catch_game.learn_return). Scored where the
ball comes down through the thrower's chest height (1.2 m). 4 runs x 8 tosses per feed. Writes results/return_throws.json.
"""

from __future__ import annotations

import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import RESULTS, SimParams, save  # noqa: E402
from catch_game import GameConfig  # noqa: E402
from robot_model import build_model  # noqa: E402
from shallow_sweep import AREA, EXPECTED, FW, FWG, HOME, R3, SPIN, V5, V6G, V6M, HumanToss  # noqa: E402

FEEDS = {"v5 plunger": ({**V5, **FW}, dict(FWG, max_catch_tilt_deg=15.0)),
         "v6 roller": ({**V6M, **FW}, dict(FWG, **V6G)),
         "v6 chute": ({**V6M, **FW, "feed": "chute"}, dict(FWG, **V6G)),
         "v6 roller, seat pulse": ({**V6M, **FW, "feed_seat_time": 0.06}, dict(FWG, **V6G)),
         "v6 roller, settle 0.3 s": ({**V6M, **FW}, dict(FWG, **V6G, fire_settle=0.3)),
         "v6 roller, seat + settle": ({**V6M, **FW, "feed_seat_time": 0.06}, dict(FWG, **V6G, fire_settle=0.3)),
         "v6 round 3 chute (GM3506, viscoelastic)": ({**R3, **FW}, dict(FWG, **V6G)),
         "v6 round 3 chute, elastic contacts": ({**R3, **FW, "contact_model": "elastic"}, dict(FWG, **V6G))}
DIST, APEX, RUNS, TOSSES = 6.5, 1.5, 4, 8


def run(args):
    name, seed = args
    pkw, gkw = FEEDS[name]
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - DIST - 0.3, 0.0), area_size=AREA, **pkw)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(APEX, APEX), return_throws=True,
                               perception="vision", vision_cfg=EXPECTED, toss_spin=SPIN), **gkw})
    game = HumanToss(m, d, p, cfg=cfg, seed=seed, verbose=False, dist=DIST)
    while len(game.results) < TOSSES or game.state not in ("wait",):
        game.step()
        if d.time > 200:
            break
    return dict(feed=name, seed=seed, tosses=[dict(caught=r.caught, launched=r.launched, err=r.return_error,
                                                   range_err=r.return_range_err) for r in game.results],
                bias=game.range_bias)


def main(names=None):
    todo = names or list(FEEDS)
    jobs = [(f, s) for f in todo for s in range(RUNS)]
    with ProcessPoolExecutor(min(len(jobs), os.cpu_count() or 4)) as pool:
        rows = list(pool.map(run, jobs))
    path = RESULTS / "return_throws.json"
    old = json.loads(path.read_text()) if path.exists() else []
    rows = [r for r in old if r["feed"] not in todo] + rows
    save("return_throws", rows)
    print(f"Return throws to a thrower {DIST} m away (chest height), {RUNS} runs x {TOSSES} tosses:")
    for f in [f for f in FEEDS if any(r["feed"] == f for r in rows)]:
        ts = [t for r in rows if r["feed"] == f for t in r["tosses"] if t["launched"] and np.isfinite(t["err"])]
        late = [t for r in rows if r["feed"] == f for t in r["tosses"][3:] if t["launched"] and np.isfinite(t["err"])]
        print(f"  {f:26s}: {len(ts)} returns, miss median {100 * np.median([t['err'] for t in ts]):.0f} cm "
              f"(after the first 3 of each run: {100 * np.median([t['err'] for t in late]):.0f} cm, 90% "
              f"{100 * np.percentile([t['err'] for t in late], 90):.0f} cm); |range error| median "
              f"{100 * np.median(np.abs([t['range_err'] for t in late])):.0f} cm; learned bias "
              f"{[round(100 * r['bias']) for r in rows if r['feed'] == f]} cm")


if __name__ == "__main__":
    main(sys.argv[1:] or None)
