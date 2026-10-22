"""v11 (design.md 30): does the catch survive the move to real code? The 6-7 m catch game (home_floor.py B's grid:
3 distances x 4 apexes x seeds x 6 throws, the same throws for both) played two ways on the v11 robot:

  ref   the sim's own catch game (catch_game.CatchGame / shallow_sweep.HumanToss): the firmware's logic in Python on
        the true pose every 0.5 ms, the vision model's detector and filter, perfect timing
  sil   sil_game.SilCatchGame: the C++ firmware in the loop (DM-2325 CAN, DShot telemetry, the MCU's own odometry),
        the Jetson's code (pixel detector, filter on the MCU's odometry and IMU attitude, behavior at 100 Hz, the USB
        link with time sync), the same camera model

    python sil_catch.py ref | sil | both [--seeds N] [--floor flat|home]
    python sil_catch.py report
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

RESULTS = HERE / "results"
OUT = RESULTS / "sil_catch.json"
DISTANCES = (6.0, 6.5, 7.0)
APEXES = (0.75, 1.0, 1.5, 2.0)
TOSSES = 6


def _why(caught, pos_error, rim):
    return "caught" if caught else ("reach" if not np.isfinite(pos_error) or pos_error > rim else "bounce")


def ref_job(args):
    floor_name, dist, apex, seed = args
    import mujoco
    from catch_game import GameConfig
    from catch_world import AREA, HOME, expected_cameras
    from home_floor import GAME_FLOORS
    from robot_export import v11_params
    from robot_model import build_model
    from shallow_sweep import FWG, V6G, HumanToss
    p = v11_params(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, **GAME_FLOORS[floor_name])
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
                               wait_between=1.2, perception="vision", vision_cfg=expected_cameras(), toss_spin=6.0),
                        **FWG, **V6G})
    game = HumanToss(m, d, p, cfg=cfg, seed=seed, verbose=False, dist=dist)
    rim = p.funnel_catch_radius - p.ball_spec.radius
    w0 = time.perf_counter()
    while len(game.results) < TOSSES and d.time < 150:
        game.step()
    out = [dict(caught=bool(r.caught), why=_why(r.caught, r.pos_error, rim), pos_error=float(r.pos_error),
                catch_speed=float(r.catch_speed), zone=float(r.zone_dist)) for r in game.results]
    return dict(kind="ref", floor=floor_name, distance=dist, apex=apex, seed=seed, tosses=out,
                rtf=d.time / (time.perf_counter() - w0))


def sil_job(args):
    floor_name, dist, apex, seed = args
    from catch_world import WorldConfig
    from home_floor import GAME_FLOORS
    from sil_game import SilCatchGame
    g = SilCatchGame(WorldConfig(dist=dist, apex_range=(apex, apex), seed=seed, floor=GAME_FLOORS[floor_name]))
    res = g.run(TOSSES, t_max=150)
    p = g.world.p
    rim = p.funnel_catch_radius - p.ball_spec.radius
    out = [dict(caught=bool(r["caught"]), why=_why(r["caught"], r["pos_error"], rim), pos_error=float(r["pos_error"]),
                catch_speed=float(r["catch_speed"]), zone=float(r["zone_dist"])) for r in res]
    st = g.link.host.stats
    extra = dict(rtf=g.world.d.time / max(g.wall, 1e-9), link_errors=st.errors,
                 sync_err_us=1e6 * abs(g.link.host.sync.offset_ns * 1e-9 - g.robot.boot_offset),
                 fw_faults=g.link.host.state.fault_names() if g.link.host.state else [])
    g.world.close()
    return dict(kind="sil", floor=floor_name, distance=dist, apex=apex, seed=seed, tosses=out, **extra)


def jobs(floor_name, seeds):
    return [(floor_name, dd, a, 9000 + 100 * s + 10 * DISTANCES.index(dd) + APEXES.index(a))
            for s in range(seeds) for dd in DISTANCES for a in APEXES]


def load():
    return json.loads(OUT.read_text()) if OUT.exists() else []


def save(rows):
    RESULTS.mkdir(exist_ok=True)
    old = [r for r in load() if not any((r["kind"], r["floor"], r["seed"], r["distance"], r["apex"]) ==
                                        (n["kind"], n["floor"], n["seed"], n["distance"], n["apex"]) for n in rows)]
    OUT.write_text(json.dumps(old + rows, indent=1))


def report(rows=None):
    rows = rows or load()
    print("v11 SIL vs the sim's own catch game: the 6-7 m catch game, the same throws, firmware adult, expected cameras")
    for fl in dict.fromkeys(r["floor"] for r in rows):
        ref = [r for r in rows if r["floor"] == fl and r["kind"] == "ref"]
        sil = [r for r in rows if r["floor"] == fl and r["kind"] == "sil"]
        keys = {(r["seed"], r["distance"], r["apex"]) for r in ref} & {(r["seed"], r["distance"], r["apex"]) for r in sil}
        for name, rr in (("ref (Python firmware, true pose)", ref), ("sil (C++ firmware, Jetson code)", sil)):
            rr = [r for r in rr if (r["seed"], r["distance"], r["apex"]) in keys]
            ts = [t for r in rr for t in r["tosses"]]
            if not ts:
                continue
            k = np.mean([t["caught"] for t in ts])
            n = len(ts)
            se = math.sqrt(max(k * (1 - k), 1e-9) / n)
            off = np.array([t["pos_error"] for t in ts])
            off = off[np.isfinite(off)]
            why = {w: sum(t["why"] == w for t in ts) for w in ("reach", "bounce")}
            print(f"  {fl:5s} {name:34s} {100 * k:5.1f}% +/- {100 * se:.1f} of {n} (not in place {why['reach']}, "
                  f"bounced {why['bounce']}), funnel offset median {100 * np.median(off):.1f} cm, "
                  f"95% {100 * np.percentile(off, 95):.1f} cm, real time x{np.median([r['rtf'] for r in rr]):.2f}")
        for a in APEXES:
            cells = []
            for name in ("ref", "sil"):
                ts = [t for r in rows if r["floor"] == fl and r["kind"] == name and r["apex"] == a and
                      (r["seed"], r["distance"], r["apex"]) in keys for t in r["tosses"]]
                cells.append(f"{name} {100 * np.mean([t['caught'] for t in ts]):.0f}%" if ts else f"{name} -")
            print(f"      apex {a:4.2f} m: " + ", ".join(cells))
        sil_rows = [r for r in sil if (r["seed"], r["distance"], r["apex"]) in keys]
        if sil_rows:
            print(f"      sil link errors {sum(r['link_errors'] for r in sil_rows)}, time sync error median "
                  f"{np.median([r['sync_err_us'] for r in sil_rows]):.0f} us, firmware faults at the end: "
                  f"{sorted({f for r in sil_rows for f in r['fw_faults']}) or 'none'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("ref", "sil", "both", "report"))
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--floor", default="flat")
    ap.add_argument("--workers", type=int, default=min(27, os.cpu_count() or 4))
    a = ap.parse_args()
    if a.what == "report":
        report()
        return
    todo = []
    if a.what in ("ref", "both"):
        todo += [(ref_job, j) for j in jobs(a.floor, a.seeds)]
    if a.what in ("sil", "both"):
        todo += [(sil_job, j) for j in jobs(a.floor, a.seeds)]
    w0 = time.perf_counter()
    with ProcessPoolExecutor(a.workers) as pool:
        futs = [pool.submit(fn, j) for fn, j in todo]
        rows = [f.result() for f in futs]
    save(rows)
    print(f"{len(rows)} jobs in {time.perf_counter() - w0:.0f} s")
    report()


if __name__ == "__main__":
    main()
