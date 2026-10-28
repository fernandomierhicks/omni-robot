"""v11 (design.md 30): does the catch survive the move to real code? The 6-7 m catch game (home_floor.py B's grid:
3 distances x 4 apexes x seeds x 6 throws, the same throws for both) played two ways on the v11 robot:

  ref   the sim's own catch game (catch_game.CatchGame / shallow_sweep.HumanToss): the firmware's logic in Python on
        the true pose every 0.5 ms, the vision model's detector and filter, perfect timing
  sil   sil_game.SilCatchGame: the C++ firmware in the loop (DM-2325 CAN, DShot telemetry, the MCU's own odometry),
        the Jetson's code (pixel detector, filter on the MCU's odometry and IMU attitude, behavior at 100 Hz, the USB
        link with time sync), the same camera model

    python sil_catch.py ref | sil | both [--seeds N] [--floor flat|home] [--robot v11|cad]
    python sil_catch.py report

--robot cad: the robot as cad/v7 draws it since the head (design.md 12.1): the plate 96 mm over the deck (the module and
its pivot with it), the cameras on the head's bar (optical centres at x 192, z 211 mm, 30 deg up), the head's mass at
the front; the Jetson's code gets that robot's description (robot_export.robot_json) instead of robot_v11.json.
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
# the drawn robot: plate gap, head cameras, the head and face (cad/v7 head.py, display.py: ~0.38 kg at the front, est
# from the CAD's volumes; the sim's 0.08 kg of cameras stay)
ROBOTS = {"v11": {}, "cad": dict(plate_gap=0.096, camera_mount="head", camera_head_xz=(0.192, 0.211),
                                 extra_masses=((0.38, 0.170, 0.0, 0.205),)),
          # the v11 robot before the head (Oct 21-24: plate 65 mm over the deck, the cameras on the plate's front)
          "oct21": dict(plate_gap=0.065, camera_mount="plate_front"),
          # the drawn robot with the CAD's masses (robot_export.drawn_params: 8.19 kg, its CG, the 515 g wheels, no
          # rocker, the bumper ring; design.md 7 step 1); its point mass is worked out in robot_kw
          "drawn": None}


def robot_kw(robot) -> dict:
    """The SimParams overrides of a robot in ROBOTS."""
    if robot == "drawn":
        from robot_export import drawn_params
        return dict(mass_model="cad", extra_masses=drawn_params().extra_masses)
    return ROBOTS[robot]


def robot_json_path(robot):
    """The Jetson code's description of a robot other than the export's (None: robot_v11.json)."""
    return None if robot == "v11" else RESULTS / f"robot_{robot}.json"


APEXES = (0.75, 1.0, 1.5, 2.0)
TOSSES = 6


def _why(caught, pos_error, rim):
    return "caught" if caught else ("reach" if not np.isfinite(pos_error) or pos_error > rim else "bounce")


def ref_job(args):
    floor_name, dist, apex, seed, robot = args
    import mujoco
    from catch_game import GameConfig
    from catch_world import AREA, HOME, expected_cameras
    from home_floor import GAME_FLOORS
    from robot_export import v11_params
    from robot_model import build_model
    from shallow_sweep import FWG, V6G, HumanToss
    p = v11_params(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, **GAME_FLOORS[floor_name],
                   **robot_kw(robot))
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
    return dict(kind="ref", robot=robot, floor=floor_name, distance=dist, apex=apex, seed=seed, tosses=out,
                rtf=d.time / (time.perf_counter() - w0))


def sil_job(args):
    floor_name, dist, apex, seed, robot = args
    from catch_world import WorldConfig
    from home_floor import GAME_FLOORS
    from sil_game import SilCatchGame
    from omni_catch.robot_config import RobotConfig      # (on the path once sil_game is imported)
    rc = RobotConfig(robot_json_path(robot)) if robot != "v11" else None
    g = SilCatchGame(WorldConfig(dist=dist, apex_range=(apex, apex), seed=seed, floor=GAME_FLOORS[floor_name]), rc=rc,
                     **robot_kw(robot))
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
    return dict(kind="sil", robot=robot, floor=floor_name, distance=dist, apex=apex, seed=seed, tosses=out, **extra)


def jobs(floor_name, seeds, robot="v11"):
    return [(floor_name, dd, a, 9000 + 100 * s + 10 * DISTANCES.index(dd) + APEXES.index(a), robot)
            for s in range(seeds) for dd in DISTANCES for a in APEXES]


def _key(r):
    return r["kind"], r.get("robot", "v11"), r["floor"], r["seed"], r["distance"], r["apex"]


def write_robot_json(robot):
    """A robot as the Jetson's code reads it (robot_v11.json's format)."""
    from robot_export import _rounded, robot_json
    from robot_export import v11_params as vp
    RESULTS.mkdir(exist_ok=True)
    robot_json_path(robot).write_text(json.dumps(_rounded(robot_json(vp(**robot_kw(robot)))), indent=1) + "\n")


def load():
    return json.loads(OUT.read_text()) if OUT.exists() else []


def save(rows):
    RESULTS.mkdir(exist_ok=True)
    new = {_key(n) for n in rows}
    old = [r for r in load() if _key(r) not in new]
    OUT.write_text(json.dumps(old + rows, indent=1))


def report(rows=None):
    rows = rows or load()
    print("v11 SIL vs the sim's own catch game: the 6-7 m catch game, the same throws, firmware adult, expected cameras")
    for rb, fl in dict.fromkeys((r.get("robot", "v11"), r["floor"]) for r in rows):
        print(f"  robot {rb}:")
        ref = [r for r in rows if r["floor"] == fl and r["kind"] == "ref" and r.get("robot", "v11") == rb]
        sil = [r for r in rows if r["floor"] == fl and r["kind"] == "sil" and r.get("robot", "v11") == rb]
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
                      r.get("robot", "v11") == rb and (r["seed"], r["distance"], r["apex"]) in keys for t in r["tosses"]]
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
    ap.add_argument("--robot", choices=tuple(ROBOTS), default="v11")
    a = ap.parse_args()
    if a.what == "report":
        report()
        return
    if a.robot != "v11" and a.what in ("sil", "both"):
        write_robot_json(a.robot)
    todo = []
    if a.what in ("ref", "both"):
        todo += [(ref_job, j) for j in jobs(a.floor, a.seeds, a.robot)]
    if a.what in ("sil", "both"):
        todo += [(sil_job, j) for j in jobs(a.floor, a.seeds, a.robot)]
    w0 = time.perf_counter()
    with ProcessPoolExecutor(a.workers) as pool:
        futs = [pool.submit(fn, j) for fn, j in todo]
        rows = [f.result() for f in futs]
    save(rows)
    print(f"{len(rows)} jobs in {time.perf_counter() - w0:.0f} s")
    report()


if __name__ == "__main__":
    main()
