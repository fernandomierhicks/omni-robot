"""v10 (design.md 28): the ROS 2 stack against the sim, from the result files the ROS runs write (studies/results/):

  ros_map_<name>.json         map quality (omni.sh auto-map / score-map; map_quality.py)
  ros_tour[_<variant>]_s<seed>.json   localization along v9's house tour (omni.sh tour; omni_bridge/scorer.py)
  ros_nav_goals*.json         Nav2 goals through the doorways and across the rugs (omni.sh goals)

  python ros_report.py            all of them, as tables comparable with design.md 27.3
"""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

import numpy as np

RES = Path(__file__).resolve().parent / "results"
NAMES = {"ros_map": "ROS: slam_toolbox localization + robot_localization (map -> base_link)",
         "ros_map_rel": "  the same, against the map (its frame's best-fit offset removed)",
         "ros_odom": "ROS: robot_localization alone, wheels + BNO085 (odom -> base_link)",
         "v9:wheels_imu": "v9 EKF, wheels + IMU (same readings)",
         "v9:wheels_imu_lidar": "v9 EKF + known-map matcher (same readings)"}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def tour_rows(prefix):
    files = sorted(RES.glob(f"{prefix}_s*.json"))
    return [json.loads(f.read_text()) for f in files]


def report_tours():
    groups = sorted({re.sub(r"_s\d+\.json$", "", f.name) for f in RES.glob("ros_tour*_s*.json")})
    for g in groups:
        runs = tour_rows(g)
        print(f"\n{g}: house tour, {len(runs)} seeds; position error median / 95% / max (cm), yaw 95% (deg); "
              "error at the end of the tour -> after the 3 s wall push (cm)")
        names = [n for n in NAMES if any(n in r["names"] for r in runs)]
        # the map this tour localized in, and the SE(2) that puts it on the truth (map_quality best fit): applied to
        # the slam estimate, it leaves the error against the map itself (the map frame's own offset removed)
        mfile = RES / f"ros_map_{runs[0].get('map', 'house')}.json"
        fit = json.loads(mfile.read_text())["fit"] if mfile.exists() else None
        if fit and "ros_map" in names:
            names.insert(names.index("ros_map") + 1, "ros_map_rel")
        for n in names:
            pe, ye, end, after = [], [], [], []
            for r in runs:
                S = r["samples"]
                if n == "ros_map_rel":
                    c, s_ = math.cos(math.radians(fit["dyaw_deg"])), math.sin(math.radians(fit["dyaw_deg"]))
                    S = [dict(s, est={**s["est"], n: None if not s["est"].get("ros_map") else
                                      (fit["dx"] + c * s["est"]["ros_map"][0] - s_ * s["est"]["ros_map"][1],
                                       fit["dy"] + s_ * s["est"]["ros_map"][0] + c * s["est"]["ros_map"][1],
                                       s["est"]["ros_map"][2] + math.radians(fit["dyaw_deg"]))}) for s in S]
                tour = [s for s in S if s["phase"] == "tour" and s["est"].get(n)]
                pe += [math.hypot(s["est"][n][0] - s["truth"][0], s["est"][n][1] - s["truth"][1]) for s in tour]
                ye += [abs(math.degrees(wrap(s["est"][n][2] - s["truth"][2]))) for s in tour]
                if tour:
                    s = tour[-1]
                    end.append(math.hypot(s["est"][n][0] - s["truth"][0], s["est"][n][1] - s["truth"][1]))
                back = [s for s in S if s["phase"] == "back" and s["est"].get(n)]
                if back:
                    s = back[-1]
                    after.append(math.hypot(s["est"][n][0] - s["truth"][0], s["est"][n][1] - s["truth"][1]))
            if not pe:
                continue
            pe = 100 * np.array(pe)
            print(f"  {NAMES[n]:78s} {np.median(pe):5.1f} / {np.percentile(pe, 95):5.1f} / {pe.max():5.1f} cm, "
                  f"yaw {np.percentile(ye, 95):5.2f} deg | {100 * np.mean(end):5.1f} -> "
                  f"{100 * np.mean(after) if after else float('nan'):5.1f} cm")


def report_maps():
    files = sorted(RES.glob("ros_map_*.json"))
    if not files:
        return
    print("\nMaps (5 cm grid): wall error of the occupied cells median / 95% / max (cm), share > 10 cm off, coverage of "
          "the true wall faces; as anchored (the map frame where it really is) | best fit (offset removed)")
    for f in files:
        s = json.loads(f.read_text())
        a, b = s["anchored"], s["best_fit"]
        w = s["walls_anchored"]
        offs = np.array([x["offset_mm"] for x in w])
        print(f"  {f.stem[8:]:16s} {100 * a['med']:4.1f} / {100 * a['p95']:4.1f} / {100 * a['max']:4.1f} cm, "
              f"{100 * a['phantom']:4.1f}%, {100 * s['coverage_anchored']:3.0f}% | {100 * b['med']:4.1f} / "
              f"{100 * b['p95']:4.1f} cm, fit moves it {100 * math.hypot(s['fit']['dx'], s['fit']['dy']):4.1f} cm "
              f"{s['fit']['dyaw_deg']:+5.2f} deg | partition faces: |offset| median {np.median(np.abs(offs)):4.1f} mm, "
              f"max {np.max(np.abs(offs)):4.1f} mm")


def report_goals():
    for f in sorted(RES.glob("ros_nav_goals*.json")):
        rows = json.loads(f.read_text())["goals"]
        ok = [r for r in rows if r["ok"]]
        print(f"\n{f.stem}: {len(ok)} of {len(rows)} goals reached, {sum(r['time'] for r in ok):.0f} s in all")
        for r in rows:
            print(f"  {r['goal'][:62]:62s} {r['result']:9s} {r['time']:5.1f} s {r['path']:5.1f} m, ends "
                  f"{100 * r['err']:5.1f} cm / {r['yaw_err']:4.1f} deg off, {r['recoveries']} recoveries")


if __name__ == "__main__":
    what = sys.argv[1:] or ["maps", "tours", "goals"]
    if "maps" in what:
        report_maps()
    if "tours" in what:
        report_tours()
    if "goals" in what:
        report_goals()
