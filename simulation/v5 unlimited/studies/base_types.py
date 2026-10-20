"""Which drive base? X4 omni (v3b) vs kiwi, mecanum, swerve and differential drive.

What limits an omni base's acceleration is traction, not torque: an AK40-10 can put ~85 N on the floor through a
96 mm wheel, the floor gives each wheel ~12 N (mu x load). And an omni (or mecanum) wheel can only push along one
fixed direction, so in the worst direction only two of four wheels push, each at 45 deg: mu g / 2 before load
transfer, whatever the wheel count (traction.py). Bases whose wheels can point their force where it's needed
(swerve) or that only ever push along their wheels (differential drive, forward) use the full mu g.

1. Traction / tip / reach model, every base on the same 5.3 kg, CG 95 mm, ~150 mm footprint:
   - X4 omni: the sim's measured table (params.ACCEL_TABLE_PHYS), top speed from the X kinematics;
   - kiwi: traction.py's quasi-static polar at the same size, scaled by the X4's sim/model ratio;
   - mecanum: kinematically the X4 (rollers at 45 deg); typical PU/rubber mecanum rollers grip less (mu ~0.7);
   - swerve: every wheel steered along the push, rubber tires (mu 1.0), load-aware torque: a = min(mu g, tip) at
     85% margin like the X4 table; 2.6 m/s; ~50 ms to steer 90 deg first;
   - differential drive: two rubber drive wheels carrying ~80% of the weight + casters, 85% margin; yaw accel from
     the wheels' traction (60 rad/s^2); it has to turn toward the target (or drive backwards), and turn back to
     face the ball/thrower for the tilting funnel and the return throw.
   Reach = time-optimal bang-bang move of 0.3 / 0.6 / 1.0 m and stop, over 24 directions around the heading.
2. The catch game (catch_sweep grid, 216 identical throws, expected cameras from camera_estimate.py, physics
   limits, moving catches) for the bases the MuJoCo model can represent: X4 at mu 0.95 (v3b), X4 at mu 0.7
   (mecanum-like rollers), and a holonomic "swerve-like" base (X4 with tire grip, 8 m/s^2 every direction,
   2.6 m/s, +50 ms for steering).
Writes results/base_types.json.
"""

from __future__ import annotations

import itertools
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import RESULTS, SimParams, save, traction_base  # noqa: E402
from catch_game import CatchGame, GameConfig  # noqa: E402
from params import ACCEL_TABLE_PHYS  # noqa: E402
from robot_model import build_model  # noqa: E402
from traction import Base  # noqa: E402
from camera_estimate import BEST, EXPECTED, SPIN  # noqa: E402
from dataclasses import replace  # noqa: E402

G = 9.81
DIRS = np.arange(0, 360, 15)
MOVES = (0.3, 0.6, 1.0)


def bangbang(r, a, v):
    """Time to move r from rest to rest at acceleration a, speed cap v."""
    if r <= v * v / a:
        return 2 * math.sqrt(r / a)
    return r / v + v / a


def x4_table(phi_deg, scale=1.0):
    ang = abs(((phi_deg + 180) % 360) - 180)
    return scale * float(np.interp(ang, *ACCEL_TABLE_PHYS))


def x4_speed(phi_deg):
    # wheel-speed ceiling 2.43 m/s; along the X's sides two wheel pairs add at 45 deg -> /cos(45)
    k = max(abs(math.cos(math.radians(phi_deg - a + 90))) for a in (45, 135, 225, 315))
    return 2.43 / k


def reach_model():
    p = SimParams()
    xb = traction_base(p)                                      # X4 + rocker, the v3b robot (quasi-static)
    kb = Base((0.0, 120.0, 240.0), 0.15, xb.h, xb.cx, xb.cy, 0.95)     # kiwi, same size and CG, a wheel in front
    ratio = np.mean([x4_table(d) / xb.limit(d)["slip"] for d in DIRS])  # sim / model on the X4
    out = {}
    bases = {
        "X4 omni (v3b), mu 0.95": lambda d, r: bangbang(r, x4_table(d), x4_speed(d)),
        "kiwi omni, mu 0.95": lambda d, r: bangbang(r, ratio * kb.limit(d)["slip"], 2.43 / max(
            abs(math.cos(math.radians(d - a + 90))) for a in (0, 120, 240))),
        "mecanum (X4 kinematics), roller mu 0.7": lambda d, r: bangbang(r, x4_table(d, 0.7 / 0.95), x4_speed(d)),
        "swerve, rubber mu 1.0": lambda d, r: 0.05 + bangbang(r, min(0.85 * 1.0 * G, 0.85 * xb.tip_accel(
            [math.cos(math.radians(d)), math.sin(math.radians(d))])), 2.6),
    }
    a_dd, alpha = 0.85 * 1.0 * G * 0.8, 60.0

    def turn(th):
        return 2 * math.sqrt(abs(th) / alpha)

    def diff(d, r, back=True):
        th = math.radians(abs(((d + 180) % 360) - 180))        # 0..pi from the heading
        th = min(th, math.pi - th)                             # it can drive backwards
        return turn(th) + bangbang(r, a_dd, 2.6) + (turn(th) if back else 0.0)
    bases["diff drive, turn there and back"] = lambda d, r: diff(d, r, True)
    bases["diff drive, no turn back (upright funnel)"] = lambda d, r: diff(d, r, False)
    for name, f in bases.items():
        t = {r: [f(d, r) for d in DIRS] for r in MOVES}
        out[name] = {f"{r:g}": dict(mean=float(np.mean(v)), worst=float(np.max(v)), best=float(np.min(v)),
                                    side=float(f(90, r)), fwd=float(f(0, r))) for r, v in t.items()}
    out["_kiwi_model"] = dict(worst=float(ratio * min(kb.limit(d)["slip"] for d in DIRS)),
                              mean=float(ratio * np.mean([kb.limit(d)["slip"] for d in DIRS])),
                              tip_worst=float(min(kb.limit(d)["tip"] for d in DIRS)))
    out["_x4_tip_worst"] = float(min(xb.limit(d)["tip"] for d in DIRS))
    out["_x4_model_worst"] = float(min(xb.limit(d)["slip"] for d in DIRS))
    print("1. Reach: time (s) to move and stop, mean over directions / worst direction / sideways / forward")
    print(f"   {'base':44s}" + "".join(f"{r:>22g} m" for r in MOVES))
    for name in bases:
        print(f"   {name:44s}" + "".join(
            f"   {out[name][f'{r:g}']['mean']:.2f}/{out[name][f'{r:g}']['worst']:.2f}/"
            f"{out[name][f'{r:g}']['side']:.2f}/{out[name][f'{r:g}']['fwd']:.2f}" for r in MOVES))
    print(f"   kiwi (scaled model): worst {out['_kiwi_model']['worst']:.2f}, mean {out['_kiwi_model']['mean']:.2f} m/s^2, "
          f"tip worst {out['_kiwi_model']['tip_worst']:.1f}; X4 tip worst {out['_x4_tip_worst']:.1f} m/s^2; "
          f"sim/model ratio {ratio:.2f}")
    return out


# --- 2. catch game ---------------------------------------------------------------------
DISTANCES = (1.5, 2.25, 3.0)
APEXES = (0.5, 1.0, 2.0, 3.0)
OFFSETS = (0.3, 0.6, 1.0)
TOSSES = 6
HOME = (1.0, 0.0)
FLAT8 = ((0.0, 180.0), (8.0, 8.0))
FLAT6 = ((0.0, 180.0), (6.0, 6.0))
LATE = replace(EXPECTED, latency_mean=EXPECTED.latency_mean + 0.05)
VARIANTS = {   # name: (SimParams kwargs, VisionConfig)
    "X4 omni (v3b), mu 0.95": (dict(), EXPECTED),
    "X4 / mecanum-like rollers, mu 0.7": (dict(wheel_mu=0.7, accel_table=(ACCEL_TABLE_PHYS[0], tuple(
        v * 0.7 / 0.95 for v in ACCEL_TABLE_PHYS[1]))), EXPECTED),
    "holonomic 8 m/s^2, X4 speeds (accel only)": (dict(wheel_mu=2.0, accel_table=FLAT8), EXPECTED),
    "swerve-like: 8 m/s^2, 2.6 m/s, +50 ms steer": (dict(wheel_mu=2.0, accel_table=FLAT8, base_max_speed=2.6), LATE),
    "X4 omni, best cameras": (dict(), BEST),
    "swerve-like, best cameras": (dict(wheel_mu=2.0, accel_table=FLAT8, base_max_speed=2.6),
                                  replace(BEST, latency_mean=BEST.latency_mean + 0.05)),
    # at 8 m/s^2 the 95 mm-CG, r 150 base is at 80% of its tip limit and rocks 5-10 deg into the catch; a calmer cap
    "swerve-like at 6 m/s^2": (dict(wheel_mu=2.0, accel_table=FLAT6, base_max_speed=2.6), LATE),
    "swerve-like at 6 m/s^2, best cameras": (dict(wheel_mu=2.0, accel_table=FLAT6, base_max_speed=2.6),
                                             replace(BEST, latency_mean=BEST.latency_mean + 0.05)),
}


def run_cell(args):
    name, dist, apex, off, seed = args
    pkw, vcfg = VARIANTS[name]
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), **pkw)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(catch_zone_radius=off, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
                     wait_between=0.8, perception="vision", vision_cfg=vcfg, toss_spin=SPIN, catch_mode="moving")
    game = CatchGame(m, d, p, cfg=cfg, seed=seed, verbose=False)
    out, tilt = [], 0.0
    q = m.joint("base").qposadr[0]
    while len(game.results) < TOSSES:
        game.step()
        w, x, y, z = d.qpos[q + 3:q + 7]
        tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))))
        if len(game.results) > len(out):
            r = game.results[-1]
            rim_margin = p.funnel_rim_radius - p.ball_spec.radius
            why = "caught" if r.caught else ("reach" if not np.isfinite(r.pos_error) or r.pos_error > rim_margin
                                             else "bounce")
            out.append(dict(caught=bool(r.caught), why=why, pos_error=r.pos_error, catch_speed=r.catch_speed,
                            peak_current=r.peak_current, tilt=tilt))
            tilt = 0.0
    return dict(variant=name, distance=dist, apex=apex, offset=off, tosses=out)


def report_game(rows):
    print("\n2. Catch game (216 identical throws, expected cameras unless noted)")
    out = {}
    for name in VARIANTS:
        ts = [t for r in rows if r["variant"] == name for t in r["tosses"]]
        if not ts:
            continue
        k = float(np.mean([t["caught"] for t in ts]))
        by_off = {f"{o:g}": float(np.mean([t["caught"] for r in rows if r["variant"] == name and r["offset"] == o
                                           for t in r["tosses"]])) for o in OFFSETS}
        out[name] = dict(caught=k, reach=sum(t["why"] == "reach" for t in ts), bounce=sum(t["why"] == "bounce" for t in ts),
                         by_offset=by_off, max_tilt=float(max(t["tilt"] for t in ts)),
                         peak_current=float(max(t["peak_current"] for t in ts)))
        print(f"   {name:46s} {100 * k:4.0f}%  reach/bounce {out[name]['reach']:3d}/{out[name]['bounce']:<3d} "
              + "  ".join(f"{o} m: {100 * v:3.0f}%" for o, v in by_off.items())
              + f"   max body tilt {out[name]['max_tilt']:.1f} deg, peak bus {out[name]['peak_current']:.1f} A")
    return out


def main():
    import json
    data = dict(reach=reach_model())
    args = sys.argv[1:]
    if "reach" in args:
        return
    todo = [n for n in VARIANTS if not args or n in args]      # named variants: rerun those, keep the saved rest
    cells = [(n, dd, a, o, j) for n in todo for j, (dd, a, o) in enumerate(itertools.product(DISTANCES, APEXES, OFFSETS))]
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(run_cell, cells, chunksize=1))
    saved = RESULTS / "base_types.json"
    if args and saved.exists():
        rows = [r for r in json.loads(saved.read_text())["rows"] if r["variant"] not in todo] + rows
        rows.sort(key=lambda r: list(VARIANTS).index(r["variant"]))
    data["game"] = report_game(rows)
    data["rows"] = rows
    save("base_types", data)


if __name__ == "__main__":
    main()
