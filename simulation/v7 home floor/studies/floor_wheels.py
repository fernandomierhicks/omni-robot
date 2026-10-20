"""Floor and wheel realism (design.md 23.4): the explicit-roller model on four wheels, floor friction that varies,
door thresholds and carpet edges.

A. Roller model on the X4 base. Every v3-v6 result used the fast "sphere" wheel (one sliding sphere per wheel with
   direction-dependent grip); v2 validated the explicit double-row roller model only on the kiwi. Compared here on
   the round-3 robot: slip-free acceleration per body direction (common.max_clean_accel), speed ripple and heading
   wander cruising at 1.5 m/s, and the 6-7 m catch game (firmware adult, 72 throws).
B. Crossing obstacles with a ball in the pocket: thresholds 6 / 12 / 18 mm high (30 mm top, 20 mm ramps) and carpet
   edges (8 and 12 mm square rug edges, 8 mm with a 25 mm bevel), approached at 0.5 / 1 / 2 m/s, moving at body
   0 deg (forward: every wheel half rolling, half on its rollers), 45 deg (two wheels purely on their rollers, the
   hardest case for an omni wheel) and 90 deg. Both wheel models; the sphere can't represent a roller meeting a
   step sideways, so the roller model is the reference. Scored: got across, peak body tilt, heading and path
   error, a wheel lifting, the ball still in the pocket, wheel-odometry error.
C. The catch game on real floors (firmware adult, 6-7 m throws, the base's acceleration table still assumes 0.95):
   a friction field (sigma 0.1 / 0.2 around 0.95, 0.5 m patches), slippery 0.5 stripes across the catch area, and a
   "home floor": a 0.1 field, a 12 mm threshold 0.5 m in front of the waiting spot and an 8 mm rug behind it,
   with and without a firmware slow zone (<= 1 m/s) over the threshold.

  python floor_wheels.py            # A, B and C
  python floor_wheels.py B          # one part
Writes results/floor_wheels.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import RESULTS, SimParams, max_clean_accel, save, tilt_deg  # noqa: E402
from catch_game import GameConfig  # noqa: E402
from floor import Carpet, Threshold  # noqa: E402
from launcher_bench import Bench  # noqa: E402
from omni_control import wrap_angle  # noqa: E402
from robot_model import build_model  # noqa: E402
from shallow_sweep import APEXES, AREA, DISTANCES, EXPECTED, FW, FWG, HOME, R3, V6G, HumanToss  # noqa: E402

DIRS_A = (0.0, 30.0, 45.0, 60.0, 90.0, 135.0, 180.0)
OBSTACLES = {
    "threshold 6 mm": Threshold(0.6, 0.0, height=0.006),
    "threshold 12 mm": Threshold(0.6, 0.0, height=0.012),
    "threshold 18 mm": Threshold(0.6, 0.0, height=0.018),
    "rug edge 8 mm": Carpet(0.6, -1.5, 2.6, 1.5, thickness=0.008),
    "rug edge 12 mm": Carpet(0.6, -1.5, 2.6, 1.5, thickness=0.012),
    "carpet trim 8 mm, 25 mm bevel": Carpet(0.6, -1.5, 2.6, 1.5, thickness=0.008, bevel=0.025),
}
DIRS_B = (0.0, 45.0, 90.0)
SPEEDS_B = (0.5, 1.0, 2.0)
MODELS = ("sphere", "rollers")


# -- A ---------------------------------------------------------------------------

def accel_job(args):
    wm, direction = args
    a, why = max_clean_accel(SimParams(wheel_model=wm), direction, lo=1.0, hi=8.0)
    return dict(model=wm, direction=direction, accel=a, why=why)


def cruise_job(args):
    """Cruise at 1.5 m/s in a body direction for 1.5 s: speed ripple, heading wander, lateral drift."""
    wm, direction = args
    b = Bench(SimParams(wheel_model=wm, area_size=(14.0, 14.0)))      # room to cruise (the netting stops the deck)
    r, d = b.r, b.d
    u = np.array([math.cos(math.radians(direction)), math.sin(math.radians(direction))])
    yaw0 = r.pose()[2]
    sp, hd, lat = [], [], []
    p0 = r.pose()[:2].copy()
    t0 = d.time
    while d.time - t0 < 2.5:
        r.drive_velocity(1.5 * u, 0.0, r.pose()[2], a_cap=2.0)
        r.step()
        if d.time - t0 > 1.0:
            v = d.sensor("base_linvel").data[:2]
            sp.append(float(np.linalg.norm(v)))
            hd.append(math.degrees(wrap_angle(r.pose()[2] - yaw0)))
            rel = r.pose()[:2] - p0
            lat.append(float(rel[0] * -u[1] + rel[1] * u[0]))
    return dict(model=wm, direction=direction, speed_mean=float(np.mean(sp)), ripple=float(np.std(sp) / np.mean(sp)),
                heading_drift=float(hd[-1] - hd[0]), heading_pp=float(np.ptp(hd)), lateral_drift=float(lat[-1] - lat[0]))


# -- B ---------------------------------------------------------------------------

def cross_job(args):
    """Drive across an obstacle (at world x = 0.6, running along y) at speed v in world +x, the robot's heading set
    so that this is body direction `direction`; a ball rests in the pocket."""
    wm, name, direction, v = args
    p = SimParams(wheel_model=wm, floor_features=(OBSTACLES[name],))
    b = Bench(p)
    m, d, r = b.m, b.d, b.r
    yaw = -math.radians(direction)                      # body direction = world +x minus the heading
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    d.qpos[adr] = -0.15
    mujoco.mj_forward(m, d)
    b.run(0.3)
    held0 = b.load_ball()
    dt = m.opt.timestep
    odo = np.array([*r.pose()[:2], r.pose()[2]])
    tilt = 0.0
    n_min = 1e9
    hd_max = 0.0
    t0 = d.time
    x_start = r.pose()[0]
    stuck = False
    while d.time - t0 < 6.0:
        pose = r.pose()
        w = float(np.clip(4.0 * wrap_angle(yaw - pose[2]), -2.0, 2.0))
        r.drive_velocity((v, 0.0), w, pose[2], a_cap=2.0)
        r.step()
        tw = r.kin.body_twist(r.wheel_speeds())
        c, s = math.cos(odo[2]), math.sin(odo[2])
        odo += dt * np.array([c * tw[0] - s * tw[1], s * tw[0] + c * tw[1], tw[2]])
        if r.pose()[0] > 0.2:                             # the front of the robot is at the obstacle
            tilt = max(tilt, tilt_deg(d))
            n_min = min(n_min, float(np.min(r.wheel_loads())))
            hd_max = max(hd_max, abs(math.degrees(wrap_angle(r.pose()[2] - yaw))))
        if r.pose()[0] > 1.25:
            break
        if d.time - t0 > 1.0 + 1.25 / v * 2.5:
            stuck = True
            break
    pose = r.pose()
    # stop gently, then see whether the ball is still in
    for _ in range(int(1.0 / dt)):
        r.drive_velocity((0.0, 0.0), 0.0, r.pose()[2], a_cap=2.0)
        r.step()
    travelled = float(pose[0] - x_start)
    return dict(model=wm, obstacle=name, direction=direction, speed=v, crossed=bool(pose[0] > 1.25 and not stuck),
                tilt=tilt, lifted=bool(n_min < 0.5), heading_max=hd_max, lateral=float(pose[1]),
                held=bool(held0 and r.ball_held()), odo_err=float(np.hypot(*(odo[:2] - pose[:2]))),
                odo_err_rel=float(np.hypot(*(odo[:2] - pose[:2])) / max(travelled, 0.1)))


# -- C ---------------------------------------------------------------------------

HOME_FLOOR = dict(floor_mu_field=(0.1, 0.5, 3), floor_features=(
    Threshold(HOME[0] - 0.5, 0.0, length=AREA[1], height=0.012),
    Carpet(HOME[0] + 0.2, -1.6, HOME[0] + 1.8, 1.6, thickness=0.008)))
FLOORS = {
    "uniform 0.95": {},
    "field sigma 0.1": dict(floor_mu_field=(0.1, 0.5, 1)),
    "field sigma 0.2": dict(floor_mu_field=(0.2, 0.5, 2)),
    "slippery stripes (0.5)": dict(floor_patches=((HOME[0] - 0.45, -3, HOME[0] - 0.15, 3, 0.5),
                                                  (HOME[0] + 0.15, -3, HOME[0] + 0.45, 3, 0.5))),
    "home floor (field, threshold, rug)": HOME_FLOOR,
    # the firmware knows where the threshold is (lidar map) and crosses it at <= 1 m/s
    "home floor, <= 1 m/s over the threshold": dict(HOME_FLOOR, slow_zones=((HOME[0] - 0.55, -3.0, HOME[0] - 0.45, 3.0, 1.0),)),
}
GAMES = [(f, "sphere") for f in FLOORS] + [(f, "rollers") for f in ("uniform 0.95", "home floor (field, threshold, rug)",
                                                                    "home floor, <= 1 m/s over the threshold")]
TOSSES = 6


def game_job(args):
    floor_name, wm, dist, apex, seed = args
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, wheel_model=wm,
                  **FLOORS[floor_name], **R3, **FW)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
                               wait_between=1.2, perception="vision", vision_cfg=EXPECTED, toss_spin=6.0), **FWG, **V6G})
    game = HumanToss(m, d, p, cfg=cfg, seed=seed, verbose=False, dist=dist)
    rim = p.funnel_catch_radius - p.ball_spec.radius
    tilt = 0.0
    while len(game.results) < TOSSES and d.time < 150:
        game.step()
        tilt = max(tilt, tilt_deg(d))
    out = [dict(caught=bool(r.caught), why="caught" if r.caught else ("reach" if not np.isfinite(r.pos_error) or
                                                                      r.pos_error > rim else "bounce"),
                pos_error=r.pos_error, ctrl_err=r.ctrl_err) for r in game.results]
    return dict(floor=floor_name, model=wm, distance=dist, apex=apex, tosses=out, tilt_max=tilt)


# -- run + report -------------------------------------------------------------------

def run(parts):
    path = RESULTS / "floor_wheels.json"
    out = json.loads(path.read_text()) if path.exists() else {}
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        if "A" in parts:
            acc = pool.map(accel_job, [(wm, a) for wm in MODELS for a in DIRS_A])
            cru = pool.map(cruise_job, [(wm, a) for wm in MODELS for a in (0.0, 45.0, 90.0)])
            ga = pool.map(game_job, [("uniform 0.95", wm, dd, a, 7000 + 10 * DISTANCES.index(dd) + APEXES.index(a))
                                     for wm in MODELS for dd in DISTANCES for a in APEXES])
            out["A"] = dict(accel=list(acc), cruise=list(cru), game=list(ga))
        if "B" in parts:
            jobs = [(wm, ob, dr, v) for wm in MODELS for ob in OBSTACLES for dr in DIRS_B for v in SPEEDS_B]
            out["B"] = list(pool.map(cross_job, jobs, chunksize=1))
        if "C" in parts:
            jobs = [(f, wm, dd, a, 8000 + 10 * DISTANCES.index(dd) + APEXES.index(a))
                    for f, wm in GAMES for dd in DISTANCES for a in APEXES]
            out["C"] = list(pool.map(game_job, jobs, chunksize=1))
    save("floor_wheels", out)
    report(out)


def _rate(rows):
    ts = [t for r in rows for t in r["tosses"]]
    return (float(np.mean([t["caught"] for t in ts])), sum(t["why"] == "reach" for t in ts),
            sum(t["why"] == "bounce" for t in ts), float(np.nanmedian([t["pos_error"] for t in ts if t["caught"]])), len(ts))


def report(out):
    if "A" in out:
        A = out["A"]
        print("A. Roller model vs sphere model, X4 round-3 robot")
        print("   slip-free acceleration by body direction (m/s^2): " + " ".join(f"{a:g}" for a in DIRS_A))
        for wm in MODELS:
            row = {x["direction"]: x for x in A["accel"] if x["model"] == wm}
            print(f"     {wm:8s} " + " ".join(f"{row[a]['accel']:.2f}{'t' if row[a]['why'] == 'tip' else ''}" for a in DIRS_A))
        print("   cruising 1.5 m/s (body 0 / 45 / 90 deg): speed ripple, heading drift over 1.5 s, lateral drift")
        for wm in MODELS:
            cells = [f"{100 * x['ripple']:.1f}% {x['heading_drift']:+.1f} deg {100 * x['lateral_drift']:+.1f} cm"
                     for x in A["cruise"] if x["model"] == wm]
            print(f"     {wm:8s} " + " | ".join(cells))
        for wm in MODELS:
            k, rc, bo, off, n = _rate([g for g in A["game"] if g["model"] == wm])
            print(f"   catch game, {wm:8s}: {100 * k:.0f}% of {n} (not in place {rc}, bounced {bo}), offset {100 * off:.1f} cm")
    if "B" in out:
        print("B. Crossing obstacles with a ball aboard: across / ball kept, peak tilt, heading error, path off, "
              "odometry error, per body direction 0 / 45 / 90 deg; at 0.5, 1, 2 m/s")
        for wm in MODELS:
            print(f"   {wm} model:")
            for ob in OBSTACLES:
                cells = []
                for v in SPEEDS_B:
                    xs = [x for x in out["B"] if x["model"] == wm and x["obstacle"] == ob and x["speed"] == v]
                    xs.sort(key=lambda x: x["direction"])
                    cells.append(" ".join(("ok" if x["crossed"] else "STUCK") + ("" if x["held"] else "/LOST") for x in xs) +
                                 f" {max(x['tilt'] for x in xs):.0f}deg {max(x['heading_max'] for x in xs):.0f}deg "
                                 f"{100 * max(abs(x['lateral']) for x in xs):.0f}cm odo {100 * max(x['odo_err'] for x in xs):.0f}cm")
                print(f"     {ob:30s} " + " | ".join(cells))
    if "C" in out:
        print("C. Catch game on real floors (firmware adult, 6-7 m shallow throws, 72 per floor)")
        for f, wm in GAMES:
            rows = [x for x in out["C"] if x["floor"] == f and x["model"] == wm]
            if not rows:
                continue
            k, rc, bo, off, n = _rate(rows)
            print(f"   {f:38s} {wm:8s} {100 * k:4.0f}% (not in place {rc}, bounced {bo}), offset {100 * off:.1f} cm, "
                  f"peak tilt {max(r['tilt_max'] for r in rows):.1f} deg")


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "report":
        report(json.loads((RESULTS / "floor_wheels.json").read_text()))
    else:
        run(a or ["A", "B", "C"])
