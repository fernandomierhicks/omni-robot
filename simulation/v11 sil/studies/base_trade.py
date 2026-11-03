"""Does the base need four wheels and sprung pods? (design.md 16, Oct 27). The drawn robot (robot_export.drawn_params:
the CAD's masses, DM-2325 drive, 127 mm big-roller wheels, firmware adult limits at 2 m/s) on four bases:

  X4 + pods     the baseline: four wheels, each on a sprung, damped pod (15 mm travel, 3 N/mm, zeta 0.6)
  X4 rigid      the same four wheels bolted to the deck (only the TPU rollers' compliance)
  kiwi rigid    three wheels at 60 / 180 / 300 deg (v2's layout: the launcher between the front pair), no pods
  kiwi + pods   the kiwi on pods retuned for three wheels (design.md 26: 20 mm, 3 N/mm, 6 N preload)

Each base keeps the drawn robot's extra point mass (the head, face, cables), so a kiwi is a wheel, a motor and a pod
lighter. Every crossing uses the explicit-roller wheel model.

T. Acceleration and tipping: slip-free acceleration per body direction (common.max_clean_accel), the quasi-static
   tip-over limit, mass and CG. The firmware table of a base it wasn't measured for: 85% of the lower of slip and tip,
   capped at 4.5 m/s^2 (the rule behind the X4's table, params.ACCEL_TABLE_V3B); the X4s keep the baseline's table.
A. House transitions (floor_wheels.OBSTACLES + floor.HOUSE_OBSTACLES) at 0.5 / 1 / 2 m/s, 13 body directions
   (0-180 deg, the bases are mirror-symmetric), a ball in the pocket.
S. Steps: a square step up onto a 0.8 m platform and down its far side, 10-40 mm; triangular bumps 76 mm (3 in)
   wide, 13-38 mm (0.5-1.5 in) tall. Same speeds and directions. Scored: across, where it stopped, belly or bumper
   on the obstacle, peak tilt, airborne, ball kept, flipped.
G. Goal to goal at 2 m/s: firmware moves of 1 / 2 / 4 m on a flat floor in 7 body directions (time, peak speed,
   peak tilt, wheel lift in the stop), and v9's house tour (19 stops, 46 m through the doors, over the rugs and
   thresholds) at the 2 m/s cap, with and without the terrain assist (the firmware's speed caps at mapped edges).
B. The 6-7 m catch game (sil_catch.py's ref game: firmware adult, expected cameras, 216 throws per row): flat,
   home (a 12 mm threshold ahead of the waiting spot, an 8 mm rug behind) and a 12 mm rug turned 45 deg.

  python base_trade.py T | A | S | G | B [BASE ...]      one part (all bases if none given)
  python base_trade.py report
Writes results/base_trade.json (merged per part and base, so parts can be rerun).
"""

from __future__ import annotations

import functools
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import RESULTS, max_clean_accel, save, tilt_deg  # noqa: E402
from floor import Threshold  # noqa: E402
from floor_wheels import OBSTACLES  # noqa: E402
from launcher_bench import Bench  # noqa: E402
from omni_control import wrap_angle  # noqa: E402
from params import ACCEL_TABLE_V3B  # noqa: E402
import floor as floor_mod  # noqa: E402

OUT = RESULTS / "base_trade.json"
WORKERS = min(27, os.cpu_count() or 4)

KIWI_PODS = dict(layout="kiwi3", suspension="pod", pod_travel=0.020, pod_rate=3000.0, pod_damping_ratio=0.6,
                 pod_preload=6.0)
BASES = {
    "X4 + pods": dict(),
    "X4 rigid": dict(suspension="roller", rocker_deg=0.0),
    "kiwi rigid": dict(layout="kiwi3", suspension="roller", rocker_deg=0.0),
    "kiwi + pods": KIWI_PODS,
    # Oct 28: the middle option, v3-v6's front rocker (the front pair on a beam pivoting about x, +/- 3 deg) on the
    # otherwise rigid X4: four wheels always touch, but no travel over an edge
    "X4 rocker": dict(suspension="roller", rocker="front", rocker_deg=3.0),
}
# variants that only the catch game and T run: one wheel 1 mm low (a floor dip or a build error) on the rigid X4s
VARIANTS = {
    "X4 rigid, a wheel 1 mm low": dict(suspension="roller", rocker_deg=0.0, wheel_z_offset=(0.001, 0.0, 0.0, 0.0)),
    "X4 rocker, a wheel 1 mm low": dict(suspension="roller", rocker="front", rocker_deg=3.0,
                                        wheel_z_offset=(0.001, 0.0, 0.0, 0.0)),
}
# Oct 30: the drive motor on the baseline base (Fernando prefers to buy AK40-10s): the same X4 + pods on the
# CubeMars AK40-10 (design.md 24), at full charge and at 10% (the AK40-10's speed is the tight one), against the
# DM-2325 at 10%. Run by name: python base_trade.py T|A|S|G|B "X4 + pods, AK40-10" ...
MOTOR_BASES = {
    "X4 + pods, AK40-10": dict(drive_motor="AK40-10"),
    "X4 + pods, AK40-10, 10%": dict(drive_motor="AK40-10", battery_soc=0.1),
    "X4 + pods, DM-2325, 10%": dict(battery_soc=0.1),
}
TABLE_DIRS = (0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0)
# part T's table for the kiwis (85% of min(slip, tip), <= 4.5 m/s^2; Oct 27). Braking from forward travel (180) is
# tip-limited: the drawn CG sits 21 mm ahead and the front edge between the two front wheels only 75 mm out. The kiwi
# on pods takes the rigid kiwi's: its own measurement trips the test's 2 deg tilt criterion on the pods' squat, not slip
KIWI_TABLE = (TABLE_DIRS, (2.90, 3.41, 4.50, 2.52, 1.64, 1.69, 3.33))
TABLES = {
    "kiwi rigid": KIWI_TABLE,
    "kiwi + pods": KIWI_TABLE,
}


@functools.lru_cache(maxsize=None)
def _drawn_extra() -> tuple:
    from robot_export import drawn_params
    return drawn_params().extra_masses


def base_kw(base: str) -> dict:
    """SimParams overrides of a base on top of robot_export.V11 (the drawn robot's masses)."""
    kw = {**BASES, **VARIANTS, **MOTOR_BASES}[base]
    table = TABLES.get(base, "x4")
    if table == "x4":                     # the X4s: the firmware's table, scaled to the wheel's grip as params does
        k = min(1.0, 0.9 / 0.95)
        table = (ACCEL_TABLE_V3B[0], tuple(v * k for v in ACCEL_TABLE_V3B[1]))
    return dict(mass_model="cad", extra_masses=_drawn_extra(), accel_table=table, **kw)


def params(base: str, **kw):
    from robot_export import v11_params
    return v11_params(**{**base_kw(base), **kw})


def _load() -> dict:
    return json.loads(OUT.read_text()) if OUT.exists() else {}


def _merge(part: str, rows: list, by=("base",)):
    out = _load()
    keys = {tuple(r[k] for k in by) for r in rows}
    out[part] = [r for r in out.get(part, []) if tuple(r[k] for k in by) not in keys] + rows
    save("base_trade", out)
    return out


def _pool(fn, jobs):
    with ProcessPoolExecutor(WORKERS) as pool:
        return list(pool.map(fn, jobs, chunksize=1))


# -- T ---------------------------------------------------------------------------------------------

def accel_job(args):
    base, direction = args
    a, why = max_clean_accel(params(base, accel_table=None), direction, lo=1.0, hi=8.0, track_tol=0.12)
    return dict(base=base, direction=direction, slip=a, why=why)


def tip_rows(base: str) -> dict:
    from robot_model import build_model, mass_properties
    from traction import Base
    p = params(base)
    m = build_model(p.with_(home_xy=(0.0, 0.0)))
    mass, cg = mass_properties(m, p)
    tb = Base(p.wheel_angles_deg, p.wheel_base_radius, float(cg[2]), float(cg[0]), float(cg[1]), p.mu)
    tip = {a: min(tb.tip_accel((math.cos(math.radians(s * a)), math.sin(math.radians(s * a)))) for s in (1, -1))
           for a in TABLE_DIRS}
    worst = min(tb.tip_accel((math.cos(a), math.sin(a))) for a in np.radians(np.arange(0, 360, 5)))
    return dict(base=base, mass=float(mass), cg=[float(x) for x in cg], tip=tip, tip_worst=float(worst),
                n_wheels=p.n_wheels, suspension=p.suspension)


def run_T(bases):
    rows = _pool(accel_job, [(b, a) for b in bases for a in TABLE_DIRS])
    out = []
    for b in bases:
        t = tip_rows(b)
        slip = {x["direction"]: x["slip"] for x in rows if x["base"] == b}
        t["slip"] = slip
        t["table"] = [round(min(4.5, 0.85 * min(slip[a], t["tip"][a])), 2) for a in TABLE_DIRS]
        out.append(t)
    report_T(_merge("T", out)["T"])


def report_T(rows):
    print("T. Mass, CG height, worst tip-over (quasi-static), slip-free acceleration by body direction "
          f"{'/'.join(f'{a:g}' for a in TABLE_DIRS)} (m/s^2), and the table it gives (85% of min(slip, tip), <= 4.5)")
    for x in rows:
        sl = " ".join(f"{x['slip'][str(a)] if str(a) in x['slip'] else x['slip'][a]:.2f}" for a in TABLE_DIRS)
        print(f"  {x['base']:30s} {x['mass']:.2f} kg  CG {100 * x['cg'][2]:.1f} cm  tip worst {x['tip_worst']:.1f}  "
              f"slip {sl}  -> table {' '.join(f'{v:.2f}' for v in x['table'])}")


# -- A and S: crossings ---------------------------------------------------------------------------

DIRS = tuple(float(a) for a in range(0, 181, 15))
SPEEDS = (0.5, 1.0, 2.0)
EDGE = 0.6                      # every obstacle's first edge is at world x = 0.6; the robot starts at x = -0.15
PLATFORM = 0.8                  # m: a step's platform is this deep, then it steps down again
STEP_MM = (10, 15, 20, 25, 30, 35, 40)
TRI_MM = (13, 19, 25, 32, 38)   # 0.5, 0.75, 1, 1.25, 1.5 in
TRI_W = 0.076                   # 3 in wide at the base


def obstacle(name: str):
    """(features, the far edge's x, where the run ends) for an obstacle name."""
    if name.startswith("step "):
        h = float(name.split()[1]) / 1000
        f = Threshold(EDGE + PLATFORM / 2, 0.0, length=3.0, height=h, top=PLATFORM, ramp=0.0005, mu=0.85)
        return (f,), EDGE + PLATFORM, EDGE + PLATFORM + 0.45
    if name.startswith("bump "):
        h = float(name.split()[1]) / 1000
        f = Threshold(EDGE + TRI_W / 2, 0.0, length=3.0, height=h, top=0.002, ramp=TRI_W / 2 - 0.001, mu=0.8)
        return (f,), EDGE + TRI_W, EDGE + TRI_W + 0.55
    f = {**OBSTACLES, **floor_mod.HOUSE_OBSTACLES}[name]
    return (f,), EDGE + 0.05, 1.25


HOUSE = list(OBSTACLES) + list(floor_mod.HOUSE_OBSTACLES)
STEPS = [f"step {h} mm" for h in STEP_MM]
BUMPS = [f"bump {h} mm" for h in TRI_MM]


def cross_job(args):
    """Drive across an obstacle at speed v in world +x, the heading held so that this is body direction `direction`,
    a ball resting in the pocket (as home_floor.cross_job). Also: where it stopped, and whether the belly (battery,
    motors) or the deck / bumper ring touched the obstacle."""
    base, name, direction, v = args
    feats, x_far, x_end = obstacle(name)
    p = params(base, floor_features=feats)
    b = Bench(p)
    m, d, r = b.m, b.d, b.r
    yaw = -math.radians(direction)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    d.qpos[adr] = -0.15
    mujoco.mj_forward(m, d)
    b.run(0.3)
    held0 = b.load_ball()
    dt = m.opt.timestep
    feat_ids = {m.geom(f"floorfeat{k}").id for k in range(len(feats))}
    belly_ids = {g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("belly")}
    base_body = m.body("base").id
    hull_ids = {g for g in range(m.ngeom) if m.geom_bodyid[g] == base_body and m.geom_contype[g] and g not in belly_ids}
    tilt = air = air_run = lift_run = lift_max = 0.0
    belly = hull = flipped = stuck = False
    t0, x_max = d.time, -1.0
    while d.time - t0 < 1.0 + (x_end + 0.15) / v * 2.5:
        pose = r.pose()
        w = float(np.clip(4.0 * wrap_angle(yaw - pose[2]), -2.0, 2.0))
        r.drive_velocity((v, 0.0), w, pose[2], a_cap=2.0)
        r.step()
        x_max = max(x_max, float(pose[0]))
        if pose[0] > 0.2:
            t = tilt_deg(d)
            tilt = max(tilt, t)
            if t > 45:
                flipped = True
                break
            loads = r.wheel_loads()
            air_run = air_run + dt if np.all(loads < 0.5) else 0.0
            air = max(air, air_run)
            lift_run = lift_run + dt if np.any(loads < 0.5) else 0.0
            lift_max = max(lift_max, lift_run)
            for k in range(d.ncon):
                c = d.contact[k]
                g1, g2 = c.geom1, c.geom2
                if g1 in feat_ids or g2 in feat_ids:
                    o = g2 if g1 in feat_ids else g1
                    belly |= o in belly_ids
                    hull |= o in hull_ids
        if pose[0] > x_end:
            break
    else:
        stuck = True
    pose = r.pose()
    for _ in range(int(0.6 / dt)):
        r.drive_velocity((0.0, 0.0), 0.0, r.pose()[2], a_cap=2.0)
        r.step()
    crossed = bool(pose[0] > x_end and not flipped)
    where = "across" if crossed else ("flipped" if flipped else
                                      "before the edge" if x_max < EDGE + 0.05 else
                                      "climbing" if x_max < EDGE + 0.25 else
                                      "on it" if x_max < x_far + 0.2 else "coming off")
    return dict(base=base, obstacle=name, direction=direction, speed=v, crossed=crossed, where=where, x_max=x_max,
                tilt=tilt, airborne_ms=1000 * air, lift_ms=1000 * lift_max, belly=belly, hull=hull,
                held=bool(held0 and r.ball_held()), flipped=flipped, stuck=stuck)


def run_cross(part, bases, names):
    jobs = [(b, n, a, v) for b in bases for n in names for a in DIRS for v in SPEEDS]
    w0 = time.perf_counter()
    rows = _pool(cross_job, jobs)
    print(f"{len(rows)} crossings in {time.perf_counter() - w0:.0f} s")
    out = _merge(part, rows, by=("base", "obstacle"))
    report_cross(part, out[part])


def report_cross(part, rows):
    title = {"A": "A. House transitions", "S": "S. Square steps (up onto a 0.8 m platform, then down) and 76 mm "
                                                "triangular bumps"}[part]
    print(f"{title}: crossed of {len(DIRS)} body directions and peak tilt at 0.5 / 1 / 2 m/s (where it stopped); "
          "driving forward; longest airborne, belly / hull (deck, bumper) on it, balls lost, flips")
    for b in dict.fromkeys(x["base"] for x in rows):
        print(f"  {b}")
        for ob in dict.fromkeys(x["obstacle"] for x in rows if x["base"] == b):
            cells = []
            for v in SPEEDS:
                rr = [x for x in rows if x["base"] == b and x["obstacle"] == ob and x["speed"] == v]
                if not rr:
                    continue
                fails = {}
                for x in rr:
                    if not x["crossed"]:
                        fails[x["where"]] = fails.get(x["where"], 0) + 1
                st = (" (" + ", ".join(f"{n} {w}" for w, n in fails.items()) + ")") if fails else ""
                cells.append(f"{sum(x['crossed'] for x in rr):2d}/{len(rr)} {max(x['tilt'] for x in rr):2.0f} deg{st}")
            rr = [x for x in rows if x["base"] == b and x["obstacle"] == ob]
            lost = sum(not x["held"] for x in rr)
            print(f"    {ob:30s} " + " | ".join(cells) + f"  forward (body 0) tilt "
                  f"{max(x['tilt'] for x in rr if x['direction'] == 0):.0f} deg, air "
                  f"{max(x['airborne_ms'] for x in rr):.0f} ms, belly {sum(x['belly'] for x in rr)}, hull "
                  f"{sum(x['hull'] for x in rr)}, lost {lost}, flips {sum(x['flipped'] for x in rr)}")


def limits(rows, prefix):
    """Per base and speed: the tallest obstacle crossed at every direction, and at any direction (mm)."""
    out = {}
    for b in dict.fromkeys(x["base"] for x in rows):
        for v in SPEEDS:
            hs = sorted({int(x["obstacle"].split()[1]) for x in rows if x["obstacle"].startswith(prefix)})
            every = [h for h in hs if all(x["crossed"] for x in rows if x["base"] == b and x["speed"] == v and
                                          x["obstacle"] == f"{prefix}{h} mm")]
            some = [h for h in hs if any(x["crossed"] for x in rows if x["base"] == b and x["speed"] == v and
                                         x["obstacle"] == f"{prefix}{h} mm")]
            # the tallest h such that every lower one also passed at every direction
            ok = 0
            for h in hs:
                if h in every:
                    ok = h
                else:
                    break
            out[(b, v)] = (ok, max(some) if some else 0)
    return out


def report_limits(rows):
    for prefix, what in (("step ", "square step (up and down)"), ("bump ", "76 mm triangular bump")):
        L = limits([x for x in rows if x["obstacle"].startswith(prefix)], prefix)
        if not L:
            continue
        print(f"  Tallest {what} crossed at every direction / at some direction (mm), at 0.5 / 1 / 2 m/s:")
        for b in dict.fromkeys(k[0] for k in L):
            print(f"    {b:14s} " + " | ".join(f"{L[(b, v)][0]} / {L[(b, v)][1]}" for v in SPEEDS))


# -- G: goal to goal at 2 m/s --------------------------------------------------------------------

MOVE_DIRS = (0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0)
MOVE_DIST = (1.0, 2.0, 4.0)


def move_job(args):
    """A firmware move from rest: dist m in a body direction, 2 m/s cap. Time to settle within 2 cm and 3 cm/s, peak
    speed, peak tilt, the longest a wheel was off the floor (the stop)."""
    base, direction, dist = args
    b = Bench(params(base, area_size=(14.0, 14.0)))
    r, d, m = b.r, b.d, b.m
    b.run(0.3)
    pose0 = r.pose().copy()
    u = np.array([math.cos(math.radians(direction)), math.sin(math.radians(direction))])
    target = np.array([*(pose0[:2] + dist * u), pose0[2]])
    t0, t_done, vmax, tilt, lift_run, lift = d.time, None, 0.0, 0.0, 0.0, 0.0
    while d.time - t0 < 6.0:
        r.drive_to(r.pose(), target)
        r.step()
        sp = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
        vmax, tilt = max(vmax, sp), max(tilt, tilt_deg(d))
        lift_run = lift_run + m.opt.timestep if np.any(r.wheel_loads() < 0.5) else 0.0
        lift = max(lift, lift_run)
        ok = np.linalg.norm(r.pose()[:2] - target[:2]) < 0.02 and sp < 0.03
        if ok and t_done is None:
            t_done = d.time - t0
        elif not ok:
            t_done = None
    return dict(base=base, direction=direction, dist=dist, t=t_done if t_done is not None else float("nan"),
                v_peak=vmax, tilt=tilt, lift_ms=1000 * lift)


def tour_job(args):
    """v9's house tour (bridge_core.TOUR: 19 stops, 46 m) goal to goal at the 2 m/s cap, a ball aboard. A leg that
    hasn't arrived in time is counted stuck and the robot is put at its goal."""
    base, assist = args
    import house
    from bridge_core import TOUR
    from robot_export import V11
    kw = {k: v for k, v in V11.items() if k not in ("wheel_model", "limits", "base_max_speed")}   # house_params sets them
    p = house.house_params("v7", assist=assist, **{**kw, **base_kw(base)})
    drv = house.Driver(p)
    r, d, m = drv.r, drv.d, drv.m
    dt = m.opt.timestep
    tilt = air = air_run = lift = lift_run = vmax = 0.0
    stuck, lost, legs = 0, 0, []
    for k, (x, y, yaw) in enumerate(TOUR):
        target = np.array([x, y, math.radians(yaw)])
        dist = float(np.linalg.norm(r.pose()[:2] - target[:2]))
        t0, done = d.time, False
        while d.time - t0 < 4.0 + 3.0 * dist:
            r.drive_to(r.pose(), target)
            r.step()
            sp = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
            vmax, tilt = max(vmax, sp), max(tilt, tilt_deg(d))
            loads = r.wheel_loads()
            air_run = air_run + dt if np.all(loads < 0.5) else 0.0
            lift_run = lift_run + dt if np.any(loads < 0.5) else 0.0
            air, lift = max(air, air_run), max(lift, lift_run)
            if np.linalg.norm(r.pose()[:2] - target[:2]) < 0.03 and sp < 0.05:
                done = True
                break
        held = r.ball_held()
        legs.append(dict(leg=k, dist=dist, t=d.time - t0, done=done, held=bool(held)))
        if not done:
            stuck += 1
            drv.teleport((x, y, yaw))
        elif not held:
            lost += 1
            drv.load_ball()
    return dict(base=base, assist=assist, t=sum(x["t"] for x in legs if x["done"]), legs=legs, stuck=stuck,
                lost=lost, v_peak=vmax, tilt=tilt, airborne_ms=1000 * air, lift_ms=1000 * lift)


def run_G(bases):
    w0 = time.perf_counter()
    mv = _pool(move_job, [(b, a, dd) for b in bases for a in MOVE_DIRS for dd in MOVE_DIST])
    tr = _pool(tour_job, [(b, a) for b in bases for a in (False, True)])
    print(f"G in {time.perf_counter() - w0:.0f} s")
    _merge("G_moves", mv)
    out = _merge("G_tour", tr)
    report_G(out)


def report_G(out):
    print("G. Goal to goal, firmware adult (2 m/s cap). Flat-floor moves of 1 / 2 / 4 m in 7 body directions: "
          "median / worst time to settle, peak speed, peak tilt, longest wheel lift")
    mv = out.get("G_moves", [])
    for b in dict.fromkeys(x["base"] for x in mv):
        cells = []
        for dd in MOVE_DIST:
            rr = [x for x in mv if x["base"] == b and x["dist"] == dd]
            cells.append(f"{dd:g} m {np.nanmedian([x['t'] for x in rr]):.2f} / {np.nanmax([x['t'] for x in rr]):.2f} s "
                         f"{max(x['v_peak'] for x in rr):.2f} m/s")
        rr = [x for x in mv if x["base"] == b]
        print(f"  {b:14s} " + " | ".join(cells) + f" | tilt {max(x['tilt'] for x in rr):.1f} deg, lift "
              f"{max(x['lift_ms'] for x in rr):.0f} ms")
    print("  House tour (19 stops, 46 m, rugs and thresholds), 2 m/s cap: time, stuck legs, balls lost, peak speed, "
          "peak tilt, longest airborne / wheel lift")
    for x in out.get("G_tour", []):
        print(f"  {x['base']:14s} assist {'on ' if x['assist'] else 'off'}  {x['t']:5.1f} s  stuck {x['stuck']}  lost "
              f"{x['lost']}  {x['v_peak']:.2f} m/s  tilt {x['tilt']:.1f} deg  air {x['airborne_ms']:.0f} / lift "
              f"{x['lift_ms']:.0f} ms")


# -- B: the catch game -----------------------------------------------------------------------------

GAME_BASES = ("X4 + pods", "X4 rigid", "kiwi rigid", "X4 rocker")
GAME_FLOOR_NAMES = ("flat", "home", "home, 12 mm rug at 45 deg")
EXTRA_ROWS = (("X4 rigid, a wheel 1 mm low", "flat"), ("X4 rigid, a wheel 1 mm low", "home"),
              ("X4 rocker, a wheel 1 mm low", "flat"), ("X4 rocker, a wheel 1 mm low", "home"),
              ("kiwi + pods", "flat"), ("kiwi + pods", "home"), ("kiwi + pods", "home, 12 mm rug at 45 deg"))
B_DIST = (6.0, 6.5, 7.0)
B_APEX = (0.75, 1.0, 1.5, 2.0)
TOSSES = 6


def game_job(args):
    base, floor_name, dist, apex, seed = args
    from catch_game import GameConfig
    from catch_world import AREA, HOME, expected_cameras
    from home_floor import GAME_FLOORS
    from robot_model import build_model
    from shallow_sweep import FWG, V6G, HumanToss
    p = params(base, home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, **GAME_FLOORS[floor_name])
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
                               wait_between=1.2, perception="vision", vision_cfg=expected_cameras(), toss_spin=6.0),
                        **FWG, **V6G})
    game = HumanToss(m, d, p, cfg=cfg, seed=seed, verbose=False, dist=dist)
    rim = p.funnel_catch_radius - p.ball_spec.radius
    tilt, flipped = 0.0, False
    while len(game.results) < TOSSES and d.time < 150:
        game.step()
        tilt = max(tilt, tilt_deg(d))
        if tilt > 60:
            flipped = True
            break
    out = [dict(caught=bool(x.caught), why="caught" if x.caught else
                ("reach" if not np.isfinite(x.pos_error) or x.pos_error > rim else "bounce")) for x in game.results]
    out += [dict(caught=False, why="flipped")] * (TOSSES - len(out)) if flipped else []
    return dict(base=base, floor=floor_name, distance=dist, apex=apex, seed=seed, tosses=out, tilt=tilt,
                flipped=flipped)


def run_B(rows_spec, seeds=3):
    jobs = [(b, fl, dd, a, 9000 + 100 * s + 10 * B_DIST.index(dd) + B_APEX.index(a))
            for b, fl in rows_spec for s in range(seeds) for dd in B_DIST for a in B_APEX]
    w0 = time.perf_counter()
    rows = _pool(game_job, jobs)
    print(f"{len(rows)} games in {time.perf_counter() - w0:.0f} s")
    report_B(_merge("B", rows, by=("base", "floor"))["B"])


def report_B(rows):
    print("B. 6-7 m catch game (firmware adult, expected cameras, the drawn robot's masses): caught of the throws "
          "(+/- 1 standard error), not in place / bounced out, peak tilt")
    for b in dict.fromkeys(x["base"] for x in rows):
        for fl in dict.fromkeys(x["floor"] for x in rows if x["base"] == b):
            rr = [x for x in rows if x["base"] == b and x["floor"] == fl]
            ts = [t for x in rr for t in x["tosses"]]
            k = float(np.mean([t["caught"] for t in ts]))
            se = math.sqrt(max(k * (1 - k), 1e-9) / len(ts))
            why = {w: sum(t["why"] == w for t in ts) for w in ("reach", "bounce", "flipped")}
            print(f"  {b:28s} {fl:26s} {100 * k:5.1f}% +/- {100 * se:.1f} of {len(ts)} (not in place {why['reach']}, "
                  f"bounced {why['bounce']}, flipped {why['flipped']}), peak tilt {max(x['tilt'] for x in rr):.0f} deg")


# -- run --------------------------------------------------------------------------------------------

def report():
    out = _load()
    if "T" in out:
        report_T(out["T"])
    for part in ("A", "S"):
        if part in out:
            report_cross(part, out[part])
    if "S" in out:
        report_limits(out["S"])
    if "G_moves" in out or "G_tour" in out:
        report_G(out)
    if "B" in out:
        report_B(out["B"])


if __name__ == "__main__":
    a = sys.argv[1:]
    what = a[0] if a else "report"
    bases = a[1:] or None
    if what == "report":
        report()
    elif what == "T":
        run_T(bases or list(BASES) + list(VARIANTS))
    elif what == "A":
        run_cross("A", bases or list(BASES), HOUSE)
    elif what == "S":
        run_cross("S", bases or list(BASES), STEPS + BUMPS)
        report_limits(_load()["S"])
    elif what == "G":
        run_G(bases or list(BASES))
    elif what == "B":
        spec = [(b, fl) for b in (bases or GAME_BASES) for fl in GAME_FLOOR_NAMES]
        run_B(spec + ([] if bases else list(EXTRA_ROWS)))
    else:
        raise SystemExit(__doc__)
