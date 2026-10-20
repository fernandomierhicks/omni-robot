"""v7 (design.md 24): a base that handles home floors without losing flat-floor catching.

Every crossing uses the explicit-roller wheel model (the sphere wheel can't climb sideways or fail to).

A. Crossings with a ball in the pocket: v6's six obstacles plus the house transitions of floor.HOUSE_OBSTACLES, at
   0.5 / 1 / 2 m/s, at every body direction (0-345 deg in 15 deg steps). Scored per crossing: got across, ball kept,
   peak body tilt, airborne time (all four wheel loads ~0 at once), the longest single-wheel lift, peak drive torque,
   the robot's belly touching the obstacle, wheel-odometry error.
B. The 6-7 m catch game (firmware adult, expected cameras): flat floor and v6's home floor (sigma 0.1 friction field,
   12 mm threshold 0.5 m in front of the waiting spot, 8 mm rug behind it), 216 throws per row.
C. Costs of each base: mass, CG, tip-over limit, ground clearance, wheel rpm at 2 m/s and the drive's speed / torque
   margin on a low battery, parts and price.

  python home_floor.py probe CONFIG [OBSTACLE DIR SPEED]   one crossing, printed
  python home_floor.py A [CONFIG ...]                       crossings (all configs if none given)
  python home_floor.py B [CONFIG ...]                       catch games
  python home_floor.py C                                    costs
  python home_floor.py report
Writes results/home_floor.json (merged per config, so parts can be rerun).
"""

from __future__ import annotations

import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import RESULTS, SimParams, save, tilt_deg  # noqa: E402
import floor as floor_mod  # noqa: E402
from floor import Carpet, Threshold  # noqa: E402
from floor_wheels import OBSTACLES, HOME_FLOOR  # noqa: E402
from launcher_bench import Bench  # noqa: E402
from omni_control import wrap_angle  # noqa: E402
from robot_model import build_model, describe, mass_properties  # noqa: E402
from shallow_sweep import APEXES, AREA, DISTANCES, EXPECTED, FW, FWG, HOME, R3, V6G, HumanToss  # noqa: E402

TOSSES = 6

OBST = {**OBSTACLES, **floor_mod.HOUSE_OBSTACLES}
DIRS = tuple(float(a) for a in range(0, 360, 15))
SPEEDS = (0.5, 1.0, 2.0)

POD = dict(suspension="pod")
CONFIGS = {
    # v6 round 3 base: 96 mm goBILDA, rubber-roller compliance only (0-6 mm), front rocker +/- 3 deg
    "v6": dict(),
    "v6 + pods": dict(**POD),
    "v6 + pods + both rockers 6 deg": dict(**POD, rocker="both", rocker_deg=6.0),
    "v6 + pendulum swing": dict(**POD, swing="pendulum"),
    "v6 + parallel swing": dict(**POD, swing="parallel"),
    "Rotacaster 125": dict(wheel="Rotacaster-125-R2"),
    "DuraOmni 6 in": dict(wheel="AM-DuraOmni-6"),
    "Nexus 152": dict(wheel="Nexus-152"),
    "printed 127 2x6 32 mm": dict(wheel="Printed-127-2x6-32"),
    "printed 152 2x7 36 mm": dict(wheel="Printed-152-2x7-36"),
    "printed 152 3x6 36 mm": dict(wheel="Printed-152-3x6-36"),
    "mecanum 140": dict(wheel="goBILDA-140-mecanum"),
    # round 2: big rollers + sprung pods, terrain-aware firmware
    "mecanum 140 + pods": dict(wheel="goBILDA-140-mecanum", **POD),
    "DuraOmni 6 in + pods": dict(wheel="AM-DuraOmni-6", **POD),
    "printed 127 2x5 36 mm + pods": dict(wheel="Printed-127-2x5-36", **POD),
    "printed 140 2x6 36 mm + pods": dict(wheel="Printed-140-2x6-36", **POD),
    "printed 152 2x7 36 mm + pods": dict(wheel="Printed-152-2x7-36", **POD),
    "printed 152 2x6 40 mm + pods": dict(wheel="Printed-152-2x6-40", **POD),
    "v6 + assist": dict(terrain_assist=True),
    "v6 + pods + assist": dict(**POD, terrain_assist=True),
    "printed 127 2x6 32 mm + pods + assist": dict(wheel="Printed-127-2x6-32", **POD, terrain_assist=True),
}


def params(cfg: str, **kw) -> SimParams:
    return SimParams(wheel_model="rollers", **{**CONFIGS[cfg], **kw})


# -- A ---------------------------------------------------------------------------

def cross_job(args):
    """Drive across an obstacle (its edge at world x = 0.6, running along y) at speed v in world +x, the heading set so
    that this is body direction `direction`; a ball rests in the pocket. With terrain_assist the firmware may turn the
    base (the commanded heading is only where it starts)."""
    cfg, name, direction, v = args[:4]
    extra = args[4] if len(args) > 4 else {}
    p = params(cfg, floor_features=(OBST[name],), **extra)
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
    odo = np.array([*r.pose()[:2], r.pose()[2]])
    tilt = air = air_run = lift_run = lift_max = tq = 0.0
    belly = False
    belly_ids = {m.geom(n).id for n in [f"belly_motor{i}" for i in range(1, p.n_wheels + 1)] + ["belly_battery"]
                 if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) >= 0}
    t0 = d.time
    x_start = r.pose()[0]
    stuck = False
    a_wheel = r.a_wheel
    while d.time - t0 < 8.0:
        pose = r.pose()
        if p.terrain_assist:
            r.drive_velocity((v, 0.0), None, pose[2], a_cap=2.0)      # firmware picks the heading
        else:
            w = float(np.clip(4.0 * wrap_angle(yaw - pose[2]), -2.0, 2.0))
            r.drive_velocity((v, 0.0), w, pose[2], a_cap=2.0)
        r.step()
        tw = r.kin.body_twist(r.wheel_speeds())
        c, s = math.cos(odo[2]), math.sin(odo[2])
        odo += dt * np.array([c * tw[0] - s * tw[1], s * tw[0] + c * tw[1], tw[2]])
        if r.pose()[0] > 0.2:                             # the front of the robot is at the obstacle
            tilt = max(tilt, tilt_deg(d))
            loads = r.wheel_loads()
            air_run = air_run + dt if np.all(loads < 0.5) else 0.0
            air = max(air, air_run)
            lift_run = lift_run + dt if np.any(loads < 0.5) else 0.0
            lift_max = max(lift_max, lift_run)
            tq = max(tq, float(np.max(np.abs(d.actuator_force[a_wheel]))))
            if belly_ids and not belly:
                for k in range(d.ncon):
                    con = d.contact[k]
                    if con.geom1 in belly_ids or con.geom2 in belly_ids:
                        belly = True
                        break
        if r.pose()[0] > 1.25:
            break
        if d.time - t0 > 1.0 + 1.25 / v * 2.5:
            stuck = True
            break
    pose = r.pose()
    for _ in range(int(1.0 / dt)):                        # stop gently, then see whether the ball is still in
        r.drive_velocity((0.0, 0.0), 0.0, r.pose()[2], a_cap=2.0)
        r.step()
    travelled = float(pose[0] - x_start)
    return dict(config=cfg, obstacle=name, direction=direction, speed=v, extra=bool(extra),
                crossed=bool(pose[0] > 1.25 and not stuck), tilt=tilt, airborne_ms=1000 * air,
                lift_ms=1000 * lift_max, torque=tq, belly=belly, heading=math.degrees(wrap_angle(pose[2] - yaw)),
                held=bool(held0 and r.ball_held()), odo_err=float(np.hypot(*(odo[:2] - pose[:2]))),
                odo_err_rel=float(np.hypot(*(odo[:2] - pose[:2])) / max(travelled, 0.1)), upright=tilt_deg(d) < 45)


def summarize_A(rows):
    """Per config and obstacle: crossings ok / all, ball kept, worst tilt, airborne, at each speed."""
    out = {}
    for x in rows:
        k = (x["config"], x["obstacle"], x["speed"])
        o = out.setdefault(k, dict(n=0, crossed=0, held=0, tilt=0.0, air=0.0, lift=0.0, belly=0, torque=0.0, odo=0.0,
                                   stuck_dirs=[]))
        o["n"] += 1
        o["crossed"] += x["crossed"]
        o["held"] += x["held"]
        o["tilt"] = max(o["tilt"], x["tilt"])
        o["air"] = max(o["air"], x["airborne_ms"])
        o["lift"] = max(o["lift"], x["lift_ms"])
        o["belly"] += x["belly"]
        o["torque"] = max(o["torque"], x["torque"])
        o["odo"] = max(o["odo"], x["odo_err"] if x["crossed"] else 0.0)
        if not x["crossed"]:
            o["stuck_dirs"].append(x["direction"])
    return out


def report_A(rows, configs=None):
    S = summarize_A(rows)
    cfgs = configs or list(dict.fromkeys(x["config"] for x in rows))
    obs = list(dict.fromkeys(x["obstacle"] for x in rows))
    print("A. Crossings, ball aboard, every 15 deg of body direction. Per speed 0.5 / 1 / 2 m/s: crossed/all (stuck "
          "directions), ball lost, peak tilt, longest airborne / single-wheel lift (ms)")
    for c in cfgs:
        print(f"  {c}")
        for ob in obs:
            cells = []
            for v in SPEEDS:
                o = S.get((c, ob, v))
                if not o:
                    continue
                st = "" if not o["stuck_dirs"] else " stuck " + ",".join(f"{a:g}" for a in o["stuck_dirs"][:6]) + \
                    ("..." if len(o["stuck_dirs"]) > 6 else "")
                lost = o["n"] - o["held"]
                cells.append(f"{o['crossed']:2d}/{o['n']}{st}{' LOST ' + str(lost) if lost else ''} {o['tilt']:.0f}deg "
                             f"air {o['air']:.0f} lift {o['lift']:.0f}")
            print(f"    {ob:32s} " + " | ".join(cells))


# -- B ---------------------------------------------------------------------------

GAME_FLOORS = {
    "flat": {},
    "home": HOME_FLOOR,
    # v7: the same home floor with the edges turned 45 deg to the throw line (the X4's worst case for square edges)
    "home, 12 mm rug at 45 deg": dict(floor_mu_field=(0.1, 0.5, 3), floor_features=(
        HOME_FLOOR["floor_features"][0],
        Carpet(HOME[0] + 0.3, -1.1, HOME[0] + 2.5, 1.1, thickness=0.012, yaw_deg=45.0))),
}
SEEDS = 3
V7 = "printed 152 2x7 36 mm + pods"          # the proposal (design.md 24)
GAME_ROWS = {      # tag: (config, floor, extra)
    "v6, flat": ("v6", "flat", {}),
    "v6, home": ("v6", "home", {}),
    "v6, home 12 mm rug at 45 deg": ("v6", "home, 12 mm rug at 45 deg", {}),
    "v6 + assist, home 12 mm rug at 45 deg": ("v6", "home, 12 mm rug at 45 deg", dict(terrain_assist=True)),
    "v7, flat": (V7, "flat", {}),
    "v7, home": (V7, "home", {}),
    "v7, home 12 mm rug at 45 deg": (V7, "home, 12 mm rug at 45 deg", {}),
    "v7 + assist, home": (V7, "home", dict(terrain_assist=True)),
    "v7 + assist, home 12 mm rug at 45 deg": (V7, "home, 12 mm rug at 45 deg", dict(terrain_assist=True)),
}


def game_job(args):
    cfg, floor_name, extra, dist, apex, seed = args
    from catch_game import GameConfig
    p = params(cfg, home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA,
               **GAME_FLOORS[floor_name], **R3, **FW, **extra)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg_g = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
                                 wait_between=1.2, perception="vision", vision_cfg=EXPECTED, toss_spin=6.0),
                          **FWG, **V6G})
    game = HumanToss(m, d, p, cfg=cfg_g, seed=seed, verbose=False, dist=dist)
    rim = p.funnel_catch_radius - p.ball_spec.radius
    tilt = air = air_run = 0.0
    flips = 0
    dt = m.opt.timestep
    while len(game.results) < TOSSES and d.time < 150:
        game.step()
        t = tilt_deg(d)
        tilt = max(tilt, t)
        if t > 60:
            flips += 1
            break
        air_run = air_run + dt if np.all(game.robot.wheel_loads() < 0.5) else 0.0
        air = max(air, air_run)
    out = [dict(caught=bool(r.caught), why="caught" if r.caught else ("reach" if not np.isfinite(r.pos_error) or
                                                                      r.pos_error > rim else "bounce"))
           for r in game.results]
    out += [dict(caught=False, why="flipped")] * (TOSSES - len(out)) if flips else []
    return dict(config=cfg, floor=floor_name, extra=json.dumps(extra), distance=dist, apex=apex, seed=seed,
                tosses=out, tilt_max=tilt, airborne_ms=1000 * air, flipped=bool(flips))


def run_B(rows_spec):
    """rows_spec: [(config, floor, extra dict, tag)]."""
    jobs, tags = [], []
    for cfg, fl, extra, tag in rows_spec:
        for s in range(SEEDS):
            for dd in DISTANCES:
                for a in APEXES:
                    jobs.append((cfg, fl, extra, dd, a, 9000 + 100 * s + 10 * DISTANCES.index(dd) + APEXES.index(a)))
                    tags.append(tag)
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(game_job, jobs, chunksize=1))
    for r, t in zip(rows, tags):
        r["tag"] = t
    out = _merge("B", rows, by=("tag",))
    report_B(out["B"])


def report_B(rows):
    print("B. 6-7 m catch game, firmware adult, expected cameras, roller wheel model (216 throws per row, +/-2.5 points)")
    for tag in dict.fromkeys(r["tag"] for r in rows):
        rr = [r for r in rows if r["tag"] == tag]
        ts = [t for r in rr for t in r["tosses"]]
        k = np.mean([t["caught"] for t in ts])
        why = {w: sum(t["why"] == w for t in ts) for w in ("reach", "bounce", "flipped")}
        print(f"  {tag:52s} {100 * k:5.1f}% of {len(ts)} (not in place {why['reach']}, bounced {why['bounce']}, "
              f"flipped {sum(r['flipped'] for r in rr)}), peak tilt {max(r['tilt_max'] for r in rr):.0f} deg, "
              f"longest airborne {max(r['airborne_ms'] for r in rr):.0f} ms")


# -- C ---------------------------------------------------------------------------

POD_PRICE = 15.0          # USD per corner: an RC-car coil-over shock, two flanged bearings, pins (est)
SWING_PRICE = 8.0         # USD per corner: the cross-axis pivot's bearings and pin (est)


def costs(cfg: str, **kw) -> dict:
    """Mass, CG, tip-over limit, clearance, wheel rpm at 2 m/s and the drive's margins, price of the wheel set +
    suspension parts, and whether the printed parts fit the bed."""
    from traction import Base
    p = params(cfg, **kw)
    b = Bench(p)
    b.run(0.6)
    m, d, r = b.m, b.d, b.r
    mass, cg = mass_properties(m, p)
    base = Base(p.wheel_angles_deg, p.wheel_base_radius, cg[2], cg[0], cg[1], p.mu)
    tip = min(base.tip_accel((math.cos(a), math.sin(a))) for a in np.radians(np.arange(0, 360, 5)))
    R = p.wheel_radius
    sag = -float(d.qpos[m.joint("base").qposadr[0] + 2])          # the base origin sinks as the wheels take load
    q = np.array([d.joint(f"susp{i}").qpos[0] for i in range(1, p.n_wheels + 1)])
    travel_left = (p.pod_travel if p.suspension == "pod" else 0.006) - float(q.max())
    batt = R - p.deck_thickness / 2 - 0.023 - sag                  # battery underside (axle deck)
    motor = R - p.motor.diameter / 2 - (sag if p.suspension != "pod" and p.swing == "none" else 0.0)
    clear = min(batt, motor)
    mot = p.motor
    w2 = 2.0 / R                                                    # rad/s at the rim for 2 m/s along a wheel's rolling line
    rpm2 = w2 * 60 / (2 * math.pi)
    rpm22 = mot.noload_rpm * 22.0 / mot.v_rated * p.wheel_speed_frac
    om = p.omni
    price = 4 * om.price + (4 * POD_PRICE if p.suspension == "pod" else 0) + (4 * SWING_PRICE if p.swing != "none" else 0)
    return dict(config=cfg, mass=mass, cg=float(cg[2]), tip=tip, clearance=clear, clearance_at_bump=clear - max(0.0, travel_left),
                static_sag=float(q.mean()), rpm_2ms=rpm2, rpm_avail_22V=rpm22, speed_margin=rpm22 / rpm2 - 1,
                top_speed_22V=rpm22 * 2 * math.pi / 60 * R, rim_force=mot.peak_torque / R, wheel_mass=om.mass,
                wheel=om.name, price=price, printable=om.diameter < 0.25)


def report_C(rows):
    print("C. Costs: mass, CG height, worst tip-over, belly clearance (static / wheel at the bump stop), wheel rpm at "
          "2 m/s on a wheel's rolling line vs the AK40-10 at 22 V, peak rim force, wheels + suspension price")
    for x in rows:
        print(f"  {x['config']:34s} {x['mass']:.2f} kg  CG {100 * x['cg']:.1f} cm  tip {x['tip']:.1f} m/s2  clear "
              f"{1000 * x['clearance']:.0f}/{1000 * x['clearance_at_bump']:.0f} mm  {x['rpm_2ms']:.0f} of "
              f"{x['rpm_avail_22V']:.0f} rpm ({100 * x['speed_margin']:+.0f}%)  top {x['top_speed_22V']:.2f} m/s  "
              f"rim {x['rim_force']:.0f} N  ${x['price']:.0f}")


# -- pod tuning -------------------------------------------------------------------------

TUNE_GRID = dict(pod_travel=(0.010, 0.015, 0.020), pod_rate=(800.0, 1500.0, 3000.0),
                 pod_damping_ratio=(0.3, 0.6, 1.0), pod_preload=(2.0, 6.0))


def tune_pods(cfg="printed 152 2x7 36 mm + pods", obstacles=("threshold 18 mm", "rug edge 12 mm"), dirs=(0.0, 45.0, 90.0, 180.0),
              speed=2.0):
    """Pod travel / rate / damping / preload, scored on the hardest crossings at 2 m/s: peak tilt and the longest
    single-wheel lift (a hop), with airborne time and the ball."""
    import itertools
    keys = list(TUNE_GRID)
    grid = list(itertools.product(*TUNE_GRID.values()))
    jobs = [(cfg, ob, a, speed, dict(zip(keys, g))) for g in grid for ob in obstacles for a in dirs]
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(cross_job, jobs, chunksize=1))
    res = []
    n = len(obstacles) * len(dirs)
    for k, g in enumerate(grid):
        rr = rows[k * n:(k + 1) * n]
        res.append(dict(**dict(zip(keys, g)), tilt=max(r["tilt"] for r in rr), tilt_mean=float(np.mean([r["tilt"] for r in rr])),
                        lift=max(r["lift_ms"] for r in rr), air=max(r["airborne_ms"] for r in rr),
                        crossed=sum(r["crossed"] for r in rr), held=sum(r["held"] for r in rr), n=n))
    res.sort(key=lambda x: x["tilt"] + 0.05 * x["lift"])
    for x in res:
        print(f"travel {1000 * x['pod_travel']:.0f} mm, rate {x['pod_rate']:.0f} N/m, zeta {x['pod_damping_ratio']}, preload "
              f"{x['pod_preload']} N: tilt {x['tilt']:.1f} (mean {x['tilt_mean']:.1f}) deg, lift {x['lift']:.0f} ms, air "
              f"{x['air']:.0f} ms, crossed {x['crossed']}/{x['n']}, held {x['held']}/{x['n']}")
    out = json.loads((RESULTS / "home_floor.json").read_text()) if (RESULTS / "home_floor.json").exists() else {}
    out["tune"] = res
    save("home_floor", out)


# -- run ----------------------------------------------------------------------------

def _merge(key, rows, by=("config",)):
    path = RESULTS / "home_floor.json"
    out = json.loads(path.read_text()) if path.exists() else {}
    new_keys = {tuple(r[k] for k in by) for r in rows}
    out[key] = [r for r in out.get(key, []) if tuple(r[k] for k in by) not in new_keys] + rows
    save("home_floor", out)
    return out


def run_A(cfgs, obstacles=None, dirs=DIRS, speeds=SPEEDS, extra=None, tag=None):
    jobs = [(c, ob, a, v, extra or {}) for c in cfgs for ob in (obstacles or OBST) for a in dirs for v in speeds]
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(cross_job, jobs, chunksize=1))
    if tag:
        for r in rows:
            r["config"] = tag
    out = _merge("A", rows)
    report_A(out["A"], [tag] if tag else cfgs)
    return rows


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a or a[0] == "report":
        out = json.loads((RESULTS / "home_floor.json").read_text())
        if "A" in out:
            report_A(out["A"])
    elif a[0] == "probe":
        cfg = a[1]
        ob = a[2] if len(a) > 2 else "rug edge 12 mm"
        dr = float(a[3]) if len(a) > 3 else 45.0
        v = float(a[4]) if len(a) > 4 else 1.0
        p = params(cfg)
        print(describe(build_model(p), p))
        print(cross_job((cfg, ob, dr, v)))
    elif a[0] == "A":
        run_A(a[1:] or list(CONFIGS))
    elif a[0] == "B":
        want = a[1:] or list(GAME_ROWS)
        run_B([(*GAME_ROWS[t], t) for t in want])
    elif a[0] == "tune":
        tune_pods(*a[1:2])
    elif a[0] == "C":
        with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
            rows = list(pool.map(costs, a[1:] or list(CONFIGS)))
        _merge("C", rows)
        report_C(rows)
