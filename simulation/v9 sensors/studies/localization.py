"""v9 (design.md 27): how well each base knows where it is, with the sensors the real robot will carry.

The robot drives on the sim's truth (shadow mode) except in part P, where it positions itself on its own fused estimate.
Every estimator variant (estimator.Localizer.VARIANTS) sees the same readings:

  wheels            dead reckoning on the encoders
  wheels+IMU        + the gyro and accelerometers (ICM-42688-P)
  wheels+IMU+flow   + a downward optical-flow sensor (PAA5100JE)
  wheels+IMU+lidar  + RPLIDAR C1 matched to the known map
  all               every sensor

  T  house tour: ~75 s through every door and transition at up to 2 m/s (firmware), then 2 s pushing into a wall
     at 0.6 m/s (wheels spin, the base doesn't move). Position / yaw error along the way (median, 95%, max), dead-
     reckoning drift per metre, lidar fixes rejected, re-localizations. 3 sensor seeds per base.
  C  the 6-7 m catch game on the flat and home floors: what a catch needs is the robot's own motion over the ~1 s
     flight (the ball is measured from the robot), so: error of the estimated displacement over every 1 s window.
  P  positioning for an arm, on the fused estimate: hold a pose against a 10 N push at 0.35 m, 5 / 10 / 20 mm moves,
     30 s of station keeping; errors are the truth's. And the static push that tips each base over, at arm heights
     (20 N at 0.35 m tipped the X4: an arm's reach is limited by tipping before positioning).

  python localization.py [T|C|P|report]
"""

from __future__ import annotations

import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import house  # noqa: E402
from common import RESULTS, save  # noqa: E402
from estimator import Localizer  # noqa: E402
from omni_control import wrap_angle  # noqa: E402
from sensors import Sensors, SensorConfig  # noqa: E402

BASES = ("v6", "v7", "v8")
PUSH = 10.0          # N at 0.35 m in the hold test. 20 N tipped the X4 over: see tip_forces()
VARIANTS = tuple(Localizer.VARIANTS)

# the tour: (x, y, yaw deg) waypoints from the hallway start, through each door (on its axis) and back
TOUR = [(1.5, 1.25, 0), (3.45, 1.25, 30), (3.45, 3.0, 90), (4.6, 3.25, 0), (3.45, 3.0, -60), (3.45, 1.25, -90),
        (0.45, 1.25, 180), (0.45, 3.2, 90), (0.45, 1.25, 45), (-3.0, 1.25, 180), (-3.0, 3.3, 90), (-3.0, 1.25, 0),
        (-3.0, -0.6, -90), (-3.1, -2.4, -135), (-0.8, -2.0, 0), (2.5, -2.0, 60), (2.5, -0.2, 90), (-1.0, 0.0, 180),
        (-3.0, 1.25, 0)]


def _new(base, seed, floor_kw=None):
    drv = house.Driver(house.house_params(base, assist=False))
    sn = Sensors(drv.r, SensorConfig(seed=seed))
    loc = Localizer(drv.r, sn)
    return drv, sn, loc


def _stats(errs):
    e = np.array(errs)
    return dict(med=float(np.median(e)), p95=float(np.percentile(e, 95)), max=float(np.max(e)))


# -- T ----------------------------------------------------------------------------------------------

def tour_job(args):
    base, seed = args
    drv, sn, loc = _new(base, seed)
    r, d = drv.r, drv.d
    pos_err = {k: [] for k in loc.f}
    yaw_err = {k: [] for k in loc.f}
    dist, last = 0.0, r.pose()[:2].copy()
    nxt = 0.0
    t_start = d.time
    for wp in TOUR:
        target = np.array([wp[0], wp[1], math.radians(wp[2])])
        t0 = d.time
        while d.time - t0 < 8.0:
            r.drive_to(r.pose(), target)
            r.step()
            loc.step()
            pose = r.pose()
            dist += float(np.linalg.norm(pose[:2] - last))
            last = pose[:2].copy()
            if d.time >= nxt:
                nxt = d.time + 0.1
                for k, (pe, ye) in loc.errors().items():
                    pos_err[k].append(pe)
                    yaw_err[k].append(ye)
            if np.linalg.norm(pose[:2] - target[:2]) < 0.03 and abs(wrap_angle(pose[2] - target[2])) < 0.05:
                break
    tour_err = {k: loc.errors()[k] for k in loc.f}
    t_tour = d.time - t_start
    # the push into a wall: the hallway's north wall from the start spot, at 0.6 m/s for 2 s, then back off 0.5 m
    before = {k: loc.errors()[k][0] for k in loc.f}
    t0 = d.time
    while d.time - t0 < 3.0:
        r.drive_velocity((0.0, 0.6), 0.0, r.pose()[2])
        r.step()
        loc.step()
    t0 = d.time
    while d.time - t0 < 1.5:
        r.drive_velocity((0.0, -0.4), 0.0, r.pose()[2])
        r.step()
        loc.step()
    after_wall = {k: loc.errors()[k][0] for k in loc.f}
    return dict(base=base, seed=seed, dist=dist, t=t_tour,
                pos={k: _stats(v) for k, v in pos_err.items()}, yaw={k: _stats(v) for k, v in yaw_err.items()},
                end={k: v for k, v in tour_err.items()}, wall_before=before, wall_after=after_wall,
                lidar={k: (f.n_lidar, f.n_lidar_rej, f.n_reloc) for k, f in loc.f.items() if f.use["lidar"]})


# -- C ----------------------------------------------------------------------------------------------

def catch_job(args):
    cfg, floor_name, dist, apex, seed = args
    import home_floor as H
    from catch_game import GameConfig
    p = H.params(cfg, home_xy=H.HOME, thrower_xy=(H.HOME[0] - dist - 0.3, 0.0), area_size=H.AREA,
                 **H.GAME_FLOORS[floor_name], **H.R3, **H.FW)
    m = H.build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    g = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
                             wait_between=1.2, perception="vision", vision_cfg=H.EXPECTED, toss_spin=6.0),
                      **H.FWG, **H.V6G})
    game = H.HumanToss(m, d, p, cfg=g, seed=seed, verbose=False, dist=dist)
    r = game.robot
    sn = Sensors(r, SensorConfig(seed=seed))
    loc = Localizer(r, sn)
    buf = []                  # (t, truth, {variant: estimate}) at 20 Hz
    nxt = 0.0
    while len(game.results) < 6 and d.time < 150:
        game.step()
        loc.step()
        if d.time >= nxt:
            nxt = d.time + 0.05
            buf.append((d.time, r.pose().copy(), {k: f.pose() for k, f in loc.f.items()}))
    # error of the estimated displacement over every 1 s window (the robot's own motion during a flight)
    rel = {k: [] for k in loc.f}
    for i in range(len(buf) - 20):
        t0, p0, e0 = buf[i]
        t1, p1, e1 = buf[i + 20]
        dp = p1[:2] - p0[:2]
        if np.linalg.norm(dp) < 0.05:          # only windows where the robot moved
            continue
        for k in loc.f:
            de = e1[k][:2] - e0[k][:2]
            rel[k].append(float(np.linalg.norm(de - dp)))
    absol = {k: loc.errors()[k] for k in loc.f}
    return dict(config=cfg, floor=floor_name, seed=seed, rel={k: v for k, v in rel.items()}, end=absol,
                caught=sum(x.caught for x in game.results))


# -- P ----------------------------------------------------------------------------------------------

def precision_job(args):
    """Positioning on the fused estimate ('all'). kind: 'hold' (10 N push at 0.35 m, 1 s, in a body direction),
    'step' (a small move), 'station' (30 s of holding still)."""
    base, kind, val, seed = args
    drv, sn, loc = _new(base, seed)
    r, d, m = drv.r, drv.d, drv.m
    f = loc.f["all"]
    for _ in range(int(2.0 / m.opt.timestep)):           # settle and let the filter converge
        r.drive_velocity((0.0, 0.0), 0.0, r.pose()[2])
        r.step()
        loc.step()
    target_est = f.pose()
    truth0 = r.pose().copy()
    bid = m.body("base").id
    out = dict(base=base, kind=kind, val=val, seed=seed)

    def run(dur, push=None, target=None):
        worst = 0.0
        t0 = d.time
        while d.time - t0 < dur:
            d.xfrc_applied[bid] = 0.0
            if push is not None:
                com = d.xipos[bid]
                pt = d.xpos[bid] + np.array([0.0, 0.0, 0.35])
                d.xfrc_applied[bid, :3] = push
                d.xfrc_applied[bid, 3:] = np.cross(pt - com, push)
            r.drive_to(f.pose(), target if target is not None else target_est)
            r.step()
            loc.step()
            worst = max(worst, float(np.linalg.norm(r.pose()[:2] - goal_truth[:2])))
        d.xfrc_applied[bid] = 0.0
        return worst

    goal_truth = truth0
    if kind == "hold":
        a = math.radians(val) + truth0[2]
        out["push_mm"] = 1000 * run(1.0, push=np.array([PUSH * math.cos(a), PUSH * math.sin(a), 0.0]))
        run(1.5)
        out["after_mm"] = 1000 * float(np.linalg.norm(r.pose()[:2] - truth0[:2]))
    elif kind == "step":
        u = np.array([math.cos(math.radians(seed * 45.0)), math.sin(math.radians(seed * 45.0))])
        tgt = target_est.copy()
        tgt[:2] += val * u
        goal_truth = truth0.copy()
        goal_truth[:2] += val * u
        run(2.0, target=tgt)
        out["err_mm"] = 1000 * float(np.linalg.norm(r.pose()[:2] - goal_truth[:2]))
    else:
        run(val)
        out["drift_mm"] = 1000 * float(np.linalg.norm(r.pose()[:2] - truth0[:2]))
        out["yaw_deg"] = math.degrees(abs(wrap_angle(r.pose()[2] - truth0[2])))
    return out


def tip_forces(base):
    """The horizontal force at a height that tips the base over, worst and best direction: F = m g d / h, d = the
    support polygon's distance from the CG in that direction (traction.Base: tip_accel = g d / h_cg)."""
    from robot_model import build_model, mass_properties
    from traction import Base
    p = house.house_params(base, assist=False)
    m = build_model(p)
    mass, cg = mass_properties(m, p)
    b = Base(p.wheel_angles_deg, p.wheel_base_radius, cg[2], cg[0], cg[1], p.mu)
    tips = [b.tip_accel((math.cos(a), math.sin(a))) for a in np.radians(np.arange(0, 360, 5))]
    out = dict(base=base, mass=mass, cg=float(cg[2]))
    for h in (0.35, 0.5):
        out[f"F_{h}"] = (mass * min(tips) * cg[2] / h, mass * max(tips) * cg[2] / h)
    return out


# -- run + report -------------------------------------------------------------------------------------

def _merge(key, rows):
    path = RESULTS / "localization.json"
    out = json.loads(path.read_text()) if path.exists() else {}
    out[key] = rows
    save("localization", out)
    return out


def run(parts):
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        if "T" in parts:
            rows = list(pool.map(tour_job, [(b, s) for b in BASES for s in range(3)], chunksize=1))
            report_T(_merge("T", rows)["T"])
        if "C" in parts:
            import home_floor as H
            cfgs = {"v6": "v6", "v7": "v7 on DM-2325", "v8": "swerve3 v2, r 170"}
            jobs = [(cfgs[b], fl, dd, a, 9000 + 10 * H.DISTANCES.index(dd) + H.APEXES.index(a))
                    for b in BASES for fl in ("flat", "home") for dd in H.DISTANCES for a in H.APEXES]
            rows = list(pool.map(catch_job, jobs, chunksize=1))
            report_C(_merge("C", rows)["C"])
        if "P" in parts:
            jobs = [(b, "hold", a, 0) for b in BASES for a in (0.0, 45.0, 90.0, 180.0)]
            jobs += [(b, "step", st, s) for b in BASES for st in (0.005, 0.01, 0.02) for s in range(4)]
            jobs += [(b, "station", 30.0, s) for b in BASES for s in range(2)]
            rows = list(pool.map(precision_job, jobs, chunksize=1))
            out = _merge("P", rows)
            out = _merge("tip", [tip_forces(b) for b in BASES])
            report_P(out["P"], out["tip"])


def report_T(rows):
    print("T. House tour (~75 s, up to 2 m/s, every door and transition), 3 seeds: position error median / 95% / max "
          "(cm), yaw 95% (deg), error at the end; then 3 s pushing a wall: error after")
    for b in dict.fromkeys(x["base"] for x in rows):
        rr = [x for x in rows if x["base"] == b]
        print(f"  {b} (tour {np.mean([x['dist'] for x in rr]):.0f} m in {np.mean([x['t'] for x in rr]):.0f} s)")
        for k in VARIANTS:
            pos = [x["pos"][k] for x in rr]
            print(f"    {k:18s} {100 * np.mean([p['med'] for p in pos]):6.1f} / {100 * np.mean([p['p95'] for p in pos]):6.1f} / "
                  f"{100 * max(p['max'] for p in pos):6.1f} cm, yaw {max(x['yaw'][k]['p95'] for x in rr):5.1f} deg, end "
                  f"{100 * np.mean([x['end'][k][0] for x in rr]):6.1f} cm | wall: {100 * np.mean([x['wall_before'][k] for x in rr]):5.1f} -> "
                  f"{100 * np.mean([x['wall_after'][k] for x in rr]):6.1f} cm"
                  + (f" | lidar fixes {sum(x['lidar'][k][0] for x in rr)}, rejected {sum(x['lidar'][k][1] for x in rr)}, "
                     f"relocalized {sum(x['lidar'][k][2] for x in rr)}" if k in rr[0]["lidar"] else ""))


def report_C(rows):
    print("C. Catch game (6-7 m, firmware adult), estimators in shadow: error of the robot's own displacement over 1 s "
          "windows while moving (median / 95% / max, mm); position error at the end of 6 throws (cm)")
    for c in dict.fromkeys(x["config"] for x in rows):
        for fl in ("flat", "home"):
            rr = [x for x in rows if x["config"] == c and x["floor"] == fl]
            if not rr:
                continue
            print(f"  {c} on {fl} ({sum(x['caught'] for x in rr)} caught of {6 * len(rr)})")
            for k in VARIANTS:
                v = np.concatenate([x["rel"][k] for x in rr]) * 1000
                print(f"    {k:18s} {np.median(v):6.1f} / {np.percentile(v, 95):6.1f} / {v.max():7.1f} mm | end "
                      f"{100 * np.median([x['end'][k][0] for x in rr]):6.1f} cm")


def report_P(rows, tips=()):
    for t in tips:
        print(f"  tip-over push, {t['base']} ({t['mass']:.2f} kg, CG {100 * t['cg']:.1f} cm): "
              + ", ".join(f"at {h} m {t[f'F_{h}'][0]:.0f} N (worst direction) to {t[f'F_{h}'][1]:.0f} N" for h in (0.35, 0.5)))
    print("P. Positioning on the fused estimate (all sensors): 10 N push at 0.35 m (worst deflection / after release, "
          "mm), small moves (worst error, mm), 30 s station keeping (drift mm, yaw deg)")
    for b in dict.fromkeys(x["base"] for x in rows):
        rr = [x for x in rows if x["base"] == b]
        h = [x for x in rr if x["kind"] == "hold"]
        st = [x for x in rr if x["kind"] == "step"]
        sk = [x for x in rr if x["kind"] == "station"]
        print(f"  {b}: push {max(x['push_mm'] for x in h):.1f} / {max(x['after_mm'] for x in h):.1f} mm | moves "
              + " ".join(f"{1000 * v:.0f} mm: {max(x['err_mm'] for x in st if x['val'] == v):.1f}" for v in (0.005, 0.01, 0.02))
              + f" mm | station {max(x['drift_mm'] for x in sk):.1f} mm, {max(x['yaw_deg'] for x in sk):.2f} deg")


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "report":
        out = json.loads((RESULTS / "localization.json").read_text())
        for k, fn in (("T", report_T), ("C", report_C)):
            if k in out:
                fn(out[k])
        if "P" in out:
            report_P(out["P"], out.get("tip", ()))
    else:
        run(a or ["T", "C", "P"])
