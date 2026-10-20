"""Motor study (design.md 23.5): is every motor and controller right-sized, are there cheaper ones, and does the base
need four AK40-10s?

A. Duty: what each actuator actually does in play. The round-3 robot (X4 + AK40-10, chute with GM3506 feed motors,
   viscoelastic contacts, rolling resistance) plays catch with return throws from 6.5 m (and 3 m), firmware adult
   mode and physics limits; every physics step logs the drive torque and speed per wheel, the flywheel current and
   speed, the tilt servo's torque and speed, the feed motors' current and the battery current.
B. Drive alternatives in the 6-7 m catch game (72 throws per robot, firmware adult and physics): the AK40-10 X4
   (baseline), the DJI M2006 + C610, a Pololu 37D 24 V brushed gearmotor, the SteadyWin GIM6010-8, and a 3-wheel
   kiwi on AK40-10s (one motor fewer).
C. Flywheel motors on the launcher bench: spin-up time to the indoor 4,500 rpm, the speed dip of a shot, peak current.
D. The tilt servo (it saturates at the sim's 3 N m in A): DS3235 and Feetech STS3250 specs in the 6-7 m catch game.

  python motor_study.py            # A, B, C
  python motor_study.py B          # one part
Writes results/motor_study.json.
"""

from __future__ import annotations

import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import RESULTS, SimParams, save  # noqa: E402
from catalog import DRIVE_MOTORS, FEED_MOTORS, FLYWHEEL_MOTORS  # noqa: E402
from catch_game import GameConfig  # noqa: E402
from launcher_bench import Bench  # noqa: E402
from robot_model import build_model, mass_properties  # noqa: E402
from shallow_sweep import APEXES, AREA, DISTANCES, EXPECTED, FW, FWG, HOME, PH, R3, V6G, HumanToss  # noqa: E402


class DutyLog:
    """Per-step actuator loads, summarized as peaks, RMS and percentiles."""

    def __init__(self, r):
        self.r = r
        self.rows = {k: [] for k in ("tau", "w", "fly_i", "fly_w", "tilt_tau", "tilt_w", "feed_i", "i_bus")}
        self.j_tilt = r.m.jnt_dofadr[r.m.joint("tilt").id]

    def step(self):
        r, d = self.r, self.r.d
        R = self.rows
        R["tau"].append([d.actuator_force[a] for a in r.a_wheel])
        R["w"].append([d.qvel[j] for j in r.j_drive_v])
        R["fly_i"].append(max(abs(f.i_phase) for f in r.fly))
        R["fly_w"].append(float(np.mean(r.flywheel_speeds())))
        R["tilt_tau"].append(float(d.actuator_force[r.a_tilt]))
        R["tilt_w"].append(float(d.qvel[self.j_tilt]))
        R["feed_i"].append(max((f.i for f in r.feed_drives), default=0.0))
        R["i_bus"].append(r.i_total)

    def summary(self, dt):
        R = {k: np.array(v) for k, v in self.rows.items()}
        tau, w = np.abs(R["tau"]), np.abs(R["w"])
        p = tau * w
        return dict(seconds=len(R["i_bus"]) * dt,
                    drive_tau_peak=float(tau.max()), drive_tau_p99=float(np.percentile(tau, 99)),
                    drive_tau_rms=float(np.sqrt((tau ** 2).mean())), drive_w_peak=float(w.max()),
                    drive_w_p99=float(np.percentile(w, 99)), drive_p_peak=float(p.max()),
                    fly_i_peak=float(R["fly_i"].max()), fly_rpm_peak=float(R["fly_w"].max() * 60 / (2 * math.pi)),
                    fly_i_rms=float(np.sqrt((R["fly_i"] ** 2).mean())),
                    tilt_tau_peak=float(np.abs(R["tilt_tau"]).max()), tilt_tau_rms=float(np.sqrt((R["tilt_tau"] ** 2).mean())),
                    tilt_w_peak=float(np.abs(R["tilt_w"]).max()),
                    feed_i_peak=float(R["feed_i"].max()), feed_i_rms=float(np.sqrt((R["feed_i"] ** 2).mean())),
                    i_bus_peak=float(R["i_bus"].max()), i_bus_mean=float(R["i_bus"].mean()))


# -- A ---------------------------------------------------------------------------

def duty_job(args):
    limits, dist, seed = args
    pkw = {**R3, **(FW if limits == "firmware" else {})}
    gkw = {**(FWG if limits == "firmware" else PH), **V6G}
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, **pkw)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(1.0, 2.0), return_throws=True,
                               perception="vision", vision_cfg=EXPECTED, toss_spin=6.0), **gkw})
    game = HumanToss(m, d, p, cfg=cfg, seed=seed, verbose=False, dist=dist)
    log = DutyLog(game.robot)
    while (len(game.results) < 6 or game.state != "wait") and d.time < 120:
        game.step()
        log.step()
    s = log.summary(m.opt.timestep)
    r = game.robot
    s.update(limits=limits, dist=dist, caught=sum(x.caught for x in game.results), n=len(game.results),
             t_winding=max(dr.thermal.t_w for dr in r.drives), soc_used=1 - r.battery.soc,
             feed_e_cu=sum(f.e_cu for f in r.feed_drives))
    return s


# -- B ---------------------------------------------------------------------------

BASES = {
    "X4 x AK40-10 (baseline)": dict(drive_motor="AK40-10"),
    "X4 x DJI M2006 + C610": dict(drive_motor="M2006"),
    "X4 x Pololu 37D 24V 19:1": dict(drive_motor="Pololu-37D-19"),
    "X4 x GIM6010-8": dict(drive_motor="GIM6010-8"),
    "kiwi 3 x AK40-10": dict(drive_motor="AK40-10", layout="kiwi3"),
}
# D. the tilt actuator (it sits at the sim's 3 N m cap in A): real servo specs at the pivot
TILTS = {
    "sim default (3 N m, 5 rad/s)": dict(),
    "DS3235 at 8.4 V (3.4 N m, 9.5 rad/s)": dict(tilt_torque=3.4, tilt_speed=9.5),
    "Feetech STS3250 at 12 V (4.9 N m, 7.9 rad/s)": dict(tilt_torque=4.9, tilt_speed=7.9),
}
TOSSES = 6


def base_job(args):
    name, limits, dist, apex, seed = args
    pkw = {**R3, **{**BASES, **TILTS}[name], **(FW if limits == "firmware" else {})}
    gkw = {**(FWG if limits == "firmware" else PH), **V6G}
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), area_size=AREA, **pkw)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
                               wait_between=1.2, perception="vision", vision_cfg=EXPECTED, toss_spin=6.0), **gkw})
    game = HumanToss(m, d, p, cfg=cfg, seed=seed, verbose=False, dist=dist)
    log = DutyLog(game.robot)
    rim = p.funnel_catch_radius - p.ball_spec.radius
    v_peak = 0.0
    while len(game.results) < TOSSES and d.time < 150:
        game.step()
        log.step()
        v_peak = max(v_peak, float(np.linalg.norm(d.sensor("base_linvel").data[:2])))
    out = [dict(caught=bool(r.caught), why="caught" if r.caught else ("reach" if not np.isfinite(r.pos_error) or
                                                                      r.pos_error > rim else "bounce"))
           for r in game.results]
    s = log.summary(m.opt.timestep)
    mass, cg = mass_properties(m, p)
    return dict(base=name, limits=limits, distance=dist, apex=apex, tosses=out, v_peak=v_peak,
                tau_peak=s["drive_tau_peak"], w_peak=s["drive_w_peak"], i_bus_peak=s["i_bus_peak"],
                t_winding=max(dr.thermal.t_w for dr in game.robot.drives), mass=float(mass), cg=float(cg[2]))


# -- C ---------------------------------------------------------------------------

def fly_job(motor):
    b = Bench(SimParams(flywheel_motor=motor))
    r, d = b.r, b.d
    out = dict(motor=motor)
    for rpm in (3000, 4500):
        r.fly_target, r.fly_brake = 0.0, True              # from rest: brake the wheels to a stop first
        while r.flywheel_speeds().max() > 2.0:
            r.step()
        r.fly_brake = False
        b.set_tilt(0.0)
        b.load_ball()
        r.fly_target = rpm * 2 * math.pi / 60
        t0 = d.time
        i_pk = 0.0
        while not r.flywheels_ready() and d.time - t0 < 3.0:
            r.step()
            i_pk = max(i_pk, max(abs(f.i_phase) for f in r.fly))
        out[f"spinup_{rpm}"] = d.time - t0
        out[f"i_spinup_{rpm}"] = i_pk
        b.run(0.1)
        w0 = r.flywheel_speeds().mean()
        r.gate_open = True
        wmin, t1, t_rec = w0, d.time, None
        while d.time - t1 < 0.6:
            r.step()
            w = r.flywheel_speeds().mean()
            wmin = min(wmin, w)
            if t_rec is None and wmin < 0.97 * w0 and w > 0.98 * w0:
                t_rec = d.time - t1
        out[f"dip_{rpm}"] = 1 - wmin / w0
        out[f"recover_{rpm}"] = t_rec
        r.gate_open = False
        r.fly_target = 0.0
        b.run(0.5)
    return out


# -- run + report -------------------------------------------------------------------

def run(parts):
    path = RESULTS / "motor_study.json"
    out = json.loads(path.read_text()) if path.exists() else {}
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        if "A" in parts:
            out["A"] = list(pool.map(duty_job, [(lim, dd, s) for lim in ("firmware", "physics") for dd in (3.0, 6.5)
                                                for s in range(4)]))
        if "B" in parts:
            jobs = [(nm, lim, dd, a, 9000 + 10 * DISTANCES.index(dd) + APEXES.index(a))
                    for nm in BASES for lim in ("firmware", "physics") for dd in DISTANCES for a in APEXES]
            out["B"] = list(pool.map(base_job, jobs, chunksize=1))
        if "C" in parts:
            out["C"] = list(pool.map(fly_job, list(FLYWHEEL_MOTORS)))
        if "D" in parts:
            jobs = [(nm, "firmware", dd, a, 9000 + 10 * DISTANCES.index(dd) + APEXES.index(a))
                    for nm in TILTS for dd in DISTANCES for a in APEXES]
            out["D"] = list(pool.map(base_job, jobs, chunksize=1))
    save("motor_study", out)
    report(out)


def report(out):
    if "A" in out:
        print("A. Actuator duty in play (catch + return throws, 6 tosses x 4 runs per row): peak / 99% / RMS")
        for lim in ("firmware", "physics"):
            for dd in (3.0, 6.5):
                rs = [x for x in out["A"] if x["limits"] == lim and x["dist"] == dd]
                if not rs:
                    continue
                mx = lambda k: max(x[k] for x in rs)  # noqa: E731
                print(f"   {lim:8s} {dd:3.1f} m: drive torque {mx('drive_tau_peak'):.2f} / {mx('drive_tau_p99'):.2f} / "
                      f"{mx('drive_tau_rms'):.3f} N m, speed {mx('drive_w_peak') * 60 / (2 * math.pi):.0f} / "
                      f"{mx('drive_w_p99') * 60 / (2 * math.pi):.0f} rpm, power {mx('drive_p_peak'):.1f} W, winding "
                      f"{mx('t_winding'):.1f} C | flywheel {mx('fly_i_peak'):.1f} A peak ({mx('fly_i_rms'):.2f} RMS), "
                      f"{mx('fly_rpm_peak'):.0f} rpm | tilt {mx('tilt_tau_peak'):.2f} / {mx('tilt_tau_rms'):.2f} N m, "
                      f"{mx('tilt_w_peak'):.1f} rad/s | feed {mx('feed_i_peak'):.2f} A peak ({mx('feed_i_rms'):.3f} RMS) | "
                      f"bus {mx('i_bus_peak'):.1f} A peak, {np.mean([x['i_bus_mean'] for x in rs]):.2f} A mean | caught "
                      f"{sum(x['caught'] for x in rs)}/{sum(x['n'] for x in rs)}")
        dm = DRIVE_MOTORS["AK40-10"]
        print(f"   AK40-10 for reference: peak {dm.peak_torque:.2f} N m, rated {dm.rated_torque:.2f} N m, no-load "
              f"{dm.noload_rpm:.0f} rpm at 24 V")
    if "B" in out:
        print("B. Drive alternatives, 6-7 m shallow throws (72 per row): caught (not in place / bounced), peak base "
              "speed, drive torque, bus current, winding temp, mass, price of the drive set")
        for nm, kw in BASES.items():
            dm = DRIVE_MOTORS[kw["drive_motor"]]
            nwh = 3 if kw.get("layout") == "kiwi3" else 4
            for lim in ("firmware", "physics"):
                rs = [x for x in out["B"] if x["base"] == nm and x["limits"] == lim]
                if not rs:
                    continue
                ts = [t for x in rs for t in x["tosses"]]
                k = np.mean([t["caught"] for t in ts])
                print(f"   {nm:28s} {lim:8s} {100 * k:4.0f}% ({sum(t['why'] == 'reach' for t in ts)}/"
                      f"{sum(t['why'] == 'bounce' for t in ts)}), {max(x['v_peak'] for x in rs):.2f} m/s, "
                      f"{max(x['tau_peak'] for x in rs):.2f} N m (motor peak {dm.peak_torque:.2f}), "
                      f"{max(x['i_bus_peak'] for x in rs):.1f} A, {max(x['t_winding'] for x in rs):.1f} C, "
                      f"{rs[0]['mass']:.2f} kg, CG {100 * rs[0]['cg']:.1f} cm, ${nwh * dm.price:.0f}")
    if "C" in out:
        print("C. Flywheel motors (bench, 3000 / 4500 rpm): spin-up from rest, dip per shot, recovery, peak current")
        for x in out["C"]:
            fm = FLYWHEEL_MOTORS[x["motor"]]
            print(f"   {x['motor']:18s} ${fm.price:.0f} {1000 * fm.mass:.0f} g: spin-up {x['spinup_3000']:.2f} / "
                  f"{x['spinup_4500']:.2f} s ({x['i_spinup_4500']:.1f} A), dip {100 * x['dip_3000']:.0f} / "
                  f"{100 * x['dip_4500']:.0f}%, recovered in {1000 * (x['recover_4500'] or float('nan')):.0f} ms")
    if "D" in out:
        report_d(out)


def report_d(out):
    print("D. Tilt actuator, 6-7 m shallow throws, firmware adult (72 per row): caught (not in place / bounced)")
    for nm in TILTS:
        ts = [t for x in out["D"] if x["base"] == nm for t in x["tosses"]]
        if ts:
            print(f"   {nm:46s} {100 * np.mean([t['caught'] for t in ts]):4.0f}% ({sum(t['why'] == 'reach' for t in ts)}/"
                  f"{sum(t['why'] == 'bounce' for t in ts)})")


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "report":
        report(json.loads((RESULTS / "motor_study.json").read_text()))
    else:
        run(a or ["A", "B", "C", "D"])
