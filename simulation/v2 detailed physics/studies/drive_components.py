"""Drive component trade study: wheel motor x omni wheel.

For each combination the firmware acceleration limit is set to 85% of the
worst-direction traction limit (quasi-static model, validated by wheel_slip.py),
capped by what the motors can deliver. Then:
  - top speed at full and nearly empty battery,
  - time to make a 0.37 m stop-and-catch move, forward (worst) and sideways,
  - catch rate over 20 tosses aimed up to 0.45 m from home (slightly past the
    nominal reach, so the options separate),
  - copper loss over a repeated catch-move duty cycle -> steady-state winding temperature,
  - peak battery current and voltage sag, mass and cost.

  python drive_components.py            # full grid
  python drive_components.py quick      # baseline only
Writes results/drive_components.json.
"""

from __future__ import annotations

import math
import sys
import time

import mujoco
import numpy as np

from common import SimParams, analytic_max_accel, cg_height, goto_run, make_robot, save
from catalog import DRIVE_MOTORS, OMNI_WHEELS
from catch_game import CatchGame, GameConfig
from robot_model import build_model

MIN_CLEARANCE = 0.015


def top_speed(p: SimParams, v_bus: float) -> float:
    """Ground speed where the motoring torque limit falls to the friction torque."""
    s = p.motor
    k_out = s.kt * s.gear * s.gear_eff
    i_f = s.backdrive_torque / k_out
    w_out = (v_bus - s.r_eq * i_f) / (s.ke * s.gear)
    return w_out * p.wheel_radius


def motor_accel_limit(p: SimParams, mass: float, direction_deg: float = 0.0) -> float:
    s = p.motor
    f_max = s.peak_torque / p.wheel_radius
    u = np.array([math.cos(math.radians(direction_deg)), math.sin(math.radians(direction_deg))])
    worst = max(abs(-math.sin(math.radians(a)) * u[0] + math.cos(math.radians(a)) * u[1]) for a in p.wheel_angles_deg)
    return 1.5 * f_max / (mass * worst)


def thermal_cycle(p: SimParams, accel: float, seconds: float = 40.0, period: float = 2.0, dist: float = 0.37):
    m, d, r = make_robot(p)
    r.ctl.max_accel = accel
    home = r.pose()
    e0 = [0.0, 0.0, 0.0]
    t0 = d.time
    i_peak, v_min = 0.0, r.battery.v
    k = 0
    while d.time - t0 < seconds:
        phase = int((d.time - t0) / period)
        ang = math.radians(60 * phase)                       # vary direction each move
        tgt = home.copy()
        if phase % 2 == 0:
            tgt[:2] += dist * np.array([math.cos(ang), math.sin(ang)])
        pose = r.pose()
        r.drive_to(pose, tgt)
        r.step()
        for j, drv in enumerate(r.drives):
            e0[j] += drv.p_loss * m.opt.timestep
        i_peak = max(i_peak, r.i_total)
        v_min = min(v_min, r.battery.v)
        k += 1
    p_avg = max(e0) / seconds
    th = p.motor.thermal
    t_ss = 25.0 + p_avg * (th.r_wc + th.r_ca)
    return dict(p_copper_avg=p_avg, t_winding_steady=t_ss, i_peak=i_peak, v_min=v_min,
                t_winding_40s=max(dr.thermal.t_w for dr in r.drives))


def catch_rate(p: SimParams, accel: float, tosses: int = 20, zone: float = 0.45, seed: int = 7):
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    game = CatchGame(m, d, p, GameConfig(catch_zone_radius=zone), seed=seed, verbose=False)
    game.robot.ctl.max_accel = accel
    while len(game.results) < tosses:
        game.step()
    caught = sum(x.caught for x in game.results)
    errs = [x.pos_error for x in game.results if np.isfinite(x.pos_error)]
    return dict(caught=caught, tosses=tosses, median_error=float(np.median(errs)) if errs else float("nan"),
                peak_current=max(x.peak_current for x in game.results),
                v_min=min(x.v_min for x in game.results))


def evaluate(motor_key: str, wheel_key: str, full: bool = True) -> dict:
    p = SimParams(drive_motor=motor_key, wheel=wheel_key)
    mot, wh = p.motor, p.omni
    h, mass, cxy = cg_height(p, with_xy=True)
    clearance = p.wheel_radius - mot.diameter / 2
    row = dict(motor=motor_key, wheel=wheel_key, mass=mass, cg_height=h, clearance=clearance,
               cost=3 * (mot.price + wh.price),
               v_top_full=top_speed(p, 29.0), v_top_low=top_speed(p, 22.0))
    a_trac = min(analytic_max_accel(p, a, h, cxy) for a in range(0, 360, 5))
    a_mot = motor_accel_limit(p, mass)
    row.update(a_traction=a_trac, a_motor=a_mot, accel_limit=min(0.85 * a_trac, a_mot))
    row["feasible"] = clearance >= MIN_CLEARANCE and row["v_top_low"] >= 1.4
    if not full:
        return row
    a = row["accel_limit"]
    p_run = p.with_(base_max_speed=min(1.4, 0.95 * row["v_top_low"]))
    t_fwd, _, _ = goto_run(p_run, (0.37, 0.0), a)
    t_side, _, _ = goto_run(p_run, (0.0, 0.37), a)
    row.update(t_move_forward=t_fwd, t_move_side=t_side)
    row.update(thermal_cycle(p_run, a))
    row["catch"] = catch_rate(p_run, a)
    return row


def main():
    quick = len(sys.argv) > 1 and sys.argv[1] == "quick"
    combos = [("AK40-10", "goBILDA-96")] if quick else [(mk, wk) for mk in DRIVE_MOTORS for wk in OMNI_WHEELS]
    rows = []
    for mk, wk in combos:
        t0 = time.perf_counter()
        pre = evaluate(mk, wk, full=False)
        full = pre["clearance"] >= MIN_CLEARANCE      # run the sim even when too slow, to show the effect
        row = evaluate(mk, wk, full=full)
        rows.append(row)
        c = row.get("catch", {})
        print(f"{mk:10s} {wk:11s} mass {row['mass']:.2f} kg  CG {100 * row['cg_height']:.1f} cm  clear {1000 * row['clearance']:.0f} mm  "
              f"vtop {row['v_top_full']:.2f}/{row['v_top_low']:.2f} m/s  a_trac {row['a_traction']:.2f} a_mot {row['a_motor']:.1f}  "
              f"moves {row.get('t_move_forward', float('nan')) or float('nan'):.2f}/{row.get('t_move_side', float('nan')) or float('nan'):.2f} s  "
              f"T_ss {row.get('t_winding_steady', float('nan')):.0f} C  "
              f"catch {c.get('caught', '-')}/{c.get('tosses', '-')}  ${row['cost']:.0f}  ({time.perf_counter() - t0:.0f}s)", flush=True)
    save("drive_components" + ("_quick" if quick else ""), rows)

    if not quick:
        # catch rate vs firmware acceleration limit for the baseline combo
        p = SimParams()
        sweep = []
        for a in (1.5, 2.0, 2.5, 3.0, 3.5, 4.0):
            c = catch_rate(p, a)
            sweep.append(dict(accel=a, **c))
            print(f"accel limit {a}: caught {c['caught']}/{c['tosses']}", flush=True)
        save("accel_sweep", sweep)


if __name__ == "__main__":
    main()
