"""Battery study: current peaks, voltage sag, runtime and low-charge performance.

Scenarios per pack option:
  burst    - hardest base move (worst direction, full traction-limited accel to
             top speed and back) while both flywheels spin up to max speed.
  session  - 20-toss catch game: average power -> runtime estimate.
  low SoC  - top speed and launcher max exit speed at 10% charge, cold-ish pack.

Writes results/battery.json.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from common import SimParams, make_robot, save
from catalog import BATTERIES
from catch_game import CatchGame, GameConfig
from models import Battery
from robot_model import build_model


def burst(p: SimParams):
    m, d, r = make_robot(p)
    r.ctl.max_accel = 3.0
    r.ctl.max_speed = 2.0
    start = r.pose()
    tgt = start + np.array([-1.0 * math.cos(math.radians(60)), -1.0 * math.sin(math.radians(60)), 0.0])
    r.fly_target = 4200 * 2 * math.pi / 60
    i_peak, v_min, t0 = 0.0, r.battery.v, d.time
    while d.time - t0 < 1.5:
        pose = r.pose()
        r.drive_to(pose, tgt if d.time - t0 < 0.9 else start)
        r.step()
        i_peak = max(i_peak, r.i_total)
        v_min = min(v_min, r.battery.v)
    return dict(i_peak=i_peak, v_min=v_min, v_open=Battery(p.pack).v)


def session(p: SimParams, tosses: int = 12):
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    game = CatchGame(m, d, p, GameConfig(), seed=11, verbose=False)
    e, t0 = 0.0, d.time
    soc0 = game.robot.battery.soc
    while len(game.results) < tosses:
        game.step()
        e += game.robot.battery.v * game.robot.i_total * m.opt.timestep
    dt = d.time - t0
    p_avg = e / dt
    wh = p.pack.capacity * 3.6 * p.pack.series      # nominal pack energy, Wh
    return dict(power_avg=p_avg, runtime_h=0.85 * wh / p_avg, soc_used_per_min=(soc0 - game.robot.battery.soc) / dt * 60)


def low_charge(p: SimParams):
    b = Battery(p.pack, soc=0.10, t_c=15.0)
    v = b.step(8.0, 0.1)               # under a typical 8 A move load
    s = p.motor
    w = (v - s.r_eq * 1.0) / (s.ke * s.gear)
    fm = p.fly_motor
    ke = 1 / (fm.kv * 2 * math.pi / 60)
    return dict(v_loaded=v, top_speed=w * p.wheel_radius, flywheel_max_rpm=0.9 * v / ke * 60 / (2 * math.pi))


def main():
    rows = []
    for key in BATTERIES:
        p = SimParams(battery=key)
        row = dict(battery=key, name=p.pack.name, mass=p.pack.mass, price=p.pack.price,
                   r0=p.pack.r0, capacity=p.pack.capacity)
        row.update(burst=burst(p), session=session(p), low=low_charge(p))
        rows.append(row)
        b, s, lo = row["burst"], row["session"], row["low"]
        print(f"{key:10s} {row['mass'] * 1000:.0f} g, R0 {row['r0'] * 1000:.0f} mOhm: burst {b['i_peak']:.1f} A, "
              f"{b['v_open']:.2f} -> {b['v_min']:.2f} V; play {s['power_avg']:.0f} W -> {s['runtime_h']:.1f} h; "
              f"at 10% SoC {lo['v_loaded']:.1f} V: top speed {lo['top_speed']:.2f} m/s, flywheel max {lo['flywheel_max_rpm']:.0f} rpm",
              flush=True)
    save("battery", rows)


if __name__ == "__main__":
    main()
