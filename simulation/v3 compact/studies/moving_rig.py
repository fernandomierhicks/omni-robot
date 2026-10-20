"""Catching on the move: what the catch module tolerates (stage 1 of the moving catch).

A. Entry while moving. The base drives at a steady velocity (with the ball's
   horizontal travel, against it, or sideways, 0-2 m/s). A ball arrives at
   6.4 m/s, 24 deg from vertical in the world (a 2.7 m toss), landing 0-7 cm
   from the rim center. The funnel faces and tilts into the ball's velocity
   *relative to the funnel* (60%, capped at 15 deg). After entry: 0.2 s coast,
   dump upright, then brake gently. Counted as caught if held 1 s later.

B. Braking with a ball aboard. The ball sits in the pocket; the base cruises at
   1.5 m/s (forward, backward or sideways relative to its heading) with the
   funnel at a catch tilt, then dumps upright and brakes at a fixed rate. Does
   the ball stay in? This sets the braking limit after a moving catch.

Writes results/moving_rig.json.
"""

from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from common import SimParams, make_robot, save
from omni_control import wrap_angle

V_BALL = 6.4 * np.array([-math.sin(math.radians(24)), 0.0, -math.cos(math.radians(24))])
OFFSETS = [(0.0, 0.0)] + [(r * math.cos(a), r * math.sin(a)) for r in (0.04, 0.07)
                          for a in np.radians(np.arange(0, 360, 45))]
DIRS = {"with the ball": (-1.0, 0.0), "against the ball": (1.0, 0.0), "sideways": (0.0, 1.0)}
SPEEDS = (0.0, 0.5, 1.0, 1.5, 2.0)
P = SimParams(base_max_speed=2.0)


def _place_base(d, m, xy, yaw):
    adr = m.joint("base").qposadr[0]
    d.qpos[adr:adr + 2] = xy
    d.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]


def _drive(r, d, v_world, yaw_des, a_cap=None):
    pose = r.pose()
    w = float(np.clip(4.0 * wrap_angle(yaw_des - pose[2]), -3, 3))
    r.drive_velocity(v_world, w, pose[2], a_cap=a_cap)
    r.step()


def entry_trial(args):
    """One ball onto a base moving at v_r; returns caught."""
    v_r, off, brake = args
    v_r = np.asarray(v_r, dtype=float)
    v_rel = V_BALL - np.array([*v_r, 0.0])
    yaw = math.atan2(-v_rel[1], -v_rel[0])
    ang = math.atan2(math.hypot(v_rel[0], v_rel[1]), -v_rel[2])
    tilt = min(0.6 * ang, math.radians(15.0))
    import mujoco
    m, d, r = make_robot(P, settle=0.05)
    sp = float(np.linalg.norm(v_r))
    start = -v_r / sp * min(1.4, 0.3 + 0.55 * sp) if sp > 0 else np.zeros(2)
    _place_base(d, m, start, yaw)
    mujoco.mj_forward(m, d)
    r.ctl.reset()
    r.tilt_target = tilt
    dt = m.opt.timestep
    t0 = d.time
    while d.time - t0 < 2.5:                          # ramp up and settle at v_r
        _drive(r, d, v_r, yaw)
        if d.time - t0 > 0.4 and np.linalg.norm(d.sensor("base_linvel").data[:2] - v_r) < 0.02 and r.tilt_settled():
            break
    # ball aimed at where the rim center will be, T seconds from now
    T = 0.06
    fc = r.funnel_center_world() + np.array([*v_r, 0.0]) * T
    tgt = fc + np.array([off[0], off[1], 0.0])
    gz = np.array([0, 0, 9.81])
    bq, bv = m.joint("ball").qposadr[0], m.joint("ball").dofadr[0]
    d.qpos[bq:bq + 7] = [*(tgt - V_BALL * T - 0.5 * gz * T * T), 1, 0, 0, 0]
    d.qvel[bv:bv + 6] = [*(V_BALL + gz * T), 0, 0, 0]
    mujoco.mj_forward(m, d)            # refresh the sensors, or the entry check sees the ball's old parked spot
    t_in = None
    while d.time - t0 < 6.0:
        now = d.time
        if t_in is None and d.sensor("ball_pos").data[2] < r.funnel_center_world()[2]:
            t_in = now
        if t_in is None or now - t_in < 0.2:
            _drive(r, d, v_r, yaw)
        else:
            _drive(r, d, (0.0, 0.0), yaw, a_cap=brake)
        if t_in is not None and now - t_in > 0.03:
            r.tilt_target = 0.0
        if t_in is not None and now - t_in > 1.0 + sp / brake:
            break
    return bool(r.ball_held())


def entry_cell(args):
    name, speed, brake = args
    v_r = np.array(DIRS[name]) * speed
    held = [entry_trial((v_r, off, brake)) for off in OFFSETS]
    return dict(direction=name, speed=speed, brake=brake, caught=int(sum(held)), n=len(held))


def brake_trial(args):
    """Ball in the pocket, base cruising at 1.5 m/s, tilt t0, then dump + brake at a_b. Held?"""
    name, tilt_deg, a_b = args
    import mujoco
    m, d, r = make_robot(P, settle=0.05)
    heading = 0.0
    u = {"forward": (1.0, 0.0), "backward": (-1.0, 0.0), "sideways": (0.0, 1.0)}[name]
    v = 1.5 * np.array(u)
    start = -v / 1.5 * 1.6 if name != "sideways" else -v / 1.5 * 1.2
    _place_base(d, m, start, heading)
    mujoco.mj_forward(m, d)
    r.ctl.reset()
    r.tilt_target = math.radians(tilt_deg)
    for _ in range(int(0.4 / m.opt.timestep)):
        _drive(r, d, (0.0, 0.0), heading)
    g = r.g
    b = r.module
    hold = d.xpos[b] + d.xmat[b].reshape(3, 3) @ np.array([g.hold[0], 0, g.hold[2] + 0.003])
    bq, bv = m.joint("ball").qposadr[0], m.joint("ball").dofadr[0]
    d.qpos[bq:bq + 7] = [*hold, 1, 0, 0, 0]
    d.qvel[bv:bv + 6] = 0
    for _ in range(int(0.4 / m.opt.timestep)):
        _drive(r, d, (0.0, 0.0), heading)
    if not r.ball_held():
        return dict(direction=name, tilt=tilt_deg, brake=a_b, held=None)
    t0 = d.time
    while d.time - t0 < 3.0:                          # gentle ramp to cruise (the ball is aboard)
        _drive(r, d, v, heading, a_cap=1.0)
        if np.linalg.norm(d.sensor("base_linvel").data[:2] - v) < 0.02:
            break
    r.tilt_target = 0.0
    t1 = d.time
    while d.time - t1 < 1.5:
        _drive(r, d, (0.0, 0.0), heading, a_cap=a_b)
    return dict(direction=name, tilt=tilt_deg, brake=a_b, held=bool(r.ball_held()))


def main():
    out = {}
    with ProcessPoolExecutor(min(26, os.cpu_count() or 4)) as pool:
        cells = [("with the ball", 0.0, 1.5)] + [(n, s, 1.5) for n in DIRS for s in SPEEDS if s > 0]
        out["entry"] = list(pool.map(entry_cell, cells))
        brake_jobs = [(n, t, a) for n in ("forward", "backward", "sideways") for t in (0.0, 15.0)
                      for a in (1.0, 2.0, 3.0, 4.0)]
        out["brake"] = list(pool.map(brake_trial, brake_jobs))
    save("moving_rig", out)
    print("A. Entry while moving (ball 6.4 m/s, 24 deg; 17 landing offsets up to 7 cm; brake 1.5 m/s^2 after):")
    for e in out["entry"]:
        v_r = np.array(DIRS[e["direction"]]) * e["speed"]
        v_rel = V_BALL - np.array([*v_r, 0.0])
        print(f"  {e['direction']:16s} {e['speed']:.1f} m/s: relative arrival {math.degrees(math.atan2(math.hypot(v_rel[0], v_rel[1]), -v_rel[2])):4.0f} deg "
              f"from vertical at {np.linalg.norm(v_rel):.1f} m/s -> caught {e['caught']}/{e['n']}")
    print("B. Braking from 1.5 m/s with the ball in the pocket (dump starts as braking starts):")
    for b in out["brake"]:
        print(f"  {b['direction']:9s} tilt {b['tilt']:4.0f} deg, brake {b['brake']:.0f} m/s^2: "
              f"{'held' if b['held'] else ('LOST' if b['held'] is not None else 'load failed')}")


if __name__ == "__main__":
    main()
