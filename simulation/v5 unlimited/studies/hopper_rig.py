"""Can the v3b catch module work as a multi-ball hopper?

The funnel is loaded with balls; the plunger feeds them to the flywheels one at a
time. Questions:

A. Capacity: drop N balls into the funnel one by one; how many stay aboard, at rest,
   after tilting to the launch extremes, and while driving at full acceleration?
B. Burst fire: with N balls aboard, the flywheels running and the module at a
   launch tilt, fire repeatedly. Each cycle: wait for the break-beam (a ball in the
   pocket) -> plunger out -> the ball leaves -> plunger back. Does every ball feed,
   does the next one drop in front of the plunger (not behind it, which jams the
   return stroke), do two leave at once (double feed), how consistent is the exit
   speed, and how fast can it fire?

Writes results/hopper_rig.json.
  python hopper_rig.py
"""

from __future__ import annotations

import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import SimParams, make_robot, save

N_MAX = 12


def _balls(m, p):
    from robot_model import ball_names
    return [m.body(n).id for n in ball_names(p)], [m.joint(n).qposadr[0] for n in ball_names(p)], \
        [m.joint(n).dofadr[0] for n in ball_names(p)]


def _aboard(d, r, p, bodies):
    """Balls inside the catch module: within the rim radius of the funnel axis, above the payload plate."""
    fc = r.funnel_center_world()
    b = r.module
    axis = d.xmat[b].reshape(3, 3)[:, 2]
    out = []
    for bid in bodies:
        q = d.xpos[bid] - fc
        h = float(q @ axis)                                  # along the funnel axis (0 at the rim)
        rad = float(np.linalg.norm(q - h * axis))
        top = 0.08 + (max(0.0, r.g.throat_z + p.hopper_tube_height - r.g.rim_z) if p.hopper == "tube" else 0.0)
        out.append(bool(-0.40 < h < top and rad < p.funnel_rim_radius + 0.01))
    return out


def load(p, n, seed=0, settle=1.0):
    """Robot at the origin, drop n balls into the funnel one at a time. Returns (m, d, r, bodies, qadr, vadr)."""
    m, d, r = make_robot(p)
    bodies, qadr, vadr = _balls(m, p)
    rng = np.random.default_rng(seed)
    fc = r.funnel_center_world()
    tube = p.hopper == "tube"
    z_drop = (r.g.throat_z + p.hopper_tube_height - r.g.rim_z + 0.06 if tube else 0.12) + fc[2]
    for k in range(n):
        xy = fc[:2] + (rng.uniform(-0.002, 0.002, 2) if tube else rng.uniform(-0.05, 0.05, 2))
        d.qpos[qadr[k]:qadr[k] + 7] = [xy[0], xy[1], z_drop, 1, 0, 0, 0]
        d.qvel[vadr[k]:vadr[k] + 6] = [0, 0, -0.5, 0, 0, 0]
        mujoco.mj_forward(m, d)
        for _ in range(int(0.35 / m.opt.timestep)):
            r.step()
    for _ in range(int(settle / m.opt.timestep)):
        r.step()
    return m, d, r, bodies, qadr, vadr


def capacity(args):
    n, pkw = args if len(args) == 2 else (args[0], {})
    p = SimParams(n_balls=N_MAX, **pkw)
    m, d, r, bodies, qadr, vadr = load(p, n)
    rest = sum(_aboard(d, r, p, bodies))
    out = dict(n=n, at_rest=rest)
    for tilt in (-30.0, 25.0, 0.0):                          # launch extreme, catch extreme, back upright
        r.tilt_target = math.radians(tilt)
        for _ in range(int(0.8 / m.opt.timestep)):
            r.step()
        out[f"after_tilt_{tilt:+.0f}"] = sum(_aboard(d, r, p, bodies))
    # drive: 1 m forward and back at the physics limits, then sideways
    pose0 = r.pose()
    for dx, dy in ((1.0, 0.0), (0.0, 0.0), (0.0, 0.8), (0.0, 0.0)):
        tgt = np.array([pose0[0] + dx, pose0[1] + dy, pose0[2]])
        for _ in range(int(1.6 / m.opt.timestep)):
            r.drive_to(r.pose(), tgt)
            r.step()
    out["after_driving"] = sum(_aboard(d, r, p, bodies))
    out["held_in_pocket"] = bool(r.ball_held())
    out["hopper"] = p.hopper
    out["tube_height"] = p.hopper_tube_height
    return out


SHAKE_TILT = 20.0          # deg: agitation flick of the module (tilt servo) when the pocket stays empty
SHAKE_AFTER = 0.25         # s of empty pocket before a flick
SHAKES = 4                 # flicks before giving up


def burst(args):
    """Fire everything aboard. Returns per-shot exit speed/elevation, feed times, and failure flags.
    agitate: "tilt" flicks the module to SHAKE_TILT and back when the pocket stays empty (balls bridge
    over the throat), "none" doesn't."""
    n, tilt, rpm, kick_speed, agitate, pkw = (*args, "tilt", {})[:6] if len(args) == 4 else \
        ((*args, {}) if len(args) == 5 else args)
    p = SimParams(n_balls=N_MAX, kick_speed=kick_speed, **pkw)
    m, d, r, bodies, qadr, vadr = load(p, n)
    dt = m.opt.timestep
    r.tilt_target = math.radians(tilt)
    for _ in range(int(0.8 / dt)):
        r.step()
    aboard0 = sum(_aboard(d, r, p, bodies))
    r.fly_target = rpm * 2 * math.pi / 60
    t0 = d.time
    while not r.flywheels_ready() and d.time - t0 < 2.0:
        r.step()
    exit_site = r.exit_site
    shots, events = [], []
    exited = set()
    gate_j = m.joint("gate").qposadr[0]
    t_start = d.time
    state, t_state, t_held = "wait", d.time, None
    last_shot = None
    shakes = 0
    yaw_hold = r.pose()[2]
    while d.time - t_start < 1.5 * n + 6.0:
        r.step()
        t = d.time
        # exits: a ball well away from the launcher exit, moving fast
        ex = d.site_xpos[exit_site]
        for k, bid in enumerate(bodies):
            if k in exited:
                continue
            v = d.qvel[vadr[k]:vadr[k] + 3]
            if np.linalg.norm(d.xpos[bid] - ex) > 0.25 and np.linalg.norm(v) > 2.0 and d.xpos[bid][2] > 0.2:
                exited.add(k)
                sp = float(np.linalg.norm(v))
                el = math.degrees(math.atan2(v[2], math.hypot(v[0], v[1])))
                shots.append(dict(t=t - t_start, speed=sp, elev=el, ball=k, double=last_shot is not None and t - last_shot < 0.08))
                last_shot = t
        if agitate == "sweep":
            # the sweeper arm turns whenever the pocket is empty and balls are still aboard
            r.sweep = state == "wait" and not r.ball_held() and t - t_state > 0.1
        if state == "wait":
            held = r.ball_held()
            t_held = (t_held or t) if held else None
            if held and t - t_held > 0.03 and r.flywheels_ready(0.03) and r.tilt_settled():
                r.gate_open = True
                state, t_state = "push", t
                shakes = 0
            elif not held and agitate in ("tilt", "yaw", "both") and t - t_state > SHAKE_AFTER and shakes < SHAKES \
                    and sum(_aboard(d, r, p, bodies)) > 0:
                if agitate in ("tilt", "both"):
                    r.tilt_target = math.radians(SHAKE_TILT)
                state, t_state = "shake", t
                shakes += 1
                events.append(dict(t=t - t_start, what="shake"))
            elif t - t_state > 1.5 or (agitate in ("tilt", "yaw", "both") and shakes >= SHAKES and t - t_state > SHAKE_AFTER):
                events.append(dict(t=t - t_start, what="no ball in the pocket"))
                break
        elif state == "shake":
            ts = t - t_state
            if ts > 0.2 and r.tilt_target > 0:
                r.tilt_target = math.radians(tilt)
            if agitate in ("yaw", "both"):
                # yaw shimmy: +/- 8 rad/s, reversing every 50 ms, for 0.3 s, then hold the heading
                if ts < 0.3:
                    r.drive_velocity((0.0, 0.0), 8.0 * (1 if int(ts / 0.05) % 2 == 0 else -1), r.pose()[2])
                else:
                    r.drive_to(r.pose(), (*r.pose()[:2], yaw_hold))
            if ts > 0.55:
                state, t_state, t_held = "wait", t, None
        elif state == "push":
            if t - t_state > 0.05 + p.kick_stroke / p.kick_speed:
                r.gate_open = False
                state, t_state = "return", t
        elif state == "return":
            # the plunger must get home; a ball that dropped behind it blocks it
            if abs(d.qpos[gate_j]) < 0.002:
                state, t_state, t_held = "wait", t, None
            elif t - t_state > 0.6:
                events.append(dict(t=t - t_start, what=f"plunger jammed at {1000 * d.qpos[gate_j]:.0f} mm"))
                break
    aboard_end = sum(_aboard(d, r, p, bodies))
    sp = [s["speed"] for s in shots]
    gaps = np.diff([s["t"] for s in shots]) if len(shots) > 1 else np.array([])
    return dict(n=n, tilt=tilt, rpm=rpm, kick_speed=kick_speed, agitate=agitate, hopper=p.hopper +
                (f" {p.sweeper_speed:.0f}/s" if p.hopper == "sweeper" else ""), aboard=aboard0, fired=len(shots),
                shakes=sum(e["what"] == "shake" for e in events), elapsed=(shots[-1]["t"] if shots else float("nan")),
                left_aboard=aboard_end, lost=max(0, aboard0 - len(shots) - aboard_end),
                doubles=sum(s["double"] for s in shots), events=events, shots=shots,
                speed_mean=float(np.mean(sp)) if sp else float("nan"),
                speed_spread=float(np.std(sp) / np.mean(sp)) if len(sp) > 1 else float("nan"),
                elev_spread=float(np.std([s["elev"] for s in shots])) if len(sp) > 1 else float("nan"),
                interval=float(np.median(gaps)) if len(gaps) else float("nan"))


def report(out):
    print("A. Capacity: balls dropped -> aboard (at rest, after tilting to -30 / +25 / 0 deg, after driving at the limits)")
    for c in out["capacity"]:
        what = c["hopper"] + (f" {100 * c['tube_height']:.0f} cm" if c["hopper"] == "tube" else "")
        print(f"   {what:12s} {c['n']:2d} dropped: {c['at_rest']:2d} at rest, {c['after_tilt_-30']:2d} / {c['after_tilt_+25']:2d} / "
              f"{c['after_tilt_+0']:2d} after tilts, {c['after_driving']:2d} after driving"
              f"{'' if c['held_in_pocket'] else '  (pocket empty)'}")
    print("B. Burst fire (the plunger cycles as soon as the beam sees a ball):")
    for b in out["burst"]:
        ev = "; ".join(e["what"] for e in b["events"] if e["what"] != "shake") or "-"
        print(f"   {b['aboard']:2d} aboard, tilt {b['tilt']:+4.0f} deg ({30 - b['tilt']:.0f} deg launch), {b['rpm']:.0f} rpm, "
              f"plunger {b['kick_speed']:.2f} m/s, {b.get('hopper', 'none'):7s} {b['agitate']:5s}: fired {b['fired']} in {b['elapsed']:.1f} s "
              f"({b['shakes']} flicks), left {b['left_aboard']}, lost {b['lost']}, doubles {b['doubles']}; exit "
              f"{b['speed_mean']:.2f} m/s +/- {100 * b['speed_spread']:.1f}%, elevation +/- {b['elev_spread']:.1f} deg; "
              f"median interval {b['interval']:.2f} s | end: {ev}")


def main():
    out = {}
    with ProcessPoolExecutor(min(26, os.cpu_count() or 4)) as pool:
        tube = dict(hopper="tube")
        tall = dict(hopper="tube", hopper_tube_height=0.45)
        jc = pool.map(capacity, [(n, kw) for kw in ({}, tube, tall) for n in (2, 4, 6, 8, 10, 12)])
        jobs = [(6, t, 3000, 0.4, ag, kw) for t in (0.0, -15.0, -30.0)
                for ag, kw in (("none", {}), ("tilt", {}), ("tilt", dict(funnel_mu=0.3, funnel_roll=0.002)),
                               ("sweep", dict(hopper="sweeper", sweeper_speed=2.0)),
                               ("sweep", dict(hopper="sweeper", sweeper_speed=6.0)), ("none", tube))]
        jobs += [(8, t, 3000, 0.4, "none", tall) for t in (0.0, -30.0)]
        jobs += [(6, -15.0, rpm, 0.4, "none", tube) for rpm in (2000, 5000)]
        jobs += [(6, -15.0, 3000, ks, "none", tube) for ks in (0.25, 0.8, 1.2)]
        jb = pool.map(burst, jobs)
        out["capacity"] = list(jc)
        out["burst"] = list(jb)
    save("hopper_rig", out)
    report(out)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "try":
        # quick comparison of agitation and funnel-surface options, 6 balls at -15 deg
        cases = [(n, t, 3000, 0.4, ag, kw) for n in (6,) for t in (0.0, -15.0, -30.0)
                 for ag, kw in (("none", dict(hopper="tube")), ("sweep", dict(hopper="sweeper")),
                                ("none", dict(hopper="sweeper")))]
        with ProcessPoolExecutor(len(cases)) as pool:
            for c, b in zip(cases, pool.map(burst, cases)):
                print(c[4], c[5], f"fired {b['fired']}/{b['aboard']} in {b['elapsed']:.1f} s, flicks {b['shakes']}, "
                      f"lost {b['lost']}, doubles {b['doubles']}, interval {b['interval']:.2f}")
    else:
        main()
