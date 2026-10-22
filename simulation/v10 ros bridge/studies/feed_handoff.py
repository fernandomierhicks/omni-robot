"""Feed-wheel to flywheel handoff: along the ball's path, where is it held by the feed wheels, where by the flywheels,
and is there a stretch where both grip it (a ball pulled at 0.8 m/s by one pair and ripped at 11 m/s by the other)?

One shot per configuration on the bench. For every physics step the ball's travel s from its resting place along the launch
line is logged, with the normal force of each contact (feed wheels, flywheels, pocket floor). Reported: the travel where
each starts and stops pushing (> 0.05 N), the unpowered gap between them (negative = overlap), and peak forces.

The ball's real diameter is varied on the same hardware (the geom is resized after the model is built; the feed-wheel
positions, sprung mounts and flywheel nip stay where the 74 mm ball put them).
  python feed_handoff.py
"""

from __future__ import annotations

import math

import numpy as np

from common import SimParams  # noqa: E402
from launcher_bench import Bench  # noqa: E402

THRESH = 0.05      # N


def handoff(diameter_mm=74.0, rpm=3000, tilt=-15.0, **pkw):
    b = Bench(SimParams(**pkw))
    m, d, r, g = b.m, b.d, b.r, b.r.g
    gid = m.geom("ball").id
    m.geom_size[gid][0] = diameter_mm / 2000.0
    b.set_tilt(tilt)
    b.load_ball()
    r.fly_target = rpm * 2 * math.pi / 60
    t0 = d.time
    while not r.flywheels_ready() and d.time - t0 < 2.0:
        r.step()
    b.run(0.1)
    names = {m.geom(n).id: n for n in ("feed_L", "feed_R", "flywheel_L", "flywheel_R", "pocket_floor", "back_stop")}
    u = np.array(g.u)
    hold = np.array([g.hold[0], g.hold[2]])
    span = {k: [None, None] for k in ("feed", "fly", "floor")}
    peak = dict(feed=0.0, fly=0.0, floor=0.0)
    speed_at = {}
    r.gate_open = True
    t_open = d.time
    f6 = np.zeros(6)
    s_last = 0.0
    while d.time - t_open < 0.6:
        r.step()
        R_ = d.xmat[r.module].reshape(3, 3)
        q = R_.T @ (d.sensor("ball_pos").data - d.xpos[r.module])
        s = float((np.array([q[0], q[2]]) - hold) @ u) * 1000       # mm of travel along the launch line
        fn = dict(feed=0.0, fly=0.0, floor=0.0)
        for i in range(d.ncon):
            c = d.contact[i]
            for a, o in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a == gid and o in names:
                    mujoco_force(m, d, i, f6)
                    key = {"feed_L": "feed", "feed_R": "feed", "flywheel_L": "fly", "flywheel_R": "fly",
                           "pocket_floor": "floor"}.get(names[o])
                    if key:
                        fn[key] += f6[0]
        for k, v in fn.items():
            peak[k] = max(peak[k], v)
            if v > THRESH:
                span[k][0] = s if span[k][0] is None else span[k][0]
                span[k][1] = s
        speed_at[round(s)] = float(np.linalg.norm(d.sensor("ball_vel").data))
        s_last = s
        if s > 150:
            break
    exit_v = float(np.linalg.norm(d.sensor("ball_vel").data))
    gap = None if span["feed"][1] is None or span["fly"][0] is None else span["fly"][0] - span["feed"][1]
    return dict(diameter=diameter_mm, span=span, gap=gap, peak=peak, exit=exit_v,
                s_nip=float((np.array([g.N[0], g.N[1]]) - hold) @ u) * 1000)


def mujoco_force(m, d, i, out):
    import mujoco
    mujoco.mj_contactForce(m, d, i, out)


def main():
    print("travel = ball-centre distance from its resting spot along the launch line (mm); forces in N")
    for dia in (72.0, 74.0, 76.0):
        for tilt in (0.0, -30.0):
            h = handoff(dia, tilt=tilt)
            sp = h["span"]
            fmt = lambda a: "-" if a[0] is None else f"{a[0]:.0f}..{a[1]:.0f}"        # noqa: E731
            gap = "n/a" if h["gap"] is None else f"{h['gap']:+.1f} mm"
            print(f"ball {dia:.0f} mm, tilt {tilt:+.0f}: feed wheels push {fmt(sp['feed']):>9s} (peak {h['peak']['feed']:.1f} N) | "
                  f"flywheels {fmt(sp['fly']):>9s} (peak {h['peak']['fly']:.1f} N) | unpowered gap {gap} | "
                  f"nip at {h['s_nip']:.0f} mm | exit {h['exit']:.2f} m/s")


if __name__ == "__main__":
    main()
