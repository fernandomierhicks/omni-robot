"""Does the feed take every ball size? One bench shot per (feed, ball diameter, launch tilt) at 3000 rpm.

The ball is resized after the model is built (collision radius and bounding radius), so the hardware stays where the
nominal 74 mm ball put it. For each case: does the ball rest in the pocket, which surfaces it rests against, do the
feed wheels and the flywheels ever grip it at the same time (gap < 0), the exit speed, elevation and spin.

  python feed_tolerance.py                 # roller (v6 round 1) and chute (round 2) at their default feed speed
  python feed_tolerance.py chute 1.2 1.6   # the chute at other feed speeds (m/s)
"""

from __future__ import annotations

import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import mujoco
import numpy as np

from common import SimParams, save  # noqa: E402
from launcher_bench import Bench  # noqa: E402

DIAMETERS = (72.0, 73.0, 74.0, 75.0, 76.0)
TILTS = (0.0, -15.0, -30.0)


def shot(args):
    feed, speed, dia, tilt = args
    kw = dict(feed=feed)
    if speed:
        kw["feed_speed"] = speed
    b = Bench(SimParams(**kw))
    m, d, r, g = b.m, b.d, b.r, b.r.g
    gid = m.geom("ball").id
    m.geom_size[gid][0] = m.geom_rbound[gid] = dia / 2000.0
    b.set_tilt(tilt)
    b.load_ball()
    b.run(0.4)
    f6 = np.zeros(6)

    def forces():
        out = {}
        for i in range(d.ncon):
            c = d.contact[i]
            if gid in (c.geom1, c.geom2):
                mujoco.mj_contactForce(m, d, i, f6)
                name = m.geom(c.geom2 if c.geom1 == gid else c.geom1).name
                key = "feed" if name.startswith("feed") else "fly" if name.startswith("flywheel") else name
                out[key] = out.get(key, 0.0) + f6[0]
        return out

    rest = {k: round(v, 2) for k, v in forces().items()}
    held = r.ball_held()
    r.fly_target = 3000 * 2 * math.pi / 60
    t0 = d.time
    while not r.flywheels_ready() and d.time - t0 < 2.0:
        r.step()
    b.run(0.1)
    r.gate_open = True
    t1 = d.time
    t_feed, t_fly, ex = [], [], None
    nip = b.module_point((g.N[0], 0, g.N[1]))
    while d.time - t1 < 0.6:
        r.step()
        fo = forces()
        if fo.get("feed", 0) > 0.05:
            t_feed.append(d.time - t1)
        if fo.get("fly", 0) > 0.05:
            t_fly.append(d.time - t1)
        if ex is None and np.linalg.norm(d.sensor("ball_pos").data - nip) > 0.15 and np.linalg.norm(d.sensor("ball_vel").data) > 3:
            v = d.sensor("ball_vel").data
            ex = (float(np.linalg.norm(v)), math.degrees(math.atan2(v[2], math.hypot(v[0], v[1]))),
                  float(np.linalg.norm(d.sensor("ball_angvel").data)))
    gap = (min(t_fly) - max(t_feed)) if t_feed and t_fly else float("nan")
    return dict(feed=feed, speed=speed or SimParams(feed=feed).feed_speed, dia=dia, tilt=tilt, held=held, rest=rest,
                gap_ms=1000 * gap, exit=ex)


def main(feeds):
    jobs = [(f, s, dia, t) for f, s in feeds for dia in DIAMETERS for t in TILTS]
    with ProcessPoolExecutor(min(len(jobs), os.cpu_count() or 4)) as pool:
        rows = list(pool.map(shot, jobs))
    save("feed_tolerance", rows)
    for f, s in feeds:
        print(f"{f}, feed {s or SimParams(feed=f).feed_speed} m/s: exit speed, elevation, spin at launch tilt 0 / -15 / -30 (gap between the feed-wheel"
              f" and flywheel grips, ms; a negative gap = both grip at once)")
        for dia in DIAMETERS:
            cells = []
            for t in TILTS:
                rw = next(x for x in rows if x["feed"] == f and x["speed"] == (s or SimParams(feed=f).feed_speed)
                          and x["dia"] == dia and x["tilt"] == t)
                cells.append("NO FEED" if rw["exit"] is None else
                             f"{rw['exit'][0]:.2f} m/s {rw['exit'][1]:.1f} deg {rw['exit'][2]:3.0f} rad/s ({rw['gap_ms']:+.0f})")
            rest = next(x for x in rows if x["feed"] == f and x["dia"] == dia and x["tilt"] == 0.0)["rest"]
            print(f"   {dia:.0f} mm: " + " | ".join(f"{c:>12s}" for c in cells) +
                  f"   resting against: {', '.join(k for k, v in rest.items() if v > 0.02) or 'nothing'}")


if __name__ == "__main__":
    a = sys.argv[1:]
    if a:
        main([(a[0], float(x)) for x in a[1:]] or [(a[0], None)])
    else:
        main([("roller", None), ("chute", None)])
