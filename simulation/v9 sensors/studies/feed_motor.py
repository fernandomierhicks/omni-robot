"""Which real motor can drive the chute's feed wheels? (v6 round 3; design.md 23.1)

v6 rounds 1-2 drove the feed wheels with an ideal velocity loop (0.3 N m, holding like a worm). The chute needs
1.6 m/s at the tire, ~764 rpm on the 40 mm wheels, which the assumed N20 worm (380 rpm) can't reach. Here each
catalog.FEED_MOTORS candidate is modeled from its datasheet (models.FeedDrive: back-EMF, resistance, driver current
limit, gearbox drag, reflected rotor inertia; a speed loop when feeding, a position hold with a current limit when
stopped, i.e. no worm needed) and put through the three jobs the feed wheels have:

  A. feed: one bench shot at 3000 rpm per (ball 72 / 74 / 76 mm, launch tilt 0 / -15 / -30 deg): does it feed, how
     fast does the ball leave the feed wheels (it must coast up the J), exit speed and elevation, peak current;
  B. hold against catches: the catch rig's fast, flat arrivals (6.5 and 8 m/s, 30-60 deg from vertical, 13 offsets)
     on the stopped wheels: caught, how far the wheels were back-driven, and leaks (a caught ball pushed past the
     feed wheels' exit beam without a trigger: it would reach the flywheels);
  C. a 6-ball magazine burst at 30 and 60 deg launch: every ball fired, no double feeds, holding the column.

  python feed_motor.py                  # every candidate
  python feed_motor.py GM2804 ideal     # only these
Writes results/feed_motor.json.
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
from catalog import FEED_MOTORS  # noqa: E402
from launcher_bench import Bench  # noqa: E402
import catch_rig  # noqa: E402
import hopper_rig  # noqa: E402

DIAMETERS = (72.0, 74.0, 76.0)
TILTS = (0.0, -15.0, -30.0)
MOTORS = ("ideal", "N20-worm-380", "Pololu-HPCB-30", "Pololu-25D-9.7", "Pololu-25D-RoboClaw", "GM2804", "GM3506", "LK-MF4005")


def bench_shot(args):
    motor, dia, tilt = args
    b = Bench(SimParams(feed_motor=motor))
    m, d, r, g = b.m, b.d, b.r, b.r.g
    gid = m.geom("ball").id
    m.geom_size[gid][0] = m.geom_rbound[gid] = dia / 2000.0
    b.set_tilt(tilt)
    held = b.load_ball()
    r.fly_target = 3000 * 2 * math.pi / 60
    t0 = d.time
    while not r.flywheels_ready() and d.time - t0 < 2.0:
        r.step()
    b.run(0.1)
    for f in r.feed_drives:
        f.i_peak = 0.0
    r.gate_open = True
    t1 = d.time
    nip = b.module_point((g.N[0], 0, g.N[1]))
    ex, v_feed, t_feed, blocked = None, None, None, False
    jv = [m.jnt_dofadr[m.joint(f"feed_{s}").id] for s in "LR"]
    w_max = 0.0
    while d.time - t1 < 0.6:
        r.step()
        w_max = max(w_max, float(np.mean([d.qvel[j] for j in jv])))
        if r.feed_exit_blocked():
            blocked = True
        elif blocked and v_feed is None:          # the ball has just left the feed wheels' exit beam
            v_feed = float(np.linalg.norm(d.sensor("ball_vel").data - d.sensor("base_linvel").data))
            t_feed = d.time - t1
        if ex is None and np.linalg.norm(d.sensor("ball_pos").data - nip) > 0.15 and \
                np.linalg.norm(d.sensor("ball_vel").data) > 3:
            v = d.sensor("ball_vel").data
            ex = (float(np.linalg.norm(v)), math.degrees(math.atan2(v[2], math.hypot(v[0], v[1]))), d.time - t1)
    rim_w = SimParams().feed_speed / (SimParams().feed_wheel_diameter / 2)
    return dict(motor=motor, dia=dia, tilt=tilt, held=held, exit=ex, v_feed=v_feed, t_feed=t_feed,
                w_max=w_max, w_target=rim_w, i_peak=max(f.i_peak for f in r.feed_drives))


def nip_progress(b) -> float:
    """How far the ball has gone from its resting spot toward and through the feed wheels, as a fraction of the
    distance from rest to the wheels' axis line (1 = its centre is between the axles: it is being fed)."""
    r, d, g = b.r, b.d, b.r.g
    mb = r.module
    q = d.xmat[mb].reshape(3, 3).T @ (d.sensor("ball_pos").data - d.xpos[mb])
    c = np.array([math.cos(g.chute_angle), math.sin(g.chute_angle)])
    s_rest = float(np.dot(np.array([g.E[0] - g.C[0], g.E[1] - g.C[1]]), c))
    return float(np.dot(np.array([q[0] - g.C[0], q[2] - g.C[1]]), c)) / s_rest


def hold_rig(motor):
    """catch_rig trials on the v6 chute module; per trial: caught, wheel back-drive (rad), and whether the ball was
    pushed into the nip (a leak: its centre between the feed wheels' axles, so the flywheels could take it)."""
    b = Bench(SimParams(feed_motor=motor))
    r, d = b.r, b.d
    out = []
    for v in catch_rig.SPEEDS:
        for th in catch_rig.ANGLES:
            for off in catch_rig.OFFSETS:
                for f in r.feed_drives:
                    f.slip_max, f.i_peak = 0.0, 0.0
                prog = [-9.0]
                step0 = r.step

                def step():
                    step0()
                    if r.ball_held():
                        prog[0] = max(prog[0], nip_progress(b))
                r.step = step
                held = catch_rig.trial(b, v, th, 0.6, 35.0, off)
                r.step = step0
                out.append(dict(v=v, theta=th, held=bool(held), leak=prog[0] >= 1.0, progress=prog[0],
                                slip=max(f.slip_max for f in r.feed_drives),
                                i_peak=max(f.i_peak for f in r.feed_drives)))
                # the trial parks the ball and closes the gate: re-arm the hold at the new angle
                for f in r.feed_drives:
                    f.q_hold = None
    return dict(motor=motor, trials=out)


def burst(args):
    motor, tilt = args
    res = hopper_rig.burst((6, tilt, 3000, 0.4, "none", dict(hopper="tube", feed_motor=motor)))
    res["motor"] = motor
    return res


def main(motors):
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        jb = pool.map(bench_shot, [(mo, dia, t) for mo in motors for dia in DIAMETERS for t in TILTS])
        jh = pool.map(hold_rig, motors)
        jr = pool.map(burst, [(mo, t) for mo in motors for t in (0.0, -30.0)])
        bench, hold, bursts = list(jb), list(jh), list(jr)
    path = RESULTS / "feed_motor.json"
    if path.exists():                       # keep earlier runs of other motors
        old = json.loads(path.read_text())
        bench = [x for x in old["bench"] if x["motor"] not in motors] + bench
        hold = [x for x in old["hold"] if x["motor"] not in motors] + hold
        bursts = [x for x in old["burst"] if x["motor"] not in motors] + bursts
    save("feed_motor", dict(bench=bench, hold=hold, burst=bursts))
    report([m for m in MOTORS if any(x["motor"] == m for x in bench)], bench, hold, bursts)


def report(motors, bench, hold, bursts):
    print("A. Feed: ball speed leaving the feed wheels (m/s) | exit at 3000 rpm (m/s, deg) per ball 72/74/76 mm; "
          "launch 30/45/60 deg")
    for mo in motors:
        fm = FEED_MOTORS[mo]
        rows = [x for x in bench if x["motor"] == mo]
        fed = sum(x["exit"] is not None for x in rows)
        vf = [x["v_feed"] for x in rows if x["v_feed"] is not None]
        ve = [x["exit"][0] for x in rows if x["exit"] is not None]
        el = [x["exit"][1] - (30 - x["tilt"]) for x in rows if x["exit"] is not None]
        wr = np.mean([x["w_max"] / x["w_target"] for x in rows])
        print(f"   {mo:16s} fed {fed}/{len(rows)}; leaves the feed wheels at {min(vf, default=0):.2f}-{max(vf, default=0):.2f} m/s; "
              f"wheel reaches {100 * wr:.0f}% of 764 rpm; exit {min(ve, default=0):.2f}-{max(ve, default=0):.2f} m/s, "
              f"elevation {min(el, default=0):+.1f}..{max(el, default=0):+.1f} deg off geometric; "
              f"peak current {max(x['i_peak'] for x in rows):.2f} A; at 764 rpm it has {fm.torque_at(80.0):.3f} N m")
        for x in rows:
            if x["exit"] is None:
                print(f"      NO FEED: {x['dia']:.0f} mm at tilt {x['tilt']:+.0f}")
    print("B. Holding against fast, flat catches (stopped feed wheels; 104 rig trials):")
    for h in hold:
        t = h["trials"]
        s = np.degrees([x["slip"] for x in t])
        pg = [x["progress"] for x in t]
        print(f"   {h['motor']:16s} caught {sum(x['held'] for x in t)}/{len(t)}, pushed into the nip "
              f"{sum(x['leak'] for x in t)} (deepest {100 * max(pg):.0f}% of the way to the axles); "
              f"back-driven median {np.median(s):.1f} deg, max {s.max():.1f} deg "
              f"({1000 * math.radians(s.max()) * 0.02:.1f} mm at the tire); peak current {max(x['i_peak'] for x in t):.2f} A")
    print("C. Magazine burst, 6 balls, 3000 rpm (30 / 60 deg launch): fired, doubles, speed spread, interval")
    for mo in motors:
        cells = [f"{b['fired']}/{b['aboard']} doubles {b['doubles']}, +/-{100 * b['speed_spread']:.1f}%, {b['interval']:.2f} s "
                 f"apart (peak {b['feed_i_peak']:.2f} A)"
                 for b in bursts if b["motor"] == mo]
        print(f"   {mo:16s} " + " | ".join(cells))


if __name__ == "__main__":
    main(sys.argv[1:] or list(MOTORS))
