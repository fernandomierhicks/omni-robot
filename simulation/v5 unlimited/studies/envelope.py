"""What the v3b robot can actually do with every firmware limit removed (v5 "physics" limits).

Only physics is left: motor torque and back-EMF at the live battery voltage, the
driver's peak current, roller traction (mu 0.95), tip-over, the BMS trip. Parts:

A. Slip-free acceleration in 24 body directions, measured properly: a jerk-limited
   ramp to 2.5 m/s at 400 m/s^3 (v3's test ramped to 1 m/s at 40 m/s^3, which can't
   reach more than sqrt(40 x 1) = 6.3 m/s^2, so its "9.9 forward" was the test's
   ceiling, not the robot's). Sets ACCEL_TABLE_PHYS.
B. Top speed by direction at 100% and 20% charge (motor back-EMF; the X drive is
   faster along its sides than along a wheel axle).
C. Point-to-point moves (0.3-2.5 m, 4 directions): time to settle within 1 cm for
   the firmware profile and for physics-profile variants of jerk and stop-planning
   margin. Picks the physics defaults.
D. Yaw: turn 180 deg and settle, for yaw-acceleration limits 15-150 rad/s^2.
E. Hard stop from top speed: the physics profile's own ramped stop, and a naive
   all-wheels-zero stop, at mu 0.95 and 1.2. Tilt and stopping distance.
F. Braking with a ball in the pocket from 2.5 m/s at 4-8 m/s^2 (v3 tested 1-4).

Writes results/envelope.json and envelope.png. ~4 min on 28 threads.
"""

from __future__ import annotations

import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from common import RESULTS, SimParams, accel_run, body_velocity, make_robot, save, tilt_deg  # noqa: E402
from models import Battery  # noqa: E402
from omni_control import wrap_angle  # noqa: E402

BIG = (40.0, 40.0)                         # an arena big enough for top-speed runs
DIRS = list(range(0, 360, 15))
TILT_TOL = 5.0                             # the free rocker rolls ~3 deg by design


def phys(**kw) -> SimParams:
    return SimParams(area_size=BIG, **kw)


# --- A ----------------------------------------------------------------------------
def clean_accel(args):
    """Bisection for the highest slip-free, tip-free acceleration in one body direction."""
    direction, = args
    p = phys()
    lo, hi, why = 1.0, 12.0, "none"
    worst = {}
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        r = accel_run(p, direction, mid, v_final=2.0, jerk=400.0)
        ok = r["err"] < 0.05 and r["e_track"] < 0.05 and r["drift"] < 2.0 and r["tilt"] < TILT_TOL
        if ok:
            lo, worst = mid, r
        else:
            hi = mid
            why = "tip" if r["tilt"] >= TILT_TOL else "slip"
    return dict(direction=direction, accel=lo, fail=why, i_peak=worst.get("i_peak", float("nan")),
                tilt=worst.get("tilt", float("nan")), n_min=worst.get("n_min", float("nan")))


# --- B ----------------------------------------------------------------------------
def top_speed(args):
    direction, soc = args
    p = phys()
    m, d, r = make_robot(p)
    r.battery = Battery(p.pack, soc=soc)
    yaw = r.pose()[2]
    u = np.array([math.cos(math.radians(direction)), math.sin(math.radians(direction))])
    best, t = 0.0, 0.0
    dt = m.opt.timestep
    while t < 3.0:
        r.drive_velocity(20.0 * u, 0.0, yaw)          # ask for far more than it can do
        r.step()
        t += dt
        best = max(best, float(np.linalg.norm(body_velocity(d, r.pose()[2]))))
    return dict(direction=direction, soc=soc, v_top=best, v_bus=r.battery.v, i_peak=r.battery.i_peak)


# --- C ----------------------------------------------------------------------------
VARIANTS = {
    "firmware (v3b)": dict(limits="firmware"),
    "physics, jerk 40": dict(base_max_jerk=40.0),
    "physics, jerk 150": dict(base_max_jerk=150.0),
    "physics, jerk 400": dict(base_max_jerk=400.0),
    "physics, no jerk limit": dict(base_max_jerk=0.0),
    "physics, jerk 150, stop margin 0.8": dict(base_max_jerk=150.0, brake_margin=0.8),
    "physics, jerk 150, stop margin 1.0": dict(base_max_jerk=150.0, brake_margin=1.0),
}
MOVE_DIST = (0.3, 0.6, 1.0, 1.5, 2.5)
MOVE_DIRS = (0, 45, 90, 135)


def move(args):
    """Rest to rest along a world direction (robot facing +x), no yaw. Time to stay within 1 cm."""
    name, dist, direction = args
    p = phys(**VARIANTS[name])
    m, d, r = make_robot(p)
    u = np.array([math.cos(math.radians(direction)), math.sin(math.radians(direction))])
    start = r.pose()
    target = np.array([*(start[:2] + dist * u), start[2]])
    dt = m.opt.timestep
    t, t_in, over, v_peak, i_peak, slip = 0.0, None, 0.0, 0.0, 0.0, 0.0
    while t < 6.0:
        pose = r.pose()
        r.drive_to(pose, target)
        r.step()
        t += dt
        e = float(np.linalg.norm(pose[:2] - target[:2]))
        along = float((pose[:2] - start[:2]) @ u)
        over = max(over, along - dist)
        v = d.sensor("base_linvel").data[:2]
        v_peak = max(v_peak, float(np.linalg.norm(v)))
        slip = max(slip, float(np.linalg.norm(v - r.ctl.v_cmd)))
        i_peak = max(i_peak, r.i_total)
        if e < 0.01:
            t_in = t_in if t_in is not None else t
        else:
            t_in = None
        if t_in is not None and t - t_in > 0.3:
            break
    return dict(variant=name, dist=dist, direction=direction, t_settle=t_in if t_in is not None else float("nan"),
                overshoot=over, v_peak=v_peak, i_peak=i_peak, track_err=slip)


# --- D ----------------------------------------------------------------------------
def turn(args):
    yaw_accel, = args
    p = phys(base_max_yaw_accel=yaw_accel)
    m, d, r = make_robot(p)
    start = r.pose()
    target = np.array([start[0], start[1], wrap_angle(start[2] + math.pi)])
    dt = m.opt.timestep
    t, t_in, w_peak, drift = 0.0, None, 0.0, 0.0
    while t < 4.0:
        pose = r.pose()
        r.drive_to(pose, target)
        r.step()
        t += dt
        w_peak = max(w_peak, abs(float(d.sensor("base_angvel").data[2])))
        drift = max(drift, float(np.linalg.norm(pose[:2] - start[:2])))
        if abs(wrap_angle(pose[2] - target[2])) < math.radians(1.0):
            t_in = t_in if t_in is not None else t
        else:
            t_in = None
        if t_in is not None and t - t_in > 0.3:
            break
    return dict(yaw_accel=yaw_accel, t_settle=t_in if t_in is not None else float("nan"), w_peak=w_peak, drift=drift)


# --- E ----------------------------------------------------------------------------
def hard_stop(args):
    """Cruise at top speed in a body direction, then stop: 'ramped' = the controller's own stop at the
    traction-limit table, 'naive' = every wheel commanded to zero at once."""
    direction, mu, kind = args
    p = phys(wheel_mu=mu, accel_table=SimParams().accel_table)
    m, d, r = make_robot(p)
    u = np.array([math.cos(math.radians(direction)), math.sin(math.radians(direction))])
    yaw = r.pose()[2]
    dt = m.opt.timestep
    t = 0.0
    while t < 2.5:
        r.drive_velocity(20.0 * u, 0.0, yaw)
        r.step()
        t += dt
    v0 = float(np.linalg.norm(d.sensor("base_linvel").data[:2]))
    p0 = r.pose()[:2].copy()
    tilt, n_min, t = 0.0, float("inf"), 0.0
    while t < 2.0:
        if kind == "naive":
            r.a_body_cmd = None
            r.wheel_cmd = np.zeros(r.nw)
        else:
            r.drive_velocity((0.0, 0.0), 0.0, yaw)
        r.step()
        t += dt
        tilt = max(tilt, tilt_deg(d))
        n_min = min(n_min, float(np.min(r.wheel_loads())))
    dist = float(np.linalg.norm(r.pose()[:2] - p0))
    return dict(direction=direction, mu=mu, kind=kind, v0=v0, stop_dist=dist, mean_decel=v0 * v0 / (2 * dist),
                peak_tilt=tilt, min_load=n_min, fell=tilt > 30)


# --- F ----------------------------------------------------------------------------
def brake_ball(args):
    import moving_rig
    name, tilt_deg_, a_b = args
    moving_rig.P = phys()
    return moving_rig.brake_trial_v((name, tilt_deg_, a_b, 2.5))


def main():
    out = {}
    with ProcessPoolExecutor(min(26, os.cpu_count() or 4)) as pool:
        ja = pool.map(clean_accel, [(a,) for a in DIRS])
        jb = pool.map(top_speed, [(a, s) for a in range(0, 360, 15) for s in (1.0, 0.2)])
        jc = pool.map(move, [(n, dd, a) for n in VARIANTS for dd in MOVE_DIST for a in MOVE_DIRS])
        jd = pool.map(turn, [(a,) for a in (15.0, 30.0, 60.0, 100.0, 150.0)])
        je = pool.map(hard_stop, [(a, mu, k) for a in (0, 45, 90, 180) for mu in (0.95, 1.2) for k in ("ramped", "naive")])
        jf = pool.map(brake_ball, [(n, t, a) for n in ("forward", "backward", "sideways") for t in (0.0, 15.0)
                                   for a in (4.0, 5.0, 6.0, 8.0)])
        out["accel"] = list(ja)
        out["top_speed"] = list(jb)
        out["moves"] = list(jc)
        out["turn"] = list(jd)
        out["hard_stop"] = list(je)
        out["brake_ball"] = list(jf)
    save("envelope", out)
    report(out)
    plot(out)


def mirrored_table(acc):
    """Worse of each left/right pair, 0..180 deg in 30 deg steps (the controller mirrors it)."""
    by = {a["direction"]: a["accel"] for a in acc}
    dirs = list(range(0, 181, 30))
    return dirs, [round(min(by[a % 360], by[(-a) % 360]), 2) for a in dirs]


def report(out):
    print("A. Slip-free acceleration (m/s^2) by body direction, 0 = forward (launcher side):")
    for a in out["accel"]:
        print(f"   {a['direction']:4d} deg: {a['accel']:5.2f}  (above it: {a['fail']}; bus current {a['i_peak']:.1f} A, "
              f"tilt {a['tilt']:.1f} deg, lightest wheel {a['n_min']:.1f} N)")
    dirs, vals = mirrored_table(out["accel"])
    print(f"   -> ACCEL_TABLE_PHYS = ({tuple(float(x) for x in dirs)}, {tuple(vals)})")
    print("B. Top speed (m/s):")
    for soc in (1.0, 0.2):
        rows = [b for b in out["top_speed"] if b["soc"] == soc]
        print(f"   {100 * soc:3.0f}% charge: " + ", ".join(f"{b['direction']}: {b['v_top']:.2f}" for b in rows[:7])
              + f"  (min {min(b['v_top'] for b in rows):.2f}, max {max(b['v_top'] for b in rows):.2f})")
    print("C. Point-to-point, time to settle within 1 cm (s), mean over directions [worst overshoot cm, peak speed m/s, "
          "peak bus A]:")
    for name in VARIANTS:
        rows = [c for c in out["moves"] if c["variant"] == name]
        cells = []
        for dd in MOVE_DIST:
            rr = [c for c in rows if c["dist"] == dd]
            cells.append(f"{dd:.1f} m {np.nanmean([c['t_settle'] for c in rr]):.2f}")
        print(f"   {name:36s} " + " | ".join(cells) +
              f"  [{100 * max(c['overshoot'] for c in rows):.1f} cm, {max(c['v_peak'] for c in rows):.2f} m/s, "
              f"{max(c['i_peak'] for c in rows):.1f} A]")
    print("D. Turn 180 deg and settle within 1 deg:")
    for t in out["turn"]:
        print(f"   yaw accel {t['yaw_accel']:5.0f} rad/s^2: {t['t_settle']:.2f} s, peak {t['w_peak']:.1f} rad/s, "
              f"drift {100 * t['drift']:.1f} cm")
    print("E. Stop from top speed:")
    for s in out["hard_stop"]:
        print(f"   {s['kind']:6s} mu {s['mu']:.2f}, {s['direction']:3d} deg from {s['v0']:.2f} m/s: {s['stop_dist']:.2f} m "
              f"(mean {s['mean_decel']:.1f} m/s^2), peak tilt {s['peak_tilt']:.1f} deg, lightest wheel {s['min_load']:.1f} N"
              + ("  FELL" if s["fell"] else ""))
    print("F. Braking from 2.5 m/s with the ball in the pocket:")
    for b in out["brake_ball"]:
        print(f"   {b['direction']:9s} tilt {b['tilt']:3.0f} deg, {b['brake']:.0f} m/s^2 requested "
              f"(achieved {b.get('achieved', float('nan')):.1f}): "
              + ("held" if b["held"] else ("LOST" if b["held"] is not None else "load failed")))


def plot(out):
    fig = plt.figure(figsize=(14, 4.6))
    ax = fig.add_subplot(1, 3, 1, projection="polar")
    th = np.radians([a["direction"] for a in out["accel"]] + [0])
    ax.plot(th, [a["accel"] for a in out["accel"]] + [out["accel"][0]["accel"]], "o-", color="#2a6fdb", ms=3,
            label="slip-free (physics)")
    from params import ACCEL_TABLE_V3B
    dv, vv = ACCEL_TABLE_V3B
    full = list(range(0, 360, 5))
    fw = [float(np.interp(min(a, 360 - a), dv, vv)) for a in full]
    ax.plot(np.radians(full + [0]), fw + fw[:1], "--", color="#e8743b", label="v3b firmware table")
    ax.set_title("Acceleration limit (m/sÂ²), 0Â° = forward", fontsize=9)
    ax.legend(loc="lower left", fontsize=7, bbox_to_anchor=(-0.2, -0.2))
    ax2 = fig.add_subplot(1, 3, 2, projection="polar")
    for soc, col in ((1.0, "#19a979"), (0.2, "#8e44ad")):
        rows = [b for b in out["top_speed"] if b["soc"] == soc]
        ax2.plot(np.radians([b["direction"] for b in rows] + [0]), [b["v_top"] for b in rows] + [rows[0]["v_top"]],
                 "o-", ms=3, color=col, label=f"{100 * soc:.0f}% charge")
    ax2.set_title("Top speed (m/s)", fontsize=9)
    ax2.legend(loc="lower left", fontsize=7, bbox_to_anchor=(-0.2, -0.2))
    ax3 = fig.add_subplot(1, 3, 3)
    for name in VARIANTS:
        rows = [c for c in out["moves"] if c["variant"] == name]
        ax3.plot(MOVE_DIST, [np.nanmean([c["t_settle"] for c in rows if c["dist"] == dd]) for dd in MOVE_DIST],
                 "o-", ms=3, label=name)
    ax3.set_xlabel("move distance (m)")
    ax3.set_ylabel("time to settle within 1 cm (s)")
    ax3.grid(alpha=0.3)
    ax3.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(RESULTS / "envelope.png", dpi=130)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "report":
        import json
        o = json.loads((RESULTS / "envelope.json").read_text())
        report(o)
        plot(o)
    else:
        main()
