"""Three wheels vs four on the compact base: traction-limited acceleration and tip-over.

1. Slip-free acceleration by direction, sim vs the quasi-static model, for
   kiwi3 (compact), kiwi3 at v2's radius, x4 with plain speed loops, and x4 with
   load-aware torque allocation.
2. Worst direction vs roller friction mu.
3. Panic stop: cruising at 1.4 m/s, every wheel commanded to zero speed at once
   (no ramp: the traction-limited deceleration a fault or naive stop gives).
   Does the base tip, and how far does a wheel lift?
4. Four wheels on an uneven floor: one wheel 0-2 mm proud, rigid chassis vs a
   front rocker. Static load sharing and worst-direction acceleration.

Writes results/wheel_count.json and .png. Uses a process pool (~2 min on 28 threads).
"""

from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from common import (RESULTS, SimParams, accel_run, make_robot, max_clean_accel, save, tilt_deg,  # noqa: E402
                    traction_base, body_velocity)

DIRS = list(range(0, 360, 30))
_HIGH = dict(deck_mode="high", feed="j_bend")          # round 1 (v3a): deck above the wheels, J-bend module
CONFIGS = {
    "kiwi3 r180 (v2 size), high deck": SimParams(layout="kiwi3", wheel_base_radius=0.18, deck_radius=0.225, **_HIGH),
    "kiwi3 r150, high deck": SimParams(layout="kiwi3", **_HIGH),
    "x4 r150 rocker, high deck (v3a)": SimParams(layout="x4", rocker_deg=3.0, **_HIGH),
    "kiwi3 r150, axle deck": SimParams(layout="kiwi3"),
    "x4 r150 rigid, axle deck": SimParams(layout="x4", rocker_deg=0.0),
    "x4 r150 rocker, axle deck (v3b)": SimParams(layout="x4", rocker_deg=3.0),
}
MUS = [0.6, 0.95, 1.2]


def _model(p):
    """Quasi-static model of this configuration (front rocker = wheels 1 and 4 of the X)."""
    return traction_base(p, rocker=(0, 3) if p.rocker_deg > 0 and p.n_wheels == 4 else None)


def _tilt_tol(p) -> float:
    """A free rocker lets the body roll ~3 deg by design; don't count that as tipping."""
    return 5.0 if p.rocker_deg > 0 and p.n_wheels == 4 else 2.0


def _clean(args):
    p, direction = args
    return max_clean_accel(p, direction, tilt_tol=_tilt_tol(p))


def panic_stop(args):
    """Cruise at v0 along a body direction, then command zero wheel speed on every wheel."""
    p, direction, v0 = args
    m, d, r = make_robot(p)
    u = np.array([math.cos(math.radians(direction)), math.sin(math.radians(direction))])
    dt = m.opt.timestep
    v = a = 0.0
    t = 0.0
    while t < 1.2:                                       # jerk-limited ramp up, then cruise
        a = min(2.5, a + 40 * dt) if v < v0 - a * a / 80 else max(0.0, a - 40 * dt)
        v = min(v0, v + a * dt)
        r.a_body_cmd = a * u
        r.wheel_cmd = r.kin.wheel_speeds([*(v * u), 0.0])
        r.step()
        t += dt
    tilt = 0.0
    n_min = float("inf")
    yaw = r.pose()[2]
    u_w = np.array([math.cos(yaw) * u[0] - math.sin(yaw) * u[1], math.sin(yaw) * u[0] + math.cos(yaw) * u[1]])
    v_cg = [float(d.sensor("base_cg_vel").data[:2] @ u_w)]
    r.a_body_cmd = None
    r.wheel_cmd = np.zeros(r.nw)
    t_stop = None
    for k in range(int(0.8 / dt)):
        r.step()
        tilt = max(tilt, tilt_deg(d))
        n_min = min(n_min, float(np.min(r.wheel_loads())))
        v_cg.append(float(d.sensor("base_cg_vel").data[:2] @ u_w))
        if t_stop is None and v_cg[-1] < 0.02:
            t_stop = (k + 1) * dt
    w = int(0.02 / dt)                                   # CG deceleration over 20 ms windows
    v_cg = np.array(v_cg)
    decel = float(np.max((v_cg[:-w] - v_cg[w:]) / (w * dt)))
    return dict(direction=direction, peak_tilt=tilt, min_load=n_min, peak_decel=decel,
                mean_decel=v0 / t_stop if t_stop else float("nan"), fell=tilt > 30)


def uneven(args):
    """Static loads and worst-direction (x4: 45 deg family) acceleration with wheel 1 proud by delta."""
    p, delta = args
    p = p.with_(wheel_z_offset=(delta, 0.0, 0.0, 0.0))
    m, d, r = make_robot(p, settle=0.6)
    loads = r.wheel_loads().tolist()
    worst = {a: max_clean_accel(p, a, iters=6, tilt_tol=_tilt_tol(p))[0] for a in (45, 135, 225, 315)}
    return dict(delta=delta, rocker=p.rocker_deg, loads=loads, accel=worst)


def main():
    out = {"directions": DIRS, "configs": {}}
    workers = min(24, os.cpu_count() or 4)
    with ProcessPoolExecutor(workers) as pool:
        # 1. polar, sim + analytic
        jobs = {name: pool.map(_clean, [(p, a) for a in DIRS]) for name, p in CONFIGS.items()}
        for name, p in CONFIGS.items():
            alloc = "optimal" if p.allocation == "load_aware" else "pinv"
            b = _model(p)
            sim = list(jobs[name])
            ana = b.polar(DIRS, alloc)
            fine = b.polar(list(range(0, 360, 5)), alloc)
            out["configs"][name] = dict(
                layout=p.layout, L=p.wheel_base_radius, allocation=p.allocation, cg_h=b.h, cg_xy=(b.cx, b.cy),
                sim=[s[0] for s in sim], sim_fail=[s[1] for s in sim],
                analytic_slip=[a["slip"] for a in ana], analytic_tip=[a["tip"] for a in ana],
                analytic_lift=[a["lift"] for a in ana], limited_by=[a["limited_by"] for a in ana],
                fine_dirs=list(range(0, 360, 5)), fine_slip=[a["slip"] for a in fine], fine_tip=[a["tip"] for a in fine])
            c = out["configs"][name]
            print(f"{name:22s} CG h {b.h * 100:.1f} cm: sim min {min(c['sim']):.2f} mean {np.mean(c['sim']):.2f} | "
                  f"model min {min(c['fine_slip']):.2f} mean {np.mean(c['fine_slip']):.2f} | tip min {min(c['fine_tip']):.2f} "
                  f"| model tip-limited dirs {[d for d, l in zip(DIRS, c['limited_by']) if l == 'tip']} "
                  f"| sim failures {dict(zip(DIRS, c['sim_fail']))}", flush=True)

        # 2. mu sweep at each config's worst analytic direction
        mu_jobs = {}
        for name, p in CONFIGS.items():
            c = out["configs"][name]
            worst = DIRS[int(np.argmin(c["sim"]))]
            c["worst_direction"] = worst
            mu_jobs[name] = pool.map(_clean, [(p.with_(wheel_mu=mu), worst) for mu in MUS])
        for name, p in CONFIGS.items():
            c = out["configs"][name]
            alloc = "optimal" if p.allocation == "load_aware" else "pinv"
            c["mu"] = dict(mu=MUS, sim=[s[0] for s in mu_jobs[name]],
                           analytic=[_model(p.with_(wheel_mu=mu)).limit(c["worst_direction"], alloc)["slip"]
                                     for mu in MUS])
            print(f"{name:22s} mu sweep at {c['worst_direction']} deg: sim {np.round(c['mu']['sim'], 2)} "
                  f"model {np.round(c['mu']['analytic'], 2)}", flush=True)

        # 3. panic stops at mu 0.95 and 1.2
        stop_jobs = {(name, mu): pool.map(panic_stop, [(p.with_(wheel_mu=mu), a, 1.4) for a in DIRS])
                     for name, p in CONFIGS.items() for mu in (0.95, 1.2)}
        out["panic_stop"] = {}
        for (name, mu), res in stop_jobs.items():
            res = list(res)
            out["panic_stop"][f"{name} | mu {mu}"] = res
            worst = max(res, key=lambda r: r["peak_tilt"])
            print(f"panic stop {name:22s} mu {mu}: peak decel {max(r['peak_decel'] for r in res):.1f} m/s^2, "
                  f"worst tilt {worst['peak_tilt']:.1f} deg moving at {worst['direction']} deg, "
                  f"min wheel load {min(r['min_load'] for r in res):.1f} N, fell: {any(r['fell'] for r in res)}", flush=True)

        # 4. uneven floor, x4
        x4 = CONFIGS["x4 r150 rigid, axle deck"]
        cases = [(x4, dl) for dl in (0.0, 0.0005, 0.001, 0.002)] + \
                [(x4.with_(roller_stiffness=8e3), dl) for dl in (0.0, 0.001, 0.002)] + \
                [(x4.with_(rocker_deg=3.0), dl) for dl in (0.0, 0.001, 0.002)]
        out["uneven"] = list(pool.map(uneven, cases))
        for u, (pc, _) in zip(out["uneven"], cases):
            u["suspension_N_per_mm"] = pc.roller_stiffness / 1000
            print(f"uneven: rocker {u['rocker']:.0f} deg, wheel springs {pc.roller_stiffness / 1000:.0f} N/mm, "
                  f"wheel 1 proud {u['delta'] * 1000:.1f} mm: loads "
                  f"{np.round(u['loads'], 1)} N, accel {u['accel']}", flush=True)
    save("wheel_count", out)
    plot(out)


def plot(out):
    fig = plt.figure(figsize=(14, 4.6))
    cols = ["#7a9cc6", "#2a6fdb", "#e8743b", "#8e44ad", "#c0392b", "#19a979"]
    ax = fig.add_subplot(1, 3, 1, projection="polar")
    for (name, c), col in zip(out["configs"].items(), cols):
        th = np.radians(c["fine_dirs"] + [360])
        ax.plot(th, c["fine_slip"] + c["fine_slip"][:1], "--", color=col, lw=1)
        ths = np.radians(out["directions"] + [360])
        ax.plot(ths, c["sim"] + c["sim"][:1], "o-", color=col, ms=3, label=name)
    ax.set_rmax(7)
    ax.set_title("Slip-free acceleration (m/s²), 0° = forward\nsolid: sim, dashed: quasi-static model", fontsize=9)
    ax.legend(loc="lower left", fontsize=7, bbox_to_anchor=(-0.25, -0.28))
    ax2 = fig.add_subplot(1, 3, 2, projection="polar")
    for (name, c), col in zip(out["configs"].items(), cols):
        th = np.radians(c["fine_dirs"] + [360])
        ax2.plot(th, c["fine_tip"] + c["fine_tip"][:1], "-", color=col, lw=1.2, label=name)
        ax2.plot(th, c["fine_slip"] + c["fine_slip"][:1], ":", color=col, lw=1)
    ax2.set_rmax(12)
    ax2.set_title("Tip-over acceleration (solid) vs slip limit (dotted), m/s²", fontsize=9)
    ax3 = fig.add_subplot(1, 3, 3)
    keys = list(out["panic_stop"].keys())
    vals = [max(r["peak_tilt"] for r in out["panic_stop"][k]) for k in keys]
    ax3.barh(range(len(keys)), vals, color=[cols[i // 2] for i in range(len(keys))])
    ax3.set_yticks(range(len(keys)), keys, fontsize=7)
    ax3.invert_yaxis()
    ax3.set_xlabel("worst tilt, panic stop from 1.4 m/s (deg)")
    ax3.axvline(30, color="0.4", lw=0.8, ls="--")
    ax3.text(31, len(keys) - 0.6, "falls over", fontsize=7, color="0.3")
    ax3.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    fig.savefig(RESULTS / "wheel_count.png", dpi=130)


if __name__ == "__main__":
    main()
