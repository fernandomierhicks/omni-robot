"""Launcher trade study: flywheel diameter x drone motor.

1. Exit speed the launcher must reach: return throws at 52 deg to chest height
   at 1.5-5 m with the 22 g foam ball (drag included), plus 15% margin.
2. For each flywheel diameter (printed PETG hub + TPU tire + steel bolt ring,
   inertia from geometry), bench-fire the ball on the full MuJoCo launcher to get
   exit speed vs rpm, elevation, scatter and per-shot speed dip.
3. For each motor on its ESC (25 A limit) and a sagging 7S pack: rpm and throttle
   needed at max speed, spin-up time, recovery after a shot, copper heat per shot.

Writes results/launcher_components.json and results/launcher_components.png.
"""

from __future__ import annotations

import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from common import RESULTS, SimParams, save  # noqa: E402
from ball_flight import BallFlight  # noqa: E402
from catalog import ESCS, FLYWHEEL_MOTORS  # noqa: E402
from launcher_bench import Bench  # noqa: E402
from models import FlywheelDrive  # noqa: E402
from robot_model import geometry  # noqa: E402

PETG, TPU, INFILL = 1270.0, 1200.0, 0.5


def flywheel_inertia(D: float, width: float = 0.028, tire: float = 0.006, bolts: int = 8, rotor: float = 4.5e-6):
    """Printed flywheel: PETG hub (50% infill), solid TPU tire, M4 steel bolt+nut ring."""
    R = D / 2
    rh = R - tire
    m_hub = math.pi * rh ** 2 * width * PETG * INFILL
    m_tire = math.pi * (R ** 2 - rh ** 2) * width * TPU
    m_bolts = bolts * 0.004
    r_b = rh - 0.008
    inertia = 0.5 * m_hub * rh ** 2 + 0.5 * m_tire * (R ** 2 + rh ** 2) + m_bolts * r_b ** 2 + rotor
    return inertia, m_hub + m_tire + m_bolts


def required_speeds(p: SimParams):
    b = p.ball_spec
    fl = BallFlight(b.mass, b.radius, b.drag_cd, p.air_density, p.air_viscosity)
    g = geometry(p)
    exit_z = g.plate_top + g.N[1] + 0.06 * g.u[1]
    th = math.radians(p.launch_angle_deg)
    rows = []
    for dist in (1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0):
        for h, who in ((1.2, "adult"), (0.9, "kid")):
            v = fl.solve_launch_speed([0, 0, exit_z], [1, 0], th, [dist, 0, h])
            rows.append(dict(distance=dist, target_height=h, who=who, speed=v))
    return rows


def spin_up(motor, esc, inertia, w_target, v_bus=25.2, dt=2e-4):
    """Time from rest to within 2% of w_target, plus peak bus current (MCU loop + ESC limit)."""
    fd = FlywheelDrive(motor, esc)
    w, t, i_peak = 0.0, 0.0, 0.0
    while t < 3.0:
        tau = fd.step(w_target, w, v_bus, dt)
        w += tau / inertia * dt
        t += dt
        i_peak = max(i_peak, fd.i_bus)
        if abs(w - w_target) < 0.02 * w_target:
            return t, i_peak
    return float("nan"), i_peak


def recovery(motor, esc, inertia, w_target, drop, v_bus=25.2, dt=2e-4):
    fd = FlywheelDrive(motor, esc)
    fd.w_meas, fd.integ = w_target, 0.0
    w, t = w_target * (1 - drop), 0.0
    e = 0.0
    while t < 1.0:
        tau = fd.step(w_target, w, v_bus, dt)
        w += tau / inertia * dt
        t += dt
        e += motor.r_m * fd.i_phase ** 2 * dt
        if w > 0.98 * w_target:
            return t, e
    return float("nan"), e


def main():
    base = SimParams()
    req = required_speeds(base)
    v_need = max(r["speed"] for r in req if r["who"] == "adult")
    v_max = 1.15 * v_need
    v_typ = next(r["speed"] for r in req if r["distance"] == 3.0 and r["who"] == "adult")
    print(f"required exit speed: {v_need:.2f} m/s at 5 m (design max {v_max:.2f}), typical 3 m throw {v_typ:.2f} m/s")

    esc = ESCS[base.esc]
    out = dict(requirements=req, v_max=v_max, v_typ=v_typ, diameters=[])
    for D in (0.050, 0.060, 0.070, 0.080):
        I, mass = flywheel_inertia(D)
        p = base.with_(flywheel_diameter=D, flywheel_inertia=I, flywheel_mass=mass)
        bench = Bench(p)
        shots = []
        for rpm in np.linspace(1500, 6000 * 0.07 / D, 5):
            s = bench.shot(float(rpm))
            if s.ok:
                shots.append(s)
        w = np.array([s.rpm * 2 * math.pi / 60 for s in shots])
        v = np.array([s.exit_speed for s in shots])
        eta = v / (w * D / 2)
        # rpm needed for v_max, extrapolating the last two points if needed
        w_max = float(np.interp(v_max, v, w, right=w[-1] + (v_max - v[-1]) * (w[-1] - w[-2]) / (v[-1] - v[-2])))
        w_typ = float(np.interp(v_typ, v, w))
        repeat = [bench.shot(w_typ * 60 / (2 * math.pi)) for _ in range(5)]
        rep = [s for s in repeat if s.ok]
        drop = float(np.mean([s.rpm_drop for s in rep]))
        row = dict(D=D, inertia=I, mass=mass, rpm=[s.rpm for s in shots], exit=v.tolist(), eta=eta.tolist(),
                   elevation=[s.elevation for s in shots], azimuth=[s.azimuth for s in shots],
                   rpm_max=w_max * 60 / (2 * math.pi), rpm_typ=w_typ * 60 / (2 * math.pi),
                   repeat_speed_std=float(np.std([s.exit_speed for s in rep])),
                   repeat_speed_mean=float(np.mean([s.exit_speed for s in rep])),
                   repeat_az_std=float(np.std([s.azimuth for s in rep])),
                   repeat_el_std=float(np.std([s.elevation for s in rep])),
                   drop=drop, motors={})
        for key, mot in FLYWHEEL_MOTORS.items():
            ke = 1 / (mot.kv * 2 * math.pi / 60)
            throttle_low = ke * w_max / 22.0 + mot.r_m * mot.kt * mot.i0 / mot.kt / 22.0
            t_spin, i_spin = spin_up(mot, esc, I + mot.rotor_inertia - 4.5e-6, w_typ)
            t_spin_max, _ = spin_up(mot, esc, I + mot.rotor_inertia - 4.5e-6, w_max)
            t_rec, e_shot = recovery(mot, esc, I, w_typ, drop)
            row["motors"][key] = dict(throttle_at_max_low_batt=throttle_low, spin_up_typ=t_spin, spin_up_max=t_spin_max,
                                      i_bus_peak=i_spin, recovery=t_rec, copper_j_per_shot=e_shot,
                                      noload_rpm_25v=mot.kv * 25.2, mass=mot.mass, price=mot.price)
        out["diameters"].append(row)
        print(f"D {D * 1000:.0f} mm: I {I * 1e6:.0f} g cm2 x10, mass {mass * 1000:.0f} g, eta {np.round(eta, 2)}, "
              f"rpm typ {row['rpm_typ']:.0f} max {row['rpm_max']:.0f}, drop {100 * drop:.0f}%, "
              f"repeat {row['repeat_speed_mean']:.2f}+-{row['repeat_speed_std']:.2f} m/s az+-{row['repeat_az_std']:.1f}", flush=True)
        for k, mrow in row["motors"].items():
            print(f"    {k:18s} throttle@max(22V) {mrow['throttle_at_max_low_batt']:.2f} spin-up {mrow['spin_up_typ']:.2f}s "
                  f"(max {mrow['spin_up_max']:.2f}s) recover {1000 * mrow['recovery']:.0f} ms, I_bus peak {mrow['i_bus_peak']:.1f} A", flush=True)
    save("launcher_components", out)
    plot(out)


def plot(out):
    fig, axs = plt.subplots(1, 3, figsize=(13, 4))
    req = out["requirements"]
    for who, c in (("adult", "#2a6fdb"), ("kid", "#e8743b")):
        rr = [r for r in req if r["who"] == who]
        axs[0].plot([r["distance"] for r in rr], [r["speed"] for r in rr], "o-", color=c,
                    label=f"to {rr[0]['target_height']:.1f} m ({who})")
    axs[0].axhline(out["v_max"], color="0.4", ls="--", lw=1, label="design max (+15%)")
    axs[0].set_xlabel("throw distance (m)")
    axs[0].set_ylabel("exit speed at 52° (m/s)")
    axs[0].set_title("Exit speed needed (22 g foam ball, with drag)", fontsize=9)
    axs[0].grid(alpha=0.3)
    axs[0].legend(fontsize=8)
    for row in out["diameters"]:
        axs[1].plot(row["rpm"], row["exit"], "o-", label=f"{row['D'] * 1000:.0f} mm")
        axs[2].plot([v for v in row["exit"]], row["eta"], "o-", label=f"{row['D'] * 1000:.0f} mm")
    axs[1].axhline(out["v_max"], color="0.4", ls="--", lw=1)
    axs[1].set_xlabel("flywheel rpm")
    axs[1].set_ylabel("ball exit speed (m/s)")
    axs[1].set_title("Bench: exit speed vs flywheel speed", fontsize=9)
    axs[1].grid(alpha=0.3)
    axs[1].legend(fontsize=8, title="flywheel Ø")
    axs[2].set_xlabel("ball exit speed (m/s)")
    axs[2].set_ylabel("exit speed / rim speed")
    axs[2].set_title("Transfer efficiency", fontsize=9)
    axs[2].grid(alpha=0.3)
    axs[2].legend(fontsize=8, title="flywheel Ø")
    fig.tight_layout()
    fig.savefig(RESULTS / "launcher_components.png", dpi=130)


if __name__ == "__main__":
    main()
