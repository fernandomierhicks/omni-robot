"""How hard can the launcher throw with no rpm cap?

The F90 2806.5 1300KV spins to ~38,000 rpm unloaded on a full 7S pack, so the
motor isn't the limit. This bench fires at 3,000-20,000 rpm (40 deg launch) at a
0.1 ms timestep (the 0.5 ms game step is too coarse for the flywheel contact
above ~5,000 rpm), and reports exit speed, spin-up time from rest, the rpm dip,
the ESC phase current, and the drag-limited range of the 22 g foam ball.

Flywheel stress is an estimate (est) for the printed Ø70 PETG hub + TPU tire:
rim hoop stress of the tire sigma = rho v^2, solid-disc hub sigma = (3 + nu)/8
rho w^2 R^2, against printed-part strengths of ~25 MPa (TPU 95A) and ~25 MPa
(PETG across layers). VERIFY with a spin test behind a guard before any shot
above the 5,000 rpm calibration.

Writes results/launcher_max.json.
"""

from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from common import SimParams, save
from ball_flight import BallFlight
from launcher_bench import Bench

RPMS = (3000, 5000, 7500, 10000, 12500, 15000, 20000)
TILT = -10.0                                  # 40 deg launch


def shot(args):
    rpm, = args
    p = SimParams(timestep=0.0001)
    b = Bench(p)
    b.set_tilt(TILT)
    r = b.r
    # spin-up from rest
    r.fly_target = rpm * 2 * math.pi / 60
    t0 = b.d.time
    i_peak = 0.0
    while not r.flywheels_ready() and b.d.time - t0 < 6.0:
        r.step()
        i_peak = max(i_peak, max(abs(f.i_phase) for f in r.fly))
    t_spin = b.d.time - t0
    r.fly_target = 0.0
    b.run(0.0001)
    s = b.shot(rpm, TILT)
    return dict(rpm=rpm, ok=s.ok, exit=s.exit_speed, elev=s.elevation, spin=s.spin, dip=s.rpm_drop,
                t_spin=t_spin, i_phase_peak=i_peak)


def stress(rpm, R=0.035, R_hub=0.029):
    w = rpm * 2 * math.pi / 60
    tire = 1210 * (w * R) ** 2 / 1e6               # MPa, thin-ring hoop stress
    hub = (3 + 0.4) / 8 * 1270 * w ** 2 * R_hub ** 2 / 1e6
    return tire, hub


def main():
    with ProcessPoolExecutor(min(len(RPMS), os.cpu_count() or 4)) as pool:
        rows = list(pool.map(shot, [(r,) for r in RPMS]))
    f = BallFlight(0.022, 0.037, 0.5)
    for row in rows:
        v = row["exit"]
        row["range_40"] = f._horizontal_range([0, 0, 0.45], [v * math.cos(math.radians(40)), 0, v * math.sin(math.radians(40))], 0.15) if row["ok"] else float("nan")
        row["best_range"], row["best_elev"] = max((f._horizontal_range([0, 0, 0.45], [v * math.cos(math.radians(e)), 0, v * math.sin(math.radians(e))], 0.15), e)
                                                  for e in range(20, 61, 5)) if row["ok"] else (float("nan"), 0)
        row["stress_tire"], row["stress_hub"] = stress(row["rpm"])
    save("launcher_max", rows)
    print("Launcher with no rpm cap (40 deg launch; range to 0.15 m height with drag):")
    for r in rows:
        print(f"  {r['rpm']:6d} rpm: " + (f"exit {r['exit']:5.1f} m/s at {r['elev']:4.1f} deg, spin {r['spin']:4.0f} rad/s, dip "
              f"{100 * r['dip']:3.0f}%, spin-up {r['t_spin']:.2f} s (phase {r['i_phase_peak']:.0f} A); range {r['range_40']:4.1f} m "
              f"(best {r['best_range']:.1f} m at {r['best_elev']} deg)" if r["ok"] else "NO EXIT") +
              f"; stress tire {r['stress_tire']:.1f} / hub {r['stress_hub']:.1f} MPa (est)")


if __name__ == "__main__":
    main()
