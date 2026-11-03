"""Fit the drive-motor model to a maker torque-speed chart (design.md 4.7, the Nov 1 motor characterization).

The chart is a full-throttle sweep at a fixed DC supply: for each output torque it gives speed, supply current,
efficiency and output power. The model, all at the output shaft (the gearbox's losses are folded in):

    voltage limit     V = ke * w + R * i
    shaft torque      T = k(i) * i - T0 - b * w,     k(i) = k0 * (1 - sat * i)
    supply power      P = k(i) * i * w + R * i^2 + p0,   I_dc = P / V

i is the equivalent q-axis current, R lumps winding, driver and wiring resistance, T0 and b the gearbox's
Coulomb and viscous friction, p0 the driver's idle draw. catalog.DriveMotor carries the fitted values (kt * gear
= k0, gear_eff = 1, r_fit = R, k_sat = sat, backdrive_torque = T0, viscous = b, p_idle = p0); models.DriveActuator
uses them.

  python motor_fit.py                  # fits data/ak40_10_kv170_24v.csv, prints the catalog values,
                                       # writes results/motor_char/ak40_fit.png and ak40_fit.json
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
from scipy.optimize import brentq, least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
RPM = 2 * math.pi / 60
OUT = os.path.join(HERE, "results", "motor_char")


def load(path):
    rows = [ln.split(",") for ln in open(path) if ln.strip() and not ln.startswith("#")]
    head, rows = rows[0], rows[1:]
    col = {h.strip(): i for i, h in enumerate(head)}
    def get(name):
        return np.array([float(r[col[name]]) if r[col[name]].strip() else np.nan for r in rows])
    return {h: get(h) for h in col}


def operating_point(x, T, V):
    """(w, i, I_dc, efficiency) at output torque T on the full-throttle line at supply V."""
    ke, R, k0, sat, T0, b, p0 = x
    def resid(i):
        w = (V - R * i) / ke
        return k0 * (1 - sat * i) * i - T0 - b * w - T
    i_hi = V / R
    i = brentq(resid, 0.0, i_hi * 0.999)
    w = (V - R * i) / ke
    P = k0 * (1 - sat * i) * i * w + R * i * i + p0
    return w, i, P / V, T * w / P if P > 0 else 0.0


def residuals(x, d, V):
    out = []
    for T, rpm, cur, eff in zip(d["torque_Nm"], d["speed_rpm"], d["current_A"], d["efficiency_pct"]):
        w, i, I, e = operating_point(x, T, V)
        out.append((w / RPM - rpm) / 5.0)                 # weights = the read accuracy
        out.append((I - cur) / 0.1)
        if not np.isnan(eff):
            out.append((100 * e - eff) / 2.0)
    return np.array(out)


def fit(path, V=24.0):
    d = load(path)
    x0 = [24 / (450 * RPM), 2.0, 0.55, 0.0, 0.05, 0.001, 3.0]
    lo = [0.3, 0.1, 0.2, 0.0, 0.0, 0.0, 0.0]
    hi = [1.0, 6.0, 1.5, 0.08, 0.5, 0.05, 15.0]
    r = least_squares(residuals, x0, bounds=(lo, hi), args=(d, V))
    return r.x, d


def report(x, d, V=24.0):
    ke, R, k0, sat, T0, b, p0 = x
    rows, se, ce, ee = [], [], [], []
    for T, rpm, cur, eff in zip(d["torque_Nm"], d["speed_rpm"], d["current_A"], d["efficiency_pct"]):
        w, i, I, e = operating_point(x, T, V)
        rows.append(dict(T=T, rpm=rpm, rpm_fit=w / RPM, A=cur, A_fit=I, eff=eff, eff_fit=100 * e, i_q=i))
        se.append((w / RPM - rpm) / rpm)
        if cur > 1.0:
            ce.append((I - cur) / cur)
        if not np.isnan(eff):
            ee.append(100 * e - eff)
    stats = dict(rms_speed_pct=100 * float(np.sqrt(np.mean(np.square(se)))),
                 rms_current_pct=100 * float(np.sqrt(np.mean(np.square(ce)))),
                 rms_eff_pts=float(np.sqrt(np.mean(np.square(ee)))))
    return rows, stats


def plot(x, d, path, V=24.0):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    Ts = np.linspace(0.0, 4.1, 80)
    pts = [operating_point(x, T, V) for T in Ts]
    fig, ax = plt.subplots(2, 2, figsize=(10, 7))
    for a, (k, f, lab) in zip(ax.flat, [("speed_rpm", lambda p: p[0] / RPM, "speed, rpm"),
                                       ("current_A", lambda p: p[2], "DC current, A"),
                                       ("efficiency_pct", lambda p: 100 * p[3], "efficiency, %"),
                                       ("power_W", None, "output power, W")]):
        a.plot(d["torque_Nm"], d[k], "o", label="maker chart (digitized)")
        if f is None:
            a.plot(Ts, [T * p[0] for T, p in zip(Ts, pts)], "-", label="model")
        else:
            a.plot(Ts, [f(p) for p in pts], "-", label="model")
        a.set_xlabel("output torque, N m"); a.set_ylabel(lab); a.grid(alpha=0.3)
    for Vb, ls in ((25.2, "--"), (21.0, ":")):
        ax[0, 0].plot(Ts, [operating_point(x, T, Vb)[0] / RPM for T in Ts], ls, label=f"model at {Vb} V")
    ax[0, 0].legend(fontsize=8)
    fig.suptitle("AK40-10 KV170: maker 24 V chart vs fitted model")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    path = os.path.join(HERE, "data", "ak40_10_kv170_24v.csv")
    x, d = fit(path)
    rows, stats = report(x, d)
    ke, R, k0, sat, T0, b, p0 = x
    names = ["ke_out", "R", "k0", "sat", "T0", "b", "p0"]
    print("fit:", ", ".join(f"{n}={v:.5g}" for n, v in zip(names, x)))
    print("stats:", {k: round(v, 2) for k, v in stats.items()})
    for r in rows:
        print("  T {T:4.2f}  rpm {rpm:5.0f} / {rpm_fit:5.0f}   A {A:4.2f} / {A_fit:4.2f}   eff {eff:5.1f} / {eff_fit:5.1f}"
              "   i_q {i_q:4.2f}".format(**r))
    os.makedirs(OUT, exist_ok=True)
    plot(x, d, os.path.join(OUT, "ak40_fit.png"))
    with open(os.path.join(OUT, "ak40_fit.json"), "w") as f:
        json.dump(dict(source=os.path.relpath(path, HERE), model=dict(zip(names, map(float, x))), stats=stats,
                       points=rows), f, indent=1)
    # the catalog form
    gear = 10
    print(f"\ncatalog: kt={k0 / gear:.5f} (k0/gear), gear=10, gear_eff=1.0, r_fit={R:.4f}, k_sat={sat:.5f}, "
          f"noload_rpm={24 / ke / RPM:.2f} (ideal, at v_rated 24), backdrive_torque={T0:.4f}, viscous={b:.5f}, "
          f"p_idle={p0:.3f}")
    return x, stats


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
