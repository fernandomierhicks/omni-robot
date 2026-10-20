"""Omni-wheel traction and slip study.

1. Slip-free acceleration in every direction (sim, sphere wheels) vs the
   quasi-static kiwi traction/tip model.
2. Worst-direction acceleration vs roller friction (mu).
3. Worst-direction acceleration vs CG height (extra payload on the funnel rim).
4. Spot checks with the explicit-roller wheel model.

Writes results/wheel_slip.json and results/wheel_slip.png.
"""

from __future__ import annotations

import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from common import RESULTS, SimParams, analytic_max_accel, cg_height, max_clean_accel, save  # noqa: E402


def main():
    out = {}
    base = SimParams()
    h0, mass0, cxy = cg_height(base, with_xy=True)
    out["baseline"] = dict(cg_height=h0, cg_xy=cxy, mass=mass0, mu=base.mu, wheel=base.wheel)
    print("baseline", out["baseline"])

    # 1. polar
    dirs = list(range(0, 360, 30))
    sim = [max_clean_accel(base, a) for a in dirs]
    ana = [analytic_max_accel(base, a, h0, cxy) for a in dirs]
    out["polar"] = dict(directions=dirs, sim=sim, analytic=ana)
    print("polar sim", np.round(sim, 2))
    print("polar analytic", np.round(ana, 2))

    # 2. mu sweep, worst direction (0 deg: between the two front wheels)
    mus = [0.6, 0.75, 0.95, 1.1]
    print("polar sim", np.round(sim, 2))
    worst = int(dirs[int(np.argmin(sim))])
    out["worst_direction"] = worst
    out["mu"] = dict(mu=mus, direction=worst,
                     sim=[max_clean_accel(base.with_(wheel_mu=m), worst) for m in mus],
                     analytic=[analytic_max_accel(base.with_(wheel_mu=m), worst, h0, cxy) for m in mus])
    print("mu", out["mu"])

    # 3. CG height: extra mass on the funnel rim (e.g. a heavier module)
    extra = [0.0, 0.5, 1.0, 2.0]
    rows = []
    for e in extra:
        p = base.with_(m_funnel=base.m_funnel + e)
        h, mass, c = cg_height(p, with_xy=True)
        rows.append(dict(extra=e, cg_height=h, mass=mass, sim=max_clean_accel(p, worst),
                         analytic=analytic_max_accel(p, worst, h, c)))
    out["cg"] = rows
    print("cg", rows)

    # 4. explicit rollers: same metric with a looser tolerance (they ripple ~3% at any accel)
    pr = base.with_(wheel_model="rollers")
    out["rollers"] = {str(a): max_clean_accel(pr, a, err_tol=0.08, drift_tol=3.0) for a in (0, 90, worst)}
    out["sphere_same_tol"] = {str(a): max_clean_accel(base, a, err_tol=0.08, drift_tol=3.0) for a in (0, 90, worst)}
    print("rollers", out["rollers"], "sphere", out["sphere_same_tol"])
    save("wheel_slip", out)
    plot(out)


def plot(out):
    fig = plt.figure(figsize=(12, 4.2))
    ax = fig.add_subplot(1, 3, 1, projection="polar")
    th = np.radians(out["polar"]["directions"] + [360])
    ax.plot(th, out["polar"]["sim"] + out["polar"]["sim"][:1], "o-", color="#2a6fdb", label="sim (sphere wheels)")
    ax.plot(th, out["polar"]["analytic"] + out["polar"]["analytic"][:1], "--", color="#e8743b", label="quasi-static model")
    for wa in (60, 180, 300):
        ax.plot([math.radians(wa)] * 2, [0, 6], color="0.6", lw=0.8)
    ax.set_title("Slip-free acceleration (m/s²) by direction\n0° = forward, grey = wheel directions", fontsize=9)
    ax.set_rmax(6)
    ax.legend(loc="lower left", fontsize=7, bbox_to_anchor=(-0.15, -0.2))
    ax2 = fig.add_subplot(1, 3, 2)
    ax2.plot(out["mu"]["mu"], out["mu"]["sim"], "o-", color="#2a6fdb", label="sim")
    ax2.plot(out["mu"]["mu"], out["mu"]["analytic"], "--", color="#e8743b", label="model")
    ax2.set_xlabel("roller friction coefficient μ")
    ax2.set_ylabel("worst-direction accel (m/s²)")
    ax2.set_title(f"Traction vs roller friction ({out['worst_direction']}° move)", fontsize=9)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)
    ax3 = fig.add_subplot(1, 3, 3)
    hs = [r["cg_height"] * 100 for r in out["cg"]]
    ax3.plot(hs, [r["sim"] for r in out["cg"]], "o-", color="#2a6fdb", label="sim")
    ax3.plot(hs, [r["analytic"] for r in out["cg"]], "--", color="#e8743b", label="model")
    ax3.set_xlabel("CG height (cm)")
    ax3.set_ylabel("worst-direction accel (m/s²)")
    ax3.set_title(f"Traction vs CG height (mass at the rim, {out['worst_direction']}° move)", fontsize=9)
    ax3.grid(alpha=0.3)
    ax3.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(RESULTS / "wheel_slip.png", dpi=130)


if __name__ == "__main__":
    main()
