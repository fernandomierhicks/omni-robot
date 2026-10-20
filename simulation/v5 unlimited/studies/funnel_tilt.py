"""Funnel trade study: does tilting the funnel into the ball's path allow a shallower funnel?

Static robot. The ball arrives at speed V, theta degrees from vertical (coming
from the front), and crosses the horizontal plane at rim-center height at a
horizontal offset from the rim center (the positioning + prediction error). The
module is tilted to gain * theta. A trial counts as caught when the break-beam
sees the ball 0.7 s later.

Configurations: wall angle (v2's 35 deg steep cone vs shallower ones),
alignment gain (0 = upright as in v2, 1 = fully aligned), throat flaps on/off.
Writes results/funnel_tilt.json and .png.
"""

from __future__ import annotations

import math
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from common import RESULTS, SimParams, save  # noqa: E402
from launcher_bench import Bench  # noqa: E402

V = 6.4                          # m/s: a 2.7 m toss with a 2 m apex (the fast end)
ANGLES = (12.0, 24.0, 36.0)      # deg from vertical: spans the 2-4 m / 1-2 m apex tosses
OFFSETS = [(0.0, 0.0)] + [(r * math.cos(a), r * math.sin(a)) for r in (0.04, 0.07, 0.10)
                          for a in np.radians(np.arange(0, 360, 45))]

CONFIGS = [
    # name, wall angle, alignment gain, throat flaps, dump (swing upright once the ball is in)
    ("v2 cone 35, upright", 35, 0.0, False, False),
    ("50, upright", 50, 0.0, False, False),
    ("50, aligned", 50, 1.0, False, False),
    ("50, aligned + dump", 50, 1.0, False, True),
    ("50, 60% aligned + dump", 50, 0.6, False, True),
    ("55, aligned + dump", 55, 1.0, False, True),
    ("55, 60% aligned + dump (v3)", 55, 0.6, False, True),
    ("60, aligned + dump", 60, 1.0, False, True),
    ("60, 60% aligned + dump", 60, 0.6, False, True),
    ("65, aligned + dump", 65, 1.0, False, True),
]


def trial(b: Bench, theta_deg: float, gain: float, offset, dump: bool = False, dump_delay: float = 0.03) -> bool:
    """dump: once the ball is below the rim, swing the funnel back upright so a ball resting on the
    (now shallow) far wall rolls into the throat."""
    r, d, g = b.r, b.d, b.r.g
    th = math.radians(theta_deg)
    tilt = gain * th
    b.set_tilt(math.degrees(tilt))
    # rim-center height at this tilt, in the world
    x_rc, z_rc = g.rim_center_robot(tilt)
    bpos = d.sensor("base_pos").data
    target = np.array([bpos[0] + x_rc + offset[0], bpos[1] + offset[1], bpos[2] + z_rc])
    v_t = V * np.array([-math.sin(th), 0.0, -math.cos(th)])
    T = 0.06
    gz = np.array([0, 0, 9.81])
    p0 = target - v_t * T - 0.5 * gz * T * T
    v0 = v_t + gz * T
    d.qpos[b.bq:b.bq + 7] = [*p0, 1, 0, 0, 0]
    d.qvel[b.bv:b.bv + 6] = [*v0, 0, 0, 0]
    t_in = None
    t0 = d.time
    while d.time - t0 < 0.9:
        r.step()
        if dump and t_in is None:
            fc = r.funnel_center_world()
            if d.sensor("ball_pos").data[2] < fc[2]:
                t_in = d.time
        if t_in is not None and d.time - t_in > dump_delay:
            r.tilt_target = 0.0
    held = r.ball_held()
    d.qpos[b.bq:b.bq + 7] = [-2.2, 1.7, 0.037, 1, 0, 0, 0]       # park the ball for the next trial
    d.qvel[b.bv:b.bv + 6] = 0
    b.run(0.05)
    return bool(held)


def run_config(cfg) -> dict:
    name, wall, gain, flaps, dump = cfg
    t0 = time.perf_counter()
    p = SimParams(funnel_wall_angle_deg=wall, throat_flaps=flaps)
    b = Bench(p)
    g = b.r.g
    res = {}
    for th in ANGLES:
        res[str(th)] = [trial(b, th, gain, off, dump=dump) for off in OFFSETS]
    rate = {k: float(np.mean(v)) for k, v in res.items()}
    by_r = {}
    for rad in (0.0, 0.04, 0.07, 0.10):
        idx = [i for i, o in enumerate(OFFSETS) if abs(math.hypot(*o) - rad) < 1e-6]
        by_r[str(rad)] = float(np.mean([res[str(th)][i] for th in ANGLES for i in idx]))
    return dict(name=name, wall=wall, gain=gain, flaps=flaps, dump=dump, funnel_depth=p.funnel_depth,
                module_height=g.rim_z, rim_height=float(g.rim_center_robot(0.0)[1]),
                held=res, rate_by_angle=rate, rate_by_offset=by_r,
                overall=float(np.mean([x for v in res.values() for x in v])), seconds=time.perf_counter() - t0)


def main():
    from concurrent.futures import ProcessPoolExecutor
    out = dict(V=V, angles=list(ANGLES), offsets=OFFSETS, configs=[])
    with ProcessPoolExecutor(len(CONFIGS)) as pool:
        rows = list(pool.map(run_config, CONFIGS))
    for row in rows:
        name = row["name"]
        rate, by_r = row["rate_by_angle"], row["rate_by_offset"]
        out["configs"].append(row)
        print(f"{name:26s} depth {row['funnel_depth'] * 1000:4.0f} mm, rim {row['rim_height'] * 1000:4.0f} mm: "
              f"caught {100 * row['overall']:5.1f}%  by angle " +
              " ".join(f"{k}:{100 * v:3.0f}%" for k, v in rate.items()) + "  by offset " +
              " ".join(f"{k}:{100 * v:3.0f}%" for k, v in by_r.items()) + f"  ({row['seconds']:.0f} s)",
              flush=True)
    save("funnel_tilt", out)
    plot(out)


def plot(out):
    cfg = out["configs"]
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    names = [c["name"] for c in cfg]
    y = np.arange(len(cfg))
    for k, th in enumerate(out["angles"]):
        ax[0].barh(y + (k - 1) * 0.27, [100 * c["rate_by_angle"][str(th)] for c in cfg], height=0.27,
                   label=f"{th:.0f} deg from vertical")
    ax[0].set_yticks(y, names, fontsize=8)
    ax[0].invert_yaxis()
    ax[0].set_xlabel("caught (%) over offsets up to 10 cm")
    ax[0].legend(fontsize=7, loc="lower right")
    ax[0].set_title(f"Catch rate at {out['V']} m/s arrival", fontsize=9)
    ax[0].grid(axis="x", alpha=0.3)
    for c in cfg:
        rs = sorted(float(r) for r in c["rate_by_offset"])
        ax[1].plot([100 * r for r in rs], [100 * c["rate_by_offset"][str(r)] for r in rs], "o-", label=c["name"])
    ax[1].set_xlabel("landing offset from rim center (cm)")
    ax[1].set_ylabel("caught (%)")
    ax[1].set_title("Catch rate vs positioning error (all arrival angles)", fontsize=9)
    ax[1].grid(alpha=0.3)
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(RESULTS / "funnel_tilt.png", dpi=130)


if __name__ == "__main__":
    main()
