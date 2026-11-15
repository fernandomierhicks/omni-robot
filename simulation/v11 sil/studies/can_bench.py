"""Two AK40-10s on CAN through the robot's own firmware: timing, response and telemetry (Nov 13, 2026; hardware).

The bench: the Teensy 4.1 (env arm_bench, protocol 9, build aea161e4) on the Pi 4's USB, the robot step (mcu_link,
mode_manager, console); motor 1 bare, motor 2 with the cad/v13 omni wheel; 24 V bench supply. Profiles through the
console's motor test, recorded at the MCU's 200 Hz STATE by ros/tools/wheel_capture.py:

    idle.csv          8 s at rest (STATE timing, current offset and noise)
    steps_m<k>.csv    rest, +20, rest, -20, +20, rest rad/s at 2000 rad/s^2 (the motor's own acceleration cap)
    lat_m<k>.csv      40 starts from rest to +-3 rad/s at 2000 rad/s^2 (onset latency)
    tri_m<k>.csv      +-10 rad/s triangles at 50 rad/s^2 (lag on ramps the motor follows)
    sweep_m<k>.csv    steady speeds 2.5 ... 37.2 rad/s both ways (friction, top speed)

    python studies/can_bench.py C:/Users/ferna/bench_data/can_2026-11-13 [--out results/can_bench_2026-11-13.json]

The replay: each triangle's asked speed through the sim's wheel drive (wheel_bench.simulate: every 4 ms, round robin;
models.DriveActuator with the 148 rad/s^2 ramp) at this motor's inertia, read FB_AGE late as STATE reads a 500 Hz
status; then the extra pure delay that fits the bench best. Report: results/can_bench_2026-11-13.md.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

FB_AGE = 0.0015           # s: status 0-2 ms old at 500 Hz, + its 0.15 ms frame, decoded 0-1 ms before the STATE tick
KT = 0.48                 # N m/A of the reported current (inference: see the report), for J from the bench's J/k


def load(path: Path) -> dict:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    out = {"t": np.array([float(r["t"]) for r in rows])}
    for k in (1, 2):
        for c in "cwie":
            out[f"{c}{k}"] = np.array([float(r[f"{c}{k}"]) for r in rows])
    return out


def timing(d):
    dt = np.diff(d["t"]) * 1e3
    return dict(n=len(d["t"]), s=round(float(d["t"][-1]), 3), dt_med_ms=float(np.median(dt)),
                dt_max_ms=float(dt.max()), missing=int(np.round(dt / 5).sum() - len(dt)))


def lag_ms(t, c, w):
    """Least-squares shift of the measured speed behind the asked speed, where |asked| > 1 rad/s."""
    act = np.abs(c) > 1.0
    errs = [(L, float(np.mean((np.interp(t[act] - L, t, c) - w[act]) ** 2))) for L in np.arange(0, 0.06, 0.0002)]
    return min(errs, key=lambda e: e[1])[0] * 1e3


def onset_lags(d, m, ramp=2000.0, step=3.0):
    """Each start from rest: command onset (from STATE's ramp value; else mid-window) to the speed ramp's onset (its
    0.6-2.4 rad/s part extrapolated to 0)."""
    t, c, w = d["t"], d[f"c{m}"], d[f"w{m}"]
    out = []
    for k in range(6, len(c) - 12):
        if not (abs(c[k - 1]) < 1e-3 < abs(c[k])):
            continue
        ck = abs(c[k])
        t_on = t[k] - ck / ramp if ck < step - 1e-3 else 0.5 * (t[k - 1] + t[k] - step / ramp)
        s = np.sign(c[k])
        tt, ww = t[k - 2:k + 12], s * w[k - 2:k + 12]
        sel = (ww > 0.6) & (ww < 2.4)
        if sel.sum() >= 2:
            a, b = np.polyfit(tt[sel], ww[sel], 1)
            out.append((-b / a - t_on) * 1e3)
    return np.array(out)


def ramps(d, m):
    """Changes of >= 5 rad/s: the slope over 20-80 %, the mean current on it, time to 90 %, overshoot."""
    t, c, w, i = d["t"], d[f"c{m}"], d[f"w{m}"], d[f"i{m}"]
    out = []
    for k in range(2, len(c)):
        if not (abs(c[k] - c[k - 1]) > 0.5 and abs(c[k - 1] - c[k - 2]) < 0.5):
            continue
        c0, c1 = c[k - 1], c[min(k + 20, len(c) - 1)]
        if abs(c1 - c0) < 5:
            continue
        seg = slice(k, min(k + 120, len(t)))
        f = (w[seg] - c0) / (c1 - c0)
        sel = (f > 0.2) & (f < 0.8)
        if sel.sum() < 3:
            continue
        a = np.polyfit(t[seg][sel], w[seg][sel], 1)[0]
        out.append(dict(c0=float(c0), c1=float(c1), accel=float(a), i_ramp=float(np.mean(i[seg][sel])),
                        t90=float(t[seg][np.argmax(f > 0.9)] - t[k]),
                        overshoot=float(np.max(np.sign(c1 - c0) * (w[seg] - c1)))))
    return out


def j_over_k(rs):
    """J/k (A s^2/rad) from the ramps that don't cross zero: i = f sign(w) + (J/k) a, least squares in (f, J/k)."""
    rows = [(np.sign(r["c0"] + r["c1"]), r["accel"], r["i_ramp"]) for r in rs if r["c0"] * r["c1"] >= 0]
    if len(rows) < 2:
        return float("nan")
    s, a, i = np.array(rows).T
    return float(np.linalg.lstsq(np.column_stack([s, a]), i, rcond=None)[0][1])


def holds(d, m):
    t, c, w, i = d["t"], d[f"c{m}"], d[f"w{m}"], d[f"i{m}"]
    out, s = [], 0
    for k in range(1, len(c) + 1):
        if k == len(c) or abs(c[k] - c[s]) > 0.05:
            if t[k - 1] - t[s] > 0.8 and abs(c[s]) > 0.5:
                late = t[s:k] > t[s] + 0.5
                out.append(dict(c=float(c[s]), w=float(np.mean(w[s:k][late])), w_sd=float(np.std(w[s:k][late])),
                                i=float(np.mean(i[s:k][late]))))
            s = k
    W = np.abs([h["w"] for h in out])
    I = np.abs([h["i"] for h in out])
    b, a = np.polyfit(W, I, 1)
    return dict(fric_a=float(a), fric_b=float(b), w_sd=float(np.mean([h["w_sd"] for h in out])),
                top=float(np.max(np.abs(w))), holds=out)


def replay_delay(path: Path, m: int, J: float) -> dict:
    """The sim's lag on this triangle capture, and the extra pure delay that fits the bench."""
    import wheel_bench as wb
    sp = wb.sim_params()
    cap = wb.load(path, m)
    t, c, w = cap["t"], cap["c"], cap["w"]

    def sim(extra):
        ts, W, _ = wb.simulate(t + extra, c, sp, J)
        return np.interp(t - FB_AGE, ts, W)       # the command's timeline shifted by extra: a pure delay

    fits = []
    for ex in np.arange(0, 0.012, 0.0005):
        ws = sim(ex)
        fits.append((ex, float(np.sqrt(np.mean((ws - w)[np.abs(c) > 1] ** 2)))))
    best = min(fits, key=lambda f: f[1])
    return dict(bench_lag_ms=lag_ms(t, c, w), sim_lag_ms=lag_ms(t, c, sim(0.0)), extra_ms=best[0] * 1e3,
                rms=best[1], starts_at_rest=bool(abs(c[0]) < 1e-3))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir")
    ap.add_argument("--out", default=str(HERE / "results" / "can_bench_2026-11-13.json"))
    ap.add_argument("--no-replay", action="store_true")
    a = ap.parse_args()
    D = Path(a.dir)
    res = {"timing": {p.name: timing(load(p)) for p in sorted(D.glob("*.csv"))}}
    idle = load(D / "idle.csv")
    res["idle"] = {m: dict(i_mean=float(idle[f"i{m}"].mean()), i_sd=float(idle[f"i{m}"].std()),
                           w_sd=float(idle[f"w{m}"].std())) for m in (1, 2)}
    for m in (1, 2):
        r = {}
        lg = onset_lags(load(D / f"lat_m{m}.csv"), m)
        r["onset_lag_ms"] = dict(n=len(lg), mean=float(lg.mean()), sd=float(lg.std()))
        tri = load(D / f"tri_m{m}.csv")
        r["tri_lag_ms"] = lag_ms(tri["t"], tri[f"c{m}"], tri[f"w{m}"])
        rs = ramps(load(D / f"steps_m{m}.csv"), m)
        r["ramps"] = rs
        r["accel_cap"] = float(np.mean([abs(x["accel"]) for x in rs]))
        r["j_over_k"] = j_over_k(rs)
        r["sweep"] = holds(load(D / f"sweep_m{m}.csv"), m)
        if not a.no_replay:
            r["replay"] = replay_delay(D / f"tri_m{m}.csv", m, r["j_over_k"] * KT)
        res[f"motor{m}"] = r
        print(f"motor {m}: onset lag {r['onset_lag_ms']['mean']:.1f} ms (n {len(lg)}), ramp lag {r['tri_lag_ms']:.1f} ms,"
              f" accel cap {r['accel_cap']:.0f} rad/s^2, J/k {r['j_over_k'] * 1e3:.2f} mA s^2/rad, friction "
              f"{r['sweep']['fric_a']:.3f} + {r['sweep']['fric_b']:.4f}|w| A, speed sd {r['sweep']['w_sd']:.3f}"
              + (f"; sim lag {r['replay']['sim_lag_ms']:.1f} ms, extra delay {r['replay']['extra_ms']:.1f} ms"
                 f"{'' if r['replay']['starts_at_rest'] else ' (capture starts mid-profile: fit invalid)'}"
                 if "replay" in r else ""))
    Path(a.out).write_text(json.dumps(res, indent=1))
    print("->", a.out)


if __name__ == "__main__":
    main()
