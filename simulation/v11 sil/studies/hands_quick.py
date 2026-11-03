"""Quick check (Nov 2 2026, Fernando: "is there a mobility benefit for the diagonal hands?"): the AK40 robot with the fork
pivot level with the axle (ARM's baseline) with cad/v10's opposite-hand pods on opposite diagonals against every pod the
same hand, at 10 and 20 kg (springs matched to the mass). The same jobs as pivot_quick.py, with acceleration / braking in
eight directions and slope starts in four (one-hand pods lose the robot's mirror symmetry). Writes
results/motor_char/hands_quick.jsonl and prints a comparison.

  python hands_quick.py [--workers N]
"""

from __future__ import annotations

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

import drive_char as D

HANDS = ("diagonal", "same")


def jobs():
    J = []
    for ms in ("10", "20"):
        for h in HANDS:
            c = D.Cfg(mass=ms, pivot_z=0.0, hands=h)
            J += [("accel", c, (dr, case)) for dr in (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0) for case in ("start", "brake")]
            J += [("cruise", c, (0.0, v)) for v in (1.0, 2.0, 3.0)]
            J += [("slope", c, (dr, "start")) for dr in (0.0, 90.0, 180.0, 270.0)]
            J += [("susp", c, (sc,)) for sc in ("rug 2 45", "rug 1 0", "step 20 1 45", "step 20 2 0", "step 30 1 0",
                                                "side 30 2", "bumpy 1", "bumpy 2")]
            J += [("ladder", c, ("step", 45.0, 1.0))]
            J += [("straight", c, (0.0, 15.2, cap, 5.0)) for cap in (2.0, float("inf"))]
            J += [("panic", c, (0.0, 2.0))]
    return J


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=D.WORKERS)
    a = ap.parse_args()
    J = jobs()
    out = D.OUT / "hands_quick.jsonl"
    w0 = time.perf_counter()
    rows = []
    with open(out, "w") as f, ProcessPoolExecutor(a.workers) as ex:
        for k, fu in enumerate(as_completed([ex.submit(D.run_job, j) for j in J]), 1):
            row = D._clean(fu.result())
            rows.append(row)
            f.write(json.dumps(row) + "\n")
            f.flush()
            if row["err"]:
                print("ERROR", row["kind"], row["args"], row["err"].splitlines()[-1], flush=True)
            if k % 20 == 0:
                print(f"  {k}/{len(J)} {(time.perf_counter() - w0) / 60:.1f} min", flush=True)
    report(rows)


def report(rows):
    import collections
    g = collections.defaultdict(dict)
    for r in rows:
        if not r["err"]:
            g[(r["cfg"]["mass"], r["kind"], json.dumps(r["args"]))][r["cfg"]["hands"]] = r["res"]
    for (ms, kind, args), d in sorted(g.items()):
        print(f"{ms:>2} kg {kind:8s} {args:28s} | diagonal: {fmt(kind, d.get('diagonal')):48s} | same: {fmt(kind, d.get('same'))}")


def fmt(kind, x):
    if x is None:
        return "-"
    if kind == "accel":
        return f"{x['a_max']:.1f} {x['cause']}"
    if kind == "cruise":
        return (f"{x['p_drive_W']:.0f} W, bounce {x['s_low_load_frac']:.2f}, fight {x['s_fight_mean']:.2f}"
                if x.get("reached") else f"not reached ({x['cause']})")
    if kind == "slope":
        ff = x["first_fail"]
        return f"{x['max_deg']:g} deg" + (f" ({ff['cause']})" if ff else "")
    if kind == "susp":
        ok = x.get("ok", x.get("crossed"))
        return f"{'ok' if ok else 'FAIL'} tilt {x.get('tilt_max', 0):.0f} lift {x.get('lift_ms', 0):.0f}ms bottom {x.get('bottom_ms', 0):.0f}ms"
    if kind == "ladder":
        return f"{x['max_mm']:g} mm"
    if kind == "straight":
        return f"{x['t_settle']:.2f} s (peak {x['v_peak']:.2f})" if x["t_settle"] else "not settled"
    if kind == "panic":
        return f"stop {x.get('stop_m', 0):.2f} m tilt {x.get('tilt_max', 0):.1f} lift {x.get('lift_ms', 0):.0f}ms"
    return "?"


if __name__ == "__main__":
    main()
