"""Quick check (Nov 2 2026, Fernando): the AK40 robot with the fork pivot collinear with the axle (pivot_z 0, the same
80.6 mm arm; a parameter, not a drawn fork) against cad/v10's pivot 43 mm above the axle, at 10 and 20 kg (springs
matched to the mass). Uses drive_char's jobs; writes results/motor_char/pivot_quick.jsonl and prints a comparison.

  python pivot_quick.py [--workers N]
"""

from __future__ import annotations

import argparse
import collections
import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

import drive_char as D

PIVOTS = {"CAD 43 mm": 0.043, "collinear": 0.0}


def jobs():
    J = []
    for ms in ("10", "20"):
        for lab, pz in PIVOTS.items():
            c = D.Cfg(mass=ms, pivot_z=pz)
            J += [("accel", c, (dr, case)) for dr in (0.0, 45.0, 90.0, 180.0) for case in ("start", "brake")]
            J += [("cruise", c, (0.0, v)) for v in (1.0, 2.0, 3.0)]
            J += [("slope", c, (dr, "start")) for dr in (0.0, 90.0)]
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
    out = D.OUT / "pivot_quick.jsonl"
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
            if k % 10 == 0:
                print(f"  {k}/{len(J)} {(time.perf_counter() - w0) / 60:.1f} min", flush=True)
    report(rows)


def report(rows):
    lab = {v: k for k, v in PIVOTS.items()}
    g = collections.defaultdict(dict)
    for r in rows:
        if r["err"]:
            continue
        g[(r["cfg"]["mass"], r["kind"], json.dumps(r["args"]))][lab[r["cfg"]["pivot_z"]]] = r["res"]

    def cell(kind, x):
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
            return f"{'ok' if ok else 'FAIL'} tilt {x.get('tilt_max', 0):.0f} lift {x.get('lift_ms', 0):.0f}ms " \
                   f"bottom {x.get('bottom_ms', 0):.0f}ms"
        if kind == "ladder":
            return f"{x['max_mm']:g} mm"
        if kind == "straight":
            return f"{x['t_settle']:.2f} s (peak {x['v_peak']:.2f})" if x["t_settle"] else "not settled"
        if kind == "panic":
            return f"stop {x.get('stop_m', 0):.2f} m tilt {x.get('tilt_max', 0):.1f} lift {x.get('lift_ms', 0):.0f}ms"
        return "?"
    for (ms, kind, args), d in sorted(g.items()):
        print(f"{ms:>2} kg {kind:8s} {args:28s} | CAD: {cell(kind, d['CAD 43 mm']) if 'CAD 43 mm' in d else '-':48s} "
              f"| collinear: {cell(kind, d['collinear']) if 'collinear' in d else '-'}")


if __name__ == "__main__":
    main()
