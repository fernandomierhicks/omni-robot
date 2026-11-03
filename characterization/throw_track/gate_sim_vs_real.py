"""The gate (design.md 8, rule 2): do the sim's cameras, set from the measurements, give what the real camera gave?

Takes a session's analysis (analyze_throws.py) and the fitted camera model (fit_vision.py). For every real throw it
rebuilds the same flight (the fitted arc, at the real frames' times), makes the sim's frames of it with the sim's own
blob model (catch_world.ball_pixels: exposure blur, edge_sigma jitter, drop_prob) and runs them through the SAME
analysis: the robot's perception, the arc fit, the tracker's crossings. Then it puts the two side by side:

  centroid noise (rms to the arc)      matched by fit_vision, so this one is a check of that step
  detection rate, track start range    the sim's blobs vs the real detector
  crossing error by time to go         the tracker on simulated vs real detections of the same flights

"Pass" = the sim's number within a factor of 1.5 of the real one (centroid noise, rates within 0.05); a lot of
throws are needed before small differences mean anything (30+), so with fewer it says so. A failure means the sim's
camera model leaves something out (blur, size noise, a gap in the detections): fix the model, not the measurement.

  python gate_sim_vs_real.py results/analysis_<session>.json [--vision results/vision_measured.json]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "ros" / "src" / "omni_catch"))
sys.path.insert(0, str(REPO / "simulation" / "v11 sil"))

import analyze_throws as A                                           # noqa: E402
import arcfit                                                        # noqa: E402
import fit_vision as FV                                              # noqa: E402
import session as S                                                  # noqa: E402


def sim_throw(real: S.Throw, arc: arcfit.Arc, cam: arcfit.Camera, ball: arcfit.Ball, vcfg, seed: int) -> S.Throw:
    """The real throw's timeline with the sim's frames of the fitted flight."""
    from catch_world import ball_pixels
    from omni_catch.frames import SparseFrame
    rng = np.random.default_rng(seed)
    t = real.t
    frames = []
    k0 = np.nonzero(t >= arc.t0 - 1e-9)[0]
    pos = vel = None
    if len(k0):
        pos, vel = arc.states(t[k0])
    j = 0
    for i in range(len(t)):
        idx = np.zeros(0, np.int64)
        if len(k0) and i >= k0[0]:
            if rng.uniform() >= vcfg.drop_prob:
                res = ball_pixels(vcfg, rng, pos[j], vel[j], ball.radius, cam.f, cam.cx, cam.cy, cam.width, cam.height)
                if res is not None:
                    mask, u0, v0, _ = res
                    iy, ix = np.nonzero(mask)
                    idx = (iy + v0) * cam.width + (ix + u0)
            j += 1
        frames.append(SparseFrame(cam.width, cam.height, idx, seed=int(rng.integers(1 << 30))))
    return S.Throw(real.path, frames, real.stamps, real.meta, real.session)


def compare(real_sum: dict, sim_sum: dict) -> list[dict]:
    rows = []

    def add(name, r, s, tol, kind="ratio"):
        if r is None or s is None or (isinstance(r, float) and not np.isfinite(r)) or (isinstance(s, float) and not np.isfinite(s)):
            return
        ok = (abs(s - r) <= tol) if kind == "abs" else (max(r, 1e-9) / tol <= s <= max(r, 1e-9) * tol)
        rows.append(dict(metric=name, real=r, sim=s, tol=tol, kind=kind, ok=bool(ok)))

    add("centroid noise rms (px)", real_sum.get("rms_uv_px", {}).get("median"), sim_sum.get("rms_uv_px", {}).get("median"), 1.5)
    add("second-difference noise (px)", real_sum.get("second_diff_px", {}).get("median"), sim_sum.get("second_diff_px", {}).get("median"), 1.5)
    add("detection rate given the arc", real_sum.get("detect_given_arc", {}).get("rate"), sim_sum.get("detect_given_arc", {}).get("rate"), 0.05, "abs")
    rl, sl = real_sum.get("live", {}), sim_sum.get("live", {})
    add("track-ready range (m)", rl.get("track_ready_range_m", {}).get("median"), sl.get("track_ready_range_m", {}).get("median"), 1.5)
    add("throws with a ready track", rl.get("throws_tracked"), sl.get("throws_tracked"), 0.25 * max(real_sum.get("fitted", 1), 1), "abs")
    for k, rv in (rl.get("crossing_error_by_ttg") or {}).items():
        sv = (sl.get("crossing_error_by_ttg") or {}).get(k)
        if sv and rv["n"] >= 3 and sv["n"] >= 3:
            add(f"crossing error median at {k} s to go (m)", rv["median_m"], sv["median_m"], 2.0)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("analysis", type=Path)
    ap.add_argument("--vision", type=Path, default=HERE / "results" / "vision_measured.json")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    res = json.loads(a.analysis.read_text())
    vis = json.loads(a.vision.read_text())
    from catch_world import measured_cameras
    vcfg = measured_cameras(a.vision)
    sess = res["session_info"]
    cam, ball = A.camera_of(sess), A.ball_of(sess)
    ups = np.array(res["up_in_camera"]) if res.get("up_in_camera") else None
    up_sigma = None
    sim_results = []
    n_fit = sum(1 for t in res["throws"] if t.get("ok"))
    if ups is not None:
        up_sigma = max(math.radians(0.05), math.radians(0.4) / math.sqrt(max(n_fit, 1)))
    for k, t in enumerate(res["throws"]):
        if not t.get("ok"):
            continue
        real = S.load_throw(Path(t["path"]), sess)
        ar = t["arc"]
        arc = arcfit.Arc(ar["t0"], np.array(ar["p0"]), np.array(ar["v0"]), math.radians(ar["phi_deg"]), math.radians(ar["psi_deg"]), ball, ok=True)
        st = sim_throw(real, arc, cam, ball, vcfg, a.seed + k)
        r = A.analyse_throw(st, sess, ups, live=True, up_sigma=up_sigma)
        sim_results.append(r)
        print(f"  {Path(t['path']).name}: sim fit rms {r.get('rms_uv', float('nan')):.2f} px (real {t['rms_uv']:.2f})")
    sim_sum = A.pool(sim_results, sess)
    rows = compare(res["summary"], sim_sum)
    print(f"\n{'metric':<42}{'real':>10}{'sim':>10}   verdict")
    for r in rows:
        print(f"{r['metric']:<42}{r['real']:>10.3f}{r['sim']:>10.3f}   {'ok' if r['ok'] else 'DIFFERENT'}")
    bad = [r for r in rows if not r["ok"]]
    few = n_fit < 30
    print("\nGATE:", "PASS" if not bad else f"CHECK ({len(bad)} differ: {', '.join(r['metric'] for r in bad)})",
          f"-- only {n_fit} throws: differences inside the noise of this many throws prove nothing" if few else "")
    out = dict(session=res["session"], vision=vis["vision"], rows=rows, real=res["summary"], sim=sim_sum, throws=n_fit, passed=not bad)
    f = a.analysis.parent / f"gate_{res['session']}.json"
    f.write_text(json.dumps(out, indent=1, default=float))
    print(f"-> {f}")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
