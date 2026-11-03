"""From a throw analysis to the sim's camera model (design.md 8, rule 2: every bench measurement goes into the sim).

Reads results/analysis_<session>.json (analyze_throws.py) and writes results/vision_measured.json: the fields of
`VisionConfig` (simulation/v11 sil/vision.py) this camera sets, each with where it came from, plus the cameras.csv
values. `catch_world.measured_cameras()` loads it.

  latency_mean / latency_jitter   sensor stamp -> host (the recorder's stamps) + half the exposure (the stamp is the
                                  start of readout, the sim counts from mid-exposure) + the Jetson's processing
  drop_prob                       1 - the detector's rate given the fitted arc, where the ball is >= 4 px across
  false_prob                      wrong blobs per in-view frame
  exposure                        the recorded exposure
  edge_sigma                      the one that needs the sim: the sim jitters the colour threshold per PIXEL, which
                                  moves the blob's centroid far less than that, so the measured centroid noise (rms of
                                  the residuals to the fitted arc) is matched by running the sim's own blob model
                                  (catch_world.ball_pixels) on the same fitted arcs at several edge_sigma values through
                                  the same detector and the same fit, and interpolating
  rolling_readout                 the camera's frame readout (rolling shutter) when the session says so (est: the frame time)
Not measurable with one camera: calib_rot_deg, calib_trans, calib_f_rel (stereo), spin; they keep the sim's values.

  python fit_vision.py results/analysis_<session>.json [--write-db]
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

import arcfit                                                       # noqa: E402

PROCESSING_S = 0.005          # detector + filter on the robot computer (catch_world.WorldConfig.processing)
MIN_B_PX = 4.0
EDGE_GRID = (0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0)


def arcs_of(res: dict) -> list[arcfit.Arc]:
    out = []
    ball = None
    for t in res["throws"]:
        if not t.get("ok"):
            continue
        a = t["arc"]
        ball = ball or arcfit.Ball()
        out.append(arcfit.Arc(a["t0"], np.array(a["p0"]), np.array(a["v0"]), math.radians(a["phi_deg"]), math.radians(a["psi_deg"]),
                              ball, ok=True))
    return out


def sim_centroid_noise(arcs, cam: arcfit.Camera, ball: arcfit.Ball, vcfg, fps: float, seed: int = 0, max_arcs: int = 12):
    """The sim's blob model + the robot's detector + the arc fit, on these arcs: (rms_uv px, second-difference noise px,
    detection rate, size residual relative to the radius) for the VisionConfig `vcfg`."""
    from catch_world import ball_pixels
    from omni_catch.detector import DetectorConfig
    from omni_catch.fast_detector import FastDetector, available
    from omni_catch.frames import SparseFrame
    from omni_catch.detector import Detector
    from analyze_throws import second_difference_noise
    det = FastDetector(DetectorConfig(backend="native")) if available() else Detector(DetectorConfig())
    rng = np.random.default_rng(seed)
    rms, sds, bs, hit, tot = [], [], [], 0, 0
    for arc in arcs[:max_arcs]:
        ts = arc.t0 + np.arange(0, 3.0, 1 / fps)
        pos, vel = arc.states(ts)
        u, v, b = arcfit.sphere_px(pos, cam, ball.radius)
        sel = np.isfinite(u) & (u > b + 2) & (u < cam.width - b - 2) & (v > b + 2) & (v < cam.height - b - 2) & (b >= 1.5)
        rows = []
        for k in np.nonzero(sel)[0]:
            tot += 1
            res = ball_pixels(vcfg, rng, pos[k], vel[k], ball.radius, cam.f, cam.cx, cam.cy, cam.width, cam.height)
            if res is None:
                continue
            mask, u0, v0, _ = res
            iy, ix = np.nonzero(mask)
            fr = SparseFrame(cam.width, cam.height, (iy + v0) * cam.width + (ix + u0), seed=int(rng.integers(1 << 30)))
            d = det.detect(fr, (u[k], v[k], b[k]))
            if d is not None and math.hypot(d[0] - u[k], d[1] - v[k]) < 2.5 * b[k] + 4:
                hit += 1
                rows.append((ts[k], d[0], d[1], d[2]))
        if len(rows) >= 12:
            D = arcfit.Detections(*(np.array(c) for c in zip(*rows)))
            fit = arcfit.fit_arc(D, cam, ball, up=arcfit.up_in_camera(arc.phi, arc.psi), fit_gravity=False)
            if fit.ok:
                rms.append(fit.rms_uv)
                bs.append(fit.rms_b_rel)
                s = second_difference_noise(D)
                if s is not None:
                    sds.append(s)
    return (float(np.median(rms)) if rms else float("nan"), float(np.median(sds)) if sds else float("nan"),
            hit / max(tot, 1), float(np.nanmedian(bs)) if bs else float("nan"))


def calibrate_edge_sigma(real_rms: float, arcs, cam, ball, base_cfg, fps: float, grid=EDGE_GRID):
    """edge_sigma whose simulated centroid noise matches real_rms (linear interpolation on the grid), and the curve."""
    curve = []
    for e in grid:
        r, sd, rate, rb = sim_centroid_noise(arcs, cam, ball, replace(base_cfg, edge_sigma=float(e)), fps)
        curve.append(dict(edge_sigma=e, rms_uv=r, second_diff=sd, detect_rate=rate, rms_b_rel=rb))
    xs = np.array([c["rms_uv"] for c in curve])
    es = np.array([c["edge_sigma"] for c in curve])
    ok = np.isfinite(xs)
    if not ok.any():
        return None, curve
    xs, es = xs[ok], es[ok]
    order = np.argsort(xs)
    xs, es = xs[order], es[order]
    if real_rms <= xs[0]:
        return float(es[0]), curve
    if real_rms >= xs[-1]:
        return float(es[-1]), curve                # beyond the grid: the sim cannot reach this noise with edge jitter alone
    return float(np.interp(real_rms, xs, es)), curve


def fit(res: dict) -> dict:
    from catch_world import expected_cameras
    summ = res["summary"]
    sess = res["session_info"]
    cam_info = sess["camera"]
    fps = cam_info["fps"]
    cam = arcfit.Camera(cam_info.get("f_px") or cam_info.get("f_px_est") or (cam_info["height"] / 2) / math.tan(math.radians(cam_info["fovy_deg"]) / 2),
                        cam_info["width"] / 2, cam_info["height"] / 2, cam_info["width"], cam_info["height"])
    ball = arcfit.Ball()
    base = expected_cameras()
    prov: dict = {}
    out: dict = {}
    # exposure: from the throws' own metadata
    exps = []
    for t in res["throws"]:
        try:
            m = json.loads((Path(t["path"]) / "meta.json").read_text())
            e = m.get("exposure_us_median") or m.get("exposure_us")
            if e:
                exps.append(e)
        except OSError:
            pass
    exposure = (float(np.median(exps)) * 1e-6) if exps else (cam_info.get("exposure_us") or 0) * 1e-6
    if exposure > 0:
        out["exposure"] = round(exposure, 5)
        prov["exposure"] = f"recorded exposure, {len(exps)} throws"
    # latency
    lat = summ.get("sensor_to_host_ms")
    if lat:
        out["latency_mean"] = round(lat["median"] * 1e-3 + exposure / 2 + PROCESSING_S, 4)
        out["latency_jitter"] = round(max((lat["p95"] - lat["median"]) / 1.645 * 1e-3, 0.0005), 5)
        prov["latency_mean"] = "sensor stamp -> host (median) + exposure/2 + 5 ms processing (est)"
        prov["latency_jitter"] = "(p95 - median) / 1.645 of sensor -> host"
    # drop / false from the detector given the arc
    rate_rows = [b for b in summ.get("detect_by_range", []) if b["hi"] <= cam.f * ball.radius / MIN_B_PX]
    fr = sum(b["frames"] for b in rate_rows)
    if fr:
        rate = sum(b["frames"] * b["rate"] for b in rate_rows) / fr
        out["drop_prob"] = round(max(0.0, 1 - rate), 4)
        prov["drop_prob"] = f"1 - detector rate given the arc, {fr} frames where the ball is >= {MIN_B_PX:g} px"
    dga = summ.get("detect_given_arc")
    if dga and dga["frames"]:
        out["false_prob"] = round(dga["wrong"] / dga["frames"], 4)
        prov["false_prob"] = "wrong blobs / in-view frames"
    if cam_info.get("shutter") == "rolling":
        out["rolling_readout"] = round(1.0 / fps, 5)
        prov["rolling_readout"] = "frame time (est: the IMX708's readout at this mode; measure with a spinning mark)"
    # edge_sigma: match the centroid noise through the sim's blob model
    arcs = arcs_of(res)
    real = summ.get("rms_uv_px", {}).get("median")
    curve = None
    size_note = None
    if real is not None and arcs:
        vc = replace(base, **{k: v for k, v in out.items() if k in ("exposure",)})
        e, curve = calibrate_edge_sigma(real, arcs, cam, ball, vc, fps)
        if e is not None:
            out["edge_sigma"] = round(e, 3)
            rb_sim = float(np.interp(e, [c["edge_sigma"] for c in curve], [c["rms_b_rel"] for c in curve]))
            rb_real = summ.get("rms_b_rel", {}).get("median")
            size_note = dict(real=rb_real, sim_at_this_edge_sigma=rb_sim)
            if rb_real and rb_sim > 1.5 * rb_real:
                size_note["warning"] = ("the sim's edge jitter that matches the centroid noise makes the ball's SIZE noisier than "
                                        "the real camera's: monocular tracks will look worse in the sim; the gate shows it")
            prov["edge_sigma"] = f"matches the measured centroid noise {real:.2f} px (median rms to the fitted arc) through catch_world.ball_pixels"
    return dict(session=res["session"], camera=cam_info.get("name"), time=res["time"], vision=out, provenance=prov,
                edge_curve=curve, size_noise_rel=size_note, centroid_noise_px=real, second_diff_px=summ.get("second_diff_px", {}).get("median"),
                not_measured="calib_rot_deg, calib_trans, calib_f_rel (stereo), spin, ball_size_rel: kept at the sim's values",
                cameras_csv=dict(meas_latency_ms=None if not lat else round(lat["median"] + 1e3 * exposure / 2, 1),
                                 meas_fps=fps, meas_notes=f"throws: {summ.get('fitted')}; centroid noise {real} px; drop {out.get('drop_prob')}"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("analysis", type=Path)
    ap.add_argument("--out", type=Path, default=HERE / "results" / "vision_measured.json")
    a = ap.parse_args()
    res = json.loads(a.analysis.read_text())
    out = fit(res)
    a.out.parent.mkdir(exist_ok=True)
    a.out.write_text(json.dumps(out, indent=1, default=float))
    print(json.dumps({k: out[k] for k in ("vision", "provenance", "centroid_noise_px", "size_noise_rel", "not_measured")}, indent=1, default=float))
    if out["edge_curve"]:
        print("edge_sigma -> simulated centroid noise (px):", ", ".join(f"{c['edge_sigma']:g}: {c['rms_uv']:.2f}" for c in out["edge_curve"]))
    print(f"-> {a.out}")


if __name__ == "__main__":
    main()
