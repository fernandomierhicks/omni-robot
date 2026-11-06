"""The tag bench's numbers per camera solution, in the terms of the sim's tag camera (simulation/v11 sil/tag_sim.py
TagCamera: corner noise sigma = 0.25 + 4 / side px, detection ramping from 12 to 32 px side, rate, misdecodes).
Reads results/tag_lock_*.json, or the runs named (the newest run of each configuration wins), and writes
results/sim_inputs.json (or --out).

  python sim_inputs.py [RUN.json ...] [--out results/sim_inputs.json]

Every number is measured on one still scene (README "Setup"): one range, one angle. Fields ending in _est are computed
from measured ones, not measured themselves.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOLUTIONS = {                       # the camera solution -> its bench configuration (decimate 2: the live default)
    "arducam_full_field_1152": "arducam_1152_d2",
    "arducam_full_field_2304": "arducam_2304_d2",
    "arducam_crop_768_100fps": "arducam_crop768_d1",
    "d435_ir_1280_projector_off": "d435_ir1280_d2",
    "d435_ir_848_60fps_projector_off": "d435_ir848_d1",
    "d435_ir_640_projector_off": "d435_ir640_d1",
    "d435_colour_1280": "d435_color1280_d2",
}
# the emulated reach ran only with full-resolution detection: where the solution's own run has none, this one's
REACH_FROM = {"arducam_1152_d2": "arducam_1152_d1", "d435_ir1280_d2": "d435_ir1280_d1"}


def sim_sigma(side: float) -> float:
    return 0.25 + 4.0 / side


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("runs", nargs="*", type=Path)
    ap.add_argument("--out", type=Path, default=HERE / "results" / "sim_inputs.json")
    a = ap.parse_args()
    runs = sorted(a.runs or (HERE / "results").glob("tag_lock_*.json"), key=lambda p: p.name)
    best: dict = {}
    for p in runs:                                       # oldest first: a newer run replaces an older one
        j = json.loads(p.read_text())
        for name, c in j["configs"].items():
            best[name] = (c, p.name)
    best_light = {p.name: json.loads(p.read_text()).get("light") for p in runs}
    out = {}
    for sol, name in SOLUTIONS.items():
        c, src = best[name]
        w, h, f = c["width"], c["height"], c["focal_px"]
        seconds = c["frames"] / c["processed_fps"]
        exp_us = c["exposure"].get("ir_exposure_us" if c["stream"] == "ir" else "exposure_us")
        rc = c if c.get("reach") else best.get(REACH_FROM.get(name, ""), ({}, None))[0]
        reach = [r for r in rc.get("reach", []) if r.get("rate", 0) >= 0.9 and "side_px" in r]
        out[sol] = dict(
            source=f"{src}: {name}", light=best_light.get(src), camera=c["camera"], stream=c["stream"], width=w, height=h, focal_px=f,
            hfov_deg=round(math.degrees(2 * math.atan(w / 2 / f)), 1), vfov_deg=round(math.degrees(2 * math.atan(h / 2 / f)), 1),
            camera_fps=round(c["camera_frames"] / seconds, 1),
            exposure_ms=None if exp_us is None else round(exp_us / 1000, 1), gain=c["exposure"].get("gain"),
            detector=f"{c['backend']}, decimate {c['decimate']:g}, 2 threads, Pi 4",
            detect_ms=c["detect_ms_median"], tag_rate_hz=c["processed_fps"],
            found=c["rate"], side_px=c.get("side_px"), range_m=c.get("range_mean_m"), off_axis_deg=c.get("angle_deg"),
            margin=c.get("margin_median"), gate_ok=c.get("gate_ok"),
            corner_sigma_px=c.get("corner_rms_px"), sim_corner_sigma_px=c.get("sim_corner_sigma_px"),
            corner_sigma_vs_sim=round(c["corner_rms_px"] / c["sim_corner_sigma_px"], 2) if c.get("corner_rms_px") else None,
            range_sigma_mm=c.get("range_std_mm"), normal_sigma_deg=c.get("normal_std_deg"),
            range_vs_tape=c.get("range_vs_tape"),
            misread_per_frame=round(c["misreads"] / c["frames"], 4), misreads_passing_gates=c.get("misreads_passing_gates"),
            min_side_px_emulated=min((r["side_px"] for r in reach), default=None),
            min_side_from=None if not reach else f"{'this run' if rc is c else REACH_FROM[name]} (frames shrunk, decimate 1: optimistic)",
            # a point at the image centre smears f * yaw rate * exposure px while the robot turns (est, not measured)
            blur_px_per_rad_s_est=None if exp_us is None else round(f * exp_us / 1e6, 1),
            # the noise in the SLAM's terms (tag_lock.covariance): corners' covariance, frame-to-frame correlation,
            # the 6-DoF pose covariance (camera frame, m and rad) and what the corner noise predicts for it
            cov=c.get("cov"),
        )
    p = a.out
    p.write_text(json.dumps(dict(note=__doc__.strip().splitlines()[0], solutions=out), indent=1))
    for k, v in out.items():
        print(f"{k:32s} {v['width']}x{v['height']} {v['hfov_deg']}x{v['vfov_deg']} deg  {v['camera_fps']} fps  exp {v['exposure_ms']} ms  "
              f"tags {v['tag_rate_hz']} Hz ({v['detect_ms']} ms)  side {v['side_px']}  sigma {v['corner_sigma_px']} (sim {v['sim_corner_sigma_px']}, x{v['corner_sigma_vs_sim']})  "
              f"range +-{v['range_sigma_mm']} mm  normal +-{v['normal_sigma_deg']}  margin {v['margin']}  gates {v['gate_ok']}  "
              f"misread {v['misread_per_frame']}  min side {v['min_side_px_emulated']}  blur {v['blur_px_per_rad_s_est']} px/(rad/s)")
        c = v["cov"]
        if c:
            print(f"{'':32s} cov: corners u {c['corner_sigma_u_px']} v {c['corner_sigma_v_px']} px, shared {c['corner_common_share']} "
                  f"(0.25 = independent), corr {c['corner_corr_same_axis']}, frame r {c['corner_lag1']}, "
                  f"independent {c['independent_share']}; pose {c['pose_sigma']}; along/across ray {c['along_ray_mm']}/"
                  f"{c['across_ray_mm']} mm; measured/predicted {c['measured_over_predicted']}")
    print(f"-> {p}")


if __name__ == "__main__":
    main()
