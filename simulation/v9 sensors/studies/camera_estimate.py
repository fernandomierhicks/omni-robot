"""What will the real OV9782 stereo pair on a Jetson Orin do? Researched/extrapolated settings, not guesses per knob.

vision_sweep.py changes one knob at a time from an assumed baseline. Here each scenario is a whole, consistent
pipeline built from published numbers (design.md section 21, "Expected camera performance"):

Sensor (OmniVision OV9782 product brief; Luxonis OV9782 docs): 1/4" color, 1280x800 at 120 fps (VGA 180 fps),
3.0 um pixels, global shutter, frame-sync input, 2-lane MIPI. Some Arducam module listings give only 60 fps at full
resolution, so the frame rate is driver-dependent: a 60 fps scenario is included.

Capture -> filter latency, built up (all at 120 fps, 8.3 ms frames):
  exposure midpoint + global-shutter readout   ~0.5-2 + ~7 ms   (readout ~ one frame time at max fps)
  MIPI -> VI -> memory, frame-end              ~1 ms
  ISP (Argus / nvarguscamerasrc)               +2-3 frames buffered: +17-25 ms (NVIDIA forum: "2-3 frames is
                                               expected"; Xavier 48-60 ms capture stamp -> app)
  raw V4L2 + low_latency_mode (no ISP)         measured 15 ms glass-to-software at 120 fps (OV2311 on a Nano);
                                               debayer/threshold the ROI on the raw Bayer frame yourself
  blob detection (ROI), stereo pairing, EKF    ~2-4 ms
  Linux scheduling jitter, CAN to the MCU      ~2-5 ms
  => raw V4L2 path ~22-26 ms; Argus path ~40-50 ms; GStreamer appsink + OpenCV conversions 60-90 ms.
  For reference, RidgeRun measures 36-42 ms glass-to-GLASS (incl. display) on an AGX Orin with a raw path, and
  73-116 ms with Argus on TX2/Xavier.

Sync: both sensors triggered on their frame-sync pins from one MCU timer -> ~us offset, and the trigger edge gives
a hardware timestamp in the MCU's clock. Free-running "software sync" is 1-5 ms.

Calibration: a good checkerboard stereo calibration reaches 0.2-0.3 px RMS (0.02-0.03 deg at f = 571 px). The
v5 cliff is drift after that: PLA/PETG brackets creep and the module takes ball impacts. 0.04 deg = metal/carbon
bar; 0.1 deg = printed bracket, recalibrated per session; 0.3 deg = printed and knocked about.

Ball spin is a property of the throws, not the cameras: an underhand lob spins ~5-15 rad/s (est). 6 rad/s per
axis (|w| ~ 10 rad/s) is used in every realistic scenario; one row has none, to separate the two.

Grid and robots as catch_sweep.py (216 identical throws). Writes results/camera_estimate.json.
"""

from __future__ import annotations

import itertools
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import mujoco
import numpy as np

from common import RESULTS, SimParams, save  # noqa: E402
from catch_game import CatchGame, GameConfig  # noqa: E402
from robot_model import build_model  # noqa: E402
from vision import VisionConfig  # noqa: E402

DISTANCES = (1.5, 2.25, 3.0)
APEXES = (0.5, 1.0, 2.0, 3.0)
OFFSETS = (0.3, 0.6, 1.0)
TOSSES = 6
HOME = (1.0, 0.0)

B = VisionConfig()
EXPECTED = replace(B, latency_mean=0.045, latency_jitter=0.010, calib_rot_deg=0.10, sync_error=0.0001,
                   edge_sigma=0.6, exposure=0.003, drop_prob=0.05, false_prob=0.01, ball_size_rel=0.015,
                   odom_drift=0.02, yaw_drift_deg=0.6)
BEST = replace(EXPECTED, latency_mean=0.024, latency_jitter=0.005, calib_rot_deg=0.04, sync_error=0.0,
               edge_sigma=0.5, exposure=0.002, drop_prob=0.03)
PESSIMISTIC = replace(EXPECTED, latency_mean=0.080, latency_jitter=0.020, calib_rot_deg=0.30, sync_error=0.003,
                      edge_sigma=0.8, exposure=0.004, drop_prob=0.10, false_prob=0.05, ball_size_rel=0.03,
                      odom_drift=0.03, yaw_drift_deg=1.0)
SPIN = 6.0

# name: (VisionConfig, toss spin, SimParams kwargs, GameConfig kwargs)
PHYS = dict(catch_mode="moving")
FW = dict(limits="firmware", catch_mode="moving")
SCENARIOS = {
    "v5 assumed baseline (50 ms, 0.02 deg, no spin)": (B, 0.0, {}, PHYS),
    "best: raw V4L2, FSIN trigger, metal bar": (BEST, SPIN, {}, PHYS),
    "expected: Argus ISP, FSIN trigger, printed bracket": (EXPECTED, SPIN, {}, PHYS),
    "expected, no spin": (EXPECTED, 0.0, {}, PHYS),
    "expected but raw V4L2 (24 ms)": (replace(EXPECTED, latency_mean=0.024, latency_jitter=0.005), SPIN, {}, PHYS),
    "expected but 60 fps (driver mode), 60 ms": (replace(EXPECTED, fps=60.0, latency_mean=0.060, latency_jitter=0.012),
                                                SPIN, {}, PHYS),
    "expected but calibration drifted to 0.3 deg": (replace(EXPECTED, calib_rot_deg=0.30), SPIN, {}, PHYS),
    "expected but software sync (3 ms)": (replace(EXPECTED, sync_error=0.003), SPIN, {}, PHYS),
    "pessimistic: GStreamer/OpenCV, software sync, knocked bracket": (PESSIMISTIC, SPIN, {}, PHYS),
    "best + 90 deg lenses": (BEST, SPIN, dict(camera_fovy_deg=90.0), PHYS),
    # the actual proposed design (v3b firmware limits, adult mode)
    "firmware adult mode, v5 assumed baseline": (B, 0.0, dict(limits="firmware", base_max_speed=2.0), FW),
    "firmware adult mode, expected": (EXPECTED, SPIN, dict(limits="firmware", base_max_speed=2.0), FW),
    "firmware adult mode, best": (BEST, SPIN, dict(limits="firmware", base_max_speed=2.0), FW),
}


def run_cell(args):
    name, dist, apex, off, seed = args
    vcfg, spin, pkw, gkw = SCENARIOS[name]
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0), **pkw)
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(catch_zone_radius=off, zone_mode="ring", apex_range=(apex, apex), return_throws=False,
                     wait_between=0.8, perception="vision", vision_cfg=vcfg, toss_spin=spin, **gkw)
    game = CatchGame(m, d, p, cfg=cfg, seed=seed, verbose=False)
    first, out = None, []
    while len(game.results) < TOSSES:
        game.step()
        if game.state == "flight" and first is None and game.perc.ready():
            first = d.time - game.t_release
        if len(game.results) > len(out):
            r = game.results[-1]
            rim_margin = p.funnel_rim_radius - p.ball_spec.radius
            why = "caught" if r.caught else ("reach" if not np.isfinite(r.pos_error) or r.pos_error > rim_margin
                                             else "bounce")
            out.append(dict(caught=bool(r.caught), why=why, pos_error=r.pos_error, first=first,
                            land_sigma=r.land_sigma, flight_time=r.flight_time))
            first = None
    return dict(scenario=name, distance=dist, apex=apex, offset=off, tosses=out)


def report(rows):
    print(f"{'scenario':66s} caught  reach/bounce  first est  offset(med)")
    out = {}
    for name in SCENARIOS:
        ts = [t for r in rows if r["scenario"] == name for t in r["tosses"]]
        if not ts:
            continue
        k = np.mean([t["caught"] for t in ts])
        rc, bo = sum(t["why"] == "reach" for t in ts), sum(t["why"] == "bounce" for t in ts)
        fe = np.nanmedian([t["first"] if t["first"] is not None else np.nan for t in ts])
        off = np.nanmedian([t["pos_error"] for t in ts if t["caught"]])
        by_off = {o: float(np.mean([t["caught"] for r in rows if r["scenario"] == name and r["offset"] == o
                                    for t in r["tosses"]])) for o in OFFSETS}
        out[name] = dict(n=len(ts), caught=float(k), reach=rc, bounce=bo, first=float(fe), offset=float(off),
                         by_offset=by_off)
        print(f"{name:66s} {100 * k:4.0f}%   {rc:3d}/{bo:<3d}      {fe:5.3f} s   {100 * off:4.1f} cm   "
              + "  ".join(f"{o:.1f} m: {100 * v:3.0f}%" for o, v in by_off.items()))
    return out


def main():
    only = sys.argv[1:]
    todo = [s for s in SCENARIOS if not only or s in only]
    cells = [(s, dd, a, o, j) for s in todo for j, (dd, a, o) in enumerate(itertools.product(DISTANCES, APEXES, OFFSETS))]
    with ProcessPoolExecutor(min(27, os.cpu_count() or 4)) as pool:
        rows = list(pool.map(run_cell, cells, chunksize=1))
    summary = report(rows)
    save("camera_estimate", dict(rows=rows, summary=summary))


if __name__ == "__main__":
    main()
