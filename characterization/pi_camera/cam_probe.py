"""Capture probe for the camera-head prototype (Pi 4 + Camera Module 3, design.md 4.9, phase 1C step 3 rehearsed).

Captures at the sensor's 120 fps mode (1536x864, 2x2 binned crop) with Picamera2 and measures, per frame:
  - the frame rate and drops, from the sensor's own timestamps (libcamera SensorTimestamp: start of readout);
  - sensor -> Python: SensorTimestamp to the moment the frame is in a numpy array here (CLOCK_MONOTONIC both);
  - the robot's own blob detector (omni_catch FastDetector, the C++ pixel loops; acquisition = the whole frame,
    no prediction, its worst case) and so sensor -> detection.
This is not the robot's camera (rolling shutter, no trigger, mono): it measures the Pi's capture path and the
detector's cost on ARM. A Pi 5 is ~2-3x faster per core (est). Results go to results/ as JSON.

On the Pi:  bash characterization/pi_camera/pi.sh run ~/.venvs/cam/bin/python pi_camera/cam_probe.py --seconds 10
"""
from __future__ import annotations

import argparse
import json
import os
import resource
import statistics as st
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
for p in (HERE.parent / "src" / "omni_catch", HERE.parents[1] / "ros" / "src" / "omni_catch"):   # deployed / repo
    if p.is_dir():
        sys.path.insert(0, str(p))

import numpy as np                                               # noqa: E402
from omni_catch.detector import DetectorConfig                   # noqa: E402
from omni_catch.fast_detector import FastDetector                # noqa: E402
from omni_catch.frames import ArrayFrame                         # noqa: E402

SENSOR_MODE = (1536, 864)          # the IMX708's 120 fps mode


def pct(x, q):
    return float(np.percentile(np.asarray(x), q)) if len(x) else float("nan")


def governor():
    try:
        return Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").read_text().strip()
    except OSError:
        return None


def summary(name, x, unit="ms"):
    return f"{name}: median {pct(x, 50):.2f} {unit}, p95 {pct(x, 95):.2f}, max {max(x):.2f}" if x else f"{name}: -"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--size", default="768x432", help="output size the ISP scales to (default 768x432)")
    ap.add_argument("--fps", type=float, default=120.0)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--exposure-us", type=int, default=0, help="fixed exposure (0: auto, capped by the frame time)")
    ap.add_argument("--no-detect", action="store_true", help="capture only")
    ap.add_argument("--buffers", type=int, default=6)
    ap.add_argument("--out", default=str(HERE / "results"))
    a = ap.parse_args()

    from picamera2 import Picamera2                              # here: the module needs the camera stack
    w, h = (int(v) for v in a.size.lower().split("x"))
    frame_us = int(round(1e6 / a.fps))
    controls = {"FrameDurationLimits": (frame_us, frame_us)}
    if a.exposure_us:
        controls.update(AeEnable=False, ExposureTime=a.exposure_us, AnalogueGain=4.0)
    cam = Picamera2()
    # BGR888 in libcamera's naming is R, G, B in memory: what the detector expects
    cfg = cam.create_video_configuration(main={"size": (w, h), "format": "BGR888"},
                                         raw={"size": SENSOR_MODE}, controls=controls, buffer_count=a.buffers,
                                         queue=False)
    cam.configure(cfg)
    det = None if a.no_detect else FastDetector(DetectorConfig(backend="native"))
    cam.start()
    time.sleep(1.0)                                              # let AE / AWB settle

    ts, to_py, det_ms, to_det, hits, exp_us = [], [], [], [], 0, []
    cpu0, t_end = time.process_time(), time.monotonic() + a.seconds
    while time.monotonic() < t_end:
        req = cam.capture_request()
        try:
            md = req.get_metadata()
            img = req.make_array("main")
        finally:
            req.release()
        t_py = time.monotonic_ns()
        sts = md.get("SensorTimestamp")
        ts.append(sts)
        to_py.append((t_py - sts) / 1e6)
        exp_us.append(md.get("ExposureTime", 0))
        if det is not None:
            t0 = time.perf_counter()
            r = det.detect(ArrayFrame(img))
            det_ms.append((time.perf_counter() - t0) * 1e3)
            to_det.append((time.monotonic_ns() - sts) / 1e6)
            hits += r is not None
    cpu = time.process_time() - cpu0
    cam.stop()
    cam.close()

    d = np.diff(np.asarray(ts, dtype=np.int64)) / 1e6
    nominal = 1e3 / a.fps
    dropped = int(sum(max(0, round(x / nominal) - 1) for x in d))
    span = (ts[-1] - ts[0]) / 1e9
    res = {
        "test": "cam_probe", "time": time.strftime("%Y-%m-%dT%H:%M:%S"), "host": os.uname().nodename,
        "machine": os.uname().machine, "sensor": "imx708", "sensor_mode": SENSOR_MODE, "output": [w, h],
        "fps_requested": a.fps, "fps_achieved": round((len(ts) - 1) / span, 2), "frames": len(ts),
        "dropped": dropped, "interval_ms": {"median": pct(d, 50), "p95": pct(d, 95), "max": float(d.max())},
        "sensor_to_python_ms": {"median": pct(to_py, 50), "p95": pct(to_py, 95), "max": max(to_py)},
        "detect_ms": {"median": pct(det_ms, 50), "p95": pct(det_ms, 95), "max": max(det_ms)} if det_ms else None,
        "sensor_to_detection_ms": {"median": pct(to_det, 50), "p95": pct(to_det, 95), "max": max(to_det)}
        if to_det else None,
        "frames_with_ball": hits, "exposure_us_median": st.median(exp_us),
        "cpu_cores": round(cpu / a.seconds, 3), "max_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
        "governor": governor(),
        "notes": "SensorTimestamp = start of readout (rolling shutter); latency excludes exposure and transport",
    }
    print(f"{res['frames']} frames, {res['fps_achieved']} fps of {a.fps:g}, dropped {dropped}; "
          f"exposure {res['exposure_us_median']} us; CPU {res['cpu_cores']} cores ({res['governor']})")
    print(summary("interval", list(d)))
    print(summary("sensor -> python", to_py))
    if det_ms:
        print(summary("detector", det_ms))
        print(summary("sensor -> detection", to_det) + f"; ball in {hits} frames")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    f = out / f"cam_probe_{w}x{h}_{int(a.fps)}_{time.strftime('%Y%m%d_%H%M%S')}.json"
    f.write_text(json.dumps(res, indent=1))
    print(f"-> {f}")


if __name__ == "__main__":
    main()
