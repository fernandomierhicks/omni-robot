"""Step 2: frame rate, drops, jitter and capture-to-host latency, mode by mode (README.md).

For each mode it streams for --seconds and records every frame's clocks: mid-exposure on the host
clock (the camera's hardware timestamp in librealsense's global time domain, minus the transfer offset
in the metadata), host arrival (metadata time_of_arrival) and the moment Python's callback ran. The
catch needs capture -> detection <= 60 ms (design.md 4.9; the sims assume 50 ms); this is the part of it
up to the host. Also the achieved fps against the nominal one, frames lost (frame counter), interval
jitter, exposure, and this process's CPU (librealsense unpacks on the host).

    python rs_timing.py                       # the default set of modes
    python rs_timing.py --mode ir:848x480@90 --exposure-us 1000 --seconds 20
    python rs_timing.py --list                # the default modes
"""
from __future__ import annotations

import argparse
import os
import threading
import time

import numpy as np

import rs_common as C
from rs_common import rs

# The catch-relevant modes first: IR at >= 60 fps (the OV9282 pair = the design's OV9782 in mono),
# depth at 90, the 300 fps strip, colour at 60, and the robot-like combination
DEFAULT_MODES = [
    "ir:848x480@90",
    "stereo:848x480@90",
    "depth:848x480@90",
    "depth:848x480@90+ir:848x480@90",
    "ir:1280x800@30",
    "ir:640x480@90",
    "depth:848x100@300",
    "ir:848x100@300",
    "color:640x480@60",
    "color:848x480@60",
    "color:1280x720@30",
    "depth:848x480@90+color:640x480@60",
]


class Recorder:
    """Pipeline callback: stamps each frame the moment librealsense hands it over."""

    def __init__(self, touch: bool = False):
        self.rows = []
        self.lock = threading.Lock()
        self.touch = touch              # read every frame's pixels into numpy, like a detector would
        self.last = {}                  # stream -> last frame number seen

    def __call__(self, frame):
        t_host = C.host_ms()
        frames = list(frame.as_frameset()) if frame.is_frameset() else [frame]
        with self.lock:
            for f in frames:
                prof = f.get_profile()
                st = str(prof.stream_type()).split(".")[-1] + str(prof.stream_index() or "")
                # a synced frameset repeats the slower stream's last frame at the faster one's rate: count each once
                if self.last.get(st) == f.get_frame_number():
                    continue
                self.last[st] = f.get_frame_number()
                row = C.frame_times(f, t_host)
                row["stream"] = st
                if self.touch:
                    np.asanyarray(f.get_data()).max()
                    row["t_touched_ms"] = C.host_ms()
                self.rows.append(row)


def cpu_seconds() -> float:
    t = os.times()
    return t.user + t.system


def run_mode(dev, mode_text: str, seconds: float, warmup: float, exposure_us, emitter, touch: bool = False) -> dict:
    mode = C.parse_mode(mode_text)
    if not C.mode_supported(dev, mode):
        print(f"{mode_text}: not offered by this camera / firmware / link, skipped")
        return dict(mode=mode_text, skipped="not offered")
    cfg = rs.config()
    cfg.enable_device(C.info(dev, "serial_number"))
    C.enable_streams(cfg, mode)
    C.configure_stereo(dev, emitter=emitter, exposure_us=exposure_us)
    if exposure_us is not None and any(k == "color" for k, *_ in mode):
        cs = C.color_sensor(dev)
        C.set_opt(cs, "enable_auto_exposure", 0)
        C.set_opt(cs, "exposure", max(1, round(exposure_us / 100)))   # RGB exposure is in 100 us units
    rec = Recorder(touch)
    pipe = rs.pipeline()
    pipe.start(cfg, rec)
    time.sleep(warmup)
    with rec.lock:
        rec.rows.clear()
    c0, w0 = cpu_seconds(), time.time()
    time.sleep(seconds)
    c1, w1 = cpu_seconds(), time.time()
    pipe.stop()
    with rec.lock:
        rows = list(rec.rows)
    out = dict(mode=mode_text, seconds=seconds, exposure_us_set=exposure_us, emitter=emitter, touch=touch,
               cpu_cores=(c1 - c0) / (w1 - w0), streams={})
    kind_of = {"depth": "depth", "color": "color"}           # infrared1 / infrared2 come from ir / ir2 / stereo
    for st in sorted({r["stream"] for r in rows}):
        rr = [r for r in rows if r["stream"] == st]
        fps = next(f for k, w, h, f in mode if kind_of.get(k, "infrared") == kind_of.get(st, "infrared"))
        dom = rr[0]["domain"] if rr else None
        t_hw = [r["t_ms"] for r in rr]
        s = dict(frames=len(rr), domain=dom,
                 intervals_hw=C.interval_stats(t_hw, fps) if dom != "system_time" else None,
                 intervals_host=C.interval_stats([r["t_host_ms"] for r in rr], fps),
                 lost_by_counter=C.counter_gaps([r["counter"] for r in rr]),
                 exposure_us=C.stats([r["exposure_us"] for r in rr]))
        if dom == "global_time":
            s["latency_mid_exposure_to_python_ms"] = C.stats([r["t_host_ms"] - r["t_mid_ms"] for r in rr])
            s["latency_mid_exposure_to_arrival_ms"] = C.stats(
                [r["arrival_ms"] - r["t_mid_ms"] for r in rr if r["arrival_ms"] is not None])
            s["latency_arrival_to_python_ms"] = C.stats(
                [r["t_host_ms"] - r["arrival_ms"] for r in rr if r["arrival_ms"] is not None])
            if touch:
                s["latency_mid_exposure_to_pixels_ms"] = C.stats([r["t_touched_ms"] - r["t_mid_ms"] for r in rr])
        else:
            s["latency_note"] = ("no hardware timestamps (metadata off: enable_metadata.ps1) - only the host "
                                 "intervals are meaningful")
        s["achieved_fraction"] = (s["frames"] / (fps * seconds)) if seconds else None
        out["streams"][st] = s
    return out


def print_mode(r: dict):
    if "skipped" in r:
        return
    print(f"\n{r['mode']}  (CPU {r['cpu_cores']:.2f} cores of this PC)")
    for st, s in r["streams"].items():
        iv = s["intervals_hw"] or s["intervals_host"]
        lat = s.get("latency_mid_exposure_to_python_ms", {})
        ex = s["exposure_us"]
        print(f"  {st:10s} {s['frames']:5d} frames  {iv.get('fps') or 0:6.1f} fps  "
              f"jitter {iv.get('dt_std_ms', 0):.2f} ms  lost {s['lost_by_counter']}  "
              + (f"exposure {ex['median']:.0f} us  " if ex.get('n') else "")
              + (f"latency median {lat['median']:.1f} p95 {lat['p95']:.1f} max {lat['max']:.1f} ms"
                 if lat.get("n") else s.get("latency_note", ""))
              + (f"  to pixels median {s['latency_mid_exposure_to_pixels_ms']['median']:.1f} "
                 f"p95 {s['latency_mid_exposure_to_pixels_ms']['p95']:.1f} ms"
                 if s.get("latency_mid_exposure_to_pixels_ms", {}).get("n") else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial")
    ap.add_argument("--mode", action="append", help="stream mode(s), e.g. ir:848x480@90 (repeatable)")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--warmup", type=float, default=2.0)
    ap.add_argument("--exposure-us", type=float, default=None,
                    help="fixed IR exposure (auto-exposure off); short exposures freeze a 6 m/s ball: "
                         "1000 us = 6 mm of blur")
    ap.add_argument("--emitter", choices=["on", "off"], default=None)
    ap.add_argument("--touch", action="store_true",
                    help="read every frame's pixels into numpy in the callback (a detector's minimum load); "
                         "adds latency_mid_exposure_to_pixels_ms")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    if a.list:
        print("\n".join(DEFAULT_MODES))
        return
    dev = C.find_device(a.serial)
    if not C.usb3(dev):
        print("WARNING: " + C.USB2_HINT)
    emitter = None if a.emitter is None else a.emitter == "on"
    results = []
    for m in a.mode or DEFAULT_MODES:
        try:
            r = run_mode(dev, m, a.seconds, a.warmup, a.exposure_us, emitter, a.touch)
        except RuntimeError as e:
            print(f"{m}: {e}")
            r = dict(mode=m, error=str(e))
        print_mode(r)
        results.append(r)

    lat = [s["latency_mid_exposure_to_python_ms"]["p95"] for r in results for s in r.get("streams", {}).values()
           if s.get("latency_mid_exposure_to_python_ms", {}).get("n")]
    summary = dict(worst_p95_latency_ms=max(lat) if lat else None, budget_ms=C.REQ["latency_max_ms"])
    if lat:
        print(f"\ncapture -> host p95, worst mode: {max(lat):.1f} ms of the {C.REQ['latency_max_ms']:.0f} ms "
              "capture -> detection budget (detection and the robot's transport come on top)")
    C.save_result("timing", dict(modes=results, summary=summary, temperatures=C.temperatures(dev)), dev)


if __name__ == "__main__":
    main()
