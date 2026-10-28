"""The ball detector in Python (numpy + scipy) against its C++ pixel loops (omni_catch.fast_detector), on the PC
(design.md 7: "a C++ or GPU detector, timed on the PC"): the same blobs, and how long each takes per camera frame.

Frames: the sim's sparse frames over real throws (catch_world), synthetic 1280 x 800 frames full of ball-coloured
clutter (the worst case for acquisition), and with --rendered the furnished house rendered through the camera model
(Windows only: MuJoCo doesn't draw in WSL). For each frame: tracking (a window around the true ball), a full search
(no prediction) and acquisition (40 candidates). Writes results/detector_speed.json.

    python detector_speed.py [--rendered] [--frames 60]
"""

from __future__ import annotations

import argparse
import json
import math
import platform
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from catch_world import CatchWorld, WorldConfig  # noqa: E402
from sil_game import robot_frame  # noqa: E402  (ros/src on the path)
from omni_catch.detector import Detector, DetectorConfig  # noqa: E402
from omni_catch.fast_detector import FastDetector, available  # noqa: E402
from omni_catch.frames import BALL_RGB, ArrayFrame  # noqa: E402


def world_frames(frames: str, n: int, seed: int = 9000):
    """(frame, truth (u, v, r) or None) from a throw, every camera frame while the ball flies."""
    w = CatchWorld(WorldConfig(seed=seed, frames=frames, scene="house" if frames == "rendered" else "arena"))
    home = np.array(w.p.home_xy)
    w.place_ball((home[0] - 5.5, home[1] + 0.2, 1.2), (6.5, -0.1, 3.2))
    out = []
    while len(out) < n and w.d.time < 3.0:
        w.robot.step()
        if w.d.time >= w.t_frame - 1e-9:
            w.t_frame = w.d.time + 1.0 / w.cfg.vision.fps
            for f in w.cams.frame_pair(w.d.time):
                if f.truth is not None and f.truth[3]:
                    out.append((robot_frame(f), f.truth[:3]))
    w.close()
    return out[:n]


def clutter_frames(n: int, seed: int = 1):
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:800, 0:1280]
    out = []
    for _ in range(n):
        img = rng.integers(60, 140, (800, 1280, 3), dtype=np.uint8)
        truth = None
        for k in range(40):
            u, v, r = rng.uniform(0, 1280), rng.uniform(0, 800), rng.uniform(2, 25)
            m = (xx + 0.5 - u) ** 2 + (yy + 0.5 - v) ** 2 < r * r
            img[m] = (np.array(BALL_RGB) * rng.uniform(0.6, 1.0, (int(m.sum()), 1))).clip(0, 255).astype(np.uint8)
            truth = truth or (u, v, r)
        out.append((ArrayFrame(img), truth))
    return out


def time_calls(det, frames, reps=3):
    calls = {"tracking": lambda f, t: det.detect(f, pred=(t[0] + 2.0, t[1] - 1.0, t[2])) if t else None,
             "search": lambda f, t: det.detect(f), "acquire_40": lambda f, t: det.candidates(f, 40)}
    out, results = {}, {}
    for name, fn in calls.items():
        res = [fn(f, t) for f, t in frames]                  # once for the results (and warm caches)
        t0 = time.perf_counter()
        for _ in range(reps):
            for f, t in frames:
                fn(f, t)
        out[name] = 1000 * (time.perf_counter() - t0) / (reps * len(frames))
        results[name] = res
    return out, results


def same(a, b) -> bool:
    if a is None or b is None:
        return a is b
    return np.allclose(np.asarray(a, dtype=float), np.asarray(b, dtype=float), atol=1e-6) if len(a) == len(b) else False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rendered", action="store_true")
    ap.add_argument("--frames", type=int, default=60)
    a = ap.parse_args()
    if not available():
        raise SystemExit("the native detector didn't build")
    sets = {"sparse (sim)": world_frames("sparse", a.frames), "clutter (synthetic)": clutter_frames(max(6, a.frames // 6))}
    if a.rendered:
        sets["rendered house"] = world_frames("rendered", a.frames)
    cfg = DetectorConfig()
    report = dict(machine=platform.processor() or platform.machine(), python=platform.python_version(), sets={})
    for name, frames in sets.items():
        tp, rp = time_calls(Detector(cfg), frames)
        tn, rn = time_calls(FastDetector(cfg), frames, reps=6)
        agree = {k: sum(same(x, y) for x, y in zip(rp[k], rn[k])) / len(frames) for k in rp}
        report["sets"][name] = dict(frames=len(frames), python_ms={k: round(v, 3) for k, v in tp.items()},
                                    native_ms={k: round(v, 3) for k, v in tn.items()},
                                    speedup={k: round(tp[k] / tn[k], 1) for k in tp}, same=agree)
        print(f"{name}: {len(frames)} frames")
        for k in tp:
            print(f"  {k:11s} python {tp[k]:7.2f} ms   native {tn[k]:6.3f} ms   x{tp[k] / tn[k]:5.1f}   "
                  f"same blobs {100 * agree[k]:.0f}%")
    out = HERE / "results" / "detector_speed.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=1))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
