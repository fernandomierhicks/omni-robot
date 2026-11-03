"""Record thrown balls on the camera head (Pi + Arducam / Camera Module 3), for analyze_throws.py (design.md 8, 1C).

Runs the camera at the bench's 768x432 @ 120 fps with the robot's own detector as the trigger. A ring buffer keeps the
last `--pre` seconds; when a ball moves (a blob that was not there a frame earlier and has a neighbour within a few
pixels), it keeps `--post` seconds more and writes the throw to data/<session>/tNNN/ (session.py's format). Capture
never stops while a throw is written (the write takes seconds on an SD card). After each throw it asks whether the
ball went in the basket: y / n (s = discard the throw, q = finish).

numpy and the robot's detector only (the Pi has no scipy or OpenCV). No camera: `--synthetic` renders throws with
synth.py, in the same format, to check the whole chain.

  bash characterization/throw_track/tt.sh record --name garage --distance 4.5 --count 30
  (settings come from ball_view's saved picam.json, if any: tune the colour there first)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import platform
import sys
import threading
import time
from collections import deque
from dataclasses import asdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for p in (HERE.parent / "src" / "omni_catch", HERE.parents[1] / "ros" / "src" / "omni_catch", HERE.parent / "ros" / "src" / "omni_catch"):
    if p.is_dir():
        sys.path.insert(0, str(p))
sys.path.insert(0, str(HERE))

import session as S                                                  # noqa: E402
from omni_catch.detector import Detector, DetectorConfig             # noqa: E402
from omni_catch.frames import ArrayFrame                             # noqa: E402

SENSOR_MODE = (1536, 864)               # the IMX708's 120 fps mode, as cam_probe.py
CROP_V_MM, F_MM = 1728 * 1.4e-3, 4.74       # Camera Module 3 standard lens in that mode (bench_config.py; est)
SETTINGS = Path.home() / "omni_ws" / "ball_view_settings" / "picam.json"


def detector_config(a) -> tuple[DetectorConfig, dict]:
    """DetectorConfig from ball_view's saved settings (if any), then the command line."""
    cfg, cam = DetectorConfig(backend="native"), {}
    f = Path(a.settings).expanduser() if a.settings else SETTINGS
    if f.is_file():
        s = json.loads(f.read_text())
        d = s.get("detector", {})
        cfg.hue_deg = tuple(d.get("hue_deg", cfg.hue_deg))
        cfg.min_sat = d.get("min_sat", cfg.min_sat)
        cfg.min_val = d.get("min_val", cfg.min_val)
        cfg.min_blob_px = int(d.get("min_blob_px", cfg.min_blob_px))
        cfg.size_gain = d.get("size_gain", cfg.size_gain)
        cfg.coarse_stride = int(d.get("coarse_stride", cfg.coarse_stride))
        cam = s.get("camera", {})
        print(f"detector settings from {f} (saved {s.get('saved', '?')})")
    if a.hue:
        cfg.hue_deg = (a.hue[0], a.hue[1])
    if a.min_sat is not None:
        cfg.min_sat = a.min_sat
    if a.min_val is not None:
        cfg.min_val = a.min_val
    if a.size_gain is not None:
        cfg.size_gain = a.size_gain
    return cfg, cam


def make_detector(cfg: DetectorConfig):
    from omni_catch.fast_detector import FastDetector, available
    return FastDetector(cfg) if available() else Detector(cfg)


class Trigger:
    """A moving ball-coloured blob: one in this frame with no blob of the last frame within `still_px`, but with one
    within `max_step` px (a thrown ball moves a few to tens of pixels per frame at 120 fps; the yellow box doesn't)."""

    def __init__(self, det, still_px=1.0, min_step=1.5, max_step=120.0, need=4):
        self.det, self.still, self.min_step, self.max_step, self.need = det, still_px, min_step, max_step, need
        self.prev = []
        self.run = 0
        self.last_move_t = -1e9
        self.cell = 24.0
        self.counts: dict = {}              # (cell x, cell y) -> frames a blob sat in it while learning
        self.frames = 0
        self.mask: set = set()              # cells (and their neighbours) of static ball-coloured things: ignored

    def learn(self, frame) -> None:
        """Collect where blobs sit while the scene is empty (flowers, a red lamp); `freeze()` turns that into the mask."""
        self.frames += 1
        for c in self.det.candidates(ArrayFrame(frame), 12):
            k = (int(c[0] // self.cell), int(c[1] // self.cell))
            self.counts[k] = self.counts.get(k, 0) + 1

    def freeze(self, min_frac: float = 0.05) -> int:
        for (i, j), n in self.counts.items():
            if n >= min_frac * max(1, self.frames):
                self.mask.update((i + di, j + dj) for di in (-1, 0, 1) for dj in (-1, 0, 1))
        return len(self.mask)

    def update(self, frame, t_s: float) -> bool:
        cs = [(c[0], c[1], c[2]) for c in self.det.candidates(ArrayFrame(frame), 6)]
        if self.mask:
            cs = [c for c in cs if (int(c[0] // self.cell), int(c[1] // self.cell)) not in self.mask]
        moving = False
        for c in cs:
            d = [math.hypot(c[0] - p[0], c[1] - p[1]) for p in self.prev]
            if d and min(d) > self.min_step and min(d) < self.max_step and c[2] >= 2.0:
                moving = True
                break
        self.prev = cs
        self.run = self.run + 1 if moving else 0
        if moving:
            self.last_move_t = t_s
        return self.run >= self.need

    def idle(self, t_s: float, quiet: float = 1.0) -> bool:
        return t_s - self.last_move_t > quiet


class PiCamera:
    """Picamera2 frames as (rgb array, sensor_ns, host_ns, exposure_us, gain)."""

    def __init__(self, size, fps, exposure_us, gain, awb):
        from picamera2 import Picamera2
        w, h = size
        us = int(round(1e6 / fps))
        controls = {"FrameDurationLimits": (us, us)}
        if exposure_us:
            controls.update(AeEnable=False, ExposureTime=int(exposure_us), AnalogueGain=float(gain))
        if not awb:
            controls.update(AwbEnable=False)
        self.cam = Picamera2()
        cfg = self.cam.create_video_configuration(main={"size": (w, h), "format": "BGR888"}, raw={"size": SENSOR_MODE},
                                                  controls=controls, buffer_count=8, queue=False)
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(1.0)
        self.size, self.fps = size, fps

    def read(self):
        req = self.cam.capture_request()
        try:
            md = req.get_metadata()
            img = req.make_array("main").copy()          # R, G, B in memory (BGR888 in libcamera's naming)
        finally:
            req.release()
        return img, int(md.get("SensorTimestamp", 0)), time.monotonic_ns(), md.get("ExposureTime", 0), md.get("AnalogueGain", 0.0)

    def close(self):
        self.cam.stop()
        self.cam.close()


class SyntheticCamera:
    """synth.py throws on a loop, as if from the camera (frames come as fast as they render; stamps are virtual)."""

    def __init__(self, size, fps, gap_s=1.5):
        from synth import SynthConfig, default_throw, render_throw
        self.cfg = SynthConfig(width=size[0], height=size[1], fps=fps, fovy_deg=50, centroid_jitter=0.3)
        self._render = (default_throw, render_throw)
        self.k, self.t, self.queue, self.gap = 0, 0, deque(), gap_s
        self.size, self.fps = size, fps

    def _fill(self):
        default_throw, render_throw = self._render
        arc, T = default_throw(self.cfg, 4.5, seed=self.k)
        frames, _, _ = render_throw(self.cfg, arc, T, pre=self.gap, seed=self.k)      # the scene alone, then the throw
        self.queue.extend(frames)
        self.k += 1

    def read(self):
        if not self.queue:
            self._fill()
        img = self.queue.popleft()
        self.t += 1
        ns = 5_000_000_000 + int(self.t * 1e9 / self.fps)
        return np.ascontiguousarray(img), ns, time.monotonic_ns(), int(self.cfg.exposure * 1e6), 1.0

    def close(self):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--name", default="session", help="session name (data/<time>_<name>)")
    ap.add_argument("--distance", type=float, default=0.0, help="thrower distance, m (metadata)")
    ap.add_argument("--count", type=int, default=0, help="stop after this many saved throws (0: until q)")
    ap.add_argument("--size", default="768x432")
    ap.add_argument("--fps", type=float, default=120.0)
    ap.add_argument("--exposure-us", type=int, default=None, help="fixed exposure (default: ball_view's saved value, else auto)")
    ap.add_argument("--gain", type=float, default=None)
    ap.add_argument("--fovy", type=float, default=round(math.degrees(2 * math.atan(CROP_V_MM / 2 / F_MM)), 1),
                    help="vertical field of view of the output image, deg (est from the datasheet: calibrate!)")
    ap.add_argument("--pre", type=float, default=0.8, help="seconds kept before the trigger")
    ap.add_argument("--post", type=float, default=1.7, help="seconds kept after the trigger")
    ap.add_argument("--settings", help="ball_view settings json (default ~/omni_ws/ball_view_settings/picam.json)")
    ap.add_argument("--hue", type=float, nargs=2)
    ap.add_argument("--min-sat", type=float)
    ap.add_argument("--min-val", type=float)
    ap.add_argument("--size-gain", type=float)
    ap.add_argument("--rim-radius", type=float, default=0.165, help="basket rim radius, m")
    ap.add_argument("--tag-id", type=int, default=0)
    ap.add_argument("--tag-size", type=float, default=0.10, help="AprilTag side (black square), m")
    ap.add_argument("--tag-below-rim", type=float, default=0.10, help="tag centre below the rim plane, m (metadata for tag_pose.py)")
    ap.add_argument("--shutter-note", default="rolling")
    ap.add_argument("--synthetic", action="store_true", help="no camera: rendered throws")
    ap.add_argument("--no-prompt", action="store_true", help="don't ask for outcomes (label later with label_throws.py)")
    ap.add_argument("--out", default=str(S.DATA))
    ap.add_argument("--notes", default="")
    a = ap.parse_args()

    w, h = (int(v) for v in a.size.lower().split("x"))
    if a.synthetic:
        a.fovy = 50.0                                      # synth.py's camera
    dcfg, camset = detector_config(a)
    exposure_us = a.exposure_us if a.exposure_us is not None else int(camset.get("exposure_us", 0) or 0)
    gain = a.gain if a.gain is not None else float(camset.get("gain", 1.0) or 1.0)
    cam = SyntheticCamera((w, h), a.fps) if a.synthetic else PiCamera((w, h), a.fps, exposure_us, gain, camset.get("awb", True))
    det = make_detector(dcfg)
    trig = Trigger(det)
    d = S.new_session_dir(a.name, Path(a.out))
    f_px = (h / 2) / math.tan(math.radians(a.fovy) / 2)
    info = dict(camera=dict(name="synthetic" if a.synthetic else "arducam_imx708_module3", width=w, height=h, fps=a.fps,
                            fovy_deg=a.fovy, f_px=None, f_px_est=f_px, exposure_us=exposure_us, gain=gain,
                            shutter=a.shutter_note, fov_is_estimate=True),
                detector=asdict(dcfg), distance_m=a.distance, notes=a.notes, host=platform.node(),
                governor=Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").read_text().strip()
                if Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").exists() else None,
                basket=dict(rim_radius=a.rim_radius, tag_id=a.tag_id, tag_size_m=a.tag_size, tag_cam=None,
                            rim_center_from_tag=[0.0, a.rim_radius, a.tag_below_rim], rim_center_world=None),
                started=time.strftime("%Y-%m-%dT%H:%M:%S"))
    S.save_session(d, info)
    np.save(d / "tag.npy", cam.read()[0])              # the basket and its tag as they stand now (tag_pose.py)
    print(f"session {d}\n{w}x{h} @ {a.fps:g} fps, exposure {exposure_us or 'auto'} us, gain {gain}, fovy {a.fovy} deg (est)")

    pre_n, post_n = int(a.pre * a.fps), int(a.post * a.fps)
    ring: deque = deque(maxlen=pre_n)
    state = dict(armed=True, saved=0, quit=False)
    jobs: list[threading.Thread] = []
    pending: "queue.Queue" = __import__("queue").Queue()
    lock = threading.Lock()

    def dump(k, frames, stamps, meta):
        t0 = time.time()
        path = S.save_throw(d, k, np.stack(frames), np.array(stamps, dtype=np.int64), meta)
        pending.put((k, path, time.time() - t0, len(frames)))

    print("armed: throw when ready (Ctrl-C to finish)")
    post = None                      # frames collected after the trigger
    trig_idx = 0
    n_asked = 0
    t_prev = None
    try:
        while not state["quit"]:
            img, sts, hns, exp_us, gn = cam.read()
            t_s = sts * 1e-9
            if post is None:
                ring.append((img, sts, hns, exp_us, gn))
                if state["armed"] and len(ring) >= pre_n // 2 and trig.update(img, t_s):
                    if sum(j.is_alive() for j in jobs) >= 2:
                        print("  (a throw was seen but two are still being written: ignored)")
                    else:
                        post = []
                        trig_idx = len(ring) - 1
                        state["armed"] = False
                        print("  throw seen ...", end="", flush=True)
            else:
                post.append((img, sts, hns, exp_us, gn))
                trig.update(img, t_s)
                if len(post) >= post_n:
                    items = list(ring) + post
                    ring.clear()
                    post = None
                    k = state["saved"] + 1
                    state["saved"] = k
                    meta = dict(trigger=trig_idx, n=len(items), exposure_us_median=float(np.median([x[3] for x in items])),
                                gain_median=float(np.median([x[4] for x in items])), outcome=None,
                                live_trigger_blob=None, saved=time.strftime("%H:%M:%S"))
                    th = threading.Thread(target=dump, args=(k, [x[0] for x in items], [(x[1], x[2]) for x in items], meta), daemon=True)
                    th.start()
                    jobs.append(th)
                    print(f" captured {len(items)} frames (throw {k}); writing")
            # re-arm when the scene has been still for a second
            if not state["armed"] and post is None and trig.idle(t_s):
                state["armed"] = True
            # report finished writes and ask for the outcome (without stopping capture: input() runs in the same
            # thread, so frames wait in the camera's buffers: only ask when the scene is idle)
            if post is None and state["armed"] and not pending.empty():
                k, path, secs, n = pending.get()
                print(f"  t{k:03d} written in {secs:.1f} s ({n} frames)")
                if not a.no_prompt:
                    ans = ""
                    while ans not in ("y", "n", "s", "q"):
                        ans = input(f"  throw {k}: did it go in the basket? [y/n, s=discard, q=finish] ").strip().lower()[:1]
                    if ans == "s":
                        import shutil
                        shutil.rmtree(path, ignore_errors=True)
                    elif ans in ("y", "n"):
                        S.label_throw(path, ans)
                    state["quit"] = ans == "q"
                    ring.clear()
                    trig.prev = []
                if a.count and state["saved"] >= a.count and pending.empty():
                    state["quit"] = True
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        for j in jobs:
            j.join()
        try:
            np.save(d / "tag_end.npy", cam.read()[0])      # and at the end: did the camera or the basket move?
        except Exception:
            pass
        cam.close()
        while not pending.empty():
            k, path, secs, n = pending.get()
            print(f"  t{k:03d} written in {secs:.1f} s")
    print(f"done: {state['saved']} throws in {d}")


if __name__ == "__main__":
    main()
