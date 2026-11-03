"""Live view of the robot's ball detector on a real camera, with knobs (README.md).

Runs on the Pi (or anywhere with the camera) and serves a page: the camera image with the detector's blobs drawn
on it, sliders for the detector's settings (hue window, saturation, value, minimum blob size, size gain), the
camera's exposure, and the ball's 3D position. Open it in a browser on the PC. The detector is the robot's own
(omni_catch.detector / fast_detector, the same C++ pixel loops), so what you tune here is what the robot would use.

    python ball_view.py --source picam        # Arducam / Pi Camera Module 3 (Picamera2): position from the ball's size
    python ball_view.py --source realsense    # RealSense D435 colour + depth aligned: position from the depth map
    python ball_view.py --source synthetic    # a rendered ball with known truth, no camera (also what the tests use)

Position, two ways (camera frame, x right, y down, z forward, metres):
  size   range = f * R / b: f the focal length in px, R the ball's radius, b the blob's measured radius (the
         minor semi-axis of its moments, as the robot measures it). Both cameras. f comes from the lens and the
         sensor crop (Pi) or the calibration (RealSense); "calibrate" fixes it from a known distance.
  depth  RealSense only: the median depth over the inner part of the ball's disc, plus R (depth sees the front
         surface). Compared with the size range it says what size_gain would make the two agree.
"""
from __future__ import annotations

import argparse
import colorsys
import json
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
for p in (HERE.parent / "src" / "omni_catch", HERE.parents[1] / "ros" / "src" / "omni_catch"):   # deployed / repo
    if p.is_dir():
        sys.path.insert(0, str(p))

from omni_catch.detector import Detector, DetectorConfig, ball_mask      # noqa: E402
from omni_catch.frames import BALL_RGB, ArrayFrame                       # noqa: E402

try:
    import simplejpeg
except ImportError:           # the PC: PIL's encoder is fast enough there
    simplejpeg = None

BALL_D_MM = 74.0              # the design's foam ball (design.md 1)
PIXEL_UM = 1.4                # IMX708
LENS_MM = {"standard": 4.74, "wide": 2.75}      # Camera Module 3 focal lengths (Raspberry Pi's documentation)


# ---------------------------------------------------------------- the maths (tested without a camera)

def size_range(b_px: float, f_px: float, ball_m: float) -> float:
    """Distance to the ball's centre along the axis from its apparent radius: f R / b."""
    return f_px * (ball_m / 2) / max(b_px, 1e-6)


def pixel_to_xyz(u: float, v: float, z: float, fx: float, fy: float, ppx: float, ppy: float):
    return ((u - ppx) / fx * z, (v - ppy) / fy * z, z)


def depth_position(depth: np.ndarray, u: float, v: float, b: float, ball_m: float, scale: float = 1.0,
                   inner: float = 0.6):
    """The ball's centre from a depth map aligned to the colour image (raw units x scale = metres).

    Median depth over the inner disc (inner x the measured radius: the rim mixes in the background), plus the
    ball's radius (depth sees the front surface). Returns dict(z, z_surface, sigma_z, fill, n) or None when
    fewer than 30% of the disc has depth."""
    h, w = depth.shape
    r = max(2.0, inner * b)
    x0, x1 = max(0, int(u - r - 1)), min(w, int(u + r + 2))
    y0, y1 = max(0, int(v - r - 1)), min(h, int(v + r + 2))
    if x1 <= x0 or y1 <= y0:
        return None
    yy, xx = np.mgrid[y0:y1, x0:x1]
    disc = (xx + 0.5 - u) ** 2 + (yy + 0.5 - v) ** 2 <= r * r
    vals = depth[y0:y1, x0:x1][disc].astype(np.float64) * scale
    ok = vals[vals > 0]
    if disc.sum() < 5 or ok.size < 0.3 * disc.sum() or ok.size < 5:
        return None
    zs = float(np.median(ok))
    mad = float(np.median(np.abs(ok - zs)))
    return dict(z_surface=zs, z=zs + ball_m / 2, sigma_z=1.4826 * mad, fill=float(ok.size / disc.sum()), n=int(ok.size))


def pick_window(rgb_patch: np.ndarray, hue_half: float = 20.0) -> dict:
    """Detector settings that would catch the colour of a clicked patch: hue +- hue_half, saturation and value at
    60% of the patch's."""
    r, g, b = (float(x) / 255 for x in rgb_patch.reshape(-1, 3).mean(axis=0))
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    return dict(hue_min=max(0.0, h * 360 - hue_half), hue_max=min(360.0, h * 360 + hue_half),
                sat_min=round(max(0.05, 0.6 * s), 2), val_min=round(max(0.05, 0.6 * v), 2),
                picked=dict(h=round(h * 360, 1), s=round(s, 2), v=round(v, 2)))


def second_difference_noise(u: list[float]) -> float | None:
    """White-noise sigma from the second differences of a smooth track (var(d2) = 6 sigma^2): the centroid's
    measurement noise in px, if the ball moves smoothly."""
    if len(u) < 8:
        return None
    d2 = np.diff(np.asarray(u, float), n=2)
    return float(np.sqrt(np.mean(d2 ** 2) / 6))


def hsv_of(rgb: np.ndarray):
    """(hue deg 0-360, saturation, value) per pixel, with the detector's definitions (ball_mask): value is max/255,
    saturation is chroma / max."""
    c = rgb.reshape(-1, 3).astype(np.float32)
    mx, mn = c.max(axis=1), c.min(axis=1)
    chroma = mx - mn
    cz = np.maximum(chroma, 1e-6)
    r, g, b = c[:, 0], c[:, 1], c[:, 2]
    h = np.where(mx == r, ((g - b) / cz) % 6, np.where(mx == g, (b - r) / cz + 2, (r - g) / cz + 4)) * 60.0
    return np.where(chroma > 0, h, 0.0) % 360.0, chroma / np.maximum(mx, 1.0), mx / 255.0


def circ_mean_deg(h: np.ndarray) -> float:
    a = np.radians(h)
    return float(np.degrees(np.arctan2(np.sin(a).mean(), np.cos(a).mean())) % 360)


def hsv_stats(px: np.ndarray) -> dict | None:
    """The colour of some pixels, for the page: 5th / 50th / 95th percentile of hue (around the circular mean, so it
    may leave 0-360), saturation and value, and the mean RGB."""
    h, s, v = hsv_of(px)
    if h.size < 5:
        return None
    c = circ_mean_deg(h)
    dh = (h - c + 180) % 360 - 180
    q = lambda a: [float(x) for x in np.percentile(a, [5, 50, 95])]          # noqa: E731
    return dict(h=[c + x for x in q(dh)], s=q(s), v=q(v), rgb=[int(round(x)) for x in px.reshape(-1, 3).mean(axis=0)],
                n=int(h.size))


def disc_pixels(rgb: np.ndarray, u: float, v: float, r: float) -> np.ndarray:
    """The (N, 3) pixels inside a disc."""
    h, w = rgb.shape[:2]
    x0, x1 = max(0, int(u - r - 1)), min(w, int(u + r + 2))
    y0, y1 = max(0, int(v - r - 1)), min(h, int(v + r + 2))
    if x1 <= x0 or y1 <= y0:
        return rgb[:0].reshape(0, 3)
    yy, xx = np.mgrid[y0:y1, x0:x1]
    return rgb[y0:y1, x0:x1][(xx + 0.5 - u) ** 2 + (yy + 0.5 - v) ** 2 <= r * r]


def tune_colour(px: np.ndarray, margin_deg: float = 8.0, slack: float = 0.85) -> dict | None:
    """A colour window for the ball from pixels that are mostly ball: the dominant hue (so some background in the disc
    doesn't widen it), its 2nd-98th percentile spread plus a margin, and saturation / value floors at `slack` x their
    5th percentile (the dim rim and a little lighting change stay inside). None when there is no single saturated colour.
    The detector's hue window can't wrap through 0 deg: `wraps` says it was clamped."""
    h, s, v = hsv_of(px)
    ok = (s >= 0.2) & (v >= 0.15)
    if ok.sum() < max(12, 0.15 * h.size):
        return None
    hist = np.bincount(np.minimum(h[ok].astype(int), 359), minlength=360).astype(float)
    peak = float(np.argmax(sum(np.roll(hist, k) for k in range(-15, 16)))) + 0.5
    dh = (h - peak + 180) % 360 - 180
    keep = ok & (np.abs(dh) <= 30)
    if keep.sum() < 10:
        return None
    lo, hi = np.percentile(dh[keep], [2, 98])
    hmin, hmax = peak + lo - margin_deg, peak + hi + margin_deg
    return dict(hue_min=float(max(0, math.floor(hmin))), hue_max=float(min(360, math.ceil(hmax))),
                sat_min=round(float(np.clip(slack * np.percentile(s[keep], 5), 0.05, 0.95)), 2),
                val_min=round(float(np.clip(slack * np.percentile(v[keep], 5), 0.05, 0.95)), 2),
                wraps=bool(hmin < 0 or hmax > 360), share=float(keep.sum() / h.size), peak=peak)


def effective_min_radius(p: dict) -> float:
    """The smallest ball radius (px) the whole-frame search finds: the minimum blob's, or about half the coarse stride
    (measured on clean discs: stride 4 finds r 2 px 80%, r 2.5 px always; the real ball's dim rim costs more)."""
    return max(math.sqrt(p["min_blob_px"] / math.pi), 0.5 * p["coarse_stride"], 0.5)


def ring_geometry(frame: "Frame", p: dict) -> dict:
    """Where the ball would sit at the 'known distance': the image centre, with the radius it would have there."""
    h, w = frame.rgb.shape[:2]
    f = (frame.fx + frame.fy) / 2 * p["f_scale"]
    return dict(u=w / 2, v=h / 2, r=f * (p["ball_mm"] / 2000) / p["known_m"], f_px=f)


def autotune(res: "Result", target: str = "ring") -> dict:
    """Set the colour window and the minimum blob size from the ball itself. target 'ring': the pixels inside the
    aiming ring at the image centre (hold the ball in it); 'blob': the detected blob. Returns dict(ok, message, applied):
    the new knob values, checked on this frame (the ball must come out as a blob at the target; the message says how
    many other blobs pass the new filter, which is how the filter is not unique)."""
    from dataclasses import replace
    p, fr = res.params, res.frame
    if target == "blob":
        if res.det is None:
            return dict(ok=False, message="No blob to tune on. Hold the ball in the cyan ring and use 'auto-tune on the ring'.")
        u, v, b, _ = res.det
        r = 0.8 * b / max(p["size_gain"], 1e-6)
        what = "the detected blob"
    else:
        g = ring_geometry(fr, p)
        u, v, r = g["u"], g["v"], 0.75 * g["r"]
        what = "the ring"
        if r < 3:
            return dict(ok=False, message=f"The ring is only {g['r']:.1f} px at {p['known_m']:.1f} m, too small to tune on: "
                                          "lower 'known distance', hold the ball there, and try again.")
    t = tune_colour(disc_pixels(fr.rgb, u, v, r))
    if t is None:
        return dict(ok=False, message=f"No single saturated colour in {what} (a ball needs saturation of 0.2 or more): "
                                      "is the ball inside the cyan ring?")
    cfg = replace(DetectorConfig(), hue_deg=(t["hue_min"], t["hue_max"]), min_sat=t["sat_min"], min_val=t["val_min"],
                  min_blob_px=1, coarse_stride=int(p["coarse_stride"]), size_gain=1.0)
    afr = ArrayFrame(fr.rgb)
    half = 2.0 * r + 10
    comps = Detector(cfg)._components(afr, u - half, v - half, u + half, v + half)
    comps = [c for c in comps if (c[0] - u) ** 2 + (c[1] - v) ** 2 <= (1.5 * r + 2) ** 2]
    if not comps:
        return dict(ok=False, message=f"The colour found in {what} doesn't make a blob there (the hue window may cross 0 deg).")
    blob = max(comps, key=lambda c: c[3])
    min_blob = int(max(3, round(0.25 * blob[3])))
    # other blobs that would pass: a decimated mask, away from the ball
    others, largest = 0, 0
    cm = ball_mask(fr.rgb[::4, ::4], cfg)
    if cm.any():
        from omni_catch.detector import label
        lab, n = label(cm)
        sizes = np.bincount(lab.ravel(), minlength=n + 1)
        for k in np.argsort(sizes[1:])[::-1][:10] + 1:
            area = int(sizes[k]) * 16
            ys, xs = np.nonzero(lab == k)
            if area < min_blob or np.hypot(xs.mean() * 4 + 2 - blob[0], ys.mean() * 4 + 2 - blob[1]) < 2.0 * blob[2] + 8:
                continue
            others, largest = others + 1, max(largest, area)
    msg = (f"Tuned on {what} ({100 * t['share']:.0f}% of its pixels share one colour): hue {t['hue_min']:.0f}-{t['hue_max']:.0f} deg, "
           f"saturation from {t['sat_min']:.2f}, value from {t['val_min']:.2f}. The ball is {blob[3]} px here, so the minimum blob "
           f"is {min_blob} px (a quarter: it is still found at about twice this distance).")
    if others:
        msg += f" {others} other blob(s) of this colour pass the filter elsewhere in the image (largest {largest} px)."
    else:
        msg += " Nothing else in the image passes."
    if t["wraps"]:
        msg += " The hue window would cross 0 deg, which the detector can't do: it was clamped, so a red ball won't work."
    return dict(ok=True, message=msg, others=others, applied=dict(hue_min=t["hue_min"], hue_max=t["hue_max"], sat_min=t["sat_min"],
                                                                 val_min=t["val_min"], min_blob_px=min_blob))


def depth_colormap(depth_u16: np.ndarray, scale: float, near: float = 0.2, far: float = 4.0) -> np.ndarray:
    """Depth as colour: near red, through yellow, green and cyan, to blue far; no depth = black."""
    z = depth_u16.astype(np.float32) * scale
    t = np.clip((z - near) / (far - near), 0, 1)
    stops = np.array([0, 0.25, 0.5, 0.75, 1.0])
    pal = np.array([[255, 40, 40], [255, 230, 40], [60, 220, 80], [40, 200, 230], [60, 60, 255]], np.float32)
    out = np.stack([np.interp(t, stops, pal[:, c]) for c in range(3)], axis=-1).astype(np.uint8)
    out[depth_u16 == 0] = 0
    return out


# ---------------------------------------------------------------- frames and sources

@dataclass
class Frame:
    rgb: np.ndarray                       # H x W x 3 uint8, RGB
    fx: float
    fy: float
    ppx: float
    ppy: float
    t: float = 0.0                        # time.monotonic() when it reached us
    age_ms: float | None = None           # capture -> here, when the camera says
    depth: np.ndarray | None = None       # H x W uint16 aligned to rgb
    depth_scale: float = 0.001
    meta: dict = field(default_factory=dict)


class SyntheticSource:
    """A rendered scene with a yellow ball moving on a known path (and a yellow box that isn't the ball), with a
    matching depth map: the truth is in frame.meta['truth'], so the pipeline can be checked end to end."""
    name = "synthetic"
    has_depth = True
    exposure_max, gain_min, gain_max = 8000, 1.0, 16.0

    def __init__(self, size=(640, 480), fps: float = 30.0, pace: bool = True):
        self.w, self.h = size
        self.f = 500.0 * self.w / 640
        self.fps, self.pace, self.k = fps, pace, 0
        self.t0 = time.monotonic()
        yy, xx = np.mgrid[0:self.h, 0:self.w]
        self.bg = np.stack([95 + 40 * yy / self.h, 100 + 30 * xx / self.w, 110 + 0 * xx], axis=-1).astype(np.float32)
        self.rng = np.random.default_rng(1)

    def truth(self, t: float):
        z = 2.0 + 0.9 * math.sin(0.5 * t)
        return 0.6 * math.sin(1.6 * t), 0.15 * math.sin(2.3 * t), z

    def read(self) -> Frame:
        t = self.k / self.fps
        self.k += 1
        if self.pace:
            time.sleep(max(0.0, self.t0 + t - time.monotonic()))
        x, y, z = self.truth(t)
        f, ppx, ppy = self.f, self.w / 2, self.h / 2
        u, v = ppx + f * x / z, ppy + f * y / z
        R = BALL_D_MM / 2000
        b = f * R / z
        img = self.bg + 3 * self.rng.standard_normal(self.bg.shape, dtype=np.float32)
        depth = 3.5 + 0.004 * self.rng.standard_normal((self.h, self.w), dtype=np.float32)
        img[300:380, 40:140] = (222, 196, 40)                     # a yellow box: hue ~48, the ball's neighbour
        depth[300:380, 40:140] = 3.4
        x0, x1 = max(0, int(u - b - 2)), min(self.w, int(u + b + 3))
        y0, y1 = max(0, int(v - b - 2)), min(self.h, int(v + b + 3))
        yy, xx = np.mgrid[y0:y1, x0:x1]
        d2 = (xx + 0.5 - u) ** 2 + (yy + 0.5 - v) ** 2
        m = d2 <= b * b
        shade = (0.72 + 0.28 * np.sqrt(np.clip(1 - d2 / (b * b), 0, 1)))[..., None]
        img[y0:y1, x0:x1][m] = (np.array(BALL_RGB, np.float32) * shade)[m]
        zs = z - np.sqrt(np.clip(R * R - d2 / (b * b) * R * R, 0, None))        # the sphere's front surface
        depth[y0:y1, x0:x1][m] = zs[m] + self.rng.normal(0, 0.002 * z * z, m.sum())
        return Frame(np.clip(img, 0, 255).astype(np.uint8), f, f, ppx, ppy, t=time.monotonic(), age_ms=0.0,
                     depth=np.round(depth / 0.001).astype(np.uint16), depth_scale=0.001,
                     meta=dict(truth=dict(x=x, y=y, z=z, u=u, v=v, b=b)))

    def set_controls(self, p: dict):
        pass

    def close(self):
        pass


class PiCamSource:
    """Pi Camera Module 3 / Arducam IMX708 through Picamera2. mode 'robot': the sensor's 1536x864 @ 120 fps (the
    design's camera head mode, cropped); 'wide': 2304x1296 @ 56 fps, the full field of view."""
    name = "picam"
    has_depth = False
    exposure_max, gain_min, gain_max = 8000, 1.0, 16.0

    def __init__(self, size=(768, 432), mode: str = "robot", lens: str = "standard"):
        from picamera2 import Picamera2
        raw, fps = ((1536, 864), 120) if mode == "robot" else ((2304, 1296), 56)
        us = int(round(1e6 / fps))
        self.focal_px = LENS_MM[lens] / (PIXEL_UM / 1000)
        self.cam = Picamera2()
        cfg = self.cam.create_video_configuration(
            main={"size": size, "format": "BGR888"},          # R, G, B in memory: what the detector expects
            raw={"size": raw}, controls={"FrameDurationLimits": (us, us)}, buffer_count=6, queue=False)
        self.cam.configure(cfg)
        self.size, self.fps, self.mode = size, fps, mode
        self.cam.start()
        self.applied = None

    def read(self) -> Frame:
        req = self.cam.capture_request()
        try:
            md = req.get_metadata()
            img = req.make_array("main")
        finally:
            req.release()
        crop = md.get("ScalerCrop")
        crop_w = crop[2] if crop else 4608
        f = self.focal_px * self.size[0] / crop_w            # full-resolution px -> output px, by the sensor crop
        sts = md.get("SensorTimestamp")
        age = (time.monotonic_ns() - sts) / 1e6 if sts else None
        return Frame(np.ascontiguousarray(img), f, f, self.size[0] / 2, self.size[1] / 2, t=time.monotonic(),
                     age_ms=age, meta=dict(exposure_us=md.get("ExposureTime"), gain=md.get("AnalogueGain"),
                                           crop=list(crop) if crop else None))

    def set_controls(self, p: dict):
        key = (p["exposure_us"], p["gain"], p["awb"])
        if key == self.applied:
            return
        self.applied = key
        c = {"AwbEnable": bool(p["awb"])}
        if p["exposure_us"] > 0:
            c.update(AeEnable=False, ExposureTime=int(p["exposure_us"]), AnalogueGain=float(p["gain"]))
        else:
            c.update(AeEnable=True)
        self.cam.set_controls(c)

    def close(self):
        self.cam.stop()
        self.cam.close()


class RealSenseSource:
    """D435: the colour camera (the detector needs colour) with the depth map aligned to it."""
    name = "realsense"
    has_depth = True
    exposure_max, gain_min, gain_max = 30000, 0.0, 128.0

    def __init__(self, size=(640, 480), fps: int = 30):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, size[0], size[1], rs.format.rgb8, fps)
        cfg.enable_stream(rs.stream.depth, size[0], size[1], rs.format.z16, fps)
        prof = self.pipe.start(cfg)
        dev = prof.get_device()
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        self.color_sensor = next(s for s in dev.query_sensors() if s.is_color_sensor())
        self.align = rs.align(rs.stream.color)
        i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intr = (i.fx, i.fy, i.ppx, i.ppy)
        self.applied = None

    def read(self) -> Frame:
        fs = self.align.process(self.pipe.wait_for_frames(5000))
        c, d = fs.get_color_frame(), fs.get_depth_frame()
        age = time.time() * 1000 - c.get_timestamp() if str(c.get_frame_timestamp_domain()).endswith("global_time") else None
        return Frame(np.asanyarray(c.get_data()).copy(), *self.intr, t=time.monotonic(), age_ms=age,
                     depth=np.asanyarray(d.get_data()).copy(), depth_scale=self.depth_scale)

    def set_controls(self, p: dict):
        key = (p["exposure_us"], p["gain"], p["awb"])
        if key == self.applied:
            return
        self.applied = key
        rs, s = self.rs, self.color_sensor
        for opt, val in ((rs.option.enable_auto_white_balance, float(p["awb"])),
                         (rs.option.enable_auto_exposure, 1.0 if p["exposure_us"] <= 0 else 0.0)):
            if s.supports(opt):
                s.set_option(opt, val)
        if p["exposure_us"] > 0:
            if s.supports(rs.option.exposure):
                s.set_option(rs.option.exposure, max(1.0, round(p["exposure_us"] / 100)))      # units of 100 us
            if s.supports(rs.option.gain):
                s.set_option(rs.option.gain, float(p["gain"]))

    def close(self):
        self.pipe.stop()


# ---------------------------------------------------------------- the knobs

# name, label, min, max, step, default, group
KNOBS = [
    ("hue_min", "hue min (deg)", 0, 360, 1, 45, "colour"),
    ("hue_max", "hue max (deg)", 0, 360, 1, 100, "colour"),
    ("sat_min", "saturation min", 0, 1, 0.01, 0.45, "colour"),
    ("val_min", "value min", 0, 1, 0.01, 0.35, "colour"),
    ("min_blob_px", "min blob (px)", 1, 400, 1, 8, "blob"),
    ("size_gain", "size gain", 0.5, 2.0, 0.01, 1.0, "blob"),
    ("coarse_stride", "coarse stride", 1, 8, 1, 4, "blob"),
    ("ball_mm", "ball diameter (mm)", 20, 300, 1, BALL_D_MM, "range"),
    ("f_scale", "focal scale", 0.3, 3.0, 0.005, 1.0, "range"),
    ("known_m", "known distance (m)", 0.2, 10, 0.05, 1.0, "range"),
]
BOOLS = [("track", "track in a window (the robot's mode after the first lock)", False),
         ("ignore_still", "acquisition ignores stationary blobs (the robot's tracker starts on the one that moves)", False),
         ("show_all", "show every candidate blob", True),
         ("show_guides", "show the aiming ring (centre) and the size legend", True),
         ("show_small", "ring the blobs rejected as too small (red)", False),
         ("show_mask", "tint the pixels that pass the colour test", False),
         ("show_trail", "show the track", True),
         ("click_picks", "a click on the image sets the colour window", True),
         ("awb", "auto white balance", True)]
INTS = {"min_blob_px", "coarse_stride"}


class Params:
    def __init__(self, source):
        self.lock = threading.Lock()
        self.spec = {k[0]: k for k in KNOBS}
        self.spec["exposure_us"] = ("exposure_us", "exposure (us, 0 = auto)", 0, source.exposure_max, 100, 0, "camera")
        self.spec["gain"] = ("gain", "gain", source.gain_min, source.gain_max, 0.5, max(1.0, source.gain_min), "camera")
        self.v = {k: s[5] for k, s in self.spec.items()}
        self.v.update({k: d for k, _, d in BOOLS})
        self.v["view"] = "color"

    def set(self, name: str, value):
        with self.lock:
            if name == "view":
                if value in ("color", "depth"):
                    self.v["view"] = value
                return
            if name in self.v and isinstance(self.v[name], bool):
                self.v[name] = bool(value)
            elif name in self.spec:
                _, _, lo, hi, _, _, _ = self.spec[name]
                x = min(hi, max(lo, float(value)))
                self.v[name] = int(round(x)) if name in INTS else x

    def get(self) -> dict:
        with self.lock:
            return dict(self.v)


# ---------------------------------------------------------------- saved settings

SETTINGS_FORMAT = "ball_view/1"
DEFAULT_SETTINGS_DIR = Path.home() / "omni_ws" / "ball_view_settings"      # outside ball_view/: a deploy replaces that folder


def settings_dict(pipe: "Pipeline") -> dict:
    """The chosen ball-tracker parameters: 'detector' is what DetectorConfig takes (and what the robot's config needs),
    the rest says how the picture was made; 'knobs' is every slider and tick box, for loading back."""
    p = pipe.params.get()
    r = pipe.latest
    out = dict(format=SETTINGS_FORMAT, saved=time.strftime("%Y-%m-%dT%H:%M:%S"), source=pipe.source.name,
               detector=dict(hue_deg=[p["hue_min"], p["hue_max"]], min_sat=p["sat_min"], min_val=p["val_min"],
                             min_blob_px=int(p["min_blob_px"]), size_gain=round(p["size_gain"], 4),
                             coarse_stride=int(p["coarse_stride"])),
               ball_mm=p["ball_mm"], focal_scale=round(p["f_scale"], 4),
               camera=dict(exposure_us=p["exposure_us"], gain=p["gain"], awb=bool(p["awb"])),
               rules=dict(track=bool(p["track"]), ignore_still=bool(p["ignore_still"])),
               knobs={k: v for k, v in p.items() if k != "view"})
    if r is not None:
        out["image_px"] = [r.frame.rgb.shape[1], r.frame.rgb.shape[0]]
        out["focal_px"] = round(ring_geometry(r.frame, p)["f_px"], 1)
    return out


def save_settings(pipe: "Pipeline") -> dict:
    """Write the settings as <dir>/<source>.json (what the next view of this source starts from) and a dated copy."""
    d = pipe.settings_dir
    if d is None:
        return dict(ok=False, message="Saving is off (no settings folder).")
    s = settings_dict(pipe)
    text = json.dumps(s, indent=1)
    try:
        d.mkdir(parents=True, exist_ok=True)
        latest = d / f"{s['source']}.json"
        dated = d / f"{s['source']}_{time.strftime('%Y%m%d_%H%M%S')}.json"
        latest.write_text(text)
        dated.write_text(text)
    except OSError as e:
        return dict(ok=False, message=f"Couldn't write {d}: {e}")
    return dict(ok=True, path=str(latest), dated=str(dated), settings=s,
                message=f"Saved to {latest} on {os.uname().nodename if hasattr(os, 'uname') else 'this computer'} "
                        f"(and {dated.name}). The next view of the {s['source']} starts from it.")


def load_settings(pipe: "Pipeline", path: Path | None = None) -> dict:
    path = path or (pipe.settings_dir / f"{pipe.source.name}.json" if pipe.settings_dir else None)
    if path is None or not path.is_file():
        return dict(ok=False, message="No saved settings for this camera yet.")
    try:
        s = json.loads(path.read_text())
        knobs = s["knobs"]
    except (OSError, ValueError, KeyError) as e:
        return dict(ok=False, message=f"Couldn't read {path}: {e}")
    n = 0
    for k, v in knobs.items():
        if k in pipe.params.v and k != "view":
            pipe.params.set(k, v)
            n += 1
    return dict(ok=True, message=f"Loaded {n} settings from {path} (saved {s.get('saved', '?')}).")


# ---------------------------------------------------------------- the pipeline

@dataclass
class Result:
    seq: int
    frame: Frame
    det: tuple | None                     # u, v, b, n_px
    cands: list
    still: list                           # per candidate: was it already there, within 1 px, in the last frame
    window: tuple | None
    detect_ms: float
    size: dict | None
    depth: dict | None
    trail: list
    params: dict
    small: list = field(default_factory=list)       # colour-passing blobs under the minimum size (when asked for)


class Pipeline:
    def __init__(self, source, params: Params, native: bool = True):
        self.source, self.params = source, params
        self.cfg = DetectorConfig()
        self.det = None
        if native:
            try:
                from omni_catch.fast_detector import FastDetector
                self.det = FastDetector(self.cfg)
            except Exception as e:                      # no compiler: numpy detector
                print(f"native detector unavailable ({str(e).splitlines()[0]}): using numpy", file=sys.stderr)
        if self.det is None:
            self.det = Detector(self.cfg)
        self.cond = threading.Condition()
        self.latest: Result | None = None
        self.seq = 0
        self.stop = threading.Event()
        self.hits = deque(maxlen=120)
        self.trail = deque(maxlen=40)
        self.t_frames = deque(maxlen=60)
        self.ms = deque(maxlen=120)
        self.ages = deque(maxlen=120)
        self.last = None
        self.prev_cands = []
        self.mode = None
        self.vel = (0.0, 0.0)
        self.error = None
        self.undo = None                               # the knob values before the last auto-tune
        self.settings_dir: Path | None = None          # where 'save settings' writes (main sets it)

    def _apply(self, p: dict):
        c = self.cfg
        c.hue_deg = (p["hue_min"], p["hue_max"])
        c.min_sat, c.min_val = p["sat_min"], p["val_min"]
        c.min_blob_px, c.size_gain, c.coarse_stride = int(p["min_blob_px"]), p["size_gain"], int(p["coarse_stride"])

    def process(self, frame: Frame, p: dict | None = None) -> Result:
        p = p or self.params.get()
        self._apply(p)
        fr = ArrayFrame(frame.rgb)
        mode = (p["track"], p["ignore_still"])
        if mode != self.mode:                          # a new rule starts from scratch, not from the old lock
            self.mode, self.last, self.prev_cands, self.vel = mode, None, [], (0.0, 0.0)
            self.trail.clear()
        t0 = time.perf_counter()
        det = window = None
        if p["track"] and self.last is not None:       # the robot's mode after the first lock: look near the prediction
            u, v, b, _ = self.last
            pu, pv = u + self.vel[0], v + self.vel[1]
            half = self.cfg.window_scale * max(b, 1.0) + self.cfg.window_px
            comps = self.det._components(fr, pu - half, pv - half, pu + half, pv + half)
            if comps:
                det = min(comps, key=lambda c: (c[0] - pu) ** 2 + (c[1] - pv) ** 2)
                window = (pu - half, pv - half, pu + half, pv + half)
        cands, still = [], []
        if det is None:                                # acquisition: the whole frame
            if p["show_all"] or p["ignore_still"]:
                cands = self.det.candidates(fr, 8)     # largest first
                # a blob within 1 px of one in the last frame is "still" (tracker.TrackerConfig.acq_still_px)
                still = [any((c[0] - o[0]) ** 2 + (c[1] - o[1]) ** 2 < 1.0 for o in self.prev_cands) for c in cands]
            if p["ignore_still"]:
                # needs a previous frame to tell moving from still
                moving = [c for c, st in zip(cands, still) if not st] if self.prev_cands else []
                det = moving[0] if moving else None
            else:
                det = self.det.detect(fr, None)
            self.prev_cands = cands
            if self.last is not None:                  # the track was lost
                self.last, self.vel = None, (0.0, 0.0)
        else:
            self.prev_cands = []
        if det is not None:
            if self.last is not None:
                self.vel = (det[0] - self.last[0], det[1] - self.last[1])
            self.trail.append((det[0], det[1]))
        else:
            self.vel = (0.0, 0.0)
            self.trail.clear()
        self.last = det
        if not p["show_all"] or (det is not None and not cands):
            cands, still = ([det] if det else []), [False] * (1 if det else 0)
        ms = (time.perf_counter() - t0) * 1e3
        small = []
        if p["show_small"] and int(p["min_blob_px"]) > 2:          # what the minimum size throws away
            mn = int(p["min_blob_px"])
            self.cfg.min_blob_px = 2
            try:
                small = [c for c in self.det.candidates(fr, 40) if c[3] < mn]
            finally:
                self.cfg.min_blob_px = mn
        self.hits.append(det is not None)
        self.ms.append(ms)
        self.t_frames.append(frame.t)
        if frame.age_ms is not None:
            self.ages.append(frame.age_ms + ms)
        size = depth = None
        if det is not None:
            ball_m = p["ball_mm"] / 1000
            f = (frame.fx + frame.fy) / 2 * p["f_scale"]
            z = size_range(det[2], f, ball_m)
            size = dict(z=z, x=(det[0] - frame.ppx) / (frame.fx * p["f_scale"]) * z,
                        y=(det[1] - frame.ppy) / (frame.fy * p["f_scale"]) * z,
                        z_1px=size_range(det[2] - 0.5, f, ball_m) - z, f_px=f)
            if frame.depth is not None:
                d = depth_position(frame.depth, det[0], det[1], det[2], ball_m, frame.depth_scale)
                if d is not None:
                    d["x"], d["y"], _ = pixel_to_xyz(det[0], det[1], d["z"], frame.fx, frame.fy, frame.ppx, frame.ppy)
                    # the size gain that makes the size range equal the depth range
                    b_need = (frame.fx * p["f_scale"]) * ball_m / 2 / d["z"]
                    d["suggest_size_gain"] = p["size_gain"] * b_need / det[2]
                    d["size_vs_depth_pct"] = 100 * (size["z"] - d["z"]) / d["z"]
                    depth = d
        self.seq += 1
        return Result(self.seq, frame, det, cands, still, window, ms, size, depth, list(self.trail), p, small)

    def run(self):
        try:
            while not self.stop.is_set():
                p = self.params.get()
                self.source.set_controls(p)
                res = self.process(self.source.read(), p)
                with self.cond:
                    self.latest = res
                    self.cond.notify_all()
        except Exception as e:                         # report it on the page instead of dying silently
            import traceback
            traceback.print_exc()
            self.error = f"{type(e).__name__}: {e}"
            with self.cond:
                self.cond.notify_all()

    def wait(self, after: int, timeout: float = 1.0) -> Result | None:
        with self.cond:
            self.cond.wait_for(lambda: (self.latest is not None and self.latest.seq != after) or self.stop.is_set()
                               or self.error, timeout)
            return self.latest if self.latest is not None and self.latest.seq != after else None

    def fps(self) -> float:
        t = list(self.t_frames)
        return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 2 and t[-1] > t[0] else 0.0

    def status(self) -> dict:
        r = self.latest
        pct = lambda a, q: float(np.percentile(a, q)) if len(a) else None          # noqa: E731
        trail = list(self.trail)
        out = dict(source=self.source.name, fps=self.fps(), detect_ms=pct(self.ms, 50), detect_ms_p95=pct(self.ms, 95),
                   age_ms=pct(self.ages, 50), hit_rate=sum(self.hits) / len(self.hits) if self.hits else None,
                   noise_px=second_difference_noise([t[0] for t in trail[-30:]]), error=self.error,
                   native=type(self.det).__name__ == "FastDetector",
                   params=self.params.get(), spec=[list(s) for s in self.params.spec.values()],
                   bools=[list(b) for b in BOOLS], has_depth=self.source.has_depth, det=None, size=None, depth=None,
                   cfg=dict(hue_deg=list(self.cfg.hue_deg), min_sat=self.cfg.min_sat, min_val=self.cfg.min_val,
                            min_blob_px=self.cfg.min_blob_px, size_gain=self.cfg.size_gain,
                            coarse_stride=self.cfg.coarse_stride))
        f = self.settings_dir / f"{self.source.name}.json" if self.settings_dir else None
        out["saved_at"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(f.stat().st_mtime)) if f and f.is_file() else None
        if r is not None:
            out.update(det=r.det, size=r.size, depth=r.depth, image=[r.frame.rgb.shape[1], r.frame.rgb.shape[0]],
                       cands=[list(c) for c in r.cands])
            p = out["params"]
            ring = ring_geometry(r.frame, p)
            r_min = math.sqrt(p["min_blob_px"] / math.pi)
            out["ring"] = ring
            r_eff = effective_min_radius(p)
            out["limits"] = dict(r_min_px=r_min, r_eff_px=r_eff, range_m=ring["f_px"] * (p["ball_mm"] / 2000) / r_eff)
            if r.det is not None:                      # the colour of the blob's own pixels, for the swatches
                out["blob_hsv"] = hsv_stats(disc_pixels(r.frame.rgb, r.det[0], r.det[1],
                                                        0.9 * r.det[2] / max(r.params["size_gain"], 1e-6)))
        return out


# ---------------------------------------------------------------- drawing

_FONT = {}


def font(size=14):
    if size not in _FONT:
        try:
            _FONT[size] = ImageFont.load_default(size=size)
        except TypeError:                      # Pillow < 10.1
            _FONT[size] = ImageFont.load_default()
    return _FONT[size]


def draw_guides(d, res: Result):
    """The aiming ring (the ball at the 'known distance', at the image centre: hold the ball in it to auto-tune) and a
    legend of true-scale circles: the smallest blob the filter keeps, the ball at the known distance, the blob found."""
    p, fr = res.params, res.frame
    f12 = font(12)
    cyan, pink, green = (0, 220, 255), (255, 80, 210), (40, 255, 60)
    g = ring_geometry(fr, p)
    cx, cy, rr = g["u"], g["v"], max(g["r"], 4.0)
    for a in range(0, 360, 20):
        d.arc((cx - rr, cy - rr, cx + rr, cy + rr), a, a + 10, fill=cyan, width=2)
    d.line((cx - 5, cy, cx + 5, cy), fill=cyan)
    d.line((cx, cy - 5, cx, cy + 5), fill=cyan)
    d.text((cx + rr + 4, cy - 7), f"ball at {p['known_m']:.1f} m", fill=cyan, font=f12)
    r_min = math.sqrt(p["min_blob_px"] / math.pi)
    far = g["f_px"] * (p["ball_mm"] / 2000) / effective_min_radius(p)
    rows = [(pink, r_min, f"min blob {int(p['min_blob_px'])} px = r {r_min:.1f}: ball dropped beyond ~{far:.1f} m"),
            (cyan, g["r"], f"ball at {p['known_m']:.1f} m = r {g['r']:.1f} px")]
    if res.det is not None:
        rows.append((green, res.det[2], f"detected r {res.det[2]:.1f} px ({res.det[3]} px)"))
    h = fr.rgb.shape[0]
    y = h - 4
    for col, r, text in reversed(rows):
        rd = min(r, 16.0)
        rh = int(max(18, 2 * rd + 6))
        text += "  (circle capped)" if r > 16 else ""
        tw = d.textlength(text, font=f12)
        d.rectangle((2, y - rh, 48 + tw + 6, y), fill=(0, 0, 0))
        cxl, cyl = 24, y - rh / 2
        d.ellipse((cxl - rd, cyl - rd, cxl + rd, cyl + rd), outline=col, width=1)
        d.point((cxl, cyl), fill=col)
        d.text((46, cyl - 7), text, fill=col, font=f12)
        y -= rh + 2


def render(res: Result, pipe: Pipeline) -> bytes:
    """The annotated frame as a JPEG."""
    p, fr = res.params, res.frame
    if p["view"] == "depth" and fr.depth is not None:
        arr = depth_colormap(fr.depth, fr.depth_scale)
    else:
        arr = fr.rgb
        if p["show_mask"]:
            arr = arr.copy()
            m = ball_mask(arr, pipe.cfg)
            arr[m] = (0.4 * arr[m] + 0.6 * np.array([255, 0, 220])).astype(np.uint8)
    img = Image.fromarray(arr)
    d = ImageDraw.Draw(img)
    f14, f12 = font(14), font(12)
    if res.window is not None:
        d.rectangle(res.window, outline=(0, 220, 255), width=1)
    if p["show_trail"] and len(res.trail) > 1:
        d.line(res.trail, fill=(255, 140, 0), width=2)
    for u, v, b, n in res.small:
        rr = max(b, 3.0)
        d.ellipse((u - rr, v - rr, u + rr, v + rr), outline=(255, 60, 60), width=1)
        d.text((u + rr + 3, v - rr), f"too small {n}px", fill=(255, 60, 60), font=f12)
    if p["show_guides"]:
        draw_guides(d, res)
    for c, is_still in zip(res.cands, res.still):
        u, v, b, n = c
        main = res.det is not None and (c[0] - res.det[0]) ** 2 + (c[1] - res.det[1]) ** 2 < 2.0
        col = (40, 255, 60) if main else ((255, 170, 40) if is_still else (200, 200, 200))
        d.ellipse((u - b, v - b, u + b, v + b), outline=col, width=2 if main else 1)
        d.line((u - 4, v, u + 4, v), fill=col)
        d.line((u, v - 4, u, v + 4), fill=col)
        d.text((u + b + 3, v - b), f"r {b:.1f}  {n}px", fill=col, font=f12)
    lines = [f"{pipe.fps():.0f} fps   detect {res.detect_ms:.1f} ms" + (f"   age {res.frame.age_ms + res.detect_ms:.0f} ms"
                                                                         if res.frame.age_ms is not None else "")]
    if res.det is not None:
        if res.depth:
            lines.append(f"depth  x {res.depth['x']:+.3f}  y {res.depth['y']:+.3f}  z {res.depth['z']:.3f} m")
        if res.size:
            lines.append(f"size   x {res.size['x']:+.3f}  y {res.size['y']:+.3f}  z {res.size['z']:.3f} m")
    else:
        lines.append("no ball")
    y = 4
    for ln in lines:
        w = d.textlength(ln, font=f14)
        d.rectangle((2, y - 1, w + 8, y + 16), fill=(0, 0, 0))
        d.text((5, y), ln, fill=(255, 255, 255), font=f14)
        y += 18
    if simplejpeg is not None:
        return bytes(simplejpeg.encode_jpeg(np.asarray(img), quality=82, colorspace="RGB"))
    import io
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=82)
    return buf.getvalue()


# ---------------------------------------------------------------- the web page

PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8"><title>Ball view</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#14171c;--panel:#1d2229;--ink:#e6e9ee;--dim:#8b95a5;--acc:#4ad66d;--line:#2c3440}
@media (prefers-color-scheme: light){:root{--bg:#f2f4f7;--panel:#fff;--ink:#1a1f26;--dim:#5b6676;--acc:#0a8f36;--line:#d5dbe3}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px system-ui,sans-serif}
main{display:flex;flex-wrap:wrap;gap:16px;padding:16px}
#view{flex:1 1 560px;min-width:0}#view img{width:100%;max-width:960px;border-radius:6px;background:#000;cursor:crosshair;display:block}
@media (min-width:980px){#view{position:sticky;top:16px;align-self:flex-start}}
#side{flex:0 0 340px;display:flex;flex-direction:column;gap:12px}
section{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px 12px}
h2{margin:0 0 6px;font-size:12px;letter-spacing:.06em;text-transform:uppercase;color:var(--dim)}
.k{display:grid;grid-template-columns:130px 1fr 52px;gap:6px;align-items:center;margin:3px 0}
.k label{color:var(--dim)}.k output{text-align:right;font-variant-numeric:tabular-nums}
input[type=range]{width:100%}
.b{display:flex;gap:8px;align-items:flex-start;margin:4px 0;color:var(--dim)}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}td{padding:2px 0}td:first-child{color:var(--dim)}td:last-child{text-align:right}
button,select{background:var(--bg);color:var(--ink);border:1px solid var(--line);border-radius:6px;padding:5px 10px;font:inherit;cursor:pointer}
button:hover{border-color:var(--acc)}textarea{width:100%;height:92px;background:var(--bg);color:var(--ink);border:1px solid var(--line);border-radius:6px;font:12px ui-monospace,monospace}
.good{color:var(--acc)}#err,.bad{color:#ff8a5b}
#knobs h2{margin-top:12px}#knobs h2:first-child{margin-top:0}
canvas{display:block;border-radius:4px;border:1px solid var(--line)}
.swrow{display:flex;gap:10px;margin-top:8px}#chips{flex:1;display:flex;flex-direction:column;gap:6px;color:var(--dim)}
.chip div{display:flex;gap:3px}.chip i{flex:0 0 26px;height:20px;border-radius:3px;border:1px solid var(--line)}.chip span{display:block;font-size:11px;line-height:1.2;margin-top:2px}
.note{font-size:12px;color:var(--dim);margin-top:4px;line-height:1.35}#tunemsg{font-size:12px;margin-top:6px;line-height:1.35}
</style></head><body><main>
<div id="view"><img id="img" src="/stream" alt="camera"><div id="err"></div></div>
<div id="side">
<section><h2>Tracking</h2><table id="tbl"></table></section>
<section><h2>Position (camera frame, m)</h2><table id="pos"></table>
<div style="margin-top:6px;display:flex;gap:6px;flex-wrap:wrap"><button id="cal">calibrate focal to known distance</button><button id="gain" style="display:none">apply suggested size gain</button></div></section>
<section><h2>Auto-tune</h2><div style="display:flex;gap:6px;flex-wrap:wrap"><button id="tune_ring" title="Hold the ball inside the cyan ring at the centre of the image">auto-tune on the ring</button><button id="tune_blob" title="Use the green detected blob as the ball">on the detected blob</button><button id="untune">undo</button></div>
<div class="note">Sets the colour window and the minimum blob size from the ball's own pixels. Hold the ball in the cyan ring (its size is the ball at the 'known distance' below), then press.</div><div id="tunemsg"></div></section>
<section id="knobs"></section>
<section><h2>Settings</h2><div style="display:flex;gap:6px;flex-wrap:wrap"><button id="save" title="Writes the detector settings and every knob to a file on the Pi, and downloads a copy here">save settings</button><button id="load" title="Load the settings last saved for this camera">load saved</button></div>
<div class="note" id="savenote"></div><div id="savemsg" class="note"></div></section>
<section><h2>Display</h2><div class="b"><label>view <select id="view_sel"><option>color</option><option>depth</option></select></label><button id="shot">snapshot</button><button id="reset">reset</button></div><div id="bools"></div></section>
<section><h2>Detector settings (paste into the robot's config)</h2><textarea id="cfg" readonly></textarea></section>
</div></main>
<script>
let S=null,built=false,drag=false;
const $=id=>document.getElementById(id);
const post=(u,b)=>fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})}).then(r=>r.json());
function build(s){
  const groups={};s.spec.forEach(k=>{(groups[k[6]]=groups[k[6]]||[]).push(k)});
  const el=$('knobs');el.innerHTML='';
  for(const g in groups){const h=document.createElement('h2');h.textContent=g;el.appendChild(h);
    groups[g].forEach(([n,l,lo,hi,st,df])=>{const d=document.createElement('div');d.className='k';
      d.innerHTML=`<label for="k_${n}">${l}</label><input type="range" id="k_${n}" min="${lo}" max="${hi}" step="${st}" value="${s.params[n]}"><output id="o_${n}"></output>`;
      el.appendChild(d);const i=d.querySelector('input');
      i.addEventListener('input',()=>{drag=true;$('o_'+n).textContent=(+i.value).toFixed(st<1?2:0);post('/set',{[n]:+i.value})});
      i.addEventListener('change',()=>{drag=false})});
    if(g==='colour'){const w=document.createElement('div');w.innerHTML=`<canvas id="hue" width="316" height="162" title="drag to draw the hue range (the bottom edge is the saturation floor); drag an edge to move it"></canvas>
      <div class="swrow"><canvas id="sv" width="132" height="132" title="click or drag: the point is the dullest and darkest colour accepted (saturation right, value up)"></canvas><div id="chips"></div></div>
      <div class="note" id="swnote"></div>`;el.appendChild(w)}
    if(g==='blob'){const w=document.createElement('div');w.className='note';w.id='sznote';el.appendChild(w)}}
  const bo=$('bools');bo.innerHTML='';
  s.bools.forEach(([n,l])=>{const d=document.createElement('label');d.className='b';
    d.innerHTML=`<input type="checkbox" id="c_${n}"> ${l}`;bo.appendChild(d);
    d.querySelector('input').addEventListener('change',e=>post('/set',{[n]:e.target.checked}))});
  if(!s.has_depth)[...$('view_sel').options].filter(o=>o.value==='depth').forEach(o=>o.remove());
  built=true}
function row(a,b){return `<tr><td>${a}</td><td>${b}</td></tr>`}
const f=(x,n=2)=>x==null?'–':(+x).toFixed(n);
function paint(s){
  if(!built)build(s);
  if(!cdrag)for(const [n,,,,st] of s.spec){const i=$('k_'+n);if(i&&!(drag&&document.activeElement===i)){i.value=s.params[n]}$('o_'+n).textContent=f(s.params[n],st<1?2:0)}
  s.bools.forEach(([n])=>{const c=$('c_'+n);if(c)c.checked=s.params[n]});
  $('view_sel').value=s.params.view;
  const d=s.det;
  $('tbl').innerHTML=row('capture',f(s.fps,1)+' fps')+row('detect (median / p95)',f(s.detect_ms,2)+' / '+f(s.detect_ms_p95,2)+' ms')
   +row('capture → detection',s.age_ms==null?'–':f(s.age_ms,1)+' ms')
   +row('frames with the ball (last 120)',s.hit_rate==null?'–':f(100*s.hit_rate,0)+' %')
   +row('centroid noise (smooth track)',s.noise_px==null?'–':f(s.noise_px,2)+' px')
   +row('blob',d?`${f(d[0],1)}, ${f(d[1],1)}  r ${f(d[2],2)} px  ${d[3]} px²`:'–')
   +row('detector',s.native?'C++ (the robot\'s)':'numpy');
  const z=s.size,dp=s.depth;
  let t='';
  if(dp)t+=row('depth map  x / y / z',`${f(dp.x,3)} / ${f(dp.y,3)} / <b class="good">${f(dp.z,3)}</b>`)+row('  depth fill / noise',`${f(100*dp.fill,0)} % / ${f(1000*dp.sigma_z,1)} mm`);
  if(z)t+=row('ball size  x / y / z',`${f(z.x,3)} / ${f(z.y,3)} / <b class="${dp?'':'good'}">${f(z.z,3)}</b>`)+row('  z per half-pixel of radius',`± ${f(1000*Math.abs(z.z_1px),0)} mm`)+row('  focal length used',f(z.f_px,0)+' px');
  if(dp)t+=row('size vs depth',f(dp.size_vs_depth_pct,1)+' %')+row('size gain that agrees',f(dp.suggest_size_gain,3));
  if(!z)t=row('','no ball in view');
  $('pos').innerHTML=t;$('gain').style.display=dp?'':'none';
  $('cfg').value=JSON.stringify({hue_deg:s.cfg.hue_deg,min_sat:s.cfg.min_sat,min_val:s.cfg.min_val,min_blob_px:s.cfg.min_blob_px,size_gain:+s.cfg.size_gain.toFixed(3),coarse_stride:s.cfg.coarse_stride,ball_mm:s.params.ball_mm,focal_scale:+(+s.params.f_scale).toFixed(3)},null,1);
  $('err').textContent=s.error||'';$('savenote').textContent=s.saved_at?`Last saved for this camera: ${s.saved_at}; the next view starts from it.`:'Nothing saved for this camera yet.';
  if(!cdrag){S=s;swatches(s)}if(s.image&&!window.arSet){window.arSet=1;$('img').style.maxWidth=`min(960px,calc((100vh - 40px)*${s.image[0]/s.image[1]}))`}}
function hsv2rgb(h,s,v){h=((h%360)+360)%360;const c=v*s,x=c*(1-Math.abs((h/60)%2-1)),m=v-c;
  const t=h<60?[c,x,0]:h<120?[x,c,0]:h<180?[0,c,x]:h<240?[0,x,c]:h<300?[x,0,c]:[c,0,x];return t.map(a=>Math.round((a+m)*255))}
const css=c=>`rgb(${c[0]},${c[1]},${c[2]})`;
function swatches(s){
  const p=s.params,hc=(p.hue_min+p.hue_max)/2,bh=s.blob_hsv;
  // the rainbow: hue across, saturation up (value 1); the window is the bright rectangle, the rest dimmed
  const cv=$('hue');if(!cv)return;const x=cv.getContext('2d'),W=cv.width,RH=120,X=h=>Math.max(0,Math.min(360,h))/360*W,Y=sa=>(1-sa)*RH;
  if(!window.rb){window.rb=x.createImageData(W,RH);for(let j=0;j<RH;j++)for(let i=0;i<W;i++){const c=hsv2rgb(i/W*360,1-j/(RH-1),1),k=(j*W+i)*4;rb.data[k]=c[0];rb.data[k+1]=c[1];rb.data[k+2]=c[2];rb.data[k+3]=255}}
  x.putImageData(rb,0,0);
  const x0=X(p.hue_min),x1=X(p.hue_max),y1=Y(p.sat_min);
  x.fillStyle='rgba(16,19,24,.78)';x.fillRect(0,0,x0,RH);x.fillRect(x1,0,W-x1,RH);x.fillRect(x0,y1,x1-x0,RH-y1);
  x.strokeStyle='#fff';x.lineWidth=2;x.strokeRect(x0+1,1,Math.max(2,x1-x0-2),Math.max(2,y1-2));
  if(bh){x.strokeStyle='#ff2db4';x.lineWidth=2;const a=X(bh.h[0]),b=X(bh.h[2]);x.strokeRect(a,Y(bh.s[2]),Math.max(3,b-a),Math.max(3,Y(bh.s[0])-Y(bh.s[2])));
    x.fillStyle='#ff2db4';x.beginPath();x.arc(X(bh.h[1]),Y(bh.s[1]),3,0,7);x.fill()}
  x.fillStyle=getComputedStyle(document.body).backgroundColor;x.fillRect(0,RH,W,cv.height-RH);
  x.fillStyle='#8b95a5';x.font='10px system-ui';x.textBaseline='top';x.textAlign='left';
  for(let h=0;h<=360;h+=60)x.fillText(h+'°',Math.min(W-22,Math.max(0,X(h)-9)),RH+3);
  x.fillStyle='#fff';x.textAlign='center';
  x.fillText(Math.round(p.hue_min)+'°',Math.min(W-14,Math.max(14,x0)),RH+17);x.fillText(Math.round(p.hue_max)+'°',Math.min(W-14,Math.max(14,x1)),RH+17);
  x.textAlign='left';x.fillStyle='#8b95a5';x.fillText('saturation ↑ '+p.sat_min.toFixed(2),2,RH+31);
  // saturation x value square at the middle hue: the accepted corner is bright
  const sv=$('sv'),y=sv.getContext('2d'),N=sv.width,im=y.createImageData(N,N);
  for(let j=0;j<N;j++)for(let i=0;i<N;i++){const sa=i/(N-1),va=1-j/(N-1),ok=sa>=p.sat_min&&va>=p.val_min,c=hsv2rgb(hc,sa,va),k=(j*N+i)*4,d=ok?1:.22;
    im.data[k]=c[0]*d+(ok?0:14);im.data[k+1]=c[1]*d+(ok?0:16);im.data[k+2]=c[2]*d+(ok?0:20);im.data[k+3]=255}
  y.putImageData(im,0,0);
  if(bh){y.strokeStyle='#ff2db4';y.lineWidth=2;const a=bh.s[0]*(N-1),b=bh.s[2]*(N-1),c=(1-bh.v[2])*(N-1),e=(1-bh.v[0])*(N-1);
    y.strokeRect(a,c,Math.max(2,b-a),Math.max(2,e-c));y.fillStyle='#fff';y.fillRect(bh.s[1]*(N-1)-2,(1-bh.v[1])*(N-1)-2,4,4)}
  const chip=(label,cols)=>`<div class="chip"><div>${cols.map(c=>`<i style="background:${css(c)}"></i>`).join('')}</div><span>${label}</span></div>`;
  const row=(sa,va)=>[p.hue_min,hc,p.hue_max].map(h=>hsv2rgb(h,sa,va));
  $('chips').innerHTML=chip('dullest it passes (hue min · mid · max)',row(p.sat_min,p.val_min))+chip('most vivid',row(1,1))
    +(bh?chip(`blob mean: H ${Math.round(bh.h[1])}° S ${bh.s[1].toFixed(2)} V ${bh.v[1].toFixed(2)}`,[bh.rgb]):'<div>no blob: nothing to compare</div>');
  $('swnote').innerHTML='<b>Draw on the rainbow</b> (hue across, saturation up): drag a rectangle, its sides are hue min / max and its bottom is the saturation floor; drag a side to move just that. <b>In the square</b> (saturation →, value ↑ at the middle hue) click or drag the dullest, darkest colour to accept. The bright part passes.'+(bh?' <b style="color:#ff2db4">Pink</b> marks where the blob\'s own pixels fall (5th–95th percentile): a window that doesn\'t cover it is losing part of the ball.':'');
  const l=s.limits;if(l&&$('sznote'))$('sznote').innerHTML=`Minimum blob = a disc of radius ${f(l.r_min_px,1)} px; the whole-frame search at stride ${s.params.coarse_stride} also needs r ≈ ${f(0.5*s.params.coarse_stride,1)} px or more (${f(l.r_eff_px,1)} px counts). The ball (${s.params.ball_mm} mm) shrinks to that at about <b>${f(l.range_m,1)} m</b>: beyond it the detector drops the ball. (A real ball's dim rim costs more than this clean-disc figure.) The legend in the picture draws it to scale; ‘ring the blobs rejected as too small’ shows what it throws away.`}
let cdrag=null,sent=0,cmode,ch0,cs0;
const clamp=v=>Math.max(0,Math.min(1,v));
function setColour(o,final){Object.assign(S.params,o);swatches(S);
  for(const n in o){const i=$('k_'+n);if(i){i.value=o[n];$('o_'+n).textContent=(+o[n]).toFixed(n.startsWith('hue')?0:2)}}
  const t=performance.now();if(final||t-sent>40){sent=t;return post('/set',o)}}
function colourMove(cv,e,final){
  const r=cv.getBoundingClientRect(),fx=(e.clientX-r.left)/r.width,fy=(e.clientY-r.top)/r.height,P=S.params;
  if(cv.id==='sv')return setColour({sat_min:+clamp(fx).toFixed(2),val_min:+clamp(1-fy).toFixed(2)},final);
  const h=clamp(fx)*360,sa=clamp(1-fy*cv.height/120),o={};
  if(cmode==='l')o.hue_min=Math.round(Math.min(h,P.hue_max));
  else if(cmode==='r')o.hue_max=Math.round(Math.max(h,P.hue_min));
  else if(cmode==='b')o.sat_min=+sa.toFixed(2);
  else{let lo=Math.min(ch0,h),hi=Math.max(ch0,h);if(hi-lo<6){lo=ch0-3;hi=ch0+3}       // a click picks a hue +-3 deg
    o.hue_min=Math.max(0,Math.round(lo));o.hue_max=Math.min(360,Math.round(hi));o.sat_min=+Math.min(cs0,sa).toFixed(2)}
  return setColour(o,final)}
$('knobs').addEventListener('pointerdown',e=>{const cv=e.target;if(!S||(cv.id!=='hue'&&cv.id!=='sv'))return;
  cdrag=cv;cv.setPointerCapture(e.pointerId);
  if(cv.id==='hue'){const r=cv.getBoundingClientRect(),px=(e.clientX-r.left)/r.width*cv.width,py=(e.clientY-r.top)/r.height*cv.height,P=S.params,
      xl=P.hue_min/360*cv.width,xr=P.hue_max/360*cv.width,yb=(1-P.sat_min)*120,g=7;
    if(Math.abs(px-xl)<g&&py<yb+g)cmode='l';else if(Math.abs(px-xr)<g&&py<yb+g)cmode='r';
    else if(Math.abs(py-yb)<g&&px>xl&&px<xr)cmode='b';
    else{cmode='n';ch0=clamp(px/cv.width)*360;cs0=clamp(1-py/120)}}
  colourMove(cv,e,false)});
$('knobs').addEventListener('pointermove',e=>{if(cdrag&&e.target===cdrag)colourMove(cdrag,e,false)});
const colourUp=e=>{const cv=cdrag;if(!cv)return;colourMove(cv,e,true);setTimeout(()=>{cdrag=null},120)};
$('knobs').addEventListener('pointerup',colourUp);$('knobs').addEventListener('pointercancel',colourUp);
$('save').addEventListener('click',async()=>{const r=await post('/save');$('savemsg').textContent=r.message;$('savemsg').className='note '+(r.ok?'good':'bad');
  if(r.ok){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(r.settings,null,1)],{type:'application/json'}));
    a.download=`ball_view_${r.settings.source}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),2000)}});
$('load').addEventListener('click',async()=>{const r=await post('/load');$('savemsg').textContent=r.message;$('savemsg').className='note '+(r.ok?'good':'bad')});
async function tune(t){const r=await post('/autotune',{target:t});$('tunemsg').textContent=r.message||r.error;$('tunemsg').className=r.ok?'good':'bad'}
$('tune_ring').addEventListener('click',()=>tune('ring'));$('tune_blob').addEventListener('click',()=>tune('blob'));
$('untune').addEventListener('click',async()=>{const r=await post('/undo');$('tunemsg').textContent=r.message;$('tunemsg').className=r.ok?'good':'bad'});
async function poll(){try{paint(await (await fetch('/status')).json())}catch(e){}setTimeout(poll,250)}
poll();
$('img').addEventListener('click',e=>{if(!S||!S.params.click_picks||!S.image)return;const r=e.target.getBoundingClientRect();
  post('/pick',{x:(e.clientX-r.left)/r.width,y:(e.clientY-r.top)/r.height})});
$('view_sel').addEventListener('change',e=>post('/set',{view:e.target.value}));
$('cal').addEventListener('click',()=>post('/calibrate',{}));
$('gain').addEventListener('click',()=>S&&S.depth&&post('/set',{size_gain:S.depth.suggest_size_gain}));
$('shot').addEventListener('click',()=>{const a=document.createElement('a');a.href='/snapshot.jpg?'+Date.now();a.download='ball_view.jpg';a.click()});
$('reset').addEventListener('click',()=>post('/reset'));
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    pipe: Pipeline = None
    display_fps: float = 20.0

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj, default=float).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/status":
            self._json(self.pipe.status())
        elif path == "/snapshot.jpg":
            r = self.pipe.latest
            if r is None:
                return self._json({"error": "no frame yet"}, 503)
            jpg = render(r, self.pipe)
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
        elif path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq, period = -1, 1.0 / self.display_fps
            try:
                while not self.pipe.stop.is_set():
                    r = self.pipe.wait(seq, 1.0)
                    if r is None:
                        continue
                    seq, t0 = r.seq, time.monotonic()
                    jpg = render(r, self.pipe)
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    time.sleep(max(0.0, period - (time.monotonic() - t0)))
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._json({"error": "bad json"}, 400)
        pipe, path = self.pipe, self.path
        if path == "/set":
            for k, v in body.items():
                pipe.params.set(k, v)
            self._json({"ok": True})
        elif path == "/pick":
            r = pipe.latest
            if r is None:
                return self._json({"error": "no frame"}, 503)
            h, w = r.frame.rgb.shape[:2]
            x, y = int(body["x"] * w), int(body["y"] * h)
            patch = r.frame.rgb[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
            if patch.size == 0:
                return self._json({"error": "outside"}, 400)
            win = pick_window(patch)
            for k in ("hue_min", "hue_max", "sat_min", "val_min"):
                pipe.params.set(k, win[k])
            self._json(win)
        elif path == "/autotune":
            r = pipe.latest
            if r is None:
                return self._json({"ok": False, "message": "no frame yet"}, 503)
            out = autotune(r, "blob" if body.get("target") == "blob" else "ring")
            if out["ok"]:
                cur = pipe.params.get()
                pipe.undo = {k: cur[k] for k in out["applied"]}
                for k, v in out["applied"].items():
                    pipe.params.set(k, v)
            self._json(out)
        elif path == "/save":
            self._json(save_settings(pipe))
        elif path == "/load":
            self._json(load_settings(pipe))
        elif path == "/undo":
            if not pipe.undo:
                return self._json({"ok": False, "message": "nothing to undo"})
            for k, v in pipe.undo.items():
                pipe.params.set(k, v)
            pipe.undo = None
            self._json({"ok": True, "message": "Back to the colour window and minimum blob size from before the auto-tune."})
        elif path == "/calibrate":
            r = pipe.latest
            if r is None or r.det is None:
                return self._json({"error": "no ball in view"}, 409)
            p = r.params
            f_base = (r.frame.fx + r.frame.fy) / 2
            pipe.params.set("f_scale", p["known_m"] * r.det[2] / (p["ball_mm"] / 2000) / f_base)
            self._json({"ok": True})
        elif path == "/reset":
            for k, s in pipe.params.spec.items():
                pipe.params.set(k, s[5])
            for k, _, d in BOOLS:
                pipe.params.set(k, d)
            pipe.params.set("view", "color")
            self._json({"ok": True})
        else:
            self._json({"error": "not found"}, 404)


def make_source(a):
    if a.source == "synthetic":
        return SyntheticSource()
    if a.source == "realsense":
        w, h = (int(x) for x in a.size.lower().split("x"))
        return RealSenseSource((w, h), a.fps or 30)
    w, h = (int(x) for x in a.size.lower().split("x"))
    return PiCamSource((w, h), a.mode, a.lens)


def serve(pipe: Pipeline, port: int, display_fps: float = 20.0, host: str = "0.0.0.0") -> ThreadingHTTPServer:
    Handler.pipe, Handler.display_fps = pipe, display_fps
    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    return srv


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--source", choices=["picam", "realsense", "synthetic"], default="synthetic")
    ap.add_argument("--port", type=int, default=8085)
    ap.add_argument("--size", default=None, help="picam 768x432, realsense 640x480")
    ap.add_argument("--fps", type=int, default=0, help="realsense frame rate (30)")
    ap.add_argument("--mode", choices=["robot", "wide"], default="robot",
                    help="picam: robot = 1536x864 @ 120 fps (the camera head's mode), wide = 2304x1296 @ 56 fps")
    ap.add_argument("--lens", choices=list(LENS_MM), default="standard", help="picam: Module 3 standard or wide lens")
    ap.add_argument("--display-fps", type=float, default=20.0)
    ap.add_argument("--numpy", action="store_true", help="the numpy detector instead of the C++ one")
    ap.add_argument("--settings-dir", default=str(DEFAULT_SETTINGS_DIR), help="where 'save settings' writes (<source>.json)")
    ap.add_argument("--fresh", action="store_true", help="don't start from the saved settings")
    a = ap.parse_args()
    a.size = a.size or ("640x480" if a.source == "realsense" else "768x432")
    src = make_source(a)
    pipe = Pipeline(src, Params(src), native=not a.numpy)
    pipe.settings_dir = Path(a.settings_dir)
    if not a.fresh:
        print(load_settings(pipe)["message"], flush=True)
    threading.Thread(target=pipe.run, daemon=True).start()
    srv = serve(pipe, a.port, a.display_fps)
    print(f"ball view ({a.source}, detector {type(pipe.det).__name__}): http://{__import__('socket').gethostname()}.local:{a.port}/",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop.set()
        time.sleep(0.2)
        src.close()


if __name__ == "__main__":
    main()
