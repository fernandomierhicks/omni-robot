"""camera_snap: the robot's cameras for the console's Cameras tab over plain HTTP, no ROS: single frames, live video,
and AprilTags found in either.

Runs on the computer the cameras are plugged into (today the Pi, rpi.md: the Arducam IMX708 and the D435;
ros/tools/cam_snap.sh starts it there). Pictures never go through rosbridge. A snapshot is one JPEG per request, so
the page decides the rate; live video is a multipart stream that runs only while a page holds it open.

    GET /cameras                              JSON: the cameras, their streams and capture profiles, open or not,
                                              the profile and settings in use, the last error, the tag detector
    GET /snap?cam=CAM&stream=S&w=640&q=70     one JPEG (w: width in px, 0 = as captured; q: JPEG quality);
                                              header X-Frame-Info (JSON): size, frame age, depth statistics, tags
    GET /stream?cam=&stream=&w=&q=&fps=       live video: multipart/x-mixed-replace JPEGs, each part with its own
                                              X-Frame-Info header (a plain <img> shows it; the page reads the parts
                                              with fetch to draw the tags on the frame they were found in). fps caps
                                              the rate (0: as fast as the camera, the detector and the link allow)
    GET /depth?u=0.5&v=0.5                    the range (m) at a point of the last depth frame sent (u, v in 0..1)
    GET /set?cam=&profile=&emitter=&focus=    change a camera: its capture profile (reopens it), the D435's IR
                                              projector (0 / 1), the Arducam's focus (auto, or a distance in m)

    snap and stream also take: profile=P (the camera reopens in it if it isn't already), tags=FAMILY (tag25h9,
    tag36h11, tag16h5: find AprilTags, omni_console/apriltag_view.py), size=0.2 (the tag's black square, m: the
    pose's scale), dec=2 (the detector's quad decimation: faster, a little less reach)

Cameras: arducam (stream color) and realsense (color, ir: the left imager, depth: coloured near red to far blue,
black where there is no depth). A camera opens on its first request and closes after --idle seconds without one, so
another program (ball_view, the catch head) can have it back; a camera another program holds answers 503 with the
reason. --fake serves test patterns with two AprilTags in them, without cameras (the tests, the page on a PC).

    ~/.venvs/cam/bin/python camera_snap.py [--port 8086] [--idle 20] [--fake]
"""
from __future__ import annotations

import argparse
import io
import itertools
import json
import math
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
from PIL import Image

PORT = 8086
RETRY_S = 3.0                       # a camera that failed to open isn't tried again sooner (each snap would retry)
BOUNDARY = "omniframe"
_SEQ = itertools.count(1)             # frame numbers, increasing across reopenings (a stream waits for a newer one)

# capture profiles: what the camera is opened in. The first is the default (and what a snapshot opens it in)
PI_PROFILES = {
    "wide30": dict(label="full field 1152×648, 30 fps", size=(1152, 648), raw=(2304, 1296), fps=30),
    "wide56": dict(label="full field 1152×648, 56 fps", size=(1152, 648), raw=(2304, 1296), fps=56),
    "wide_full": dict(label="full field 2304×1296, 30 fps", size=(2304, 1296), raw=(2304, 1296), fps=30),
    "crop100": dict(label="2/3 crop 768×432, 100 fps", size=(768, 432), raw=(1536, 864), fps=100),
}
RS_PROFILES = {
    "vga15": dict(label="640×480, 15 fps", size=(640, 480), fps=15),
    "vga30": dict(label="640×480, 30 fps", size=(640, 480), fps=30),
    "wvga60": dict(label="848×480, 60 fps", size=(848, 480), fps=60),
    "hd30": dict(label="1280×720, 30 fps", size=(1280, 720), fps=30),
}


class CameraError(RuntimeError):
    pass


def _av():
    """apriltag_view: in the package, or next to this file (the Pi's copy)."""
    try:
        from omni_console import apriltag_view
    except ImportError:
        import apriltag_view
    return apriltag_view


def depth_colormap(depth: np.ndarray, scale: float, near: float = 0.2, far: float = 4.0) -> np.ndarray:
    """Depth as colour: near red, through yellow, green and cyan, to blue far; no depth = black
    (characterization/ball_view's palette)."""
    z = depth.astype(np.float32) * scale
    t = np.clip((z - near) / max(far - near, 1e-3), 0, 1)
    stops = np.array([0, 0.25, 0.5, 0.75, 1.0])
    pal = np.array([[255, 40, 40], [255, 230, 40], [60, 220, 80], [40, 200, 230], [60, 60, 255]], np.float32)
    out = np.stack([np.interp(t, stops, pal[:, c]) for c in range(3)], axis=-1).astype(np.uint8)
    out[depth == 0] = 0
    return out


def depth_stats(depth: np.ndarray, scale: float) -> dict:
    """What the picture can't say exactly: the share of pixels with depth, the 5th / 50th / 95th percentile range
    and the range at the image centre (median of its middle 5%), metres."""
    ok = depth[depth > 0].astype(np.float32) * scale
    h, w = depth.shape
    c = depth[int(h * 0.475):int(h * 0.525) + 1, int(w * 0.475):int(w * 0.525) + 1]
    c = c[c > 0]
    out = dict(valid=round(float(ok.size / depth.size), 3), centre_m=round(float(np.median(c)) * scale, 3) if c.size else None)
    if ok.size:
        p5, p50, p95 = np.percentile(ok, [5, 50, 95])
        out.update(p5_m=round(float(p5), 3), median_m=round(float(p50), 3), p95_m=round(float(p95), 3))
    return out


def range_at(depth: np.ndarray, scale: float, u: float, v: float, half: int = 3) -> float | None:
    """The median depth (m) of the (2 half + 1)^2 pixels around (u, v) in 0..1 image coordinates; None: no depth."""
    h, w = depth.shape
    x, y = int(min(max(u, 0.0), 1.0) * (w - 1)), int(min(max(v, 0.0), 1.0) * (h - 1))
    win = depth[max(0, y - half):y + half + 1, max(0, x - half):x + half + 1]
    win = win[win > 0]
    return round(float(np.median(win)) * scale, 3) if win.size else None


def _resize(arr: np.ndarray, width: int) -> np.ndarray:
    if not 0 < width < arr.shape[1]:
        return arr
    h = max(1, round(arr.shape[0] * width / arr.shape[1]))
    try:
        import cv2
        return cv2.resize(arr, (width, h), interpolation=cv2.INTER_AREA)
    except ImportError:
        return np.asarray(Image.fromarray(arr).resize((width, h), Image.BILINEAR))


def encode(arr: np.ndarray, stream: str, width: int = 640, quality: int = 70, depth_scale: float = 0.001,
           near: float = 0.2, far: float = 4.0) -> bytes:
    """A frame (H x W x 3 RGB, H x W mono, or H x W uint16 depth) as a JPEG, scaled down to `width` (0: as is).
    libjpeg-turbo through simplejpeg when it's there (the Pi: a 1152 px frame in ~10 ms), else Pillow."""
    if stream == "depth":
        arr = depth_colormap(_resize(arr, width) if width else arr, depth_scale, near, far)
    else:
        arr = _resize(arr, width)
    q = int(min(max(quality, 10), 95))
    try:
        import simplejpeg
        a = np.ascontiguousarray(arr if arr.ndim == 3 else arr[..., None])
        return simplejpeg.encode_jpeg(a, quality=q, colorspace="RGB" if a.shape[2] == 3 else "GRAY")
    except ImportError:
        buf = io.BytesIO()
        Image.fromarray(arr).save(buf, "JPEG", quality=q)
        return buf.getvalue()


# ---------------------------------------------------------------- frames and the cameras

class Frame:
    """One capture: its streams' pictures (copied out once, on first use), each stream's pinhole (for tag poses),
    and for the D435 the depth image and the transforms to the depth camera (to check a tag's range)."""

    def __init__(self, seq: int, t: float, getters: dict, info: dict, cams: dict | None = None, depth=None):
        self.seq, self.t, self.info, self.cams = seq, t, info, cams or {}
        self.getters, self.cache, self.depth = getters, {}, depth
        self.tags: dict = {}                               # (stream, family, size, dec) -> (tags, ms): shared by viewers
        self.lock = threading.Lock()

    def image(self, stream: str) -> np.ndarray:
        with self.lock:
            if stream not in self.cache:
                self.cache[stream] = np.ascontiguousarray(self.getters[stream]())
            return self.cache[stream]


class DepthRef:
    """The D435's depth image of a frame, its pinhole and each stream's transform into the depth camera."""

    def __init__(self, get, scale: float, cam, extr: dict):
        self.get, self.scale, self.cam, self.extr = get, scale, cam, extr

    def check(self, stream: str, R: np.ndarray, t: np.ndarray, size: float) -> dict | None:
        """Depth over the tag's inner 60% against the pose's distance along the depth camera's axis (both m), so a
        wrong tag size or focal length shows as a ratio away from 1."""
        if stream not in self.extr:
            return None
        Rx, tx = self.extr[stream]
        s = 0.3 * size
        P = (np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0], [0, 0, 0]]) @ R.T + t) @ Rx.T + tx
        uv = self.cam.project(P)
        if not np.all(np.isfinite(uv)):
            return None
        d = self.get()
        h, w = d.shape
        x0, y0 = np.clip(np.floor(uv[:4].min(0)).astype(int), 0, [w - 1, h - 1])
        x1, y1 = np.clip(np.ceil(uv[:4].max(0)).astype(int) + 1, 1, [w, h])
        win = d[y0:y1, x0:x1]
        ok = win[win > 0]
        if not ok.size:
            return dict(depth_z=None, pose_z=round(float(P[4, 2]), 4), valid=0.0)
        z = float(np.median(ok)) * self.scale
        return dict(depth_z=round(z, 4), pose_z=round(float(P[4, 2]), 4), ratio=round(z / float(P[4, 2]), 4),
                    valid=round(ok.size / max(win.size, 1), 3))


class Source:
    """A camera's capture thread: keeps the latest frame; readers wait for one newer than what they had."""
    warmup = 0                                             # frames dropped after opening (auto exposure settling)

    def _start_feed(self):
        self.cond = threading.Condition()
        self.frame: Frame | None = None
        self.n = 0
        self.error: str | None = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _publish(self, make):
        self.n += 1
        if self.n <= self.warmup:
            return
        with self.cond:
            self.frame = make(next(_SEQ))
            self.error = None
            self.cond.notify_all()

    def wait(self, after: int = 0, timeout: float = 5.0) -> Frame:
        with self.cond:
            self.cond.wait_for(lambda: (self.frame is not None and self.frame.seq > after) or self.stop.is_set(), timeout)
            f = self.frame
        if f is None or f.seq <= after:
            raise CameraError(f"no new frame in {timeout:.0f} s ({self.error or 'timed out'})")
        return f

    def apply(self, settings: dict):
        pass

    def _join(self):
        self.stop.set()
        with self.cond:
            self.cond.notify_all()
        self.thread.join(3.0)


class PiCam(Source):
    """The Arducam / Camera Module 3 (IMX708) through Picamera2. Its focal length is the lens's (4.74 mm over
    1.4 µm pixels = 3386 px across the 4608 px sensor), scaled by the sensor crop libcamera reports for each frame:
    a nominal pinhole, not a calibration (ball_view does the same)."""
    streams = ("color",)
    profiles = PI_PROFILES
    F_FULL_PX = 4.74e-3 / 1.4e-6

    def __init__(self, profile: str = "wide30", settings: dict | None = None):
        from picamera2 import Picamera2
        p = self.profiles[profile]
        self.size = p["size"]
        self.cam = Picamera2()
        try:
            us = int(round(1e6 / p["fps"]))
            self.cam.configure(self.cam.create_video_configuration(
                main={"size": p["size"], "format": "BGR888"},      # R, G, B in memory
                raw={"size": p["raw"]}, controls={"FrameDurationLimits": (us, us)}, buffer_count=4, queue=False))
            self.cam.start()
            self.apply(settings or {})
        except Exception:
            self.cam.close()
            raise
        self._start_feed()

    def apply(self, settings: dict):
        f = settings.get("focus")
        if f == "auto":
            self.cam.set_controls({"AfMode": 2})                   # continuous
        elif f:
            self.cam.set_controls({"AfMode": 0, "LensPosition": 1.0 / max(float(f), 0.05)})   # dioptres

    def _run(self):
        Pinhole = _av().Pinhole
        w, h = self.size
        while not self.stop.is_set():
            try:
                req = self.cam.capture_request()
                try:
                    md = req.get_metadata()
                    img = req.make_array("main").copy()
                finally:
                    req.release()
            except Exception as e:
                self.error = _why(e)
                time.sleep(0.2)
                continue
            crop = md.get("ScalerCrop")
            f = self.F_FULL_PX * w / (crop[2] if crop else 4608)
            sts = md.get("SensorTimestamp")
            t = sts / 1e9 if sts else time.monotonic()            # both CLOCK_MONOTONIC
            info = dict(exposure_us=md.get("ExposureTime"), gain=round(md.get("AnalogueGain", 0.0), 2),
                        lens_dioptres=None if md.get("LensPosition") is None else round(md["LensPosition"], 3),
                        focal_px=round(f, 1))
            self._publish(lambda n, img=img: Frame(n, t, {"color": lambda: img}, info,
                                                   {"color": Pinhole(f, f, w / 2, h / 2, w, h)}))

    def close(self):
        self._join()
        self.cam.stop()
        self.cam.close()


class RealSense(Source):
    """The D435: colour, the left IR imager and depth, all at the profile's size and rate. The thread keeps the
    latest frame set (the camera's own queue would otherwise hand out stale frames). Each stream's pinhole is the
    camera's factory calibration; depth is registered to the left imager, and the colour camera's transform to it
    comes from the factory extrinsics, so a tag's range can be checked against depth on either picture."""
    streams = ("color", "ir", "depth")
    profiles = RS_PROFILES
    warmup = 20                    # the colour camera's auto exposure (Nov 3, on the Pi: frame 4 nearly black, 21 on settled)

    def __init__(self, profile: str = "vga15", settings: dict | None = None):
        import pyrealsense2 as rs
        Pinhole = _av().Pinhole
        self.rs = rs
        p = self.profiles[profile]
        (w, h), fps = p["size"], p["fps"]
        self.pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
        cfg.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, fps)
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        prof = self.pipe.start(cfg)
        try:
            self.sensor = prof.get_device().first_depth_sensor()
            self.depth_scale = self.sensor.get_depth_scale()
            sp = dict(color=prof.get_stream(rs.stream.color), ir=prof.get_stream(rs.stream.infrared, 1),
                      depth=prof.get_stream(rs.stream.depth))
            self.cams = {}
            for k, s in sp.items():
                i = s.as_video_stream_profile().get_intrinsics()
                self.cams[k] = Pinhole(i.fx, i.fy, i.ppx, i.ppy, i.width, i.height)
            e = sp["color"].get_extrinsics_to(sp["depth"])
            self.extr = {"color": (np.array(e.rotation).reshape(3, 3).T, np.array(e.translation)),
                         "ir": (np.eye(3), np.zeros(3))}       # depth is in the left imager's frame
            self.apply(settings or {})
        except Exception:
            self.pipe.stop()
            raise
        self._start_feed()

    def apply(self, settings: dict):
        if settings.get("emitter") is not None:
            self.sensor.set_option(self.rs.option.emitter_enabled, 1.0 if int(settings["emitter"]) else 0.0)

    def _run(self):
        while not self.stop.is_set():
            try:
                fs = self.pipe.wait_for_frames(2000)
            except RuntimeError as e:                  # no frames for 2 s: say so, keep trying
                self.error = str(e)
                continue
            t = time.monotonic()
            c, ir, d = fs.get_color_frame(), fs.get_infrared_frame(1), fs.get_depth_frame()
            if not (c and ir and d):
                continue
            get_d = lambda d=d: np.asanyarray(d.get_data())          # noqa: E731
            info = dict(frame=c.get_frame_number(), depth_scale=self.depth_scale)
            for k, f in (("exposure_us", c), ("ir_exposure_us", ir)):
                try:
                    info[k] = int(f.get_frame_metadata(self.rs.frame_metadata_value.actual_exposure))
                except RuntimeError:
                    pass
            self._publish(lambda n, c=c, ir=ir, get_d=get_d: Frame(
                n, t, {"color": lambda: np.asanyarray(c.get_data()), "ir": lambda: np.asanyarray(ir.get_data()),
                       "depth": get_d}, info, self.cams, DepthRef(get_d, self.depth_scale, self.cams["depth"], self.extr)))

    def close(self):
        self._join()
        self.pipe.stop()


# two tags for the test patterns (OpenCV's drawMarker, 1-cell black border included): tag25h9 id 0, tag36h11 id 0
FAKE_TAGS = {
    "tag25h9": ["0000000", "0100010", "0111110", "0010010", "0110100", "0101010", "0000000"],
    "tag36h11": ["00000000", "00010000", "00110100", "00001010", "00001100", "01011100", "01010110", "00000000"],
}


class Fake(Source):
    """Test patterns with every stream's format, at the profile's rate: a gradient with two AprilTags circling
    slowly in it (tag25h9 id 0 on the left, tag36h11 id 0 on the right, fronto-parallel, 70 and 80 px on a 600 px
    focal length); depth is a known ramp (1.0 m at the left edge to 3.0 m at the right)."""

    def __init__(self, streams, profiles=None, profile: str | None = None, settings: dict | None = None):
        Pinhole = _av().Pinhole
        self.streams = tuple(streams)
        self.profiles = profiles or {"vga": dict(label="640×480, 30 fps", size=(640, 480), fps=30)}
        p = self.profiles.get(profile) or next(iter(self.profiles.values()))
        self.size, self.fps = (640, 480), min(p["fps"], 30)
        w, h = self.size
        self.cam = Pinhole(600.0, 600.0, w / 2, h / 2, w, h)
        self._start_feed()

    def pattern(self, k: int):
        w, h = self.size
        xx = np.linspace(0, 1, w, dtype=np.float32)[None, :].repeat(h, 0)
        yy = np.linspace(0, 1, h, dtype=np.float32)[:, None].repeat(w, 1)
        gray = (90 + 80 * xx * yy).astype(np.uint8)
        a = k / max(self.fps, 1) * 0.6
        cell = 10                                           # px: a 0.2 m tag25h9 at 1.71 m, a tag36h11 at 1.5 m
        for j, bits in enumerate(FAKE_TAGS.values()):
            n = len(bits)
            x0 = int(w * (0.3 + 0.4 * j) + 25 * math.cos(a)) - cell * n // 2
            y0 = int(h * 0.5 + 25 * math.sin(a)) - cell * n // 2
            gray[y0 - cell:y0 + cell * (n + 1), x0 - cell:x0 + cell * (n + 1)] = 255       # the white quiet zone
            for r, row in enumerate(bits):
                for c, b in enumerate(row):
                    gray[y0 + r * cell:y0 + (r + 1) * cell, x0 + c * cell:x0 + (c + 1) * cell] = 255 if b == "1" else 20
        rgb = np.stack([gray, gray, (0.6 * gray + 100 * xx).astype(np.uint8)], -1)
        depth = np.round((1.0 + 2.0 * xx) / 0.001).astype(np.uint16)
        return rgb, gray, depth

    def _run(self):
        while not self.stop.is_set():
            k = self.n + 1
            rgb, gray, depth = self.pattern(k)
            t = time.monotonic()
            self._publish(lambda n, rgb=rgb, gray=gray, depth=depth: Frame(
                n, t, {"color": lambda: rgb, "ir": lambda: gray, "depth": lambda: depth},
                dict(frame=n, depth_scale=0.001), {s: self.cam for s in ("color", "ir")}))
            self.stop.wait(1.0 / self.fps)

    def close(self):
        self._join()


class Camera:
    """One camera, opened on demand in a capture profile and closed when idle. Settings (the D435's projector, the
    Arducam's focus) stay across reopenings."""

    def __init__(self, name: str, label: str, streams, opener, idle_s: float = 20.0, profiles: dict | None = None,
                 settings: dict | None = None):
        self.name, self.label, self.streams, self.opener, self.idle_s = name, label, tuple(streams), opener, idle_s
        self.profiles = profiles or {}
        self.profile = next(iter(self.profiles), None)
        self.settings = dict(settings or {})
        self.lock = threading.Lock()
        self.src: Source | None = None
        self.used = self.failed = 0.0
        self.error: str | None = None
        self.last_depth: tuple | None = None           # (array, scale) of the last depth frame sent
        self.streaming = 0                             # live streams open on it: it isn't idle while one runs
        self.workers: dict[tuple, TagWorker] = {}      # live streams' tag finders, per (stream, family, size, dec)

    def _open(self, profile: str | None):
        """Under the lock: the source, opened (or reopened) in `profile`."""
        now = self.used = time.monotonic()
        if profile and profile in self.profiles and profile != self.profile:
            self._close()
            self.profile = profile
        if self.src is None:
            if self.error and now - self.failed < RETRY_S:
                raise CameraError(self.error)
            try:
                self.src = self.opener(self.profile, self.settings)
            except Exception as e:                     # held by another program, unplugged, no driver here
                self.error, self.failed = _why(e), now
                raise CameraError(self.error) from e
            self.error = None
        return self.src

    def frame(self, profile: str | None = None, after: int = 0, timeout: float = 5.0) -> Frame:
        with self.lock:
            src = self._open(profile)
        try:
            f = src.wait(after, timeout)
        except CameraError as e:
            with self.lock:
                self.error, self.failed = str(e), time.monotonic()
                if self.src is src:
                    self._close()
            raise
        self.used = time.monotonic()
        return f

    def grab(self, stream: str, profile: str | None = None):
        """(picture, info) of the latest frame: the snapshot's path."""
        if stream not in self.streams:
            raise KeyError(stream)
        f = self.frame(profile)
        arr = f.image(stream)
        info = dict(f.info, age_ms=round((time.monotonic() - f.t) * 1e3, 1))
        if stream == "depth":
            self.last_depth = (arr, info.get("depth_scale", 0.001))
        return arr, info

    def set(self, profile: str | None = None, **settings):
        with self.lock:
            self.settings.update({k: v for k, v in settings.items() if v is not None})
            if profile and profile in self.profiles and profile != self.profile:
                self._close()
                self.profile = profile
            elif self.src is not None:
                self.src.apply(settings)

    def close_if_idle(self, now: float):
        with self.lock:
            if self.src is not None and not self.streaming and now - self.used > self.idle_s:
                self._close()

    def _close(self):
        src, self.src = self.src, None
        if src is not None:
            try:
                src.close()
            except Exception:
                pass

    def describe(self) -> dict:
        return dict(name=self.name, label=self.label, streams=list(self.streams), open=self.src is not None,
                    error=self.error, profile=self.profile, settings=self.settings, streaming=self.streaming,
                    profiles=[dict(name=k, label=v["label"]) for k, v in self.profiles.items()])


def _why(e: Exception) -> str:
    if isinstance(e, ImportError):
        return f"{e.name or 'its driver'} isn't installed on this computer"
    return f"{type(e).__name__}: {e}".strip()


def cameras(fake: bool = False, idle_s: float = 20.0) -> dict[str, Camera]:
    def opener(cls):
        if fake:
            return lambda profile, settings: Fake(cls.streams, cls.profiles, profile, settings)
        return cls
    return {c.name: c for c in (
        Camera("arducam", "Arducam IMX708", PiCam.streams, opener(PiCam), idle_s, PI_PROFILES, {"focus": None}),
        Camera("realsense", "RealSense D435", RealSense.streams, opener(RealSense), idle_s, RS_PROFILES, {"emitter": 1}))}


# ---------------------------------------------------------------- tags

def frame_tags(f: Frame, stream: str, family: str, size: float, dec: float) -> dict:
    """The tags in one stream of a frame (found once, whoever asks), with the depth check where there is depth."""
    av = _av()
    key = (stream, family, size, dec)
    with f.lock:
        hit = f.tags.get(key)
    if hit is None:
        tags, ms = av.find(f.image(stream), f.cams.get(stream), family, size, dec)
        if f.depth is not None:
            for tg in tags:
                if "R" in tg:
                    tg["depth"] = f.depth.check(stream, np.array(tg["R"]), np.array(tg["xyz"]), size)
        for tg in tags:
            tg.pop("R", None)
        hit = (tags, ms)
        with f.lock:
            f.tags[key] = hit
    tags, ms = hit
    c = f.cams.get(stream)
    return dict(tags=tags, detect_ms=round(ms, 1), family=family, size_m=size, decimate=dec,
                focal_px=round(c.fx, 1) if c else None)


class TagWorker:
    """Finds tags in the newest frame of one stream, again and again, while a live stream wants them, so the video
    goes at the camera's rate and the tags at the detector's: each picture carries the newest result, the frame it
    came from and how much older that frame is. Stops 3 s after the last stream asked."""

    def __init__(self, cam: Camera, stream: str, key: tuple):
        self.cam, self.stream, self.key = cam, stream, key
        self.result: dict | None = None
        self.wanted = time.monotonic()
        threading.Thread(target=self._run, daemon=True).start()

    def latest(self) -> dict | None:
        self.wanted = time.monotonic()
        return self.result

    def _run(self):
        seq = 0
        try:
            while time.monotonic() - self.wanted < 3.0:
                try:
                    f = self.cam.frame(after=seq, timeout=2.0)
                    r = frame_tags(f, self.stream, *self.key)
                except Exception as e:                      # the camera went, or no detector: say so, retry
                    self.result = dict(tag_error=_why(e))
                    time.sleep(0.5)
                    continue
                seq = f.seq
                self.result = dict(r, tags_seq=f.seq, tags_t=f.t)
        finally:
            with self.cam.lock:
                if self.cam.workers.get((self.stream, *self.key)) is self:
                    del self.cam.workers[(self.stream, *self.key)]


def tag_worker(cam: Camera, stream: str, family: str, size: float, dec: float) -> TagWorker:
    with cam.lock:
        k = (stream, family, size, dec)
        w = cam.workers.get(k)
        if w is None:
            w = cam.workers[k] = TagWorker(cam, stream, k[1:])
        return w


# ---------------------------------------------------------------- the server

class Handler(BaseHTTPRequestHandler):
    cams: dict[str, Camera] = {}

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[-1] for k, v in parse_qs(url.query).items()}
        num = lambda k, d: float(q[k]) if q.get(k, "").replace(".", "", 1).isdigit() else d     # noqa: E731
        if url.path == "/cameras":
            try:
                av = _av()
                tags = dict(detector=av.backend(), families=list(av.FAMILIES))
            except Exception as e:                          # no scipy here: pictures without tags
                tags = dict(detector=None, families=[], error=_why(e))
            return self._json(200, dict(cameras=[c.describe() for c in self.cams.values()], tags=tags))
        if url.path in ("/snap", "/stream"):
            cam, stream = self.cams.get(q.get("cam", "")), q.get("stream", "color")
            if cam is None or stream not in cam.streams:
                return self._json(404, dict(error=f"no camera/stream {q.get('cam')}/{stream}"))
            opts = dict(profile=q.get("profile") or None, w=int(num("w", 640)), q=int(num("q", 70)),
                        near=num("near", 0.2), far=num("far", 4.0), fps=num("fps", 0.0),
                        tags=q.get("tags") if stream != "depth" and q.get("tags") else None,
                        size=num("size", 0.2), dec=num("dec", 2.0))
            if url.path == "/snap":
                return self._snap(cam, stream, opts)
            return self._stream(cam, stream, opts)
        if url.path == "/set":
            cam = self.cams.get(q.get("cam", ""))
            if cam is None:
                return self._json(404, dict(error=f"no camera {q.get('cam')}"))
            emitter = q.get("emitter")
            focus = q.get("focus")
            try:
                cam.set(q.get("profile") or None, emitter=None if emitter is None else int(emitter),
                        focus=None if focus is None else ("auto" if focus == "auto" else float(focus)))
            except Exception as e:
                return self._json(400, dict(error=_why(e)))
            return self._json(200, cam.describe())
        if url.path == "/depth":
            cam = next((c for c in self.cams.values() if c.last_depth is not None), None)
            if cam is None:
                return self._json(404, dict(error="no depth frame sent yet"))
            arr, scale = cam.last_depth
            return self._json(200, dict(range_m=range_at(arr, scale, num("u", 0.5), num("v", 0.5))))
        self._json(404, dict(error="see /cameras, /snap?cam=&stream=, /stream?cam=&stream=, /set?cam=, /depth?u=&v="))

    def _picture(self, cam: Camera, f: Frame, stream: str, o: dict, live: bool = False) -> tuple[bytes, dict]:
        arr = f.image(stream)
        info = dict(f.info, width=int(arr.shape[1]), height=int(arr.shape[0]), seq=f.seq, profile=cam.profile)
        if o["tags"] and live:                              # the newest result the stream's tag finder has
            r = tag_worker(cam, stream, o["tags"], o["size"], o["dec"]).latest()
            if r:
                r = dict(r)
                if "tags_t" in r:
                    r["tags_lag_ms"] = round((f.t - r.pop("tags_t")) * 1e3, 1)
                info.update(r)
        elif o["tags"]:
            try:
                info.update(frame_tags(f, stream, o["tags"], o["size"], o["dec"]), tags_seq=f.seq, tags_lag_ms=0.0)
            except Exception as e:                          # no detector here: the picture still goes
                info["tag_error"] = _why(e)
        t0 = time.perf_counter()
        body = encode(arr, stream, o["w"], o["q"], f.info.get("depth_scale", 0.001), o["near"], o["far"])
        info.update(encode_ms=round((time.perf_counter() - t0) * 1e3, 1), age_ms=round((time.monotonic() - f.t) * 1e3, 1))
        if stream == "depth":
            info.update(depth_stats(arr, info["depth_scale"]), near_m=o["near"], far_m=o["far"])
            cam.last_depth = (arr, info["depth_scale"])
        return body, info

    def _snap(self, cam: Camera, stream: str, o: dict):
        try:
            f = cam.frame(o["profile"])
            body, info = self._picture(cam, f, stream, o)
        except CameraError as e:
            return self._json(503, dict(error=str(e)))
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Frame-Info", json.dumps(info))
        self.end_headers()
        self.wfile.write(body)

    def _stream(self, cam: Camera, stream: str, o: dict):
        try:
            f = cam.frame(o["profile"])
        except CameraError as e:
            return self._json(503, dict(error=str(e)))
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        period = 1.0 / o["fps"] if o["fps"] > 0 else 0.0
        t_next = 0.0
        with cam.lock:
            cam.streaming += 1
        try:
            while True:
                if period:                                  # the cap: wait out the period, then the latest frame
                    wait = t_next - time.monotonic()
                    if wait > 0:
                        time.sleep(wait)
                    t_next = max(t_next, time.monotonic()) + period
                try:                                        # the profile only opened it: /set may change it now
                    f = cam.frame(None, after=f.seq)
                    body, info = self._picture(cam, f, stream, o, live=True)
                except CameraError as e:                    # tell the page, keep trying (another program, a hiccup)
                    body, info = b"", dict(error=str(e))
                    time.sleep(1.0)
                head = (f"--{BOUNDARY}\r\nContent-Type: {'image/jpeg' if body else 'text/plain'}\r\n"
                        f"Content-Length: {len(body)}\r\nX-Frame-Info: {json.dumps(info)}\r\n\r\n").encode()
                self.wfile.write(head + body + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass                                            # the page closed it
        finally:
            with cam.lock:
                cam.streaming -= 1
                cam.used = time.monotonic()

    def _json(self, code: int, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")          # the console page is served from elsewhere
        self.send_header("Access-Control-Expose-Headers", "X-Frame-Info")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass


def serve(cams: dict[str, Camera], port: int = PORT, host: str = "0.0.0.0") -> ThreadingHTTPServer:
    """Start the server (and the idle closer) in background threads; returns it (shutdown() stops it)."""
    httpd = ThreadingHTTPServer((host, port), type("SnapHandler", (Handler,), {"cams": cams}))
    httpd.daemon_threads = True

    def closer():
        while True:
            time.sleep(1.0)
            for c in cams.values():
                c.close_if_idle(time.monotonic())

    threading.Thread(target=closer, daemon=True).start()
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--idle", type=float, default=20.0, help="close a camera after this many seconds without a request")
    ap.add_argument("--fake", action="store_true", help="test patterns, no cameras")
    a = ap.parse_args()
    cams = cameras(a.fake, a.idle)
    httpd = serve(cams, a.port)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))         # cam_snap.sh stop: close the cameras on the way out
    print(f"camera_snap on :{a.port} ({'fake' if a.fake else ', '.join(cams)}): GET /cameras, /snap, /stream, /set",
          flush=True)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        for c in cams.values():
            with c.lock:
                c._close()


if __name__ == "__main__":
    main()
