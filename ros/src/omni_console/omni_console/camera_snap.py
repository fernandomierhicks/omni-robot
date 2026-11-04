"""camera_snap: single frames from the robot's cameras for the console's Cameras tab, over plain HTTP, no ROS.

Runs on the computer the cameras are plugged into (today the Pi, rpi.md: the Arducam IMX708 and the D435;
ros/tools/cam_snap.sh starts it there) and answers one JPEG per request, so the page decides the rate: on demand,
or one every few seconds, never a stream that runs whether anyone looks or not. Like the Vision tab's video,
pictures never go through rosbridge.

    GET /cameras                              JSON: the cameras, their streams, open or not, the last error
    GET /snap?cam=CAM&stream=S&w=640&q=70     one JPEG (w: width in px, 0 = as captured; q: JPEG quality);
                                              header X-Frame-Info (JSON): size, frame age, depth statistics
    GET /depth?u=0.5&v=0.5                    the range (m) at a point of the last depth frame sent (u, v in 0..1)

Cameras: arducam (stream color) and realsense (color, ir: the left imager, emitter dots included; depth: coloured
near red to far blue, black where there is no depth). A camera opens on its first request and closes after --idle
seconds without one, so another program (ball_view, the catch head) can have it back; a camera another program
holds answers 503 with the reason. --fake serves test patterns without cameras (the tests, the page on a PC).

    ~/.venvs/cam/bin/python camera_snap.py [--port 8086] [--idle 20] [--fake]
"""
from __future__ import annotations

import argparse
import io
import json
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


class CameraError(RuntimeError):
    pass


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


def encode(arr: np.ndarray, stream: str, width: int = 640, quality: int = 70, depth_scale: float = 0.001,
           near: float = 0.2, far: float = 4.0) -> bytes:
    """A frame (H x W x 3 RGB, H x W mono, or H x W uint16 depth) as a JPEG, scaled down to `width` (0: as is)."""
    img = Image.fromarray(depth_colormap(arr, depth_scale, near, far) if stream == "depth" else arr)
    if 0 < width < img.width:
        img = img.resize((width, max(1, round(img.height * width / img.width))), Image.BILINEAR)
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=int(min(max(quality, 10), 95)))
    return buf.getvalue()


# ---------------------------------------------------------------- the cameras

class PiCam:
    """The Arducam / Camera Module 3 (IMX708) through Picamera2: the full field of view (the 2304x1296 sensor mode)
    scaled to 1152x648, at a low frame rate (the ISP and auto exposure keep running between snaps)."""
    streams = ("color",)

    def __init__(self, size=(1152, 648), fps: float = 10.0):
        from picamera2 import Picamera2
        self.cam = Picamera2()
        try:
            us = int(round(1e6 / fps))
            self.cam.configure(self.cam.create_video_configuration(
                main={"size": size, "format": "BGR888"},          # R, G, B in memory
                raw={"size": (2304, 1296)}, controls={"FrameDurationLimits": (us, us)}, buffer_count=4))
            self.cam.start()
        except Exception:
            self.cam.close()
            raise

    def read(self, stream: str):
        req = self.cam.capture_request()
        try:
            md = req.get_metadata()
            img = np.ascontiguousarray(req.make_array("main"))
        finally:
            req.release()
        sts = md.get("SensorTimestamp")
        return img, dict(age_ms=round((time.monotonic_ns() - sts) / 1e6, 1) if sts else None,
                         exposure_us=md.get("ExposureTime"), gain=round(md.get("AnalogueGain", 0.0), 2))

    def close(self):
        self.cam.stop()
        self.cam.close()


class RealSense:
    """The D435: colour, the left IR imager and depth, 640x480 at a low frame rate. A thread keeps the latest frame
    set (the camera's own queue would otherwise hand out stale frames); a snap copies one stream out of it."""
    streams = ("color", "ir", "depth")
    WARMUP = 20

    def __init__(self, size=(640, 480), fps: int = 15):
        import pyrealsense2 as rs
        self.pipe, cfg = rs.pipeline(), rs.config()
        cfg.enable_stream(rs.stream.color, size[0], size[1], rs.format.rgb8, fps)
        cfg.enable_stream(rs.stream.infrared, 1, size[0], size[1], rs.format.y8, fps)
        cfg.enable_stream(rs.stream.depth, size[0], size[1], rs.format.z16, fps)
        prof = self.pipe.start(cfg)
        self.depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
        self.lock, self.fs, self.t, self.n, self.error = threading.Lock(), None, 0.0, 0, None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            try:
                fs = self.pipe.wait_for_frames(2000)
            except RuntimeError as e:                  # no frames for 2 s: say so, keep trying
                self.error = str(e)
                continue
            with self.lock:
                self.fs, self.t, self.n, self.error = fs, time.monotonic(), self.n + 1, None

    def read(self, stream: str):
        # after opening, the colour camera's auto exposure needs ~20 frames (Nov 3, on the Pi: frame 4 was nearly
        # black, frame 21 on settled)
        t_end = time.monotonic() + 5.0
        while self.n < self.WARMUP and time.monotonic() < t_end:
            time.sleep(0.02)
        with self.lock:
            fs, t = self.fs, self.t
        if fs is None:
            raise CameraError(f"no frames from the D435 ({self.error or 'timed out'})")
        f = dict(color=fs.get_color_frame, ir=lambda: fs.get_infrared_frame(1), depth=fs.get_depth_frame)[stream]()
        info = dict(age_ms=round((time.monotonic() - t) * 1e3, 1), frame=f.get_frame_number())
        if stream == "depth":
            info["depth_scale"] = self.depth_scale
        return np.asanyarray(f.get_data()).copy(), info

    def close(self):
        self.stop.set()
        self.thread.join(3.0)
        self.pipe.stop()


class Fake:
    """Test patterns with every stream's format; depth is a known ramp (1.0 m at the left edge to 3.0 m at the right)."""

    def __init__(self, streams, size=(640, 480)):
        self.streams, self.size, self.k = tuple(streams), size, 0

    def read(self, stream: str):
        w, h = self.size
        self.k += 1
        xx = np.linspace(0, 1, w, dtype=np.float32)[None, :].repeat(h, 0)
        yy = np.linspace(0, 1, h, dtype=np.float32)[:, None].repeat(w, 1)
        if stream == "depth":
            return np.round((1.0 + 2.0 * xx) / 0.001).astype(np.uint16), dict(age_ms=0.0, frame=self.k, depth_scale=0.001)
        if stream == "ir":
            return (255 * (0.5 + 0.5 * np.sin(20 * xx + self.k)) * yy).astype(np.uint8), dict(age_ms=0.0, frame=self.k)
        rgb = np.stack([255 * xx, 255 * yy, 255 * (0.5 + 0.5 * np.sin(self.k / 3)) + 0 * xx], -1)
        return rgb.astype(np.uint8), dict(age_ms=0.0, frame=self.k)

    def close(self):
        pass


class Camera:
    """One camera, opened on demand and closed when idle; one request at a time."""

    def __init__(self, name: str, label: str, streams, opener, idle_s: float = 20.0):
        self.name, self.label, self.streams, self.opener, self.idle_s = name, label, tuple(streams), opener, idle_s
        self.lock = threading.Lock()
        self.src = None
        self.used = self.failed = 0.0
        self.error: str | None = None
        self.last_depth: tuple | None = None           # (array, scale) of the last depth frame sent

    def grab(self, stream: str):
        if stream not in self.streams:
            raise KeyError(stream)
        with self.lock:
            now = self.used = time.monotonic()
            if self.src is None:
                if self.error and now - self.failed < RETRY_S:
                    raise CameraError(self.error)
                try:
                    self.src = self.opener()
                except Exception as e:                 # held by another program, unplugged, no driver here
                    self.error, self.failed = _why(e), now
                    raise CameraError(self.error) from e
                self.error = None
            try:
                arr, info = self.src.read(stream)
            except Exception as e:
                self.error, self.failed = _why(e), now
                self._close()
                raise CameraError(self.error) from e
            if stream == "depth":
                self.last_depth = (arr, info.get("depth_scale", 0.001))
            return arr, info

    def close_if_idle(self, now: float):
        with self.lock:
            if self.src is not None and now - self.used > self.idle_s:
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
                    error=self.error)


def _why(e: Exception) -> str:
    if isinstance(e, ImportError):
        return f"{e.name or 'its driver'} isn't installed on this computer"
    return f"{type(e).__name__}: {e}".strip()


def cameras(fake: bool = False, idle_s: float = 20.0) -> dict[str, Camera]:
    opener = lambda cls: (lambda: Fake(cls.streams)) if fake else cls          # noqa: E731
    return {c.name: c for c in (
        Camera("arducam", "Arducam IMX708", PiCam.streams, opener(PiCam), idle_s),
        Camera("realsense", "RealSense D435", RealSense.streams, opener(RealSense), idle_s))}


# ---------------------------------------------------------------- the server

class Handler(BaseHTTPRequestHandler):
    cams: dict[str, Camera] = {}

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[-1] for k, v in parse_qs(url.query).items()}
        num = lambda k, d: float(q[k]) if q.get(k, "").replace(".", "", 1).isdigit() else d     # noqa: E731
        if url.path == "/cameras":
            return self._json(200, dict(cameras=[c.describe() for c in self.cams.values()]))
        if url.path == "/snap":
            cam, stream = self.cams.get(q.get("cam", "")), q.get("stream", "color")
            if cam is None or stream not in cam.streams:
                return self._json(404, dict(error=f"no camera/stream {q.get('cam')}/{stream}"))
            try:
                arr, info = cam.grab(stream)
            except CameraError as e:
                return self._json(503, dict(error=str(e)))
            t0 = time.perf_counter()
            near, far = num("near", 0.2), num("far", 4.0)
            body = encode(arr, stream, int(num("w", 640)), int(num("q", 70)), info.get("depth_scale", 0.001), near, far)
            info.update(width=int(arr.shape[1]), height=int(arr.shape[0]), encode_ms=round((time.perf_counter() - t0) * 1e3, 1))
            if stream == "depth":
                info.update(depth_stats(arr, info["depth_scale"]), near_m=near, far_m=far)
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("X-Frame-Info", json.dumps(info))
            self.end_headers()
            self.wfile.write(body)
            return
        if url.path == "/depth":
            cam = next((c for c in self.cams.values() if c.last_depth is not None), None)
            if cam is None:
                return self._json(404, dict(error="no depth frame sent yet"))
            arr, scale = cam.last_depth
            return self._json(200, dict(range_m=range_at(arr, scale, num("u", 0.5), num("v", 0.5))))
        self._json(404, dict(error="see /cameras, /snap?cam=&stream=, /depth?u=&v="))

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
    ap.add_argument("--idle", type=float, default=20.0, help="close a camera after this many seconds without a snap")
    ap.add_argument("--fake", action="store_true", help="test patterns, no cameras")
    a = ap.parse_args()
    cams = cameras(a.fake, a.idle)
    httpd = serve(cams, a.port)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))         # cam_snap.sh stop: close the cameras on the way out
    print(f"camera_snap on :{a.port} ({'fake' if a.fake else ', '.join(cams)}): GET /cameras, /snap?cam=&stream=", flush=True)
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
