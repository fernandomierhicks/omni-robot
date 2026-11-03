"""Record thrown balls on a RealSense D435: every imager at once (colour, depth, the IR pair), for offline replay and analysis.

The robot's own colour detector on the RGB stream is the trigger (as record_throws.py does on the Pi camera). A ring buffer per
stream keeps the last `--pre` seconds; when a ball moves it keeps `--post` more and writes the throw to data/<session>/tNNN/
(session.py's format: frames.npy = colour, depth.npy, ir1.npy, ir2.npy, each with its stamps on one shared clock). Capture never
stops while a throw is written. No labels are asked for (`--ask` brings the y / n prompt back): the throw's own best arc
is the reference (rs_analyze.py). Runs on the Pi's blue USB 3 port (the D435 links only at USB 2.1 on the PC, README.md).

  bash characterization/throw_track/tt.sh rsrecord --name bucket --distance 4.5 --count 30
  bash characterization/throw_track/tt.sh rscheck                 # 8 s: each stream's rate, lost frames, clock offsets
  bash characterization/throw_track/tt.sh rssynthetic             # no camera: rendered throws through the same code

Streams. Colour and the depth / IR pair are two librealsense pipelines (one pipeline would hold the 90 fps frames for the 60 fps
colour, +5 ms median on this Pi, characterization/realsense/README.md); if the camera refuses two, it falls back to one. All
timestamps are librealsense's global time (mid-exposure on the host's clock); the host-arrival time is stored next to each.
UNTESTED on the real camera until the first `rscheck` (the D435 was off the bus when this was written): see README.md.
"""
from __future__ import annotations

import argparse
import json
import math
import platform
import queue
import sys
import threading
import time
from collections import deque, namedtuple
from dataclasses import asdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for p in (HERE.parent / "realsense", HERE.parents[1] / "characterization" / "realsense"):
    if p.is_dir():
        sys.path.insert(0, str(p))
sys.path.insert(0, str(HERE))

import arc2d                                               # noqa: E402
import session as S                                        # noqa: E402
from record_throws import Trigger, detector_config, make_detector      # noqa: E402

SETTINGS = Path.home() / "omni_ws" / "ball_view_settings" / "realsense.json"
COLOR_MODES = [(848, 480, 60), (640, 480, 60), (848, 480, 30), (640, 480, 30)]
Item = namedtuple("Item", "t_ns host_ns counter arr info")      # info: (exposure us, gain, laser power)


# ----------------------------------------------------------------------------------------------- ring buffers
class Ring:
    """The last `keep_s` seconds of one stream (by the frames' own time), and its running health counters."""

    def __init__(self, keep_s: float):
        self.keep_ns = int(keep_s * 1e9)
        self.q: deque = deque()
        self.lock = threading.Lock()
        self.n = self.lost = 0
        self.t_first = self.t_last = None
        self.max_dt = 0
        self.last_counter = None

    def push(self, it: Item):
        with self.lock:
            self.q.append(it)
            while self.q and self.q[0].t_ns < it.t_ns - self.keep_ns:
                self.q.popleft()
        self.n += 1
        if self.last_counter is not None and it.counter and self.last_counter:
            self.lost += max(0, it.counter - self.last_counter - 1)
        self.last_counter = it.counter
        if self.t_last is not None:
            self.max_dt = max(self.max_dt, it.t_ns - self.t_last)
        self.t_first = it.t_ns if self.t_first is None else self.t_first
        self.t_last = it.t_ns

    def window(self, t0: int, t1: int) -> list:
        with self.lock:
            return [it for it in self.q if t0 <= it.t_ns <= t1]

    def latest(self, n: int = 1) -> list:
        with self.lock:
            return list(self.q)[-n:]

    def health(self) -> dict:
        span = (self.t_last - self.t_first) * 1e-9 if self.n > 1 else 0.0
        return dict(n=self.n, fps=(self.n - 1) / span if span > 0 else None, lost=self.lost, max_gap_ms=self.max_dt * 1e-6)


def arrays_of(items: list) -> dict:
    """A ring window as save_throw's stream entry."""
    return dict(frames=[it.arr for it in items],
                stamps=np.array([[it.t_ns, it.host_ns, it.counter] for it in items], dtype=np.int64),
                info=np.array([it.info for it in items], dtype=np.float32))


# ----------------------------------------------------------------------------------------------- the rigs
class RigBase:
    streams: list
    rings: dict
    calib: dict
    domain: str | None = None

    def skip_color(self, n: int):
        """Read n colour frames (about n / fps seconds of the stream) and drop them: a settle, not a wall-clock wait, so a
        synthetic rig's first throw isn't consumed while it renders."""
        for _ in range(n):
            try:
                self.next_color(1.0)
            except queue.Empty:
                return

    def stills(self, n: int = 15) -> dict:
        """Median of the last n frames of each non-colour stream, the last colour frame: the empty scene (floor, back wall)."""
        out = {}
        for name, ring in self.rings.items():
            items = ring.latest(n)
            if not items:
                continue
            if name == "color":
                out[name] = items[-1].arr
            else:
                out[name] = np.median(np.stack([it.arr for it in items]), axis=0).astype(items[0].arr.dtype)
        return out


class SyntheticRig(RigBase):
    """synth_rs.py throws on a loop, as if from the camera: frames come as fast as they render, stamps are virtual (and
    monotonic), the streams arrive in time order so the rings fill as they would."""

    def __init__(self, a, streams, keep_s):
        from synth_rs import RSSynthConfig
        w, h = (int(v) for v in a.size.lower().split("x"))
        self.cfg = RSSynthConfig(width=w, height=h, color_fps=a.fps, depth_fps=a.depth_fps, pitch_deg=a.synthetic_pitch)
        self.streams, self.distance, self.gap = streams, a.distance or 4.5, a.synthetic_gap
        self.rings = {n: Ring(keep_s) for n in streams}
        self.domain = "synthetic"
        self.pending: deque = deque()
        self.k = 0
        self.t_off = time.time_ns()
        self.counters = {n: 0 for n in streams}
        self.calib = {}

    def start(self) -> dict:
        cam = self.cfg.camera()
        intr = dict(width=cam.width, height=cam.height, fx=cam.f, fy=cam.f, ppx=cam.cx, ppy=cam.cy, model="none", coeffs=[0.0] * 5)
        sizes = {n: (self.cfg.width, self.cfg.height, self.cfg.color_fps if n == "color" else self.cfg.depth_fps) for n in self.streams}
        self.calib = dict(device=dict(name="synthetic D435", serial_number="synthetic"), depth_scale=0.001, baseline_m=self.cfg.baseline,
                          streams={n: dict(width=w, height=h, fps=f, intrinsics=intr) for n, (w, h, f) in sizes.items()},
                          extrinsics=dict(depth_to_color=dict(R=np.eye(3).ravel().tolist(), t=[0.0, 0.0, 0.0]),
                                          ir1_to_ir2=dict(R=np.eye(3).ravel().tolist(), t=[self.cfg.baseline, 0.0, 0.0])),
                          timestamp_domain="synthetic", options={}, shutter=dict(color="synthetic", ir="synthetic"))
        return self.calib

    def _fill(self):
        from synth import default_throw
        from synth_rs import render_streams
        ccfg = self.cfg.color_cfg()
        arc, T = default_throw(ccfg, self.distance, seed=self.k)
        out, _ = render_streams(self.cfg, arc, T * 1.02, pre=self.gap, seed=self.k)
        items = []
        for name, (frames, ts) in out.items():
            if name not in self.rings:
                continue
            exp = (self.cfg.exposure if name == "color" else self.cfg.ir_exposure) * 1e6
            for img, t in zip(frames, ts):
                self.counters[name] += 1
                items.append((self.t_off + int((t + self.gap) * 1e9), name,
                              Item(0, time.time_ns(), self.counters[name], np.ascontiguousarray(img), (exp, 1.0, 1.0 if name != "color" else 0.0))))
        for t_ns, name, it in sorted(items, key=lambda x: x[0]):
            self.pending.append((name, it._replace(t_ns=t_ns)))
        self.t_off += int((self.gap + T * 1.02) * 1e9)
        self.k += 1

    def next_color(self, timeout: float = 1.0) -> Item:
        while True:
            if not self.pending:
                self._fill()
            name, it = self.pending.popleft()
            self.rings[name].push(it)
            if name == "color":
                return it

    def close(self):
        pass


class RealSenseRig(RigBase):
    """The D435 through pyrealsense2: colour in one pipeline, depth + IR in another, a thread each."""

    NAMES = {"color": ("color", 0), "depth": ("depth", 0), "ir1": ("infrared", 1), "ir2": ("infrared", 2)}

    def __init__(self, a, streams, keep_s):
        import rs_common as C
        self.C, self.rs = C, C.need_rs()
        self.a, self.streams = a, streams
        self.rings = {n: Ring(keep_s) for n in streams}
        self.cq: queue.Queue = queue.Queue(maxsize=600)
        self.dropped_color = 0
        self.dev = C.find_device(a.serial)
        self.pipes, self.threads, self.stop_ev = [], [], threading.Event()
        self.calib = {}
        self.timeouts = 0
        self.domain = None

    # -- setup
    def _color_mode(self):
        w0, h0 = (int(v) for v in self.a.size.lower().split("x"))
        for w, h, f in [(w0, h0, int(self.a.fps))] + COLOR_MODES:
            if self.C.mode_supported(self.dev, [("color", w, h, f)]):
                return w, h, f
        raise SystemExit("no usable colour mode on this link (USB 2?): " + self.C.USB2_HINT)

    def _apply_options(self, exposure_us, gain, awb, white_balance=None):
        C, a = self.C, self.a
        cs, ds = C.color_sensor(self.dev), C.depth_sensor(self.dev)
        C.set_opt(cs, "global_time_enabled", 1)
        C.set_opt(cs, "enable_auto_white_balance", 1 if awb else 0)
        if white_balance and not awb:
            C.set_opt(cs, "white_balance", white_balance)                   # rs_autotune.py's value, locked in the room's light
        C.set_opt(cs, "auto_exposure_priority", 0)                    # never trade frame rate for exposure
        C.set_opt(cs, "enable_auto_exposure", 0 if exposure_us else 1)
        if exposure_us:
            C.set_opt(cs, "exposure", max(1, round(exposure_us / 100)))        # the colour sensor counts in 100 us
        if gain is not None:
            C.set_opt(cs, "gain", gain)
        if a.emitter == "alternate":
            C.set_opt(ds, "emitter_on_off", 1)
            C.configure_stereo(self.dev, emitter=True, exposure_us=a.ir_exposure_us or None)
        else:
            C.set_opt(ds, "emitter_on_off", 0)
            C.configure_stereo(self.dev, emitter=(a.emitter == "on"), exposure_us=a.ir_exposure_us or None)

    def start(self, exposure_us=0, gain=None, awb=False, white_balance=None) -> dict:
        C, rs, a = self.C, self.rs, self.a
        serial = C.info(self.dev, "serial_number")
        if not C.usb3(self.dev):
            print("WARNING: the camera is on USB", C.info(self.dev, "usb_type_descriptor"), "- " + C.USB2_HINT)
        cw, ch, cf = self._color_mode()
        dw, dh = (int(v) for v in a.depth_size.lower().split("x"))
        dmode = [(kind, dw, dh, int(a.depth_fps)) for kind, name in (("depth", "depth"), ("ir", "ir1"), ("ir2", "ir2")) if name in self.streams]
        if dmode and not C.mode_supported(self.dev, dmode):
            raise SystemExit(f"{dw}x{dh}@{a.depth_fps:g} not offered on this link ({C.info(self.dev, 'usb_type_descriptor')})")
        cfg_d, cfg_c = rs.config(), rs.config()
        for c in (cfg_d, cfg_c):
            c.enable_device(serial)
        C.enable_streams(cfg_d, dmode)
        cfg_c.enable_stream(rs.stream.color, cw, ch, rs.format.rgb8, cf)          # RGB in memory, what the detector takes
        profs = []
        self.mode = "two pipelines"
        try:
            for cfg in ([cfg_d] if dmode else []) + [cfg_c]:
                pipe = rs.pipeline()
                profs.append(pipe.start(cfg))
                self.pipes.append(pipe)
        except RuntimeError as e:
            print(f"two pipelines refused ({e}); falling back to one (IR / depth wait for the 60 fps colour)")
            for p in self.pipes:
                p.stop()
            self.pipes, profs = [], []
            cfg = rs.config()
            cfg.enable_device(serial)
            C.enable_streams(cfg, dmode)
            cfg.enable_stream(rs.stream.color, cw, ch, rs.format.rgb8, cf)
            pipe = rs.pipeline()
            profs.append(pipe.start(cfg))
            self.pipes.append(pipe)
            self.mode = "one pipeline"
        self._apply_options(exposure_us, gain, awb, white_balance)
        for pipe in self.pipes:
            t = threading.Thread(target=self._loop, args=(pipe,), daemon=True)
            t.start()
            self.threads.append(t)
        self.calib = self._calibration(profs, (cw, ch, cf), (dw, dh, a.depth_fps))
        return self.calib

    def _calibration(self, profs, color_mode, depth_mode) -> dict:
        C, rs = self.C, self.rs
        sel = {}
        for prof in profs:
            for sp in prof.get_streams():
                kind = (str(sp.stream_type()).split(".")[-1], sp.stream_index())
                for name, k in self.NAMES.items():
                    if k == (kind[0], kind[1] if kind[0] == "infrared" else 0):
                        sel[name] = sp
        out = dict(device=C.device_summary(self.dev), mode=self.mode, streams={}, extrinsics={}, options={})
        for name, sp in sel.items():
            vp = sp.as_video_stream_profile()
            out["streams"][name] = dict(width=vp.width(), height=vp.height(), fps=vp.fps(), intrinsics=C.intrinsics_dict(vp.get_intrinsics()))
        # Extrinsics between streams of two pipelines are not available from the pipelines' own profiles ("Requested extrinsics are
        # not available", measured on this D435 + Pi): the device's sensor profiles of the same modes do carry them.
        dev_prof = {}
        for sensor in self.dev.query_sensors():
            for p in sensor.get_stream_profiles():
                if not p.is_video_stream_profile():
                    continue
                kind = (str(p.stream_type()).split(".")[-1], p.stream_index())
                for name, k in self.NAMES.items():
                    v = p.as_video_stream_profile()
                    want = out["streams"].get(name)
                    if k == (kind[0], kind[1] if kind[0] == "infrared" else 0) and want and (v.width(), v.height(), v.fps()) == (want["width"], want["height"], want["fps"]) \
                            and (name != "color" or str(v.format()).endswith("rgb8")):
                        dev_prof.setdefault(name, p)

        def ext(a, b):
            if a in dev_prof and b in dev_prof:
                e = dev_prof[a].get_extrinsics_to(dev_prof[b])
                return dict(R=list(e.rotation), t=list(e.translation))
        for key, (a, b) in dict(depth_to_color=("depth", "color"), ir1_to_ir2=("ir1", "ir2"), ir1_to_color=("ir1", "color")).items():
            e = ext(a, b)
            if e:
                out["extrinsics"][key] = e
        out["depth_scale"] = self.dev.first_depth_sensor().get_depth_scale()
        out["baseline_m"] = C.ir_baseline_m(self.dev)
        cs, ds = C.color_sensor(self.dev), C.depth_sensor(self.dev)
        out["options"] = dict(color_exposure_100us=C.get_opt(cs, "exposure"), color_gain=C.get_opt(cs, "gain"),
                              color_auto_exposure=C.get_opt(cs, "enable_auto_exposure"), color_awb=C.get_opt(cs, "enable_auto_white_balance"),
                              depth_exposure_us=C.get_opt(ds, "exposure"), depth_gain=C.get_opt(ds, "gain"),
                              emitter=C.get_opt(ds, "emitter_enabled"), emitter_on_off=C.get_opt(ds, "emitter_on_off"),
                              laser_power=C.get_opt(ds, "laser_power"), asic_temperature=C.get_opt(ds, "asic_temperature"))
        out["shutter"] = dict(color="rolling (OV2740)", ir="global (OV9282)")
        return out

    # -- capture
    def _name_of(self, f):
        p = f.get_profile()
        kind, idx = str(p.stream_type()).split(".")[-1], p.stream_index()
        for name, k in self.NAMES.items():
            if k == (kind, idx if kind == "infrared" else 0):
                return name
        return None

    def _item(self, f, host_ns) -> Item:
        ft = self.C.frame_times(f)
        t_ms = ft["t_mid_ms"] if ft["t_mid_ms"] is not None else ft["t_ms"]
        self.domain = ft["domain"]
        return Item(int(round(t_ms * 1e6)), host_ns, int(ft["counter"] or f.get_frame_number()),
                    np.asanyarray(f.get_data()).copy(), (ft["exposure_us"] or 0.0, ft["gain"] or 0.0, ft["laser"] or 0.0))

    def _loop(self, pipe):
        while not self.stop_ev.is_set():
            try:
                fs = pipe.wait_for_frames(2000)
            except RuntimeError:
                self.timeouts += 1
                continue
            host = time.time_ns()
            try:
                frames = list(fs)
            except TypeError:
                frames = [fs[i] for i in range(fs.size())]
            for f in frames:
                name = self._name_of(f)
                if name is None or name not in self.rings:
                    continue
                it = self._item(f, host)
                self.rings[name].push(it)
                if name == "color":
                    try:
                        self.cq.put_nowait(it)
                    except queue.Full:
                        self.dropped_color += 1

    def next_color(self, timeout: float = 1.0) -> Item:
        return self.cq.get(timeout=timeout)

    def close(self):
        self.stop_ev.set()
        for t in self.threads:
            t.join(timeout=3)
        for p in self.pipes:
            try:
                p.stop()
            except RuntimeError:
                pass


# ----------------------------------------------------------------------------------------------- the recorder
def lateral_speeds(hist: list, ball_r: float, max_step: float = 240.0, steps: int = 3) -> list:
    """Lateral speeds (m/s) of the blobs in the newest frame of `hist` [(t_s, [(u, v, r_px), ...]), ...], each chained back up to `steps`
    frames through the nearest blob (within max_step px) of every earlier frame: the median of pixel step x (ball radius / blob radius in
    px) / dt. The ball's known size gives the metres per pixel, so no range or calibration is needed. A thrown ball moves several m/s
    sideways or up in its first frames; a hand carrying it, about 1 m/s. [] when no blob has a chain of at least two steps."""
    out = []
    for c in (hist[-1][1] if hist else []):
        chain = [(hist[-1][0], c)]
        for t_k, cs in reversed(hist[:-1]):
            if len(chain) > steps or not cs:
                break
            u, v, _ = chain[-1][1]
            best = min(cs, key=lambda p: math.hypot(p[0] - u, p[1] - v))
            if math.hypot(best[0] - u, best[1] - v) > max_step:
                break
            chain.append((t_k, best))
        if len(chain) < 3:
            continue
        sp = [math.hypot(a[0] - b[0], a[1] - b[1]) * ball_r / max(0.5 * (a[2] + b[2]), 0.5) / (ta - tb)
              for (ta, a), (tb, b) in zip(chain[:-1], chain[1:]) if ta > tb]
        if sp:
            out.append(float(np.median(sp)))
    return out


def snapshot(rig: RigBase, t_trig: int, pre_ns: int, post_ns: int) -> dict:
    """Every stream's frames in [t_trig - pre, t_trig + post], as save_throw's `streams`."""
    return {name: arrays_of(ring.window(t_trig - pre_ns, t_trig + post_ns)) for name, ring in rig.rings.items()}


def write_throw(d: Path, k: int, snap: dict, meta: dict) -> Path:
    streams = {n: s for n, s in snap.items() if len(s["frames"])}
    t0 = int(streams["color"]["stamps"][0, 0])
    meta = dict(meta, streams={n: S.stream_stats(s["stamps"], t0) for n, s in streams.items()})
    return S.save_throw(d, k, None, None, meta, streams=streams), meta


class Spooler:
    """Writes throws off the capture thread. A four-stream throw is ~420 MB and the Pi's SD card takes ~15 MB/s (measured: 13-18 MB/s
    with dd), so one throw is ~30 s of writing and a burst of throws would be refused. With a `spool` directory (RAM: /dev/shm on the
    Pi) the throw goes there at memory speed and a second thread moves it to the session folder in the background; a throw is
    refused only when the spool's budget is full. events: ("written", k, meta, secs, path) when a throw is safe in the spool (or on
    disk, with no spool), ("on_disk", k, None, secs, path) when the move is done."""

    def __init__(self, final: Path, spool: Path | None, limit_bytes: int):
        self.final, self.spool, self.limit = final, spool, limit_bytes
        if spool is not None:
            spool.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.bytes = 0
        self.n_open = 0                       # submitted and not yet on disk
        self.events: queue.Queue = queue.Queue()
        self.wq: queue.Queue = queue.Queue()
        self.mq: queue.Queue = queue.Queue()
        self.threads = [threading.Thread(target=self._writer, daemon=True), threading.Thread(target=self._mover, daemon=True)]
        for t in self.threads:
            t.start()

    def room(self, nbytes: int) -> bool:
        with self.lock:
            return self.bytes + nbytes <= self.limit

    def submit(self, k: int, snap: dict, meta: dict, nbytes: int):
        with self.lock:
            self.bytes += nbytes
            self.n_open += 1
        self.wq.put((k, snap, meta, nbytes))

    def _writer(self):
        while True:
            job = self.wq.get()
            if job is None:
                self.mq.put(None)
                return
            k, snap, meta, nbytes = job
            t0 = time.time()
            path, meta = write_throw(self.spool or self.final, k, snap, meta)
            job = snap = None                                      # the frames are in the file: free them now, not at the next job
            self.events.put(("written", k, meta, time.time() - t0, path))
            self.mq.put((k, path, nbytes))

    def _mover(self):
        while True:
            job = self.mq.get()
            if job is None:
                return
            k, path, nbytes = job
            t0 = time.time()
            if self.spool is not None:
                import shutil
                shutil.move(str(path), str(self.final / path.name))
                path = self.final / path.name
            with self.lock:
                self.bytes -= nbytes
                self.n_open -= 1
            self.events.put(("on_disk", k, None, time.time() - t0, path))

    def finish(self):
        self.wq.put(None)
        for t in self.threads:
            t.join()
        if self.spool is not None:
            try:
                self.spool.rmdir()
            except OSError:
                pass


def ball_info(a) -> dict:
    """The ball for the session (the analysis' drag law and size model read it): from the circumference and mass given, or the synthetic
    rig's own ball. mass / radius / drag_cd are arcfit.Ball's fields."""
    if a.synthetic:
        from arcfit import Ball
        b = Ball()
        return dict(mass=b.mass, radius=b.radius, drag_cd=b.drag_cd, source="synthetic")
    r = a.ball_circumference_in * 0.0254 / (2 * math.pi)
    return dict(mass=a.ball_mass_g / 1000.0, radius=r, drag_cd=a.ball_cd, diameter_m=2 * r, circumference_in=a.ball_circumference_in,
                material="foam", colour="orange", source="Fernando, Oct 30 2026; drag_cd is the robot config's estimate")


def expected_bytes(calib: dict, streams: list, seconds: float) -> int:
    """About how much a throw of `seconds` holds, from the stream modes."""
    per = {"color": 3, "depth": 2, "ir1": 1, "ir2": 1}
    return int(sum(calib["streams"][n]["width"] * calib["streams"][n]["height"] * per[n] * calib["streams"][n]["fps"] * seconds for n in streams))


def summary_line(k: int, meta: dict) -> str:
    parts = []
    for n, s in meta["streams"].items():
        parts.append(f"{n} {s['n']}" + (f" @{s['fps']:.0f}" if s.get("fps") else "") + (f" LOST {s['lost']}" if s.get("lost") else ""))
    return f"  t{k:03d}: " + ", ".join(parts)


def check(rig: RigBase, seconds: float):
    """rscheck: no triggering; every stream's rate, lost frames and clock offsets, so the first hardware run is one command."""
    rig.skip_color(int(seconds * float(rig.calib["streams"]["color"]["fps"])))
    n = rig.rings["color"].n
    print(f"\n{seconds:.0f} s, mode: {getattr(rig, 'mode', 'synthetic')}, colour frames read {n}")
    base = rig.rings["color"].t_last
    for name, ring in rig.rings.items():
        h = ring.health()
        last = ring.latest(1)
        age = f", newest frame {(base - last[0].t_ns) * 1e-6:+.1f} ms from colour's" if last and base else ""
        lat = f", stamp -> host {np.median([(it.host_ns - it.t_ns) * 1e-6 for it in ring.latest(30)]):.1f} ms" if last else ""
        print(f"  {name:6s} {h['n']:5d} frames  {h['fps'] or 0:6.1f} fps  lost {h['lost']}  max gap {h['max_gap_ms']:.1f} ms{lat}{age}")
    print("timestamp domain:", rig.domain, "(global_time = the camera's own clock mapped to the host's; system_time = arrival times: metadata off)")
    print("options:", json.dumps(rig.calib.get("options"), default=str))
    if getattr(rig, "dropped_color", 0) or getattr(rig, "timeouts", 0):
        print(f"colour frames dropped by the reader: {rig.dropped_color}, timeouts: {rig.timeouts}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--name", default="session")
    ap.add_argument("--distance", type=float, default=0.0, help="thrower distance, m (metadata)")
    ap.add_argument("--count", type=int, default=0, help="stop after this many saved throws (0: until Ctrl-C)")
    ap.add_argument("--streams", default="color,depth,ir1,ir2", help="comma list of color, depth, ir1, ir2 (color is the trigger: always on)")
    ap.add_argument("--size", default="848x480", help="colour size (falls back to the nearest the link offers)")
    ap.add_argument("--fps", type=float, default=60.0, help="colour fps")
    ap.add_argument("--depth-size", default="848x480")
    ap.add_argument("--depth-fps", type=float, default=90.0)
    ap.add_argument("--exposure-us", type=int, default=None, help="colour exposure (default: ball_view's saved value, else 3000; 0 = auto)")
    ap.add_argument("--gain", type=float, default=None)
    ap.add_argument("--awb", action="store_true", help="colour auto white balance (default off: a fixed colour window)")
    ap.add_argument("--emitter", choices=["on", "off", "alternate"], default="on", help="the IR dot projector (stored with the session)")
    ap.add_argument("--ir-exposure-us", type=int, default=1000, help="depth / IR exposure (0 = auto)")
    ap.add_argument("--serial")
    ap.add_argument("--pre", type=float, default=0.7, help="seconds kept before the trigger")
    ap.add_argument("--post", type=float, default=1.3, help="seconds kept after the trigger")
    ap.add_argument("--settings", default=str(SETTINGS), help="ball_view settings json (the colour window; default ~/omni_ws/ball_view_settings/realsense.json)")
    ap.add_argument("--hue", type=float, nargs=2)
    ap.add_argument("--min-sat", type=float)
    ap.add_argument("--min-val", type=float)
    ap.add_argument("--size-gain", type=float)
    ap.add_argument("--rim-radius", type=float, default=0.165, help="bucket / basket rim radius, m (metadata)")
    ap.add_argument("--learn-s", type=float, default=2.5, help="seconds at the start to learn the static ball-coloured spots (flowers, a lamp) to ignore; keep the ball out of view")
    ap.add_argument("--ball-circumference-in", type=float, default=19.875, help="the ball's circumference, inches (19 7/8: Fernando's orange foam ball)")
    ap.add_argument("--ball-mass-g", type=float, default=73.0)
    ap.add_argument("--ball-cd", type=float, default=0.5, help="drag coefficient (the robot config's value: an estimate, not measured)")
    ap.add_argument("--synthetic", action="store_true", help="no camera: rendered throws")
    ap.add_argument("--synthetic-gap", type=float, default=1.5)
    ap.add_argument("--synthetic-pitch", type=float, default=0.0)
    ap.add_argument("--check", type=float, default=0.0, help="no recording: run this many seconds and report each stream's health")
    ap.add_argument("--min-throw-speed", type=float, default=1.5,
                    help="m/s: ball-coloured motion slower than this (lateral, from the blob's size) is not a throw: carrying the ball by hand doesn't "
                         "start a capture (0 = trigger on any motion)")
    ap.add_argument("--trigger", choices=["arc", "motion"], default="arc",
                    help="arc: a blob track that follows a ballistic arc (arc2d.py: quadratic in time, travels, fast, ~g, smooth size; ignores the "
                         "scene's static orange things and a carried ball); motion: the older moving-blob + speed test")
    ap.add_argument("--force-every", type=float, default=0.0, help="TEST: capture every this many seconds whether or not a ball moved (the write path under load)")
    ap.add_argument("--ask", action="store_true", help="ask y / n after each throw (default: no labels, the arc is the reference)")
    ap.add_argument("--out", default=str(S.DATA))
    ap.add_argument("--spool", default="/dev/shm/omni_spool" if sys.platform.startswith("linux") and Path("/dev/shm").is_dir() else "",
                    help="write throws to this RAM directory first and move them to --out in the background ('' = straight to disk: the Pi's SD "
                         "card takes ~15 MB/s, ~30 s per four-stream throw)")
    ap.add_argument("--spool-mb", type=float, default=1200.0, help="the spool's budget; a throw seen while it is full is ignored")
    ap.add_argument("--notes", default="")
    a = ap.parse_args(argv)
    sys.stdout.reconfigure(line_buffering=True)                                   # READY / throw seen show up through ssh pipes too

    streams = [s.strip() for s in a.streams.split(",") if s.strip()]
    bad = [s for s in streams if s not in S.STREAMS]
    if bad or "color" not in streams:
        raise SystemExit(f"--streams: choose from {S.STREAMS}, and color is needed for the trigger (got {streams})")
    streams = [s for s in S.STREAMS if s in streams]
    keep_s = a.pre + a.post + 0.6
    dcfg, camset = detector_config(a)
    exposure_us = a.exposure_us if a.exposure_us is not None else int(camset.get("exposure_us", 0) or 3000)
    gain = a.gain if a.gain is not None else camset.get("gain")
    awb = a.awb or bool(camset.get("awb", False))

    rig = SyntheticRig(a, streams, keep_s) if a.synthetic else RealSenseRig(a, streams, keep_s)
    try:                                                   # close() always runs: a pipeline left open kills the process at exit
        calib = rig.start() if a.synthetic else rig.start(exposure_us, gain, awb, camset.get("white_balance"))
        cs = calib["streams"]["color"]
        print(f"{calib['device'].get('name')} {calib['device'].get('serial_number')}: colour {cs['width']}x{cs['height']}@{cs['fps']}, "
              f"streams {streams}" + (f", {rig.mode}" if hasattr(rig, 'mode') else ""))
        if a.check:
            rig.skip_color(int(cs["fps"]))
            check(rig, a.check)
            return
        run(a, rig, calib, dcfg, exposure_us, gain, streams)
    finally:
        rig.close()


def run(a, rig, calib, dcfg, exposure_us, gain, streams):
    cs = calib["streams"]["color"]
    ci = cs["intrinsics"]
    fps = float(cs["fps"])
    det = make_detector(dcfg)
    scale = 120.0 / fps
    trig = Trigger(det, max_step=120.0 * scale)
    arc = arc2d.ArcTracker(arc2d.ArcCfg(ball_radius=ball_info(a)["radius"], min_speed=a.min_throw_speed))
    front_check = None
    if "depth" in rig.rings and "depth" in calib["streams"]:
        # a blob whose depth is much nearer than its size says sits on a hand or an arm (Oct 30: a walking person's hand fired the arc trigger)
        di, ext = calib["streams"]["depth"]["intrinsics"], calib.get("extrinsics", {}).get("depth_to_color")
        ic_t, id_t = (ci["fx"], ci["fy"], ci["ppx"], ci["ppy"]), (di["fx"], di["fy"], di["ppx"], di["ppy"])
        R_dc = np.array(ext["R"], float).reshape(3, 3, order="F") if ext else np.eye(3)             # librealsense: column-major
        t_dc = np.array(ext["t"], float) if ext else np.zeros(3)
        d_scale, b_r = float(calib.get("depth_scale", 0.001)), ball_info(a)["radius"]

        def front_check(hit, t_ns):
            last = rig.rings["depth"].latest(1)
            if not last or abs(last[0].t_ns - t_ns) > 40_000_000:
                return False
            return arc2d.depth_in_front(hit["u"], hit["v"], hit["r_px"], last[0].arr, d_scale, ic_t, id_t, R_dc, t_dc,
                                        ic_t[0] * b_r / max(hit["r_px"], 0.5))
    d = S.new_session_dir(a.name, Path(a.out))
    fovy = math.degrees(2 * math.atan(ci["height"] / 2 / ci["fy"]))
    info = dict(camera=dict(name="intel_realsense_d435" if not a.synthetic else "synthetic_realsense", width=ci["width"], height=ci["height"],
                            fps=fps, fovy_deg=fovy, f_px=ci["fy"], f_px_est=ci["fy"], exposure_us=exposure_us, gain=gain,
                            shutter="rolling" if not a.synthetic else "synthetic", fov_is_estimate=False,
                            fov_source="the camera's own factory calibration (intrinsics)"),
                rs=calib, streams=streams, emitter=a.emitter, ir_exposure_us=a.ir_exposure_us,
                ball=ball_info(a), detector=asdict(dcfg), distance_m=a.distance, notes=a.notes, host=platform.node(),
                basket=dict(rim_radius=a.rim_radius, rim_center_world=None, rim_center_cam=None),
                pre_s=a.pre, post_s=a.post, started=time.strftime("%Y-%m-%dT%H:%M:%S"))
    print("settling, taking the empty-scene stills ...")
    rig.skip_color(int(fps))
    calib["timestamp_domain"] = rig.domain
    if rig.domain not in ("global_time", "synthetic"):
        print(f"WARNING: timestamps are in the '{rig.domain}' domain: the streams' clocks are the host's arrival times, not the camera's "
              "(metadata off: enable_metadata.ps1 on Windows; it works out of the box on the Pi)")
    S.save_session(d, info)
    for name, img in rig.stills().items():
        np.save(d / f"setup_{name}.npy", img)
    if "color" in rig.rings and rig.rings["color"].latest(1):
        np.save(d / "tag.npy", rig.rings["color"].latest(1)[0].arr)
    print(f"session {d}\nexposure {exposure_us or 'auto'} us, fovy {fovy:.1f} deg (factory calibration), emitter {a.emitter}")

    pre_ns, post_ns = int(a.pre * 1e9), int(a.post * 1e9)
    blobs: deque = deque(maxlen=int((a.pre + a.post + 1) * fps) + 60)
    state = dict(armed=True, saved=0, quit=False, t_trig=None, first_t=None, ready=None, slow_reported=False)
    hist: deque = deque(maxlen=6)                                                 # the last frames' blobs, for the throw-speed gate
    ball_r = ball_info(a)["radius"]
    spool = Path(a.spool) / d.name if (a.spool and not a.ask) else None           # --ask labels the files in place: no spool
    est = expected_bytes(calib, streams, a.pre + a.post)
    spooler = Spooler(d, spool, int(a.spool_mb * 1e6) if spool else int(2.2 * est))          # no spool: about two throws in flight
    print(f"a throw is ~{est / 1e6:.0f} MB; " + (f"spooled in {spool} (budget {a.spool_mb:.0f} MB) and moved to disk in the background" if spool
                                                else "written straight to disk"))
    print("it triggers on ball-coloured motion faster than "
          + (f"{a.min_throw_speed:g} m/s (a ball carried by hand is ignored; a ball held still never triggers)" if a.min_throw_speed > 0 else "anything")
          + "; throw when it says READY (Ctrl-C to finish)")
    idle_since = time.time()
    try:
        while not state["quit"]:
            try:
                it = rig.next_color(1.0)
            except queue.Empty:
                if time.time() - idle_since > 10:
                    print("no colour frames for 10 s: is the camera streaming?")
                    idle_since = time.time()
                continue
            idle_since = time.time()
            t_s = it.t_ns * 1e-9
            state["first_t"] = it.t_ns if state["first_t"] is None else state["first_t"]
            learning = it.t_ns - state["first_t"] < int(a.learn_s * 1e9)             # the empty scene: where the static ball-coloured things are
            if learning:
                trig.learn(it.arr)
            elif not state.get("frozen"):
                state["frozen"] = True
                n_mask = trig.freeze()
                print(f"  learned {len(trig.counts)} ball-coloured cell(s) in {trig.frames} frames, strongest {sorted(trig.counts.values(), reverse=True)[:5]}; {n_mask} cells masked")
            last_move = trig.last_move_t
            fired = trig.update(it.arr, t_s) and not learning
            hist.append((t_s, [(float(c[0]), float(c[1]), float(c[2])) for c in trig.prev]))
            if a.trigger == "arc":
                hit = None if learning else arc.update(t_s, trig.prev)           # trig.prev: this frame's blobs, the learned static spots masked
                if hit and front_check is not None and front_check(hit, it.t_ns):
                    if state["armed"] and not state["slow_reported"]:
                        print(f"  (moving blob on a nearer object - a hand or an arm, its depth much nearer than its size - not a throw; {arc2d.fmt(hit)})")
                        state["slow_reported"] = True
                    hit = None
                fired = hit is not None
                # re-arm on a second with no throw-like arc: any blob motion (a noise blob flickering at the frame's edge, the ball carried
                # back) kept the old test from ever re-arming (Oct 30, the first multi-throw session)
                trig.last_move_t = t_s if fired else last_move
                if hit and state["t_trig"] is None and state["armed"]:
                    state["arc_hit"] = hit
                if arc.best_reject and state["armed"] and not state["slow_reported"]:
                    why, st = arc.best_reject
                    print(f"  (moving blob, not a throw: {why}; {arc2d.fmt(st)})")
                    state["slow_reported"] = True
                arc.best_reject = None
            elif fired and a.min_throw_speed > 0:                              # moving ball-coloured blob: fast enough to be a throw?
                sp = lateral_speeds(list(hist), ball_r)
                if not sp or max(sp) < a.min_throw_speed:
                    if state["armed"] and not state["slow_reported"]:
                        print(f"  (ball-coloured motion at {max(sp, default=0.0):.1f} m/s: slower than {a.min_throw_speed:g} m/s, not a throw: ignored)")
                    state["slow_reported"] = True
                    fired = False
            if a.force_every and state["armed"] and it.t_ns - state.setdefault("last_force", it.t_ns) >= a.force_every * 1e9:
                fired, state["last_force"] = True, it.t_ns                   # a test capture with no ball: write path, rates, sizes
            blobs.append((it.t_ns, [(round(float(c[0]), 1), round(float(c[1]), 1), round(float(c[2]), 1)) for c in trig.prev[:3]]))
            if state["t_trig"] is None:
                if state["armed"] and it.t_ns - state["first_t"] > pre_ns and fired and not (a.count and state["saved"] >= a.count):
                    if not spooler.room(est):
                        state["armed"] = False                           # once per throw: re-armed when the scene has been still for a second
                        print(f"  (a throw was seen but {spooler.n_open} are still being written: ignored; wait for 'on disk')")
                    else:
                        state["t_trig"], state["armed"] = it.t_ns, False
                        print("  throw seen ...", end="", flush=True)
            else:
                if it.t_ns >= state["t_trig"] + post_ns + int(0.05e9):         # 50 ms for the other streams' lag
                    t_trig, state["t_trig"] = state["t_trig"], None
                    k = state["saved"] + 1
                    state["saved"] = k
                    snap = snapshot(rig, t_trig, pre_ns, post_ns)
                    col_t = snap["color"]["stamps"][:, 0]
                    idx = int(np.searchsorted(col_t, t_trig))
                    lo, hi = int(col_t[0]), int(col_t[-1])
                    meta = dict(kind="realsense", trigger=idx, t_trigger_s=float((t_trig - col_t[0]) * 1e-9), n=len(col_t), outcome=None,
                                exposure_us_median=float(np.median(snap["color"]["info"][:, 0])), gain_median=float(np.median(snap["color"]["info"][:, 1])),
                                live_blobs=[b for t, b in blobs if lo <= t <= hi], trigger_kind=a.trigger, arc_trigger=state.pop("arc_hit", None),
                                saved=time.strftime("%H:%M:%S"))
                    spooler.submit(k, snap, meta, sum(x.nbytes for s in snap.values() for x in s["frames"]))
                    snap = None
                    print(f" captured {len(col_t)} colour frames (throw {k}); writing")
            if state["t_trig"] is None and trig.idle(t_s):
                state["slow_reported"] = False
                state["armed"] = True
            ready = (state["armed"] and state["t_trig"] is None and not learning and it.t_ns - state["first_t"] > pre_ns
                     and not (a.count and state["saved"] >= a.count))
            if not ready:
                state["ready"] = None
            elif not spooler.room(est):                                      # armed, but the RAM spool is full: a throw now would be refused
                if state["ready"] != "full":
                    print(f"  NOT READY: {spooler.n_open} throw(s) still being copied to disk, the spool is full")
                state["ready"] = "full"
            elif state["ready"] != "ready":
                print("  READY - throw now")
                state["ready"] = "ready"
            while not spooler.events.empty():
                kind, k, meta, secs, path = spooler.events.get()
                if kind == "on_disk":
                    print(f"  t{k:03d} on disk ({secs:.0f} s to copy from the spool)")
                    continue
                print(summary_line(k, meta) + f"   [{secs:.1f} s to {'the spool' if spool else 'write'}]")
                if a.ask and state["armed"]:
                    ans = ""
                    while ans not in ("y", "n", "s", "q"):
                        ans = input(f"  throw {k}: in the bucket? [y/n, s=discard, q=finish] ").strip().lower()[:1]
                    if ans == "s":
                        import shutil
                        shutil.rmtree(path, ignore_errors=True)
                    elif ans in ("y", "n"):
                        S.label_throw(path, ans)
                    state["quit"] = ans == "q"
            if a.count and state["saved"] >= a.count and spooler.n_open == 0:
                state["quit"] = True
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        if spooler.n_open:
            print(f"finishing {spooler.n_open} throw(s) still being written ...")
        spooler.finish()
        while not spooler.events.empty():
            kind, k, meta, secs, path = spooler.events.get()
            print(f"  t{k:03d} on disk" if kind == "on_disk" else summary_line(k, meta))
        for name, img in rig.stills(1).items():                      # did the camera or the bucket move?
            np.save(d / f"end_{name}.npy", img)
    print(f"done: {state['saved']} throws in {d}")


if __name__ == "__main__":
    main()
