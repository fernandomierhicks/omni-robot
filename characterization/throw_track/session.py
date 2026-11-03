"""On-disk format of a recorded throw session (numpy + json only: it runs on the Pi too).

  data/<session>/session.json     camera, detector settings, basket / tag geometry, notes (RealSense: "rs", see record_rs.py)
  data/<session>/tag.npy          one still frame with the basket's AprilTag in view (optional)
  data/<session>/t001/frames.npy  N x H x W x 3 uint8, R G B (what the robot's detector takes)
                    stamps.npy    N x 2 (RealSense N x 3) int64: sensor timestamp (ns), host arrival (ns)[, frame counter]
                    meta.json     trigger, live detections, exposure / gain per frame, the typed outcome label
                    depth.npy, ir1.npy, ir2.npy   (RealSense sessions, record_rs.py) N x H x W uint16 / uint8 / uint8, each with
                    <name>_stamps.npy  N x 3 int64 [mid-exposure ns, host arrival ns, frame counter]
                    <name>_info.npy    N x 3 float32 [exposure us, gain, laser power]   (per frame, from the camera's metadata)

`frames.npy` is the colour stream (the recorder's trigger and `analyze_throws.py` read it); a RealSense session without colour
has none, and `Throw.frames` is None. The streams of a RealSense throw run at their own rates (colour 60 fps, depth and IR 90 fps)
but share one clock: librealsense's global time (the host's system clock, ms since the epoch), so `Throw.t_of(name)` puts every
stream on one time axis, seconds from the throw's first frame (`Throw.t0_ns`).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"
COLOR = "color"
STREAMS = ("color", "depth", "ir1", "ir2")


def new_session_dir(name: str, root: Path | None = None) -> Path:
    import time
    d = (root or DATA) / f"{time.strftime('%Y%m%d_%H%M%S')}_{name}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def save_session(d: Path, info: dict):
    (d / "session.json").write_text(json.dumps(info, indent=1))


def load_session(d: Path) -> dict:
    return json.loads((Path(d) / "session.json").read_text())


def stream_files(name: str) -> tuple[str, str, str]:
    """(frames, stamps, per-frame info) file names of a stream; colour keeps the original names."""
    if name == COLOR:
        return "frames.npy", "stamps.npy", "color_info.npy"
    return f"{name}.npy", f"{name}_stamps.npy", f"{name}_info.npy"


def save_frames(path: Path, frames) -> None:
    """Write an array, or a list of equal-shaped images, to a .npy without stacking the list first (a RealSense throw is
    hundreds of MB: np.stack would double it on a 4 GB Pi)."""
    if isinstance(frames, np.ndarray):
        np.save(path, frames)
        return
    first = np.asarray(frames[0])
    mm = np.lib.format.open_memmap(path, mode="w+", dtype=first.dtype, shape=(len(frames),) + first.shape)
    for i, f in enumerate(frames):
        mm[i] = f
    mm.flush()
    del mm


def save_throw(d: Path, k: int, frames, stamps: np.ndarray | None, meta: dict, streams: dict | None = None) -> Path:
    """frames / stamps: the colour stream (None for a session without colour). streams: {name: dict(frames=, stamps=,
    info=)} for the others (and, for convenience, "color" too, which then replaces frames / stamps)."""
    t = d / f"t{k:03d}"
    t.mkdir(parents=True, exist_ok=True)
    streams = dict(streams or {})
    if frames is not None:
        streams.setdefault(COLOR, dict(frames=frames, stamps=stamps))
    for name, s in streams.items():
        f, st, inf = stream_files(name)
        save_frames(t / f, s["frames"])
        np.save(t / st, np.asarray(s["stamps"], dtype=np.int64))
        if s.get("info") is not None:
            np.save(t / inf, np.asarray(s["info"], dtype=np.float32))
    (t / "meta.json").write_text(json.dumps(meta, indent=1))
    return t


@dataclass
class Stream:
    frames: np.ndarray             # N x H x W (x 3), memory-mapped
    stamps: np.ndarray             # N x 2 or 3 int64
    info: np.ndarray | None = None


@dataclass
class Throw:
    path: Path
    frames: np.ndarray | None      # N x H x W x 3 (memory-mapped): the colour stream
    stamps: np.ndarray | None      # N x 2 int64
    meta: dict
    session: dict = field(default_factory=dict)

    @property
    def names(self) -> list[str]:
        """The streams on disk, in STREAMS' order."""
        return [n for n in STREAMS if (self.path / stream_files(n)[0]).exists()]

    def stream(self, name: str) -> Stream:
        f, st, inf = stream_files(name)
        p = self.path
        return Stream(np.load(p / f, mmap_mode="r"), np.load(p / st), np.load(p / inf) if (p / inf).exists() else None)

    @property
    def t0_ns(self) -> int:
        """The clock's origin for t / t_of: the first colour frame's stamp, else the earliest first stamp."""
        if self.stamps is not None:
            return int(self.stamps[0, 0])
        return min(int(np.load(self.path / stream_files(n)[1])[0, 0]) for n in self.names)

    def t_of(self, name: str) -> np.ndarray:
        """Seconds since the throw's first frame, on the shared clock, for any stream."""
        st = self.stamps if (name == COLOR and self.stamps is not None) else self.stream(name).stamps
        return (st[:, 0].astype(np.float64) - self.t0_ns) * 1e-9

    @property
    def t(self) -> np.ndarray:
        """Seconds since the first frame, from the sensor's own clock."""
        s = self.stamps[:, 0].astype(np.float64)
        return (s - s[0]) * 1e-9

    @property
    def label(self):
        return self.meta.get("outcome")     # "y" / "n" / None


def load_throw(path: Path, session: dict | None = None) -> Throw:
    path = Path(path)
    has_color = (path / "frames.npy").exists()
    return Throw(path, np.load(path / "frames.npy", mmap_mode="r") if has_color else None,
                 np.load(path / "stamps.npy") if has_color else None,
                 json.loads((path / "meta.json").read_text()), session or {})


def throw_dirs(session_dir: Path) -> list[Path]:
    return sorted(p for p in Path(session_dir).iterdir() if p.is_dir() and (p / "meta.json").exists()
                  and any((p / stream_files(n)[0]).exists() for n in STREAMS))


def stream_stats(stamps: np.ndarray, t0_ns: int | None = None) -> dict:
    """Rate and health of one stream from its stamps (col 0 = sensor time ns, optional col 2 = the camera's frame counter):
    frames, achieved fps, the largest gap, frames lost by the counter (else by the interval), span on the shared clock."""
    st = np.asarray(stamps)
    n = len(st)
    if n < 2:
        return dict(n=int(n))
    t = st[:, 0].astype(np.float64)
    dt = np.diff(t) * 1e-6
    nominal = float(np.median(dt))
    lost = int(np.sum(np.maximum(np.diff(st[:, 2]) - 1, 0))) if st.shape[1] > 2 and np.all(st[:, 2] != 0) \
        else int(np.sum(np.maximum(np.round(dt / nominal) - 1, 0)))
    o = t[0] if t0_ns is None else float(t0_ns)
    return dict(n=int(n), fps=float((n - 1) / ((t[-1] - t[0]) * 1e-9)), dt_median_ms=nominal, dt_max_ms=float(dt.max()),
                lost=lost, first_s=float((t[0] - o) * 1e-9), last_s=float((t[-1] - o) * 1e-9))


def label_throw(path: Path, outcome: str, **extra):
    m = Path(path) / "meta.json"
    meta = json.loads(m.read_text())
    meta["outcome"] = outcome
    meta.update(extra)
    m.write_text(json.dumps(meta, indent=1))
