"""The calibration results (design.md 10): one JSON file per item, every run kept, so a result can be compared with
the drawing and with the last sessions before it is used.

    store = Store("~/.omni/calibration")
    store.add("cam_left_intrinsics", value={...}, metrics={"rms_px": 0.21, ...}, method="aprilgrid, 42 views")
    store.latest("stereo")                    # the last run that passed its limits
    store.frames()                            # frame overrides for robot_frames (only passed results)

Each item has limits (LIMITS: what a run must reach to be used, est until the bench says otherwise). A run that misses
them is kept as failed, never applied. drift() says how far the latest moved from the run before: a stereo pair whose
rotation moves > 0.1 deg between sessions is the design's biggest catch-rate risk (design.md 3: 0.04 -> 0.3 deg costs
~19 points), so the console shows it.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

# what a run must reach (est): reprojection px, epipolar px, plane residual mm, time offset sigma s, spreads ...
LIMITS = {
    "intrinsics": dict(rms_px=0.5, coverage=0.6),
    "stereo": dict(rms_px=0.5, epipolar_px=0.3, sigma_rot_deg=0.02),
    "cam_imu": dict(time_sigma_s=0.5e-3, rot_rms=0.05),
    "lidar_camera": dict(rms_mm=15.0, on_board=0.9),
    "base_extrinsics": dict(rms_m=0.01, yaw_mismatch_deg=1.0),
    "dock": dict(spread_mm=5.0, spread_deg=1.0),
    "timing": dict(sync_rms_us=50.0),
}


def passes(kind: str, metrics: dict) -> tuple:
    """(ok, failures): each limit is a maximum, except coverage and on_board (minimums)."""
    fails = []
    for k, lim in LIMITS.get(kind, {}).items():
        v = metrics.get(k)
        if v is None or (isinstance(v, float) and math.isnan(v)):
            fails.append(f"{k}: missing")
        elif k in ("coverage", "on_board"):
            if v < lim:
                fails.append(f"{k} {v:.3g} < {lim}")
        elif v > lim:
            fails.append(f"{k} {v:.3g} > {lim}")
    return not fails, fails


class Store:
    def __init__(self, root: str | Path):
        self.root = Path(root).expanduser()
        self.root.mkdir(parents=True, exist_ok=True)

    def _file(self, item: str) -> Path:
        return self.root / f"{item}.json"

    def runs(self, item: str) -> list:
        f = self._file(item)
        return json.loads(f.read_text()) if f.exists() else []

    def add(self, item: str, kind: str, value: dict, metrics: dict, method: str = "", t: float | None = None) -> dict:
        ok, fails = passes(kind, metrics)
        run = dict(item=item, kind=kind, value=value, metrics=metrics, method=method, passed=ok, failures=fails,
                   time=t if t is not None else time.time())
        runs = self.runs(item) + [run]
        self._file(item).write_text(json.dumps(runs, indent=1))
        return run

    def latest(self, item: str, passed_only: bool = True) -> dict | None:
        rs = [r for r in self.runs(item) if r["passed"] or not passed_only]
        return rs[-1] if rs else None

    def drift(self, item: str, key: str) -> float | None:
        """How far value[key] moved between the last two passed runs (scalar or vector: the norm)."""
        rs = [r for r in self.runs(item) if r["passed"]]
        if len(rs) < 2:
            return None
        a, b = rs[-2]["value"][key], rs[-1]["value"][key]
        if isinstance(a, list):
            return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))
        return abs(b - a)

    def frames(self) -> dict:
        """child frame -> {"xyz", "quat", "sigma"} from every passed item that places a frame (value["frame"])."""
        out = {}
        for f in sorted(self.root.glob("*.json")):
            r = self.latest(f.stem)
            if r and "frame" in r["value"]:
                fr = r["value"]["frame"]
                out[fr["child"]] = dict(xyz=fr["xyz"], quat=fr["quat"], sigma=r["metrics"])
        return out
