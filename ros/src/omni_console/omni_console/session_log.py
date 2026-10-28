"""Session logs, one format for the robot and the sim (design.md 7: "one log format shared by the sim and the robot,
so the first real session can be replayed against the sim"). ROS-free and standard library only: the robot's console
writes them (mission_node), the sim writes them (simulation/v11 sil/sil_game.py --log), the page reads them (Logbook),
and simulation/v11 sil/studies/replay_log.py replays a real session's throws in the sim.

omni-log/1: JSON Lines, one record per line, t = seconds since the session started. A file is
<YYYY-MM-DD_HHMMSS>_<first mission>.jsonl in <state_dir>/logs/sessions, with a .summary.json beside it once closed.

  session  {format, source: "robot" | "sim", start (unix s), mission, software, firmware, map, settings, world (sim)}
  pose     {t, frame, x, y, yaw}                           the robot, 2 Hz
  status   {t, mission, task, v_bus, battery, mcu, faults}  1 Hz
  throw    {t, toss, player, caught, speed, apex, release, path [[t, x, y, z]], predicted, robot [x, y, yaw],
            flight_time, frame: "odom", truth (sim only: the world's own release state, pos_error, caught)}
  event    {t, level, text}
  end      {t, summary}

Throws are in the robot's odom frame, whose origin is where it waited at the start (catch_world puts the sim's at
HOME facing the thrower), so a real throw maps into the sim's field directly.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

FORMAT = "omni-log/1"
KINDS = ("session", "pose", "status", "throw", "event", "end")


def _clean(v):
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    return v


class SessionLog:
    def __init__(self, folder: str | Path, mission: str, source: str = "robot", start: float | None = None, **header):
        self.start = start if start is not None else time.time()
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y-%m-%d_%H%M%S", time.localtime(self.start))
        self.path = self.folder / f"{stamp}_{mission.lower()}.jsonl"
        self.f = open(self.path, "a")
        self.counts = {k: 0 for k in KINDS}
        self.throws = self.caught = 0
        self.missions = [mission]
        self.dist = 0.0
        self._last_xy = None
        self.write("session", t=0.0, format=FORMAT, source=source, start=round(self.start, 3), mission=mission,
                   **header)

    def t(self, now: float | None = None) -> float:
        return round((now if now is not None else time.time()) - self.start, 3)

    def write(self, kind: str, t: float | None = None, **fields):
        if kind not in KINDS:
            raise ValueError(kind)
        rec = dict(type=kind, t=self.t() if t is None else t, **fields)
        self.f.write(json.dumps(_clean(rec), separators=(",", ":")) + "\n")
        self.counts[kind] += 1
        if kind == "throw":
            self.throws += 1
            self.caught += bool(fields.get("caught"))
        elif kind == "pose" and "x" in fields:
            if self._last_xy is not None and fields.get("frame") == self._last_xy[2]:
                d = math.hypot(fields["x"] - self._last_xy[0], fields["y"] - self._last_xy[1])
                self.dist += d if d < 3.0 else 0.0
            self._last_xy = (fields["x"], fields["y"], fields.get("frame"))
        elif kind == "status" and fields.get("mission") and fields["mission"] not in self.missions:
            self.missions.append(fields["mission"])

    def flush(self):
        self.f.flush()

    def summary(self) -> dict:
        return dict(name=self.path.name, start=round(self.start, 3), duration_s=self.t(), missions=self.missions,
                    throws=self.throws, caught=self.caught, distance_m=round(self.dist, 1), records=dict(self.counts))

    def close(self, now: float | None = None) -> dict:
        s = self.summary()
        s["duration_s"] = self.t(now)
        self.write("end", t=s["duration_s"], summary=s)
        self.f.close()
        self.path.with_suffix(".summary.json").write_text(json.dumps(s))
        return s


def read(path: str | Path) -> tuple:
    """(header, [records]) of a log; lines that don't parse are skipped (a robot that lost power mid-line)."""
    header, recs = None, []
    for line in Path(path).read_text().splitlines():
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if r.get("type") == "session" and header is None:
            header = r
        else:
            recs.append(r)
    if header is None or header.get("format") != FORMAT:
        raise ValueError(f"{path}: not an {FORMAT} log")
    return header, recs


def throws(path: str | Path) -> list:
    return [r for r in read(path)[1] if r.get("type") == "throw"]


def listing(folder: str | Path) -> list:
    """The sessions in a folder, newest first, with their summaries (a log still open has none yet)."""
    out = []
    for p in sorted(Path(folder).glob("*.jsonl"), reverse=True):
        s = p.with_suffix(".summary.json")
        try:
            summ = json.loads(s.read_text()) if s.exists() else None
        except ValueError:
            summ = None
        out.append(dict(name=p.name, bytes=p.stat().st_size, mtime=round(p.stat().st_mtime), summary=summ))
    return out
