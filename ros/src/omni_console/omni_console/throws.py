"""The throw radar and the leaderboard (the console's Catch tab), ROS-free.

Every throw the catch code tracked comes with its record (omni_catch.game: speed, apex, release point, path, where it
would cross the funnel, caught or not). Here it gets the player who threw it and joins their tally. A person's scores
are about their throw, not the robot's catch: how fast, how high, how close to the robot's waiting spot it would have
come down (aim), and their longest run of throws the robot caught.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

PLAYER_NAME_MAX = 24


def kmh(v):
    return None if v is None or not math.isfinite(v) else round(3.6 * v, 1)


def aim_error(rec) -> float | None:
    """Metres from the robot's waiting spot (odom 0, 0: where it starts) to where the throw would cross the funnel."""
    p = rec.get("predicted")
    if not p or len(p) < 2:
        return None
    return round(math.hypot(p[0], p[1]), 3)


class ThrowLog:
    """All throws (throws.jsonl, one per line, appended) and a tally per player. Pass path=None to keep it in memory."""

    def __init__(self, path: str | Path | None = None):
        self.path = Path(path) if path else None
        self.players: dict = {}
        self.recent: list = []                       # the newest throws, newest last (at most 50)
        self.player = "Player 1"
        if self.path and self.path.exists():
            for line in self.path.read_text().splitlines():
                try:
                    self._tally(json.loads(line))
                except (ValueError, KeyError, TypeError):
                    continue

    def set_player(self, name: str) -> str:
        name = " ".join(str(name).split())[:PLAYER_NAME_MAX]
        if not name:
            raise ValueError("a player needs a name")
        self.player = name
        return name

    def _tally(self, r: dict):
        p = self.players.setdefault(r["player"], dict(throws=0, caught=0, streak=0, best_streak=0, best_speed=None,
                                                      best_apex=None, aim_sum=0.0, aim_n=0, last=None))
        p["throws"] += 1
        if r.get("caught"):
            p["caught"] += 1
            p["streak"] += 1
            p["best_streak"] = max(p["best_streak"], p["streak"])
        else:
            p["streak"] = 0
        for k, key in (("speed", "best_speed"), ("apex", "best_apex")):
            v = r.get(k)
            if v is not None and math.isfinite(v) and (p[key] is None or v > p[key]):
                p[key] = v
        a = aim_error(r)
        if a is not None:
            p["aim_sum"] += a
            p["aim_n"] += 1
        p["last"] = r.get("time")
        self.recent = (self.recent + [r])[-50:]

    def add(self, rec: dict, now: float | None = None) -> dict:
        """A throw record from the catch code: stored with the player and the time; returns it as stored."""
        r = dict(rec)
        r["player"] = self.player
        r["time"] = round(now if now is not None else time.time(), 2)
        r["aim"] = aim_error(r)
        self._tally(r)
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.path, "a") as f:
                f.write(json.dumps(r) + "\n")
        return r

    def board(self) -> list:
        """The leaderboard: players by catches, then aim."""
        out = []
        for name, p in self.players.items():
            out.append(dict(player=name, throws=p["throws"], caught=p["caught"],
                            rate=round(100 * p["caught"] / p["throws"]) if p["throws"] else None,
                            best_streak=p["best_streak"], best_kmh=kmh(p["best_speed"]),
                            best_apex=None if p["best_apex"] is None else round(p["best_apex"], 2),
                            aim=round(p["aim_sum"] / p["aim_n"], 2) if p["aim_n"] else None, last=p["last"]))
        out.sort(key=lambda d: (-d["caught"], d["aim"] if d["aim"] is not None else 99))
        return out

    def summary(self) -> dict:
        return dict(player=self.player, board=self.board(),
                    recent=[{k: v for k, v in r.items() if k != "path"} for r in self.recent[-10:]])
