"""Dance party, ROS-free: a routine of moves on the beat, as the base's twist (body frame) at any time, and the face's
tricks on the way. missions.py's DANCE asks twist(t) at 20 Hz (drive_output) and sends the tricks as they come up.

Every move ends where it began: a translation is whole periods of a sine (or a closed square), a turn is whole turns,
and no move translates and turns at once. So a routine of them ends where it started, facing the same way, and the
farthest it strays is reach() (checked against MAX_REACH). The speeds scale with the dance's speed setting and stay
under the firmware's profile anyway (kid 0.8 m/s). The robot is holonomic: slides, circles that keep facing the
room and spins are what an omni base dances best.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

MAX_REACH = 0.6                     # m: no routine strays farther than this from where it started


@dataclass(frozen=True)
class Move:
    name: str
    beats: int
    kind: str                       # slide | rock | circle | box | spin | shimmy | pause
    amp: float = 1.0                # a share of the dance's speed (translations) or of TURN (turns)
    turns: float = 1.0              # spin: whole turns (sign: direction)
    trick: str | None = None        # the face's trick as the move starts (missions.TRICKS)


TURN = 2.5                          # rad/s at amp 1 for shimmies (the firmware caps turning by profile)

MOVES = {
    "slide": Move("slide", 4, "slide"),                         # left and back, then right and back
    "rock": Move("rock", 4, "rock", 0.7),                       # forward and back
    "circle": Move("circle", 8, "circle", 0.8),                 # a circle, still facing the room
    "box": Move("box", 4, "box", 0.6),                          # a square step
    "spin": Move("spin", 4, "spin", turns=1.0, trick="dizzy"),
    "spin_back": Move("spin_back", 4, "spin", turns=-1.0),
    "shimmy": Move("shimmy", 4, "shimmy", 0.5, trick="giggle"),
    "pose": Move("pose", 2, "pause", trick="wink"),
}

ROUTINES = {
    "groove": ("slide", "shimmy", "rock", "pose", "slide", "spin", "box", "pose"),
    "party": ("shimmy", "circle", "spin", "slide", "spin_back", "rock", "shimmy", "pose"),
    "robot": ("box", "pose", "rock", "pose", "slide", "pose", "spin", "pose"),
}


class Dance:
    def __init__(self, routine: str = "groove", bpm: float = 110.0, speed: float = 0.4, seconds: float = 60.0):
        if routine not in ROUTINES:
            raise ValueError(f"routine: one of {', '.join(ROUTINES)}")
        if not 60.0 <= bpm <= 160.0:
            raise ValueError("bpm: 60 to 160")
        self.routine, self.bpm, self.speed = routine, float(bpm), float(speed)
        self.beat = 60.0 / self.bpm
        self.moves = [MOVES[n] for n in ROUTINES[routine]]
        one = sum(m.beats for m in self.moves) * self.beat
        self.laps = max(1, int(round(seconds / one)))
        self.timeline = []                                  # (t0, t1, move)
        t = 0.0
        for _ in range(self.laps):
            for m in self.moves:
                d = m.beats * self.beat
                self.timeline.append((t, t + d, m))
                t += d
        self.length = t
        self.t0 = None
        self.next_trick = 0
        if self.reach() > MAX_REACH:                        # too fast for the beat: slower, never farther
            self.speed *= MAX_REACH / self.reach()

    # -- the moves ------------------------------------------------------------------------------------------------------
    def _twist(self, m: Move, u: float, d: float) -> tuple:
        """The move's twist at u in [0, 1) of its d seconds."""
        v = self.speed * m.amp
        if m.kind == "slide":                               # two whole sine periods: left-back, right-back
            return 0.0, v * math.sin(4 * math.pi * u), 0.0
        if m.kind == "rock":
            return v * math.sin(4 * math.pi * u), 0.0, 0.0
        if m.kind == "circle":                              # one turn of a circle, the heading kept
            return -v * math.sin(2 * math.pi * u), v * math.cos(2 * math.pi * u), 0.0
        if m.kind == "box":                                 # forward, left, back, right: a beat each
            k = min(3, int(u * 4))
            return ((v, 0.0, 0.0), (0.0, v, 0.0), (-v, 0.0, 0.0), (0.0, -v, 0.0))[k]
        if m.kind == "spin":
            return 0.0, 0.0, 2 * math.pi * m.turns / d
        if m.kind == "shimmy":                              # a quick wiggle, a beat a side
            return 0.0, 0.0, TURN * m.amp * math.sin(2 * math.pi * u * m.beats / 2)
        return 0.0, 0.0, 0.0

    def twist_at(self, t: float) -> tuple | None:
        """The twist t s into the dance; None once it is over."""
        if t < 0 or t >= self.length:
            return None
        for t0, t1, m in self.timeline:
            if t0 <= t < t1:
                return self._twist(m, (t - t0) / (t1 - t0), t1 - t0)
        return None

    def tricks_due(self, t: float) -> list:
        """The face's tricks whose moves started by t (each once)."""
        out = []
        while self.next_trick < len(self.timeline) and self.timeline[self.next_trick][0] <= t:
            m = self.timeline[self.next_trick][2]
            if m.trick:
                out.append(m.trick)
            self.next_trick += 1
        return out

    def move_at(self, t: float) -> str:
        for t0, t1, m in self.timeline:
            if t0 <= t < t1:
                return m.name
        return ""

    def reach(self, dt: float = 0.01) -> float:
        """The farthest the routine strays from its start (m), integrating it."""
        x = y = yaw = r = 0.0
        t = 0.0
        while t < self.length / self.laps:
            vx, vy, wz = self.twist_at(t)
            c, s = math.cos(yaw), math.sin(yaw)
            x += (c * vx - s * vy) * dt
            y += (s * vx + c * vy) * dt
            yaw += wz * dt
            r = max(r, math.hypot(x, y))
            t += dt
        return r

    # -- running ------------------------------------------------------------------------------------------------------
    def tick(self, now: float):
        """(twist or "done", the move's name, tricks due) at wall time now."""
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        tw = self.twist_at(t)
        return ("done" if tw is None else tw), self.move_at(t), self.tricks_due(t)
