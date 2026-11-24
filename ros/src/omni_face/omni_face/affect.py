"""The robot's feelings (ROS-free): what it feels from what happens to it, and how much of it may show.

Every feeling comes from something real (Fernando, Nov 21: "a sense that the robot is feeling stuff", Disney-level,
"subtle but there" through tasks). An event the robot sensed or did is appraised (APPRAISAL: event -> an emotion and
how strongly); the emotion moves a fast state (valence, arousal: seconds) that decays back to a slow mood (minutes),
which every event nudges a little. Needs colour the mood: energy (the battery), company (how long since anyone
talked to it or was seen) and boredom (how long since it did anything). Repeated failures build frustration until a
success releases it as pride or relief.

Valence -1..1 (unpleasant .. pleasant), arousal -1..1 (sleepy .. alert). EMOTIONS places each named emotion in that
plane; emotion() says the one it shows now: the last event's while it is still strong, else the mood's nearest.

Levels: how much of it may show, per channel, 0-10 (Fernando, Nov 21: "tunable ... from 0 to 10"). 5 is the
designed default; 0 never; 10 the most. A channel's level sets both how often it reacts (often(): 0 at 0, 1 at 5, 2
at 10, used as a chance or a rate) and how big (size(): level / 10, a share of the designed maximum).
  speech   its spoken lines (faults and answers to what it was asked are always said)
  sounds   its chirps, vocalizations and the antics' sounds (fault alerts always play)
  face     how much the eyes and mouth move and how far its expressions go
  base     wheel gestures when the base is free (a happy wiggle): never during a task
  arm      arm gestures when the arm is free (a nod, a head tilt): never while it holds or picks
  nav      on: its mood may set the pace of its own driving (tired: up to 15 % slower; never faster than set). Off:
           navigation exactly as set, always ("easily disable emotion from interfering with navigation").
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# name -> (valence, arousal)
EMOTIONS = {
    "neutral": (0.0, 0.0), "content": (0.45, -0.2), "happy": (0.7, 0.35), "excited": (0.8, 0.85),
    "proud": (0.75, 0.5), "affection": (0.85, 0.15), "relieved": (0.5, -0.15), "curious": (0.3, 0.5),
    "surprised": (0.15, 0.8), "startled": (-0.35, 0.95), "focused": (0.1, 0.3), "effort": (-0.1, 0.7),
    "worried": (-0.5, 0.5), "frustrated": (-0.6, 0.6), "sad": (-0.6, -0.3), "tired": (-0.2, -0.7),
    "bored": (-0.25, -0.45),
}
# what the mood alone shows (an event's emotion shows only while it is fresh)
MOOD_SHOWS = ("neutral", "content", "happy", "excited", "tired", "bored", "worried", "sad")
# event -> (emotion, intensity 0-1). What happened to it, or what it did
APPRAISAL = {
    "hello": ("happy", 0.4),             # it started up
    "picked": ("proud", 0.55),           # a thing in the bin
    "grab_failed": ("frustrated", 0.4),  # builds with each failure in a row (frustration())
    "too_big": ("surprised", 0.3),
    "tidy_done": ("proud", 0.8), "tidy_empty": ("content", 0.3),
    "arrived": ("content", 0.5), "nav_failed": ("frustrated", 0.55),
    "mission_start": ("focused", 0.35),
    "strain": ("startled", 0.9),         # the arm's reflex fired: something pushed it (design.md 4.5)
    "effort": ("effort", 0.6),           # pulling hard (the arm's force reading near its limit)
    "lost": ("worried", 0.7), "found": ("relieved", 0.6),
    "battery_low": ("tired", 0.6), "fault": ("worried", 0.85),
    "person_found": ("happy", 0.45), "person_lost": ("worried", 0.4), "outrun": ("effort", 0.4),
    "praise": ("affection", 0.8), "love": ("affection", 0.95), "thanks": ("happy", 0.5),
    "greeted": ("happy", 0.55), "heard_name": ("curious", 0.35), "not_understood": ("curious", 0.35),
    "joke": ("happy", 0.45), "asked": ("content", 0.25), "refused": ("sad", 0.3), "stop": ("surprised", 0.35),
    "caught": ("excited", 0.7), "missed": ("sad", 0.4), "streak": ("excited", 0.9),
    "fast": ("excited", 0.6), "dizzy": ("surprised", 0.5), "mapped": ("curious", 0.4),
    "checkpoint": ("content", 0.25), "step": ("focused", 0.2),
}
FAST_TAU = 7.0          # s: an emotion fades back to the mood
MOOD_TAU = 300.0        # s: the mood fades back to its baseline
MOOD_PULL = 0.18        # how much one event (at intensity 1) moves the mood toward it
SHOW_MIN = 0.22         # an event's emotion shows while its strength is above this
LONELY_AFTER = 600.0    # s with nobody: the mood drifts down a little
BORED_AFTER = 240.0     # s with nothing to do


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


@dataclass
class Levels:
    """How much of its feelings may show, per channel, 0-10 (see the module's docstring)."""
    on: bool = True
    speech: float = 5.0
    sounds: float = 5.0
    face: float = 6.0
    base: float = 3.0
    arm: float = 3.0
    nav: bool = True

    def level(self, channel: str) -> float:
        if not self.on:
            return OFF_LEVELS.get(channel, 0.0)
        return clamp(float(getattr(self, channel)), 0.0, 10.0)

    def often(self, channel: str) -> float:
        """How often, as a factor: 0 at level 0, 1 at 5 (the design), 2 at 10."""
        return self.level(channel) / 5.0

    def size(self, channel: str) -> float:
        """How big, as a share of the designed maximum: level / 10."""
        return self.level(channel) / 10.0

    def nav_ok(self) -> bool:
        return self.on and bool(self.nav)

    @staticmethod
    def from_settings(vals: dict, cur: "Levels | None" = None) -> "Levels":
        """The console's settings (emotion.*) over cur (missing keys keep cur's)."""
        c = cur or Levels()
        g = lambda k, d: vals.get(f"emotion.{k}", d)          # noqa: E731
        return Levels(on=bool(g("on", c.on)), speech=float(g("speech", c.speech)), sounds=float(g("sounds", c.sounds)),
                      face=float(g("face", c.face)), base=float(g("base", c.base)), arm=float(g("arm", c.arm)),
                      nav=bool(g("nav", c.nav)))


# emotion off: the robot as it was before (its status moods, lines and antics), no body gestures, navigation as set
OFF_LEVELS = {"speech": 5.0, "sounds": 5.0, "face": 5.0, "base": 0.0, "arm": 0.0}


class Affect:
    """Its feeling now. feel(event) when something happens; tick(now) every update; snapshot() for the face,
    the console (the pace of its driving) and the logs."""

    def __init__(self, now: float = 0.0, baseline=(0.15, 0.0)):
        self.base = baseline
        self.v, self.a = baseline
        self.mv, self.ma = baseline           # the mood
        self.last = ("neutral", 0.0, now)     # the last event's emotion, its intensity, when
        self.event = ""                       # the last event's name
        self.t = now
        self.fails = 0                        # failures in a row (frustration builds)
        self.energy = 1.0                     # 0-1: the battery
        self.t_company = now                  # someone talked to it, or was seen
        self.t_active = now                   # it did something

    # -- what happened ----------------------------------------------------------------------------------------------
    def feel(self, event: str, now: float, intensity: float | None = None) -> tuple:
        """Appraise an event (APPRAISAL; an emotion's name works too): -> (emotion, intensity) it feels now."""
        emo, k = APPRAISAL.get(event, (event, 0.5)) if event not in EMOTIONS else (event, 0.5)
        if emo not in EMOTIONS:
            return "neutral", 0.0
        k = float(intensity if intensity is not None else k)
        if event in ("grab_failed", "nav_failed", "missed", "refused"):
            self.fails += 1
            k = min(1.0, k + 0.15 * (self.fails - 1))             # the third failure in a row stings more
        elif event in ("picked", "tidy_done", "arrived", "caught", "streak", "found"):
            if self.fails >= 2:                                    # it finally worked
                emo, k = ("relieved" if event == "found" else "proud"), min(1.0, k + 0.1 * self.fails)
            self.fails = 0
        if event in ("praise", "love", "thanks", "greeted", "heard_name", "asked", "joke", "person_found"):
            self.t_company = now
        if event not in ("battery_low", "lost", "fault"):
            self.t_active = now
        self.tick(now)
        ev, ea = EMOTIONS[emo]
        self.v += (ev - self.v) * k
        self.a += (ea - self.a) * k
        self.mv += (ev - self.mv) * MOOD_PULL * k
        self.ma += (ea - self.ma) * MOOD_PULL * k
        self.last, self.event = (emo, k, now), event
        return emo, k

    def battery(self, pct: float | None):
        if pct is not None:
            self.energy = clamp(pct / 100.0, 0.0, 1.0)

    def company(self, now: float):
        """Someone is here (seen or heard)."""
        self.t_company = now

    def active(self, now: float):
        self.t_active = now

    # -- time ---------------------------------------------------------------------------------------------------
    def tick(self, now: float):
        dt = max(0.0, now - self.t)
        self.t = now
        if dt == 0.0:
            return
        bv, ba = self.baseline(now)
        km = 1.0 - math.exp(-dt / MOOD_TAU)
        self.mv += (bv - self.mv) * km
        self.ma += (ba - self.ma) * km
        kf = 1.0 - math.exp(-dt / FAST_TAU)
        self.v += (self.mv - self.v) * kf
        self.a += (self.ma - self.a) * kf

    def baseline(self, now: float) -> tuple:
        """Where the mood settles: a little pleasant and calm, lower with a flat battery, less pleasant alone or
        bored for long."""
        v, a = self.base
        low = clamp((0.35 - self.energy) / 0.35, 0.0, 1.0)            # under 35 %: tired
        v -= 0.25 * low
        a -= 0.6 * low
        lonely = clamp((now - self.t_company - LONELY_AFTER) / LONELY_AFTER, 0.0, 1.0)
        bored = clamp((now - self.t_active - BORED_AFTER) / BORED_AFTER, 0.0, 1.0)
        return v - 0.15 * lonely - 0.15 * bored, a - 0.35 * bored

    # -- what shows ----------------------------------------------------------------------------------------------
    def strength(self, now: float) -> float:
        """The last event's emotion now (its intensity, fading)."""
        emo, k, t0 = self.last
        return k * math.exp(-max(0.0, now - t0) / FAST_TAU)

    def emotion(self, now: float) -> tuple:
        """(the emotion it shows, 0-1 how much): the last event's while it is fresh, else the mood's nearest."""
        s = self.strength(now)
        if s >= SHOW_MIN:
            return self.last[0], s
        best = min(MOOD_SHOWS, key=lambda e: (EMOTIONS[e][0] - self.v) ** 2 + (EMOTIONS[e][1] - self.a) ** 2)
        d = math.hypot(*EMOTIONS[best])
        return best, clamp(math.hypot(self.v, self.a) / d if d > 0 else 0.0, 0.0, 1.0)

    def tempo(self) -> float:
        """The pace its own driving may take (0.85-1.0: tired or sad slows it; it never speeds up past what is set)."""
        return clamp(1.0 + 0.15 * min(0.0, self.a) + 0.05 * min(0.0, self.v), 0.85, 1.0)

    def snapshot(self, now: float) -> dict:
        emo, s = self.emotion(now)
        return dict(emotion=emo, strength=round(s, 2), valence=round(self.v, 2), arousal=round(self.a, 2),
                    mood=[round(self.mv, 2), round(self.ma, 2)], energy=round(self.energy, 2), event=self.event,
                    fails=self.fails, tempo=round(self.tempo(), 3))
