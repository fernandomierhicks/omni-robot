"""The face on the 5 in display and the voice on the speaker (pygame: no ROS, no browser, no desktop).

The panel is a 5 in 800 x 480 at the robot's front, between the cameras, leaning back 10 deg behind a polycarbonate
window (design.md 12.2; on the arm robot the same unit stands over the bin, facing forward too). Its viewers: the
thrower at 6-7 m (the eyes, their colour and the score's big digits carry there: text that small is not readable at
that range), kids and adults at 1-2 m (the prompt, what it says, the fault card).

On the robot it runs full screen on the panel, started by systemd at boot (systemd/omni-face.service), and owns the
speaker and the microphones. In the sim it opens an 800 x 480 window on Windows with sound (ros/tools/face_view.py;
the face node starts it). Either way it connects to the face node (face_node.py, TCP, JSON lines) for what to show,
say and listen for; while the robot's software isn't running it asks the console's supervisor
(http://<host>:8080/api/state) and says so.

  python display.py [--host 127.0.0.1] [--port 9881] [--fullscreen] [--scale 1.25] [--mute] [--voice sapi|piper|...]
                    [--listen auto|sapi|vosk|none]
  python display.py --demo            a scripted session (catch game, a fault, follow, driving, antics) with no robot

Drawn at 30 fps on an 800 x 480 canvas (scaled to the window): two eyes (mood: lids, slant, colour; pupils follow
the ball or the person, else wander; blinks), a mouth while it speaks or plays, a top line (what it's doing, the
score, the battery), the prompt or what it's saying, a fault card over everything, a short note for warnings, and
the antics (moments.ANTICS: closed eyes and Zzz, notes while it sings, hearts, spirals, a yawn, a sneeze ...).
Listening (listen.py): the face node says when and for which phrases; what is heard goes back to it. M mutes the
speaker, L the microphone. In a window (the sim) the Launcher's Face buttons set its size, mute it or hide it
(face_settings; UDP 9883 on localhost).
"""

from __future__ import annotations

import argparse
import heapq
import itertools
import json
import math
import os
import queue
import random
import socket
import sys
import threading
import time
import urllib.request
from pathlib import Path

if __package__ in (None, ""):                         # run as a script (ros/tools/face_view.py, systemd)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402

from omni_face import sounds  # noqa: E402
from omni_face.moments import Brain, BrainConfig, FaceState  # noqa: E402
from omni_face.voice import Voice, choose  # noqa: E402

W, H = 800, 480
# the Launcher's face buttons (the sim's window on Windows): UDP datagrams {"mute", "size", "hidden"} on localhost, and
# the same under "face" in the repo's logs/controller.json when the window opens
SIZES = {"small": 0.5, "normal": 1.0, "large": 1.5}
CONTROL_PORT = 9883
BG = (8, 12, 24)
INK = (235, 240, 250)
DIM = (120, 132, 155)
ACCENT = (255, 214, 120)
PINK = (255, 120, 170)
HEARD = (150, 220, 255)
MOODS = {             # lid top, lid bottom, slant deg (+ = outer corner down), scale, colour, pupil size
    "idle": (0.06, 0.0, 0.0, 1.0, (110, 225, 255), 1.0),
    "happy": (0.0, 0.0, 0.0, 1.0, (120, 255, 190), 1.0),
    "sad": (0.34, 0.0, 16.0, 0.95, (110, 150, 255), 1.0),
    "focused": (0.28, 0.18, -8.0, 1.0, (140, 235, 255), 0.85),
    "surprised": (0.0, 0.0, 0.0, 1.12, (150, 235, 255), 0.7),
    "sleepy": (0.58, 0.05, 4.0, 1.0, (90, 170, 210), 1.0),
    "worried": (0.22, 0.0, 12.0, 1.0, (255, 196, 90), 1.0),
    "alert": (0.08, 0.0, -6.0, 1.05, (255, 110, 100), 0.9),
    # the personas' looks at rest (moments.PERSONAS)
    "stern": (0.30, 0.10, -18.0, 0.96, (255, 85, 45), 0.75),       # patrol: narrowed, inner corners down, brows
    "eager": (0.0, 0.0, -4.0, 1.1, (120, 240, 110), 1.1),           # explore: wide open, big pupils
    "cheerful": (0.0, 0.0, 0.0, 1.0, (255, 215, 80), 0.95),         # tidy: smiling eyes (a cheek under each), blush
}
# the personas' colours: the eyes (in the calm moods: sad, worried, alert and sleepy keep their own), the background,
# and an accent (the top line, the scanner, sparkles)
PALETTES = {"patrol": ((255, 85, 45), (30, 6, 4), (255, 150, 60)),
            "explore": ((120, 240, 110), (6, 22, 12), (255, 214, 120)),
            "tidy": ((255, 215, 80), (26, 18, 6), (255, 140, 200))}
CALM = ("idle", "happy", "focused", "surprised", "stern", "eager", "cheerful")
CARD = {"error": (150, 28, 32), "warn": (150, 98, 10)}
# the layout (800 x 480): the eyes take the middle, a top line, a text band at the bottom
TOP_H = 56                         # the top line: what it's doing, the score, the battery
EYE_Y, EYE_DX = 205, 165           # the eyes' centres: (W/2 -+ EYE_DX, EYE_Y)
EYE_W, EYE_H = 172, 196            # an eye at scale 1
MOUTH_Y = 350
TEXT_Y = 392                       # the prompt / caption band (two lines at most)
SUB_Y = 452


def font(size, bold=False):
    return pygame.font.SysFont(["segoeui", "dejavusans", "freesans", "arial"], size, bold=bold)


def wrap(text, f, width):
    words, lines, cur = text.split(), [], ""
    for w in words:
        t = (cur + " " + w).strip()
        if f.size(t)[0] <= width or not cur:
            cur = t
        else:
            lines.append(cur)
            cur = w
    return lines + ([cur] if cur else [])


def _ease(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


def arc(s, col, rect, a0, a1, width):
    """A thick arc without pygame's gaps: thin arcs stacked inward."""
    r = pygame.Rect(rect)
    for i in range(0, int(width), 2):
        pygame.draw.arc(s, col, r.inflate(-i, -i), a0, a1, 3)


def _bump(t, t0, length):
    """0 -> 1 -> 0 over [t0, t0 + length]."""
    k = (t - t0) / length
    return math.sin(math.pi * k) if 0.0 <= k <= 1.0 else 0.0


# -- the voice and the sounds ---------------------------------------------------------------------------------------

class Audio:
    """Effects at once; speech by priority (a higher one interrupts, chatter older than 8 s is dropped), made by a
    worker thread so a new phrase never stalls the drawing. level() is the speech's loudness now (the mouth)."""

    def __init__(self, voice: Voice | None, mute: bool = False, volume: float = 0.8):
        self.ok = False
        self.mute = mute
        try:
            pygame.mixer.init(22050, -16, 2, 512)
            self.ok = True
        except pygame.error as e:
            print(f"no audio device: {e}")
        self.voice, self.volume = voice, volume
        self.fx = {}
        if self.ok:
            rate, _, ch = pygame.mixer.get_init()
            for n in sounds.NAMES + sounds.SONGS:
                a = sounds.sound(n, rate, 0.45)
                self.fx[n] = pygame.sndarray.make_sound(np.repeat(a[:, None], ch, axis=1) if ch > 1 else a)
            self.chan = pygame.mixer.Channel(0)
            pygame.mixer.set_reserved(1)
        self.todo: queue.Queue = queue.Queue()
        self.ready: list = []                         # heap: (-prio, seq, text, path, t_req)
        self.seq = itertools.count()
        self.lock = threading.Lock()
        self.cur = None                               # (prio, text, t_start, envelope)
        self.caption = ("", 0.0)
        threading.Thread(target=self._worker, daemon=True).start()

    def play(self, name):
        if self.ok and not self.mute and name in self.fx:
            self.fx[name].set_volume(self.volume * 0.7)
            self.fx[name].play()

    def say(self, text, prio=1, style=""):
        self.todo.put((prio, next(self.seq), text, style, time.monotonic()))

    def busy(self) -> bool:
        """Something is playing (its own voice or a sound): what the microphones hear now is mostly itself."""
        return self.cur is not None or (self.ok and pygame.mixer.get_busy())

    def _worker(self):
        while True:
            prio, seq, text, style, t_req = self.todo.get()
            path = self.voice.wav(text, style) if self.voice is not None else None
            with self.lock:
                heapq.heappush(self.ready, (-prio, seq, text, path, t_req))

    def tick(self):
        """Start the next phrase when its turn comes (call every frame)."""
        now = time.monotonic()
        with self.lock:
            busy = self.cur is not None and (self.ok and self.chan.get_busy() or now - self.cur[2] < self.cur[4])
            if not busy:
                self.cur = None
            while self.ready:
                p, seq, text, path, t_req = self.ready[0]
                if -p <= 1 and now - t_req > 8.0:                # stale chatter
                    heapq.heappop(self.ready)
                    continue
                if busy and -p <= self.cur[0]:
                    break
                heapq.heappop(self.ready)
                self._start(-p, text, path, now)
                busy = True
                break

    def _start(self, prio, text, path, now):
        env, dur = np.zeros(1), 0.4 + 0.06 * len(text)            # no voice: the caption stays as long as it'd take
        if path is not None and self.ok and not self.mute:
            try:
                s = pygame.mixer.Sound(str(path))
                s.set_volume(self.volume)
                self.chan.stop()
                self.chan.play(s)
                a = pygame.sndarray.array(s).astype(float)
                a = a.mean(axis=1) if a.ndim > 1 else a
                rate = pygame.mixer.get_init()[0]
                hop = max(1, rate // 30)
                env = np.sqrt(np.convolve(a ** 2, np.ones(hop) / hop, "valid")[::hop]) / 32767.0
                env = np.clip(env / max(env.max(), 1e-6), 0, 1)
                dur = s.get_length()
            except pygame.error:
                pass
        self.cur = (prio, text, now, env, dur)
        self.caption = (text, now + dur + 1.2)

    def level(self) -> float:
        c = self.cur
        if c is None:
            return 0.0
        k = int((time.monotonic() - c[2]) * 30)
        if len(c[3]) > 1:
            return float(c[3][k]) if k < len(c[3]) else 0.0
        return 0.5 + 0.5 * math.sin(time.monotonic() * 18) if k < c[4] * 30 else 0.0

    def caption_now(self) -> str:
        return self.caption[0] if time.monotonic() < self.caption[1] else ""


# -- the picture ----------------------------------------------------------------------------------------------------

class FaceView:
    def __init__(self):
        self.gaze = np.zeros(2)
        self.wander = np.zeros(2)
        self.t_wander = 0.0
        self.t_blink, self.blink = time.monotonic() + 3.0, 0.0
        self.p = np.array(MOODS["idle"][:4], float)
        self.col = np.array(MOODS["idle"][4], float)
        self.bgv = np.array(BG, float)
        self.bg = BG                                # the background now (the lids and cut-outs are drawn in it)
        self.pupil = 1.0
        self.act, self.act_id, self.act_s, self.t_act = "", 0, 0.0, 0.0
        self.f = dict(big=font(44, True), mid=font(34), small=font(24), tiny=font(20), title=font(48, True),
                      hint=font(28), score=font(40, True), z=[font(22, True), font(30, True), font(40, True)],
                      mark=font(52, True))

    # -- animation state ---------------------------------------------------------------------------------------
    def _anim(self, st: FaceState, dt):
        now = time.monotonic()
        if st.act_id != self.act_id:                # a new antic: its animation starts now
            self.act, self.act_id, self.act_s, self.t_act = st.act, st.act_id, st.act_s, now
        if not st.act:
            self.act = ""
        m = MOODS.get(st.mood, MOODS["idle"])
        pal = PALETTES.get(st.persona)
        k = min(1.0, dt * 8)
        self.p += (np.array(m[:4]) - self.p) * k
        self.col += (np.array(pal[0] if pal and st.mood in CALM else m[4]) - self.col) * k
        self.bgv += (np.array(pal[1] if pal else BG) - self.bgv) * min(1.0, dt * 3)
        self.bg = tuple(int(round(c)) for c in self.bgv)
        self.pupil += (m[5] - self.pupil) * k
        target = self._act_gaze(now)
        if target is None and st.gaze is None:
            if now > self.t_wander:
                self.t_wander = now + random.uniform(1.5, 4.0)
                self.wander = np.array([random.uniform(-0.6, 0.6), random.uniform(-0.3, 0.3)])
            target = self.wander
        elif target is None:
            target = np.array(st.gaze, float)
        self.gaze += (target - self.gaze) * min(1.0, dt * 10)
        if now > self.t_blink:                      # blink: 0.15 s, every 2-6 s (slower when sleepy)
            self.blink = 0.15
            self.t_blink = now + random.uniform(2.0, 6.0) * (2.0 if st.mood == "sleepy" else 1.0)
        self.blink = max(0.0, self.blink - dt)

    def _t(self):
        """Seconds into the current antic."""
        return time.monotonic() - self.t_act

    def _act_gaze(self, now):
        t = now - self.t_act
        if self.act == "look":                      # left, right, up, then back
            seq = [(-0.9, 0.1), (0.9, 0.1), (0.2, 0.8), (0.0, 0.0)]
            return np.array(seq[min(3, int(t / 0.8))])
        if self.act == "babble":
            return np.array([0.6 * math.sin(t * 7.3), 0.3 * math.sin(t * 5.1 + 1)])
        if self.act == "think":
            return np.array([0.6, 0.7])
        if self.act in ("nap", "sigh"):
            return np.array([0.0, -0.4])
        if self.act == "whistle":
            return np.array([0.3, 0.5])
        if self.act == "scan":                      # a slow sweep to one side, the other, and back
            return np.array([0.95 * math.sin(2 * math.pi * t / max(self.act_s, 0.5)), -0.1])
        return None

    def _offset(self):
        """The whole face's offset (px): bobbing to a song, a hiccup's jump, a sneeze's jolt, a dizzy wobble."""
        a, t = self.act, self._t()
        if a in ("sing", "hum"):
            return 0.0, -10 * abs(math.sin(math.pi * 2.0 * t))
        if a == "dance":
            return 34 * math.sin(2 * math.pi * 0.9 * t), -12 * abs(math.sin(2 * math.pi * 1.8 * t))
        if a == "giggle":
            return 0.0, -8 * abs(math.sin(math.pi * 7 * t))
        if a == "hiccup":
            return 0.0, -22 * max(_bump(t, 0.0, 0.25), _bump(t, 1.4, 0.25), _bump(t, 2.5, 0.25))
        if a == "sneeze":
            if 0.85 <= t < 1.4:
                return 10 * math.sin(t * 90), 18 * _bump(t, 0.85, 0.55)
            return 0.0, -8 * _ease(t / 0.8) if t < 0.85 else 0.0
        if a == "dizzy":
            return 14 * math.cos(t * 5), 8 * math.sin(t * 5)
        if a == "wheee":
            return 6 * math.sin(t * 40), 0.0
        if a == "sigh":
            return 0.0, 10 * _bump(t, 0.2, 1.6)
        return 0.0, 0.0

    # -- the eyes ------------------------------------------------------------------------------------------------
    def _eye(self, s, cx, cy, side, st):
        a, t = self.act, self._t()
        col = tuple(int(c) for c in self.col)
        top, bot, slant, scale = self.p
        w, h = EYE_W * scale, EYE_H * scale
        if a == "listen" or a == "wheee":
            w, h = w * 1.08, h * 1.08
        r = pygame.Rect(0, 0, w, h)
        r.center = (cx, cy)
        # the antics that replace the eye
        if a == "love":
            k = 1.0 + 0.08 * math.sin(t * 9)
            self._heart(s, cx, cy + 6, 0.55 * w * k, PINK)
            return
        if a == "dizzy":
            self._spiral(s, cx, cy, 0.48 * w, t * (6 if side < 0 else -6), col)
            return
        closed = (a == "nap" and 0.4 < t < self.act_s - 0.6) or (a in ("sing", "hum", "dance")) or \
            (a == "wink" and side > 0 and 0.15 < t < 0.75) or (a == "sneeze" and 0.85 <= t < 1.5) or \
            (a == "giggle") or (a == "yawn" and 0.6 < t < 2.0)
        if closed:
            self._closed(s, cx, cy, w, col, a, side)
            return
        pygame.draw.rect(s, col, r, border_radius=int(w * 0.42))
        # pupil and its glint
        px, py = cx + self.gaze[0] * w * 0.26, cy - self.gaze[1] * h * 0.22
        pr = 42 * self.pupil * scale
        if st.mood != "happy":                      # happy eyes are plain arcs
            pygame.draw.circle(s, (10, 18, 36), (px, py), pr)
            pygame.draw.circle(s, (240, 250, 255), (px + pr * 0.35, py - pr * 0.4), pr * 0.28)
        if st.mood == "happy":                      # "^ ^": the lower half covered by the cheek
            pygame.draw.ellipse(s, self.bg, pygame.Rect(cx - w * 0.75, cy - h * 0.05, w * 1.5, h * 1.1))
        elif st.mood == "cheerful":                 # smiling eyes: a round cheek pushes up from below, blushing
            pygame.draw.ellipse(s, self.bg, pygame.Rect(cx - w * 0.62, cy + h * 0.14, w * 1.24, h * 0.8))
            pygame.draw.ellipse(s, (215, 95, 130), pygame.Rect(cx - 42 + side * 34, cy + h * 0.44, 84, 28))
        # lids: the top one slanted (outer corner = away from the face's centre line), the bottom one flat
        extra = 0.0
        if a == "yawn":
            extra = 0.55 * _bump(t, 0.2, 2.2)
        elif a == "sneeze" and t < 0.85:            # ah... ah...
            extra = 0.45 * _ease(t / 0.8)
        elif a == "nap":                            # closing at the start
            extra = _ease(t / 0.4) if t < 0.4 else 0.0
        elif a == "sigh":
            extra = 0.25
        cover = min(1.0, top + extra + (1.0 if self.blink > 0 and a != "listen" else 0.0))
        if a == "nap" and t >= self.act_s - 0.6:    # "huh?": wide awake at the end
            cover = 0.0
        outer, inner = (r.left, r.right) if side < 0 else (r.right, r.left)
        dy = math.tan(math.radians(slant)) * w / 2
        y_edge = r.top + cover * h
        pts = [(outer, r.top - 40), (inner, r.top - 40), (inner, y_edge - dy), (outer, y_edge + dy)]
        pygame.draw.polygon(s, self.bg, pts)
        if bot > 0:
            pygame.draw.rect(s, self.bg, pygame.Rect(r.left - 5, r.bottom - bot * h, w + 10, bot * h + 10))
        if st.mood == "stern":                      # brows over the lids (not following a blink): inner ends down
            yb = r.top + top * h
            pygame.draw.line(s, col, (outer, yb + dy - 48), (inner, yb - dy - 18), 14)

    def _closed(self, s, cx, cy, w, col, act, side):
        """A closed eye: a smiling arc (singing, giggling), a sleeping one, or a squeezed > < (a sneeze)."""
        th = max(8, int(w * 0.09))
        if act == "sneeze" or act == "yawn":
            d = w * 0.32 * (-side)                   # > on the viewer's left, < on the right
            pts = [(cx - d, cy - w * 0.28), (cx + d, cy), (cx - d, cy + w * 0.28)]
            pygame.draw.lines(s, col, False, pts, th)
            return
        rr = pygame.Rect(cx - w * 0.42, cy - w * 0.3, w * 0.84, w * 0.6)
        if act == "nap":                            # asleep: the arc hangs down
            arc(s, col, rr.move(0, -w * 0.15), math.pi * 1.05, math.pi * 1.95, th)
        else:                                       # content: "^"
            arc(s, col, rr, math.pi * 0.08, math.pi * 0.92, th)

    @staticmethod
    def _heart(s, cx, cy, size, col):
        pts = []
        for i in range(48):
            u = 2 * math.pi * i / 48
            x = 16 * math.sin(u) ** 3
            y = 13 * math.cos(u) - 5 * math.cos(2 * u) - 2 * math.cos(3 * u) - math.cos(4 * u)
            pts.append((cx + x * size / 17, cy - y * size / 17))
        pygame.draw.polygon(s, col, pts)

    @staticmethod
    def _spiral(s, cx, cy, radius, rot, col):
        pts = []
        for i in range(90):
            u = i / 89 * 3.2 * math.pi
            rr = radius * u / (3.2 * math.pi)
            pts.append((cx + rr * math.cos(u + rot), cy + rr * math.sin(u + rot)))
        pygame.draw.lines(s, col, False, pts, 9)

    # -- the mouth ------------------------------------------------------------------------------------------------
    def _mouth(self, s, cx, cy, speech_level, mood="idle"):
        a, t = self.act, self._t()
        col = tuple(int(c) for c in self.col)
        if speech_level > 0.02 and a not in ("yawn", "sneeze"):
            mh = 8 + 30 * speech_level
            if a in ("smile", "love", "giggle", "wheee") or a == "":
                pygame.draw.rect(s, col, pygame.Rect(cx - 62, cy - mh / 2, 124, mh), border_radius=int(mh / 2))
            else:
                pygame.draw.ellipse(s, col, pygame.Rect(cx - 30, cy - mh / 2, 60, mh))
            return
        if a == "yawn":
            k = _bump(t, 0.2, 2.2)
            if k > 0.02:
                pygame.draw.ellipse(s, col, pygame.Rect(cx - 32 - 14 * k, cy - 46 * k, 64 + 28 * k, 92 * k + 6))
                pygame.draw.ellipse(s, self.bg, pygame.Rect(cx - 22 - 10 * k, cy - 36 * k, 44 + 20 * k, 72 * k + 2))
        elif a in ("sing", "dance"):
            o = 12 + 12 * abs(math.sin(math.pi * 2.0 * t))
            pygame.draw.ellipse(s, col, pygame.Rect(cx - o * 0.8, cy - o, o * 1.6, o * 2))
        elif a in ("hum", "smile", "love", "wink", "quip"):
            arc(s, col, pygame.Rect(cx - 60, cy - 40, 120, 60), math.pi * 1.1, math.pi * 1.9, 9)
        elif a in ("giggle", "wheee"):
            pygame.draw.ellipse(s, col, pygame.Rect(cx - 50, cy - 24, 100, 52))
            pygame.draw.rect(s, self.bg, pygame.Rect(cx - 52, cy - 26, 104, 26))
        elif a == "whistle":
            pygame.draw.circle(s, col, (cx + 18, cy), 13, 6)
        elif a == "sneeze" and 0.85 <= t < 1.4:
            pygame.draw.ellipse(s, col, pygame.Rect(cx - 26, cy - 18, 52, 36))
        elif a == "dizzy":
            pts = [(cx - 60 + i * 6, cy + 8 * math.sin(i * 0.9 + t * 8)) for i in range(21)]
            pygame.draw.lines(s, col, False, pts, 7)
        elif a == "sigh":
            pygame.draw.line(s, col, (cx - 34, cy), (cx + 34, cy + 4), 7)
        elif a == "babble":
            mh = 8 + 18 * abs(math.sin(t * 17))
            pygame.draw.rect(s, col, pygame.Rect(cx - 40, cy - mh / 2, 80, mh), border_radius=int(mh / 2))
        elif a == "listen":
            pygame.draw.circle(s, col, (cx, cy), 9)
        elif a in ("", "scan", "look") and mood == "stern":       # the personas' mouths at rest: a hard line
            pygame.draw.line(s, col, (cx - 46, cy + 6), (cx + 46, cy + 6), 9)
        elif a in ("", "look") and mood == "cheerful":            # a smile
            arc(s, col, pygame.Rect(cx - 60, cy - 40, 120, 60), math.pi * 1.1, math.pi * 1.9, 9)
        elif a in ("", "look") and mood == "eager":               # an open grin
            pygame.draw.ellipse(s, col, pygame.Rect(cx - 40, cy - 18, 80, 42))
            pygame.draw.rect(s, self.bg, pygame.Rect(cx - 42, cy - 20, 84, 20))

    # -- what floats around it ----------------------------------------------------------------------------------
    def _extras(self, s, ox, oy, st=None):
        a, t = self.act, self._t()
        pal = PALETTES.get(st.persona) if st is not None else None
        if a == "scan" and pal:                     # patrol: a scanner bar across both eyes, down and up
            y = EYE_Y + oy + 80 * math.sin(2 * math.pi * t / 1.4)
            x0, x1 = W / 2 - EYE_DX - EYE_W / 2 - 20 + ox, W / 2 + EYE_DX + EYE_W / 2 + 20 + ox
            pygame.draw.line(s, pal[2], (x0, y), (x1, y), 5)
            pygame.draw.line(s, tuple(c // 2 for c in pal[2]), (x0, y - 7), (x1, y - 7), 3)
        elif a == "" and st is not None and st.persona == "tidy" and not st.card:   # a sparkly clean room
            tt = time.monotonic()
            for i, (x, y) in enumerate(((W / 2 - EYE_DX - 125, EYE_Y - 95), (W / 2 + EYE_DX + 120, EYE_Y - 70),
                                        (W / 2 + EYE_DX + 95, EYE_Y + 105))):
                k = max(0.0, math.sin(tt * 1.7 + i * 2.1))
                if k > 0.05:
                    self._sparkle(s, x + ox, y + oy, 6 + 14 * k, pal[2])
        if a == "nap" and 0.6 < t < self.act_s - 0.6:
            for i in range(3):                      # z z Z rising from the right eye
                k = ((t - 0.6) * 0.5 + i / 3) % 1.0
                img = self.f["z"][i].render("z" if i < 2 else "Z", True, INK)
                img.set_alpha(int(255 * math.sin(math.pi * k)))
                s.blit(img, (W / 2 + EYE_DX + 60 + 40 * k + i * 12 + ox, EYE_Y - 30 - 100 * k + oy))
        elif a in ("sing", "hum", "dance", "whistle"):
            n = 1 if a == "whistle" else 3
            for i in range(n):
                k = (t * 0.45 + i / n) % 1.0
                side = 1 if i % 2 == 0 else -1
                self._note(s, W / 2 + side * (EYE_DX + 120) + 25 * math.sin(k * 6) + ox, MOUTH_Y - 40 - 220 * k + oy,
                           int(255 * math.sin(math.pi * k)))
        elif a == "love":
            for i in range(4):
                k = (t * 0.5 + i / 4) % 1.0
                x = W / 2 + (-1) ** i * (90 + 30 * i) + ox
                self._heart(s, x, MOUTH_Y + 10 - 260 * k + oy, 14 + 4 * i, PINK)
        elif a == "wink" and 0.15 < t < 0.95:
            self._sparkle(s, W / 2 + EYE_DX + 105 + ox, EYE_Y - 85 + oy, 18 + 10 * _bump(t, 0.15, 0.8))
        elif a == "look" and t > 0.4:
            img = self.f["mark"].render("?", True, ACCENT)
            s.blit(img, (W / 2 + EYE_DX + 95 + ox, EYE_Y - 140 + oy))
        elif a == "think":
            for i in range(3):
                if t * 3 > i:
                    pygame.draw.circle(s, ACCENT, (W / 2 + EYE_DX + 90 + i * 22 + ox, EYE_Y - 120 - i * 14 + oy),
                                       5 + 2 * i)
        elif a == "listen":
            for i in range(3):                      # sound waves coming in at both sides
                k = (t * 1.2 + i / 3) % 1.0
                for side in (-1, 1):
                    rr = 28 + 70 * (1 - k)
                    rect = pygame.Rect(0, 0, rr * 2, rr * 2)
                    rect.center = (W / 2 + side * (EYE_DX + 150), EYE_Y)
                    c = tuple(int(x * (0.3 + 0.7 * math.sin(math.pi * k))) for x in HEARD)
                    a0 = -0.6 if side > 0 else math.pi - 0.6
                    pygame.draw.arc(s, c, rect, a0, a0 + 1.2, 5)
        elif a == "wheee":
            for i in range(4):                      # speed lines
                y = EYE_Y - 80 + i * 50 + oy
                x0 = 20 + 30 * ((t * 3 + i * 0.3) % 1.0)
                pygame.draw.line(s, DIM, (x0, y), (x0 + 60, y), 4)
                pygame.draw.line(s, DIM, (W - x0 - 60, y), (W - x0, y), 4)

    def _note(self, s, x, y, alpha):
        img = pygame.Surface((44, 64), pygame.SRCALPHA)
        c = (*ACCENT, max(0, min(255, alpha)))
        pygame.draw.ellipse(img, c, pygame.Rect(2, 44, 22, 16))
        pygame.draw.line(img, c, (22, 52), (22, 6), 4)
        pygame.draw.polygon(img, c, [(22, 4), (40, 14), (40, 24), (22, 14)])
        s.blit(img, (x - 22, y - 32))

    @staticmethod
    def _sparkle(s, x, y, r, col=ACCENT):
        pts = []
        for i in range(8):
            u = math.pi / 4 * i
            rr = r if i % 2 == 0 else r * 0.3
            pts.append((x + rr * math.cos(u), y + rr * math.sin(u)))
        pygame.draw.polygon(s, col, pts)

    # -- the whole face -----------------------------------------------------------------------------------------
    def draw(self, s, st: FaceState, dt, speech_level=0.0, caption="", note="", mic=None):
        self._anim(st, dt)
        s.fill(self.bg)
        ox, oy = self._offset()
        self._eye(s, W / 2 - EYE_DX + ox, EYE_Y + oy, -1, st)
        self._eye(s, W / 2 + EYE_DX + ox, EYE_Y + oy, 1, st)
        self._mouth(s, W / 2 + ox, MOUTH_Y + oy, speech_level, st.mood)
        self._extras(s, ox, oy, st)
        # the top line: what it's doing (in the persona's accent), the score, the battery
        f = self.f
        if st.label:
            pal = PALETTES.get(st.persona)
            s.blit(f["small"].render(st.label, True, pal[2] if pal else DIM), (18, 14))
        if st.score:
            sc = st.score
            big = f["score"].render(f"{sc['caught']} / {sc['throws']}", True, INK)
            x = W / 2 - big.get_width() / 2
            s.blit(big, (x, 4))
            name = f["tiny"].render(sc["player"], True, DIM)
            s.blit(name, (x - name.get_width() - 12, 18))
            if sc["streak"] >= 2:
                t = f["tiny"].render(f"streak {sc['streak']}", True, ACCENT)
                s.blit(t, (x + big.get_width() + 12, 18))
        if st.battery is not None:
            self._battery(s, st.battery, st.battery_low)
        if mic is not None:
            self._mic(s, W - 160, 16, mic)
        if st.heard:                                 # what it heard, for a moment, under the top line
            t = f["tiny"].render(f"“{st.heard}”", True, HEARD)
            s.blit(t, (W / 2 - t.get_width() / 2, TOP_H + 2))
        elif st.toast:
            t = f["tiny"].render(st.toast, True, (255, 220, 150))
            s.blit(t, (W / 2 - t.get_width() / 2, TOP_H + 2))
        # the prompt, or what it's saying
        text, fnt, colr = (caption, f["mid"], INK) if caption else (st.prompt, f["big"], INK)
        if not text and st.listening:
            text, fnt, colr = "Listening...", f["mid"], HEARD
        if note and not text:
            text, fnt, colr = note, f["mid"], DIM
        if text:
            lines = wrap(text, fnt, W - 60)[:2]
            y0 = TEXT_Y - (16 if len(lines) > 1 else 0)
            for i, line in enumerate(lines):
                t = fnt.render(line, True, colr)
                s.blit(t, (W / 2 - t.get_width() / 2, y0 + i * (fnt.get_height() - 2)))
        if st.sub and not caption:
            t = f["tiny"].render(st.sub, True, DIM)
            s.blit(t, (W / 2 - t.get_width() / 2, SUB_Y))
        if st.card:
            self._card(s, st.card)

    def _battery(self, s, pct, low):
        x, y, w, h = W - 76, 16, 52, 24
        col = (240, 80, 70) if low or pct < 15 else (120, 220, 140) if pct > 40 else (240, 200, 90)
        pygame.draw.rect(s, DIM, (x, y, w, h), 2, border_radius=5)
        pygame.draw.rect(s, DIM, (x + w, y + 7, 4, 10), border_radius=2)
        pygame.draw.rect(s, col, (x + 4, y + 4, (w - 8) * max(0.0, min(1.0, pct / 100)), h - 8), border_radius=3)
        t = self.f["tiny"].render(f"{pct:.0f}%", True, col)
        s.blit(t, (x - t.get_width() - 6, y))

    @staticmethod
    def _mic(s, x, y, on):
        """A small microphone: it listens (on), or the microphone is off (L)."""
        col = HEARD if on else (90, 96, 110)
        pygame.draw.rect(s, col, pygame.Rect(x, y, 12, 18), border_radius=6)
        pygame.draw.arc(s, col, pygame.Rect(x - 5, y + 4, 22, 20), math.pi * 1.0, math.pi * 2.0, 2)
        pygame.draw.line(s, col, (x + 6, y + 23), (x + 6, y + 27), 2)
        if not on:
            pygame.draw.line(s, (220, 90, 90), (x - 6, y - 2), (x + 18, y + 28), 3)

    def _card(self, s, card):
        shade = pygame.Surface((W, H), pygame.SRCALPHA)
        shade.fill((0, 0, 0, 150))
        s.blit(shade, (0, 0))
        r = pygame.Rect(28, 52, W - 56, H - 104)
        pygame.draw.rect(s, CARD.get(card.get("level"), CARD["warn"]), r, border_radius=22)
        cx, top = r.left + 56, r.top + 44                          # a warning triangle
        pygame.draw.polygon(s, INK, [(cx, top - 18), (cx - 30, top + 34), (cx + 30, top + 34)])
        s.blit(self.f["title"].render("!", True, CARD.get(card.get("level"), CARD["warn"])), (cx - 8, top - 18))
        y = r.top + 24
        for line in wrap(card.get("title", ""), self.f["title"], r.width - 130)[:2]:
            s.blit(self.f["title"].render(line, True, INK), (r.left + 104, y))
            y += self.f["title"].get_height()
        y += 14
        for line in wrap(card.get("hint", ""), self.f["hint"], r.width - 60)[:6]:
            s.blit(self.f["hint"].render(line, True, INK), (r.left + 30, y))
            y += self.f["hint"].get_height() + 2


# -- what feeds it ------------------------------------------------------------------------------------------------------

class Link:
    """The face node's stream (JSON lines): the latest face, what to say / play since the last line, whether to
    listen and for which phrases. send() puts a line back (what was heard)."""

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.face, self.t_face = None, 0.0
        self.listen, self.grammar = False, []
        self.outs: queue.Queue = queue.Queue()
        self.sock = None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                with socket.create_connection((self.host, self.port), timeout=2.0) as s:
                    s.settimeout(3.0)
                    self.sock = s
                    buf = b""
                    while True:
                        d = s.recv(65536)
                        if not d:
                            break
                        buf += d
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            m = json.loads(line)
                            if "face" in m:
                                self.face, self.t_face = m["face"], time.monotonic()
                            if "listen" in m:
                                self.listen = bool(m["listen"])
                            if "grammar" in m:
                                self.grammar = list(m["grammar"])
                            for o in m.get("out", []):
                                self.outs.put(o)
            except (OSError, ValueError):
                pass
            self.sock = None
            time.sleep(1.0)

    def send(self, obj) -> bool:
        s = self.sock
        if s is None:
            return False
        try:
            s.sendall((json.dumps(obj) + "\n").encode())
            return True
        except OSError:
            return False

    def connected(self) -> bool:
        return self.face is not None and time.monotonic() - self.t_face < 2.0


class Supervisor:
    """While the face node is away: what the console's supervisor says (software stopped, starting, crashed)."""

    def __init__(self, url):
        self.url, self.text = url, ""
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=1.0) as r:
                    st = json.loads(r.read())
                cur = st.get("current")
                if not cur:
                    self.text = "My software is stopped. Start it from the console."
                elif cur.get("state") in ("starting", "running"):
                    self.text = f"Starting: {cur.get('label') or cur.get('id')}..."
                elif cur.get("state") == "stopping":
                    self.text = "Stopping..."
                else:
                    self.text = f"{cur.get('label') or cur.get('id')} {cur.get('state')}. Check the console."
            except (OSError, ValueError):
                self.text = "Waiting for my software..."
            time.sleep(2.0)


DEMO_S = 136.0


def demo_status(t: float):
    """A scripted 136 s session for --demo: (status, pose, body twist) at time t."""
    t = t % DEMO_S
    st = dict(mission="IDLE", player="Ana", mcu=dict(mode="AUTO", profile="adult", faults=[], battery_pct=82,
                                                      battery_low=False, age=0.05))
    pose, vel = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    if 4 <= t < 44:                                     # catch: a throw every 8 s from 6 m in front
        st["mission"] = "CATCH"
        k = int((t - 4) // 8)
        ph = (t - 4) % 8
        state = "wait" if ph < 3 else "flight" if ph < 4.3 else "settle" if ph < 5 else "launch" if ph < 6 else "wait"
        caught_n = [1, 1, 1, 0, 1][min(k, 4)]
        tosses = k + (1 if ph >= 4.3 else 0)
        catches = sum([1, 1, 1, 0, 1][:k]) + (caught_n if ph >= 4.3 else 0)
        ball = None
        if 3 <= ph < 4.3:
            f = (ph - 3) / 1.3
            ball = [6.0 * (1 - f), 0.4 * math.sin(k), 1.2 + 2.0 * math.sin(math.pi * f)]
        decided = k + 1 if ph >= 4.3 else k              # the record of the last decided throw (game.last)
        st["catch"] = dict(state=state, tosses=tosses, catches=catches, ball=ball,
                           last={} if decided == 0 else dict(toss=decided, caught=bool([1, 1, 1, 0, 1][min(decided - 1, 4)]),
                                                             speed=9.5 if decided == 3 else 7.0, predicted=[0.2, 0.1]))
    elif 44 <= t < 52:                                  # a bumper fault
        st["mcu"]["faults"] = [dict(name="BUMPER", hint="The bumper is pressed. Move the robot clear by hand.")]
    elif 52 <= t < 62:                                  # follow a person wandering ahead of it
        st["mission"] = "FOLLOW"
        st["follow"] = dict(text="Following Ana", person=[1.5, 0.6 * math.sin(t)])
        vel = (0.5, 0.0, 0.3 * math.cos(t))
    elif 62 <= t < 72:                                  # driven by hand: fast, then a spin
        st["mission"] = "MANUAL"
        st["manual"] = dict(active=True)
        vel = (1.6, 0.0, 0.0) if t < 65 else (0.0, 0.0, 4.0) if t < 69 else (0.0, 0.0, 0.0)
    elif 72 <= t < 80:                                  # go to the kitchen: start, halfway, almost there, arrived
        f = (t - 72) / 7.0
        st["mission"], st["progress"] = ("GOTO", min(1.0, f)) if f < 1 else ("MANUAL", None)
        st["goal"] = dict(x=6.0, y=0.0, yaw=0.0, name="kitchen")
        st["task"] = f"Going to kitchen: {6.0 * (1 - f):.1f} m left" if f < 1 else "Arrived at kitchen"
        vel = (0.8, 0.0, 0.0) if f < 1 else vel
    elif 92 <= t < 104:                                 # the personas: patrol (stern, red)
        st["mission"], st["task"] = "PATROL", "Patrolling: point 2 of 4"
        st["patrol"] = dict(route="house", i=1, n=4, lap=0 if t < 100 else 1, laps=0)
        vel = (0.4, 0.0, 0.0)
    elif 104 <= t < 114:                                # explore (adventurous, green)
        st["mission"], st["task"] = "EXPLORE", "Exploring: heading for an opening"
        vel = (0.4, 0.0, 0.2)
    elif 114 <= t < 126:                                # tidy (a joyful worker, yellow): one in the bin every 4 s
        st["mission"], st["task"] = "TIDY", "Tidying: picking up"
        st["arm"] = dict(tidy_on=True, tidy=dict(state="PICK", picked=int((t - 114) // 4)))
    elif t >= 126:                                      # low battery
        st["mcu"]["battery_pct"], st["mcu"]["battery_low"] = 8, True
    return st, pose, vel                                # 80-92: resting (the antics)


def face_settings(msg, cur: dict) -> dict:
    """cur with what msg changes: mute (bool), size (SIZES), hidden (bool); anything else is ignored."""
    out = dict(cur)
    if isinstance(msg, dict):
        for k in ("mute", "hidden"):
            if isinstance(msg.get(k), bool):
                out[k] = msg[k]
        if msg.get("size") in SIZES:
            out["size"] = msg["size"]
    return out


def saved_face() -> dict:
    """The Launcher's last face settings (the repo's logs/controller.json), {} without them (the robot)."""
    try:
        return json.loads((Path(__file__).resolve().parents[4] / "logs" / "controller.json").read_text()).get("face") or {}
    except (OSError, ValueError, IndexError, AttributeError):
        return {}


def control_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", CONTROL_PORT))
    except OSError:
        return None
    s.setblocking(False)
    return s


def show_window(visible: bool):
    try:
        from pygame._sdl2.video import Window
        w = Window.from_display_module()
        w.show() if visible else w.hide()
    except Exception as e:                              # noqa: BLE001  (an older pygame: minimise instead)
        if not visible:
            pygame.display.iconify()
        print(f"face window: {e}")


def main():
    from omni_face.listen import choose as choose_listener
    ap = argparse.ArgumentParser(description="the robot's face and voice")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9881)
    ap.add_argument("--supervisor", default=None, help="the console's /api/state (default http://<host>:8080)")
    ap.add_argument("--fullscreen", action="store_true")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--mute", action="store_true")
    ap.add_argument("--voice", default=None, help="kokoro | piper | espeak | sapi | none (default: the first that works)")
    ap.add_argument("--listen", default=os.environ.get("OMNI_FACE_LISTEN", "auto"),
                    help="vosk | sapi | none | auto (the first that works); it listens only while the face node says "
                         "so (the console's Face tab: voice commands)")
    ap.add_argument("--demo", action="store_true", help="a scripted session, no robot needed")
    ap.add_argument("--shot", default=None, help="save a screenshot here every 2 s (tests, docs)")
    a = ap.parse_args()
    guard = socket.socket()                             # one face per computer
    try:
        guard.bind(("127.0.0.1", a.port + 1))
    except OSError:
        print("the face is already running")
        return
    pygame.init()
    flags = pygame.FULLSCREEN if a.fullscreen else 0
    cur = dict(mute=a.mute, size=None, hidden=False)
    ctl = None
    if not a.fullscreen:                                # the sim's window: the Launcher's buttons
        cur = face_settings(saved_face(), cur)
        ctl = control_socket()
    scale = SIZES[cur["size"]] if cur["size"] and a.scale == 1.0 else a.scale
    win = pygame.display.set_mode((0, 0) if a.fullscreen else (int(W * scale), int(H * scale)), flags)
    pygame.display.set_caption("omni-robot face" + (" (demo)" if a.demo else ""))
    pygame.mouse.set_visible(not a.fullscreen)
    if cur["hidden"]:
        show_window(False)
    canvas = pygame.Surface((W, H))
    voice = Voice(choose(a.voice))
    audio = Audio(voice, cur["mute"])
    view = FaceView()
    link = None if a.demo else Link(a.host, a.port)
    sup = None if a.demo else Supervisor(a.supervisor or f"http://{a.host}:8080/api/state")
    brain = Brain(BrainConfig(antics="lively", antic_first=6.0)) if a.demo else None
    ear = None if a.demo else choose_listener(a.listen)
    mic_on = True                                       # L: the microphone off here, whatever the node says
    grammar = None
    print(f"voice: {voice.name}; listening: {ear.name if ear else 'no'}; the face node at {a.host}:{a.port}"
          + (" (demo)" if a.demo else ""))
    clock = pygame.time.Clock()
    t0, t_shot = time.monotonic(), 0.0
    st = FaceState(connected=False, mood="sleepy")
    while True:
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q)):
                if ear is not None:
                    ear.stop()
                pygame.quit()
                return
            if e.type == pygame.KEYDOWN and e.key == pygame.K_m:
                audio.mute = cur["mute"] = not audio.mute
            if e.type == pygame.KEYDOWN and e.key == pygame.K_l:
                mic_on = not mic_on
        while ctl is not None:                          # the Launcher: mute, size, hide
            try:
                new = face_settings(json.loads(ctl.recv(512)), cur)
            except (BlockingIOError, OSError):
                break
            except ValueError:
                continue
            audio.mute = new["mute"]
            if new["size"] != cur["size"] and new["size"]:
                s = SIZES[new["size"]]
                win = pygame.display.set_mode((int(W * s), int(H * s)))
            if new["hidden"] != cur["hidden"]:
                show_window(not new["hidden"])
            cur = new
        dt = clock.tick(30) / 1000.0
        note = ""
        mic = None
        if brain is not None:                           # the demo drives the same brain the robot runs
            status, pose, vel = demo_status(time.monotonic() - t0)
            st, outs = brain.update(time.monotonic(), status, pose, vel=vel)
            outs = [o.as_dict() for o in outs]
        else:
            outs = []
            while not link.outs.empty():
                outs.append(link.outs.get())
            if link.connected():
                st = FaceState(**{k: v for k, v in link.face.items() if k in FaceState.__dataclass_fields__})
            else:
                st = FaceState(connected=False, mood="sleepy")
                note = sup.text
            # the microphones: on while the node asks (and L hasn't switched them off here)
            want = ear is not None and ear.name != "none" and link.connected() and link.listen and mic_on
            if want and (not ear.running() or grammar != link.grammar):
                grammar = list(link.grammar)
                ear.start(grammar)
            elif not want and ear is not None and ear.running():
                ear.stop()
                grammar = None
            if ear is not None and ear.name != "none" and link.listen:
                mic = want
            for text, conf in ear.results() if ear is not None else ():
                link.send(dict(heard=text, conf=conf, echo=audio.busy()))
        for o in outs:
            if o.get("kind") == "sound":
                audio.play(o["value"])
            elif o.get("kind") == "say":
                audio.say(o["value"], int(o.get("prio", 1)), str(o.get("style") or ""))
        audio.tick()
        view.draw(canvas, st, dt, audio.level(), audio.caption_now(), note, mic)
        if canvas.get_size() == win.get_size():
            win.blit(canvas, (0, 0))
        else:
            win.blit(pygame.transform.smoothscale(canvas, win.get_size()), (0, 0))
        pygame.display.flip()
        if a.shot and time.monotonic() - t_shot > 2.0:
            t_shot = time.monotonic()
            pygame.image.save(canvas, a.shot)


if __name__ == "__main__":
    main()
