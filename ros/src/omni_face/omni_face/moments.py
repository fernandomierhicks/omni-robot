"""What the face shows and the voice says, from the robot's status (ROS-free; face_node feeds it, the tests too).

Input, every update (10 Hz): the operator console's status (/console/status: the MCU's mode, faults and battery,
the mission and task, the catch game's counts and last throw, the followed person, the localization's health), the
robot's odometry pose (for where the ball is) and speed, the console's events, text someone asked it to say
(/face/say: plain text, or JSON with an expression: the console's Say box), and what it heard (the voice commands:
on_wake, answer, on_unknown).

Output: a face state (mood, where the eyes look, a prompt, the score, a fault card, the battery, an antic) that the
display draws as it likes, and things to say / play, each with a priority:

  3  safety     the e-stop, a fault: interrupts anything, never dropped
  2  game       a catch, a miss, a streak, the robot's turn to throw; the operator's Say box; voice answers
  1  chatter    "throw it to me!", mission greetings, antics' quips, /face/say as plain text
Not shown: engineering data (it can't be read on a moving robot); the console page has that.

Antics (ANTICS): now and then, while nothing else is going on, it yawns, naps, sings, hums, whistles, giggles,
winks, sneezes, hiccups, babbles in beeps, looks around or says something silly; it says "wheee!" when driven fast
and gets dizzy after a spin. How often: BrainConfig.antics (off, quiet, normal, lively; the console's Face tab). Never
during a fault, a throw, the Thrower bench mode or a calibration; only light ones while it drives. It talks little
(Fernando, Oct 26): "throw it to me!" three times at most while it waits (8 s, then 30 s and 60 s apart), a spoken
quip at most every 5 min.

Personas (PERSONAS): some missions change who it is. Patrol is stern (red-orange, narrowed eyes, radar sweeps, a lower
voice), Explore adventurous (green, wide eyes, looking around), Tidy a joyful worker (sunny yellow, smiling eyes,
whistling, a cheer for each thing in the bin). FaceState.persona tells the display (its colours and look) and each
spoken Out carries it as its style (voice.STYLES: how it sounds).

How it's going (MISSION_SAY, _progress, _ended; Fernando, Oct 27): each mode says its progress when it's useful, from
the console's status: Go To's start, halfway, almost there and arrival or failure; Patrol's checkpoints, laps and end;
Explore's area; Follow finding, outrunning or losing its person, Tag's moments; Program's steps; the tidy's count.

The screen faces forward (design.md 12.2: the 5 in panel between the cameras; on the arm robot over the bin, also
forward), so where the eyes look is worked out in the screen's own direction (facing_deg, 0 = forward): a target in
front of the screen is looked at, one behind it is glanced at from the nearer side.
"""

from __future__ import annotations

import math
import random
import re
import time as _time
from dataclasses import dataclass

from omni_face import sounds

# what the faults mean to a person standing next to the robot (the console's FAULT_HINTS say what to do)
FAULT_WORDS = {
    "ESTOP": ("Emergency stop", "Emergency stop."),
    "BUMPER": ("Bumper pressed", "Ouch! Something is pressing my bumper."),
    "LOW_BATT": ("Battery low", "My battery is low. Please charge me."),
    "MOTOR_ERR": ("Motor problem", "One of my wheel motors has a problem."),
    "MOTOR_SILENT": ("A motor isn't answering", "I can't hear one of my wheel motors."),
    "IMU_SILENT": ("Balance sensor stopped", "My balance sensor stopped working."),
    "FLY_TELEM": ("Thrower not answering", "My thrower isn't answering, so I can't throw."),
    "LINK_LOST": ("Lost my controller link", "My motor controller lost contact with me."),
}
FAULT_LEVEL = {"ESTOP": "error", "MOTOR_ERR": "error", "MOTOR_SILENT": "error", "LOW_BATT": "warn", "BUMPER": "warn",
               "IMU_SILENT": "warn", "FLY_TELEM": "warn", "LINK_LOST": "warn"}
MISSION_LABEL = {"IDLE": "Ready", "MANUAL": "Driving", "GOTO": "Going", "EXPLORE": "Exploring", "PATROL": "Patrolling",
                 "CATCH": "Playing catch", "THROWER": "Thrower", "FOLLOW": "Following", "PROGRAM": "Program",
                 "CALIBRATE": "Calibrating", "TIDY": "Tidying up"}
MISSION_HELLO = {"FOLLOW": ["I'll follow you!", "Lead the way!"],
                 "EXPLORE": ["Adventure time!", "Let's see what's out there!", "Into the unknown!"],
                 "PATROL": ["Patrol mode. Nothing gets past me.", "On guard!", "Starting my patrol. Stay alert."],
                 "TIDY": ["Tidy time! Let's make it sparkle.", "Ooh, cleaning up! My favourite job."],
                 "PROGRAM": ["Running your program!"],
                 "CATCH": ["Let's play catch!", "Ready to play catch!"], "CALIBRATE": ["Calibrating. Give me a moment."]}
# mission -> persona: its look at rest (display.MOODS), the antics that fit (weights) and its quips. The persona's name
# picks the display's colours (display.PALETTES) and the voice's style (voice.STYLES)
PERSONAS = {
    "PATROL": dict(name="patrol", look="stern", antics=dict(scan=5, look=2, think=1, quip=1), quips=[
        "Perimeter secure.", "Nothing gets past me.", "I see you, dust bunny.", "All quiet. Too quiet.",
        "Scanning. Scanning. Still scanning."]),
    "EXPLORE": dict(name="explore", look="eager", antics=dict(look=5, smile=2, whistle=1, think=1, quip=1), quips=[
        "Ooh, what's over there?", "Uncharted territory!", "Adventure awaits!",
        "I bet nobody has ever been here before. Except the vacuum.", "Every room is a new world!"]),
    "TIDY": dict(name="tidy", look="cheerful", antics=dict(whistle=3, hum=3, smile=2, giggle=1, quip=1), quips=[
        "Tidy tidy tidy!", "Clean floors are happy floors.", "Whistle while you work!",
        "I love a tidy room.", "Everything in its place!"]),
    "CATCH": dict(name="catch", look=None, antics=None, quips=[
        "Bet you can't throw it higher!", "I never drop the ball. Well, almost never.",
        "My funnel is ready.", "Throw me your best one!"]),
    # the rest keep its own look and voice, with their own quips
    "GOTO": dict(name="goto", look=None, antics=None, quips=[
        "A robot on a mission!", "Make way! Robot coming through.", "I was born for this.",
        "Excuse me. Important robot business."]),
    "FOLLOW": dict(name="follow", look=None, antics=None, quips=[
        "Where are we going?", "I like this walk.", "Are we there yet?", "Following you is my favourite thing."]),
    "MANUAL": dict(name="manual", look=None, antics=None, quips=[
        "Vroom vroom!", "You drive, I'll do the beeping.", "Easy on the turns!", "Left? Right? Surprise me!"]),
    "PROGRAM": dict(name="program", look=None, antics=None, quips=[
        "Computing. Computing.", "I love following instructions.", "Beep. Executing."]),
}
# what it says as a mission goes along (Fernando, Oct 26: progress when it's useful, funny or cool). mission -> moment ->
# lines; {place} {i} {n} {lap} {m2} {picked} are filled in, a line whose field is unknown is skipped (_line)
MISSION_SAY = {
    "GOTO": {
        "start": ["Yes, my master!", "It shall be done!", "Onward!", "Only the brave go there.",
                  "Onward to the {place}!", "To the {place}! Adventure awaits.", "Consider it done.",
                  "The {place}? I know the way.", "Your wish is my command."],
        "half": ["Halfway there!", "Half done. Onward!", "Making good progress.", "Halfway to the {place}."],
        "near": ["Almost there!", "I can see the goal!", "Nearly there. Victory is close!",
                 "The {place} is in sight!"],
        "arrived": ["Arrived! Mission accomplished.", "Ta-da! Here I am.", "I have arrived. What next, my master?",
                    "Destination reached!", "Here I am at the {place}!", "The {place}, as you wished."],
        "failed": ["I can't get there. Something's in the way.", "Blocked! I need a different plan.",
                   "Hmm, no way through. Sorry!", "I couldn't reach the {place}. Too scary."],
    },
    "PATROL": {
        "point": ["Checkpoint {i} of {n}. All clear.", "Checkpoint {i} clear.", "Everything clear here.",
                  "Nothing suspicious.", "Sector {i} secure.", "All quiet on this side.", "Clear. Moving on."],
        "lap": ["Lap {lap} done. All secure.", "Area clear. Going round again.", "Another lap. Nothing to report.",
                "Lap {lap} complete. Still on guard."],
        "done": ["Patrol complete. The house is safe.", "Patrol finished. All secure.",
                 "Patrol over. Nothing got past me."],
        "stopped": ["I can't reach my checkpoints. Patrol stopped.", "Too many blocked paths. I'm stopping my patrol."],
    },
    "EXPLORE": {
        "mapped": ["{m2} square metres mapped!", "My map is growing: {m2} square metres.",
                   "Ooh, new territory! {m2} square metres so far.", "{m2} square metres discovered!"],
        "home": ["That's everything! Heading back to base.", "Exploration done. Back to where I started.",
                 "Map complete. Returning home, hero style."],
        "done": ["Exploring complete! I mapped {m2} square metres.", "Mission complete. What a journey!",
                 "Back home. That was a great adventure!"],
    },
    "FOLLOW": {
        "found": ["Got you! Lead the way.", "There you are! Let's go.", "Locked on. Where are we going?",
                  "Okay, you're the leader!"],
        "lost": ["Where are you going? Don't leave me!", "Hey, wait for me!", "Where did you go?",
                 "Don't leave me behind!", "Hello? Come back!"],
        "far": ["Wait for me!", "Slow down, my legs are wheels!", "You're too fast for me!", "Hey, not so fast!"],
        "chase": ["Ready or not, here I come!", "I'm gonna get you!", "You can't escape me!"],
        "tagged": ["Tag! You're it!", "Got you! Tag!", "Tagged you!"],
        "count": ["Run! Three, two, one!", "Run away! I'm counting!"],
    },
    "PROGRAM": {
        "step": ["Step {i} of {n}!", "Next: step {i}.", "Step {i}. Beep boop.", "On to step {i} of {n}."],
        "done": ["Program complete! Did I do it right?", "Ta-da! All done.", "Program finished. Give me another one!"],
        "stopped": ["Oops, my program stopped.", "Hmm, I couldn't finish that program."],
    },
    "TIDY": {
        "picked": ["Into the bin!", "Got one!", "Tidy!", "One more for the bin!", "Yay, a clean spot!",
                   "That's {picked}!", "{picked} in the bin and counting!"],
        "done": ["All clean! {picked} things in the bin.", "Tidy time is over. Sparkly!",
                 "Done! The floor says thank you."],
    },
}
CATCH_PROMPT = {"wait": "Throw it to me!", "flight": "", "settle": "Got it?", "aim": "My turn!", "launch": "Here it comes!",
                "return": ""}
SAY = {
    "ready": ["Throw it to me!", "I'm ready. Throw!", "Toss it over!"],
    "caught": ["Got it!", "Nice throw!", "Caught it!", "Yes!", "Great throw!", "Too easy!", "Nothing but net!",
               "Smooth!"],
    "missed": ["Oops!", "So close!", "Missed that one. Try again!", "Almost!", "Butter wheels!", "I'll get the next one!"],
    "far": ["That one was too far for me!", "Too far! Throw it closer.", "I'm fast, but not that fast!"],
    "launch": ["Here it comes!", "Catch!", "Your turn!", "Incoming!", "Special delivery!"],
    "fast": ["Wow, that was fast!", "Super fast throw!"],
}
STREAK_SAY = {3: "Three in a row!", 5: "Five in a row! You're great at this!", 10: "Ten in a row! Amazing!"}

# antics: name -> (seconds, sound at the start, mood while it lasts). The display draws each (display.py)
ANTICS = {
    "yawn": (2.6, "yawn", "sleepy"), "nap": (9.0, None, "sleepy"), "sing": (5.0, None, "happy"),
    "hum": (3.2, "hum", "happy"), "whistle": (1.8, "whistle", None), "smile": (2.5, None, "happy"),
    "giggle": (1.6, "giggle", "happy"), "wink": (1.2, "sparkle", None), "look": (3.2, None, "surprised"),
    "sneeze": (2.4, "sneeze", None), "hiccup": (3.2, "hic", "surprised"), "babble": (1.8, "babble", None),
    "sigh": (2.2, "sigh", "sleepy"), "love": (3.0, "aww", "happy"), "dizzy": (3.2, "woozy", None),
    "wheee": (2.2, None, "surprised"), "dance": (5.0, None, "happy"), "listen": (6.0, "listen", "surprised"),
    "think": (1.6, None, "focused"), "quip": (3.0, None, None), "scan": (2.8, "scan", "stern"),
}
# which antics fit (weights): resting (idle, parked), sleepy (resting a long time), moving, waiting for a throw
ANTIC_MIX = {
    "rest": dict(yawn=2, nap=1, sing=2, hum=2, whistle=2, smile=2, giggle=2, wink=2, look=3, sneeze=1, hiccup=1,
                 babble=3, sigh=1, quip=1, dance=1),
    "sleepy": dict(yawn=4, nap=5, sigh=2, hum=1, quip=1),
    "moving": dict(hum=2, whistle=2, babble=2, look=1, wink=1, smile=1, quip=1),
    "waiting": dict(giggle=2, wink=2, look=2, babble=2, smile=2, hum=1, quip=1),
    **{p["name"]: p["antics"] for p in PERSONAS.values() if p["antics"]},
}
ANTIC_EVERY = {"off": None, "quiet": (240.0, 480.0), "normal": (90.0, 240.0), "lively": (15.0, 40.0)}
QUIPS = [
    "Beep boop. That's robot for hello.", "I counted my wheels. Still four!", "Is it time to play catch yet?",
    "I wonder what's under the sofa.", "Do you think balls get dizzy?", "I'm not lazy. I'm in power saving mode.",
    "My favourite colour is blinky.", "Somebody throw me something!", "I just learned a new beep. Beep!",
    "If you need me, I'll be right here. Being awesome.", "I like this room. It has a floor.",
]
JOKES = [
    "Why did the robot go on holiday? It needed to recharge its batteries!",
    "What do you call a robot that takes the long way round? R2 detour!",
    "Why was the robot so tired? It had a hard drive!",
    "What is a robot's favourite snack? Computer chips!",
    "Why did the ball stop rolling? It was tired of going round in circles!",
    "What do robots do at a party? They bust a few moves... in binary!",
    "Knock knock. Who's there? Robot. Robot who? Ro-bot you a present, but I ate it. It was a battery.",
]
CHAT = {
    "name": ["I'm Omni! I love playing catch.", "My name is Omni. Nice to meet you!"],
    "hello": ["Hello!", "Hi there!", "Hello, friend!"],
    "thanks": ["You're welcome!", "Any time!"],
    "good": ["Thank you! You're nice too.", "Yay! Beep boop!"],
    "love": ["Aww, I love you too!", "You're my favourite human!"],
    "quiet": ["Okay. I'll be quiet.", "Shh. Quiet as a mouse."],
    "chatty": ["Yay! I'll keep you company.", "Okay, chatterbox mode on!"],
    "wake": ["I'm awake! I'm awake!", "Good morning! I'm ready."],
    "unknown": ["Sorry, I didn't get that.", "Hmm? Say that again?", "I didn't understand. Try again?"],
}


@dataclass
class Out:
    kind: str                      # "say" | "sound"
    value: str
    prio: int = 1
    style: str = ""                # how it's said: the persona (voice.STYLES)

    def as_dict(self) -> dict:
        return dict(kind=self.kind, value=self.value, prio=self.prio, style=self.style)


@dataclass
class FaceState:
    mood: str = "idle"             # idle happy sad focused surprised sleepy worried alert
    gaze: list | None = None       # [x, y] in -1..1 (x right, y up, as the viewer sees the screen); None: look around
    prompt: str = ""               # big text at the bottom
    sub: str = ""                  # a smaller line under it
    card: dict | None = None       # a fault: {level, title, hint}
    score: dict | None = None      # {player, caught, throws, streak}
    battery: float | None = None   # %
    battery_low: bool = False
    label: str = ""                # what the robot is doing, a word or two (top left)
    toast: str = ""                # a short note for a few seconds
    kid: bool = False
    connected: bool = True         # the robot's software answers
    act: str = ""                  # an antic now (ANTICS: the display animates it), "" = none
    act_id: int = 0                # changes when one starts (the display restarts its animation)
    act_s: float = 0.0             # how long it lasts
    heard: str = ""                # what it just heard (a voice command), shown for a moment
    listening: bool = False        # it heard its name and waits for a command
    persona: str = ""              # who it is in this mission (PERSONAS: patrol, explore, tidy, catch), "" = itself

    def as_dict(self) -> dict:
        return dict(self.__dict__)


@dataclass
class BrainConfig:
    facing_deg: float = 0.0        # which way the screen looks, robot frame (forward: design.md 12.2)
    antics: str = "normal"         # off | quiet | normal | lively (ANTIC_EVERY)
    antic_first: float = 20.0      # s after start-up before the first
    wheee_speed: float = 1.3       # m/s for 0.5 s: "wheee!"
    dizzy_turns: float = 1.5       # turns at more than 2.5 rad/s, then slowing: dizzy
    ready_first: float = 8.0       # s after the game is ready before the first "throw it to me!"
    ready_every: float = 30.0      # s before the second, doubling after each
    ready_max: int = 3             # then only the screen asks, until the next throw
    quip_every: float = 300.0      # s at least between two spoken quips (antics)
    sleepy_after: float = 120.0    # s of nothing going on
    toast_s: float = 5.0
    mood_s: float = 2.5            # how long a happy / sad moment lasts
    fast_throw: float = 9.0        # m/s
    far_throw: float = 1.5         # m from the waiting spot


class Brain:
    def __init__(self, cfg: BrainConfig = BrainConfig(), seed: int = 0):
        self.c = cfg
        self.rng = random.Random(seed)
        self.faults_seen: set = set()
        self.mission = None
        self.tosses = self.catches = None
        self.last_toss = None
        self.catch_state = None
        self.t_ready = None
        self.ready_n, self.t_ready_said = 0, -1e9   # "throw it to me!" said this wait, and when
        self.persona = {}              # PERSONAS[mission], {} = itself
        self._reset_progress()
        self.moment = None             # (mood, until)
        self.toast = ("", 0.0)
        self.t_active = 0.0
        self.streak = 0
        self.best = {}                 # player -> best streak this session
        self.run_best0 = 0
        self.loc_level = None
        self.mcu_was_silent = False
        self.batt_said = False
        self.hello_said = False
        self.last_said: dict = {}
        self.act = None                # (name, t0, seconds, id) of the antic now
        self.n_act = 0
        self.t_next_antic = None
        self.t_said = -1e9             # the last thing said (antics wait for a quiet moment)
        self.later: list = []          # (t, Out) said / played later (a nap's snores, a second hiccup)
        self.queued: list = []         # outs from the voice handlers, sent with the next update
        self.hush_until = -1e9         # "be quiet": no antics or chatter until then
        self.listen_until = -1e9       # it heard its name: a command without it is taken until then
        self.heard = ("", -1e9)
        self.fast_since = None
        self.spin = 0.0                # rad turned in the current fast spin
        self.last_status = None

    def pick(self, key: str) -> str:
        return self.rng.choice(SAY[key])

    def _say(self, outs, text, prio, key=None, now=None, every=0.0):
        """Say text, at most once per `every` s for its key."""
        k = key or text
        if every and now is not None and now - self.last_said.get(k, -1e9) < every:
            return
        if now is not None:
            self.last_said[k] = now
            self.t_said = now
        outs.append(Out("say", text, prio, "" if prio >= 3 else self.persona.get("name", "")))   # faults: plainly

    def update(self, now: float, status: dict | None, pose=None, events=(), say=(), vel=None) -> tuple:
        """-> (FaceState, [Out]). pose: the robot's (x, y, yaw) in odom (where the ball's track is); vel: its body
        twist (vx, vy, wz) (the driving antics). say: plain text (chatter), or dicts {text, mood, act} (the
        operator's Say box: said at game priority, the face showing that mood or antic while it speaks)."""
        c, outs, st = self.c, [], FaceState()
        for req in say:
            self._say_request(now, req, outs)
        outs += self.queued
        self.queued = []
        for t, o in [x for x in self.later if x[0] <= now]:
            outs.append(o)
        self.later = [x for x in self.later if x[0] > now]
        for ev in events:
            if ev.get("level") in ("warn", "error") and ev.get("text"):
                self.toast = (str(ev["text"])[:80], now + c.toast_s)
        if status is None:
            st.connected = False
            st.mood, st.prompt = "sleepy", "Waiting for my software"
            return st, outs
        self.last_status = status
        if not self.hello_said:
            self.hello_said = True
            outs.append(Out("sound", "hello", 1))
            self._say(outs, "Hi! I'm ready.", 1)
        mcu = status.get("mcu") or {}
        st.kid = mcu.get("profile") == "kid"
        st.battery, st.battery_low = mcu.get("battery_pct"), bool(mcu.get("battery_low"))
        mission = status.get("mission") or "IDLE"
        st.label = MISSION_LABEL.get(mission, mission.title())
        was = self.persona
        self.persona = PERSONAS.get(mission, {})            # who says this tick's lines (faults: plainly, _say)

        # -- faults first: a card, a sound, said once each time one appears -----------------------------------------
        names = [f["name"] for f in mcu.get("faults") or []]
        hints = {f["name"]: f.get("hint", "") for f in mcu.get("faults") or []}
        disarmed = mcu.get("mode") in ("DISARMED", "BOOT")
        shown = [n for n in names if not (n == "LINK_LOST" and disarmed)]       # idle with no commands: not news
        for n in shown:
            if n not in self.faults_seen:
                title, words = FAULT_WORDS.get(n, (n.replace("_", " ").title(), f"I have a problem: {n}."))
                outs.append(Out("sound", "alert" if FAULT_LEVEL.get(n) == "error" else "warn", 3))
                self._say(outs, words, 3)
        self.faults_seen = set(shown)
        silent = bool(mcu) and (mcu.get("age") or 0) > 1.5
        if silent and not self.mcu_was_silent:
            self._say(outs, "I can't reach my motor controller.", 3)
        self.mcu_was_silent = silent
        if silent:
            st.card = dict(level="error", title="Motor controller silent",
                           hint="The Teensy stopped sending its state: check its USB cable and power.")
        elif shown:
            worst = sorted(shown, key=lambda n: FAULT_LEVEL.get(n) != "error")[0]
            title = FAULT_WORDS.get(worst, (worst.replace("_", " ").title(), ""))[0]
            more = len(shown) - 1
            st.card = dict(level=FAULT_LEVEL.get(worst, "warn"), title=title + (f" (+{more} more)" if more else ""),
                           hint=hints.get(worst, ""))
        if st.battery_low and not self.batt_said:
            self.batt_said = True
            if "LOW_BATT" not in shown:
                self._say(outs, "My battery is getting low.", 2)
        if not st.battery_low:
            self.batt_said = False

        # -- localization ---------------------------------------------------------------------------------------------
        loc = status.get("localization") or {}
        level = loc.get("level")
        if level == "LOST" and self.loc_level != "LOST":
            self._say(outs, "I'm not sure where I am. Let me look around.", 2)
        elif level in ("OK", "DEGRADED") and self.loc_level == "LOST":
            self._say(outs, "Found my way again.", 1)
        self.loc_level = level

        # -- the mission ----------------------------------------------------------------------------------------------
        st.persona = self.persona.get("name", "")
        if mission != self.mission:
            if self.mission in MISSION_SAY and now >= self.hush_until:
                self.persona, now_persona = was, self.persona        # the mission that ended says how it went
                self._ended(now, self.mission, str(status.get("task") or ""), outs)
                self.persona = now_persona
            if self.mission is not None and mission in MISSION_HELLO and now >= self.hush_until:
                self._say(outs, self.rng.choice(MISSION_HELLO[mission]), 1)
            self.mission = mission
            self.t_ready, self.ready_n = now, 0
            self._reset_progress()
        if mission not in ("IDLE",) or status.get("manual", {}).get("active"):
            self.t_active = now
        st.prompt = ""
        if self.persona.get("look"):
            st.mood = self.persona["look"]
        if mission in ("GOTO", "EXPLORE", "PATROL", "PROGRAM", "CALIBRATE", "TIDY"):
            st.sub = str(status.get("task") or "")[:70]
        self._progress(now, status, mission, outs)

        # -- the catch game -----------------------------------------------------------------------------------------
        cat = status.get("catch")
        if cat is not None:
            self._catch(now, cat, status, pose, st, outs)
        elif mission == "FOLLOW":
            person = (status.get("follow") or {}).get("person")
            if person:
                st.gaze = self._gaze_bearing(person[1], 0.0, person[0])
                st.mood = "focused"
            st.prompt = (status.get("follow") or {}).get("text", "")[:40]

        # -- mood ------------------------------------------------------------------------------------------------------
        if self.moment and now < self.moment[1]:
            st.mood = self.moment[0]
        elif st.card is not None:
            st.mood = "alert" if st.card["level"] == "error" else "worried"
        elif level == "LOST":
            st.mood = "worried"
        elif st.mood == "idle" and now - self.t_active > c.sleepy_after:
            st.mood = "sleepy"
        if self.toast[1] > now:
            st.toast = self.toast[0]
        self._antics(now, status, st, vel, outs)
        if now - self.heard[1] < 3.0:
            st.heard = self.heard[0]
        st.listening = now < self.listen_until
        return st, outs

    # -- how a mission is going (MISSION_SAY) ----------------------------------------------------------------------
    def _reset_progress(self):
        self.goal_key, self.goal_name, self.goto_said = None, None, set()   # Go To: the goal, half / near said
        self.patrol_i = self.lap = None                                      # Patrol: the point and lap
        self.m2 = self.m2_step = self.explore_phase = None                   # Explore: m² mapped, per 25 m², phase
        self.follow_phase, self.far_since = None, None                       # Follow: its phase, the person far since
        self.prog_i = self.picked = None                                     # Program's step, the tidy's count

    def _line(self, outs, mission, moment, now, prio=1, every=0.0, **fields):
        """Say one of MISSION_SAY[mission][moment] (the ones whose fields are known), at most once per `every` s."""
        lines = [s for s in MISSION_SAY[mission][moment]
                 if all(fields.get(k) is not None for k in re.findall(r"{(\w+)}", s))]
        if lines:
            self._say(outs, self.rng.choice(lines).format(**fields), prio, key=f"{mission}.{moment}", now=now,
                      every=every)

    def _progress(self, now, status, mission, outs):
        """Lines as a mission goes along: Go To's start, halfway and almost there; Patrol's checkpoints and laps;
        Explore's area mapped (every 25 m²) and the way back; Follow finding, losing or outrunning its person and
        Tag's moments; Program's steps; the tidy's things in the bin. Hushed by "be quiet" (sounds still play)."""
        o = outs if now >= self.hush_until else []
        if mission == "GOTO":
            g = status.get("goal") or {}
            key = (g.get("x"), g.get("y"))
            if g and key != self.goal_key:                       # a new goal (a mission or a change of mind)
                self.goal_key, self.goal_name, self.goto_said = key, g.get("name"), set()
                self._line(o, "GOTO", "start", now, every=4.0, place=self.goal_name)
            m = re.search(r"([\d.]+) m left", str(status.get("task") or ""))
            left, prog = (float(m.group(1)) if m else None), status.get("progress")
            if left is not None and prog is not None:
                if left < 1.5 and prog > 0.5 and "near" not in self.goto_said:
                    self.goto_said |= {"half", "near"}
                    self._line(o, "GOTO", "near", now, place=self.goal_name)
                elif prog >= 0.5 and left > 2.5 and "half" not in self.goto_said:
                    self.goto_said.add("half")
                    self._line(o, "GOTO", "half", now, place=self.goal_name)
        elif mission == "PATROL":
            p = status.get("patrol") or {}
            i, n, lap = p.get("i"), p.get("n"), p.get("lap")
            if lap is not None and self.lap is not None and lap > self.lap:
                self._line(o, "PATROL", "lap", now, every=30.0, lap=self.lap)       # (laps count from 1)
            elif i is not None and self.patrol_i is not None and i != self.patrol_i:
                self._line(o, "PATROL", "point", now, every=40.0, i=self.patrol_i + 1, n=n)
            self.patrol_i = i if i is not None else self.patrol_i
            self.lap = lap if lap is not None else self.lap
        elif mission == "EXPLORE":
            e = status.get("explore") or {}
            m2 = (e.get("info") or {}).get("known_m2")
            if m2:
                self.m2, step = m2, int(m2 // 25)
                if self.m2_step is not None and step > self.m2_step:
                    self._line(o, "EXPLORE", "mapped", now, every=45.0, m2=step * 25)
                self.m2_step = step if self.m2_step is None else max(step, self.m2_step)
            if e.get("phase") == "home" and self.explore_phase not in (None, "home"):
                self._line(o, "EXPLORE", "home", now, 2, m2=round(self.m2) if self.m2 else None)
            self.explore_phase = e.get("phase") or self.explore_phase
        elif mission == "FOLLOW":
            f = status.get("follow") or {}
            ph, was, person = f.get("phase"), self.follow_phase, f.get("person")
            if ph is not None and was is not None and ph != was:
                moment = {"follow": "found", "look": "lost", "chase": "chase" if was in ("look", "count") else None,
                          "tagged": "tagged", "count": "count"}.get(ph)
                if moment:
                    self._line(o, "FOLLOW", moment, now, 2 if moment == "lost" else 1, every=6.0)
            self.follow_phase = ph or was
            far = ph == "follow" and person and person[0] > 2.6
            self.far_since = (self.far_since or now) if far else None
            if far and now - self.far_since > 1.5:
                self._line(o, "FOLLOW", "far", now, every=25.0)
        elif mission == "PROGRAM":
            pr = status.get("program") or {}
            i, n = pr.get("i"), pr.get("n")
            if i is not None and self.prog_i is not None and i > self.prog_i and n and n >= 3 and i < n:
                self._line(o, "PROGRAM", "step", now, every=20.0, i=i + 1, n=n)
            self.prog_i = i if i is not None else self.prog_i
        elif mission == "TIDY":
            picked = ((status.get("arm") or {}).get("tidy") or {}).get("picked")
            if picked is not None and self.picked is not None and picked > self.picked:
                outs.append(Out("sound", "tada", 2))
                self._line(o, "TIDY", "picked", now, every=15.0, picked=picked)
                self.moment = ("happy", now + self.c.mood_s)
            self.picked = picked if picked is not None else self.picked

    def _ended(self, now, mission, task, outs):
        """A mission just ended: say how (the console's task line says: omni_console.missions)."""
        if mission == "GOTO":
            moment = "arrived" if task.startswith("Arrived") else "failed" if task.startswith("Could not reach") else None
            if moment:
                outs.append(Out("sound", "tada" if moment == "arrived" else "miss", 2))
        elif mission == "PATROL":
            moment = "done" if task.endswith("finished") else "stopped" if task.startswith("Patrol stopped") else None
        elif mission == "EXPLORE":
            moment = "done" if task.startswith("Exploring done") else None
        elif mission == "PROGRAM":
            moment = "done" if task.endswith("finished") else "stopped" if " stopped: " in task else None
        elif mission == "TIDY":
            moment = "done" if task.startswith("Tidy done") else None
        else:
            moment = None
        if moment:
            self._line(outs, mission, moment, now, 2, place=self.goal_name, picked=self.picked,
                       m2=round(self.m2) if self.m2 else None)
            if moment in ("arrived", "done"):
                self.moment = ("happy", now + self.c.mood_s)

    # -- the operator's Say box (/face/say) --------------------------------------------------------------------
    def _say_request(self, now, req, outs):
        if not isinstance(req, dict):
            self._say(outs, str(req), 1, now=now)
            return
        text = str(req.get("text") or "").strip()[:300]
        mood, act = req.get("mood"), req.get("act")
        if text:
            self._say(outs, text, 2, now=now)
        dur = 0.6 + 0.07 * len(text)                       # about how long it takes to say (SAPI / Piper ~15 chars/s)
        if act in ANTICS:
            self.start_act(now, act, outs, max(dur, ANTICS[act][0]) if text and act not in ("sing", "dance") else None)
        if mood:
            self.moment = (str(mood), now + max(dur, 2.0))

    # -- antics ---------------------------------------------------------------------------------------------------
    def start_act(self, now, name, outs, seconds=None):
        """Start an antic now: its sound (a song picked for sing / dance), its later sounds and words."""
        dur, snd, _ = ANTICS[name]
        dur = seconds or dur
        self.n_act += 1
        if name in ("sing", "dance"):
            snd = self.rng.choice(("twinkle", "jacques", "ode", "jingle"))
            dur = max(dur, sounds.song_seconds(snd) + 0.6)
        self.act = (name, now, dur, self.n_act)
        if snd:
            outs.append(Out("sound", snd, 1))
        if name == "nap":
            self.later += [(now + 1.5, Out("sound", "snore", 1)), (now + 4.5, Out("sound", "snore", 1))]
            if self.rng.random() < 0.5:
                self.later.append((now + dur - 0.3, Out("say", self.rng.choice(
                    ["Huh? I'm awake!", "I wasn't sleeping. I was thinking.", "Five more minutes..."]), 1)))
        elif name == "hiccup":
            self.later += [(now + 1.4, Out("sound", "hic", 1)), (now + 2.5, Out("sound", "hic", 1))]
        elif name == "sneeze" and self.rng.random() < 0.6:
            self.later.append((now + 1.5, Out("say", self.rng.choice(["Excuse me!", "Bless me!"]), 1)))
        elif name == "quip":
            self.last_said["quip"] = now
            self._say(outs, self.rng.choice(self.persona.get("quips") or QUIPS), 1, now=now)
        elif name == "yawn" and self.rng.random() < 0.3:
            self.later.append((now + 2.0, Out("say", "I'm a bit sleepy.", 1)))
        elif name == "wheee":
            self._say(outs, self.rng.choice(["Wheee!", "Woohoo!", "Zoom zoom!"]), 1, now=now)
        elif name == "dizzy":
            self.later.append((now + 1.0, Out("say", self.rng.choice(["Whoa, I'm dizzy!", "The room is spinning!"]),
                                              1)))

    def _antics(self, now, status, st, vel, outs):
        """Pick an antic now and then, react to the driving, and show the current one on the face."""
        c = self.c
        mission = status.get("mission") or "IDLE"
        cat = status.get("catch") or {}
        busy = (st.card is not None or mission in ("THROWER", "CALIBRATE") or
                (cat and cat.get("state", "wait") != "wait") or (status.get("localization") or {}).get("level") == "LOST")
        if busy and self.act is not None and self.act[0] not in ("listen",):
            self.act = None                                       # a fault or a throw ends it at once
            self.later = [x for x in self.later if x[1].prio > 1]
        if self.act is not None and now > self.act[1] + self.act[2]:
            self.act = None
        every = ANTIC_EVERY.get(c.antics)
        hushed = now < self.hush_until
        # the driving: "wheee!" when fast, dizzy after a fast spin (any mission that drives, never in a throw)
        if vel is not None and every is not None and not busy:
            v, wz = math.hypot(vel[0], vel[1]), vel[2]
            self.fast_since = (self.fast_since or now) if v > c.wheee_speed else None
            if self.fast_since and now - self.fast_since > 0.5 and self.act is None and \
                    now - self.last_said.get("wheee", -1e9) > 45.0:
                self.last_said["wheee"] = now
                self.start_act(now, "wheee", outs)
            if abs(wz) > 2.5:
                self.spin += abs(wz) * 0.1                        # update() runs at 10 Hz
            elif abs(wz) < 0.5:
                if self.spin > c.dizzy_turns * 2 * math.pi and now - self.last_said.get("dizzy", -1e9) > 30.0:
                    self.last_said["dizzy"] = now
                    self.start_act(now, "dizzy", outs)
                self.spin = 0.0
        # now and then, when nothing else is going on
        if self.t_next_antic is None:
            self.t_next_antic = now + c.antic_first
        moving = vel is not None and (math.hypot(vel[0], vel[1]) > 0.05 or abs(vel[2]) > 0.2)
        if mission == "CATCH":
            waited = now - (self.t_ready if self.t_ready is not None else now)
            mix = "waiting" if not busy and waited > 8.0 and not cat.get("ball") else None
        elif self.persona.get("name") in ANTIC_MIX:
            mix = self.persona["name"]
        elif moving or mission in ("GOTO", "EXPLORE", "PATROL", "FOLLOW", "PROGRAM", "TIDY"):
            mix = "moving"
        else:
            mix = "sleepy" if st.mood == "sleepy" else "rest"
        awake = set(ANTIC_MIX["moving"]) | set(ANTIC_MIX.get(st.persona, ()))      # what it may do while driving
        if moving and self.act is not None and self.act[0] not in awake and \
                self.act[0] not in ("wheee", "dizzy", "listen") and now - self.act[1] > 0.5:
            self.act = None                                       # no napping at the wheel: it wakes as it moves
            self.later = [x for x in self.later if x[1].prio > 1]
        if every is None or busy or hushed or self.act is not None or mix is None or now - self.t_said < 4.0:
            if every is not None and self.t_next_antic is not None and now > self.t_next_antic:
                self.t_next_antic = now + 3.0                     # not now: try again in a moment
        elif now >= self.t_next_antic:
            names, w = zip(*ANTIC_MIX[mix].items())
            name = self.rng.choices(names, w)[0]
            if name == "quip" and now - self.last_said.get("quip", -1e9) < c.quip_every:
                name = "smile"                                    # it talks little: a smile instead
            self.start_act(now, name, outs)
            self.t_next_antic = now + self.rng.uniform(*every)
        if self.act is not None:
            name, t0, dur, n = self.act
            st.act, st.act_id, st.act_s = name, n, dur
            m = ANTICS[name][2]
            if m and st.card is None and not (self.moment and now < self.moment[1]):
                st.mood = m

    # -- what it heard (the voice commands: face_node, commands.py) ------------------------------------------------
    def on_wake(self, now):
        """It heard its name alone: listen for a command for a few seconds."""
        from omni_face.commands import LISTEN_S
        self.listen_until = now + LISTEN_S
        self.start_act(now, "listen", self.queued, LISTEN_S)

    def listening(self, now) -> bool:
        return now < self.listen_until

    def on_heard(self, now, text):
        self.heard = (text, now)
        self.listen_until = -1e9
        if self.act is not None and self.act[0] in ("listen", "nap"):
            self.act = None

    def on_unknown(self, now, text=""):
        self.on_heard(now, text)
        self.queued.append(Out("sound", "huh", 2))
        self._say(self.queued, self.rng.choice(CHAT["unknown"]), 2, now=now)

    def answer(self, now, intent, ok=True, message=""):
        """Say the answer to a voice command: intent is a commands.Intent; for the console's ones, ok / message are
        the console's reply."""
        q = self.queued
        name = intent.name
        if intent.verb is not None:
            if ok:
                q.append(Out("sound", "heard", 2))
                self._say(q, intent.reply or message or "Okay!", 2, now=now)
                if intent.act:
                    self.start_act(now, intent.act, q)
            else:
                q.append(Out("sound", "huh", 2))
                self._say(q, f"I can't do that now. {message}".strip(), 2, now=now)
            return
        st = self.last_status or {}
        mcu = st.get("mcu") or {}
        pct = mcu.get("battery_pct")
        if name in CHAT:
            self._say(q, self.rng.choice(CHAT[name]), 2, now=now)
        if name == "joke":
            self._say(q, self.rng.choice(JOKES), 2, now=now)
            self.later.append((now + 4.5, Out("sound", "giggle", 1)))
        elif name in ("sing", "dance"):
            self.start_act(now, name, q)
        elif name == "battery":
            if pct is None:
                self._say(q, "I can't feel my battery right now.", 2, now=now)
            else:
                self._say(q, f"My battery is at {pct:.0f} percent." + (" I'm getting hungry!" if pct < 25 else ""), 2,
                          now=now)
        elif name == "howareyou":
            low = pct is not None and pct < 25
            bad = bool(mcu.get("faults"))
            self._say(q, "Not so good. Something's wrong, look at the console." if bad else
                      "A bit tired. My battery is low." if low else
                      self.rng.choice(["I feel great!", "Super! Want to play?", "Happy as a robot can be!"]), 2, now=now)
        elif name == "doing":
            label = MISSION_LABEL.get(st.get("mission") or "IDLE", "")
            self._say(q, "Just hanging out." if label in ("", "Ready") else f"I'm {label.lower()}.", 2, now=now)
        elif name == "time":
            t = _time.localtime()
            self._say(q, f"It's {t.tm_hour % 12 or 12}:{t.tm_min:02d}.", 2, now=now)
        elif name == "quiet":
            self.hush_until = now + 600.0
            self.act = None
        elif name == "chatty":
            self.hush_until = -1e9
        elif name == "wake":
            self.act = None
            self.t_active = now
            self.start_act(now, "look", q)
        elif name in ("good", "love"):
            self.start_act(now, "love", q)
        elif name in ("hello", "thanks"):
            self.start_act(now, "smile", q)
        elif name == "unknown_place":
            q.append(Out("sound", "huh", 2))
            self._say(q, intent.reply, 2, now=now)

    def _catch(self, now, cat, status, pose, st, outs):
        c = self.c
        state = cat.get("state", "wait")
        tosses, catches = cat.get("tosses", 0), cat.get("catches", 0)
        last = cat.get("last") or {}
        player = status.get("player") or "Player 1"
        # a throw is decided when the game writes its record (omni_catch.game: tosses counts at the release, the
        # record with "caught" comes after the settle)
        decided = last.get("toss") is not None and last.get("toss") != self.last_toss and "caught" in last
        if decided and self.last_toss is None and self.tosses is None:
            decided = False                                           # an old record seen at start-up
        self.last_toss = last.get("toss", self.last_toss)
        if decided:
            caught = bool(last.get("caught"))
            if caught:
                self.streak += 1
                outs.append(Out("sound", "catch", 2))
                said = STREAK_SAY.get(self.streak)
                best = self.best.get(player, 0)
                if self.streak == 1:
                    self.run_best0 = best                     # the record this run has to beat
                if self.streak == self.run_best0 + 1 and self.run_best0 >= 3 and said is None:
                    said = f"New record: {self.streak} in a row!"     # once, when it's beaten
                elif said is None and self.streak > 10 and self.streak % 5 == 0:
                    said = f"{self.streak} in a row!"
                speed = last.get("speed")
                if said is None and speed and speed > c.fast_throw and self.rng.random() < 0.5:
                    said = self.pick("fast")
                self._say(outs, said or self.pick("caught"), 2)
                self.best[player] = max(best, self.streak)
                self.moment = ("happy", now + c.mood_s)
            else:
                self.streak = 0
                outs.append(Out("sound", "miss", 2))
                aim = last.get("aim")
                pred = last.get("predicted")
                if aim is None and pred:
                    aim = math.hypot(pred[0], pred[1])
                self._say(outs, self.pick("far") if aim and aim > c.far_throw else self.pick("missed"), 2)
                self.moment = ("sad", now + c.mood_s)
            self.t_ready, self.ready_n = now, 0
        self.tosses, self.catches = tosses, catches
        if state != self.catch_state:
            if state == "launch":
                outs.append(Out("sound", "launch", 2))
                self._say(outs, self.pick("launch"), 2, key="launch", now=now, every=4.0)
            if state == "flight":
                outs.append(Out("sound", "whoosh", 2))
            self.catch_state = state
            if state == "wait":
                self.t_ready, self.ready_n = now, 0
        # "throw it to me!": 8 s into the wait, then 30 s and 60 s later; after that the screen asks alone
        since, gap = ((self.t_ready, c.ready_first) if self.ready_n == 0 else
                      (self.t_ready_said, c.ready_every * 2 ** (self.ready_n - 1)))
        if state == "wait" and self.t_ready is not None and self.ready_n < c.ready_max and now - since > gap and \
                now >= self.hush_until:
            self.ready_n, self.t_ready_said = self.ready_n + 1, now
            self._say(outs, self.pick("ready"), 1, key="ready", now=now)
        st.prompt = CATCH_PROMPT.get(state, "")
        st.score = dict(player=player, caught=catches, throws=tosses, streak=self.streak)
        ball = cat.get("ball")
        if ball is not None and pose is not None:
            st.gaze = self._gaze_point(pose, ball)
            if state in ("flight", "wait"):
                st.mood = "focused" if state == "flight" else "surprised"
        if state == "flight":
            st.mood = "focused"
        self.t_active = now

    # -- where the eyes look ----------------------------------------------------------------------------------------
    def _gaze_point(self, pose, p):
        x, y, yaw = pose
        dx, dy = p[0] - x, p[1] - y
        bearing = math.atan2(dy, dx) - yaw
        dist = math.hypot(dx, dy)
        return self._gaze_bearing(bearing, (p[2] if len(p) > 2 else 0.5) - 0.5, dist)

    def _gaze_bearing(self, bearing, dz, dist):
        """bearing (rad, robot frame), height above the eyes (m), distance (m) -> [x, y] for the screen."""
        b = math.atan2(math.sin(bearing - math.radians(self.c.facing_deg)),
                       math.cos(bearing - math.radians(self.c.facing_deg)))
        # b > 0: to the screen's own left, which is the viewer's right (they face it): the pupils go right
        if abs(b) > math.pi / 2:                      # behind the screen: glance that way from the nearer side
            gx = 1.0 if b > 0 else -1.0
        else:
            gx = max(-1.0, min(1.0, b / math.radians(60)))
        gy = max(-1.0, min(1.0, math.atan2(dz, max(dist, 0.3)) / math.radians(40)))
        return [round(gx, 3), round(gy, 3)]
