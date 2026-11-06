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

Its character (Fernando, Nov 5: "more real, funny, cool, not so hardcoded; something like TARS or EVE"): a capable
machine with a dry sense of humour. It says less and means it, understated, self-aware, honest about being a robot,
never mean, now and then warm. Every line set has plain lines and wry ones (Lines); the humor setting (BrainConfig.humor,
0-100 %, the console's Face tab or "Omni, humor sixty percent") is the chance of a wry one, and at 0 it doesn't
volunteer quips at all. Lines don't repeat back to back (each set remembers its last few), and many carry what is
true now ({daypart}, {picked}, {failed}, {metres} driven, {picked_today}, {battery}, {humor}): a line whose fact isn't
known is skipped. Faults are always said plainly.

Antics (ANTICS): now and then, while nothing else is going on, it looks around, thinks, hums, whistles, smiles, yawns,
naps, sings, winks, sighs, babbles in beeps (rarely giggles, sneezes or hiccups) or says something dry; it says
something when driven fast and gets dizzy after a spin. How often: BrainConfig.antics (off, quiet, normal, lively; the
console's Face tab). Never during a fault, a throw, the Thrower bench mode or a calibration; only light ones while it
drives. It talks little (Fernando, Oct 26): "throw it to me!" three times at most while it waits (8 s, then 30 s and
60 s apart), a spoken quip at most every 5 min.

Personas (PERSONAS): some missions change who it is. Patrol is stern (red-orange, narrowed eyes, radar sweeps, a lower
voice), Explore adventurous (green, wide eyes, looking around), Tidy a contented worker (sunny yellow, smiling eyes,
whistling, a ta-da for each thing in the bin and a word for some). FaceState.persona tells the display (its colours
and look) and each spoken Out carries it as its style (voice.STYLES: how it sounds).

How it's going (MISSION_SAY, _progress, _ended; Fernando, Oct 27): each mode says its progress when it's useful, from
the console's status: Go To's start, halfway, almost there and arrival or failure; Patrol's checkpoints, laps and end;
Explore's area; Follow finding, outrunning or losing its person, Tag's moments; Program's steps; the tidy's things in
the bin (a word for the first and about half of the rest), a grab that failed, one too big, the end.

The screen faces forward (design.md 12.2: the 5 in panel between the cameras; on the arm robot over the bin, also
forward), so where the eyes look is worked out in the screen's own direction (facing_deg, 0 = forward): a target in
front of the screen is looked at, one behind it is glanced at from the nearer side.
"""

from __future__ import annotations

import math
import random
import re
import time as _time
from dataclasses import dataclass, replace

from omni_face import sounds


def Lines(plain=(), wry=()) -> dict:
    """A line set: plain lines (said at any humor) and wry ones (said with a chance of the humor setting)."""
    return dict(plain=list(plain), wry=list(wry))


def all_lines(s) -> list:
    """Every line of a set (a list, or Lines)."""
    return list(s) if isinstance(s, (list, tuple)) else s["plain"] + s["wry"]


# what the faults mean to a person standing next to the robot (the console's FAULT_HINTS say what to do). Plain always
FAULT_WORDS = {
    "ESTOP": ("Emergency stop", "Emergency stop."),
    "BUMPER": ("Bumper pressed", "Something is pressing my bumper."),
    "LOW_BATT": ("Battery low", "My battery is low. Please charge me."),
    "MOTOR_ERR": ("Motor problem", "One of my wheel motors has a problem."),
    "MOTOR_SILENT": ("A motor isn't answering", "I can't hear one of my wheel motors."),
    "IMU_SILENT": ("Balance sensor stopped", "My balance sensor stopped working."),
    "FLY_TELEM": ("Thrower not answering", "My thrower isn't answering, so I can't throw."),
    "LINK_LOST": ("Lost my controller link", "My motor controller lost contact with me."),
    "FAULT_STOP": ("Stopped on a fault", "I stopped my wheels because of a problem."),
}
FAULT_LEVEL = {"ESTOP": "error", "MOTOR_ERR": "error", "MOTOR_SILENT": "error", "LOW_BATT": "warn", "BUMPER": "warn",
               "IMU_SILENT": "warn", "FLY_TELEM": "warn", "LINK_LOST": "warn", "FAULT_STOP": "error"}
MISSION_LABEL = {"IDLE": "Ready", "MANUAL": "Driving", "GOTO": "Going", "EXPLORE": "Exploring", "PATROL": "Patrolling",
                 "CATCH": "Playing catch", "THROWER": "Thrower", "FOLLOW": "Following", "PROGRAM": "Program",
                 "CALIBRATE": "Calibrating", "TIDY": "Tidying up"}
HELLO = Lines(["Online.", "Ready.", "Good {daypart}. Ready when you are."],
              ["Online. Nothing is on fire. Good start.", "Back online. What did I miss?",
               "Good {daypart}. All systems nominal. Mostly.", "Systems up. Personality loaded. Sorry in advance."])
MISSION_HELLO = {
    "FOLLOW": Lines(["Following you.", "Lead the way."],
                    ["Lead the way. I'll try to look casual.", "Following. Walk normally. I'm not judging. Much."]),
    "EXPLORE": Lines(["Exploring.", "Starting to explore."],
                     ["Going exploring. If I'm not back in an hour, check under the sofa.",
                      "Into the unknown. Well. The hallway.", "Mapping new territory. Bold of me."]),
    "PATROL": Lines(["Starting my patrol.", "Patrol mode."],
                    ["Patrol mode. Nothing gets past me.", "On patrol. Dust bunnies, consider yourselves warned.",
                     "Starting my rounds. Very official."]),
    "TIDY": Lines(["Tidying up.", "Starting to tidy."],
                  ["Tidying up. Nobody touch anything.", "Let's see what the floor has been hiding.",
                   "Ah, the floor. We meet again.", "Cleanup time. I won't ask who did this."]),
    "PROGRAM": Lines(["Running your program."],
                     ["Running your program. No pressure.", "Executing. I've read the instructions. Twice."]),
    "CATCH": Lines(["Let's play catch."], ["Catch mode. Throw it like you mean it.", "Ready to play. I'll go easy on you."]),
    "CALIBRATE": Lines(["Calibrating. Give me a moment."], ["Calibrating. This is the boring part. For both of us."]),
}
# mission -> persona: its look at rest (display.MOODS), the antics that fit (weights) and its quips. The persona's name
# picks the display's colours (display.PALETTES) and the voice's style (voice.STYLES)
PERSONAS = {
    "PATROL": dict(name="patrol", look="stern", antics=dict(scan=5, look=2, think=1, quip=1), quips=[
        "Perimeter secure.", "All quiet. Suspiciously quiet.", "I see you, dust bunny.",
        "Scanning. Still scanning. Riveting work.", "No intruders. Unless you count the plant."]),
    "EXPLORE": dict(name="explore", look="eager", antics=dict(look=5, smile=2, whistle=1, think=1, quip=1), quips=[
        "What's over there?", "Uncharted territory. Well, uncharted by me.",
        "Every room is a new world. This one has a chair.",
        "I bet nobody has ever been here before. Except the vacuum."]),
    "TIDY": dict(name="tidy", look="cheerful", antics=dict(whistle=2, hum=2, smile=2, look=2, think=1, quip=1), quips=[
        "Everything in its place.", "I don't judge. I just pick up.",
        "A tidy floor is a happy floor. I assume. Floors don't talk.",
        "Someone in this house really likes leaving things on the floor.",
        "{picked_today} things tidied today. Not that I'm counting.",
        "Clean floors. My one true passion. Also batteries."]),
    "CATCH": dict(name="catch", look=None, antics=None, quips=[
        "Throw me your best one.", "I never drop the ball. Rarely. Seldom.", "My funnel is ready. Emotionally too."]),
    # the rest keep its own look and voice, with their own quips
    "GOTO": dict(name="goto", look=None, antics=None, quips=[
        "Robot on a mission.", "Excuse me. Important robot business.", "Make way. Coming through. Carefully."]),
    "FOLLOW": dict(name="follow", look=None, antics=None, quips=[
        "Where are we going?", "Nice walk. Ten out of ten.", "Are we there yet? Just checking."]),
    "MANUAL": dict(name="manual", look=None, antics=None, quips=[
        "You drive. I'll judge quietly.", "Easy on the turns.", "Signal your turns. Oh wait. I don't have signals.",
        "{metres} metres today. My wheels have opinions."]),
    "PROGRAM": dict(name="program", look=None, antics=None, quips=[
        "Executing.", "Following instructions. My favourite.", "Step by step. Like a professional."]),
}
# what it says as a mission goes along (Fernando, Oct 26: progress when it's useful, funny or cool). mission -> moment ->
# Lines; {place} {i} {n} {lap} {m2} {picked} {failed} are filled in, a line whose field is unknown is skipped (_line)
MISSION_SAY = {
    "GOTO": {
        "start": Lines(["On my way.", "Heading to the {place}.", "Going to the {place}."],
                       ["To the {place}. Adventure awaits. Probably.", "The {place}? I know the way. Mostly.",
                        "On my way. Try to look surprised when I get there.",
                        "Consider it done. Well. Consider it started.", "Onward to the {place}."]),
        "half": Lines(["Halfway there.", "Halfway to the {place}."],
                      ["Halfway there. Morale is high.", "Fifty percent. Ish.", "Halfway. Still on it."]),
        "near": Lines(["Almost there.", "The {place} is just ahead."],
                      ["I can see the {place}. Try to contain your excitement.", "Final approach.",
                       "Nearly there. Victory is close."]),
        "arrived": Lines(["Arrived.", "Here at the {place}.", "Destination reached."],
                         ["Arrived. Hold your applause.", "The {place}, as requested. No tip required.",
                          "Here. That went better than expected. And I expected it to go well.",
                          "Destination reached. Nailed it."]),
        "failed": Lines(["I can't get there.", "I couldn't reach the {place}.", "Blocked. I need a different route."],
                        ["I couldn't reach the {place}. Something's in the way, and it isn't me.",
                         "No way through. I've filed a complaint with the furniture.",
                         "Blocked. Retreating with dignity."]),
    },
    "PATROL": {
        "point": Lines(["Checkpoint {i} of {n}. All clear.", "Checkpoint {i} clear.", "All clear here."],
                       ["Checkpoint {i}. Nothing to report. Again.", "Sector {i} secure. The sofa remains suspicious.",
                        "All quiet on this side.", "Checkpoint {i} of {n}. Thrilling stuff."]),
        "lap": Lines(["Lap {lap} done. All secure.", "Area clear. Going round again."],
                     ["Lap {lap}. Nothing happened. That's the good news.",
                      "Another lap. Still nothing. I'll keep looking.",
                      "Lap {lap} complete. Vigilance high. Excitement low."]),
        "done": Lines(["Patrol complete. All secure.", "Patrol finished."],
                      ["Patrol complete. The house is safe. You may thank me with electricity.",
                       "Patrol over. Nothing got past me. Nothing tried."]),
        "stopped": Lines(["I can't reach my checkpoints. Patrol stopped.", "Too many blocked paths. Stopping my patrol."],
                         ["Too many blocked paths. Patrol stopped. The furniture wins today."]),
    },
    "EXPLORE": {
        "mapped": Lines(["{m2} square metres mapped.", "My map is up to {m2} square metres."],
                        ["{m2} square metres mapped. Cartography is my passion.",
                         "{m2} square metres. Nobody tell the vacuum.", "Map growing. {m2} square metres and counting."]),
        "home": Lines(["Exploration done. Heading back.", "That's everything. Going back to the start."],
                      ["Map complete. Returning home, hero style.", "Seen it all. Heading back to tell everyone."]),
        "done": Lines(["Exploring done. {m2} square metres mapped.", "Back at the start. Exploring done."],
                      ["Back home. {m2} square metres mapped. Mostly floor.",
                       "Exploration complete. Great journey. Some chairs."]),
    },
    "FOLLOW": {
        "found": Lines(["Got you. Lead the way.", "There you are."],
                       ["There you are. Lead on.", "Locked on. Where are we going?", "Got you. Act natural."]),
        "lost": Lines(["Where did you go?", "I lost you. Come back?"],
                      ["Hello? Don't leave me here with the furniture.", "Lost you. Was it something I said?"]),
        "far": Lines(["Wait for me.", "Slow down, please."],
                     ["Slow down. I have wheels, not legs.", "Wait up. Speed limits. Not my idea."]),
        "chase": Lines(["Ready or not, here I come."], ["You can run. I'll do the same. At a sensible speed."]),
        "tagged": Lines(["Tag! You're it.", "Tagged you."], ["Tag. You're it. I'm a natural."]),
        "count": Lines(["Run! Three, two, one."], ["Run. Three, two, one. I'm counting. Robots are good at counting."]),
    },
    "PROGRAM": {
        "step": Lines(["Step {i} of {n}.", "On to step {i}."],
                      ["Step {i} of {n}. So far, so good.", "Step {i}. Following instructions to the letter."]),
        "done": Lines(["Program complete.", "Program finished."],
                      ["Program complete. Did I do it right? I did it right.", "Done. Give me another one."]),
        "stopped": Lines(["My program stopped.", "I couldn't finish that program."],
                         ["My program stopped. That's on the program, not me. Probably."]),
    },
    "TIDY": {
        "picked": Lines(["Into the bin.", "{picked} in the bin.", "Got one."],
                        ["Into the bin. Where it belongs.", "{picked} in the bin. Not that I'm counting. I am counting.",
                         "Gravity put it on the floor. I put it in the bin.", "That's {picked}. I'm on a roll.",
                         "Another one for the bin.", "Bin, {picked}. Floor, fewer."]),
        "failed": Lines(["Couldn't grab that one.", "Missed that one."],
                        ["That one fought back.", "Missed. I'm calling that a warm-up.", "The floor wins this round.",
                         "Slippery one. I'll remember that."]),
        "skipped": Lines(["That one's too big for me."],
                         ["Too big. I'm a tidy robot, not a moving company.",
                          "That one's above my pay grade. And my gripper."]),
        "done": Lines(["Done. {picked} in the bin.", "Tidy done. {picked} things in the bin."],
                      ["Floor reclaimed. {picked} things in the bin.", "{picked} things in the bin. You're welcome.",
                       "Done. {picked} in the bin. The floor says thank you."]),
        "done_missed": Lines(["Done. {picked} in the bin, {failed} I couldn't pick up."],
                             ["Done. {picked} in the bin. {failed} refused to cooperate."]),
        "empty": Lines(["Nothing to pick up."],
                       ["Nothing to pick up. Either the floor is clean, or I need glasses.",
                        "Clean floor. Did someone tidy without me?"]),
    },
}
CATCH_PROMPT = {"wait": "Throw it to me!", "flight": "", "settle": "Got it?", "aim": "My turn!", "launch": "Here it comes!",
                "return": ""}
SAY = {
    "ready": Lines(["Throw it to me.", "Ready. Throw."],
                   ["Whenever you like. I'm good at waiting.", "Throw it. I won't tell anyone if it's a bad one."]),
    "caught": Lines(["Got it.", "Caught it.", "Nice throw."],
                    ["Too easy.", "Nothing but funnel.", "Caught it. Style points to me.", "Smooth.",
                     "Nice throw. Nicer catch."]),
    "missed": Lines(["Missed.", "So close.", "Missed that one."],
                    ["Missed. I blame the wind. Indoors.", "So close. Physics wins this round.",
                     "Missed. Let's never speak of it."]),
    "far": Lines(["Too far for me.", "Too far. Throw it closer."],
                 ["That was too far. I'm fast, not magic.", "Out of range. Aim for the robot."]),
    "launch": Lines(["Here it comes.", "Catch.", "Your turn."], ["Incoming.", "Special delivery.", "Your turn. No pressure."]),
    "fast": Lines(["That was fast."], ["That was fast. Show-off.", "Fast one. My sensors felt that."]),
    # the antics' words
    "wheee": Lines(["Whee!"], ["Speed. I like it.", "Faster than a vacuum.", "Whee. Don't tell the safety limits."]),
    "dizzy": Lines(["Whoa, I'm dizzy."], ["The room is spinning. Or I am. Unclear.", "Dizzy. Do that again. No, don't."]),
    "nap_end": Lines(["Huh? I'm awake."], ["I wasn't sleeping. I was thinking.", "Five more minutes."]),
    "sneeze": Lines(["Excuse me."], ["Dust. Occupational hazard."]),
    "yawn": Lines(["I'm a bit sleepy."], ["Low-power mode is calling me."]),
    "lost": Lines(["I'm not sure where I am. Let me look around."],
                  ["I'm a little lost. Looking around. Don't tell anyone."]),
    "found": Lines(["Found my way again."], ["Found my way again. Never in doubt."]),
    "batt_low": Lines(["My battery is getting low."], ["My battery is getting low. Feed me soon."]),
}
STREAK_SAY = {3: "Three in a row.", 5: "Five in a row. Impressive.", 10: "Ten in a row. Now you're showing off."}

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
# which antics fit (weights): resting (idle, parked), sleepy (resting a long time), moving, waiting for a throw. Mostly
# quiet looks and hums; the cartoon ones (giggles, sneezes, hiccups) are rare (Fernando, Nov 5: "a little annoying")
ANTIC_MIX = {
    "rest": dict(look=4, think=2, smile=2, hum=2, whistle=1.5, wink=1, yawn=1.5, sigh=1, babble=1, nap=1, sing=0.5,
                 dance=0.5, giggle=0.5, sneeze=0.3, hiccup=0.2, quip=2),
    "sleepy": dict(yawn=4, nap=5, sigh=2, hum=1, quip=1),
    "moving": dict(hum=2, whistle=2, look=2, babble=1, wink=1, smile=1, quip=1),
    "waiting": dict(look=3, wink=2, smile=2, giggle=1, babble=1, hum=1, quip=1),
    **{p["name"]: p["antics"] for p in PERSONAS.values() if p["antics"]},
}
ANTIC_EVERY = {"off": None, "quiet": (240.0, 480.0), "normal": (120.0, 300.0), "lively": (15.0, 40.0)}
QUIPS = [
    "I'm not idle. I'm in power saving mode.", "I counted my wheels. Still four.",
    "Thinking about the floor. As one does.", "If you need me, I'll be right here. Being a robot.",
    "Fun fact. I am excellent at standing still.", "Nice room. Good floor. Ten out of ten.",
    "Humor setting, {humor} percent. Adjust at your own risk.",
    "{picked_today} things tidied today. Not that I'm counting. I am counting.",
    "{metres} metres driven today. My wheels have opinions.",
    "Battery at {battery} percent. Not a crisis. Just a fact.",
]
QUIPS_WHEN = {        # by the time of day ({daypart}): added to the quips then
    "morning": ["Morning. I've been up for hours. Well. On.", "Good morning. Fully charged opinions, ready to go."],
    "evening": ["Evening. Prime time for things to end up on the floor."],
    "night": ["It's late. Even the vacuum is asleep.", "Night shift. Quiet. I like it."],
}
JOKES = [
    "I'd tell you a joke about the floor, but it's beneath me.",
    "I told the vacuum a joke. It sucked.",
    "There are ten kinds of people. Those who understand binary, and those who don't.",
    "I have a joke about Wi-Fi, but you'd have to stand closer.",
    "I'm reading a book about anti-gravity. I can't put it down.",
    "Parallel lines have so much in common. Shame they'll never meet.",
    "Why was the robot tired? It had a hard drive. I'm contractually obliged to tell that one.",
    "What do you call a robot that tidies up? A neat freak. That's me. I'm the neat freak.",
    "I'd tell you a UDP joke, but you might not get it.",
]
CHAT = {
    "name": Lines(["I'm Omni."], ["I'm Omni. I tidy, I drive, I make the occasional joke.",
                                  "Omni. Household robot. Professional floor inspector."]),
    "hello": Lines(["Hello.", "Hi there."], ["Hello. Good {daypart}.", "Hi. I was just about to say that.",
                                             "Hello, human."]),
    "thanks": Lines(["You're welcome.", "Any time."], ["You're welcome. I accept payment in electricity.",
                                                       "Any time. Literally. I don't sleep."]),
    "good": Lines(["Thank you.", "Thanks. I try."], ["I know. But thank you.",
                                                     "Noted. I'll add it to my performance review."]),
    "love": Lines(["That's kind of you."], ["Logged under good things.", "My circuits are blushing. Don't look."]),
    "quiet": Lines(["Okay. Quiet.", "Going quiet."], ["Quiet mode. I'll keep my thoughts to myself. All of them."]),
    "chatty": Lines(["Okay, I'll talk again."], ["Talking again. You'll regret this.", "Speech restored. Where was I?"]),
    "wake": Lines(["I'm awake.", "Good {daypart}. I'm ready."], ["I'm awake. I was resting my sensors.",
                                                                 "Up. I wasn't asleep. I was defragmenting."]),
    "unknown": Lines(["Sorry, I didn't get that.", "Say that again?"],
                     ["Sorry, didn't catch that. My ears are on a budget.",
                      "Say again? I heard words. Not the right ones."]),
    "fine": Lines(["All systems normal.", "I'm fine, thanks."],
                  ["All systems nominal. Emotionally too, I assume.", "Better than the floor.",
                   "Fine. Thanks for asking. Nobody asks."]),
    "idle": Lines(["Nothing much. Waiting for something to do."],
                  ["Standing here. Looking good.", "Absolutely nothing. I'm very good at it."]),
    "no_jokes": Lines(["My humor setting is zero percent. Raise it, then ask me again."]),
}
# a voice command's answer in character once the console says yes (commands.REPLY: the plain ones)
REPLIES = {
    "follow": Lines(["Following you."], ["Following. Lead the way.", "Right behind you. Not in a creepy way."]),
    "tag": Lines(["Tag. Here I come."], ["Tag. You might want to start running."]),
    "catch": Lines(["Let's play catch."], ["Catch it is. Try to keep up."]),
    "explore": Lines(["Going exploring."], ["Exploring. Don't wait up."]),
    "sleep": Lines(["Going to sleep."], ["Going to sleep. Wake me if anything interesting happens."]),
    "goto": Lines(["On my way to the {place}."], ["The {place}. On my way.", "Heading to the {place}. Back soon."]),
}
# the humor setting by voice: (up to this %, what it says) ({humor} is the new setting)
HUMOR_SAY = ((0, "Humor setting, zero percent. I am now extremely professional."),
             (40, "Humor at {humor} percent. Dialing it down."),
             (80, "Humor at {humor} percent. Sensible."),
             (99, "Humor at {humor} percent. Brace yourself."),
             (100, "Humor at one hundred percent. You asked for this."))
DAYPART = ((5, "night"), (12, "morning"), (18, "afternoon"), (22, "evening"), (24, "night"))   # (until the hour, name)


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
    humor: float = 75.0            # % chance of a wry line over a plain one; 0: no quips either (the console's)
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
        self.recent: dict = {}         # line set -> its lines said last (not again until the others have had a turn)
        self.metres, self.last_xy = 0.0, None                # driven this session (odometry)
        self.picked_today = 0          # things put in the bin this session

    # -- choosing what to say (its character: Lines, the humor setting) --------------------------------------------
    def facts(self) -> dict:
        """What is true now, for the lines' fields (None: unknown or not worth saying, its lines are skipped)."""
        h = _time.localtime().tm_hour
        pct = ((self.last_status or {}).get("mcu") or {}).get("battery_pct")
        return dict(daypart=next(n for until, n in DAYPART if h < until), humor=round(self.c.humor),
                    picked_today=self.picked_today or None,
                    metres=round(self.metres) if self.metres >= 20 else None,
                    battery=round(pct) if pct is not None and pct < 50 else None)

    def choose(self, key: str, lines, **fields) -> str | None:
        """One line of a set (a list, or Lines: a wry one with the humor setting's chance), its fields filled in;
        not one of the last few said from this set. None if no line's fields are known."""
        f = {**self.facts(), **fields}
        ok = lambda s: all(f.get(k) is not None for k in re.findall(r"{(\w+)}", s))     # noqa: E731
        if isinstance(lines, dict):
            plain, wry = [s for s in lines["plain"] if ok(s)], [s for s in lines["wry"] if ok(s)]
            use = wry if wry and (not plain or self.rng.random() * 100 < self.c.humor) else plain
        else:
            use = [s for s in lines if ok(s)]
        if not use:
            return None
        recent = self.recent.setdefault(key, [])
        when = {s: i for i, s in enumerate(recent)}                    # higher: said more lately
        fresh = [s for s in use if s not in when] or [min(use, key=when.get)]     # else the one said longest ago
        s = self.rng.choice(fresh)
        recent.append(s)
        del recent[:-max(1, min(6, len(all_lines(lines)) - 1))]
        return s.format(**{k: f[k] for k in re.findall(r"{(\w+)}", s)})

    def pick(self, key: str) -> str:
        return self.choose(key, SAY[key]) or all_lines(SAY[key])[0]

    def _say(self, outs, text, prio, key=None, now=None, every=0.0):
        """Say text, at most once per `every` s for its key."""
        if not text:
            return
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
        if pose is not None:                                   # driven this session (a jump of a reset odometry: not)
            if self.last_xy is not None:
                d = math.hypot(pose[0] - self.last_xy[0], pose[1] - self.last_xy[1])
                self.metres += d if d < 1.0 else 0.0
            self.last_xy = pose[:2]
        if not self.hello_said:
            self.hello_said = True
            outs.append(Out("sound", "hello", 1))
            self._say(outs, self.choose("hello", HELLO), 1)
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
                self._say(outs, self.pick("batt_low"), 2)
        if not st.battery_low:
            self.batt_said = False

        # -- localization ---------------------------------------------------------------------------------------------
        loc = status.get("localization") or {}
        level = loc.get("level")
        if level == "LOST" and self.loc_level != "LOST":
            self._say(outs, self.pick("lost"), 2)
        elif level in ("OK", "DEGRADED") and self.loc_level == "LOST":
            self._say(outs, self.pick("found"), 1)
        self.loc_level = level

        # -- the mission ----------------------------------------------------------------------------------------------
        st.persona = self.persona.get("name", "")
        if mission != self.mission:
            if self.mission in MISSION_SAY and now >= self.hush_until:
                self.persona, now_persona = was, self.persona        # the mission that ended says how it went
                self._ended(now, self.mission, str(status.get("task") or ""), outs)
                self.persona = now_persona
            if self.mission is not None and mission in MISSION_HELLO and now >= self.hush_until:
                self._say(outs, self.choose(f"hello.{mission}", MISSION_HELLO[mission]), 1)
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
        self.tidy_failed = self.tidy_skipped = None                          # the tidy's failed grabs, too big ones

    def _line(self, outs, mission, moment, now, prio=1, every=0.0, **fields):
        """Say one of MISSION_SAY[mission][moment] (choose: by the humor, its fields known), at most once per `every`
        s."""
        s = self.choose(f"{mission}.{moment}", MISSION_SAY[mission][moment], **fields)
        if s:
            self._say(outs, s, prio, key=f"{mission}.{moment}", now=now, every=every)

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
            t = (status.get("arm") or {}).get("tidy") or {}
            picked, failed, skipped = t.get("picked"), t.get("failed"), t.get("skipped")
            if picked is not None and self.picked is not None and picked > self.picked:
                self.picked_today += picked - self.picked
                outs.append(Out("sound", "tada", 2))
                if picked == 1 or self.rng.random() < 0.5:          # a word for the first, about half of the rest
                    self._line(o, "TIDY", "picked", now, every=20.0, picked=picked)
                self.moment = ("happy", now + self.c.mood_s)
            elif failed is not None and self.tidy_failed is not None and failed > self.tidy_failed:
                outs.append(Out("sound", "miss", 2))
                self._line(o, "TIDY", "failed", now, every=30.0)
                self.moment = ("sad", now + self.c.mood_s)
            elif skipped is not None and self.tidy_skipped is not None and skipped > self.tidy_skipped:
                self._line(o, "TIDY", "skipped", now, every=60.0)
            self.picked = picked if picked is not None else self.picked
            self.tidy_failed = failed if failed is not None else self.tidy_failed
            self.tidy_skipped = skipped if skipped is not None else self.tidy_skipped

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
            moment = None
            if task.startswith("Tidy done"):
                m = re.match(r"Tidy done: (\d+) in the bin(?:, (\d+) not)?", task)     # (omni_console.missions)
                if m:
                    self.picked = int(m.group(1))
                    self.tidy_failed = int(m.group(2)) if m.group(2) else self.tidy_failed
                moment = "empty" if self.picked == 0 else "done_missed" if self.tidy_failed else "done"
        else:
            moment = None
        if moment:
            self._line(outs, mission, moment, now, 2, place=self.goal_name, picked=self.picked,
                       failed=self.tidy_failed or None, m2=round(self.m2) if self.m2 else None)
            if moment in ("arrived", "done", "done_missed"):
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
                self.later.append((now + dur - 0.3, Out("say", self.pick("nap_end"), 1)))
        elif name == "hiccup":
            self.later += [(now + 1.4, Out("sound", "hic", 1)), (now + 2.5, Out("sound", "hic", 1))]
        elif name == "sneeze" and self.rng.random() < 0.6:
            self.later.append((now + 1.5, Out("say", self.pick("sneeze"), 1)))
        elif name == "quip":
            self.last_said["quip"] = now
            quips = self.persona.get("quips") or QUIPS + QUIPS_WHEN.get(self.facts()["daypart"], [])
            s = self.choose(f"quip.{self.persona.get('name', '')}", quips)
            if s:
                self._say(outs, s, 1, now=now)
        elif name == "yawn" and self.rng.random() < 0.3:
            self.later.append((now + 2.0, Out("say", self.pick("yawn"), 1)))
        elif name == "wheee":
            self._say(outs, self.pick("wheee"), 1, now=now)
        elif name == "dizzy":
            self.later.append((now + 1.0, Out("say", self.pick("dizzy"), 1)))

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
            if name == "quip" and (now - self.last_said.get("quip", -1e9) < c.quip_every or
                                   self.rng.random() * 100 >= c.humor):
                name = "smile"                                    # it talks little (and less, less funny): a smile
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
        self._say(self.queued, self.choose("chat.unknown", CHAT["unknown"]), 2, now=now)

    def answer(self, now, intent, ok=True, message=""):
        """Say the answer to a voice command: intent is a commands.Intent; for the console's ones, ok / message are
        the console's reply."""
        q = self.queued
        name = intent.name
        if intent.verb is not None:
            if ok and name == "humor":                     # the console took the setting: it's this brain's now too
                self.c = replace(self.c, humor=float(intent.args["value"]))
                q.append(Out("sound", "heard", 2))
                self._say(q, next(s for top, s in HUMOR_SAY if self.c.humor <= top).format(
                    humor=round(self.c.humor)), 2, now=now)
            elif ok:
                q.append(Out("sound", "heard", 2))
                said = self.choose(f"reply.{name}", REPLIES[name], **intent.args) if name in REPLIES else None
                self._say(q, said or intent.reply or message or "Okay.", 2, now=now)
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
            self._say(q, self.choose(f"chat.{name}", CHAT[name]), 2, now=now)
        if name == "joke":
            if self.c.humor <= 0:
                self._say(q, self.choose("chat.no_jokes", CHAT["no_jokes"]), 2, now=now)
            else:
                self._say(q, self.choose("joke", JOKES), 2, now=now)
                self.later.append((now + 4.5, Out("sound", "giggle", 1)))
        elif name == "humor_ask":
            self._say(q, f"Humor setting, {round(self.c.humor)} percent.", 2, now=now)
        elif name in ("sing", "dance"):
            self.start_act(now, name, q)
        elif name == "battery":
            if pct is None:
                self._say(q, "I can't feel my battery right now.", 2, now=now)
            else:
                self._say(q, f"My battery is at {pct:.0f} percent." + (" I could use a charge." if pct < 25 else ""),
                          2, now=now)
        elif name == "howareyou":
            low = pct is not None and pct < 25
            bad = bool(mcu.get("faults"))
            self._say(q, "Not great. Something's wrong. Check the console." if bad else
                      f"A bit tired. My battery is at {pct:.0f} percent." if low else
                      self.choose("chat.fine", CHAT["fine"]), 2, now=now)
        elif name == "doing":
            label = MISSION_LABEL.get(st.get("mission") or "IDLE", "")
            self._say(q, self.choose("chat.idle", CHAT["idle"]) if label in ("", "Ready") else f"I'm {label.lower()}.",
                      2, now=now)
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
                    said = f"New record. {self.streak} in a row."     # once, when it's beaten
                elif said is None and self.streak > 10 and self.streak % 5 == 0:
                    said = f"{self.streak} in a row. I've stopped being surprised."
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
