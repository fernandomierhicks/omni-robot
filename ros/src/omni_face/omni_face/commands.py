"""Voice commands: what the robot understands when spoken to (ROS-free, standard library only).

Every command starts with the robot's name, the wake word ("omni", "hey omni", "robot", "hey robot"); the name alone
opens a 6 s window for a command without it ("Omni?" ... "follow me"). A command is either the console's (a verb of
/console/command, the same as a click on the page: the MCU's interlocks and the console's checks apply), or the
face's own (a joke, a song, the battery):

  stop, halt, freeze, wait                    stop                     the console: stop (IDLE, a ramped stop)
  follow me, come here, come with me          follow                   the console: follow (the lidar's legs)
  let's play tag                              tag                      the console: follow, mode tag
  let's play catch, play catch                catch                    the console: mission CATCH
  go to the <place>, go to <place>            goto                     the console: goto, a saved place of the map
  go home, go back home                       goto "home"              (a place called home, dock or start)
  explore, explore the house                  explore                  the console: explore
  go to sleep, take a nap, sleep              sleep                    the console: stop; the face naps
  wake up                                     wake
  be quiet, shh, quiet please                 quiet                    fewer antics, no chatter (the face)
  you can talk, talk to me                    chatty
  tell me a joke, sing a song, sing, dance, what's your battery, what's your name, how are you, hello, thank you,
  good robot, i love you, what are you doing, what time is it      the face answers
phrases(places) lists them for a recognizer's grammar (Windows' System.Speech, Vosk); parse() takes a grammar's
result or free text (a Whisper-style recognizer) the same way.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

WAKE = ("hey omni", "omni", "hey robot", "robot", "okay omni", "ok omni")
LISTEN_S = 6.0                      # after the name alone: how long a command without it is taken
MIN_CONF = 0.55                     # a grammar recognizer's confidence below this: not taken (a guess)


@dataclass
class Intent:
    name: str                       # stop follow tag catch goto explore sleep wake quiet chatty joke sing dance ...
    verb: str | None = None         # the console's verb, or None: the face's own
    args: dict = field(default_factory=dict)
    reply: str = ""                 # said once the console says yes ("" = the console's own message)
    act: str = ""                   # the face's antic that goes with it
    moves: bool = False             # it drives the robot (refused when voice commands are "chat only")


# (intent, phrases): a phrase is matched whole, after the wake word and fillers are stripped
PHRASES = (
    ("stop", ("stop", "halt", "freeze", "wait", "stop it", "stop now", "stop moving", "stand still")),
    ("follow", ("follow me", "come here", "come with me", "come on", "follow")),
    ("tag", ("let's play tag", "play tag", "tag")),
    ("catch", ("let's play catch", "play catch", "catch", "let's play ball", "play ball")),
    ("explore", ("explore", "explore the house", "go explore", "go exploring")),
    ("home", ("go home", "go back home", "go to your home", "go back")),
    ("sleep", ("go to sleep", "take a nap", "sleep", "nap time", "good night")),
    ("wake", ("wake up", "good morning", "rise and shine")),
    ("quiet", ("be quiet", "shh", "quiet", "quiet please", "hush", "shush")),
    ("chatty", ("you can talk", "talk to me", "you can talk again", "talk")),
    ("joke", ("tell me a joke", "tell a joke", "say something funny", "joke")),
    ("sing", ("sing a song", "sing", "sing for me", "sing something")),
    ("dance", ("dance", "do a dance", "let's dance", "can you dance")),
    ("battery", ("what's your battery", "how is your battery", "battery", "are you hungry", "how much battery")),
    ("name", ("what's your name", "who are you", "what is your name")),
    ("howareyou", ("how are you", "how are you doing", "how do you feel")),
    ("hello", ("hello", "hi", "hi there", "good afternoon", "good evening")),
    ("thanks", ("thank you", "thanks")),
    ("good", ("good robot", "good job", "well done", "nice job", "you're the best")),
    ("love", ("i love you", "you're cute", "you are cute")),
    ("doing", ("what are you doing", "what's up", "what is happening")),
    ("time", ("what time is it", "what's the time")),
)
HOME_PLACES = ("home", "dock", "start", "base")
REPLY = {"stop": "Stopping.", "follow": "Okay, I'll follow you!", "tag": "Tag! Here I come!",
         "catch": "Yay, let's play catch!", "explore": "Off I go exploring!", "sleep": "Goodnight! Zzz.",
         "goto": "On my way to the {place}!"}
_FILL = re.compile(r"\b(please|now|okay|ok|um|uh|hey|so|can you|could you|would you)\b")


def _clean(text: str) -> str:
    t = text.lower().replace("’", "'")
    t = re.sub(r"[^a-z0-9' ]+", " ", t)
    return re.sub(r"\s+", " ", t).strip()


def strip_wake(text: str) -> tuple:
    """(woke, the rest): the robot's name at the start is taken off."""
    t = _clean(text)
    for w in sorted(WAKE, key=len, reverse=True):
        if t == w or t.startswith(w + " "):
            return True, t[len(w):].strip()
    return False, t


def phrases(places=()) -> list:
    """Every command, with and without "the" before a place: the grammar a recognizer listens for (the wake word
    goes in front, see listen.py)."""
    out = [p for _, ps in PHRASES for p in ps]
    for name in places:
        n = _clean(name)
        if n:
            out += [f"go to the {n}", f"go to {n}", f"take me to the {n}"]
    return sorted(set(out))


def parse(text: str, places=(), woke: bool | None = None) -> Intent | None:
    """The command in text (wake word already stripped when woke is given), or None. places: the map's saved
    places (name -> anything)."""
    t = _clean(text) if woke is not None else strip_wake(text)[1]
    t = re.sub(r"\s+", " ", _FILL.sub(" ", t)).strip()
    if not t:
        return None
    by = {_clean(n): n for n in places}
    for name, ps in PHRASES:                    # whole phrases first ("go to sleep" is no place)
        if t in ps:
            return _intent(name, by)
    m = re.match(r"^(?:go|drive|take me|head|come) (?:over )?(?:to|into) (?:the |my |your )?(.+)$", t)
    if m:
        want = m.group(1).strip()
        name = by.get(want) or next((by[k] for k in by if want in k or k in want), None)
        if name is None:
            return Intent("unknown_place", None, dict(place=want), f"I don't know where the {want} is.")
        return Intent("goto", "goto", dict(place=name), REPLY["goto"].format(place=name), moves=True)
    # free text: a phrase inside a longer sentence ("robot, can you please follow me now")
    for name, ps in PHRASES:
        for p in sorted(ps, key=len, reverse=True):
            if len(p) > 3 and re.search(rf"\b{re.escape(p)}\b", t):
                return _intent(name, by)
    return None


def decide(text: str, conf: float, echo: bool, listening: bool, places=(), level: str = "on") -> tuple:
    """What to do with something heard -> (action, rest, intent):
      ignore    not meant for it (no name, not listening), its own voice (echo: only "stop" is taken), voice off
      wake      the name alone: listen for a command
      unknown   the name, then something it doesn't know (or too unsure)
      refused   a command that drives while voice commands are "chat only"
      chat      the face answers (intent.verb is None)
      console   send intent.verb to the console
    level: the console's face.voice (off, chat only, on)."""
    if level == "off":
        return "ignore", "", None
    woke, rest = strip_wake(text)
    if not (woke or listening):
        return "ignore", rest, None
    if conf < MIN_CONF:
        return ("unknown" if woke and not echo else "ignore"), rest, None
    if woke and not rest:
        return ("ignore" if echo else "wake"), rest, None
    intent = parse(rest, places, woke=True)
    if echo and (intent is None or intent.name != "stop"):
        return "ignore", rest, intent
    if intent is None:
        return "unknown", rest, None
    if intent.verb is None:
        return "chat", rest, intent
    if level == "chat only" and intent.moves:
        return "refused", rest, intent
    return "console", rest, intent


def _intent(name: str, by: dict) -> Intent:
    if name == "stop":
        return Intent("stop", "stop", {}, REPLY["stop"])
    if name == "follow":
        return Intent("follow", "follow", dict(mode="follow"), REPLY["follow"], "", True)
    if name == "tag":
        return Intent("tag", "follow", dict(mode="tag"), REPLY["tag"], "giggle", True)
    if name == "catch":
        return Intent("catch", "mission", dict(name="CATCH"), REPLY["catch"], "smile", True)
    if name == "explore":
        return Intent("explore", "explore", {}, REPLY["explore"], "", True)
    if name == "home":
        place = next((by[k] for k in HOME_PLACES if k in by), None)
        if place is None:
            return Intent("unknown_place", None, dict(place="home"), "I don't have a home on this map yet.")
        return Intent("goto", "goto", dict(place=place), "Going home!", moves=True)
    if name == "sleep":
        return Intent("sleep", "stop", {}, REPLY["sleep"], "nap")
    return Intent(name)
