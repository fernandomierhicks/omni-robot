"""Talking with the robot (ROS-free; Nov 21, Fernando: "LLM ok"): what it says when it's asked something it has no
line for, in character, from what is true now and what it remembers.

Asked by voice ("Omni, what do you think of cats?": the recognizer's dictation after its name, listen.py) or typed
(the Face tab's "Hear"), a sentence that isn't a command goes here (face_node -> the display -> voice_server's /chat:
it runs on the main PC, which has the network and the voices). Claude answers as the robot, from CHARACTER.md's
character, the robot's facts now (its feeling, mission, battery, the time, what just happened) and its memory, and
says how it feels about it: {say, emotion, intensity, remember}. The face says it with that feeling and the body
shows it (expression.py), as for anything else it feels.

Memory (MEMORY_PATH, JSON on the PC): the last few exchanges (a conversation goes on), and notes it chose to keep
("Ana likes the red blocks"), the newest MAX_NOTES. Nothing leaves the PC except the request to the Claude API.
The model: OMNI_CHAT_MODEL (default Claude Haiku 5.5, Nov 23, Fernando: fast replies; at low effort: replies are
short). Another model (OMNI_CHAT_MODEL=claude-opus-5-5) gets the API's server-side fallback on a refusal; Haiku has
none, so a refusal there is answered politely in character. Credentials: the Anthropic SDK's own (ANTHROPIC_API_KEY,
or an `ant auth login` profile). Without them, or offline, it says it can't think about that right now.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

from omni_face.affect import EMOTIONS

MODEL = os.environ.get("OMNI_CHAT_MODEL", "claude-haiku-5-5")
MEMORY_PATH = Path(os.environ.get("OMNI_CHAT_MEMORY",
                                  Path.home() / ".local" / "share" / "omni_voice" / "chat_memory.json"))
CHARACTER = Path(__file__).resolve().parents[1] / "CHARACTER.md"
MAX_TURNS = 8                    # exchanges kept as the conversation
MAX_NOTES = 40                   # things it chose to remember
TURN_GAP = 600.0                 # s: after this long a new conversation starts (the old one stays as memory)
OFFLINE = "I'd love to think about that, but my thinking module is offline right now."

SCHEMA = {
    "type": "object",
    "properties": {
        "say": {"type": "string", "description": "What the robot says aloud: one or two short sentences."},
        "emotion": {"type": "string", "enum": sorted(EMOTIONS)},
        "intensity": {"type": "number", "description": "0 to 1: how strongly it feels it."},
        "remember": {"type": "string", "description": "A short fact worth remembering about the people or the "
                                                      "house, or an empty string."},
    },
    "required": ["say", "emotion", "intensity", "remember"],
    "additionalProperties": False,
}

RULES = """You are the voice of the household robot described above, talking with a person who just spoke to you.
Answer as the robot, in its character: short (one or two sentences, said aloud, so no lists, markdown, emoji or
stage directions), understated, honest about being a machine. Use the facts below when they help; never invent
things you did or saw. You cannot move or act from this conversation: if someone asks you to do something, say how
to ask (for example "say: Omni, follow me") rather than pretending you did it. Pick the emotion you genuinely feel
about what they said, and how strongly. If they tell you something worth remembering about themselves or the house,
put it in "remember" in a few words; otherwise leave it empty."""


def character_text() -> str:
    try:
        return CHARACTER.read_text(encoding="utf-8")
    except OSError:
        return "Omni: a capable household robot with a dry sense of humour. Understated, honest, never mean."


class Memory:
    def __init__(self, path: Path = MEMORY_PATH):
        self.path = path
        self.d = dict(turns=[], notes=[], t_last=0.0)
        try:
            self.d.update(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, ValueError):
            pass

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.d, indent=1), encoding="utf-8")
        os.replace(tmp, self.path)

    def turns(self, now: float) -> list:
        """The conversation so far: [] if the last exchange was long ago."""
        return list(self.d["turns"]) if now - float(self.d.get("t_last", 0.0)) < TURN_GAP else []

    def add(self, now: float, heard: str, said: str, note: str = ""):
        t = self.turns(now) + [dict(heard=heard, said=said)]
        self.d["turns"], self.d["t_last"] = t[-MAX_TURNS:], now
        note = note.strip()
        if note and note not in self.d["notes"]:
            self.d["notes"] = (self.d["notes"] + [note])[-MAX_NOTES:]
        self.save()


def facts_text(ctx: dict) -> str:
    """The robot's facts now, as lines for the model (ctx: face_node's context)."""
    lines = []
    for k, label in (("feeling", "You feel"), ("mission", "You are doing"), ("battery", "Battery (%)"),
                     ("time", "Local time"), ("picked_today", "Things tidied today"), ("metres", "Metres driven "
                     "today"), ("recent", "What just happened")):
        v = ctx.get(k)
        if v not in (None, "", []):
            lines.append(f"- {label}: {', '.join(v) if isinstance(v, list) else v}")
    return "\n".join(lines) or "- (nothing known)"


class Chat:
    """reply(heard, ctx) -> {say, emotion, intensity, remember}; always answers (OFFLINE when it can't think)."""

    def __init__(self, model: str = MODEL, memory: Memory | None = None, client=None):
        self.model = model
        self.memory = memory or Memory()
        self.client = client
        self.system = None

    def _client(self):
        if self.client is None:
            import anthropic
            self.client = anthropic.Anthropic(timeout=30.0, max_retries=1)
        return self.client

    def messages(self, heard: str, ctx: dict, now: float) -> list:
        msgs = []
        for t in self.memory.turns(now):
            msgs += [dict(role="user", content=t["heard"]), dict(role="assistant", content=t["said"])]
        notes = self.memory.d.get("notes") or []
        context = (f"Facts right now:\n{facts_text(ctx)}\n"
                   + (f"Things you remember:\n" + "\n".join(f"- {n}" for n in notes) + "\n" if notes else "")
                   + f"\nThe person says: {heard}")
        return msgs + [dict(role="user", content=context)]

    def reply(self, heard: str, ctx: dict | None = None, now: float | None = None) -> dict:
        now = time.time() if now is None else now
        ctx = ctx or {}
        heard = " ".join(str(heard).split())[:400]
        if not heard:
            return dict(say="", emotion="neutral", intensity=0.0, remember="")
        try:
            import anthropic
        except ImportError:
            return dict(say=OFFLINE, emotion="sad", intensity=0.3, remember="", error="no anthropic package")
        system = [{"type": "text", "text": character_text() + "\n\n" + RULES, "cache_control": {"type": "ephemeral"}}]
        req = dict(model=self.model, max_tokens=1024, system=system, messages=self.messages(heard, ctx, now),
                   output_config={"effort": "low", "format": {"type": "json_schema", "schema": SCHEMA}})
        try:
            if "haiku" in self.model:     # no server-side fallback for Haiku (a fallbacks list is a 400)
                r = self._client().messages.create(**req)
            else:
                r = self._client().beta.messages.create(**req, betas=["server-side-fallback-2026-07-01"],
                                                        fallbacks="default")
        except anthropic.AuthenticationError:
            return dict(say=OFFLINE, emotion="sad", intensity=0.3, remember="", error="no API credentials")
        except anthropic.RateLimitError:
            return dict(say="Give me a moment. Too many thoughts at once.", emotion="tired", intensity=0.3,
                        remember="", error="rate limited")
        except anthropic.APIStatusError as e:
            return dict(say=OFFLINE, emotion="sad", intensity=0.3, remember="", error=f"API {e.status_code}")
        except anthropic.APIConnectionError:
            return dict(say=OFFLINE, emotion="sad", intensity=0.3, remember="", error="no connection")
        if r.stop_reason == "refusal":
            return dict(say="I'd rather not talk about that.", emotion="neutral", intensity=0.3, remember="")
        text = next((b.text for b in r.content if b.type == "text"), "")
        try:
            d = json.loads(text)
        except ValueError:
            return dict(say=OFFLINE, emotion="sad", intensity=0.3, remember="", error="unreadable reply")
        out = dict(say=str(d.get("say", "")).strip()[:300],
                   emotion=d.get("emotion") if d.get("emotion") in EMOTIONS else "neutral",
                   intensity=max(0.0, min(1.0, float(d.get("intensity", 0.5) or 0.5))),
                   remember=str(d.get("remember", "")).strip()[:120])
        self.memory.add(now, heard, out["say"], out["remember"])
        return out
