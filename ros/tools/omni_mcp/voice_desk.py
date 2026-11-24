"""The voice desk (Nov 23, Fernando: "session kept open, route all unknown things to Claude even if it's slow"): how a
heard sentence reaches a Claude Code session that stays open on the PC, and how its answer comes back. Standard
library only (the voice service, the session's Monitor and its MCP server each use it, from different venvs).

    voice service (/chat)  --post-->   inbox.jsonl   --follow-->  the session's Monitor (one line per sentence)
    voice service          <--reply--  outbox.jsonl  <--------    robot_voice_reply (omni_mcp server.py --voice)

Files in DESK (OMNI_VOICE_DESK, default ~/.local/share/omni_voice/desk):
  inbox.jsonl    one request per line: {id, t, text, ctx, kind: "say" | "answer"} ("answer": "yes" / "no" to a plan)
  outbox.jsonl   quick replies {id, say, emotion, intensity}: the voice service answers /chat with one if it comes
                 within REPLY_S; later answers are said through the robot's console instead (server.py)
  cursor         how far follow() has delivered (a re-armed Monitor carries on where the last one stopped)
  heartbeat      touched by follow() every few seconds: the session is listening (alive())
  plan.json      a plan waits for "go" until its time (robot_propose_plan)

    python voice_desk.py follow          what the session's Monitor runs (voice_follow.sh)
    python voice_desk.py post "text"     a sentence as if heard (testing)
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
import uuid
from pathlib import Path

DESK = Path(os.environ.get("OMNI_VOICE_DESK", Path.home() / ".local" / "share" / "omni_voice" / "desk"))
REPLY_S = 10.0          # the voice service waits this long for a quick reply, then says "One moment."
FRESH_S = 120.0         # a sentence older than this isn't delivered (nobody was listening when it was said)
ALIVE_S = 90.0          # the session counts as listening if follow() beat within this
BEAT_S = 5.0
_lock = threading.Lock()


def _p(name: str) -> Path:
    return DESK / name


def _append(name: str, obj: dict):
    DESK.mkdir(parents=True, exist_ok=True)
    with _lock, open(_p(name), "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, default=str) + "\n")


def _lines(name: str) -> list:
    try:
        with open(_p(name), encoding="utf-8") as f:
            out = []
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
            return out
    except OSError:
        return []


# -- the voice service's side -----------------------------------------------------------------------------------------
def alive(now: float | None = None) -> bool:
    try:
        return (now or time.time()) - _p("heartbeat").stat().st_mtime < ALIVE_S
    except OSError:
        return False


def plan_waiting(now: float | None = None) -> bool:
    try:
        d = json.loads(_p("plan.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False
    return (now or time.time()) < float(d.get("until", 0))


def post(text: str, ctx: dict | None = None) -> dict:
    """A heard sentence into the inbox -> its entry. "yes" / "no" while a plan waits is an answer to it."""
    text = " ".join(str(text).split())[:400]
    kind = "answer" if text in ("yes", "no") and plan_waiting() else "say"
    e = dict(id=uuid.uuid4().hex[:10], t=time.time(), text=text, ctx=ctx or {}, kind=kind)
    _append("inbox.jsonl", e)
    return e


def wait_reply(rid: str, until: float, poll: float = 0.1):
    """The quick reply to request rid, if it comes before `until` (time.time()), else None."""
    while True:
        for d in reversed(_lines("outbox.jsonl")):
            if d.get("id") == rid:
                return d
        if time.time() >= until:
            return None
        time.sleep(poll)


# -- the session's side -----------------------------------------------------------------------------------------------
def request(rid: str):
    return next((e for e in reversed(_lines("inbox.jsonl")) if e.get("id") == rid), None)


def latest(kind: str = "say"):
    return next((e for e in reversed(_lines("inbox.jsonl")) if e.get("kind") == kind), None)


def reply(rid: str, say: str, emotion: str = "neutral", intensity: float = 0.5) -> bool:
    """A quick reply -> True if the voice service is still waiting for it (else say it through the robot)."""
    e = request(rid)
    if e is None or time.time() > float(e["t"]) + REPLY_S - 1.0:
        return False
    _append("outbox.jsonl", dict(id=rid, say=str(say)[:300], emotion=emotion, intensity=float(intensity),
                                 remember="", t=time.time()))
    return True


def plan_open(seconds: float):
    DESK.mkdir(parents=True, exist_ok=True)
    _p("plan.json").write_text(json.dumps(dict(t=time.time(), until=time.time() + seconds)), encoding="utf-8")


def plan_close():
    try:
        _p("plan.json").unlink()
    except OSError:
        pass


def wait_answer(after: float, timeout: float, stop=None):
    """'yes' | 'no' said after time `after`, or None when `timeout` s pass."""
    t_end = time.time() + timeout
    while time.time() < t_end and not (stop and stop()):
        for e in reversed(_lines("inbox.jsonl")):
            if float(e.get("t", 0)) <= after:
                break
            if e.get("kind") == "answer":
                return e["text"]
        time.sleep(0.2)
    return None


def _brief(ctx: dict) -> str:
    keys = (("feeling", "feels"), ("mission", "doing"), ("battery", "battery %"), ("time", "time"),
            ("recent", "just happened"), ("voice", "voice commands"))
    out = []
    for k, label in keys:
        v = ctx.get(k)
        if v not in (None, "", []):
            out.append(f"{label}: {', '.join(map(str, v)) if isinstance(v, list) else v}")
    return "; ".join(out)


def follow(out=None, once: bool = False, poll: float = 0.3):
    """Print each new sentence as one JSON line {id, heard, facts} (the session's Monitor events), from the cursor;
    beat the heartbeat. Answers ("yes"/"no" to a plan) are for robot_propose_plan, not printed. once: one pass (tests)."""
    out = out or sys.stdout
    DESK.mkdir(parents=True, exist_ok=True)
    inbox, cur = _p("inbox.jsonl"), _p("cursor")
    try:
        pos = int(cur.read_text())
    except (OSError, ValueError):
        pos = inbox.stat().st_size if inbox.exists() else 0     # first run: nothing old is replayed
        cur.write_text(str(pos))
    t_beat = 0.0
    while True:
        now = time.time()
        if now - t_beat >= BEAT_S:
            _p("heartbeat").write_text(str(now))
            t_beat = now
        size = inbox.stat().st_size if inbox.exists() else 0
        if size < pos:
            pos = 0                                              # the inbox was cleared
        if size > pos:
            with open(inbox, "rb") as f:
                f.seek(pos)
                data = f.read()
            end = data.rfind(b"\n") + 1                          # whole lines only
            for raw in data[:end].splitlines():
                try:
                    e = json.loads(raw)
                except ValueError:
                    continue
                if e.get("kind") != "say" or now - float(e.get("t", 0)) > FRESH_S:
                    continue
                out.write(json.dumps(dict(id=e["id"], heard=e["text"], facts=_brief(e.get("ctx") or {}))) + "\n")
                out.flush()
            pos += end
            cur.write_text(str(pos))
        if once:
            return
        time.sleep(poll)


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "follow":
        try:
            follow()
        except KeyboardInterrupt:
            pass
    elif len(sys.argv) >= 3 and sys.argv[1] == "post":
        print(json.dumps(post(" ".join(sys.argv[2:]))))
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
