"""Inventing a skill (voice_brain_handoff_2026-11-23.md): when the voice brain has no tool or skill for a feasible
request, it asks a stronger model (Claude Opus 5.5 by default, OMNI_INVENT_MODEL) to write one. The new skill is
checked (skill_check.py), dry-run in a made-up house (skillbox.FakeAPI) and, if both pass, saved in the user's skill
folder for next time; the brain then asks the person before its first real run. One more attempt with the errors when
the first one fails its checks.

invent(request, context, client, library) -> {"ok", "skill" (skillbox.Skill), "plan", "message", "s"}
"""

from __future__ import annotations

import json
import os
import time

from skillbox import API, FakeAPI, Library, run_skill, sample_args, validate

MODEL = os.environ.get("OMNI_INVENT_MODEL", "claude-opus-5-5")
EFFORT = os.environ.get("OMNI_INVENT_EFFORT", "high")

SCHEMA = {
    "type": "object",
    "properties": {
        "feasible": {"type": "boolean", "description": "false if the robot can't do this safely with its API"},
        "why_not": {"type": "string", "description": "when not feasible: why, in one short sentence the robot says"},
        "plan": {"type": "string", "description": "what the skill will do, one sentence in the robot's voice, "
                                                  "said before the first run (e.g. 'I'll spin three times, drive "
                                                  "two metres and come back to you.')"},
        "code": {"type": "string", "description": "the whole skill file"},
    },
    "required": ["feasible", "why_not", "plan", "code"],
    "additionalProperties": False,
}

RULES = """You write skills for a household robot: an omnidirectional base (about 0.45 m across, driven by its own
navigation with a collision monitor) with an SO-101 arm and a bin, lidar, a depth camera looking at the floor ahead,
a wrist camera, a face screen and a speaker. A skill is a small Python file the robot runs to do one spoken request.

The file, exactly in this shape:
- a docstring: what the skill does, one or two sentences (people read it in a list);
- SKILL = dict(name="snake_case_name", says=[two or three example phrases], params={JSON-schema properties of run's
  keyword arguments, each with a description}, required=[names, optional], moves=True if it drives or moves the arm,
  max_s=seconds it may take at most, 30-900);
- optional constants and helper functions;
- def run(robot, <params with defaults>): does the work through `robot` and returns one short sentence saying what
  happened (the robot turns it into speech).

Rules for the code:
- Plain Python: loops, if, try/except, comprehensions, f-strings; import only math and random. No classes, no
  with, no async, no global, no yield, nothing named or attributed with a leading underscore, no eval / exec / open /
  getattr. Nothing else is importable and there is no file, network or clock besides robot's.
- Call robot's methods with keyword arguments only (robot.turn(deg=90), never robot.turn(90)).
- Motion methods wait until done and return {ok, state, message, evidence}; check ok and say what went wrong
  rather than carrying on blindly. A refused call raises RobotError (catch it when you can do something useful,
  e.g. say so and stop). When a person stops the robot the skill ends by itself; don't try to catch that.
- "me", "you" (the asker) = robot.nearest_person(). Places and rooms come from robot.places() / robot.rooms(); check
  a named place exists before driving there. Distances in metres, angles in degrees, + = left.
- Keep it small, safe and calm: no more than a few metres at a time unless asked, never a loop without an end
  (bound every loop by a count or robot.now()), short spoken lines only where they help.
- If the request needs something the API can't do (opening things, carrying people, going outside, anything
  dangerous, or a sense it doesn't have), set feasible to false and say why in why_not; then plan and code are "".

The robot's API (robot.<method>, keyword arguments):
"""


def api_text() -> str:
    return "\n".join(f"- {v}" for v in API.values())


def examples(library: Library) -> str:
    out = []
    for name in ("come_here", "check_room", "where_is"):
        s = library.all().get(name)
        if s is not None:
            out.append(f"Example skill {name}.py:\n```python\n{s.code.strip()}\n```")
    return "\n\n".join(out)


def _ask(client, system: str, prompt: str) -> dict:
    req = dict(model=MODEL, max_tokens=16000, system=system, messages=[dict(role="user", content=prompt)],
               output_config={"effort": EFFORT, "format": {"type": "json_schema", "schema": SCHEMA}})
    if "haiku" in MODEL:                          # no server-side fallback for Haiku
        r = client.messages.create(**req)
    else:
        r = client.beta.messages.create(**req, betas=["server-side-fallback-2026-07-01"], fallbacks="default")
    if r.stop_reason == "refusal":
        return dict(feasible=False, why_not="I'd rather not do that one.", plan="", code="")
    if r.stop_reason == "max_tokens":
        raise ValueError("the skill writer ran out of room")
    text = next((b.text for b in r.content if b.type == "text"), "")
    return json.loads(text)


def invent(request: str, context: dict, client, library: Library, attempts: int = 2) -> dict:
    """Write, check and save a skill for `request` (what the person asked, in their words). context: the robot's
    places, rooms and what it is doing, for the writer."""
    t0 = time.monotonic()
    existing = {n: s.description for n, s in library.all().items()}
    system = RULES + api_text() + "\n\n" + examples(library)
    prompt = (f"The person asked: {request!r}\n\nThe robot now: {json.dumps(context, default=str)[:1500]}\n\n"
              f"Skills it already has (don't duplicate them; a new skill may do something similar with a twist): "
              f"{json.dumps(existing)[:3000]}\n\nWrite the skill.")
    last = ""
    for k in range(attempts):
        d = _ask(client, system, prompt if not last else prompt + "\n\nYour last attempt failed its checks:\n" + last
                 + "\nFix it and write the whole file again.")
        if not d.get("feasible"):
            return dict(ok=False, message=d.get("why_not") or "I can't do that one.", s=round(time.monotonic() - t0, 1))
        code = str(d.get("code") or "")
        meta, bad = validate(code)
        if not bad and meta.get("name") in existing:
            bad = [f"the name {meta['name']} is taken: pick another"]
        if not bad:
            dry = run_skill(code, sample_args(meta), FakeAPI(), max_s=min(20.0, float(meta.get("max_s", 60))))
            if dry["outcome"] != "done":
                bad = [f"a dry run in a made-up house (one person 2 m ahead to the left; places kitchen, living room, "
                       f"home, front door; every motion succeeds) ended {dry['outcome']}: {dry['result']}"]
        if not bad:
            try:
                skill = library.save(code, request)
            except ValueError as e:
                bad = [str(e)]
            else:
                return dict(ok=True, skill=skill, plan=str(d.get("plan") or skill.description), attempts=k + 1,
                            message=f"wrote {skill.name}", s=round(time.monotonic() - t0, 1))
        last = "\n".join(f"- {b}" for b in bad) + f"\n\nThe file was:\n```python\n{code[:6000]}\n```"
    return dict(ok=False, message="I couldn't work that one out.", errors=last[:800], s=round(time.monotonic() - t0, 1))
