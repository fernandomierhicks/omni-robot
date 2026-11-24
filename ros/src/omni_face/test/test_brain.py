"""The voice brain (brain.py) end to end without an API key: a scripted model in place of Claude, the real toolkit,
skills and sandbox, and the console's real core (omni_console missions.py) behind omni_mcp's fake rosbridge. Also the
voice commands' routing for it (commands.py).

    C:\\Users\\ferna\\.venvs\\omni-voice\\Scripts\\python.exe -m pytest -q ros/src/omni_face/test/test_brain.py
(the voice venv: anthropic and websockets; the sim venv runs only the routing tests)
"""

from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "omni_face"))

from omni_face.commands import confirmation, parse  # noqa: E402


# -- routing (commands.py) -------------------------------------------------------------------------------------------
def test_plans_are_answered_with_or_without_the_name():
    assert confirmation("go") == "yes" and confirmation("Omni, go ahead") == "yes" and confirmation("yes please") == "yes"
    assert confirmation("no") == "no" and confirmation("never mind") == "no" and confirmation("stop") == "no"
    assert confirmation("go to the kitchen") is None and confirmation("what") is None


def test_requests_for_the_brain_are_not_eaten_by_a_phrase_inside_them():
    places = {"kitchen": (1, 2, 0)}
    assert parse("come to me", places, woke=True) is None                 # not a place called "me"
    assert parse("go over there", places, woke=True) is None
    assert parse("go dance in the kitchen", places, woke=True) is None    # not the face's dance antic
    assert parse("take a picture of me", places, woke=True) is None
    assert parse("could you follow me", places, woke=True).name == "follow"     # still a command
    assert parse("go to the kitchen", places, woke=True).name == "goto"
    assert parse("please stop that right now you silly robot", places, woke=True).name == "stop"


# -- the brain on the console's core ---------------------------------------------------------------------------------
anthropic = pytest.importorskip("anthropic")
pytest.importorskip("websockets")
sys.path.insert(0, str(ROOT / "tools" / "omni_mcp" / "test"))
sys.path.insert(0, str(ROOT / "tools" / "omni_mcp"))

from test_omni_mcp import ConsoleFake, free_port, wait  # noqa: E402

from omni_face.brain import Brain  # noqa: E402
from omni_face.chat import Memory  # noqa: E402

B = SimpleNamespace


def use(name, **inp):
    return B(type="tool_use", id=f"tu_{name}_{time.monotonic_ns()}", name=name, input=inp)


def fin(say, emotion="happy", intensity=0.6, remember=""):
    return use("finish", say=say, emotion=emotion, intensity=intensity, remember=remember)


def resp(*blocks):
    stop = "tool_use" if any(b.type == "tool_use" for b in blocks) else "end_turn"
    return B(stop_reason=stop, content=list(blocks))


def text(s):
    return B(type="text", text=s)


def last_results(req) -> list:
    """The tool results of the request's last user turn, parsed."""
    out = []
    for c in req["messages"][-1]["content"]:
        body = c["content"]
        if isinstance(body, list):
            body = body[0]["text"]
        out.append(json.loads(body) if body.startswith(("{", "[")) else body)
    return out


class Model:
    """Claude, scripted: each step is a response or f(request) -> response."""

    def __init__(self, steps):
        self.steps, self.reqs = list(steps), []
        self.messages = SimpleNamespace(create=self.create)

    def create(self, **req):
        req = dict(req, messages=list(req["messages"]))          # as sent (the brain appends to its list later)
        self.reqs.append(req)
        s = self.steps.pop(0)
        return s(req) if callable(s) else s


@pytest.fixture
def rig(tmp_path):
    f = ConsoleFake(free_port())
    made = []

    def make(steps, writer=None, ctx_extra=None):
        from robot_client import Robot
        from skillbox import Library
        model = Model(steps)
        brain = Brain(memory=Memory(tmp_path / "mem.json"), client=model,
                      library=Library(user_dir=tmp_path / "skills"), inventor_client=writer, log=lambda m: None)
        brain.robot = Robot("127.0.0.1", f.port, free_port(), free_port(), session=brain.session, lease=False,
                            log=lambda m: None)
        assert brain.robot.rb.wait_up(3.0)
        assert wait(lambda: brain.robot.status()["ok"], 3.0)
        made.append(brain)
        return brain, model
    yield f, make
    for b in made:
        b.cancel()
        if b.job is not None:
            b.job.done.wait(10.0)
        b.robot.close()
    f.close()


def moves(f) -> list:
    return [a for v, a in f.calls if v == "move"]


def test_plain_talk_uses_no_robot_and_remembers(rig):
    f, make = rig
    brain, model = make([resp(fin("Cats are fine. They don't shed bolts.", "content", 0.4, "Ana likes cats"))])
    r = brain.reply("what do you think of cats", dict(feeling="content (0.3)"))
    assert r["say"].startswith("Cats are fine") and r["emotion"] == "content"
    assert brain.job.done.wait(3.0) and not moves(f) and not any(v == "ai_lease" for v, _ in f.calls)
    assert brain.memory.d["notes"] == ["Ana likes cats"]
    req = model.reqs[0]
    names = {t["name"] for t in req["tools"]}
    assert {"robot_move", "robot_look", "skill_come_here", "skill_take_photo", "propose_plan", "invent_skill",
            "finish"} <= names and "robot_topic" not in names and "robot_task" not in names
    assert "Saved places" in req["messages"][-1]["content"] and req["model"] == "claude-haiku-5-5"


def test_one_ready_action_starts_at_once_and_reports_the_evidence(rig):
    f, make = rig
    brain, model = make([
        resp(text("Watch this."), use("robot_move", steps=[{"do": "spin", "turns": 1}], turn_rate="max")),
        lambda req: resp(use("robot_wait", task_id=last_results(req)[0]["data"]["task_id"], max_s=30)),
        lambda req: resp(fin(f"Done: I turned {round(last_results(req)[0]['data']['evidence']['turned_deg'])} "
                             "degrees.", "proud", 0.7)),
    ])
    t0 = time.monotonic()
    r = brain.reply("spin around once")
    assert r["say"] == "Watch this." and time.monotonic() - t0 < 3.0        # said as the task starts
    assert brain.job.done.wait(20.0)
    mv = moves(f)
    assert len(mv) == 1 and mv[0]["ai"] == brain.session and mv[0]["steps"] == [{"do": "spin", "turns": 1}]
    said = [a for v, a in f.calls if v == "say"]
    assert said and said[-1]["feel"] == "proud" and said[-1]["text"].startswith("Done: I turned ")
    assert abs(int(said[-1]["text"].split()[3]) - 360) <= 5                   # the odometry's evidence, reported
    assert brain.job.lease is None and any(v == "ai_lease" for v, _ in f.calls)


def test_more_than_one_motion_waits_for_go(rig):
    f, make = rig
    plan = "I'll spin a quarter turn, then turn back. Say go."
    brain, model = make([
        resp(use("robot_move", steps=[{"do": "spin", "turns": 0.25}, {"do": "turn", "deg": -90}])),
        lambda req: resp(use("propose_plan", say=plan)),
        lambda req: resp(use("robot_move", steps=[{"do": "spin", "turns": 0.25}, {"do": "turn", "deg": -90}],
                             turn_rate="max")),
        lambda req: resp(use("robot_wait", task_id=last_results(req)[0]["data"]["task_id"])),
        resp(fin("All done.", "happy")),
    ])
    r = brain.reply("spin a quarter and turn back")
    assert r["say"] == plan and r["pending"] > 0 and not moves(f)
    assert "Ask first" in json.dumps(last_results(model.reqs[1])[0])          # the gate, not the model, said so
    r2 = brain.reply("yes")
    assert r2["say"].startswith("Okay")
    assert brain.job.done.wait(20.0)
    assert len(moves(f)) == 1 and brain.job.final["say"] == "All done."


def test_a_no_means_nothing_moves(rig):
    f, make = rig
    brain, model = make([
        resp(use("propose_plan", say="I'll spin twice and drive a metre. Say go.")),
        lambda req: resp(fin("Okay, staying put." if not last_results(req)[0]["confirmed"] else "Going.", "neutral")),
    ])
    assert brain.reply("spin twice then drive a metre")["pending"]
    assert brain.reply("no")["say"] == "Okay, I won't."
    assert brain.job.done.wait(5.0) and not moves(f) and brain.job.final["say"] == "Okay, staying put."


def test_a_skill_runs_in_its_sandbox_on_the_robot(rig):
    f, make = rig
    brain, model = make([
        resp(use("skill_face_me")),
        lambda req: resp(fin(last_results(req)[0]["result"], "neutral")),
    ])
    brain.reply("look at me")
    assert brain.job.done.wait(30.0)
    res = last_results(model.reqs[1])[0]
    assert res["outcome"] == "done" and "couldn't see anyone" in res["result"]   # no lidar in the fake: nobody
    assert moves(f) and moves(f)[0]["steps"] == [{"do": "turn", "deg": 180.0}]   # it looked behind, via the console


def test_a_person_stopping_the_robot_ends_the_request(rig):
    f, make = rig

    def stop_soon(req):
        tid = last_results(req)[0]["data"]["task_id"]
        threading.Timer(0.8, lambda: f.on_call("stop", {"by": "voice"})).start()   # "Omni, stop" (a person)
        return resp(use("robot_wait", task_id=tid))
    brain, model = make([
        resp(use("robot_move", steps=[{"do": "spin", "turns": 3}], turn_rate="max")),
        stop_soon,
        resp(use("robot_move", steps=[{"do": "spin", "turns": 1}])),         # a model that tries to carry on
        resp(fin("Stopped, as you asked.", "neutral")),
    ])
    brain.reply("spin three times")
    assert brain.job.done.wait(20.0)
    assert last_results(model.reqs[2])[0]["data"]["state"] == "CANCELLED"
    assert "person stopped" in json.dumps(last_results(model.reqs[3])[0]) and len(moves(f)) == 1


def test_an_invented_skill_is_written_checked_asked_about_run_and_kept(rig):
    f, make = rig
    code = '''"""Spin once on the spot and say whee."""
SKILL = dict(name="spin_whee", says=["spin and say whee"], params={}, moves=True, max_s=30)


def run(robot):
    r = robot.move(steps=[{"do": "spin", "turns": 1}], turn_rate="max")
    robot.say(text="Whee!", feeling="excited")
    return "spun once" if r["ok"] else "couldn't spin: " + str(r["message"])
'''
    answer = json.dumps(dict(feasible=True, why_not="", plan="I'll spin once and say whee.", code=code))
    writer = SimpleNamespace(beta=SimpleNamespace(messages=SimpleNamespace(create=lambda **req: B(
        stop_reason="end_turn", content=[text(answer)]))))
    brain, model = make([
        resp(use("invent_skill", request="spin and shout whee")),
        lambda req: resp(use(last_results(req)[0]["skill"])),                # straight to it: the gate says ask
        lambda req: resp(use("propose_plan", say="I'll spin once and say whee. Say go.")),
        resp(use("skill_spin_whee")),
        lambda req: resp(fin("Whee indeed.", "excited")),
    ], writer=writer)
    r = brain.reply("spin and shout whee")
    assert r["say"] == "Let me work that out."
    assert wait(lambda: brain.job.pending, 10.0)
    assert "new and nobody has tried it" in json.dumps(last_results(model.reqs[2])[0]) and not moves(f)
    brain.reply("go")
    assert brain.job.done.wait(30.0)
    assert len(moves(f)) == 1 and brain.job.final["say"] == "Whee indeed."
    s = brain.library.get("spin_whee")
    assert s is not None and not s.builtin and s.info["tried"] and s.info["request"] == "spin and shout whee"
    assert any(t["name"] == "skill_spin_whee" for t in model.reqs[-1]["tools"])


def test_chat_only_offers_no_motion(rig):
    f, make = rig
    brain, model = make([resp(fin("Voice driving is off, so I'll stay here.", "neutral"))])
    brain.reply("go dance in the kitchen", dict(voice="chat only"))
    assert brain.job.done.wait(5.0)
    names = {t["name"] for t in model.reqs[0]["tools"]}
    assert "robot_move" not in names and "skill_dance_at" not in names and "invent_skill" not in names
    assert "skill_celebrate" in names and "robot_look" in names
