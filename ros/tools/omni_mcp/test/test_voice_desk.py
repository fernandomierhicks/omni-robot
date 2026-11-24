"""The Claude Code voice brain's plumbing (voice_desk.py, voice_tools.py, server.py --voice): the desk's inbox, outbox,
cursor and heartbeat; the voice service's side (omni_face voice_server.DeskChat); the voice tools through MCP on the
console's real core: the quick reply, the gate per sentence, the plan answered "go", a skill, saving a skill, the lease
let go when idle. No Claude: the test plays the session's part."""

from __future__ import annotations

import asyncio
import io
import json
import sys
import threading
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

import voice_desk as desk  # noqa: E402


@pytest.fixture(autouse=True)
def tmp_desk(tmp_path, monkeypatch):
    monkeypatch.setattr(desk, "DESK", tmp_path / "desk")
    yield tmp_path / "desk"


def test_follow_delivers_new_sentences_once_and_skips_answers_and_stale_ones():
    out = io.StringIO()
    desk.follow(out, once=True)                                     # first run: starts at the end, beats
    assert desk.alive() and out.getvalue() == ""
    a = desk.post("take a picture of me", dict(feeling="happy (0.4)", battery=80))
    desk._append("inbox.jsonl", dict(id="old", t=time.time() - 500, text="stale", ctx={}, kind="say"))
    desk.plan_open(10)
    yes = desk.post("yes")
    assert yes["kind"] == "answer" and desk.plan_waiting()
    desk.follow(out, once=True)
    lines = [json.loads(x) for x in out.getvalue().splitlines()]
    assert lines == [dict(id=a["id"], heard="take a picture of me", facts="feels: happy (0.4); battery %: 80")]
    desk.follow(out, once=True)                                     # the cursor: nothing twice
    assert len(out.getvalue().splitlines()) == 1
    assert desk.wait_answer(a["t"], 0.5) == "yes"
    desk.plan_close()
    assert desk.post("yes")["kind"] == "say"                        # no plan waiting: an ordinary sentence


def test_a_quick_reply_reaches_the_voice_service_a_late_one_does_not():
    e = desk.post("hello there")
    assert desk.reply(e["id"], "Hello.", "happy", 0.5)
    assert desk.wait_reply(e["id"], time.time() + 1)["say"] == "Hello."
    late = desk.post("hi")
    desk._append("inbox.jsonl", dict(late, t=time.time() - 30, id="late1"))
    assert not desk.reply("late1", "Too late.")


def test_the_voice_service_hands_sentences_to_the_session(monkeypatch):
    face = HERE.parents[2] / "src" / "omni_face"
    sys.path.insert(0, str(face))
    pytest.importorskip("numpy")
    from omni_face.voice_server import DeskChat, NOT_LISTENING
    chat = DeskChat()
    assert chat.reply("what do you think of cats")["say"] == NOT_LISTENING     # no session listening
    desk.follow(io.StringIO(), once=True)                                      # a session beats

    def session():                                                             # it answers within the window
        t_end = time.time() + 5
        while time.time() < t_end:
            e = desk.latest()
            if e and e["text"] == "what do you think of cats":
                desk.reply(e["id"], "Fine animals. Poor at chess.", "content", 0.4)
                return
            time.sleep(0.05)
    threading.Thread(target=session, daemon=True).start()
    r = chat.reply("what do you think of cats", {})
    assert r["say"] == "Fine animals. Poor at chess." and r["emotion"] == "content"
    monkeypatch.setattr(desk, "REPLY_S", 0.5)
    assert chat.reply("do a figure of eight")["say"] == "One moment."          # slow: the answer comes later
    desk.plan_open(10)
    assert chat.reply("yes")["say"].startswith("Okay, here I go")


# -- the voice tools through MCP, on the console's core ------------------------------------------------------------
def call(server, name, args=None):
    from mcp import Client

    async def go():
        async with Client(server) as c:
            r = await c.call_tool(name, args or {})
            return json.loads(r.content[0].text)
    return asyncio.run(go())


@pytest.fixture
def voice(tmp_path, monkeypatch):
    pytest.importorskip("mcp")
    from test_omni_mcp import ConsoleFake, free_port, wait
    import voice_tools
    from robot_client import Robot
    from server import build_server
    from skillbox import Library
    monkeypatch.setattr(voice_tools, "IDLE_S", 1.5)
    f = ConsoleFake(free_port())
    robot = Robot("127.0.0.1", f.port, free_port(), free_port(), session="voice-test", lease=False)
    assert robot.rb.wait_up(3.0) and wait(lambda: robot.status()["ok"], 3.0)
    st = voice_tools.VoiceState(robot, library=Library(user_dir=tmp_path / "skills"))
    yield f, robot, st, build_server(robot, st)
    st.close()
    robot.close()
    f.close()


def moves(f):
    return [a for v, a in f.calls if v == "move"]


def test_the_voice_server_offers_its_tools_and_answers_quickly(voice):
    f, robot, st, server = voice
    from mcp import Client

    async def names():
        async with Client(server) as c:
            return {t.name for t in (await c.list_tools()).tools}
    n = asyncio.run(names())
    assert {"robot_voice_reply", "robot_skill", "robot_skill_wait", "robot_propose_plan", "robot_save_skill",
            "robot_remind", "robot_move", "robot_look"} <= n
    e = desk.post("hello robot")
    r = call(server, "robot_voice_reply", dict(request_id=e["id"], say="Hello.", feeling="happy"))
    assert r["data"]["how"] == "chat reply" and desk.wait_reply(e["id"], time.time() + 1)["say"] == "Hello."
    late = dict(desk.post("hi"), t=time.time() - 60, id="late2")
    desk._append("inbox.jsonl", late)
    r = call(server, "robot_voice_reply", dict(request_id="late2", say="Hi again.", feeling="happy"))
    assert r["data"]["how"] == "said through the console"
    assert [a for v, a in f.calls if v == "say"][-1]["text"] == "Hi again."


def test_a_second_motion_waits_for_go_and_the_lease_goes_when_idle(voice):
    from test_omni_mcp import wait
    f, robot, st, server = voice
    desk.post("spin a quarter, then turn back")
    assert robot.lease is None
    r = call(server, "robot_move", dict(steps=[{"do": "spin", "turns": 0.25}, {"do": "turn", "deg": -90}]))
    assert not r["ok"] and r["data"]["error"] == "ask_first" and not moves(f)          # several steps: ask
    r = call(server, "robot_move", dict(steps=[{"do": "spin", "turns": 0.25}], turn_rate="max"))
    assert r["ok"] and robot.lease is not None                                          # one: at once
    assert call(server, "robot_task", dict(task_id=r["data"]["task_id"], wait_s=10))["ok"]
    r2 = call(server, "robot_move", dict(steps=[{"do": "turn", "deg": -90}]))
    assert not r2["ok"] and r2["data"]["error"] == "ask_first"                          # a second: ask

    def say_go():                                                                       # the person says go
        assert wait(desk.plan_waiting, 5.0)
        desk.post("yes")
    threading.Thread(target=say_go, daemon=True).start()
    p = call(server, "robot_propose_plan", dict(say="I'll turn back a quarter. Say go.", wait_s=10))
    assert p["data"]["confirmed"]
    r3 = call(server, "robot_move", dict(steps=[{"do": "turn", "deg": -90}], turn_rate="max"))
    assert r3["ok"] and len(moves(f)) == 2
    call(server, "robot_task", dict(task_id=r3["data"]["task_id"], wait_s=10))
    desk.post("and now spin")                                                           # a new sentence: a fresh count
    assert st.gate() == ""
    assert wait(lambda: robot.lease is None, 8.0)                                       # idle: the lease is let go


def test_skills_run_and_new_ones_are_checked_saved_and_asked_about(voice):
    f, robot, st, server = voice
    desk.post("look at me")
    r = call(server, "robot_skill", dict(name="face_me"))
    if r["data"]["state"] == "RUNNING":
        r = call(server, "robot_skill_wait", dict(run_id=r["data"]["run_id"], wait_s=30))
    assert r["data"]["state"] == "DONE" and "couldn't see anyone" in r["data"]["result"]
    assert moves(f)[0]["steps"] == [{"do": "turn", "deg": 180.0}]
    bad = call(server, "robot_save_skill", dict(code="import os\ndef run(robot):\n    return 1", request="x"))
    assert not bad["ok"] and any("import os" in p for p in bad["data"]["problems"])
    code = '''"""Say hello and wiggle."""
SKILL = dict(name="hello_wiggle", says=["say hello and wiggle"], params={}, moves=False, max_s=20)


def run(robot):
    robot.gesture(base="wiggle", feeling="happy")
    robot.say(text="Hello!", feeling="happy")
    return "said hello"
'''
    ok = call(server, "robot_save_skill", dict(code=code, request="say hello and wiggle"))
    assert ok["ok"] and ok["data"]["name"] == "hello_wiggle"
    desk.post("say hello and wiggle")
    r = call(server, "robot_skill", dict(name="hello_wiggle"))
    assert not r["ok"] and r["data"]["error"] == "ask_first"                            # new: ask first
    st.confirm()
    r = call(server, "robot_skill", dict(name="hello_wiggle"))
    assert r["data"]["state"] == "DONE" and st.lib().get("hello_wiggle").info["tried"]
