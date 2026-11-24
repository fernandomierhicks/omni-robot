"""Skills (skillbox.py, skill_check.py, skill_child.py, inventor.py): what a skill may contain, its sandboxed run, the
hand-written skills in a made-up house, the library, and the inventor with a scripted model. No robot, no API key."""

from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skill_check import problems  # noqa: E402
from skillbox import FakeAPI, Library, run_skill, sample_args, validate  # noqa: E402

GOOD = '''"""Spin once and say so."""
import math
SKILL = dict(name="spin_once", says=["spin once"], params={}, moves=True, max_s=30)


def run(robot):
    r = robot.move(steps=[{"do": "spin", "turns": 1}])
    robot.say(text="Wheee.", feeling="happy")
    return "spun" if r["ok"] else "couldn't spin"
'''


@pytest.mark.parametrize("bad, why", [
    ("import os\ndef run(robot):\n    return 1", "import os"),
    ("def run(robot):\n    return ().__class__", "attribute .__class__"),
    ("def run(robot):\n    return open('x')", "name open"),
    ("def run(robot):\n    g = (x for x in [1])\n    return g.gi_frame", "attribute .gi_frame"),
    ("class A:\n    pass\ndef run(robot):\n    return 1", "ClassDef"),
    ("def go(robot):\n    return 1", "no run"),
    ("def run(robot):\n    return robot._names", "attribute ._names"),
    ("def run(robot):\n    return getattr(robot, 'x')", "name getattr"),
    ("x = robot.move(steps=[])\ndef run(robot):\n    return 1", "at the top only"),
    ("def run(robot):\n    with x:\n        pass", "With"),
])
def test_the_check_refuses_the_ways_out(bad, why):
    assert any(why in p for p in problems(bad)), problems(bad)


def test_a_good_skill_passes_and_runs_in_its_own_process():
    meta, bad = validate(GOOD)
    assert not bad and meta["name"] == "spin_once" and meta["description"] == "Spin once and say so."
    api = FakeAPI()
    r = run_skill(GOOD, {}, api)
    assert r["outcome"] == "done" and r["result"] == "spun" and r["said"] == ["Wheee."]
    assert api.calls[0] == ("move", dict(steps=[{"do": "spin", "turns": 1}]))


def test_positional_arguments_and_unknown_methods_are_errors_the_skill_sees():
    code = GOOD.replace('robot.move(steps=[{"do": "spin", "turns": 1}])', "robot.move([1])")
    r = run_skill(code, {}, FakeAPI())
    assert r["outcome"] == "failed" and "keyword arguments" in r["result"]
    code = GOOD.replace("robot.say(", "robot.fly(")
    r = run_skill(code, {}, FakeAPI())
    assert r["outcome"] == "failed" and "no 'fly'" in r["result"]


def test_a_refusal_is_catchable_but_a_persons_stop_is_not():
    code = '''"""x"""
SKILL = dict(name="try_it", says=[], params={})


def run(robot):
    try:
        robot.goto(place="nowhere")
    except RobotError as e:
        return "refused: " + str(e)
'''
    assert run_skill(code, {}, FakeAPI())["result"].startswith("refused: Unknown place")
    code2 = code.replace('robot.goto(place="nowhere")', 'robot.move(steps=[{"do": "spin", "turns": 1}])') \
        .replace("except RobotError as e:", "except Exception as e:")
    r = run_skill(code2, {}, FakeAPI(fail=dict(move="stopped")))
    assert r["outcome"] == "stopped" and "person" in r["result"]


def test_the_time_limit_and_the_brains_stop_end_a_skill():
    loop = '''"""x"""
SKILL = dict(name="forever", says=[], params={})


def run(robot):
    while True:
        robot.wait(s=1)
'''
    t0 = time.monotonic()
    r = run_skill(loop, {}, FakeAPI(), max_s=1.5)
    assert r["outcome"] == "timeout" and time.monotonic() - t0 < 4.0
    stop = threading.Event()
    threading.Timer(0.5, stop.set).start()
    r = run_skill(loop, {}, FakeAPI(), max_s=30, stop=stop)
    assert r["outcome"] == "stopped" and r["s"] < 3.0


def test_a_skill_cant_import_or_reach_builtins_at_run_time():
    sneaky = '''"""x"""
SKILL = dict(name="sneaky", says=[], params={})


def run(robot):
    return str(sorted(dir_names()))


def dir_names():
    return [k for k in ("open", "eval")]
'''
    assert run_skill(sneaky, {}, FakeAPI())["outcome"] == "done"     # a string, nothing more
    imp = sneaky.replace('return str(sorted(dir_names()))', 'import random\n    return random.random() < 2')
    assert run_skill(imp, {}, FakeAPI())["result"] is True


ARGS = dict(check_room=dict(place="kitchen"), dance_at=dict(place="kitchen"), where_is=dict(color="red"),
            watch_door=dict(minutes=0.5), body_language=dict(do="nod"))


@pytest.mark.parametrize("name", sorted(Library(user_dir=Path("C:/nonexistent")).all()))
def test_each_hand_written_skill_runs_in_the_made_up_house(name):
    s = Library(user_dir=Path("C:/nonexistent")).get(name)
    assert s is not None and s.builtin and s.description
    api = FakeAPI()
    r = run_skill(s.code, ARGS.get(name, sample_args(s.meta)), api, s.max_s)
    assert r["outcome"] == "done", r
    moved = any(c[0] in ("move", "goto", "mission", "arm_pose", "pick") for c in api.calls)
    assert moved == s.moves or name == "where_is"           # where_is turns only when the thing isn't ahead


def test_skills_cope_with_nobody_there_and_with_a_failed_drive():
    lib = Library(user_dir=Path("C:/nonexistent"))
    r = run_skill(lib.get("come_here").code, {}, FakeAPI(people=[]))
    assert r["outcome"] == "done" and "couldn't see anyone" in r["result"]
    r = run_skill(lib.get("check_room").code, dict(place="kitchen"), FakeAPI(fail=dict(goto="failed")))
    assert r["outcome"] == "done" and "couldn't get to kitchen" in r["result"]
    r = run_skill(lib.get("where_is").code, dict(color="blue"), FakeAPI())
    assert "couldn't see a blue thing" in r["result"]


def test_the_library_keeps_renames_and_deletes_invented_skills(tmp_path):
    lib = Library(user_dir=tmp_path)
    s = lib.save(GOOD, "spin once for me")
    assert s.name == "spin_once" and not s.builtin and s.needs_ok and s.info["request"] == "spin once for me"
    lib.mark_tried("spin_once")
    assert not lib.get("spin_once").needs_ok
    lib.keep("spin_once")
    lib.rename("spin_once", "twirl")
    t = lib.get("twirl")
    assert t is not None and lib.get("spin_once") is None and t.info["kept"] and 'name="twirl"' in t.code
    with pytest.raises(ValueError):
        lib.save(GOOD.replace("spin_once", "come_here"), "x")             # a hand-written name
    with pytest.raises(KeyError):
        lib.delete("come_here")
    lib.delete("twirl")
    assert lib.get("twirl") is None and not list(tmp_path.glob("twirl*"))


# -- the inventor, with a scripted model ----------------------------------------------------------------------------
class FakeWriter:
    """client.beta.messages.create(...) -> the next scripted JSON answer."""

    def __init__(self, answers):
        self.answers, self.reqs = list(answers), []
        create = self.create
        self.beta = SimpleNamespace(messages=SimpleNamespace(create=create))
        self.messages = SimpleNamespace(create=create)

    def create(self, **req):
        self.reqs.append(req)
        text = json.dumps(self.answers.pop(0))
        return SimpleNamespace(stop_reason="end_turn", content=[SimpleNamespace(type="text", text=text)])


def test_the_inventor_retries_with_the_checks_errors_then_saves(tmp_path):
    from inventor import invent
    bad = GOOD.replace("import math", "import os")
    w = FakeWriter([dict(feasible=True, why_not="", plan="I'll spin once.", code=bad),
                    dict(feasible=True, why_not="", plan="I'll spin once.", code=GOOD)])
    lib = Library(user_dir=tmp_path)
    out = invent("spin around once", dict(places=["kitchen"]), w, lib)
    assert out["ok"] and out["skill"].name == "spin_once" and out["attempts"] == 2 and out["plan"] == "I'll spin once."
    assert "import os" in w.reqs[1]["messages"][0]["content"]            # the errors went back
    assert w.reqs[0]["model"] == "claude-opus-5-5" and w.reqs[0]["output_config"]["effort"] == "high"
    assert "robot.nearest_person" in w.reqs[0]["system"] or "nearest_person()" in w.reqs[0]["system"]
    assert lib.get("spin_once").needs_ok


def test_the_inventor_says_no_to_the_infeasible_and_to_a_skill_that_fails_its_dry_run(tmp_path):
    from inventor import invent
    w = FakeWriter([dict(feasible=False, why_not="I can't open doors yet.", plan="", code="")])
    out = invent("open the fridge", {}, w, Library(user_dir=tmp_path))
    assert not out["ok"] and out["message"] == "I can't open doors yet."
    crash = GOOD.replace('return "spun"', 'return 1 / 0 and "spun"')
    w = FakeWriter([dict(feasible=True, why_not="", plan="p", code=crash)] * 2)
    out = invent("spin", {}, w, Library(user_dir=tmp_path))
    assert not out["ok"] and "ZeroDivisionError" in out["errors"] and not list(tmp_path.glob("*.py"))
