"""omni_face: what the robot shows and says, its antics, its voice commands, and that the display draws every state
(headless). Run: python -m pytest -q ros/src/omni_face/test"""

import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

from omni_face import commands, moments, sounds  # noqa: E402
from omni_face.moments import ANTICS, Brain, BrainConfig, FaceState  # noqa: E402

QUIET = BrainConfig(antics="off")


def status(**kw):
    s = dict(mission="IDLE", player="Ana", mcu=dict(mode="AUTO", profile="adult", faults=[], battery_pct=80,
                                                     battery_low=False, age=0.05))
    s.update(kw)
    return s


def says(outs, prio=None):
    return [o.value for o in outs if o.kind == "say" and (prio is None or o.prio == prio)]


def plays(outs):
    return [o.value for o in outs if o.kind == "sound"]


def test_no_status_is_a_sleepy_face_waiting_for_its_software():
    st, outs = Brain().update(0.0, None)
    assert not st.connected and st.mood == "sleepy" and outs == []


def test_a_fault_shows_a_card_is_said_once_and_clears():
    b = Brain(QUIET)
    b.update(0.0, status())
    bump = dict(name="BUMPER", hint="Move the robot clear by hand.")
    st, outs = b.update(1.0, status(mcu=dict(mode="AUTO", faults=[bump], battery_pct=80, age=0.05)))
    assert st.card["title"] == "Bumper pressed" and st.card["hint"].startswith("Move")
    assert says(outs, 3) and any(o.kind == "sound" and o.value == "warn" for o in outs)
    st, outs = b.update(1.1, status(mcu=dict(mode="AUTO", faults=[bump], battery_pct=80, age=0.05)))
    assert st.card and not says(outs)                                  # still shown, not said again
    st, _ = b.update(2.0, status())
    assert st.card is None
    # the e-stop wins the card over a lesser fault, in red
    es = dict(name="ESTOP", hint="Release it.")
    st, outs = b.update(3.0, status(mcu=dict(mode="ESTOP", faults=[bump, es], battery_pct=80, age=0.05)))
    assert st.card["level"] == "error" and st.card["title"].startswith("Emergency stop") and st.mood == "alert"
    # an idle robot that isn't being commanded isn't news
    st, outs = b.update(4.0, status(mcu=dict(mode="DISARMED", faults=[dict(name="LINK_LOST")], battery_pct=80, age=0.05)))
    assert st.card is None and not says(outs, 3)


def test_a_silent_mcu_gets_its_own_card():
    b = Brain()
    st, outs = b.update(0.0, status(mcu=dict(mode="AUTO", faults=[], battery_pct=80, age=3.0)))
    assert st.card["title"] == "Motor controller silent" and says(outs, 3)


def test_catch_game_catches_misses_streaks_and_prompts():
    b = Brain(QUIET)
    cat = lambda state, t, c, last=None, ball=None: status(mission="CATCH", catch=dict(   # noqa: E731
        state=state, tosses=t, catches=c, last=last or {}, ball=ball))
    st, outs = b.update(0.0, cat("wait", 0, 0))
    assert st.prompt == "Throw it to me!" and st.score["caught"] == 0
    _, outs = b.update(9.0, cat("wait", 0, 0))
    assert says(outs, 1)                                               # "throw it to me!" after 8 s
    _, outs = b.update(10.0, cat("wait", 0, 0))
    assert not says(outs)                                              # ... not again for 30 s
    n, last = 0, {}
    for k in range(3):        # three catches in a row: tosses counts at the release, the record comes after the settle
        n += 1
        _, outs = b.update(10.0 + 10 * k, cat("flight", n, n - 1, last=last, ball=[3.0, 0.0, 1.5]),
                           pose=(0.0, 0.0, 0.0))
        assert not any(o.value in ("catch", "miss") for o in outs)    # nothing decided yet
        last = dict(toss=n, caught=True, speed=6.0)
        st, outs = b.update(11.0 + 10 * k, cat("aim", n, n, last=last))
        assert any(o.value == "catch" for o in outs) and st.mood == "happy"
    assert "Three in a row!" in says(outs, 2) and st.score["streak"] == 3
    b.update(49.0, cat("flight", n + 1, n, last=last))
    st, outs = b.update(50.0, cat("wait", n + 1, n, last=dict(toss=n + 1, caught=False, predicted=[2.5, 0.0])))
    assert any(o.value == "miss" for o in outs) and st.mood == "sad" and st.score["streak"] == 0
    assert says(outs, 2)[0] in ("That one was too far for me!", "Too far! Throw it closer.")
    # the next run beats the record of 3 once (at 4), not at every catch after it
    said = []
    for k in range(6):
        n += 1
        b.update(60.0 + 10 * k, cat("flight", n, n - 1, last=last))
        last = dict(toss=n, caught=True)
        said += says(b.update(61.0 + 10 * k, cat("aim", n, n, last=last))[1], 2)
    assert said.count("New record: 4 in a row!") == 1 and not any("New record: 6" in s for s in said)
    _, outs = b.update(55.0, cat("launch", n + 1, n))
    assert any(o.value == "launch" for o in outs)


def test_it_asks_for_a_throw_three_times_at_most_then_waits_quietly():
    """Fernando (Oct 26): "ready when you are" got annoying. 8 s, then 30 s and 60 s apart, then only the screen."""
    b = Brain(QUIET)
    cat = lambda state, t=0, c=0: status(mission="CATCH", catch=dict(state=state, tosses=t, catches=c))  # noqa: E731
    asked = []
    for k in range(0, 6000):
        _, outs = b.update(k * 0.1, cat("wait"))
        asked += [(k * 0.1, s) for s in says(outs, 1) if s != "Hi! I'm ready."]     # (its hello at start-up)
    assert [round(t) for t, _ in asked] == [8, 38, 98]                 # 10 minutes: three times
    assert all(s != "Ready when you are!" for _, s in asked)
    st, _ = b.update(600.0, cat("wait"))
    assert st.prompt == "Throw it to me!"                              # the screen still asks
    b.update(601.0, cat("flight", 1))                                  # a throw: the count starts again
    asked = [s for k in range(6100, 6200) for s in says(b.update(k * 0.1, cat("wait", 1, 1))[1], 1)]
    assert len(asked) == 1


def test_a_spoken_quip_at_most_every_five_minutes():
    b = Brain(BrainConfig(antics="lively", antic_first=1.0), seed=2)
    acts, outs = run_rest(b, 0.0, 1800.0, lambda t: status())
    quips = [s for s in says(outs, 1) if s in moments.QUIPS]
    assert len(acts) > 40 and 1 <= len(quips) <= 7


def test_personas_patrol_is_stern_explore_eager_tidy_cheerful():
    b = Brain(QUIET)
    b.update(0.0, status())
    st, outs = b.update(1.0, status(mission="PATROL", patrol=dict(lap=1)))
    assert st.persona == "patrol" and st.mood == "stern" and st.label == "Patrolling"
    hello = [o for o in outs if o.kind == "say"]
    assert hello and hello[0].value in moments.MISSION_HELLO["PATROL"] and hello[0].style == "patrol"
    _, outs = b.update(30.0, status(mission="PATROL", patrol=dict(lap=2)))
    assert says(outs, 1)                                               # a lap: "all secure"
    # a fault is said plainly, not in the persona's voice
    _, outs = b.update(31.0, status(mission="PATROL", mcu=dict(mode="AUTO", faults=[dict(name="BUMPER")],
                                                                battery_pct=80, age=0.05)))
    assert [o.style for o in outs if o.kind == "say" and o.prio == 3] == [""]
    st, _ = b.update(40.0, status(mission="EXPLORE"))
    assert st.persona == "explore" and st.mood == "eager"
    tidy = lambda n: status(mission="TIDY", arm=dict(tidy_on=True, tidy=dict(state="PICK", picked=n)))  # noqa: E731
    st, _ = b.update(50.0, tidy(0))
    assert st.persona == "tidy" and st.mood == "cheerful" and st.label == "Tidying up"
    st, outs = b.update(60.0, tidy(1))
    assert "tada" in plays(outs) and says(outs, 1) and st.mood == "happy"
    st, _ = b.update(70.0, status())
    assert st.persona == "" and st.mood == "idle"                      # itself again
    # patrol's antics are its own: scans and looks, no giggles
    b = Brain(BrainConfig(antics="lively", antic_first=1.0), seed=4)
    acts, _ = run_rest(b, 0.0, 400.0, lambda t: status(mission="PATROL"), lambda t: (0.4, 0.0, 0.0))
    assert acts and {a for a, _, _ in acts} <= {"scan", "look", "think", "quip", "smile", "wheee", "dizzy"}
    assert "scan" in {a for a, _, _ in acts}


def test_every_mode_says_how_it_is_going():
    """Fernando (Oct 26): phrases after each goal, at checkpoints, when Follow loses you; progress when useful."""
    def run(b, t, **kw):
        return says(b.update(t, status(**kw))[1])

    def said_in(lines, out, **fields):              # out holds one of these lines (its fields filled in)
        return any(s.format(**{k: fields.get(k, "") for k in re.findall(r"{(\w+)}", s)}) in out for s in lines)

    import re
    G = moments.MISSION_SAY["GOTO"]
    b = Brain(QUIET)
    b.update(0.0, status())
    goal = dict(x=3.0, y=1.0, yaw=0.0, name="kitchen")
    out = run(b, 1.0, mission="GOTO", goal=goal, progress=0.0, task="Going to kitchen: 6.0 m left")
    assert said_in(G["start"], out, place="kitchen")                    # "yes, my master!", "onward to the kitchen!"
    assert said_in(G["half"], run(b, 5.0, mission="GOTO", goal=goal, progress=0.5, task="Going to kitchen: 3.0 m left"),
                   place="kitchen")
    assert not run(b, 6.0, mission="GOTO", goal=goal, progress=0.6, task="Going to kitchen: 2.6 m left")
    assert said_in(G["near"], run(b, 9.0, mission="GOTO", goal=goal, progress=0.85, task="Going to kitchen: 1.0 m left"),
                   place="kitchen")
    assert not run(b, 10.0, mission="GOTO", goal=goal, progress=0.95, task="Going to kitchen: 0.3 m left")
    out = run(b, 12.0, mission="MANUAL", task="Arrived at kitchen")
    assert said_in(G["arrived"], out, place="kitchen")
    b.update(20.0, status(mission="GOTO", goal=dict(goal, x=9.0), progress=0.0, task="Going to kitchen: 9.0 m left"))
    assert said_in(G["failed"], run(b, 25.0, mission="MANUAL", task="Could not reach kitchen: Nav2 gave up"),
                   place="kitchen")
    # patrol: a checkpoint, a lap, the end
    P = moments.MISSION_SAY["PATROL"]
    patrol = lambda i, lap=1: dict(mission="PATROL", patrol=dict(route="house", i=i, n=3, lap=lap, laps=0))  # noqa: E731
    run(b, 30.0, **patrol(0))
    assert said_in(P["point"], run(b, 40.0, **patrol(1)), i=1, n=3)
    assert said_in(P["lap"], run(b, 90.0, **patrol(0, 2)), lap=1)
    assert said_in(P["done"], run(b, 95.0, mission="MANUAL", task="Patrol 'house' finished"))
    # explore: each 25 m², the way back, the end with the area
    E = moments.MISSION_SAY["EXPLORE"]
    ex = lambda m2, phase="explore": dict(mission="EXPLORE", explore=dict(phase=phase, info=dict(known_m2=m2)))  # noqa: E731
    run(b, 100.0, **ex(10.0))
    assert not run(b, 105.0, **ex(20.0))
    assert said_in(E["mapped"], run(b, 110.0, **ex(31.0)), m2=25)
    assert said_in(E["home"], run(b, 120.0, **ex(64.0, "home")), m2=64)
    assert said_in(E["done"], run(b, 130.0, mission="MANUAL", task="Exploring done: back at the start, 64 m² mapped"),
                   m2=64)
    # follow: found, outrun, lost; tag
    F = moments.MISSION_SAY["FOLLOW"]
    fo = lambda ph, person=None, mode="follow": dict(mission="FOLLOW", follow=dict(mode=mode, phase=ph, person=person,  # noqa: E731
                                                                                text=""))
    run(b, 140.0, **fo("look"))
    assert said_in(F["found"], run(b, 141.0, **fo("follow", [1.2, 0.0])))
    assert not run(b, 142.0, **fo("follow", [3.0, 0.1]))
    assert said_in(F["far"], run(b, 144.0, **fo("follow", [3.2, 0.1])))
    assert said_in(F["lost"], run(b, 150.0, **fo("look")))
    assert said_in(F["chase"], run(b, 160.0, **fo("chase", [1.0, 0.0], "tag")))
    assert said_in(F["tagged"], run(b, 170.0, **fo("tagged", [0.4, 0.0], "tag")))
    # program: its steps, the end; tidy: the end with the count
    R = moments.MISSION_SAY["PROGRAM"]
    pg = lambda i: dict(mission="PROGRAM", program=dict(name="square", i=i, n=4, label=""))  # noqa: E731
    run(b, 200.0, **pg(0))
    assert said_in(R["step"], run(b, 205.0, **pg(1)), i=2, n=4)
    assert said_in(R["done"], run(b, 230.0, mission="MANUAL", task="Program 'square' finished"))
    tidy = lambda n: dict(mission="TIDY", arm=dict(tidy_on=True, tidy=dict(state="PICK", picked=n)))  # noqa: E731
    run(b, 240.0, **tidy(0))
    run(b, 250.0, **tidy(3))
    assert said_in(moments.MISSION_SAY["TIDY"]["done"], run(b, 260.0, mission="MANUAL", task="Tidy done: 3 in the bin"),
                   picked=3)
    # "be quiet": none of it (the end of a mission neither)
    b.answer(300.0, commands.parse("be quiet", woke=True))
    b.update(300.1, status())
    run(b, 301.0, mission="GOTO", goal=goal, progress=0.0, task="Going to kitchen: 6.0 m left")
    assert not run(b, 305.0, mission="GOTO", goal=goal, progress=0.5, task="Going to kitchen: 3.0 m left")
    assert not run(b, 310.0, mission="MANUAL", task="Arrived at kitchen")


def test_the_voice_is_raised_for_cuteness_and_styled_per_persona(tmp_path, monkeypatch):
    from omni_face import voice
    monkeypatch.setattr(voice, "CACHE", tmp_path)

    class Tone(voice.Backend):                         # a stand-in voice: one second of 200 Hz at 24 kHz
        name = key = "tone"
        rates = []

        def synth(self, text, out, rate=1.0):
            self.rates.append(rate)
            voice.write_wav(out, 0.5 * np.sin(2 * np.pi * 200 * np.arange(24000) / 24000), 24000)
            return True

    v = voice.Voice(Tone(), pitch=1.12)
    plain, patrol = v.wav("Hello"), v.wav("Hello", "patrol")
    assert plain != patrol and v.wav("Hello") == plain                 # one WAV per style, cached
    a, r = voice.read_wav(plain)
    assert r == voice.OUT_RATE and len(a) / r == pytest.approx(1 / 1.12, rel=0.01)   # higher, so shorter
    a, _ = voice.read_wav(patrol)
    assert len(a) / r == pytest.approx(1 / (1.12 * voice.STYLES["patrol"][0]), rel=0.01)
    assert Tone.rates == [1.0, voice.STYLES["patrol"][1]]


@pytest.mark.skipif(__import__("omni_face.voice", fromlist=["x"]).kokoro_files() is None or
                    __import__("importlib.util").util.find_spec("kokoro_onnx") is None,
                    reason="Kokoro isn't installed (python voice.py fetch; pip install kokoro-onnx)")
def test_kokoro_speaks_a_line(tmp_path, monkeypatch):
    from omni_face import voice
    monkeypatch.setattr(voice, "CACHE", tmp_path)
    v = voice.Voice(voice.choose("kokoro"))
    p = v.wav("Throw it to me!", "tidy")
    assert v.name == "kokoro" and p is not None
    a, r = voice.read_wav(p)
    assert 0.6 < len(a) / r < 3.0 and np.abs(a).max() > 0.1


def test_the_eyes_look_where_the_ball_is_as_the_screen_sees_it():
    b = Brain()                                         # the screen faces forward (design.md 12.2)
    # the robot faces +x; a ball straight ahead is straight in front of the screen
    assert b._gaze_point((0.0, 0.0, 0.0), [3.0, 0.0, 0.5]) == pytest.approx([0.0, 0.0], abs=1e-3)
    # ahead, to the robot's left (+y): the viewer faces the robot, so it's on their right: the pupils go right
    gx, _ = b._gaze_point((0.0, 0.0, 0.0), [3.0, 1.5, 0.5])
    assert gx > 0.3
    # high up: the eyes look up; behind the robot (behind the screen): a glance to the side
    assert b._gaze_point((0.0, 0.0, 0.0), [2.0, 0.0, 2.5])[1] > 0.5
    assert abs(b._gaze_point((0.0, 0.0, 0.0), [-4.0, 0.3, 0.5])[0]) == 1.0
    # the robot turned round (yaw pi): a ball at -x is ahead again
    assert b._gaze_point((0.0, 0.0, math.pi), [-3.0, 0.0, 0.5])[0] == pytest.approx(0.0, abs=1e-3)


def test_lost_low_battery_missions_and_say_requests():
    b = Brain(QUIET)
    b.update(0.0, status())
    st, outs = b.update(1.0, status(localization=dict(level="LOST")))
    assert st.mood == "worried" and says(outs, 2)
    _, outs = b.update(2.0, status(localization=dict(level="OK")))
    assert "Found my way again." in says(outs)
    _, outs = b.update(3.0, status(mcu=dict(mode="AUTO", faults=[], battery_pct=12, battery_low=True, age=0.05)))
    assert "My battery is getting low." in says(outs)
    _, outs = b.update(4.0, status(mission="FOLLOW", follow=dict(text="Following", person=[1.5, 0.0])))
    assert says(outs, 1)
    _, outs = b.update(5.0, status(), say=["Hello Ana!"])
    assert "Hello Ana!" in says(outs, 1)
    st, _ = b.update(500.0, status())
    assert st.mood == "sleepy"


def test_the_say_box_speaks_at_game_priority_with_its_expression():
    b = Brain(QUIET)
    b.update(0.0, status())
    st, outs = b.update(1.0, status(), say=[dict(text="Excuse me, coming through!", mood="happy", act="smile")])
    assert says(outs, 2) == ["Excuse me, coming through!"]
    assert st.mood == "happy" and st.act == "smile" and st.act_s >= 2.5
    st, _ = b.update(1.5, status())
    assert st.act == "smile"                                           # it lasts while the line is said
    st, outs = b.update(5.0, status(), say=[dict(text="", act="sing")])  # a trick alone
    assert st.act == "sing" and set(plays(outs)) & set(sounds.SONGS) and not says(outs)
    # a fault ends a trick at once
    st, _ = b.update(5.5, status(mcu=dict(mode="ESTOP", faults=[dict(name="ESTOP")], battery_pct=80, age=0.05)))
    assert st.act == "" and st.card is not None


def run_rest(b, t0, t1, st_fn, vel=None, dt=0.5):
    acts, outs = [], []
    t = t0
    while t < t1:
        st, o = b.update(t, st_fn(t), vel=vel(t) if vel else None)
        if st.act and (not acts or acts[-1][1] != st.act_id):
            acts.append((st.act, st.act_id, t))
        outs += o
        t += dt
    return acts, outs


def test_antics_happen_now_and_then_and_never_during_a_throw_or_a_fault():
    b = Brain(BrainConfig(antics="lively", antic_first=5.0), seed=3)
    acts, outs = run_rest(b, 0.0, 600.0, lambda t: status())
    names = {a for a, _, _ in acts}
    assert 12 <= len(acts) <= 45                       # lively: every 15-40 s (and a quiet moment first)
    assert len(names) >= 6 and names <= set(ANTICS)
    assert any(o.kind == "sound" for o in outs) and any(o.prio == 1 for o in outs)
    # sleepy (resting a long time): naps and yawns
    sleepy = [a for a, _, t in acts if t > 200]
    assert sum(a in ("nap", "yawn", "sigh") for a in sleepy) >= len(sleepy) // 2
    # off: none
    acts, _ = run_rest(Brain(QUIET), 0.0, 300.0, lambda t: status())
    assert acts == []
    # a fault, the Thrower bench mode, a throw in flight: none
    b = Brain(BrainConfig(antics="lively", antic_first=1.0))
    bump = dict(name="BUMPER", hint="")
    for fn in (lambda t: status(mcu=dict(mode="AUTO", faults=[bump], battery_pct=80, age=0.05)),
               lambda t: status(mission="THROWER"),
               lambda t: status(mission="CATCH", catch=dict(state="flight", tosses=1, catches=0, ball=[3, 0, 1]))):
        acts, _ = run_rest(b, 0.0, 200.0, fn)
        assert acts == []
    # while waiting for a throw: only light ones
    b = Brain(BrainConfig(antics="lively", antic_first=1.0), seed=1)
    acts, _ = run_rest(b, 0.0, 300.0, lambda t: status(mission="CATCH", catch=dict(state="wait", tosses=0, catches=0)))
    assert acts and all(a in ("giggle", "wink", "look", "babble", "smile", "hum", "quip") for a, _, _ in acts)


def test_be_quiet_hushes_the_antics_and_the_chatter():
    b = Brain(BrainConfig(antics="lively", antic_first=1.0))
    b.update(0.0, status())
    b.answer(1.0, commands.parse("be quiet", woke=True))
    _, outs = b.update(1.1, status())
    assert says(outs, 2)                               # "okay, I'll be quiet"
    acts, outs = run_rest(b, 2.0, 400.0, lambda t: status(mission="CATCH", catch=dict(state="wait", tosses=0,
                                                                                     catches=0)))
    assert acts == [] and not says(outs, 1)            # no antics, no "throw it to me!"
    b.answer(400.0, commands.parse("you can talk", woke=True))
    acts, _ = run_rest(b, 400.0, 600.0, lambda t: status())
    assert acts


def test_fast_driving_says_wheee_and_a_spin_makes_it_dizzy():
    b = Brain(BrainConfig(antics="normal", antic_first=1000.0))
    drive = lambda t: (1.7, 0.0, 0.0) if t < 3 else (0.0, 0.0, 4.0) if 10 <= t < 14 else (0.0, 0.0, 0.0)  # noqa: E731
    acts, outs = run_rest(b, 0.0, 20.0, lambda t: status(mission="MANUAL", manual=dict(active=True)), drive, dt=0.1)
    assert [a for a, _, _ in acts] == ["wheee", "dizzy"]
    assert any(s in ("Wheee!", "Woohoo!", "Zoom zoom!") for s in says(outs))
    assert "woozy" in plays(outs)
    # a nap ends when the robot starts to move
    b = Brain(BrainConfig(antics="normal", antic_first=1000.0))
    b.update(0.0, status(mission="MANUAL"), vel=(0.0, 0.0, 0.0))
    b.start_act(1.0, "nap", [])
    assert b.update(2.0, status(mission="MANUAL"), vel=(0.0, 0.0, 0.0))[0].act == "nap"
    assert b.update(2.1, status(mission="MANUAL"), vel=(0.5, 0.0, 0.0))[0].act == ""


def test_voice_commands_parse_with_the_robots_name():
    places = {"kitchen": [1, 2, 0], "Living room": [3, 4, 0], "home": [0, 0, 0]}
    p = lambda t: commands.parse(t, places)           # noqa: E731
    assert p("Omni, follow me!").verb == "follow" and p("hey omni come here").args == {"mode": "follow"}
    assert p("robot stop").verb == "stop" and p("omni, please stop now").verb == "stop"
    assert p("omni go to the kitchen").args == {"place": "kitchen"}
    assert p("omni take me to the living room").args == {"place": "Living room"}
    assert p("omni go home").args == {"place": "home"} and p("omni go to sleep").name == "sleep"
    assert p("omni go to the garage").name == "unknown_place" and p("omni go to the garage").verb is None
    assert p("omni let's play catch").args == {"name": "CATCH"} and p("omni play tag").args == {"mode": "tag"}
    assert p("omni tell me a joke").name == "joke" and p("omni what's your battery").name == "battery"
    assert p("omni, can you please follow me now") .verb == "follow"      # free text, as a Whisper-like recognizer
    assert p("omni banana") is None
    assert "go to the kitchen" in commands.phrases(places) and "follow me" in commands.phrases()
    # what the face node does with each (decide)
    d = lambda t, conf=0.9, echo=False, listening=False, level="on": commands.decide(t, conf, echo, listening,  # noqa: E731
                                                                                       places, level)[0]
    assert d("follow me") == "ignore"                                  # no name: not meant for it
    assert d("follow me", listening=True) == "console"                 # ... unless it just heard its name
    assert d("omni") == "wake" and d("omni follow me") == "console" and d("omni sing") == "chat"
    assert d("omni follow me", conf=0.3) == "unknown" and d("omni banana") == "unknown"
    assert d("omni follow me", echo=True) == "ignore" and d("omni stop", echo=True) == "console"
    assert d("omni follow me", level="chat only") == "refused" and d("omni stop", level="chat only") == "console"
    assert d("omni stop", level="off") == "ignore"


def test_voice_answers_wake_and_listen():
    b = Brain(QUIET)
    b.update(0.0, status())
    b.on_wake(1.0)
    st, outs = b.update(1.1, status())
    assert st.listening and st.act == "listen" and "listen" in plays(outs)
    st, _ = b.update(8.0, status())
    assert not st.listening and st.act == ""                           # 6 s, then it stops listening
    b.on_heard(9.0, "what's your battery")
    b.answer(9.0, commands.parse("battery", woke=True))
    st, outs = b.update(9.1, status())
    assert st.heard == "what's your battery" and "My battery is at 80 percent." in says(outs, 2)
    b.answer(10.0, commands.parse("tell me a joke", woke=True))
    assert any("?" in s for s in says(b.update(10.1, status())[1]))
    i = commands.parse("follow me", woke=True)
    b.answer(11.0, i, True, "Following")
    _, outs = b.update(11.1, status())
    assert "heard" in plays(outs) and "Okay, I'll follow you!" in says(outs)
    b.answer(12.0, i, False, "Follow needs the lidar.")
    _, outs = b.update(12.1, status())
    assert "huh" in plays(outs) and any("Follow needs the lidar." in s for s in says(outs))
    b.on_unknown(13.0, "banana")
    _, outs = b.update(13.1, status())
    assert "huh" in plays(outs) and says(outs, 2)


def test_every_sound_is_short_and_loud_enough_and_the_songs_are_songs():
    for n in sounds.NAMES:
        a = sounds.sound(n, 22050)
        assert a.dtype == np.int16 and 0.05 < len(a) / 22050 < 1.5 and np.abs(a).max() > 5000, n
    for n in sounds.SONGS:
        a = sounds.song(n, 22050)
        assert 1.5 < len(a) / 22050 < 7.0 and np.abs(a).max() > 5000, n
        assert sounds.song_seconds(n) == pytest.approx(len(a) / 22050, abs=0.2)


def test_the_display_draws_every_mood_antic_card_and_the_demo_at_800x480():
    pygame = pytest.importorskip("pygame")          # (WSL's venv has none: the sim draws the face on Windows)
    pygame.init()
    pygame.display.set_mode((1, 1))
    from omni_face.display import DEMO_S, MOODS, Audio, FaceView, W, H, demo_status
    from omni_face.voice import Backend, Voice
    assert (W, H) == (800, 480)
    view, s = FaceView(), pygame.Surface((W, H))
    for mood in MOODS:
        view.draw(s, FaceState(mood=mood, gaze=[0.5, -0.2], prompt="Throw it to me!", battery=50,
                               score=dict(player="Ana", caught=3, throws=4, streak=3)), 1 / 30, 0.6, "Got it!")
    for k, act in enumerate(ANTICS):                # every antic, through its whole animation
        st = FaceState(act=act, act_id=k + 1, act_s=ANTICS[act][0], heard="follow me", listening=act == "listen")
        view.draw(s, st, 1 / 30, mic=True)
        view.t_act -= ANTICS[act][0] / 2            # halfway
        view.draw(s, st, 1 / 30, 0.5, mic=False)
        view.t_act -= ANTICS[act][0] / 2 - 0.05     # the end
        view.draw(s, st, 1 / 30)
    view.draw(s, FaceState(card=dict(level="error", title="Emergency stop", hint="Release it. " * 20)), 1 / 30)
    # each persona: its look, its colours (the background too), its antics
    from omni_face.display import PALETTES
    for name, (eye, bg, _) in PALETTES.items():
        look = next(p["look"] for p in moments.PERSONAS.values() if p["name"] == name)
        pv = FaceView()
        for i, act in enumerate(("", "scan", "whistle", "look")):
            kw = dict(mood=look, persona=name, act=act, act_id=i + 1, act_s=2.8, gaze=[0.5, 0.0])
            pv.draw(s, FaceState(label="x", **kw), 1.0, 0.0)
            pv.t_act -= 1.0
            pv.draw(s, FaceState(**kw), 1.0, 0.4)
        assert s.get_at((W // 2, 300))[:3] == bg                       # between the eyes and the mouth
        assert s.get_at((W // 2 - 225, 215))[:3] == eye, name          # the left eye (beside its pupil)
    # the eyes sit in the panel's middle: an open idle eye's centre is lit, the space between the eyes is not
    view2 = FaceView()
    view2.draw(s, FaceState(mood="idle", gaze=[0.0, 0.0]), 1.0)
    assert s.get_at((W // 2 - 165, 240))[:3] != (8, 12, 24) and s.get_at((W // 2, 240))[:3] == (8, 12, 24)
    b = Brain(BrainConfig(antics="lively", antic_first=3.0))
    for k in range(0, int(DEMO_S * 10), 5):           # the whole demo script through the brain and the picture
        status_, pose, vel = demo_status(k / 10)
        st, _ = b.update(k / 10, status_, pose, vel=vel)
        view.draw(s, st, 0.05)
    # with no voice the caption still shows, and a fault interrupts chatter
    a = Audio(Voice(Backend()))
    a.say("Throw it to me!", 1)
    t0 = time.monotonic()
    while not a.caption_now() and time.monotonic() - t0 < 2:
        a.tick()
    assert a.caption_now() == "Throw it to me!"
    a.say("Emergency stop.", 3)
    t0 = time.monotonic()
    while a.caption_now() != "Emergency stop." and time.monotonic() - t0 < 2:
        a.tick()
    assert a.caption_now() == "Emergency stop."


def test_the_launchers_face_buttons_change_only_what_they_name():
    """The sim's face window: the Launcher's size / mute / hide datagrams (UDP 9883) and its saved settings."""
    pytest.importorskip("pygame")
    from omni_face.display import SIZES, face_settings
    cur = dict(mute=False, size=None, hidden=False)
    assert face_settings({"size": "small"}, cur) == dict(mute=False, size="small", hidden=False)
    assert face_settings({"mute": True, "hidden": True}, cur) == dict(mute=True, size=None, hidden=True)
    assert face_settings({"size": "huge", "mute": "yes", "x": 1}, cur) == cur          # unknown values: no change
    assert face_settings([1, 2], cur) == cur and set(SIZES) == {"small", "normal", "large"}


@pytest.mark.skipif(sys.platform != "win32", reason="Windows' recognizer (the sim's listening)")
def test_windows_hears_a_spoken_command_from_a_wav(tmp_path):
    from omni_face.listen import Sapi
    from omni_face.voice import Sapi as SapiVoice
    wav = tmp_path / "omni_follow.wav"
    v = SapiVoice()
    if not v.synth("Hey omni, follow me.", wav):
        pytest.skip("no Windows voice")
    heard = Sapi().recognize_wav(commands.phrases({"kitchen": [0, 0, 0]}), wav)
    assert heard, "nothing recognized"
    text, conf = heard[0]
    action, rest, intent = commands.decide(text, conf, False, False, {"kitchen": [0, 0, 0]})
    assert action == "console" and intent.verb == "follow", (text, conf)
