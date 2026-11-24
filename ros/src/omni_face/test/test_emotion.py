"""The robot's feelings (Nov 21): affect.py (what it feels), the levels (0-10 per channel), expression.py (how it
shows), the chirps' intonation (sounds.py) and the chat's replies and memory (chat.py, with a fake client).

    python -m pytest -q ros/src/omni_face/test/test_emotion.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from omni_face import sounds  # noqa: E402
from omni_face.affect import Affect, Levels  # noqa: E402
from omni_face.moments import Brain, BrainConfig  # noqa: E402


def status(**kw):
    s = dict(mission="MANUAL", mcu=dict(mode="TELEOP", faults=[], battery_pct=80, age=0.05))
    s.update(kw)
    return s


class Intent:                                       # a commands.Intent stand-in
    def __init__(self, name, verb=None):
        self.name, self.verb, self.args, self.reply, self.act = name, verb, {}, "", ""


def brain(levels=None, seed=3):
    b = Brain(BrainConfig(antics="off"), seed=seed)
    b.levels = levels or Levels()
    b.update(0.0, status())
    return b


def outs_of(b, now, fn):
    fn()
    return b.update(now, status())[1]


# -- what it feels ----------------------------------------------------------------------------------------------------
def test_praise_lifts_it_and_the_feeling_fades_back_to_its_mood():
    a = Affect(0.0)
    emo, k = a.feel("praise", 1.0)
    assert emo == "affection" and a.v > 0.5 and a.emotion(1.0)[0] == "affection"
    a.tick(120.0)
    assert a.emotion(120.0)[0] != "affection" and a.v < 0.5        # faded, a little happier mood left
    assert a.mv > 0.15


def test_failures_build_frustration_and_a_success_after_them_is_pride():
    a = Affect(0.0)
    ks = [a.feel("grab_failed", t)[1] for t in (1.0, 2.0, 3.0)]
    assert ks[0] < ks[1] < ks[2] and a.emotion(3.0)[0] == "frustrated"
    emo, k = a.feel("picked", 4.0)
    assert emo == "proud" and k > 0.55 and a.fails == 0


def test_a_flat_battery_makes_it_tired_and_slows_its_pace_never_speeds_it():
    a = Affect(0.0)
    a.battery(10.0)
    a.tick(900.0)
    assert a.emotion(900.0)[0] == "tired" and 0.85 <= a.tempo() < 0.97
    b = Affect(0.0)
    b.feel("excited", 1.0)
    assert b.tempo() == 1.0


# -- the levels -------------------------------------------------------------------------------------------------------
def test_levels_scale_how_often_and_how_big():
    lv = Levels(speech=0, sounds=10, face=5)
    assert lv.often("speech") == 0 and lv.often("sounds") == 2.0 and lv.often("face") == 1.0
    assert lv.size("sounds") == 1.0
    assert Levels.from_settings({"emotion.base": 7, "emotion.nav": False}).base == 7.0
    off = Levels(on=False, base=10, arm=10, nav=True)
    assert off.level("base") == 0 and off.level("arm") == 0 and not off.nav_ok()


def test_praise_makes_it_shake_its_wheels_at_high_levels_and_never_at_zero():
    def body(levels, seed):
        b = brain(levels, seed)
        b.answer(10.0, Intent("good"))
        return [json.loads(o.value) for o in b.update(10.0, status())[1] if o.kind == "express"]
    hits = [body(Levels(base=10, arm=10), s) for s in range(12)]
    assert sum(bool(h) for h in hits) >= 9                          # a big moment gets through at level 10
    assert any(h and h[0]["base"] == "wiggle" for h in hits)
    assert not any(body(Levels(base=0, arm=0), s) for s in range(12))
    assert not any(body(Levels(on=False, base=10, arm=10), s) for s in range(12))


def test_speech_zero_keeps_only_faults_and_answers():
    b = brain(Levels(speech=0))
    said = []
    for k in range(40):                                              # a tidy: every kind of chatter
        b.update(1.0 + k, status(mission="TIDY", arm=dict(tidy_on=True, tidy=dict(state="PICK", picked=k, failed=0))))
    st, outs = b.update(50.0, status(mcu=dict(mode="TELEOP", faults=[dict(name="ESTOP")], battery_pct=80, age=0.05)))
    said = [o for o in outs if o.kind == "say"]
    assert said and all(o.prio == 3 for o in said)                  # the fault, plainly
    b.answer(60.0, Intent("battery"))
    assert any(o.kind == "say" for o in b.update(60.0, status())[1])   # asked: it answers


def test_sounds_zero_is_silent_but_alerts_still_sound():
    b = brain(Levels(sounds=0))
    plays = []
    for k in range(20):
        _, outs = b.update(1.0 + k, status(mission="TIDY", arm=dict(tidy_on=True, tidy=dict(state="PICK", picked=k))))
        plays += [o for o in outs if o.kind in ("sound", "vocal")]
    assert not plays
    _, outs = b.update(30.0, status(mcu=dict(mode="TELEOP", faults=[dict(name="MOTOR_ERR")], battery_pct=80, age=0.05)))
    assert any(o.kind == "sound" and o.value == "alert" for o in outs)


def test_face_zero_starts_no_antics_and_the_face_carries_its_level():
    b = Brain(BrainConfig(antics="lively", antic_first=1.0), seed=2)
    b.levels = Levels(face=0)
    acts = set()
    for k in range(600):
        st, _ = b.update(k * 0.5, status())
        acts.add(st.act)
    assert acts == {""} and st.face_level == 0


def test_emotion_off_is_the_robot_as_before():
    b = brain(Levels(on=False))
    st, outs = b.update(5.0, status(mission="TIDY", arm=dict(tidy_on=True, tidy=dict(state="PICK", picked=0))))
    st, outs = b.update(6.0, status(mission="TIDY", arm=dict(tidy_on=True, tidy=dict(state="PICK", picked=1))))
    assert st.emotion == "" and st.mood == "happy"                  # the old moment
    assert not [o for o in outs if o.kind in ("express", "vocal")]


def test_the_arms_reflex_startles_it():
    b = brain(Levels(face=10, sounds=10))
    b.update(1.0, status(arm=dict(reflexes=0)))
    st, outs = b.update(2.0, status(arm=dict(reflexes=1)))
    assert st.emotion == "startled" and st.mood == "startled"
    assert not [o for o in outs if o.kind == "express"]             # the arm already backed off: no gesture


def test_lines_are_spoken_with_the_feeling():
    b = brain(Levels(speech=10))
    b.answer(5.0, Intent("good"))
    said = [o for o in b.update(5.0, status())[1] if o.kind == "say"]
    assert said and said[0].emotion == "affection"


def test_asked_to_feel_it_feels_and_the_consoles_list_matches():
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "omni_console"))
    from omni_console.console_defs import FEELINGS
    from omni_face.affect import EMOTIONS
    assert set(FEELINGS) == set(EMOTIONS)
    b = brain(Levels(face=10))
    st, outs = b.update(5.0, status(), say=[dict(text="I did it.", feel="proud", intensity=0.9)])
    assert st.emotion == "proud" and [o.emotion for o in outs if o.kind == "say"] == ["proud"]


# -- the chirps: a feeling's intonation --------------------------------------------------------------------------------
def _pitch_track(a, sr=22050, win=1024):
    a = a.astype(float)
    out = []
    for i in range(0, len(a) - win, win // 2):
        seg = a[i:i + win] * np.hanning(win)
        if np.abs(seg).max() < 300:
            continue
        f = np.fft.rfftfreq(win, 1 / sr)
        spec = np.abs(np.fft.rfft(seg))
        band = (f > 150) & (f < 3000)
        out.append(f[band][np.argmax(spec[band])])
    return out


def test_a_curious_chirp_rises_and_a_sad_one_falls():
    up = _pitch_track(sounds.chirp("curious"))
    down = _pitch_track(sounds.chirp("sad"))
    assert up[-1] > up[0] * 1.2 and down[-1] < down[0] * 0.8


def test_every_feeling_has_its_own_chirps_and_variants_differ():
    for e in sounds.CHIRP_EMOTIONS:
        a, b = sounds.chirp(e, variant=0), sounds.chirp(e, variant=1)
        assert a.dtype == np.int16 and 0.1 < len(a) / 22050 < 1.6
        assert len(a) != len(b) or not np.array_equal(a, b)


# -- the chat (chat.py) ---------------------------------------------------------------------------------------------
class FakeClient:
    def __init__(self, reply):
        self.reply, self.sent = reply, None
        self.beta = self
        self.messages = self

    def create(self, **kw):
        self.sent = kw
        block = type("B", (), dict(type="text", text=json.dumps(self.reply)))()
        return type("R", (), dict(stop_reason="end_turn", content=[block]))()


def test_the_chat_answers_in_character_with_a_feeling_and_remembers(tmp_path):
    pytest.importorskip("anthropic")
    from omni_face.chat import Chat, Memory
    fake = FakeClient(dict(say="Cats are fine. They don't leave socks.", emotion="content", intensity=0.4,
                           remember="Ana has a cat named Miso"))
    c = Chat(memory=Memory(tmp_path / "m.json"), client=fake)
    r = c.reply("Omni, do you like cats? My cat Miso is here.", dict(feeling="content", battery=80), now=100.0)
    assert r["say"].startswith("Cats") and r["emotion"] == "content" and r["intensity"] == 0.4
    assert fake.sent["output_config"]["format"]["type"] == "json_schema"
    assert fake.sent["model"] == "claude-haiku-5-5" and "fallbacks" not in fake.sent     # Haiku has no fallback
    assert "Battery (%): 80" in fake.sent["messages"][-1]["content"]
    c2 = Chat(memory=Memory(tmp_path / "m.json"), client=FakeClient(dict(say="Hi.", emotion="happy", intensity=1,
                                                                       remember="")))
    c2.reply("hello again", {}, now=150.0)
    msgs = c2.client.sent["messages"]
    assert msgs[0]["content"].startswith("Omni, do you like cats") and "Miso" in msgs[-1]["content"]   # remembered
