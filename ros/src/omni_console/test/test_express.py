"""Its feelings in the body, ROS-free (missions.py _express, tempo; gestures.py): a wheel gesture plays only while the
base is free, ends where it began and gives way to anything; arm gestures only while the arm is free; the mood's
pace touches navigation only when emotion.nav is on, and never speeds it up. Run from the repo root:
  & $py -m pytest -q ros/src/omni_console/test/test_express.py
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_console.gestures import GESTURES, MAX_REACH, MAX_TURN_DEG, Playing, size_for   # noqa: E402
from omni_console.missions import Console                                                # noqa: E402


def ready(t=100.0) -> Console:
    c = Console()
    c.now = t
    c.on_robot_mode("TELEOP adult")
    c.set_caps(nav=True, catch=False, collision=False)
    c.on_arm_state(dict(connected=True, tidy_on=False, source="hold", events=[]), t)
    c.pose = ("map", 0.0, 0.0, 0.0)
    c.odom_pose = (0.0, 0.0, 0.0)
    return c


def manual(c):
    assert c.command("mission", {"name": "MANUAL"}).ok


def run(c, t0, seconds, dt=0.05):
    """drive_output every dt from t0: the twists sent (None: nothing)."""
    out, t = [], t0
    while t < t0 + seconds:
        out.append(c.drive_output(t))
        t += dt
    return out, t


@pytest.mark.parametrize("name", sorted(GESTURES))
def test_every_gesture_ends_where_it_began_within_its_bounds(name):
    p, dt = Playing(name, 1.0, 0.0), 0.002
    x = y = th = 0.0
    far = turn = 0.0
    t = 0.0
    while not p.done(t):
        vx, vy, wz = p.twist(t)
        x, y, th = x + vx * dt, y + vy * dt, th + wz * dt
        far, turn = max(far, math.hypot(x, y)), max(turn, abs(th))
        t += dt
    assert math.hypot(x, y) < 2e-3 and abs(th) < math.radians(0.5), name        # back where it started
    assert far <= MAX_REACH + 2e-3 and math.degrees(turn) <= MAX_TURN_DEG + 0.5, name


def test_size_comes_from_the_level_and_the_moment():
    assert size_for(0.0, 1.0) == 0.0 and size_for(10.0, 1.0) == 1.0
    assert size_for(3.0, 0.5) == pytest.approx(0.3 * 0.8)


def test_praise_wiggles_the_wheels_only_while_it_rests_in_drive_with_nobody_at_the_stick():
    c = ready()
    assert c.command("mission", {"name": "IDLE"}).ok
    r = c.command("express", {"emotion": "happy", "intensity": 0.8, "base": "wiggle", "by": "face"})
    assert not r.ok and "motors are off" in r.message                           # IDLE: the motors are off
    manual(c)
    c.manual_input(0.3, 0.0, 0.0, False, 100.0)
    c.now = 100.5
    assert not c.command("express", {"emotion": "happy", "base": "wiggle"}).ok    # someone is driving
    run(c, 100.5, 3.0)                                                          # they let go: 2 s later it's free
    c.now = 103.6
    r = c.command("express", {"emotion": "happy", "intensity": 0.8, "base": "wiggle"})
    assert r.ok and c.gesture is not None and c.summary()["emotion"]["gesture"] == "wiggle"
    tws, t = run(c, 103.6, 3.0)
    turns = [tw[2] for tw in tws if tw is not None]
    assert max(turns) > 0.1 and min(turns) < -0.1                               # it wiggled
    assert c.gesture is None and tws[-1] is None                                # over, then silent
    assert not c.command("express", {"emotion": "happy", "base": "wiggle"}).ok    # one at a time (GAP)


def test_the_stick_or_a_mission_ends_a_gesture_at_once():
    c = ready()
    manual(c)
    c.now = 110.0
    assert c.command("express", {"emotion": "excited", "intensity": 1.0, "base": "shimmy"}).ok
    c.drive_output(110.1)
    c.manual_input(0.0, 0.0, 0.0, False, 110.2)                                 # the stick is touched
    tw = c.drive_output(110.25)
    assert c.gesture is None and tw == (0.0, 0.0, 0.0)                          # the stick's own command wins


def test_levels_and_the_switch_refuse_the_body():
    c = ready()
    manual(c)
    c.now = 110.0
    c.command("set", {"key": "emotion.base", "value": 0})
    assert not c.command("express", {"emotion": "happy", "base": "wiggle"}).ok
    c.command("set", {"key": "emotion.base", "value": 5})
    c.command("set", {"key": "emotion.on", "value": False})
    r = c.command("express", {"emotion": "happy", "base": "wiggle"})
    assert not r.ok and "off" in r.message


def test_an_arm_gesture_goes_to_the_arm_only_while_it_is_free():
    c = ready()
    manual(c)
    c.now = 110.0
    c.on_arm_state(dict(connected=True, tidy_on=False, source="hold", events=[]), 110.0)
    r = c.command("express", {"emotion": "curious", "intensity": 1.0, "arm": "tilt"})
    req = [f[1] for f in r.effects if f[0] == "arm"]
    assert r.ok and req and req[0]["verb"] == "express" and req[0]["gesture"] == "tilt"
    assert req[0]["size"] == pytest.approx(0.3)                                 # level 3 at full intensity
    c.on_arm_state(dict(connected=True, tidy_on=True, source="task", events=[]), 110.5)
    r = c.command("express", {"emotion": "curious", "arm": "tilt"})
    assert not r.ok and not [f for f in r.effects if f[0] == "arm"]


def test_say_can_carry_a_feeling_for_the_face():
    c = ready()
    r = c.command("say", {"feel": "curious", "intensity": 0.7})
    face = [f[1] for f in r.effects if f[0] == "face"]
    assert r.ok and face[0]["feel"] == "curious" and face[0]["intensity"] == 0.7
    assert not c.command("say", {"feel": "hangry"}).ok


def test_the_mood_sets_the_pace_only_with_the_switch_on_and_never_faster():
    c = ready()
    c.now = 120.0
    c.on_affect(dict(emotion="tired", tempo=0.85), 120.0)
    assert c._nav_limits(1.0, 90.0)[1] == pytest.approx(0.85)                   # tired: 15 % slower
    c.command("set", {"key": "emotion.nav", "value": False})
    assert c._nav_limits(1.0, 90.0)[1] == pytest.approx(1.0)                    # off: exactly as set
    c.command("set", {"key": "emotion.nav", "value": True})
    c.on_affect(dict(emotion="excited", tempo=1.4), 120.0)
    assert c._nav_limits(1.0, 90.0)[1] == pytest.approx(1.0)                    # never faster than set
    c.on_affect(dict(emotion="tired", tempo=0.85), 120.0)
    c.now = 130.0
    assert c._nav_limits(1.0, 90.0)[1] == pytest.approx(1.0)                    # a stale feeling: as set
