"""omni_console's ROS-free cores (missions.py, status.py). Run from the repo root, Windows or WSL:
    & $py -m pytest -q ros/src/omni_console/test
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_console import status as st                     # noqa: E402
from omni_console.missions import (MANUAL_TIMEOUT, SETTINGS, ZERO_TAIL, Console, load_json,   # noqa: E402
                                   save_json)


def ready(**caps) -> Console:
    c = Console()
    c.on_robot_mode("TELEOP adult")
    c.set_caps(**{"nav": True, "catch": True, "collision": False, **caps})
    return c


def kinds(r):
    return [fx[0] for fx in r.effects]


def test_adopts_the_robots_mode_once():
    c = Console()
    c.on_robot_mode("CATCH kid")
    assert c.mission == "CATCH" and c.values["profile.kid"] is True
    c2 = Console()
    c2.on_robot_mode("TELEOP adult")
    assert c2.mission == "MANUAL"


def test_stop_goes_idle_and_cancels_a_goal():
    c = ready()
    r = c.command("goto", {"x": 1, "y": 2, "yaw": 0.5, "name": "Kitchen"})
    assert r.ok and c.mission == "GOTO"
    assert kinds(r) == ["nav_limits", "nav_goal"]          # already TELEOP: no mode change
    r = c.command("stop")
    assert c.mission == "IDLE" and kinds(r) == ["nav_cancel", "set_mode"] and r.effects[1][1] == "IDLE"


def test_goto_sends_its_speed_and_facing_and_a_change_applies_on_the_way():
    c = ready()
    r = c.command("goto", {"x": 1, "y": 2})
    lim = r.effects[0]
    assert lim[:2] == ("nav_limits", 2.0) and lim[3] == "forward"            # the defaults: 2 m/s, facing forward
    r = c.command("set", {"key": "goto.facing", "value": "spinning"})
    assert r.ok and r.effects[0][3] == "spinning" and abs(r.effects[0][4] - math.radians(120)) < 1e-9
    assert not c.command("set", {"key": "goto.facing", "value": "moonwalk"}).ok
    assert not c.command("set", {"key": "goto.speed", "value": 2.1}).ok       # over the firmware's 2 m/s


def test_goto_needs_navigation_and_catch_needs_the_catch_code():
    c = ready(nav=False, catch=False)
    assert not c.command("goto", {"x": 0, "y": 0}).ok
    assert not c.command("mission", {"name": "CATCH"}).ok
    assert not c.command("mission", {"name": "GOTO"}).ok   # Go To starts from a goal, not a button
    assert c.mission == "MANUAL"


def test_new_goal_replaces_the_old_one_and_old_results_are_ignored():
    c = ready()
    c.command("goto", {"x": 1, "y": 0})
    first = c.goal["seq"]
    r = c.command("goto", {"x": 3, "y": 0})
    assert kinds(r)[0] == "nav_cancel"
    c.on_nav_done(first, "canceled")                       # the replaced goal's result
    assert c.mission == "GOTO"
    c.on_nav_feedback(c.goal["seq"], 3.0)
    c.on_nav_feedback(c.goal["seq"], 1.5)
    assert c.progress == pytest.approx(0.5)
    c.on_nav_done(c.goal["seq"], "succeeded")
    assert c.mission == "MANUAL" and c.task.startswith("Arrived")


def test_nav_failure_is_reported():
    c = ready()
    c.command("goto", {"x": 1, "y": 0, "name": "Hall"})
    c.pop_events()
    c.on_nav_done(c.goal["seq"], "aborted")
    ev = c.pop_events()
    assert c.mission == "MANUAL" and ev and ev[-1][0] == "warn" and "Hall" in ev[-1][1]


def test_losing_navigation_drops_the_goal():
    c = ready()
    c.command("goto", {"x": 1, "y": 0})
    c.set_caps(nav=False)
    assert c.mission == "MANUAL" and c.goal is None


def test_settings_validate_and_drive_effects():
    c = ready()
    assert not c.command("set", {"key": "manual.max_speed", "value": 9}).ok      # above 2.0
    assert not c.command("set", {"key": "nope", "value": 1}).ok
    r = c.command("set", {"key": "manual.max_speed", "value": "1.5"})
    assert r.ok and c.values["manual.max_speed"] == 1.5 and c.settings_changed
    r = c.command("set", {"key": "profile.kid", "value": True})
    assert ("set_mode", "TELEOP", True) in r.effects and ("param", "catch_behavior", "kid", True) in r.effects
    r = c.command("set", {"key": "catch.returns", "value": "off"})
    assert ("param", "catch_behavior", "return_throws", False) in r.effects
    assert all(s.key in {d["key"] for d in c.schema()["settings"]} for s in SETTINGS)


def test_bad_saved_settings_fall_back_to_defaults():
    c = Console({"manual.max_speed": 99, "manual.expo": 0.5, "unknown": 1})
    assert c.values["manual.max_speed"] == 1.0 and c.values["manual.expo"] == 0.5


def test_manual_stick_scales_times_out_and_falls_silent():
    c = ready()
    c.command("set", {"key": "manual.expo", "value": 0})
    c.manual_input(1.0, 0.0, 0.5, False, now=10.0)
    vx, vy, wz = c.manual_output(10.1)
    assert vx == pytest.approx(1.0) and vy == 0 and wz == pytest.approx(0.5 * math.radians(120))
    c.manual_input(1.0, 1.0, 0.0, True, now=11.0)          # diagonal clipped to the unit circle, boost speed
    vx, vy, _ = c.manual_output(11.0)
    assert math.hypot(vx, vy) == pytest.approx(2.0)
    assert c.manual_output(11.0 + MANUAL_TIMEOUT + 0.05) == (0.0, 0.0, 0.0)       # dead man released: zero
    assert c.manual_output(11.0 + MANUAL_TIMEOUT + ZERO_TAIL + 0.05) is None      # then silence
    assert not c.manual_active(12.0)


def test_manual_input_is_ignored_outside_manual():
    c = ready()
    c.command("stop")
    c.manual_input(1, 0, 0, False, now=1.0)
    assert c.manual_output(1.0) is None


def test_expo_keeps_full_stick_full_speed():
    c = ready()
    c.manual_input(1.0, 0.0, 0.0, False, now=0.0)
    assert c.manual_output(0.0)[0] == pytest.approx(c.values["manual.max_speed"])
    c.manual_input(0.3, 0.0, 0.0, False, now=0.0)
    assert c.manual_output(0.0)[0] < 0.3 * c.values["manual.max_speed"]


def test_mode_changed_elsewhere_moves_the_mission():
    c = ready()
    c.command("goto", {"x": 1, "y": 0})
    fx = c.on_robot_mode("IDLE adult")
    assert c.mission == "IDLE" and ("nav_cancel",) in fx


def test_own_mode_change_is_not_mistaken_for_someone_elses():
    c = ready()
    c.command("mission", {"name": "CATCH"})
    assert c.pending_mode == "CATCH"
    c.on_robot_mode("CATCH adult")
    assert c.mission == "CATCH" and c.pending_mode is None


def test_places_round_trip(tmp_path):
    c = ready()
    assert c.command("place_save", {"name": "Kitchen", "x": 1, "y": 2, "yaw": 3}).ok
    assert not c.command("place_save", {"name": "  ", "x": 0, "y": 0}).ok
    p = tmp_path / "house.places.json"
    save_json(p, c.places)
    c2 = Console(places=load_json(p))
    assert c2.places == {"Kitchen": [1.0, 2.0, 3.0]}
    assert c2.command("place_delete", {"name": "Kitchen"}).ok and not c2.places
    assert load_json(tmp_path / "missing.json") == {}


def test_unknown_verb_and_bad_args():
    c = ready()
    assert not c.command("fly").ok
    assert not c.command("goto", {"y": 1}).ok
    assert c.command("ping").data["version"]


def test_battery_estimate():
    assert st.battery_pct(29.4) == pytest.approx(100.0)
    assert st.battery_pct(21.0) == 0.0
    assert 40 < st.battery_pct(7 * 3.7) < 60
    assert st.battery_pct(0) is None


def test_mcu_summary_explains_faults():
    m = dict(mode="AUTO", profile="adult", faults=["LOW_BATT"], v_bus=22.0, dm_err=[1, 1, 1, 1], fly_rpm=[0.0, 0.0],
             tilt=0.0, ball_held=False, drive_mode=1)
    s = st.mcu_summary(m, 0.01)
    assert s["battery_low"] and s["faults"][0]["hint"] and s["mode_hint"]


def test_wireless_parsing():
    text = ("Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n"
            " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n"
            "wlan0: 0000   54.  -56.  -256        0      0      0      0      0        0\n")
    assert st.parse_wireless(text) == {"iface": "wlan0", "quality": 54.0, "dbm": -56.0}
    assert st.parse_wireless("header\nheader\n") is None
