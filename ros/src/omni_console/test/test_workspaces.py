"""The console's workspaces, ROS-free: the feature list, the Test workspace (preflight, its gate, the bring-up
records), the activity history, who has control and the speed cap. Run from the repo root:
  & $py -m pytest -q ros/src/omni_console/test
"""

import json
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

from omni_console import checks                                          # noqa: E402
from omni_console.console_defs import FEATURES, SETTINGS, VERB_FEATURE, WORKSPACES   # noqa: E402
from omni_console.missions import Console                                # noqa: E402

WEB = os.path.join(REPO, "ros", "src", "omni_console", "web", "js")


def ready(**caps) -> Console:
    c = Console()
    c.on_robot_mode("TELEOP adult")
    c.set_caps(**{"nav": True, "catch": False, "collision": False, **caps})
    return c


def good_status(**kw) -> dict:
    s = dict(source="robot", caps=dict(nav=True, arm=True),
             mcu=dict(mode="TELEOP", profile="adult", faults=[], v_bus=24.1, battery_pct=80, age=0.1, sync_rtt_ms=1.2),
             fw=dict(build="abc", protocol=6), rc=dict(ok=True, manual=False, ch=[0] * 8),
             localization=dict(level="GOOD", reasons=[], sigma_xy=0.04),
             arm=dict(ok=True, torque=True, temps=[35, 36, 38, 34, 33, 30], volts=12.1),
             vision=dict(images=["/d435/color/image_raw"], video_port=8081), sys=dict(load=1.2, temp_c=55),
             wifi=dict(dbm=-55, iface="wlan0", quality=60))
    s.update(kw)
    return s


FW = dict(expected="abc", protocol_host=6)


def by_id(rows):
    return {r["id"]: r for r in rows}


# -- features ------------------------------------------------------------------------------------------------------
def test_features_are_unique_and_grouped_into_known_workspaces():
    ids = [f["id"] for f in FEATURES]
    assert len(ids) == len(set(ids))
    groups = {k for k, _ in WORKSPACES}
    assert all(f["group"] in groups for f in FEATURES)
    assert set(VERB_FEATURE.values()) <= set(ids)
    caps = set(Console().caps)
    assert all(set(f["needs"]) <= caps for f in FEATURES), "a feature needs a cap the console never reports"
    # every setting's tab is a feature (or the AI's own, which omni_mcp reads): the generic page can draw them all
    assert {t for s in SETTINGS for t in s.tabs} <= set(ids) | {"ai"}


def test_the_page_has_a_page_for_every_feature():
    """A feature without one still works (the generic page), but the robot's own list should all be drawn."""
    src = open(os.path.join(WEB, "app.js"), encoding="utf-8").read()
    pages = set(re.findall(r"^\s+(\w+): \w+Tab,", src, re.M))
    missing = {f["id"] for f in FEATURES} - pages
    assert not missing, missing


def test_schema_lists_the_features_and_the_status_reports_control_and_cap():
    c = ready()
    sch = c.schema()
    assert [w["id"] for w in sch["workspaces"]][:3] == ["home", "drive", "map"]
    assert {f["id"] for f in sch["features"]} >= {"tidy", "manual", "preflight", "bringup", "settings"}
    s = c.summary()
    assert s["caps"]["arm"] is False and s["control"]["who"] == "ready" and s["cap"]["speed"] == 1.0
    json.dumps(s)


# -- preflight -----------------------------------------------------------------------------------------------------
def test_preflight_all_good():
    rows = checks.preflight(good_status(), FW)
    bad = [r for r in rows if r["state"] != "ok"]
    assert not bad, bad
    assert checks.brief(rows)["fail"] == 0


def test_preflight_fails_what_would_fail_anyway_and_blocks_it():
    rows = by_id(checks.preflight(good_status(mcu=None), FW))
    assert rows["mcu_link"]["state"] == "fail" and checks.MOTION in rows["mcu_link"]["blocks"]
    s = good_status()
    s["mcu"] = dict(s["mcu"], faults=[dict(name="FAULT_STOP", hint="")], v_bus=20.4)
    rows = by_id(checks.preflight(s, FW))
    assert rows["faults"]["state"] == "fail" and rows["battery"]["state"] == "fail"
    rows = by_id(checks.preflight(good_status(fw=dict(build="abc", protocol=5)), FW))
    assert rows["firmware"]["state"] == "fail"
    rows = by_id(checks.preflight(good_status(localization=dict(level="LOST", reasons=["no scan match"])), FW))
    assert rows["localization"]["state"] == "fail" and rows["localization"]["blocks"] == list(checks.NAV_FEATURES)
    rows = by_id(checks.preflight(good_status(arm=dict(ok=False)), FW))
    assert rows["arm"]["state"] == "fail" and "tidy" in rows["arm"]["blocks"]


def test_preflight_warns_and_never_fails_on_what_it_cannot_see():
    s = good_status(rc=dict(ok=False, manual=False), wifi=dict(dbm=-80), localization=None, arm=None, sys={})
    s["mcu"] = dict(s["mcu"], v_bus=21.8)
    rows = by_id(checks.preflight(s, None))
    assert rows["rc"]["state"] == "warn" and rows["wifi"]["state"] == "warn" and rows["battery"]["state"] == "warn"
    assert rows["localization"]["state"] == "na" and rows["arm"]["state"] == "na"
    assert all(r["state"] != "fail" for r in rows.values())
    assert all(not r["blocks"] for r in rows.values())             # only failures block
    assert by_id(checks.preflight(dict(s, source="sim"), None))["rc"]["state"] == "na"


def test_the_gate_refuses_blocked_starts_with_the_reason_but_never_stop_or_manual():
    c = ready()
    c.preflight = checks.preflight(good_status(mcu=None), FW)
    r = c.command("goto", {"x": 1, "y": 0})
    assert not r.ok and r.data["error"] == "fault" and "MCU link" in r.message and c.mission == "MANUAL"
    assert c.command("mission", {"name": "MANUAL"}).ok
    assert c.command("stop").ok and c.mission == "IDLE"
    c.preflight = checks.preflight(good_status(localization=dict(level="LOST")), FW)
    assert not c.command("explore", {}).ok
    assert c.command("say", {"text": "hello"}).ok                   # not a start
    c.preflight = checks.preflight(good_status(), FW)
    assert c.command("goto", {"x": 1, "y": 0}).ok


def test_the_gate_lets_the_arm_hold_and_relax():
    c = ready()
    c.preflight = checks.preflight(good_status(arm=dict(ok=False)), FW)
    assert c.gate("arm", {"do": "pose", "name": "ready"}) is not None
    assert c.gate("arm", {"do": "relax"}) is None and c.gate("arm", {"do": "hold"}) is None


# -- bring-up ------------------------------------------------------------------------------------------------------
def test_bringup_catalog_is_well_formed_and_its_actions_are_console_verbs():
    from omni_console.console_defs import VERBS
    assert len(checks.BRINGUP_IDS) == len(set(checks.BRINGUP_IDS))
    for ch in checks.BRINGUP:
        assert ch["title"] and ch["expect"] and ch["steps"]
        for a in ch["actions"]:
            assert a["verb"] in VERBS, a
        for w in ch["watch"]:
            assert re.match(r"^[a-z_]+(\.[a-z_]+)*$", w)


def test_bringup_record_keeps_the_outcome_with_the_config():
    c = ready()
    c.robot_config = dict(source="robot", firmware="abc")
    r = c.command("test_record", {"id": "ramped_stop", "result": "pass", "measured": {"speed": 1.0, "distance": ""},
                                  "pre": ["Kid profile on for the first runs", "made up"], "notes": "smooth"})
    assert r.ok
    rec = c.bringup["ramped_stop"][-1]
    assert rec["measured"] == {"speed": 1.0} and rec["pre"] == ["Kid profile on for the first runs"]
    assert rec["config"]["firmware"] == "abc" and rec["config"]["kid"] is False
    assert c.bringup_changed and c.bringup_new
    assert not c.command("test_record", {"id": "ramped_stop", "result": "maybe"}).ok
    assert not c.command("test_record", {"id": "nope", "result": "pass"}).ok
    assert not c.command("test_record", {"id": "ramped_stop", "result": "pass", "measured": {"x": 1}}).ok
    assert not c.command("test_record", {"id": "ramped_stop", "result": "pass", "measured": {"speed": "nan"}}).ok
    t = c.command("tests").data
    assert len(t["checks"]) == len(checks.BRINGUP) and t["results"]["ramped_stop"]
    c2 = Console(bringup=c.bringup)
    assert c2.bringup["ramped_stop"][-1]["result"] == "pass"


def test_preflight_verb():
    c = ready()
    assert c.command("preflight").data == dict(rows=[], assessed=False)
    c.preflight = checks.preflight(good_status(), FW)
    assert c.command("preflight").data["assessed"]


# -- activity ------------------------------------------------------------------------------------------------------
def test_activity_records_who_started_a_task_and_how_it_ended():
    c = ready()
    c.command("goto", {"x": 1, "y": 0, "name": "Kitchen"}, )
    seq = c.goal["seq"]
    c.on_nav_done(seq, "succeeded")
    c.command("goto", {"x": 3, "y": 0, "name": "Hall", "by": "voice"})
    c.on_nav_done(c.goal["seq"], "aborted")
    c.command("goto", {"x": 4, "y": 0, "name": "Den"})
    c.command("goto", {"x": 5, "y": 0, "name": "Bath"})
    c.command("stop")
    items = c.command("activity").data["items"]
    assert [(i["label"].split("'")[1], i["who"], i["state"]) for i in reversed(items)] == [
        ("Kitchen", "console", "done"), ("Hall", "voice", "failed"), ("Den", "console", "replaced"),
        ("Bath", "console", "stopped")]
    assert items[-1]["result"].startswith("Arrived") and items[0]["t1"] is not None
    assert c.summary()["activity_n"] == c.activity_n > 0


def test_activity_opens_for_a_mode_the_robot_came_up_in():
    c = Console()
    c.on_robot_mode("CATCH adult")
    assert c.act is not None and c.act["mission"] == "CATCH"


# -- control and speed cap -----------------------------------------------------------------------------------------
def test_control_ladder():
    c = ready()
    assert c.control(0.0)["who"] == "ready"
    c.mcu = dict(mode="ESTOP")
    assert c.control(0.0)["who"] == "estop"
    c.mcu = dict(mode="TELEOP")
    c.on_rc(True)
    assert c.control(0.0)["who"] == "rc"
    c.on_rc(False)
    c.command("goto", {"x": 1, "y": 0})
    ctl = c.control(c.now)
    assert ctl["who"] == "task" and "Nav2" in ctl["text"] and ctl["ai_hold_s"] is None    # no AI connected
    c.command("ai_lease", {"ai": "s1", "ttl": 3})
    assert c.control(c.now)["ai_lease"] and c.control(c.now)["ai_hold_s"] is not None     # a person just acted
    c.command("stop")
    assert c.control(c.now)["who"] == "idle"
    assert [x["id"] for x in ctl["ladder"]] == ["estop", "rc", "stick", "task", "ai"]


def test_speed_cap_takes_the_lowest_with_its_reason():
    c = ready()
    assert c.speed_cap() == dict(speed=1.0, reason="Drive's top speed")
    c.command("set", {"key": "profile.kid", "value": True})
    c.command("set", {"key": "manual.max_speed", "value": 1.5})
    assert c.speed_cap() == dict(speed=0.8, reason="kid profile")
    c.command("set", {"key": "profile.kid", "value": False})
    c.on_arm_state(dict(connected=True, out=True), c.now)
    assert c.speed_cap() == dict(speed=1.0, reason="arm out")
    c.slow = (0.3, "kitchen rug")
    assert c.speed_cap()["reason"] == "slow band: kitchen rug"
    c.command("stop")
    assert c.speed_cap() is None


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
