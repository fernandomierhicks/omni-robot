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

from omni_console import checks                                          # noqa: E402
from omni_console.console_defs import FEATURES, SETTINGS, VERB_FEATURE, WORKSPACES   # noqa: E402
from omni_console.missions import Console                                # noqa: E402

WEB = os.path.join(os.path.dirname(__file__), "..", "web", "js")


def ready(**caps) -> Console:
    c = Console()
    c.on_robot_mode("TELEOP adult")
    c.set_caps(**{"nav": True, "catch": False, "collision": False, **caps})
    return c


def good_status(**kw) -> dict:
    s = dict(source="robot", caps=dict(nav=True, arm=True),
             mcu=dict(mode="TELEOP", profile="adult", faults=[], v_bus=24.1, battery_pct=80, age=0.1, sync_rtt_ms=1.2,
                      roll=0.01, pitch=-0.02, dm_err=[1, 1, 1, 1]),
             lidar=dict(hz=10.0, age=0.05),
             fw=dict(build="abc", protocol=6), rc=dict(ok=True, manual=False, ch=[0] * 8),
             localization=dict(level="GOOD", reasons=[], sigma_xy=0.04),
             arm=dict(ok=True, torque=True, temps=[35, 36, 38, 34, 33, 30], volts=12.1),
             vision=dict(images=["/d435/color/image_raw", "/d435/depth/image_rect_raw"], video_port=8081),
             sys=dict(load=1.2, temp_c=55),
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


def test_server_pages_are_the_pages_fallback_and_the_supervisor_lists_them_all():
    """`server` pages work without the robot's software (the page offers to start it on every other page): they are
    the page's own fallback list, and the supervisor (no ROS) serves the whole list before the software runs."""
    src = open(os.path.join(WEB, "features.js"), encoding="utf-8").read()
    fallback = set(re.findall(r'\bf\("(\w+)"', src.split("export const FALLBACK", 1)[1].split("];", 1)[0]))
    assert fallback == {f["id"] for f in FEATURES if f["server"]}
    from omni_console import supervisor
    assert supervisor.FEATURES is FEATURES


def test_schema_lists_the_features_and_the_status_reports_control_and_cap():
    c = ready()
    sch = c.schema()
    assert [w["id"] for w in sch["workspaces"]][:3] == ["home", "map", "payload"]
    assert {f["id"] for f in sch["features"]} >= {"tidy", "manual", "preflight", "bringup", "settings"}
    feat = {f["id"]: f for f in sch["features"]}
    assert feat["manual"]["group"] == "home" and feat["manual"]["base"] and not feat["manual"]["moves"]  # a mode
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


def test_preflight_lidar_and_imu():
    rows = by_id(checks.preflight(good_status(lidar=dict(hz=0.0, age=3.2)), FW))
    assert rows["lidar"]["state"] == "fail" and set(rows["lidar"]["blocks"]) == set(checks.LIDAR_FEATURES)
    assert by_id(checks.preflight(good_status(lidar=dict(hz=3.5, age=0.2)), FW))["lidar"]["state"] == "warn"
    # no scan at all: what needs it would fail, so it's refused (the self-test says whether the lidar itself works)
    row = by_id(checks.preflight(good_status(lidar=None), FW))["lidar"]
    assert row["state"] == "fail" and "follow" in row["blocks"]
    t = dict(t=1.7e9, ok=True, model="A2M8", health="good", hz=12.4, samples=330, detail="spun, scanned and stopped")
    row = by_id(checks.preflight(good_status(lidar=None, lidar_test=t, t=1.7e9 + 60), FW))["lidar"]
    assert row["state"] == "fail" and "The lidar itself works" in row["detail"] and "A2M8" in row["detail"]
    t = dict(t=1.7e9, ok=False, detail="no lidar serial port")
    assert "self-test failed" in by_id(checks.preflight(good_status(lidar=None, lidar_test=t), FW))["lidar"]["detail"]
    s = good_status()
    s["mcu"] = dict(s["mcu"], faults=[dict(name="IMU_SILENT", hint="")])
    rows = by_id(checks.preflight(s, FW))
    assert rows["imu"]["state"] == "fail" and rows["faults"]["state"] == "ok"        # its own row, not twice
    assert rows["imu"]["blocks"] == [checks.SELF_DRIVE]
    c = ready()
    c.preflight = checks.preflight(s, FW)                    # no heading: nothing drives itself, by hand still works
    assert not c.command("follow", {"mode": "follow"}).ok and not c.command("goto", {"x": 1, "y": 0}).ok
    assert c.command("mission", {"name": "MANUAL"}).ok and c.command("mission", {"name": "MOTORS"}).ok
    s["mcu"] = dict(s["mcu"], faults=[], pitch=0.4)
    assert by_id(checks.preflight(s, FW))["imu"]["state"] == "warn"                  # tilted 23 deg
    c = ready()
    c.preflight = checks.preflight(good_status(lidar=dict(hz=0.0, age=3.2)), FW)
    assert not c.command("follow", {"mode": "follow"}).ok and c.command("mission", {"name": "MANUAL"}).ok


def test_missing_drive_motors_refuse_driving_but_not_the_motor_test_of_the_ones_there():
    """Protocol 8: STATE dm_err 255 = the motor isn't answering. Driving needs all four, by hand too; the motor test
    runs the wheels that answer and refuses a missing one (the firmware would fault-stop on turning it)."""
    s = good_status()
    s["mcu"] = dict(s["mcu"], dm_err=[1, 255, 255, 255])
    row = by_id(checks.preflight(s, FW))["drive"]
    assert row["state"] == "fail" and row["blocks"] == [checks.BASE]
    assert "Wheels 2, 3 and 4 not answering" in row["detail"] and "turns wheel 1" in row["detail"]
    assert "No drive motor" in checks.drive_row([255] * 4)["detail"]
    assert checks.drive_row([1, 1, 1, 1])["state"] == "ok"
    c = ready()
    c.mcu = s["mcu"]
    c.preflight = checks.preflight(s, FW)
    r = c.command("mission", {"name": "MANUAL"})
    assert not r.ok and "Drive motors" in r.message and r.data["error"] == "fault"
    assert not c.command("dance", {}).ok
    assert c.command("mission", {"name": "MOTORS"}).ok and c.motors["wheels"] == [0]
    assert not c.command("motors", {"wheels": [0, 1]}).ok                 # wheel 2 isn't there
    assert c.summary()["motors"]["missing"] == [1, 2, 3]
    c.command("stop")
    c.motors["wheels"] = [2]
    c.mcu = dict(s["mcu"], dm_err=[255, 1, 1, 255])
    assert c.command("mission", {"name": "MOTORS"}).ok and c.motors["wheels"] == [2]      # kept: it's there
    c.command("stop")
    c.mcu = dict(s["mcu"], dm_err=[255] * 4)
    assert not c.command("mission", {"name": "MOTORS"}).ok


def test_motor_test_profiles_play_timed_speeds_at_their_ramp():
    """Protocol 9: motors_run plays a profile on the chosen wheels (steps, a sweep, or segments), each segment's ramp
    sent as /wheels/cmd a_brake (motors_ramp); the stick's dead man takes over; stop ends it."""
    c = ready()
    assert not c.command("motors_run", {"preset": "steps"}).ok                 # MOTORS first
    assert c.command("mission", {"name": "MOTORS"}).ok
    assert c.motors_ramp() == 20.0 and c.motors_output(0.0) is None            # nobody asks anything
    r = c.command("motors_run", {"preset": "steps", "speed": 10.0, "ramp": 1000.0, "hold": 0.5})
    assert r.ok and c.summary()["motors"]["profile"]["name"] == "steps"
    tot = c.motors["profile"]["total"]
    assert r.data["total"] == pytest.approx(tot, abs=1e-3) and len(r.data["segments"]) == 6     # for wheel_capture
    assert tot == pytest.approx(0.5 + 5 * 0.5 + (10 + 10 + 10 + 20 + 10) / 1000.0)    # holds + ramps
    out = {t: c.motors_output(t) for t in (100.0, 100.3, 100.7, 101.2, 101.7, 102.3)}
    assert out[100.0] == [0.0] * 4                                              # its first 0.5 s at rest
    assert out[100.7] == [10.0, 0.0, 0.0, 0.0] and c.motors_ramp() == 1000.0   # wheel 1 only (the chosen one)
    assert out[101.2] == [0.0] * 4 and out[101.7] == [-10.0, 0.0, 0.0, 0.0] and out[102.3][0] == 10.0
    assert c.motors_output(100.0 + tot + 0.01) == [0.0] * 4 and c.motors["profile"] is None     # done: at rest
    assert c.motors_ramp() == 20.0
    sweep = c.command("motors_run", {"preset": "sweep", "speed": 12.0})
    assert sweep.ok and c.motors_output(200.0) == [2.5, 0.0, 0.0, 0.0]
    c.manual_input(1.0, 0.0, 0.0, False, 200.1)                                 # the dead man: the stick takes over
    assert c.motors_output(200.1)[0] > 0 and c.motors["profile"] is None
    assert c.command("motors_run", {"segments": [[5.0, 100.0, 0.2], [0.0, 100.0, 0.2]]}).ok
    assert c.command("motors_run", {"stop": True}).ok and c.motors["profile"] is None
    for bad in ({"preset": "spin"}, {"segments": [[60.0, 100.0, 1.0]]}, {"segments": [[5.0, 9000.0, 1.0]]},
                {"ramp": 1.0}, {"segments": [[5.0, 100.0, 10.0]] * 7}):
        assert not c.command("motors_run", bad).ok, bad


def test_the_motor_test_is_refused_only_by_what_makes_it_impossible():
    """A bench test a person judges (wheels up): a fault stop, a low pack, a silent IMU, no lidar, missing motors don't
    refuse it (the bench, Nov 13); no MCU, other firmware and the e-stop do."""
    s = good_status(lidar=None)
    s["mcu"] = dict(s["mcu"], dm_err=[0, 255, 255, 255], v_bus=20.5,
                    faults=[dict(name=n, hint="") for n in ("FAULT_STOP", "LOW_BATT", "MOTOR_ERR", "IMU_SILENT",
                                                            "MOTOR_SILENT")])
    c = ready()
    c.mcu = s["mcu"]
    c.preflight = checks.preflight(s, FW)
    assert "motors" not in c.unavailable() and c.command("mission", {"name": "MOTORS"}).ok
    c.command("stop")
    for bad in (good_status(mcu=None), good_status(fw=dict(build="abc", protocol=5)),
                good_status(mcu=dict(good_status()["mcu"], mode="ESTOP"))):
        c.preflight = checks.preflight(bad, FW)
        assert "motors" in c.unavailable() and not c.command("mission", {"name": "MOTORS"}).ok


def test_the_robot_says_what_can_start_and_why_not():
    """checks.unavailable: {feature: why} from the caps and the failing rows; the status's "avail", what the page
    greys. A healthy robot with navigation and the arm: everything but the optional catch and the sim's tools."""
    c = ready(arm=True)
    c.on_arm_state(dict(connected=True), c.now)
    c.preflight = checks.preflight(good_status(), FW)
    av = c.unavailable()
    assert set(av) == {"catch", "thrower", "faults"}, av
    s = good_status(lidar=None)
    s["mcu"] = dict(s["mcu"], dm_err=[1, 255, 255, 255], faults=[dict(name="IMU_SILENT", hint="")])
    c.preflight = checks.preflight(s, FW)
    av = c.unavailable()
    assert {"manual", "follow", "goto", "tidy", "calibrate"} <= set(av) and "motors" not in av and "arm" not in av
    assert "base" not in av                       # the Base page (the MCU's samples) isn't the drive group
    assert not {checks.MOTION, checks.BASE, checks.SELF_DRIVE} & {f["id"] for f in FEATURES}
    assert "Drive motors" in av["manual"] and "IMU" not in av["manual"]      # by hand: only the motors stop it
    assert "IMU" in av["follow"] and "Lidar" in av["follow"]                  # every reason, in one line
    b = checks.brief(c.preflight)
    assert {r["id"]: r["state"] for r in b["rows"]}["drive"] == "fail" and "detail" in by_id(b["rows"])["drive"]
    assert "detail" not in by_id(b["rows"])["battery"]                       # details only where something's wrong
    c2 = ready(nav=False)
    c2.preflight = checks.preflight(good_status(), FW)
    r = c2.command("goto", {"x": 1, "y": 0})
    assert not r.ok and r.data["error"] == "unavailable" and "navigation" in r.message


def test_the_gate_refuses_blocked_starts_with_the_reason_but_never_stop():
    c = ready()
    c.preflight = checks.preflight(good_status(mcu=None), FW)
    r = c.command("goto", {"x": 1, "y": 0})
    assert not r.ok and r.data["error"] == "fault" and "MCU link" in r.message and c.mission == "MANUAL"
    assert not c.command("mission", {"name": "MANUAL"}).ok           # no MCU: nothing drives, by hand neither
    assert c.command("stop").ok and c.mission == "IDLE"
    s = good_status()                                                # a low pack leaves driving by hand (the charger)
    s["mcu"] = dict(s["mcu"], faults=[dict(name="LOW_BATT", hint="")], v_bus=20.6)
    c.preflight = checks.preflight(s, FW)
    assert not c.command("dance", {}).ok and c.command("mission", {"name": "MANUAL"}).ok
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
        assert ch["auto"] in (None, "lidar")


def test_lidar_self_test_spins_counts_and_stops_on_the_fake_lidar(tmp_path):
    from omni_console.rplidar import LidarSession
    s = LidarSession(fake=True, idle_s=20.0)
    path = tmp_path / checks.LIDAR_TEST_FILE
    r = s.self_test(seconds=1.0, path=str(path))
    assert r["ok"] is True and r["model"] == "A2M8" and r["hz"] > 5 and r["samples"] > 100, r
    assert s.dev is None                                             # stopped and let go: the ROS driver may want it
    assert json.loads(path.read_text())["ok"] is True and s.describe(open_=False)["self_test"] == r
    assert "passed" in checks.lidar_test_text(r)


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
