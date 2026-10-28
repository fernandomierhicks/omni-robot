"""omni_console phase 2 cores: frontier exploration, Explore / Patrol / Thrower missions, SLAM requests, choice
settings, the firmware build id. Run from the repo root:  & $py -m pytest -q ros/src/omni_console/test
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_console import status as st                                      # noqa: E402
from omni_console.explore import ExploreParams, Grid, frontier_goal, still_open   # noqa: E402
from omni_console.missions import Console                                   # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))


def room(known_cols=60, rows=60, cols=100, res=0.05):
    """A 3 x 5 m room with walls; the left known_cols columns seen, the rest unknown."""
    d = np.full((rows, cols), -1, np.int8)
    d[:, :known_cols] = 0
    d[0, :known_cols] = d[-1, :known_cols] = d[:, 0] = 100
    return Grid(d, res, 0.0, 0.0)


def ready(**caps) -> Console:
    c = Console()
    c.on_robot_mode("TELEOP adult")
    c.set_caps(**{"nav": True, "catch": True, "collision": False, **caps})
    return c


def kinds(fx):
    return [f[0] for f in fx]


def test_frontier_is_the_edge_of_the_known_part():
    g = room()
    goal, info = frontier_goal(g, (1.0, 1.5, 0.0), (1.0, 1.5), ExploreParams())
    assert goal is not None and info["frontiers"] == 1
    assert 2.6 < goal[0] < 3.0 and 0.3 < goal[1] < 2.7          # just inside the seen part, facing the unknown
    assert abs(goal[2]) < 0.3
    assert still_open(g, goal)


def test_no_frontier_when_all_known_or_out_of_range():
    g = room(known_cols=100)
    g.data[:, -1] = 100                                             # closed room
    assert frontier_goal(g, (1.0, 1.5, 0.0), (1.0, 1.5), ExploreParams())[0] is None
    g2 = room()
    assert frontier_goal(g2, (1.0, 1.5, 0.0), (1.0, 1.5), ExploreParams(radius=1.0))[0] is None
    goal, _ = frontier_goal(g2, (1.0, 1.5, 0.0), (1.0, 1.5), ExploreParams())
    assert frontier_goal(g2, (1.0, 1.5, 0.0), (1.0, 1.5), ExploreParams(), blacklist=[goal[:2]])[0] is None


def test_explore_mission_goes_to_frontiers_then_home():
    c = ready()
    assert not c.command("explore").ok                              # SLAM isn't mapping
    c.set_slam(mode="mapping", managed=True, map=None, maps=[])
    r = c.command("explore")
    assert r.ok and c.mission == "EXPLORE"
    c.pose = ("map", 1.0, 1.5, 0.0)
    g = room()
    fx = c.tick(1.0, g)                                             # start pose taken, first frontier sent
    assert kinds(fx) == ["nav_limits", "nav_goal"] and c.explore["start"] == (1.0, 1.5, 0.0)
    assert c.tick(1.5, g) == ()                                     # driving there
    r = c.command("set", {"key": "goto.facing", "value": "spinning"})
    assert r.effects[0][1] == c.values["explore.speed"] and r.effects[0][3] == "forward"   # a spinning lidar maps worse
    seq = c.goal["seq"]
    c.on_nav_done(seq, "succeeded", 10.0)
    full = room(known_cols=100)
    full.data[:, -1] = 100
    fx = c.tick(12.0, full)                                         # nothing left: back to the start
    assert kinds(fx)[-1] == "nav_goal" and c.goal["name"] == "start"
    c.on_nav_done(c.goal["seq"], "succeeded", 20.0)
    assert c.mission == "MANUAL" and "done" in c.task


def test_explore_skips_a_failed_opening():
    c = ready()
    c.set_slam(mode="mapping", managed=True)
    c.command("explore")
    c.pose = ("map", 1.0, 1.5, 0.0)
    c.tick(1.0, room())
    c.on_nav_done(c.goal["seq"], "aborted", 2.0)
    assert len(c.explore["blacklist"]) == 1 and c.mission == "EXPLORE"


def test_new_map_and_explore_waits_for_the_map():
    c = ready()
    c.set_slam(mode="localization", managed=True, map="house", maps=["house"])
    r = c.command("explore", {"new_map": True})
    assert r.ok and ("slam_start", "mapping", None, None) in r.effects
    assert c.tick(1.0, None) == () and c.explore["phase"] == "wait"
    c.set_slam(mode="mapping", busy=False)
    c.pose = ("map", 0.0, 0.0, 0.0)
    c.tick(2.0, room())
    assert c.explore["phase"] == "explore"


def test_patrol_loop_once_and_back_and_forth():
    c = ready()
    pts = [[0, 0, 0], [1, 0, 0], [2, 0, 0]]
    assert c.command("route_save", {"name": "hall", "points": pts}).ok
    assert not c.command("route_save", {"name": "x", "points": [[0, 0]]}).ok
    order = []

    def run(n):
        for k in range(n):
            fx = c.tick(100.0 * k)
            if c.mission != "PATROL":
                return
            assert kinds(fx)[-1] == "nav_goal"
            order.append(c.goal["name"])
            c.on_nav_done(c.goal["seq"], "succeeded", 100.0 * k)
    c.command("set", {"key": "patrol.laps", "value": 2})
    c.command("patrol", {"route": "hall"})
    run(20)
    assert order == ["point 1", "point 2", "point 3"] * 2 and c.task.endswith("finished")
    order.clear()
    c.command("set", {"key": "patrol.mode", "value": "once"})
    c.command("patrol", {"route": "hall"})
    run(20)
    assert order == ["point 1", "point 2", "point 3"]
    order.clear()
    c.command("set", {"key": "patrol.mode", "value": "back and forth"})
    c.command("set", {"key": "patrol.laps", "value": 1})
    c.command("patrol", {"route": "hall"})
    run(20)
    assert order == ["point 1", "point 2", "point 3", "point 2", "point 1"]
    assert not c.command("set", {"key": "patrol.mode", "value": "zigzag"}).ok


def test_patrol_waits_at_each_point_and_gives_up_after_three_failures():
    c = ready()
    c.command("route_save", {"name": "r", "points": [[0, 0], [1, 0]]})
    c.command("set", {"key": "patrol.dwell", "value": 5})
    c.command("patrol", {"route": "r"})
    c.tick(0.0)
    c.on_nav_done(c.goal["seq"], "succeeded", 1.0)
    assert c.tick(3.0) == () and kinds(c.tick(6.5))[-1] == "nav_goal"
    for k in range(3):
        c.on_nav_done(c.goal["seq"], "aborted", 10.0 + k)
        if c.mission == "PATROL":
            c.tick(20.0 + k)
    assert c.mission == "MANUAL" and "unreachable" in c.task


def test_thrower_bench_mode():
    c = ready()
    assert not c.command("fire").ok and not c.command("thrower", {"rpm": 1000}).ok
    r = c.command("mission", {"name": "THROWER"})
    assert r.ok and ("set_mode", "THROWER", False) in r.effects
    assert c.thrower_cmd() == (0.0, 0.0, 0)
    assert c.command("thrower", {"rpm": 3000, "tilt": 20}).ok
    assert not c.command("thrower", {"rpm": 9000}).ok and not c.command("thrower", {"tilt": 80}).ok
    # +20 deg tilt launches at 10 deg: into the face (design.md 12.2). The flattest fired is 25 deg (tilt +5)
    r = c.command("fire")
    assert not r.ok and "25" in r.message and c.thrower_cmd()[2] == 0
    assert c.summary()["thrower"]["fire_tilt_max"] == pytest.approx(5.0)
    assert c.command("thrower", {"tilt": 5}).ok and c.command("fire").ok
    c.command("thrower", {"tilt": 0})
    c.command("fire")
    tilt, rpm, seq = c.thrower_cmd()
    assert tilt == pytest.approx(0.0) and rpm == 3000 and seq == 2
    c.command("stop")
    assert c.thrower_cmd() is None


def test_say_box_tricks_and_typed_voice_commands():
    c = ready()
    r = c.command("say", {"text": "Excuse me,\ncoming through!", "expression": "happy"})
    assert r.ok and r.effects == (("face", dict(text="Excuse me, coming through!", mood="happy", act="smile")),)
    assert c.command("say", {"act": "sing"}).effects == (("face", dict(text="", mood=None, act="sing")),)
    assert not c.command("say", {"text": ""}).ok and not c.command("say", {"text": "x" * 201}).ok
    assert not c.command("say", {"text": "hi", "expression": "angry"}).ok
    assert not c.command("say", {"act": "explode"}).ok
    assert c.command("hear", {"text": "omni follow me"}).effects == (("face_hear", "omni follow me"),)
    keys = {s["key"]: s for s in c.schema()["settings"]}
    assert keys["face.voice"]["value"] == "on" and keys["face.antics"]["options"] == ["off", "quiet", "normal", "lively"]
    assert c.command("set", {"key": "face.voice", "value": "chat only"}).ok


def test_tidy_mission_drives_the_arm_robot():
    """design.md 11: Tidy needs omni_arm's arm_node; it switches the mode manager to PICK and asks the arm to tidy;
    the arm ending the tidy ends the mission; any other mode stops the arm first."""
    c = ready()
    c.now = 100.0
    assert not c.command("tidy").ok and not c.command("arm", {"do": "pose", "name": "stow"}).ok      # no arm yet
    arm = dict(connected=True, torque=True, source="hold", tidy=None, tidy_on=False,
               events=[dict(n=1, t=0.0, text="arm connected")])
    assert c.on_arm_state(arm, 100.0) == ()
    r = c.command("tidy")
    assert r.ok and c.mission == "TIDY" and ("set_mode", "PICK", False) in r.effects
    req = [f[1] for f in r.effects if f[0] == "arm"][0]
    assert req["verb"] == "tidy" and req["max_objects"] == 20 and req["radius"] == 2.0
    assert not c.command("arm", {"do": "relax"}).ok                                    # the tidy owns the arm
    c.on_robot_mode("PICK adult")
    running = dict(arm, tidy_on=True, tidy=dict(state="HOVER", picked=1, failed=0),
                   events=[dict(n=2, t=1.0, text="tidy: over object 3")])
    c.on_arm_state(running, 101.0)
    assert "1 in the bin" in c.task and any("over object 3" in t for _, t in c.pop_events())
    fx = c.on_arm_state(dict(running, tidy_on=False, tidy=dict(state="DONE", picked=3, failed=1,
                                                               reason="nothing in sight")), 102.0)
    assert c.mission == "MANUAL" and ("set_mode", "TELEOP", False) in fx and "3 in the bin" in c.task
    c.command("tidy")
    r = c.command("stop")
    assert ("arm", {"verb": "stop"}) in r.effects and c.mission == "IDLE"
    assert c.command("arm", {"do": "gripper", "gap_mm": 40}).effects == (("arm", {"verb": "gripper", "gap": 0.04}),)
    assert c.command("arm", {"do": "pose", "name": "drop"}).ok and not c.command("arm", {"do": "pose", "name": "x"}).ok
    # one joint by hand: a joint the arm reports, up to 30 deg at a time
    c.on_arm_state(dict(arm, names=["shoulder_pan", "gripper"], joints=[0.0, 5.0]), 103.0)
    (kind, req), = c.command("arm", {"do": "jog", "joint": "shoulder_pan", "deg": -5}).effects
    assert kind == "arm" and req["verb"] == "jog" and req["joint"] == "shoulder_pan"
    assert abs(req["delta"] + math.radians(5)) < 1e-12
    assert not c.command("arm", {"do": "jog", "joint": "elbow", "deg": 5}).ok
    assert not c.command("arm", {"do": "jog", "joint": "gripper", "deg": 45}).ok
    assert not c.command("arm", {"do": "jog", "joint": "gripper", "deg": 0}).ok
    c.now = 110.0
    assert not c.command("tidy").ok                                                   # /arm/state went quiet


def test_slam_requests():
    c = ready()
    assert not c.command("slam_new").ok                             # the launch runs SLAM, not the console
    c.set_slam(mode="localization", managed=True, map="house", maps=["house", "flat"])
    r = c.command("slam_new")
    assert r.ok and r.effects[-1] == ("slam_start", "mapping", None, None)
    c.set_slam(mode="mapping", busy=False)
    assert not c.command("slam_save", {"name": "bad name!"}).ok
    assert c.command("slam_save", {"name": "flat2"}).effects == (("slam_save", "flat2"),)
    assert not c.command("initial_pose", {"x": 1, "y": 2}).ok      # only when localizing
    c.pose = ("map", 1.0, 2.0, 0.5)
    r = c.command("slam_load", {"name": "flat", "here": True})
    assert r.effects[-1] == ("slam_start", "localization", "flat", [1.0, 2.0, 0.5])
    assert not c.command("slam_load", {"name": "nope"}).ok
    c.set_slam(mode="localization", map="flat", busy=False)
    assert not c.command("slam_save", {"name": "flat3"}).ok          # slam_toolbox can't save while localizing
    assert c.command("initial_pose", {"x": 1, "y": 2, "yaw": 0}).effects == (("initial_pose", 1.0, 2.0, 0.0),)


def test_changing_maps_stops_a_nav_mission():
    c = ready()
    c.set_slam(mode="localization", managed=True, map="house", maps=["house"])
    c.command("goto", {"x": 1, "y": 1})
    r = c.command("slam_new")
    assert c.mission == "MANUAL" and "nav_cancel" in kinds(r.effects)


def test_firmware_build_id_matches_the_firmware_script():
    sys.path.insert(0, os.path.join(REPO, "firmware"))
    from fw_build_id import build_id
    core = os.path.join(REPO, "firmware", "lib", "omni_fw")
    assert st.firmware_build_id(core) == build_id(core)
    assert st.firmware_build_id(os.path.join(REPO, "nope")) is None


def test_slam_files(tmp_path):
    from omni_console.slam import localization_params, saved_maps
    for n in ("house", "flat"):
        (tmp_path / f"{n}.posegraph").write_text("x")
        (tmp_path / f"{n}.data").write_text("x")
    (tmp_path / "half.posegraph").write_text("x")                  # no .data: not loadable
    assert saved_maps(tmp_path) == ["flat", "house"]
    text = localization_params("slam_toolbox:\n  ros__parameters:\n    mode: localization\n", "/m/house", (1, 2, 0.5))
    assert text.endswith("    map_file_name: /m/house\n    map_start_pose: [1.000, 2.000, 0.5000]\n")


def test_map_image(tmp_path):
    from omni_console.slam import write_map_image
    data = [-1, 0, 100, 0, 0, 0]                                     # 3 x 2, row 0 at the origin
    write_map_image(tmp_path / "m", 3, 2, 0.05, (1.0, -2.0, 0.0), data)
    raw = (tmp_path / "m.pgm").read_bytes()
    assert raw.startswith(b"P5\n") and raw.endswith(bytes([254, 254, 254, 205, 254, 0]))   # top row = grid row 1
    y = (tmp_path / "m.yaml").read_text()
    assert "image: m.pgm" in y and "resolution: 0.0500" in y and "origin: [1.0000, -2.0000, 0.0000]" in y
