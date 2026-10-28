"""The console's new tabs, ROS-free: Program, Follow / Tag, Calibrate, Map edit (zones, edges), the throw radar and
leaderboard, Health and the session logs (omni-log/1). Run from the repo root:
  & $py -m pytest -q ros/src/omni_console/test
"""

import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "ros", "src", "omni_catch"))

from omni_console import session_log                                      # noqa: E402
from omni_console.calibration import DriveCal, launcher_fit                # noqa: E402
from omni_console.health import Health                                    # noqa: E402
from omni_console.missions import Console                                 # noqa: E402
from omni_console.people import Follower, detect                          # noqa: E402
from omni_console.program import ProgramRunner, check, flatten            # noqa: E402
from omni_console.throws import ThrowLog                                  # noqa: E402
from omni_console.zones import masks, speed_cap                           # noqa: E402


class Robot:
    """A perfect holonomic base: the body twist integrated at 20 Hz (odom frame)."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.p = [x, y, yaw]

    def step(self, tw, dt=0.05):
        if tw is None or isinstance(tw, str):
            return
        vx, vy, wz = tw
        c, s = math.cos(self.p[2]), math.sin(self.p[2])
        self.p[0] += (c * vx - s * vy) * dt
        self.p[1] += (s * vx + c * vy) * dt
        self.p[2] += wz * dt

    @property
    def pose(self):
        return tuple(self.p)


def run(runner, robot, t_max=120.0):
    t = 0.0
    while t < t_max:
        tw = runner.tick(t, robot.pose)
        if tw == "done":
            return t
        robot.step(tw)
        t += 0.05
    return None


def test_program_blocks_check_and_flatten():
    prog = check([{"do": "forward", "m": 1}, {"do": "repeat", "times": 2, "body": [{"do": "turn", "deg": 90},
                                                                                 {"do": "wiggle"}]}])
    steps = flatten(prog)
    assert len(steps) == 1 + 2 * (1 + 4) and steps[0][:3] == ("move", 1.0, 0.0)
    with pytest.raises(ValueError):
        check([{"do": "forward", "m": 9}])
    with pytest.raises(ValueError):
        check([{"do": "fly"}])
    with pytest.raises(ValueError):
        flatten(check([{"do": "repeat", "times": 20, "body": [{"do": "repeat", "times": 20, "body": [{"do": "wait"}]}]}]))


def test_program_drives_a_square_back_to_the_start():
    sq = [{"do": "repeat", "times": 4, "body": [{"do": "forward", "m": 1.0}, {"do": "turn", "deg": 90}]},
          {"do": "left", "m": 0.5}, {"do": "right", "m": 0.5}, {"do": "wait", "s": 0.5}]
    r = Robot()
    t = run(ProgramRunner("square", sq, 0.3, math.radians(60)), r)
    assert t is not None and t < 60
    assert math.hypot(r.p[0], r.p[1]) < 0.05 and abs(math.remainder(r.p[2], 2 * math.pi)) < math.radians(4)


def test_a_blocked_program_stops_and_says_why():
    p = ProgramRunner("wall", [{"do": "forward", "m": 1.0}], 0.3, 1.0)
    t = 0.0
    while p.tick(t, (0.0, 0.0, 0.0)) != "done" and t < 60:       # the robot never moves
        t += 0.05
    assert p.error and "didn't finish" in p.error and t < 20


def ring_scan(points, n=1600, noise=0.0, rng=None):
    """A 360 deg scan (angle_min 0) of circles [(x, y, radius)] and a far wall, from the origin."""
    a = np.arange(n) * 2 * np.pi / n
    r = np.full(n, 8.0)
    d = np.stack([np.cos(a), np.sin(a)], axis=1)
    for cx, cy, rad in points:
        c = np.array([cx, cy])
        b = d @ c
        disc = b * b - (c @ c - rad * rad)
        hit = (disc > 0) & (b > 0)
        r = np.where(hit, np.minimum(r, b - np.sqrt(np.where(disc > 0, disc, 0))), r)
    if rng is not None:
        r = r + rng.normal(0, noise, n)
    return r, 0.0, 2 * np.pi / n


def test_two_legs_make_a_person_and_a_mapped_chair_doesnt():
    r, a0, da = ring_scan([(1.5, 0.12, 0.06), (1.5, -0.12, 0.06), (0.0, 2.0, 0.02), (-2.0, 0.0, 0.5)])
    people = detect(r, a0, da)
    assert people and math.hypot(people[0][0] - 1.47, people[0][1]) < 0.06        # the legs' front arcs
    assert all(abs(p[1] - 2.0) > 0.3 for p in people)                            # a 4 cm pole is too thin
    assert not any(math.hypot(p[0] + 2.0, p[1]) < 0.8 for p in people)            # a 1 m drum is too wide
    chair = lambda x, y: abs(x - 1.5) < 0.3 and abs(y) < 0.3                     # noqa: E731 (on the map)
    assert not detect(r, a0, da, occupied=chair)


def test_follow_keeps_its_distance_and_tag_tags():
    rob = Robot()
    f = Follower("follow", distance=1.0, speed=0.6)
    person = np.array([2.0, 0.3])
    rng = np.random.default_rng(1)
    for k in range(400):                                   # the person walks away along x at 0.4 m/s
        t = k * 0.05
        person = person + np.array([0.4 * 0.05, 0.0]) if k > 40 else person
        c, s = math.cos(rob.p[2]), math.sin(rob.p[2])
        dx, dy = person - rob.p[:2]
        seen = [(c * dx + s * dy + rng.normal(0, 0.02), -s * dx + c * dy + rng.normal(0, 0.02))]
        rob.step(f.tick(t, seen, rob.pose))
    gap = math.hypot(*(person - rob.p[:2]))
    assert f.phase == "follow" and 0.85 < gap < 1.4
    tag = Follower("tag", speed=0.5)
    rob = Robot()
    for k in range(400):
        c, s = math.cos(rob.p[2]), math.sin(rob.p[2])
        dx, dy = np.array([2.0, 0.0]) - rob.p[:2]
        rob.step(tag.tick(k * 0.05, [(c * dx + s * dy, -s * dx + c * dy)], rob.pose))
    assert tag.tags >= 2                                    # tagged, backed off, counted, tagged again


def test_follow_waits_when_the_person_is_gone():
    f = Follower()
    assert f.tick(0.0, [(1.5, 0.0)], (0, 0, 0)) != (0.0, 0.0, 0.0)
    f.tick(1.0, [], (0, 0, 0))
    assert f.tick(3.5, [], (0, 0, 0)) == (0.0, 0.0, 0.0) and f.phase == "look" and f.target is None
    assert f.tick(4.0, [(-1.0, 0.0)], (0, 0, 0)) == (0.0, 0.0, 0.0)        # behind it: not taken


def test_odometry_calibrations_on_a_robot_whose_wheels_read_short():
    """The robot really moves 2% more than its odometry says, and turns 1% more than its heading says."""
    true = Robot()
    cal = DriveCal("odom_line", distance=2.0)
    t = 0.0
    while cal.phase != "measure" and t < 60:
        tw = cal.tick(t, (true.p[0] / 1.02, true.p[1] / 1.02, true.p[2]), dict(roll=0, pitch=0, vx=0, vy=0, wz=0))
        true.step(tw)
        t += 0.05
    res = cal.measured(true.p[0])
    assert res["scale"] == pytest.approx(1.02, abs=0.002)
    true = Robot()
    cal = DriveCal("odom_spin", turns=3)
    t = 0.0
    while cal.phase != "measure" and t < 120:
        true.step(cal.tick(t, (0, 0, true.p[2] / 1.01), dict(roll=0, pitch=0, vx=0, vy=0, wz=0)))
        t += 0.05
    past = math.degrees(true.p[2] - 6 * math.pi)
    assert cal.measured(past)["scale"] == pytest.approx(1.01, abs=0.001)


def test_imu_level_separates_the_mount_from_the_floor():
    rob = Robot()
    cal = DriveCal("imu_level")
    mount, floor = (math.radians(0.8), math.radians(-0.3)), (math.radians(0.5), math.radians(0.2))
    t = 0.0
    while cal.phase != "done" and t < 60:
        c, s = math.cos(rob.p[2]), math.sin(rob.p[2])            # the floor's slope turns with the robot
        mcu = dict(roll=mount[0] + c * floor[0] + s * floor[1], pitch=mount[1] - s * floor[0] + c * floor[1],
                   vx=0, vy=0, wz=0)
        rob.step(cal.tick(t, rob.pose, mcu))
        t += 0.05
    r = cal.result
    assert r["mount_roll_deg"] == pytest.approx(0.8, abs=0.05) and r["mount_pitch_deg"] == pytest.approx(-0.3, abs=0.05)


def test_launcher_fit_finds_a_launcher_that_needs_more_rpm():
    from omni_catch.flight import BallFlight
    from omni_catch.robot_config import RobotConfig
    rc = RobotConfig()
    b = rc.ball
    fl = BallFlight(b["mass"], b["radius"], b["drag_cd"], b["air_density"], b["air_viscosity"])
    tilt = math.radians(10)
    shots = []
    for rpm in (2500, 3500, 4500):                 # a launcher 5% weaker than the table: lands where the table says
        v = rc.speed_for(0, tilt)                  # ... (rpm / 1.05) would land
        w = rpm / 1.05 * 2 * math.pi / 60
        v = float(np.interp(w, rc.cal_w[0], rc.cal_v[0]))
        el = rc.elevation_at(v, tilt)
        ex, ez = rc.exit_point(tilt)
        hit = fl.crossing([ex, 0, ez], [v * math.cos(el), 0, v * math.sin(el)], b["radius"])
        shots.append(dict(rpm=rpm, tilt_deg=10, distance_m=float(hit[0][0])))
    res = launcher_fit(shots, rc, fl)
    assert res["rpm_scale"] == pytest.approx(1.05, abs=0.02) and len(res["rows"]) == 3


def test_zone_and_edge_masks_and_the_speed_cap():
    zones = {"kitchen": [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]]}
    edges = {"rug": dict(points=[[0.0, -1.0], [0.0, 1.0]], height_mm=12, speed=0.6, kind="rug")}
    keep, speed = masks(zones, edges, 80, 80, 0.05, -1.0, -1.5)                 # 4 x 4 m from (-1, -1.5)
    cell = lambda x, y: (int((y + 1.5) / 0.05), int((x + 1.0) / 0.05))          # noqa: E731
    assert keep[cell(1.5, 1.5)] == 100 and keep[cell(0.5, 0.5)] == 0
    assert speed[cell(0.1, 0.0)] == 60 and speed[cell(0.6, 0.0)] == 0 and speed[cell(0.0, 1.3)] == 60
    assert speed_cap(edges.values(), 0.2, 0.5)[0] == 0.6 and speed_cap(edges.values(), 0.5, 0.0)[0] is None
    # the masks stay while the map's rebuilds shift its bounds a little; a map that grows (mapping) gets new ones
    from omni_console.zones import covers
    g = (80, 80, 0.05, -1.0, -1.5)
    assert covers(g, g) and covers(g, (79, 80, 0.05, -0.98, -1.5)) and not covers(None, g)
    assert not covers(g, (100, 80, 0.05, -1.0, -1.5)) and not covers(g, (80, 80, 0.025, -1.0, -1.5))


def test_console_edges_cap_the_stick_and_zones_go_to_nav2():
    c = Console()
    c.on_robot_mode("TELEOP adult")
    c.set_slam(mode="localization", managed=True, map="house", maps=["house"])
    c.map_name = "house"
    r = c.command("edge_save", {"name": "rug", "points": [[1, -2], [1, 2]], "speed": 0.5})
    assert r.ok and ("masks",) in r.effects and "rug" in r.data["edges"]
    assert c.command("zone_save", {"name": "stairs", "points": [[3, 3], [4, 3], [4, 4]]}).ok
    assert not c.command("zone_save", {"name": "x", "points": [[0, 0], [1, 1]]}).ok
    c.command("mission", {"name": "MANUAL"})
    c.pose = ("map", 1.1, 0.0, 0.0)
    c.manual_input(1.0, 0.0, 0.0, False, 1.0)
    vx, vy, _ = c.drive_output(1.05)
    assert math.hypot(vx, vy) == pytest.approx(0.5) and c.summary()["slow"]["edge"] == "rug"
    c.pose = ("map", 3.0, 0.0, 0.0)
    c.manual_input(1.0, 0.0, 0.0, False, 1.1)
    assert c.drive_output(1.15)[0] == pytest.approx(1.0)
    assert c.extras()["zones"]["stairs"] and c.command("edge_delete", {"name": "rug"}).ok


def test_console_program_run_with_a_goto_and_the_follow_mission():
    c = Console()
    c.on_robot_mode("TELEOP adult")
    c.set_caps(nav=True, catch=False, collision=False)
    c.places["sofa"] = [2.0, 0.0, 0.0]
    assert c.command("program_save", {"name": "hi", "blocks": [{"do": "turn", "deg": 90}, {"do": "goto", "place": "sofa"}]}).ok
    assert not c.command("program_run", {"name": "nope"}).ok
    assert not c.command("program_run", {"blocks": [{"do": "goto", "place": "moon"}]}).ok
    assert c.command("program_run", {"name": "hi"}).ok and c.mission == "PROGRAM"
    rob = Robot()
    fx = []
    for k in range(200):
        c.odom_pose = rob.pose
        tw = c.drive_output(k * 0.05)
        fx += c.pop_fx()
        if any(f[0] == "nav_goal" for f in fx):
            break
        rob.step(tw)
    goal = next(f for f in fx if f[0] == "nav_goal")
    assert goal[1:3] == (2.0, 0.0) and rob.p[2] == pytest.approx(math.pi / 2, abs=math.radians(2))
    assert c.drive_output(20.0) is None                                     # Nav2 drives meanwhile
    c.on_nav_done(goal[4], "succeeded", 21.0)
    c.drive_output(21.1)
    assert c.mission == "MANUAL" and "finished" in c.task
    assert c.command("follow", {"mode": "tag"}).ok and c.mission == "FOLLOW"
    c.odom_pose = (0.0, 0.0, 0.0)
    c.on_people([(1.5, 0.0)], 30.0)
    assert c.drive_output(30.05)[0] > 0.2 and "tag" in c.summary()["follow"]["mode"]
    assert c.drive_output(31.0) == (0.0, 0.0, 0.0)                          # no scan for 0.5 s: stop
    c.command("stop")
    assert c.follower is None and c.mission == "IDLE"


def test_console_calibration_verbs():
    c = Console()
    c.on_robot_mode("TELEOP adult")
    assert not c.command("calib_start", {"proc": "odom_line"}).ok        # no odometry yet
    c.odom_pose = (0.0, 0.0, 0.0)
    assert c.command("calib_start", {"proc": "odom_line"}).ok and c.mission == "CALIBRATE"
    rob = Robot()
    c.mcu = dict(roll=0, pitch=0, vx=0, vy=0, wz=0)
    for k in range(600):
        c.odom_pose = rob.pose
        rob.step(c.drive_output(k * 0.05))
        if c.calib_run.phase == "measure":
            break
    r = c.command("calib_measure", {"value": rob.p[0] * 1.01})
    assert r.ok and c.calib_results["odom_line"][-1]["scale"] == pytest.approx(1.01, abs=0.002) and c.calib_changed
    assert not c.command("calib_shot", {"distance_m": 3}).ok              # not in the Thrower bench mode
    c.command("mission", {"name": "THROWER"})
    c.command("thrower", {"rpm": 3000, "tilt": 10})
    assert c.command("calib_shot", {"distance_m": 3.2}).ok and c.calib_shots[0]["rpm"] == 3000
    assert c.command("calib_record", {"proc": "camera_latency", "value": 47}).ok


def test_throw_log_and_the_leaderboard(tmp_path):
    log = ThrowLog(tmp_path / "throws.jsonl")
    log.set_player("Ana")
    for caught, sp in ((True, 6.0), (True, 6.5), (False, 7.5)):
        log.add(dict(caught=caught, speed=sp, apex=1.8, predicted=[0.3, 0.4, 0.5], path=[[0, 1, 2, 3]]), now=10.0)
    log.set_player("Ben")
    log.add(dict(caught=True, speed=5.0, apex=float("nan"), predicted=None))
    b = ThrowLog(tmp_path / "throws.jsonl").board()                     # read back from the file
    ana = next(p for p in b if p["player"] == "Ana")
    assert ana["throws"] == 3 and ana["caught"] == 2 and ana["best_streak"] == 2 and ana["best_kmh"] == 27.0
    assert ana["aim"] == 0.5 and b[0]["player"] == "Ana"
    assert "path" not in log.summary()["recent"][0]
    with pytest.raises(ValueError):
        log.set_player("   ")


def test_health_counts_distance_hours_and_maintenance():
    h = Health()
    for k in range(10):
        h.sample(k, dict(v_bus=27.0, battery_pct=80, dm_err=[1, 1, 13, 1]), dict(motor_temp=[40] * 4, driver_temp=[35] * 4),
                 dict(load=1.0, temp_c=50), odom_xy=(k * 1.0, 0.0))
    assert h.counters["km"] == pytest.approx(0.009) and h.counters["hours"] == pytest.approx(10 / 3600)
    assert h.hist[-1]["werr"] == 1 and h.hist[-1]["motor"] == [40] * 4
    h.count("fires", 2500)
    due = {d["item"]: d for d in h.due()}
    assert due["flywheels"]["due"] and not due["rollers"]["due"]
    h.done("flywheels")
    assert not {d["item"]: d for d in h.due()}["flywheels"]["due"]
    assert Health(json.loads(json.dumps(h.saved()))).counters["fires"] == 2500


def test_session_log_round_trip(tmp_path):
    log = session_log.SessionLog(tmp_path, "CATCH", source="sim", start=1000.0, map=None, world=dict(dist=6.5))
    log.write("pose", t=0.5, frame="odom", x=0.0, y=0.0, yaw=0.0)
    log.write("pose", t=1.0, frame="odom", x=1.0, y=0.0, yaw=0.0)
    log.write("throw", t=2.0, toss=1, caught=True, speed=float("nan"), release=[6.5, 0, 1.0],
              path=[[0, 6.5, 0, 1.0]], frame="odom")
    log.write("status", t=3.0, mission="MANUAL")
    s = log.close(now=1005.0)
    assert s["throws"] == 1 and s["caught"] == 1 and s["distance_m"] == 1.0 and s["missions"] == ["CATCH", "MANUAL"]
    head, recs = session_log.read(log.path)
    assert head["source"] == "sim" and head["format"] == session_log.FORMAT and recs[-1]["type"] == "end"
    th = session_log.throws(log.path)
    assert th[0]["speed"] is None and th[0]["release"] == [6.5, 0, 1.0]        # NaN written as null
    assert session_log.listing(tmp_path)[0]["summary"]["throws"] == 1
    with pytest.raises(ValueError):
        (tmp_path / "bad.jsonl").write_text('{"type": "pose"}\n')
        session_log.read(tmp_path / "bad.jsonl")
