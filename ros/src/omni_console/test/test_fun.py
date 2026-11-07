"""The fun missions, ROS-free (console_fun.py, dance.py, goalie.py): Fetch, Messenger, Block tower, Dance party, Goalie,
driven with the arm's states, Nav2's results and the people the node would pass. Run from the repo root:
  & $py -m pytest -q ros/src/omni_console/test
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_console.dance import MAX_REACH, ROUTINES, Dance                 # noqa: E402
from omni_console.goalie import Goalie, GoalieParams                      # noqa: E402
from omni_console.missions import Console                                 # noqa: E402


def kinds(fx):
    return [f[0] for f in fx]


def arm_req(fx, verb):
    return [f[1] for f in fx if f[0] == "arm" and f[1].get("verb") == verb]


def ready(t=100.0) -> Console:
    c = Console()
    c.now = t
    c.on_robot_mode("TELEOP adult")
    c.set_caps(nav=True, catch=False, collision=False)
    c.on_arm_state(dict(connected=True, tidy_on=False, events=[]), t)
    c.pose = ("map", 0.0, 0.0, 0.0)
    c.odom_pose = (0.0, 0.0, 0.0)
    return c


def arm(c, t, **kw):
    s = dict(connected=True, tidy_on=False, events=[])
    s.update(kw)
    return list(c.on_arm_state(s, t))


# -- Fetch -----------------------------------------------------------------------------------------------------------
def test_fetch_finds_picks_brings_and_puts_down():
    c = ready()
    r = c.command("fetch", {"color": "red", "shape": "square"})
    assert r.ok and c.mission == "FETCH"
    find = arm_req(r.effects, "find")[0]
    assert find["color"] == "red" and find["shape"] == "square" and ("set_mode", "PICK", False) in r.effects
    c.on_robot_mode("PICK adult")                                   # its own mode change: still fetching
    assert c.mission == "FETCH"
    fx = arm(c, 101, found=dict(id=find["id"], candidates=[dict(id=3, range_m=1.2, color="red", shape="square")]))
    pick = arm_req(fx, "tidy")[0]
    assert pick["only"] == 3 and pick["keep"] is True
    arm(c, 102, tidy_on=True, tidy=dict(state="APPROACH"))
    c.fun_people([(2.5, 1.0)], 140.0)
    c.now = 140.0
    fx = arm(c, 140, tidy_on=False, tidy=dict(state="DONE", outcome="held", held=dict(id=3)))
    assert "nav_goal" in kinds(fx) and ("set_mode", "TELEOP", False) in fx
    g = c.goal
    assert math.hypot(2.5 - g["x"], 1.0 - g["y"]) == pytest.approx(0.7, abs=1e-6)       # 0.7 m short, facing them
    assert g["yaw"] == pytest.approx(math.atan2(1.0, 2.5))
    c.on_robot_mode("TELEOP adult")
    c.on_nav_done(g["seq"], "succeeded")
    place = arm_req(c.pop_fx(), "place")
    assert place and place[0]["ahead"] is True
    arm(c, 160, tidy_on=True, tidy=dict(state="P_HOVER", mode="place"))
    fx = arm(c, 170, tidy_on=False, tidy=dict(state="DONE", outcome="placed", mode="place"))
    assert any(f[0] == "face" and "Here you go" in f[1]["text"] for f in fx)
    c.tick(171.0)
    assert c.mission == "FETCH"
    c.tick(174.0)
    assert c.mission == "MANUAL" and c.task == "Fetched the red square"
    act = c.command("activity").data["items"][0]
    assert act["mission"] == "FETCH" and act["state"] == "done"


def test_fetch_looks_round_then_gives_up():
    c = ready()
    r = c.command("fetch", {"color": "green"})
    fid = arm_req(r.effects, "find")[0]["id"]
    yaw = 0.0
    for look in range(4):
        fx = arm(c, 101 + look, found=dict(id=fid, candidates=[]))
        if look == 3:
            assert c.mission == "MANUAL" and "no green" in c.task
            break
        assert c.fun["phase"] == "turn"
        for _ in range(60):                                        # the quarter turn, on the odometry
            tw = c.drive_output(c.now)
            if c.fun["phase"] != "turn":
                break
            yaw += tw[2] * 0.05
            c.odom_pose = (0.0, 0.0, yaw)
        fid = arm_req(c.pop_fx(), "find")[0]["id"]


def test_fetch_stop_puts_what_it_holds_away():
    c = ready()
    c.command("fetch", {"color": "red"})
    r = c.command("stop")
    assert c.mission == "IDLE" and arm_req(r.effects, "stop")


def test_fetch_needs_the_arm():
    c = Console()
    c.on_robot_mode("TELEOP adult")
    assert not c.command("fetch", {"color": "red"}).ok
    assert not ready().command("fetch", {"shape": "triangle"}).ok


# -- Messenger -------------------------------------------------------------------------------------------------------
def spin(c, until_phase_changes=True, n=400):
    yaw = c.odom_pose[2]
    ph = c.fun["phase"]
    for _ in range(n):
        tw = c.drive_output(c.now)
        if tw is None:
            break
        yaw += tw[2] * 0.05
        c.odom_pose = (0.0, 0.0, yaw)
        c.now += 0.05
        if c.fun is None or c.fun["phase"] != ph or tw == (0.0, 0.0, 0.0):
            break


def test_message_goes_room_to_room_and_tells_the_first_person():
    c = ready()
    c.set_map("house", {}, {}, rooms={"kitchen": [[2, 0], [4, 0], [4, 2], [2, 2]],
                                      "hall": [[-3, -1], [-1, -1], [-1, 1], [-3, 1]]})
    r = c.command("message", {"text": "dinner's ready"})
    assert r.ok and c.mission == "MESSAGE" and c.fun["phase"] == "look"
    spin(c)
    fx = c.tick(c.now)
    assert "nav_goal" in kinds(fx) and c.goal["name"] == "hall"     # nearest first
    c.on_nav_done(c.goal["seq"], "succeeded")
    assert c.fun["phase"] == "look"
    spin(c)
    c.tick(c.now)
    assert c.goal["name"] == "kitchen"
    c.on_nav_done(c.goal["seq"], "succeeded")
    c.pose = ("map", 3.0, 1.0, 0.0)
    c.fun_people([(3.0, 0.0)], c.now)                               # someone 3 m ahead
    c.tick(c.now)
    assert c.fun["phase"] == "approach" and c.goal["name"] == "the person"
    c.on_nav_done(c.goal["seq"], "succeeded")
    said = [f for f in c.pop_fx() if f[0] == "face"]
    assert said and "dinner's ready" in said[0][1]["text"]
    c.tick(c.now + 5.0)
    assert c.mission == "MANUAL" and c.task.startswith("Message delivered")


def test_message_with_nobody_anywhere():
    c = ready()
    c.set_map("house", {}, {}, rooms={"kitchen": [[2, 0], [4, 0], [4, 2], [2, 2]]})
    c.command("message", {"text": "hello"})
    spin(c)
    c.tick(c.now)
    c.on_nav_done(c.goal["seq"], "aborted")                          # can't reach the kitchen: next (none)
    assert c.mission == "MANUAL" and "nobody found" in c.task


def test_message_checks():
    c = ready()
    assert not c.command("message", {"text": ""}).ok
    assert not c.command("message", {"text": "hi", "rooms": ["attic"]}).ok


# -- Block tower ------------------------------------------------------------------------------------------------------
def test_tower_asks_the_arm_and_reports_the_height():
    c = ready()
    r = c.command("tower", {})
    req = arm_req(r.effects, "tidy")[0]
    assert req["tower"] is True and req["max_blocks"] == 4 and c.mission == "TOWER"
    arm(c, 101, tidy_on=True, tidy=dict(state="P_HOVER", tower=dict(n=2)))
    assert "2 high" in c.task
    fx = arm(c, 200, tidy_on=False, tidy=dict(state="DONE", outcome="tower done", tower=dict(n=3)))
    assert c.mission == "MANUAL" and c.task == "Block tower: 3 blocks high"
    assert any(f[0] == "face" for f in fx)


# -- Dance party ------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("routine", list(ROUTINES))
def test_dance_routines_come_back_where_they_started(routine):
    d = Dance(routine, 120, 0.5, 40)
    assert d.reach() <= MAX_REACH + 1e-6
    x = y = yaw = 0.0
    t, dt = 0.0, 0.002
    while t < d.length:
        vx, vy, wz = d.twist_at(t)
        assert math.hypot(vx, vy) <= 0.5 + 1e-9
        c, s = math.cos(yaw), math.sin(yaw)
        x += (c * vx - s * vy) * dt
        y += (s * vx + c * vy) * dt
        yaw += wz * dt
        t += dt
    assert math.hypot(x, y) < 0.03 and abs(math.remainder(yaw, 2 * math.pi)) < math.radians(3)


def test_dance_mission_runs_its_twist_and_tricks_then_ends():
    c = ready()
    c.command("set", {"key": "dance.seconds", "value": 15})
    r = c.command("dance", {"routine": "robot"})
    assert r.ok and c.mission == "DANCE" and any(f[0] == "face" and f[1]["act"] == "sing" for f in r.effects)
    tricks = 0
    for k in range(20 * 40):
        tw = c.drive_output(100.0 + k * 0.05)
        tricks += sum(1 for f in c.pop_fx() if f[0] == "face")
        if c.mission != "DANCE":
            break
        assert tw is not None
    assert c.mission == "MANUAL" and tricks >= 2 and c.task.startswith("Dance over")
    assert not c.command("dance", {"routine": "tango"}).ok


# -- Goalie -------------------------------------------------------------------------------------------------------------
def play_shot(v_lat: float, aim: float, speed: float = 2.0, robot0: float = 0.0, p=None):
    """A ball rolled from 3 m at the goal line aimed at v = aim; the robot follows the goalie's slide. Returns the goalie
    and where the robot was when the ball reached its front."""
    g = Goalie((0.0, 0.0, 0.0), p or GoalieParams())
    rx, ry = 0.0, robot0
    bx, by = 3.0, v_lat
    t_hit = (bx - 0.25) / speed
    bvx, bvy = -speed, (aim - by) / t_hit
    t = 0.0
    hit_y = None
    for _ in range(200):
        ball = dict(x=bx, y=by, vx=bvx, vy=bvy, age=0.03)
        vx, vy, wz = g.tick(t, (rx, ry, 0.0), ball)
        rx += vx * 0.02
        ry += vy * 0.02
        bx += bvx * 0.02
        by += bvy * 0.02
        if hit_y is None and bx <= 0.25:
            hit_y = ry
            if abs(by - ry) < 0.3:                                 # the robot's 0.45 m-wide front: bounced back
                bvx = 1.0
        t += 0.02
        if g.state in ("save", "goal", "miss"):
            break
    return g, hit_y


def test_goalie_slides_to_block_a_shot_and_counts_the_save():
    g, hit_y = play_shot(0.0, 0.45)
    assert hit_y == pytest.approx(0.45, abs=0.12) and g.saves == 1 and g.shots == 1


def test_goalie_lets_a_wide_shot_go_and_never_leaves_its_goal():
    p = GoalieParams(half_width=0.5)
    g, hit_y = play_shot(0.0, 1.2, p=p)
    assert hit_y <= 0.5 + 0.05 and g.goals == 0


def test_goalie_mission_wakes_the_tracker_and_keeps_score():
    c = ready()
    r = c.command("goalie", {})
    assert c.mission == "GOALIE" and ("goalie", True) in r.effects
    c.on_ball(dict(x=3.0, y=0.0, vx=-2.0, vy=0.3, seen=True), c.now)
    tw = c.drive_output(c.now)
    assert tw[1] > 0                                                # sliding left toward where it will cross
    c.goalie.events.append("save")
    c.goalie.saves = 1
    fx = c.tick(c.now + 0.5)
    assert any(f[0] == "face" and f[1]["text"] == "Saved!" for f in fx)
    r = c.command("stop")
    assert ("goalie", False) in r.effects and c.goalie is None


def test_people_reach_the_core_outside_follow():
    c = ready()
    c.command("message", {"text": "hi"})
    assert c.wants_people()
    c.on_people([(1.0, 0.0)], c.now)
    assert c.people_seen == [(1.0, 0.0)]
