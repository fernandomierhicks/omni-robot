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


# -- Goalie's score --------------------------------------------------------------------------------------------------
def _ball(u, v, du, age=0.0):
    return dict(x=u, y=v, vx=du, vy=0.0, age=age)


def test_a_gap_in_the_track_is_still_the_same_shot():
    g = Goalie((0.0, 0.0, 0.0), GoalieParams())
    t = 0.0
    for u in (2.5, 2.2, 1.9):
        g.tick(t, (0.0, 0.0, 0.0), _ball(u, 0.0, -2.0))
        t += 0.15
    assert g.shots == 1
    g.tick(t + 0.6, (0.0, 0.0, 0.0), _ball(1.9, 0.0, -2.0, age=0.6))      # stale, not yet lost
    g.tick(t + 0.7, (0.0, 0.0, 0.0), _ball(1.0, 0.0, -2.0))
    assert g.shots == 1 and g.state == "shot"


def test_a_saved_balls_bounce_isnt_a_new_shot():
    g = Goalie((0.0, 0.0, 0.0), GoalieParams())
    g.tick(0.0, (0.0, 0.0, 0.0), _ball(1.0, 0.0, -2.0))
    g.tick(0.2, (0.0, 0.0, 0.0), _ball(0.4, 0.0, -2.0))
    g.tick(0.3, (0.0, 0.0, 0.0), _ball(0.4, 0.0, 1.0))                    # back off the robot
    assert (g.shots, g.saves) == (1, 1)
    g.tick(0.5, (0.0, 0.0, 0.0), _ball(0.6, 0.0, -0.8))                   # the bounce's noise: not a shot
    assert g.shots == 1
    g.tick(2.0, (0.0, 0.0, 0.0), _ball(2.0, 0.0, -2.0))                   # the next one is
    assert g.shots == 2


# -- Fetch -----------------------------------------------------------------------------------------------------------
def test_fetch_finds_picks_brings_and_puts_down():
    c = ready()
    r = c.command("fetch", {"color": "red", "shape": "square"})
    assert r.ok and c.mission == "FETCH"
    find = arm_req(r.effects, "find")[0]
    assert find["color"] == "red" and find["shape"] == "square" and ("set_mode", "PICK", False) in r.effects
    c.on_robot_mode("PICK adult")                                   # its own mode change: still fetching
    assert c.mission == "FETCH"
    fx = arm(c, 101, found=dict(id=find["id"], candidates=[dict(id=3, range_m=1.2, score=0.95, color="red", shape="square")]))
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


def test_a_lidar_candidate_is_not_someone_to_talk_to():
    from omni_console.people import from_tracks
    tracks = [dict(status="candidate", base_x=0.8, base_y=0.0, range=0.8), dict(status="not_person", base_x=1.0, base_y=0.0, range=1.0),
              dict(status="person", base_x=2.0, base_y=0.5, range=2.06)]
    assert from_tracks(tracks) == [(2.0, 0.5), (0.8, 0.0)]                   # Follow: the person first, then the candidate
    assert from_tracks(tracks, confirmed_only=True) == [(2.0, 0.5)]
    assert from_tracks(tracks[:2], confirmed_only=True) == []


def test_fetch_never_picks_a_near_miss():
    """A find lists everything it sees, best first: a wrong colour (score 0) or wrong shape (0.2) is not the thing."""
    c = ready()
    r = c.command("fetch", {"color": "red", "shape": "square"})
    fid = arm_req(r.effects, "find")[0]["id"]
    near_misses = [dict(id=1, range_m=1.0, score=0.19), dict(id=2, range_m=0.8, score=0.0)]
    fx = arm(c, 101, found=dict(id=fid, candidates=near_misses))
    assert not arm_req(fx, "tidy") and c.fun["phase"] == "turn"       # looks on instead


def test_fetch_takes_the_right_colour_with_a_shape_guess():
    c = ready()
    r = c.command("fetch", {"color": "red", "shape": "square"})
    fid = arm_req(r.effects, "find")[0]["id"]
    fx = arm(c, 101, found=dict(id=fid, candidates=[dict(id=7, range_m=1.4, score=0.47), dict(id=2, range_m=0.8, score=0.0)]))
    assert arm_req(fx, "tidy")[0]["only"] == 7


def test_fetch_looks_round_then_gives_up():
    c = ready()
    r = c.command("fetch", {"color": "green"})
    fid = arm_req(r.effects, "find")[0]["id"]
    yaw = 0.0
    for look in range(4):
        arm(c, 101 + look, found=dict(id=fid, candidates=[]))
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


def test_fetch_drives_closer_to_a_patch_of_its_colour_then_finds_it():
    c = ready()
    r = c.command("fetch", {"color": "red", "shape": "square"})
    fid = arm_req(r.effects, "find")[0]["id"]
    blob = dict(pixels=120, bearing_deg=0.0, range_m=1.4, height_m=0.01, flat=False)
    arm(c, 101, found=dict(id=fid, candidates=[], image_only=[blob]))
    g = c.goal
    assert c.fun["phase"] == "approach" and g["x"] == pytest.approx(0.8) and g["y"] == pytest.approx(0.0)
    assert c.fun["looks"] == 1                                      # driving closer isn't one of the four looks
    c.pose = ("map", 0.8, 0.0, 0.0)
    c.on_nav_done(g["seq"], "succeeded")
    fid = arm_req(c.pop_fx(), "find")[0]["id"]
    assert c.fun["looks"] == 1 and c.fun["phase"] == "find"
    fx = arm(c, 110, found=dict(id=fid, candidates=[dict(id=4, range_m=0.6, score=0.95)]))
    assert arm_req(fx, "tidy")[0]["only"] == 4


@pytest.mark.parametrize("blob", [dict(pixels=500, bearing_deg=20.0, range_m=1.2, height_m=0.004, flat=True),   # a rug
                                  dict(pixels=500, bearing_deg=0.0, range_m=0.6, height_m=0.05, flat=False),    # faced, near
                                  dict(pixels=500, bearing_deg=20.0, range_m=5.0, height_m=0.05, flat=False),   # too far
                                  dict(pixels=500, bearing_deg=20.0, range_m=1.5, height_m=0.6, flat=False),    # not the floor
                                  dict(pixels=3800, bearing_deg=27.0, range_m=2.2, height_m=0.01, flat=False)])  # a rug's piece
def test_fetch_ignores_a_patch_not_worth_driving_to(blob):
    c = ready()
    r = c.command("fetch", {"color": "red"})
    fid = arm_req(r.effects, "find")[0]["id"]
    arm(c, 101, found=dict(id=fid, candidates=[], image_only=[blob]))
    assert c.fun["phase"] == "turn" and c.goal is None


def test_fetch_drives_closer_at_most_twice_and_never_to_the_same_spot():
    c = ready()
    r = c.command("fetch", {"color": "red"})
    fid = arm_req(r.effects, "find")[0]["id"]
    arm(c, 101, found=dict(id=fid, candidates=[], image_only=[dict(pixels=99, bearing_deg=45.0, range_m=2.0, flat=False)]))
    g = c.goal
    c.on_nav_done(g["seq"], "aborted")                               # Nav2 couldn't: it looks from here
    fid = arm_req(c.pop_fx(), "find")[0]["id"]
    arm(c, 110, found=dict(id=fid, candidates=[], image_only=[dict(pixels=99, bearing_deg=45.0, range_m=2.0, flat=False)]))
    assert c.fun["phase"] == "turn" and c.fun["approaches"] == 1     # the same spot: look round instead
    c = ready()
    r = c.command("fetch", {"color": "red"})
    fid = arm_req(r.effects, "find")[0]["id"]
    for k, brg in enumerate((60.0, -60.0)):
        arm(c, 101 + k, found=dict(id=fid, candidates=[], image_only=[dict(pixels=99, bearing_deg=brg, range_m=2.0, flat=False)]))
        c.on_nav_done(c.goal["seq"], "succeeded")
        fid = arm_req(c.pop_fx(), "find")[0]["id"]
    arm(c, 120, found=dict(id=fid, candidates=[], image_only=[dict(pixels=99, bearing_deg=120.0, range_m=2.0, flat=False)]))
    assert c.fun["phase"] == "turn" and c.fun["approaches"] == 2


def test_fetch_without_nav_only_turns():
    c = ready()
    c.set_caps(nav=False, catch=False, collision=False)
    r = c.command("fetch", {"color": "red"})
    fid = arm_req(r.effects, "find")[0]["id"]
    arm(c, 101, found=dict(id=fid, candidates=[], image_only=[dict(pixels=99, bearing_deg=20.0, range_m=1.5, flat=False)]))
    assert c.fun["phase"] == "turn"


def test_fetch_to_me_with_nobody_found_says_so():
    c = ready()
    r = c.command("fetch", {"color": "red", "to": "me"})
    fid = arm_req(r.effects, "find")[0]["id"]
    fx = arm(c, 101, found=dict(id=fid, candidates=[dict(id=3, range_m=1.2, score=0.95)]))
    arm(c, 102, tidy_on=True, tidy=dict(state="APPROACH"))
    c.now = 140.0
    arm(c, 140, tidy_on=False, tidy=dict(state="DONE", outcome="held", held=dict(id=3)))
    assert c.fun["phase"] == "look_person"
    yaw = 0.0
    stops = 0
    for _ in range(3000):                                            # the look round, nobody in it: in steps
        c.now += 0.05
        tw = c.drive_output(c.now)
        if c.fun["phase"] != "look_person":
            break
        stops += tw[2] == 0.0
        yaw += tw[2] * 0.05
        c.odom_pose = (0.0, 0.0, yaw)
    assert stops > 100                                               # it stood still to look
    assert c.fun["phase"] == "deliver" and c.fun["nobody"]
    c.on_nav_done(c.goal["seq"], "succeeded")
    c.pop_fx()
    arm(c, 160, tidy_on=True, tidy=dict(state="P_HOVER", mode="place"))
    fx = arm(c, 170, tidy_on=False, tidy=dict(state="DONE", outcome="placed", mode="place"))
    assert c.mission == "MANUAL" and "couldn't find anyone" in c.task
    assert c.command("activity").data["items"][0]["state"] != "done"
    assert any(f[0] == "face" and "couldn't find you" in f[1]["text"] for f in fx)


def test_fetch_gets_closer_to_an_unsure_shape_before_picking_it():
    c = ready()
    r = c.command("fetch", {"color": "red", "shape": "square"})
    fid = arm_req(r.effects, "find")[0]["id"]
    far = dict(id=9, range_m=1.4, bearing_deg=0.0, score=0.46, color=dict(name="red", match=0.95),
               shape={"class": "round", "sure": False, "maybe": ["square"]})
    fx = arm(c, 101, found=dict(id=fid, candidates=[far]))
    assert not arm_req(fx, "tidy") and c.fun["phase"] == "approach" and c.goal["x"] == pytest.approx(0.8)
    c.pose = ("map", 0.8, 0.0, 0.0)
    c.on_nav_done(c.goal["seq"], "succeeded")
    fid = arm_req(c.pop_fx(), "find")[0]["id"]
    near = dict(far, range_m=0.6, score=1.0, shape={"class": "square", "sure": True, "maybe": []})
    fx = arm(c, 110, found=dict(id=fid, candidates=[near]))
    assert arm_req(fx, "tidy")[0]["only"] == 9


def test_fetch_gets_closer_to_the_right_colour_with_a_low_shape_score():
    c = ready()
    r = c.command("fetch", {"color": "red", "shape": "square"})
    fid = arm_req(r.effects, "find")[0]["id"]
    long_guess = dict(id=5, range_m=1.3, bearing_deg=10.0, score=0.19, color=dict(name="red", match=0.95),
                      shape={"class": "long", "sure": False, "maybe": []})
    fx = arm(c, 101, found=dict(id=fid, candidates=[long_guess]))
    assert not arm_req(fx, "tidy") and c.fun["phase"] == "approach"


def test_fetch_takes_what_it_drove_up_to_when_the_shape_guess_flickers():
    c = ready()
    r = c.command("fetch", {"color": "red", "shape": "square"})
    fid = arm_req(r.effects, "find")[0]["id"]
    far = dict(id=9, range_m=1.4, bearing_deg=0.0, score=0.46, color=dict(name="red", match=0.95),
               shape={"class": "round", "sure": False, "maybe": ["square"]})
    arm(c, 101, found=dict(id=fid, candidates=[far]))
    c.pose = ("map", 0.8, 0.0, 0.0)
    c.on_nav_done(c.goal["seq"], "succeeded")
    fid = arm_req(c.pop_fx(), "find")[0]["id"]
    near = dict(far, range_m=0.65, bearing_deg=1.0, score=0.2, shape={"class": "other", "sure": False, "maybe": []})
    fx = arm(c, 110, found=dict(id=fid, candidates=[near]))
    assert arm_req(fx, "tidy")[0]["only"] == 9


def test_fetch_doesnt_drive_to_a_sure_wrong_shape():
    c = ready()
    r = c.command("fetch", {"color": "red", "shape": "square"})
    fid = arm_req(r.effects, "find")[0]["id"]
    crayon = dict(id=5, range_m=1.3, bearing_deg=10.0, score=0.19, color=dict(name="red", match=0.95),
                  shape={"class": "long", "sure": True, "maybe": []})
    arm(c, 101, found=dict(id=fid, candidates=[crayon]))
    assert c.fun["phase"] == "turn" and c.goal is None


def test_fetch_without_a_map_says_it_brought_nothing():
    c = ready()
    c.pose = ("odom", 0.0, 0.0, 0.0)
    r = c.command("fetch", {"color": "red", "to": "here"})
    fid = arm_req(r.effects, "find")[0]["id"]
    arm(c, 101, found=dict(id=fid, candidates=[dict(id=3, range_m=1.2, score=0.95)]))
    arm(c, 102, tidy_on=True, tidy=dict(state="APPROACH"))
    fx = arm(c, 140, tidy_on=False, tidy=dict(state="DONE", outcome="held", held=dict(id=3)))
    assert arm_req(fx, "place") and c.fun["no_map"]
    arm(c, 160, tidy_on=True, tidy=dict(state="P_HOVER", mode="place"))
    arm(c, 170, tidy_on=False, tidy=dict(state="DONE", outcome="placed", mode="place"))
    assert c.mission == "MANUAL" and "not localized" in c.task
    assert c.command("activity").data["items"][0]["state"] != "done"


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
