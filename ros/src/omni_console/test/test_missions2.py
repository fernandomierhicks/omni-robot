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
from omni_console.explore import ExploreParams, Grid, frontier_goal, still_open, with_blocked   # noqa: E402
from omni_console.missions import BLOCK_AHEAD, Console                      # noqa: E402

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
    bl = []                                    # each spot tried once, the opening's others after it, then none
    for _ in range(20):
        goal, _ = frontier_goal(g2, (1.0, 1.5, 0.0), (1.0, 1.5), ExploreParams(), blacklist=bl)
        if goal is None:
            break
        assert all(math.hypot(goal[0] - b[0], goal[1] - b[1]) >= 0.6 for b in bl)
        bl.append(goal[:2])
    assert goal is None and 2 <= len(bl) <= 6         # a 2.6 m opening: a few spots 0.6 m apart


def test_a_narrow_doorway_leads_on():
    """Nov 25 (hm16): goals keep 0.35 m from walls, and so did the way to them, so every doorway under 0.7 m read as
    shut. The way needs only the robot's radius and a little (pass_clearance 0.28)."""
    d = np.full((60, 140), -1, np.int8)
    d[:, :110] = 0                                                   # 3 x 5.5 m seen; the far room's end unknown
    d[0, :] = d[-1, :] = d[:, 0] = 100
    d[:, 60] = 100                                                   # a wall at x 3.0 m ...
    d[27:39, 60] = 0                                                 # ... with a 0.6 m doorway
    g = Grid(d, 0.05, 0.0, 0.0)
    robot = (1.0, 1.5, 0.0)
    goal, info = frontier_goal(g, robot, robot[:2], ExploreParams())
    assert goal is not None and goal[0] > 3.0                       # through it, in the far room
    assert frontier_goal(g, robot, robot[:2], ExploreParams(pass_clearance=0.35))[0] is None   # the old rule: shut
    d[37:39, 60] = 100                                               # 0.5 m: shut (on 5 cm cells 0.55 m still passes)
    assert frontier_goal(Grid(d, 0.05, 0.0, 0.0), robot, robot[:2], ExploreParams())[0] is None


def test_a_robot_in_a_narrow_passage_still_sees_the_rooms_openings():
    """Nov 26 (hm26): in a 0.55 m passage the passable strip under the robot was an island cut off by a 0.5 m neck;
    every opening looked unreachable and Explore went home early. It starts from what it reaches within 1 m, and from
    what the start reaches (hm31: Nav2 drove it into bathroom 3 through a door Explore saw no way out of)."""
    d = np.full((80, 100), 100, np.int8)
    d[36:79, 1:60] = 0                                               # a room, its east part unknown
    d[36:79, 60:100] = -1
    d[30:36, 20:28] = 0                                              # a 0.4 m neck ...
    d[4:30, 18:30] = 0                                               # ... into a 0.6 m passage
    g = Grid(d, 0.05, 0.0, 0.0)
    robot = (1.225, 1.225, 0.0)                                      # in the passage, 0.8 m from the room's open part
    goal, info = frontier_goal(g, robot, robot[:2], ExploreParams())
    assert goal is not None and goal[0] > 2.5 and info["frontiers"] == 1
    assert frontier_goal(g, robot, robot[:2], ExploreParams(near=0.0))[0] is None     # the robot's own island only
    deep = (1.225, 0.425, 0.0)                                       # 1.6 m down it: out of reach, but the start is
    assert frontier_goal(g, deep, deep[:2], ExploreParams())[0] is None
    assert frontier_goal(g, deep, (1.0, 3.0), ExploreParams())[0] is not None


def test_a_block_that_would_cut_the_way_back_is_refused():
    """Nov 26 (hm22): a block at a ramp's top cut the robot off from the family room (home); blocks must keep the
    robot connected to where exploring started."""
    from omni_console.explore import connected
    d = np.full((40, 120), 100, np.int8)                             # two rooms joined by a 1 m corridor
    d[1:39, 1:40] = 0
    d[1:39, 80:119] = 0
    d[10:30, 40:80] = 0
    g = Grid(d, 0.05, 0.0, 0.0)
    a, b = (1.0, 1.0), (5.0, 1.0)
    assert connected(g, a, b)
    assert not connected(g, a, b, [(3.0, 1.0)], 0.45)                # across the corridor: cut
    assert connected(g, a, b, [(1.0, 0.4)], 0.2)                     # in a room's corner: not


def test_an_opening_past_a_drop_is_left_alone():
    """Nov 25 (hm11): the lidar mapped the den's floor, 165 mm down, as free; Explore sent the robot for its openings
    and the floor camera's drop held it at the doors until the jam check gave up."""
    g = room()                                                       # the opening at x 3 m
    robot = (1.0, 1.5, 0.0)
    assert frontier_goal(g, robot, robot[:2], ExploreParams())[0] is not None
    rows, cols = 40, 30                                              # the height map: its own cells, 0.1 m, offset
    b = Grid(np.full((rows, cols), -1, np.int8), 0.1, 0.0, -0.5)
    b.data[:, 22:25] = 100                                           # a drop band across the room at x 2.2-2.5 m
    gb = with_blocked(g, b)
    assert gb is not g and g.data[30, 46] == 0 and gb.data[30, 46] == 100
    assert frontier_goal(gb, robot, robot[:2], ExploreParams())[0] is None
    assert with_blocked(g, None) is g and with_blocked(g, Grid(np.full((5, 5), -1, np.int8), 0.1, 0, 0)) is g


def test_a_new_maps_starburst_sends_the_robot_off_its_spot():
    """A new map's first scan: a free disc ringed by one frontier, whose centroid is the robot. The goal must be a
    reachable spot on the ring's side, out of Nav2's goal tolerance (Nov 24: it was the robot's own cell, forever)."""
    rows = cols = 80
    d = np.full((rows, cols), -1, np.int8)
    yy, xx = np.mgrid[:rows, :cols]
    d[(yy - 40) ** 2 + (xx - 40) ** 2 <= 22 ** 2] = 0                  # 1.1 m around the robot at (2, 2)
    g = Grid(d, 0.05, 0.0, 0.0)
    robot = (2.0, 2.0, 0.0)
    goal, info = frontier_goal(g, robot, robot[:2], ExploreParams())
    assert goal is not None and info["frontiers"] == 1
    assert math.hypot(goal[0] - 2.0, goal[1] - 2.0) >= 0.5
    # standing on it, the same opening gives another goal, never the robot's own spot; reached, it is left alone
    goal2, _ = frontier_goal(g, (goal[0], goal[1], 0.0), robot[:2], ExploreParams())
    assert goal2 is not None and math.hypot(goal2[0] - goal[0], goal2[1] - goal[1]) >= 0.5
    goal3, _ = frontier_goal(g, (goal[0], goal[1], 0.0), robot[:2], ExploreParams(), [goal[:2], goal2[:2]])
    assert goal3 is not None and all(math.hypot(goal3[0] - b[0], goal3[1] - b[1]) >= 0.6 for b in (goal, goal2))


def test_explore_does_not_return_to_a_reached_goal():
    c = ready()
    c.set_slam(mode="mapping", managed=True)
    c.command("explore")
    c.pose = ("map", 1.0, 1.5, 0.0)
    g = room()
    c.tick(1.0, g)
    first = (c.goal["x"], c.goal["y"])
    c.on_nav_done(c.goal["seq"], "succeeded", 2.0)
    c.pose = ("map", first[0], first[1], 0.0)
    assert c.tick(3.5, g) == () and c.goal is None                  # no plan on the map from before the drive
    fx = c.tick(4.0, room())                                        # the next map: the opening is still there
    assert c.explore["reached"] == [first]
    assert "nav_goal" in kinds(fx) and c.goal["name"] == "frontier"  # another spot of it, never the same one
    assert math.hypot(c.goal["x"] - first[0], c.goal["y"] - first[1]) >= 0.6


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


def test_explore_tries_the_drive_back_again_before_giving_up():
    c = ready()
    c.set_slam(mode="mapping", managed=True)
    c.command("explore")
    c.pose = ("map", 1.0, 1.5, 0.0)
    c.tick(1.0, room())
    c.on_nav_done(c.goal["seq"], "succeeded", 10.0)
    full = room(known_cols=100)
    full.data[:, -1] = 100
    c.tick(17.0, full)
    assert c.goal["name"] == "start"
    t = 20.0
    for k in range(2):                                              # Nav2 gives up: tried again 10 s later
        c.on_nav_done(c.goal["seq"], "aborted", t)
        assert c.mission == "EXPLORE" and c.goal is None and "trying again" in c.events[-1][1]
        assert c.tick(t + 5.0, full) == ()
        fx = c.tick(t + 10.5, full)
        assert "nav_goal" in kinds(fx) and c.goal["name"] == "start"
        t += 20.0
    c.on_nav_done(c.goal["seq"], "aborted", t)                      # the third time: done, not back
    assert c.mission == "MANUAL" and "could not drive back" in c.task


def test_explore_skips_a_failed_opening():
    c = ready()
    c.set_slam(mode="mapping", managed=True)
    c.command("explore")
    c.pose = ("map", 1.0, 1.5, 0.0)
    c.tick(1.0, room())
    c.on_nav_done(c.goal["seq"], "aborted", 2.0)
    assert len(c.explore["blacklist"]) == 1 and c.mission == "EXPLORE"


def test_explore_tries_the_skipped_openings_once_more_before_it_is_done():
    """Nov 26 (hm23): a failure is often the moment's; the skipped spots get one more try at the end."""
    c = _exploring()
    c.blocks = [(9.0, 9.0)]
    t, tried = 2.0, []
    while c.mission == "EXPLORE" and c.explore["phase"] == "explore" and t < 400.0:
        if c.goal is not None:
            tried.append((c.goal["x"], c.goal["y"]))
            c.on_nav_done(c.goal["seq"], "aborted", t)
        c.tick(t + 3.0, room())
        t += 4.0
    assert c.explore["phase"] == "home" and c.explore["second"]
    assert any("once more" in m for _, m in c.events) and c.blocks == []
    n = len(c.explore["blacklist"])
    assert n >= 2 and len(tried) == 2 * n                             # each spot twice, then home


def _exploring(start=(1.0, 1.5, 0.0)):
    c = ready()
    c.set_slam(mode="mapping", managed=True)
    c.command("explore")
    c.pose = ("map",) + tuple(start)
    c.tick(1.0, room())
    assert c.goal is not None
    return c


def test_wedged_on_the_way_it_says_stuck_drives_home_then_stops():
    """Nov 25: wedged under the coffee table, Explore replanned and Nav2 recovered for the last ~4 min of the run."""
    c = _exploring()
    c.pose = ("map", 2.0, 1.5, 0.0)                                 # drove 1 m, then wedged
    t, fx = 1.5, ()
    while c.mission == "EXPLORE" and t < 200.0:
        x = 2.0 + (0.3 if int(t) % 10 < 3 else 0.0)                   # Nav2's backup recovery: 0.3 m and back
        c.pose = ("map", x, 1.5, 0.0)
        fx = c.tick(t, room())
        t += 0.5
    assert c.mission == "GOTO" and c.homing and c.goal["name"] == "home"
    assert 80.0 <= t - 1.5 < 90.0                                     # 40 s: the opening skipped; 40 s more: stuck
    assert (c.goal["x"], c.goal["y"]) == (1.0, 1.5)                   # where exploring started
    assert c.stuck["why"].startswith("no headway") and c.summary()["stuck"] == c.stuck
    assert "nav_goal" in kinds(fx) and c.events[-1][0] == "error"
    while c.mission == "GOTO" and t < 200.0:                          # stuck on the way home too: motors off
        fx = c.tick(t)
        t += 0.5
    assert c.mission == "IDLE" and c.stuck is not None and ("nav_cancel",) in fx
    c.command("stop")
    assert c.stuck is not None                                       # holds through a stop
    c.command("mission", {"name": "MANUAL"})
    assert c.stuck is None                                           # a person took it on


def test_near_a_frontier_goal_is_there():
    """Nov 25 (hm9): in bathroom 1 MPPI hovered 0.17 m short of a frontier goal (Nav2's tolerance 0.10 m) for 100 s."""
    c = _exploring()
    g = (c.goal["x"], c.goal["y"])
    c.pose = ("map", g[0] - 0.4, g[1], 0.0)
    assert c.tick(1.5, room()) == () and c.goal is not None           # 0.4 m: still driving
    c.pose = ("map", g[0] - 0.2, g[1], 0.0)
    assert c.tick(2.0, room()) == () and c.goal is not None           # passing through: Nav2 finishes it
    for k in range(15):                                               # 7.5 s hovering there: still Nav2's
        assert c.tick(2.5 + 0.5 * k, room()) == ()
    fx = c.tick(10.0, room())                                         # 8 s: there
    assert fx == (("nav_cancel",),) and c.goal is None and c.explore["reached"] == [g]
    assert c.mission == "EXPLORE" and c.stuck is None
    c.on_nav_done(1, "canceled", 10.1)                                # Nav2's answer to the cancel: ignored
    assert c.explore["blacklist"] == []
    fx = c.tick(11.0, room())                                         # the next map: another spot of the opening
    assert "nav_goal" in kinds(fx) and c.goal["name"] == "frontier"
    assert math.hypot(c.goal["x"] - g[0], c.goal["y"] - g[1]) >= 0.6


def test_a_stall_just_short_of_a_frontier_skips_it_once_then_is_a_jam():
    c = _exploring()
    g = (c.goal["x"], c.goal["y"])
    c.pose = ("map", g[0] - 0.45, g[1], 0.0)                         # just short, the controller giving nothing
    t = 1.5
    while c.goal is not None and (c.goal["x"], c.goal["y"]) == g and t < 100.0:
        c.tick(t, room())
        t += 0.5
    assert 40.0 <= t - 1.5 < 42.0
    assert c.mission == "EXPLORE" and c.stuck is None and c.explore["blacklist"] == [g]
    assert c.events[-1][0] == "warn" and "Skipping" in c.events[-1][1]
    assert c.goal is None
    c.tick(t, room())                                                 # the next map: another spot of the opening
    t += 0.5
    assert c.goal["name"] == "frontier" and math.hypot(c.goal["x"] - g[0], c.goal["y"] - g[1]) >= 0.6
    for _ in range(200):                                              # no headway since: the next stall is a jam
        if c.stuck is not None:
            break
        c.tick(t, room())
        t += 0.5
    assert c.stuck["why"].startswith("no headway") and c.homing       # a jam: it drives home
    c = _exploring()                                                  # stalled far from its goal: skipped too
    g = (c.goal["x"], c.goal["y"])
    c.pose = ("map", g[0] - 1.5, g[1], 0.0)
    t = 1.5
    while c.goal is not None and (c.goal["x"], c.goal["y"]) == g and t < 100.0:
        c.tick(t, room())
        t += 0.5
    assert c.mission == "EXPLORE" and c.explore["blacklist"] == [g] and c.stuck is None


def test_a_stall_on_the_way_marks_the_plan_ahead_impassable():
    """Nov 26 (hm18-19): a door open across the hallway left a gap the planner took and the robot couldn't pass, goal
    after goal. The stall marks the plan 0.6 m ahead as a no-go spot for the rest of the exploration."""
    c = _exploring()
    g = (c.goal["x"], c.goal["y"])
    c.pose = ("map", g[0] - 1.5, g[1], 0.0)
    c.plan_xy = [(g[0] - 2.0 + 0.1 * k, g[1]) for k in range(21)]   # straight on to the goal, from behind it
    fx, t = list(c.tick(1.5, room())), 2.0
    c.plan_xy = [(g[0] - 1.5 - 0.1 * k, g[1]) for k in range(16)]   # Nav2's replan in a recovery: back the other way
    while c.goal is not None and (c.goal["x"], c.goal["y"]) == g and t < 100.0:
        fx += list(c.tick(t, room()))
        t += 0.5
    assert ("masks",) in fx and len(c.blocks) == 1
    bx, by = c.blocks[0]
    assert abs(math.hypot(bx - (g[0] - 1.5), by - g[1]) - BLOCK_AHEAD) < 0.06 and bx > g[0] - 1.5     # ahead
    z = c.nav_zones()
    assert "_block0" in z and len(z["_block0"]) == 8 and "_block0" not in c.zones
    c.command("stop")                                                 # the run over: off the mask again
    assert c.blocks == [] and ("masks",) in c.pop_fx() and "_block0" not in c.nav_zones()


def test_a_goal_failing_in_a_tight_spot_backs_the_robot_out_and_keeps_the_goal():
    """Nov 26 (hm23): beside the kitchen island's stools the robot stood inside Nav2's inscribed radius; every plan
    failed from there and 13 kitchen goals were blacklisted in 400 s. A failure from a tight spot is the spot's: the
    robot backs out the way it came and the goal stays (once; failing again it is blacklisted)."""
    from omni_console.explore import backout_path, clearance_at
    c = _exploring()
    t = 1.5
    for k in range(26):                                               # down toward the wall at y 0: 1.5 -> 0.25
        c.pose = ("map", 2.0, 1.5 - 0.05 * k, -math.pi / 2)
        assert c.drive_output(t) is None                              # Nav2 drives; the trail is kept
        t += 0.1
    g = room()
    assert clearance_at(g, (2.0, 0.25)) < 0.29 <= clearance_at(g, (2.0, 0.40))
    c.tick(t, g)
    goal = dict(c.goal)
    c.on_nav_done(goal["seq"], "aborted", t)
    assert c.explore["blacklist"] == [] and c.explore["backout"] is not None and "Backing out" in c.events[-1][1]
    assert c.tick(t + 0.5, g) == () and c.goal is None                # no goal while backing out
    for _ in range(100):                                              # follow its twist (body frame, yaw -90 deg)
        tw = c.drive_output(t)
        if c.explore["backout"] is None:
            break
        vx, vy, _ = tw
        assert vx < 0 and abs(vy) < 1e-9                              # straight back the way it came
        c.pose = ("map", 2.0, c.pose[2] - vx * 0.1, -math.pi / 2)
        t += 0.1
    assert tw == (0.0, 0.0, 0.0) and clearance_at(g, c.pose[1:3]) >= 0.29 and c.pose[2] < 0.5
    assert c.explore["trail"][-1] == c.pose[1:3]
    c.explore["backout"] = None
    c.pose = ("map", 2.0, 0.25, -math.pi / 2)                         # the same goal fails from the spot again
    c.goal = goal
    c.on_nav_done(goal["seq"], "aborted", t)
    assert c.explore["blacklist"] == [(goal["x"], goal["y"])]
    # the way out: only within BACKOUT_MAX of trail, only to a clear point
    trail = [(2.0, 1.5 - 0.05 * k) for k in range(26)]
    assert backout_path(g, trail, 0.29, 2.0)[-1][1] >= 0.34
    assert backout_path(g, trail, 0.29, 0.08) == [] and backout_path(g, trail[-2:], 0.29, 2.0) == []


def test_a_stall_on_the_way_backs_out_the_way_it_came():
    """Nov 26 (hm30): pinned on a ramp doorway's jamb with the pose 0.4 m off (on the doorway's middle), the spot didn't
    look tight; the stall now backs the robot out >= 0.5 m along its trail before the next goal."""
    c = _exploring()
    t = 1.5
    for k in range(16):                                               # 1.5 m east, then stuck
        c.pose = ("map", 1.0 + 0.1 * k, 1.5, 0.0)
        c.drive_output(t)
        c.tick(t, room())
        t += 0.5
    goal = (c.goal["x"], c.goal["y"])
    while c.explore.get("backout") is None and t < 100.0:
        c.drive_output(t)
        c.tick(t, room())
        t += 0.5
    b = c.explore["backout"]
    assert b is not None and c.explore["blacklist"] == [goal] and c.stuck is None
    assert math.hypot(b["path"][-1][0] - 2.5, b["path"][-1][1] - 1.5) >= 0.5
    for _ in range(200):                                              # it can't move: the back-out times out
        tw = c.drive_output(t)
        t += 0.5
        if c.explore["backout"] is None:
            break
        assert tw[0] < 0                                              # backwards along its trail (facing east)
    assert c.jam is not None                                          # not moved: the next stall is a jam


def test_headway_keeps_the_jam_check_quiet_and_a_slow_goal_times_out():
    c = _exploring()
    for k in range(120):                                              # 60 s at 0.1 m/s: never stuck
        c.pose = ("map", 1.0 + 0.05 * k, 1.5, 0.0)
        c.tick(1.5 + 0.5 * k)
    assert c.mission == "EXPLORE" and c.stuck is None
    c = ready()                                                       # Go To: 60 s + 3 x 2 m / 2 m/s = 63 s
    c.pose = ("map", 0.0, 0.0, 0.0)
    c.command("goto", {"x": 2.0, "y": 0.0})
    assert abs(c.goal["limit"] - 63.0) < 0.5
    t, x = 0.0, 0.0
    while not c.homing and t < 100.0:                                 # moving in circles, never arriving
        x = (x + 0.6) % 1.8
        c.pose = ("map", x - 0.9, 0.5, 0.0)
        c.tick(t)
        t += 1.0
    assert c.mission == "GOTO" and c.homing and c.stuck["why"].startswith("not there")
    assert 63.0 <= t < 66.0
    c2 = ready()                                                      # Nav2's route around a wall: 12 m, not 2
    c2.pose = ("map", 0.0, 0.0, 0.0)
    c2.command("goto", {"x": 2.0, "y": 0.0})
    c2.on_nav_feedback(c2.goal["seq"], 12.0)
    assert abs(c2.goal["limit"] - 78.0) < 0.5
    c2.on_nav_feedback(c2.goal["seq"], 5.0)                           # its fixed budget: progress doesn't extend it
    assert abs(c2.goal["limit"] - 78.0) < 0.5
    c.on_nav_done(c.goal["seq"], "succeeded", t)                      # home: driving by hand, the fault kept
    assert c.mission == "MANUAL" and c.stuck is not None and not c.homing


def test_a_stall_creeps_along_nav2s_plan_then_hands_back():
    """Nov 26 (world:=course): MPPI sat at the spiral's 0.8 m gap asking ~0.02 m/s while NavFn's plan ran down its
    middle. After 10 s without headway the console follows the plan 1.5 m itself (the manual input), once per spot."""
    c = ready()
    c.pose = ("map", 0.0, -0.25, 0.0)                                # 0.25 m off the plan's line, facing +x
    c.command("goto", {"x": 4.0, "y": 0.0})
    c.plan_xy = [(0.2 * i, 0.0) for i in range(21)]                  # Nav2's plan: along y = 0
    t = 0.0
    while t < 30.0:
        c.tick(t)
        if c.creep is not None:
            break
        assert c.drive_output(t) is None                             # Nav2 drives meanwhile
        t += 0.5
    assert 10.0 <= t <= 11.0 and "Creeping along the plan" in c.events[-1][1]
    tw = c.drive_output(t)
    assert tw[0] == pytest.approx(0.0, abs=1e-9) and tw[1] > 0.1     # first onto the plan: sideways (+y), holonomic
    x, y = 0.0, -0.25
    for k in range(400):                                             # it moves as told
        tw = c.drive_output(t)
        if c.creep is None:
            break
        x, y = x + tw[0] * 0.1, y + tw[1] * 0.1
        c.pose = ("map", x, y, 0.0)
        t += 0.1
        c.tick(t)
    assert c.creep is None and tw == (0.0, 0.0, 0.0) and "Creep done" in c.events[-1][1]
    assert 1.4 <= x <= 1.7 and abs(y) < 0.1                          # along the plan ~1.5 m
    assert c.jam is None or (c.jam["x"] > 1.0 and c.jam["still"] < 5.0)   # headway: the jam check starts again
    assert c.mission == "GOTO" and c.stuck is None
    c.command("goto", {"x": 0.0, "y": 3.0})                          # a new goal ends any creep
    assert c.creep is None


def test_the_jam_check_runs_on_the_robots_clock():
    """Nov 25: the sim ran at ~0.35x real time and a robot driving across the room timed out on the wall clock."""
    c = ready()
    c.set_slam(mode="mapping", managed=True)
    c.command("explore")
    c.pose, c.pose_t = ("map", 2.0, 1.5, 0.0), 99.9
    c.tick(1.0, room())
    for k in range(200):                                              # 100 s of wall time, 35 s of robot time
        c.pose_t = 100.0 + 0.175 * k
        c.tick(1.5 + 0.5 * k)
    assert c.mission == "EXPLORE" and c.stuck is None
    for k in range(200, 900):                                         # 40 s of robot time: the opening skipped,
        c.pose_t = 100.0 + 0.175 * k                                  # 40 s more: stuck
        c.tick(1.5 + 0.5 * k, room())
        if c.mission != "EXPLORE":
            break
    assert 0.175 * k >= 80.0 and len(c.explore or {}) == 0
    assert c.mission == "IDLE" and c.stuck["why"].startswith("no headway")      # stuck at its start: it stops


def test_a_failed_drive_home_after_a_jam_stops_it():
    c = _exploring()
    c.pose = ("map", 2.0, 1.5, 0.0)
    for k in range(400):
        c.tick(1.5 + 0.5 * k, room())
        if c.homing:
            break
    assert c.homing
    c.on_nav_done(c.goal["seq"], "aborted", 60.0)
    fx = c.pop_fx()
    assert c.mission == "IDLE" and c.stuck is not None and ("set_mode", "IDLE", False) in fx


def test_stuck_already_at_home_it_stops():
    c = ready()
    c.places["home"] = [5.0, 5.0, 0.0]
    c.pose = ("odom", 0.0, 0.0, 0.0)                                   # no map pose: no start recorded
    c.command("goto", {"x": 2.0, "y": 0.0})
    c.pose = ("map", 5.2, 5.0, 0.0)                                    # within 0.5 m of 'home'
    for k in range(100):
        c.tick(0.5 * k)
    assert c.mission == "IDLE" and c.stuck is not None


def test_explore_stows_the_arm_and_waits_for_the_floor_camera():
    """Nov 25: the arm never read as stowed, the height map skipped every frame, and Explore drove to the den's edge."""
    c = ready(arm=True)
    c.set_slam(mode="mapping", managed=True)
    r = c.command("explore")
    assert ("arm", {"verb": "stop"}) in r.effects                     # the arm stows
    c.pose = ("map", 1.0, 1.5, 0.0)
    assert c.tick(1.0, room()) == () and c.explore["phase"] == "wait"    # no word from the height map yet
    c.on_floor_state({"blind": "arm out"}, 2.0)
    assert c.tick(2.0, room()) == () and "arm out" in c.task
    assert c.summary()["floor"] == {"blind": "arm out"}
    c.on_floor_state({"blind": None}, 3.0)
    fx = c.tick(3.0, room())
    assert c.explore["phase"] == "explore" and "nav_goal" in kinds(fx)
    for t in (3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0):               # the camera goes quiet on the way
        c.on_floor_state({"blind": "camera silent"}, t)
        fx = c.tick(t, room())
        if c.mission != "EXPLORE":
            break
    assert c.mission == "MANUAL" and t >= 3.5 + 3.0 - 0.01 and ("nav_cancel",) in fx
    assert c.events[-1][0] == "error" and "camera silent" in c.events[-1][1]


def test_explore_gives_up_waiting_for_a_blind_floor_camera():
    c = ready(arm=True)
    c.set_slam(mode="mapping", managed=True)
    c.command("explore")
    c.pose = ("map", 1.0, 1.5, 0.0)
    for t in range(1, 50):
        c.on_floor_state({"blind": "arm out"}, float(t))
        c.tick(float(t), room())
    assert c.mission == "MANUAL" and "can't see drops" in c.task
    c2 = ready()                                                     # no arm, no height map: nothing to wait for
    c2.set_slam(mode="mapping", managed=True)
    assert ("arm", {"verb": "stop"}) not in c2.command("explore").effects
    c2.pose = ("map", 1.0, 1.5, 0.0)
    c2.tick(1.0, room())
    assert c2.explore["phase"] == "explore" and c2.summary()["floor"] is None


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
    firmware = os.environ.get("OMNI_FIRMWARE_DIR", os.path.join(REPO, "firmware"))
    sys.path.insert(0, firmware)
    from fw_build_id import build_id
    core = os.path.join(firmware, "lib", "omni_fw")
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


def test_a_restarted_console_resumes_where_the_robot_is():
    import math
    from omni_console.slam import RESUME_S, pose_record, resume_pose
    # map -> odom: odom's origin at map (1, 2), turned 90 deg. base_link at odom (1, 0, 0) = map (1, 3, 90 deg)
    rec = pose_record("house", (1.0, 3.0, math.pi / 2), (1.0, 0.0, 0.0), 100.0, 50_000_000)
    mcu = 56_000_000                                                          # the MCU's clock 6 s on: the same boot
    p = resume_pose(rec, "house", (1.0, 0.0, 0.0, 99.0), 110.0, mcu)          # it hasn't moved
    assert p == pytest.approx((1.0, 3.0, math.pi / 2), abs=1e-3)
    p = resume_pose(rec, "house", (2.0, 0.0, 0.5), 110.0, mcu)                # odom says 1 m on, turned 0.5 rad
    assert p == pytest.approx((1.0, 4.0, math.pi / 2 + 0.5), abs=1e-3)
    assert resume_pose(rec, "flat", (1.0, 0.0, 0.0), 110.0, mcu) is None      # another map
    assert resume_pose(rec, "house", (1.0, 0.0, 0.0), 100.0 + RESUME_S + 1, mcu) is None   # a reboot, not a restart
    assert resume_pose(rec, "house", None, 110.0, mcu) is None                # no odometry yet
    assert resume_pose({}, "house", (1.0, 0.0, 0.0), 110.0, mcu) is None
    # the MCU booted again (or a new sim session): its clock, and its odometry, started over
    assert resume_pose(rec, "house", (0.0, 0.0, 0.0), 110.0, 3_000_000) is None
    assert resume_pose(rec, "house", (0.0, 0.0, 0.0), 110.0, None) is None
    assert resume_pose(rec, "house", (0.0, 0.0, 0.0), 110.0, 70_000_000) is None   # 20 s on in 10 s: not this boot


def test_map_image(tmp_path):
    from omni_console.slam import write_map_image
    data = [-1, 0, 100, 0, 0, 0]                                     # 3 x 2, row 0 at the origin
    write_map_image(tmp_path / "m", 3, 2, 0.05, (1.0, -2.0, 0.0), data)
    raw = (tmp_path / "m.pgm").read_bytes()
    assert raw.startswith(b"P5\n") and raw.endswith(bytes([254, 254, 254, 205, 254, 0]))   # top row = grid row 1
    y = (tmp_path / "m.yaml").read_text()
    assert "image: m.pgm" in y and "resolution: 0.0500" in y and "origin: [1.0000, -2.0000, 0.0000]" in y
