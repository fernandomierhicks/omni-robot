"""The tidy's coverage (coverage.py), without the sim or ROS: look rounds in a synthetic room until nothing reachable is
left to check."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from omni_arm import coverage as cv  # noqa: E402

NEAR, FAR = 0.36, cv.CoverParams().check_range      # the look ring: the camera at the base's centre
BOX = (0.9, 1.3, -0.6, 0.6)                 # x0, x1, y0, y1: a sofa ahead, 0.4 x 1.2 m


def in_box(x, y, box=BOX):
    return (x > box[0]) & (x < box[1]) & (y > box[2]) & (y < box[3])


def look_round(pose, box=BOX, step=0.02):
    """A full turn's depth points (base_link, the base not turned) with their heights: floor within 1.6 m of the base,
    except where the box hides it; the box's near faces, 0.4 m tall."""
    g = np.arange(-1.6, 1.6 + 1e-9, step)                 # the depth sees farther than it checks
    X, Y = np.meshgrid(g, g)
    X, Y = X.ravel(), Y.ravel()
    r = np.hypot(X, Y)
    keep = (r >= NEAR) & (r <= 1.6)
    X, Y = X[keep], Y[keep]
    wx, wy = pose[0] + X, pose[1] + Y
    hidden = np.zeros(len(X), bool)
    for t in np.linspace(0.05, 0.98, 25):                 # the ray from the base to the point crosses the box
        hidden |= in_box(pose[0] + t * X, pose[1] + t * Y, box)
    floor = ~hidden & ~in_box(wx, wy, box)
    on_box = in_box(wx, wy, box) & ~np.array([in_box(pose[0] + t * X, pose[1] + t * Y, box) for t in (0.9,)]).any(0)
    P = np.c_[np.r_[X[floor], X[on_box]], np.r_[Y[floor], Y[on_box]], np.r_[np.zeros(floor.sum()), np.full(on_box.sum(), 0.4)]]
    return P, P[:, 2].copy()


def explore(c, pose, n_max=30):
    visited, plans = [pose[:2]], []
    for _ in range(n_max):
        P, h = look_round(pose)
        c.update((pose[0], pose[1], 0.0), P, h, (0.0, 0.0))
        c.end_look(pose[0], pose[1], NEAR, FAR)
        p = c.plan(pose, NEAR, FAR)
        if p is None:
            return visited, plans
        plans.append((p, c.free.copy(), c.blocked.copy()))
        pose = (p.goal[0], p.goal[1], 0.0)
        c.occupy(pose[0], pose[1])
        visited.append(pose[:2])
    raise AssertionError("never done")


def test_a_look_checks_near_floor_and_blocks_tall_things():
    c = cv.Coverage((0.0, 0.0), 2.0)
    P, h = look_round((0.0, 0.0, 0.0))
    c.update((0.0, 0.0, 0.0), P, h, (0.0, 0.0))
    assert c.checked[c.cell(0.5, 0.0)] and c.checked[c.cell(0.0, -1.0)]
    assert c.blocked[c.cell(0.92, 0.0)] and not c.free[c.cell(0.92, 0.0)]
    assert not c.checked[c.cell(1.6, 0.0)]                     # beyond the check range
    far = cv.Coverage((0.0, 0.0), 2.0, cv.CoverParams(check_range=0.6))
    far.update((0.0, 0.0, 0.0), P, h, (0.0, 0.0))
    assert far.free[far.cell(0.0, 1.0)] and not far.checked[far.cell(0.0, 1.0)]   # seen as floor, too far to count


def test_a_thing_the_detector_turned_down_stays_to_look_at():
    """A 2 cm tall patch the look saw but floor_vision didn't take for a thing (its ring cut off, say): not checked,
    still to look at though it blocks the base; once a look finds it (a detection there), checked."""
    from types import SimpleNamespace
    P, h = look_round((0.0, 0.0, 0.0), box=(9, 9, 9, 9))
    toy = (np.abs(P[:, 0] - 0.7) < 0.02) & (np.abs(P[:, 1] - 0.3) < 0.02)
    h[toy] = 0.02
    for dets, checked in (((), False), ((SimpleNamespace(x=0.7, y=0.3),), True)):
        c = cv.Coverage((0.0, 0.0), 2.0)
        c.update((0.0, 0.0, 0.0), P, h, (0.0, 0.0), dets)
        ij = c.cell(0.7, 0.3)
        assert c.checked[ij] == checked and c.todo()[ij] != checked
        assert c.checked[c.cell(0.7, 0.0)]                      # the bare floor beside it


def test_the_tidy_looks_until_the_region_is_covered_round_the_sofa():
    c = cv.Coverage((0.0, 0.0), 2.0)
    c.occupy(0.0, 0.0, NEAR + 0.05)
    visited, plans = explore(c, (0.0, 0.0, 0.0))
    cov = c.coverage()
    behind = c.inside & ~in_box(c.X, c.Y) & (c.X > BOX[1]) & (np.abs(c.Y) < BOX[3] - 0.1)
    assert c.checked[behind].mean() > 0.9, cov                 # it went round the sofa to look behind it
    assert cov["share"] > 0.95, cov
    assert len(visited) <= 10, visited                         # 9 looks for 12.6 m^2 (Nov 6)
    for (sx, sy), (p, free, blocked) in zip(visited, plans):  # only through floor seen free then, the bumper off the sofa
        for (x0, y0), (x1, y1) in zip([(sx, sy)] + p.path[:-1], p.path):
            for t in np.linspace(0, 1, 30):
                x, y = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
                gap = math.hypot(max(BOX[0] - x, 0, x - BOX[1]), max(BOX[2] - y, 0, y - BOX[3]))
                assert gap > c.p.robot_radius, (x, y, gap)
                assert free[c.cell(x, y)], (x, y)


def test_floor_it_cant_reach_or_see_is_left_as_unseen_not_looked_for_forever():
    walled = (0.9, 1.0, -3.0, 3.0)                             # a wall across the region: nothing past it is reachable
    c = cv.Coverage((0.0, 0.0), 2.0)
    c.occupy(0.0, 0.0, NEAR + 0.05)
    pose = (0.0, 0.0, 0.0)
    for _ in range(30):
        P, h = look_round(pose, walled)
        c.update((pose[0], pose[1], 0.0), P, h, (0.0, 0.0))
        c.end_look(pose[0], pose[1], NEAR, FAR)
        p = c.plan(pose, NEAR, FAR)
        if p is None:
            break
        assert p.goal[0] < walled[0] - 0.2
        pose = (p.goal[0], p.goal[1], 0.0)
        c.occupy(pose[0], pose[1])
    else:
        raise AssertionError("never done")
    cov = c.coverage()
    assert cov["unchecked"] > 1.0 and cov["share"] < 0.9, cov                  # past the wall: reported, not looked for
    assert math.isclose(cov["share"], cov["checked"] / (cov["checked"] + cov["unchecked"]), abs_tol=0.02)
