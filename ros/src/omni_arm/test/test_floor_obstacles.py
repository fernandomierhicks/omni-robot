"""floor_obstacles.py (design.md 4.5, 4.11): Nav2's view of the floor from the D435, on synthetic depth. No ROS.

    python -m pytest -q ros/src/omni_arm/test/test_floor_obstacles.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from omni_arm import floor_obstacles as fo  # noqa: E402
from omni_arm import floor_vision as fv  # noqa: E402
from omni_arm.kinematics import ArmConfig, config_path  # noqa: E402
from omni_arm.pick import MJ_TO_OPTICAL  # noqa: E402

WH = (480, 270)                                  # the D435's streamed depth mode (params.d435_wh)
FY = (WH[1] / 2) / math.tan(math.radians(58.0) / 2)
K = (FY, FY, WH[0] / 2, WH[1] / 2)


def t_cam() -> np.ndarray:
    c = ArmConfig(config_path("d435")).cameras[0]
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = np.array(c["R"]), c["pos"]
    return T @ MJ_TO_OPTICAL


def pitched(T: np.ndarray, deg: float) -> np.ndarray:
    """The camera with the base pitched nose-down by deg (about base_link's y axis)."""
    a = math.radians(deg)
    P = np.eye(4)
    P[:3, :3] = [[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]]
    return P @ T


def depth(T, boxes, noise=0.0, seed=1) -> np.ndarray:
    """The floor plus axis-aligned boxes (x0, x1, y0, y1, h) as the camera at T sees them (ray casting)."""
    W, H = WH
    fx, fy, cx, cy = K
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], axis=-1) @ T[:3, :3].T
    o = T[:3, 3]
    down = np.minimum(rays[..., 2], -1e-9)
    s_best = np.where(rays[..., 2] < 0, -o[2] / down, np.inf)
    for x0, x1, y0, y1, h in boxes:
        for s in ((h - o[2]) / down,):                                   # the top face
            p = o + rays * s[..., None]
            hit = (rays[..., 2] < 0) & (p[..., 0] > x0) & (p[..., 0] < x1) & (p[..., 1] > y0) & (p[..., 1] < y1)
            s_best = np.where(hit & (s < s_best), s, s_best)
        # the four sides, for tall things
        for axis, plane, lo, hi in ((0, x0, y0, y1), (0, x1, y0, y1), (1, y0, x0, x1), (1, y1, x0, x1)):
            r = rays[..., axis]
            s = (plane - o[axis]) / np.where(np.abs(r) > 1e-9, r, 1e-9)
            p = o + rays * s[..., None]
            q = p[..., 1 - axis]
            hit = (s > 0) & (q > lo) & (q < hi) & (p[..., 2] > 0) & (p[..., 2] < h)
            s_best = np.where(hit & (s < s_best), s, s_best)
    z = s_best
    if noise:
        z = z * (1 + np.random.default_rng(seed).normal(0, noise, z.shape))
    z = np.where(np.isfinite(z) & (z < 10.0), z, np.nan)
    return z.astype(np.float32)


def run(boxes, pitch=0.0, noise=0.002, seed=1):
    """(marks, floor, detections): the things and the tall marks together, the seen floor."""
    T = t_cam()
    z = depth(pitched(T, pitch), boxes, noise, seed)
    things, tall, floor, dets = fo.floor_obstacles(fv.DepthFrame(0.0, z, K, T), rng=np.random.default_rng(0))
    return np.concatenate([things, tall]), floor, dets


def near(pts, x, y, r=0.06) -> int:
    return int(np.sum(np.hypot(pts[:, 0] - x, pts[:, 1] - y) < r))


def test_low_toys_are_marked_and_the_floor_is_seen():
    """A crayon (12 mm), a brick (15 mm) and a 30 mm cube on the floor 0.6-1.1 m ahead: all marked (the raw layer's
    4 cm floor marked none); the bare floor isn't, it is seen out to ~2.7 m, and never on or right beside a toy."""
    toys = [(0.60, 0.69, -0.20, -0.188, 0.012), (0.80, 0.848, 0.10, 0.124, 0.015), (1.10, 1.13, -0.05, -0.02, 0.03)]
    marks, clears, dets = run(toys)
    for x0, x1, y0, y1, _ in toys:
        assert near(marks, (x0 + x1) / 2, (y0 + y1) / 2) >= 2, (x0, y0)
    stray = [p for p in marks if all(math.hypot(p[0] - (b[0] + b[1]) / 2, p[1] - (b[2] + b[3]) / 2) > 0.1 for b in toys)]
    assert stray == []
    assert len(clears) > 300 and near(clears, 2.0, 0.0, 0.3) > 0
    for x0, x1, y0, y1, _ in toys:
        assert near(clears, (x0 + x1) / 2, (y0 + y1) / 2, 0.06) == 0, (x0, y0)


def test_a_things_shadow_and_the_views_edge_are_not_seen_floor():
    """A toy just outside the view (under it, past its side, or in another's shadow) must keep its mark: a 5 cm map
    cell holding a seen-floor point (up to 3.75 cm past it) stays inside the seen floor."""
    T = t_cam()
    marks, clears, _ = run([(1.0, 1.04, -0.02, 0.02, 0.04)])                # a 40 mm cube 1 m ahead
    cx, cz = T[0, 3], T[2, 3]                                               # its shadow ends where the camera's ray over the
    end = cx + (1.04 - cx) * cz / (cz - 0.04)                               # cube's back top edge meets the floor (1.27 m on
    sh = (clears[:, 0] > 1.04) & (clears[:, 0] < end - 0.04) & (np.abs(clears[:, 1]) < 0.03)   # the riser, 1.53 on the nose)
    assert not sh.any()                                                       # its shadow on the floor
    P = fv.points(fv.DepthFrame(0.0, depth(T, []), K, T))
    near_edge = np.min(np.hypot(P[:, 0], P[:, 1]))                         # ~0.31 m (riser; nose 0.34): the view's near edge
    assert np.min(np.hypot(clears[:, 0], clears[:, 1])) > near_edge + 0.035
    ang = np.arctan2(P[:, 1] - T[1, 3], P[:, 0] - T[0, 3])                 # the field's sides
    ca = np.arctan2(clears[:, 1] - T[1, 3], clears[:, 0] - T[0, 3])
    assert ca.max() < ang.max() and ca.min() > ang.min()


def test_the_bases_pitch_marks_nothing_on_a_bare_floor():
    """2 deg nose down (braking on the pods; odom -> base_link carries none): a flat-z threshold of 4 cm would have
    been 3.5 cm off at 1 m; measured against the floor the frame sees, nothing."""
    for pitch in (-2.0, 0.0, 2.0):
        marks, clears, _ = run([], pitch)
        assert len(marks) == 0, pitch
        assert len(clears) > 300


def test_a_toy_still_marks_when_the_base_pitches():
    marks, _, _ = run([(0.80, 0.848, 0.10, 0.124, 0.015)], pitch=2.0)
    assert near(marks, 0.824, 0.112) >= 2


def test_a_rugs_edge_is_floor_and_a_chair_leg_is_an_obstacle():
    rug = (0.55, 2.5, -2.5, -0.05, 0.012)                                # a 12 mm rug to the right
    marks, _, _ = run([rug], noise=0.003)
    assert len(marks) == 0
    leg = (0.90, 0.94, 0.20, 0.24, 0.45)                                 # a chair leg: tall, marked as it is
    marks, _, _ = run([rug, leg], noise=0.003)
    assert near(marks, 0.92, 0.22, 0.08) >= 1
    assert all(math.hypot(p[0] - 0.92, p[1] - 0.22) < 0.1 for p in marks)


def test_too_little_floor_gives_nothing():
    T = t_cam()
    z = np.full((WH[1], WH[0]), np.nan, np.float32)
    things, tall, floor, dets = fo.floor_obstacles(fv.DepthFrame(0.0, z, K, T))
    assert len(things) == len(tall) == len(floor) == 0 and dets == []


# -- the grid over many frames ---------------------------------------------------------------------------------------
def grid():
    g = fo.ObstacleGrid()
    assert g.geometry(0.05, -2.0, -1.0, 80, 40)                          # a 4 x 2 m map from (-2, -1)
    return g


def cell(g, x, y):
    return g.kind[int((y - g.origin[1]) / g.res), int((x - g.origin[0]) / g.res)]


def test_a_mark_stays_until_its_floor_is_seen():
    g = grid()
    toy, none = np.array([[0.51, 0.22]]), np.zeros((0, 2))
    assert fo.THING_HITS == 2
    assert not g.update(0.0, toy, none, none) and cell(g, 0.51, 0.22) == fo.EMPTY       # once: not yet
    assert g.update(0.2, toy, none, none) and cell(g, 0.51, 0.22) == fo.THING
    # out of view for a long while (no floor seen there): still marked, and never lapses
    assert not g.update(100.0, none, none, np.array([[1.52, 0.52]]))
    assert cell(g, 0.51, 0.22) == fo.THING
    # its floor seen again: cleared; seen with the toy still there: the toy wins
    assert not g.update(101.0, toy, none, toy) and cell(g, 0.51, 0.22) == fo.THING
    assert g.update(102.0, none, none, toy) and cell(g, 0.51, 0.22) == fo.EMPTY
    # a patch that comes and goes (a thick rug's edge in the depth noise): its floor seen between sightings, never marked
    for k in range(10):
        g.update(200.0 + k, toy if k % 2 == 0 else none, none, none if k % 2 == 0 else toy)
    assert cell(g, 0.51, 0.22) == fo.EMPTY
    occ = g.occupancy()
    assert occ.dtype == np.int8 and len(occ) == 80 * 40 and set(np.unique(occ)) == {-1}


def test_tall_marks_lapse_and_things_dont_turn_tall():
    g = grid()
    none = np.zeros((0, 2))
    for t in (0.0, 0.2):
        g.update(t, np.array([[0.0, 0.0]]), np.array([[1.0, 0.5]]), none)
    g.update(5.0, none, np.array([[0.0, 0.0]]), none)                     # a tall bit over the toy's cell
    assert cell(g, 0.0, 0.0) == fo.THING and cell(g, 1.0, 0.5) == fo.TALLMARK
    g.update(fo.TALL_TTL + 1.0, none, none, none)
    assert cell(g, 1.0, 0.5) == fo.EMPTY and cell(g, 0.0, 0.0) == fo.THING


def test_forget_clears_a_disc_and_the_grid_follows_the_maps_geometry():
    g = grid()
    none = np.zeros((0, 2))
    for t in (0.0, 0.2):
        g.update(t, np.array([[0.3, 0.0], [1.5, 0.0]]), none, none)
    assert g.forget(0.0, 0.0, 0.5) and cell(g, 0.3, 0.0) == fo.EMPTY and cell(g, 1.5, 0.0) == fo.THING
    assert g.marked() > 0
    # the map grows 1 m to the left (slam's map while mapping): the marks keep their place in the world
    assert g.geometry(0.05, -3.0, -1.0, 100, 40) and cell(g, 1.5, 0.0) == fo.THING and g.marked() == 1
    assert not g.geometry(0.05, -3.0, -1.0, 100, 40)
    # points off the map are ignored
    assert not g.update(1.0, np.array([[50.0, 50.0]]), none, none)


def test_a_long_low_strip_is_floor_but_a_long_tall_thing_isnt():
    """A carpet's edge seen through a door, a threshold: long (floor_vision's 'big') and low, the robot crosses it
    (Nov 5: such a strip closed the bedroom's door). A 30 cm long box 4 cm tall is an obstacle."""
    strip = (0.90, 0.92, -0.20, 0.20, 0.012)                            # 40 cm long, 2 cm wide, 12 mm
    marks, _, dets = run([strip])
    assert any(d.big for d in dets) or not dets
    assert near(marks, 0.91, 0.0, 0.25) == 0
    box = (0.90, 0.95, -0.15, 0.15, 0.04)
    marks, _, _ = run([box])
    assert near(marks, 0.92, 0.0, 0.2) >= 3


def test_things_far_off_wait_until_nearer():
    """A 20 mm block ~1.6 m from the camera isn't marked yet; at ~1.0 m it is (THING_RANGE 1.5 m)."""
    T = t_cam()
    for x, marked in ((1.80, False), (1.20, True)):
        marks, _, dets = run([(x - 0.02, x + 0.02, 0.10, 0.14, 0.02)])         # 20 mm: a thing, not tall
        assert (math.hypot(x - T[0, 3], 0.12 - T[1, 3]) < fo.THING_RANGE) == marked
        assert (near(marks, x, 0.12, 0.08) > 0) == marked, x
