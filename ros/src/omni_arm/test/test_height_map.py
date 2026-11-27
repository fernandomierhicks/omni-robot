"""height_map.py (design.md 4.5, 4.11): the D435's fused height map for navigation, on synthetic depth seen from a
robot driving past. No ROS.

    python -m pytest -q ros/src/omni_arm/test/test_height_map.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from omni_arm import floor_vision as fv  # noqa: E402
from omni_arm import height_map as hm  # noqa: E402
from omni_arm.floor_obstacles import to_frame  # noqa: E402
from test_floor_obstacles import K, depth, t_cam  # noqa: E402

RUG = (0.6, 2.2, 0.35, 1.2, 0.012)          # 1.6 x 0.85 m, 12 mm (a level step: drivable)
SHOE = (1.6, 1.88, -0.45, -0.35, 0.11)      # 28 x 10 cm, 11 cm: under the lidar's plane, an obstacle
BRICK = (1.20, 1.248, -0.06, -0.036, 0.015)  # a low thing (a 48 x 24 x 15 mm brick)
SCENE = [RUG, SHOE, BRICK]


def base_pose(x, y=0.0, yaw=0.0):
    T = np.eye(4)
    T[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    T[:2, 3] = (x, y)
    return T


def drive(m: hm.HeightMap, boxes, poses, t0=0.0, noise=0.003):
    """Frames from the robot at each (x, y, yaw) (world = map), 0.2 s apart."""
    Tc = t_cam()
    t = t0
    for k, (x, y, yaw) in enumerate(poses):
        z = depth(base_pose(x, y, yaw) @ Tc, boxes, noise, seed=k + 1)
        pts = hm.frame_points(fv.DepthFrame(t, z, K, Tc), rng=np.random.default_rng(k))
        if pts is not None:
            pts = np.c_[to_frame(pts[:, :2], (x, y, yaw)), pts[:, 2]]
        m.update(t, pts, to_frame(Tc[None, :2, 3], (x, y, yaw))[0])
        t += 0.2
    return t


def fresh():
    m = hm.HeightMap()
    m.geometry(0.05, -1.0, -2.0, 100, 80)                  # 5 x 4 m from (-1, -2)
    return m


def cls(m, c, x, y):
    i, j, _ = m.ij(np.array([[x, y]]))
    return hm.NAMES[c[i[0], j[0]]]


def test_a_rug_is_a_step_a_shoe_an_obstacle_a_crayon_low_and_only_the_shoe_blocks():
    m = fresh()
    t = drive(m, SCENE, [(x, 0.0, 0.0) for x in np.linspace(-0.6, 0.4, 6)])
    c = m.classes(t)
    assert cls(m, c, 1.4, 0.8) == "step"                    # the rug's middle, and its edge
    assert cls(m, c, 0.62, 0.8) in ("step", "floor")
    assert cls(m, c, 1.74, -0.40) == "obstacle"
    assert cls(m, c, 1.224, -0.048) == "low"
    assert cls(m, c, 1.0, -0.8) == "floor"
    assert cls(m, c, -0.9, 1.9) == "unknown"                # never seen
    occ = m.occupancy(c).reshape(c.shape)
    i, j = np.nonzero(occ == 100)
    x, y = m.xy(i, j)
    assert len(i) and np.all((x > SHOE[0] - 0.1) & (x < SHOE[1] + 0.1) & (y > SHOE[2] - 0.1) & (y < SHOE[3] + 0.1))


def test_a_mark_outlives_its_view_then_goes_stale_and_a_moved_thing_clears():
    m = fresh()
    t = drive(m, SCENE, [(x, 0.0, 0.0) for x in np.linspace(-0.6, 0.4, 6)])
    t = drive(m, SCENE, [(1.0, 0.0, math.pi)] * 4, t0=t)    # turned away: the shoe out of view
    assert cls(m, m.classes(t), 1.74, -0.40) == "obstacle"
    assert cls(m, m.classes(t + hm.OBS_TTL + 1), 1.74, -0.40) == "unknown"
    t = drive(m, [RUG, BRICK], [(0.3, 0.0, 0.0)] * 6, t0=t)   # back, the shoe gone: floor again
    assert cls(m, m.classes(t), 1.74, -0.40) == "floor"


def test_a_drop_ahead():
    """Stairs down: the floor beyond x 1.0 m is 0.18 m lower (a box of negative height can't be drawn; the floor of
    the near part is a box 0.18 m tall and the robot stands on it)."""
    m = fresh()
    near_floor = (-3.0, 1.0, -3.0, 3.0, 0.18)
    Tc = t_cam()
    t = 0.0
    for k, x in enumerate(np.linspace(0.0, 0.6, 10)):
        T = base_pose(x) @ Tc
        T[2, 3] += 0.18                                    # the robot on the upper floor
        z = depth(T, [near_floor], 0.003, seed=k + 3)
        pts = hm.frame_points(fv.DepthFrame(t, z, K, Tc), rng=np.random.default_rng(k))
        m.update(t, np.c_[to_frame(pts[:, :2], (x, 0.0, 0.0)), pts[:, 2]], to_frame(Tc[None, :2, 3], (x, 0.0, 0.0))[0])
        t += 0.2
    c = m.classes(t)
    assert cls(m, c, 1.8, 0.0) == "drop" and cls(m, c, 0.8, 0.0) == "floor"
    assert cls(m, c, 1.1, 0.0) == "drop"                    # just past the edge: unseen from above, the drop's shadow
    assert m.occupancy(c).reshape(c.shape)[m.ij(np.array([[1.8, 0.0]]))[0][0], m.ij(np.array([[1.8, 0.0]]))[1][0]] == 100


LIP, RAMP_RUN, RAMP_DEG, UP = 0.0127, 0.152, 20.0, 0.068    # the home's family room entries (Nov 25)


def ramp_down(edge=1.0):
    """The robot's floor UP high until x = edge, then a 20 deg ramp down over RAMP_RUN and a 13 mm lip to the floor
    (stairs of 4 mm boxes: a ramp to the camera)."""
    boxes = [(-3.0, edge, -3.0, 3.0, UP)]
    n = 40
    for k in range(n):
        x0 = edge + k * RAMP_RUN / n
        boxes.append((x0, x0 + RAMP_RUN / n + 1e-3, -3.0, 3.0, UP - (k + 0.5) * RAMP_RUN / n * math.tan(math.radians(RAMP_DEG))))
    return boxes


def drive_raised(m, boxes, xs, lift, t0=0.0, seed=3):
    """Frames from the robot at each x (y 0, facing +x) standing lift up (on a box floor)."""
    Tc = t_cam()
    t = t0
    for k, x in enumerate(xs):
        T = base_pose(x) @ Tc
        T[2, 3] += lift
        z = depth(T, boxes, 0.003, seed=k + seed)
        pts = hm.frame_points(fv.DepthFrame(t, z, K, Tc), rng=np.random.default_rng(k))
        m.update(t, np.c_[to_frame(pts[:, :2], (x, 0.0, 0.0)), pts[:, 2]], to_frame(Tc[None, :2, 3], (x, 0.0, 0.0))[0])
        t += 0.2
    return t


def test_a_step_seen_only_from_far_is_suspected_and_slows_the_robot():
    """The stairs of test_a_drop_ahead seen from 1-1.5 m only: not yet a drop (a ramp would look the same), unknown,
    and the speed limit is V_EDGE short of it."""
    m = fresh()
    t = drive_raised(m, [(-3.0, 1.0, -3.0, 3.0, 0.18)], np.linspace(-0.8, -0.4, 6), 0.18)
    c = m.classes(t)
    assert cls(m, c, 1.8, 0.0) == "unknown" and not (c == hm.DROP).any()
    assert m.suspect[m.ij(np.array([[1.8, 0.0]]))[0][0], m.ij(np.array([[1.8, 0.0]]))[1][0]]
    v, why = m.speed_limit(c, t, 0.6, 0.0, 0.0, 2.0)
    assert v == hm.V_EDGE and "drop" in why


def test_a_ramp_down_is_floor_and_a_step_the_same_height_is_a_drop():
    """The family room's entry seen from its raised floor, driving up to the edge: the floor past the ramp (68 mm
    lower) is floor; the same 68 mm as a sharp step is a drop."""
    m = fresh()
    t = drive_raised(m, ramp_down(), np.linspace(-0.4, 0.55, 12), UP)
    c = m.classes(t)
    assert cls(m, c, 1.6, 0.0) == "floor" and cls(m, c, 1.6, 0.4) == "floor"
    assert not (c == hm.DROP).any() and not (c == hm.OBSTACLE).any()
    m = fresh()
    t = drive_raised(m, [(-3.0, 1.0, -3.0, 3.0, UP)], np.linspace(-0.4, 0.55, 12), UP)
    assert cls(m, m.classes(t), 1.6, 0.0) == "drop"


def test_a_ramp_up_is_a_raised_floor_not_an_obstacle():
    """From the lower floor: a 13 mm lip, the ramp up, the raised floor (68 mm) past x 1.15: a step, drivable; a box
    on it is still an obstacle."""
    boxes = [(1.0, 3.0, -3.0, 3.0, LIP)]
    n = 40
    for k in range(n):
        x0 = 1.0 + k * RAMP_RUN / n
        boxes.append((x0, 3.0, -3.0, 3.0, LIP + (k + 1) * RAMP_RUN / n * math.tan(math.radians(RAMP_DEG))))
    box = (2.0, 2.2, 0.3, 0.5, UP + 0.08)
    m = fresh()
    t = drive(m, boxes + [box], [(x, 0.0, 0.0) for x in np.linspace(-0.4, 0.6, 10)])
    c = m.classes(t)
    assert cls(m, c, 1.6, 0.0) == "step" and cls(m, c, 1.08, 0.0) in ("floor", "step")
    assert cls(m, c, 2.1, 0.4) == "obstacle"
    occ = c == hm.OBSTACLE
    i, j = np.nonzero(occ)
    x, y = m.xy(i, j)
    assert np.all((x > 1.9) & (x < 2.3) & (y > 0.2) & (y < 0.6))     # only the box


def _surface_pts(x0, x1, y0, y1, height, rng, res=0.05, xy=False):
    """4 points per 5 cm cell of the floor patch x0..x1, y0..y1 (map frame), heights height(x) (height(x, y) with
    xy) (+1 mm noise)."""
    xs, ys = np.meshgrid(np.arange(x0, x1, res / 2) + res / 4, np.arange(y0, y1, res / 2) + res / 4)
    x, y = xs.ravel(), ys.ravel()
    return np.c_[x, y, (height(x, y) if xy else height(x)) + rng.normal(0.0, 0.001, x.size)]


def test_heights_share_one_datum_across_floors():
    """Nov 25 (world:=home, hm12): the family room seen from on top was stored against its own floor, so from the
    hallway below it read level with the hallway, and the ramp's top, glimpsed through the doorway, became a 68 mm
    ridge: an obstacle across the doorway, no path home. Each frame is now registered on the map."""
    rise = lambda x: np.clip((x - 1.0) / RAMP_RUN, 0.0, 1.0) * UP                 # the ramp, 1.0..1.15, up UP
    rng = np.random.default_rng(0)
    for reg in (False, True):
        m = fresh()
        if not reg:
            m_reg, hm.REG_MIN = hm.REG_MIN, 10 ** 9
        try:
            t = 0.0
            for _ in range(4):                    # from the raised floor: the lower floor, the ramp, the raised floor
                pts = _surface_pts(0.2, 2.6, -1.0, 1.0, lambda x: rise(x) - UP, rng)
                m.update(t, pts, (2.0, 0.0))
                t += 0.2
            for _ in range(4):                    # from the lower floor, its doorway's sliver: the ramp's top and
                low = _surface_pts(-0.5, 1.0, -1.0, 1.0, lambda x: 0.0 * x, rng)        # 10 cm of the raised floor
                sliver = _surface_pts(1.0, 1.25, -0.2, 0.2, rise, rng)
                m.update(t, np.r_[low, sliver], (0.0, 0.0))
                t += 0.2
            c = m.classes(t)
        finally:
            if not reg:
                hm.REG_MIN = m_reg
        if reg:
            assert not (c == hm.OBSTACLE).any() and abs(m.z + UP) < 0.005 and m.n_unreg == 0
            assert cls(m, c, 1.2, 0.0) == "step" and cls(m, c, 2.0, 0.5) == "step" and cls(m, c, 0.5, 0.0) == "floor"
        else:                                     # the old map: the raised floor read level with the robot's
            assert cls(m, c, 2.0, 0.5) == "floor"
    m = fresh()                                   # a frame overlapping the map but not agreeing is not stored
    m.update(0.0, _surface_pts(0.0, 2.0, -1.0, 1.0, lambda x: 0.0 * x, rng), (0.0, 0.0))
    m.update(0.1, _surface_pts(0.0, 2.0, -1.0, 1.0, lambda x: 0.0 * x, rng), (0.0, 0.0))
    p80 = m.p80.copy()
    m.update(0.2, _surface_pts(0.0, 2.0, -1.0, 1.0, lambda x: 0.03 * (np.floor(x / 0.05) % 3), rng), (0.0, 0.0))
    assert m.n_unreg == 1 and np.array_equal(m.p80, p80)


def test_a_raised_floors_rough_edge_is_not_an_obstacle():
    """Nov 25 (hm14): the ramp's top cells, half slope half platform (not a smooth surface, so not part of the raised
    floor), stood ~34 mm over the two floors' mean: obstacles, and inflated they closed the doorway. A box on the lower
    floor beside the ramp is still one."""
    rng = np.random.default_rng(1)

    def h(x, y):                                  # the ramp 1.0..1.15; every third cell at its top rough
        z = np.where(x < 1.0, 0.0, np.where(x < 1.15, (x - 1.0) / 0.15 * UP, UP))
        rough = (x >= 1.10) & (x < 1.15) & (np.floor(y / 0.05) % 3 == 0)
        return np.where(rough & (np.arange(x.size) % 2 == 0), 0.02, z)    # half those cells' points low

    for box in (False, True):
        m = fresh()
        for k in range(5):
            pts = _surface_pts(-0.5, 1.35, -1.0, 1.0, h, rng, xy=True)   # the raised floor: a strip, as through
            if box:                                                           # a doorway from below                                                       # 12 cm, 0.10 x 0.10 m, at x 0.80
                pts = np.r_[pts, _surface_pts(0.75, 0.85, 0.3, 0.4, lambda x: 0.12 + 0.0 * x, rng)]
                keep = ~((pts[:, 0] > 0.75) & (pts[:, 0] < 0.85) & (pts[:, 1] > 0.3) & (pts[:, 1] < 0.4)
                         & (pts[:, 2] < 0.1))
                pts = pts[keep]
            m.update(0.2 * k, pts, (0.0, 0.0))
        c = m.classes(1.0)
        assert cls(m, c, 1.12, 0.025) != "obstacle" and cls(m, c, 1.3, 0.0) == "step"
        occ = c == hm.OBSTACLE
        if box:
            i, j = np.nonzero(occ)
            x, y = m.xy(i, j)
            assert occ.any() and np.all((x > 0.7) & (x < 0.9) & (y > 0.25) & (y < 0.45))
        else:
            assert not occ.any()


def test_nothing_is_marked_where_the_robot_stands():
    """Nov 25 (hm15): a mark 0.25 m from the robot's centre put its own cell in collision (inscribed 0.24 m): Nav2
    planned nowhere and its backup and spin refused to move. Marks within FOOTPRINT_CLEAR of the robot are dropped."""
    rng = np.random.default_rng(2)
    m = fresh()
    for k in range(4):
        pts = _surface_pts(-0.5, 2.5, -1.0, 1.0, lambda x: 0.0 * x, rng)
        pts = pts[~((pts[:, 0] > 1.0) & (pts[:, 0] < 1.1) & (pts[:, 1] > 0.0) & (pts[:, 1] < 0.1))]
        pts = np.r_[pts, _surface_pts(1.0, 1.1, 0.0, 0.1, lambda x: 0.12 + 0.0 * x, rng)]     # a 12 cm thing
        m.update(0.2 * k, pts, (0.0, 0.0))
    assert cls(m, m.classes(1.0), 1.05, 0.05) == "obstacle"
    assert cls(m, m.classes(1.0, (1.4, 0.05)), 1.05, 0.05) == "obstacle"          # 0.35 m from its centre: kept
    assert cls(m, m.classes(1.0, (1.05 + hm.FOOTPRINT_CLEAR - 0.06, 0.05)), 1.05, 0.05) == "floor"


def test_speed_limits_by_how_sure_the_way_ahead_is():
    m = fresh()
    v_max = 2.0
    t = 0.0
    m.t_frame = t
    v, why = m.speed_limit(m.classes(t), t, 0.0, 0.0, 0.0, v_max)
    assert v < 0.6 and "unknown" in why                     # nothing seen yet: creep
    t = drive(m, [RUG], [(x, 0.0, 0.0) for x in np.linspace(-0.6, 0.4, 6)])
    c = m.classes(t)
    v, why = m.speed_limit(c, t, 0.0, 0.0, 0.0, v_max)
    assert v > 1.0, why                                     # floor and rug ahead, seen: fast
    v, _ = m.speed_limit(c, t, 0.0, 0.0, math.pi, v_max)    # backwards, into the unseen
    assert v < 0.5
    assert m.speed_limit(c, t + 2.0, 0.0, 0.0, 0.0, v_max)[0] == hm.V_BLIND     # no frames for 2 s
    t = drive(m, [RUG, BRICK], [(x, 0.0, 0.0) for x in np.linspace(-0.6, 0.4, 6)], t0=t)
    c = m.classes(t)
    v, why = m.speed_limit(c, t, 0.4, 0.0, 0.0, v_max)
    assert hm.V_LOW < v < 1.0 and "low" in why               # a low thing ~0.8 m ahead: braking for it
    v, why = m.speed_limit(c, t, 0.85, 0.0, 0.0, v_max)
    assert v == hm.V_LOW and "low" in why                   # at it
    v2, why2 = m.speed_limit(c, t, 0.4, 0.9, 0.0, v_max)     # the same thing 0.9 m to the side: no low limit
    assert "low" not in why2


def test_an_ais_label_overrides_until_the_cell_changes():
    m = fresh()
    t = drive(m, SCENE, [(x, 0.0, 0.0) for x in np.linspace(-0.6, 0.4, 6)])
    assert cls(m, m.classes(t), 1.224, -0.048) == "low"
    assert m.set_label(1.224, -0.048, 0.08, "rug") > 0
    assert cls(m, m.classes(t), 1.224, -0.048) == "step"
    t = drive(m, [RUG, SHOE], [(0.4, 0.0, 0.0)] * 6, t0=t)  # the brick picked up: the cell changes, label gone
    assert cls(m, m.classes(t), 1.224, -0.048) == "floor"
    m.forget(1.74, -0.40, 0.3)
    assert cls(m, m.classes(t), 1.74, -0.40) == "unknown"


def test_the_map_follows_the_maps_geometry():
    m = fresh()
    t = drive(m, SCENE, [(x, 0.0, 0.0) for x in np.linspace(-0.6, 0.4, 6)])
    assert m.geometry(0.05, -2.0, -2.0, 120, 80)            # grown 1 m to the left
    assert cls(m, m.classes(t), 1.74, -0.40) == "obstacle"
    assert not m.geometry(0.05, -2.0, -2.0, 120, 80)


def test_no_label_clears_an_obstacle_or_a_drop_but_caution_always_applies():
    m = fresh()
    t = drive(m, SCENE, [(x, 0.0, 0.0) for x in np.linspace(-0.6, 0.4, 6)])
    m.set_label(1.74, -0.40, 0.2, "floor")                      # the floor round it may be relabelled; the shoe not
    assert cls(m, m.classes(t), 1.74, -0.40) == "obstacle"
    assert m.set_label(1.0, -0.8, 0.1, "obstacle") > 0          # caution: always
    assert cls(m, m.classes(t), 1.0, -0.8) == "obstacle"


def test_things_go_into_a_catalog_once_and_take_labels():
    m = fresh()
    t = drive(m, SCENE, [(x, 0.0, 0.0) for x in np.linspace(-0.6, 0.4, 6)])
    things = m.things(m.classes(t), 0.4, 0.0, r=2.0)
    kinds = {th["cls"] for th in things}
    assert "obstacle" in kinds and "low" in kinds              # the shoe and the brick; the rug is a step, not a thing
    assert all(th["size"] <= 0.6 for th in things)
    cat = hm.Catalog()
    new = [cat.add(th, t) for th in things]
    assert all(new) and all(cat.add(th, t + 1) is None for th in things)   # seen again: the same entries
    shoe = min(cat.items, key=lambda it: math.hypot(it["x"] - 1.74, it["y"] + 0.40))
    assert cat.label(shoe["id"], "obstacle", "a shoe")["label"] == "obstacle"
    assert len(cat.unlabelled()) == len(things) - 1
    again = hm.Catalog(cat.items)                               # restored from the saved file
    assert again.next_id == cat.next_id
