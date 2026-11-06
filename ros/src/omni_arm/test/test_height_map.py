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
    v, why = m.speed_limit(m.classes(t), t, 0.4, 0.0, 0.0, v_max)
    assert v == hm.V_LOW and "low" in why


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
