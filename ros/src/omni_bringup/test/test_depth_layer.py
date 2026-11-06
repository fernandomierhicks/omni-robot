"""The D435's depth layer in Nav2's costmaps (stack.with_depth_layer, design.md 4.5, 4.11). Needs ROS's launch packages
(stack.py imports them): runs in WSL, skips on Windows.
    python3 -m pytest -q ros/src/omni_bringup/test
"""

import os
import sys

import pytest

pytest.importorskip("launch_ros")
yaml = pytest.importorskip("yaml")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_bringup import stack  # noqa: E402

NAV2 = os.path.join(os.path.dirname(__file__), "..", "config", "nav2.yaml")
FLOOR_GRID = "/floor_obstacles/grid"         # omni_arm's floor_obstacles_node


def costmaps(profile, layer):
    y = yaml.safe_load(stack.profiled(stack.with_depth_layer(open(NAV2).read(), layer), "nav2.yaml", profile))
    for c in ("local_costmap", "global_costmap"):
        q = y[c][c]["ros__parameters"]
        assert q["plugins"][-2:] == ["depth_layer", "inflation_layer"] and "obstacle_layer" in q["plugins"]
        assert q["obstacle_layer"]["observation_sources"] == "scan"                     # the lidar's layer as it was
        yield c, q


@pytest.mark.parametrize("profile", ["default", "lean", "pi"])
def test_the_floor_layer_reads_the_nodes_grid_over_the_other_layers(profile):
    for c, q in costmaps(profile, "floor"):
        dl = q["depth_layer"]
        assert dl["plugin"] == "nav2_costmap_2d::StaticLayer" and dl["map_topic"] == FLOOR_GRID
        assert dl["map_subscribe_transient_local"] and not dl["footprint_clearing_enabled"]
        # its marks go over the lidar's (max), never overwrite them; its -1 cells are no information in the global
        # costmap (which tracks unknown space: skipped) and free in the local one (max(free, x) = x)
        assert q["use_maximum"] is True
        assert q.get("track_unknown_space", False) == (c == "global_costmap")
        assert q["plugins"].index("depth_layer") > q["plugins"].index("obstacle_layer")


@pytest.mark.parametrize("profile", ["default", "lean"])
def test_the_raw_layer_is_nov_5s(profile):
    for _, q in costmaps(profile, "raw"):
        assert "use_maximum" not in q
        mark, clear = q["depth_layer"]["depth_mark"], q["depth_layer"]["depth_clear"]
        assert mark["topic"] == clear["topic"] == "/d435/depth/color/points"
        assert mark["marking"] and not mark["clearing"] and clear["clearing"] and not clear["marking"]
        assert clear["min_obstacle_height"] < 0.0 < mark["min_obstacle_height"]       # the floor clears, never marks


def test_a_changed_nav2_yaml_or_layer_fails_loudly():
    with pytest.raises(RuntimeError):                    # its plugins lines gone: no silent robot without the layer
        stack.with_depth_layer("local_costmap: {}\n")
    with pytest.raises(RuntimeError):
        stack.with_depth_layer(open(NAV2).read(), "off")
    assert set(stack.DEPTH_LAYERS) == {"height", "floor", "raw", "off"}


@pytest.mark.parametrize("profile, top", [("default", 2.0), ("pi", 0.6)])
def test_the_height_layer_takes_the_zones_speed_limit_through_its_node(profile, top):
    """depth_layer:=height: the same grid in a StaticLayer, the console's speed filter on /speed_limit_zones (the
    height_map node passes the lower of it and its own on /speed_limit, which the controller reads)."""
    nav = stack.profiled(stack.with_depth_layer(open(NAV2).read(), "height"), "nav2.yaml", profile)
    y = yaml.safe_load(nav)
    g = y["global_costmap"]["global_costmap"]["ros__parameters"]
    assert g["speed_filter"]["speed_limit_topic"] == "/speed_limit_zones"
    assert y["controller_server"]["ros__parameters"]["speed_limit_topic"] == "/speed_limit"
    assert g["depth_layer"]["map_topic"] == FLOOR_GRID
    assert stack.vx_max(nav) == top
    raw = yaml.safe_load(stack.with_depth_layer(open(NAV2).read(), "raw"))
    assert raw["global_costmap"]["global_costmap"]["ros__parameters"]["speed_filter"]["speed_limit_topic"] == "/speed_limit"
