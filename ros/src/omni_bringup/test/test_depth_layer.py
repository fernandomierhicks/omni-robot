"""The D435's depth layer in Nav2's costmaps (stack.with_depth_layer, design.md 4.11). Needs ROS's launch packages
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


@pytest.mark.parametrize("profile", ["default", "lean"])
def test_both_costmaps_take_the_depth_cloud_in_a_layer_of_their_own(profile):
    nav = stack.profiled(stack.with_depth_layer(open(NAV2).read()), "nav2.yaml", profile)
    y = yaml.safe_load(nav)
    for c in ("local_costmap", "global_costmap"):
        q = y[c][c]["ros__parameters"]
        assert q["plugins"][-2:] == ["depth_layer", "inflation_layer"] and "obstacle_layer" in q["plugins"]
        dl = q["depth_layer"]
        mark, clear = dl["depth_mark"], dl["depth_clear"]
        assert mark["topic"] == clear["topic"] == "/d435/depth/color/points"
        assert mark["marking"] and not mark["clearing"] and clear["clearing"] and not clear["marking"]
        assert clear["min_obstacle_height"] < 0.0 < mark["min_obstacle_height"]       # the floor clears, never marks
        assert q["obstacle_layer"]["observation_sources"] == "scan"                     # the lidar's layer as it was


def test_a_changed_nav2_yaml_fails_loudly():
    with pytest.raises(RuntimeError):                    # its plugins lines gone: no silent robot without the layer
        stack.with_depth_layer("local_costmap: {}\n")
