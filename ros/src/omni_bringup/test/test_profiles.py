"""nav_profile (stack.PROFILES): full | lean | pi (design.md 4.9, rpi.md). Every replacement must still be in the configs
(a changed config fails loudly), the result must be valid YAML, and the Nav2 numbers must stay consistent with each other.
Needs ROS's launch packages (stack.py imports them): runs in WSL, skips on Windows.
    python3 -m pytest -q ros/src/omni_bringup/test
"""

import os
import sys

import pytest

pytest.importorskip("launch_ros")
yaml = pytest.importorskip("yaml")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_bringup import stack  # noqa: E402

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")
NAMES = ("default", "full", "lean", "pi")


def nav2(profile):
    return yaml.safe_load(stack.profiled(open(os.path.join(CONFIG, "nav2.yaml")).read(), "nav2.yaml", profile))


def slam(profile):
    return yaml.safe_load(stack.profiled(open(os.path.join(CONFIG, "slam_localization.yaml")).read(),
                                         "slam_localization.yaml", profile))


def cells(y):
    q = y["local_costmap"]["local_costmap"]["ros__parameters"]
    return q["width"] * q["height"] / q["resolution"] ** 2


@pytest.mark.parametrize("profile", NAMES)
def test_every_profile_applies_and_parses(profile):
    nav2(profile)
    slam(profile)


def test_full_is_the_default_unchanged():
    text = open(os.path.join(CONFIG, "nav2.yaml")).read()
    assert stack.profiled(text, "nav2.yaml", "full") == stack.profiled(text, "nav2.yaml", "default") == text


def test_an_unknown_profile_fails_loudly():
    with pytest.raises(RuntimeError):
        stack.profiled("x: 1\n", "nav2.yaml", "orin-nano")


@pytest.mark.parametrize("profile", NAMES)
def test_the_control_numbers_agree_with_each_other(profile):
    y = nav2(profile)
    ctrl = y["controller_server"]["ros__parameters"]
    mppi = ctrl["FollowPath"]
    smoother = y["velocity_smoother"]["ros__parameters"]
    assert mppi["model_dt"] * ctrl["controller_frequency"] == pytest.approx(1.0)       # MPPI warns otherwise
    assert smoother["smoothing_frequency"] == ctrl["controller_frequency"]
    assert mppi["vx_max"] <= smoother["max_velocity"][0] and mppi["vy_max"] <= smoother["max_velocity"][1]
    assert mppi["wz_max"] <= smoother["max_velocity"][2] + 0.5                              # the smoother caps the turn
    assert -mppi["vx_min"] <= -smoother["min_velocity"][0] + 1e-9
    # the path MPPI scores against reaches as far as its horizon at top speed
    assert mppi["prune_distance"] >= 0.9 * mppi["time_steps"] * mppi["model_dt"] * mppi["vx_max"] * 0.5


def test_pi_is_cheaper_than_lean_which_is_cheaper_than_full():
    sizes = {p: cells(nav2(p)) for p in ("full", "lean", "pi")}
    assert sizes["pi"] < sizes["lean"] < sizes["full"]
    assert sizes["pi"] < sizes["full"] / 5
    batch = {p: nav2(p)["controller_server"]["ros__parameters"]["FollowPath"]["batch_size"] for p in ("full", "lean", "pi")}
    assert batch["pi"] < batch["lean"] < batch["full"]
    assert nav2("pi")["controller_server"]["ros__parameters"]["controller_frequency"] < 20.0


def test_pi_caps_the_speed_everywhere_it_is_set():
    y = nav2("pi")
    cap = stack.PROFILE_EXTRAS["pi"]["speed_cap"]
    assert y["controller_server"]["ros__parameters"]["FollowPath"]["vx_max"] == cap
    assert y["velocity_smoother"]["ros__parameters"]["max_velocity"][0] == cap


@pytest.mark.parametrize("profile", NAMES)
def test_nothing_waits_on_a_costmap_for_less_than_its_update_period(profile):
    y = nav2(profile)
    local = y["local_costmap"]["local_costmap"]["ros__parameters"]["update_frequency"]
    glob = y["global_costmap"]["global_costmap"]["ros__parameters"]["update_frequency"]
    assert y["planner_server"]["ros__parameters"]["costmap_update_timeout"] >= 1.0 / glob
    if profile == "pi":           # (full's 0.3 s against its 5 Hz local costmap is Nav2's default pairing)
        assert y["controller_server"]["ros__parameters"]["costmap_update_timeout"] >= 1.0 / local


def test_a_pi_driving_the_robot_gets_the_pi_profile_by_default():
    assert stack.resolved_profile("default", "both", pi=True) == "pi"
    assert stack.resolved_profile("default", "robot", pi=True) == "pi"
    assert stack.resolved_profile("default", "robot", pi=False) == "default"
    assert stack.resolved_profile("default", "sim", pi=True) == "default"
    assert stack.resolved_profile("full", "robot", pi=True) == "full"          # a named profile stays named


def test_pi_takes_scans_as_they_are_and_full_shifts_them():
    assert nav2("full")["collision_monitor"]["ros__parameters"]["base_shift_correction"] is True
    # at 0.2 s the shift's odom lookup failed ~120 times a run on the PC too (C4; 1 at 0.5 s, C8)
    assert nav2("full")["collision_monitor"]["ros__parameters"]["transform_tolerance"] >= 0.5
    assert nav2("pi")["collision_monitor"]["ros__parameters"]["base_shift_correction"] is False


def test_pi_runs_the_arm_slower_and_full_leaves_it_alone():
    assert 0 < stack.PROFILE_EXTRAS["pi"]["arm_hz"] < 50.0
    assert "arm_hz" not in stack.PROFILE_EXTRAS.get("full", {})


def test_pi_gives_nav2_a_slow_start_up():
    assert stack.PROFILE_EXTRAS["pi"]["bond_timeout"] > 25.0      # bt_navigator took 25 s to activate on the Pi
    assert "bond_timeout" not in stack.PROFILE_EXTRAS.get("full", {})


def test_pi_slam_corrects_less_often_but_keeps_loop_closing():
    full, pi = slam("full")["slam_toolbox"]["ros__parameters"], slam("pi")["slam_toolbox"]["ros__parameters"]
    assert pi["minimum_travel_distance"] > full["minimum_travel_distance"]
    assert pi["minimum_time_interval"] > full["minimum_time_interval"]
    # without loop closing the robot slipped ~1 m at the same spots even on a PC (Nov 2, C3 against C7)
    assert full["do_loop_closing"] is True and pi["do_loop_closing"] is True
    # a coarser correlation grid crashed slam_toolbox on activation (Nov 2): the scan matcher stays as it is
    assert pi["correlation_search_space_resolution"] == full["correlation_search_space_resolution"]


def test_the_depth_layer_still_goes_in_under_every_profile():
    for profile in NAMES:
        y = yaml.safe_load(stack.profiled(stack.with_depth_layer(open(os.path.join(CONFIG, "nav2.yaml")).read()),
                                          "nav2.yaml", profile))
        for c in ("local_costmap", "global_costmap"):
            assert "depth_layer" in y[c][c]["ros__parameters"]["plugins"]
