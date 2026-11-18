"""lifecycle_up's step choice (ROS-free) and stack.py's use of it: slam_toolbox's launch files never autostart it."""
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omni_bringup.lifecycle_up import (ACTIVATE, ACTIVE, CONFIGURE, FINALIZED, INACTIVE, UNCONFIGURED,  # noqa: E402
                                       next_transition)


def test_each_state_moves_towards_active():
    assert next_transition(UNCONFIGURED) == CONFIGURE
    assert next_transition(INACTIVE) == ACTIVATE
    for wait in (None, ACTIVE, FINALIZED, 10, 13):      # no answer, done, or a transition under way (configuring, activating)
        assert next_transition(wait) is None


def test_stack_starts_slam_toolbox_without_its_launch_autostart():
    src = open(os.path.join(os.path.dirname(__file__), "..", "omni_bringup", "stack.py")).read()
    includes = re.findall(r'"(online_async_launch|localization_launch)\.py"\)\),\s*launch_arguments=\{(.*?)\}\.items\(\)\)\)\s*'
                          r'(\w+)\(\)', src, re.S)
    assert len(includes) == 2
    for _, launch_args, after in includes:
        assert '"autostart": "false"' in launch_args and after == "slam_up"
    setup = open(os.path.join(os.path.dirname(__file__), "..", "setup.py")).read()
    assert "lifecycle_up = omni_bringup.lifecycle_up:main" in setup
