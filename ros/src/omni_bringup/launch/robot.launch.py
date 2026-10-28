"""omni.sh robot: the whole robot at once, every console mode switchable at runtime; see omni_bringup/stack.py."""

from omni_bringup.stack import describe


def generate_launch_description():
    return describe("robot")
