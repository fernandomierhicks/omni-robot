"""omni.sh arm: the arm robot in the house, every console mode plus Tidy (design.md 11); see omni_bringup/stack.py."""

from omni_bringup.stack import describe


def generate_launch_description():
    return describe("arm")
