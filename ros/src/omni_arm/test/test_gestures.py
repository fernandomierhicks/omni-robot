"""The arm's feeling gestures (gestures.py): checked like any motion, back where they began, bounded by their size.

    python -m pytest -q ros/src/omni_arm/test/test_gestures.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from omni_arm import gestures  # noqa: E402
from omni_arm import motion as mo  # noqa: E402
from omni_arm.kinematics import ArmKinematics  # noqa: E402
from omni_arm.pick import TidyTask  # noqa: E402


@pytest.fixture(scope="module")
def task():
    return TidyTask(ArmKinematics())


@pytest.mark.parametrize("pose", ["ready", "high", "look"])
def test_every_gesture_plays_from_the_free_poses_and_ends_where_it_began(task, pose):
    kin, q0 = task.kin, task.poses[pose]
    for name in gestures.GESTURES:
        times, wps = gestures.track(kin, q0, name, 1.0)
        assert times is not None, f"{name} from {pose}: {wps}"
        assert np.allclose(wps[-1], q0, atol=1e-9), name                   # back exactly where it started
        assert times[0] == 0.0 and all(b > a for a, b in zip(times[:-1], times[1:]))
        dev = max(float(np.max(np.abs(np.asarray(w) - q0))) for w in wps)
        assert dev <= math.radians(26.0), name                             # small: a gesture, not a move


def test_size_scales_a_gesture_and_zero_or_unknown_is_refused(task):
    kin, q0 = task.kin, task.poses["ready"]
    _, big = gestures.track(kin, q0, "nod", 1.0)
    _, small = gestures.track(kin, q0, "nod", 0.3)
    d = lambda wps: max(float(np.max(np.abs(np.asarray(w) - q0))) for w in wps)   # noqa: E731
    assert d(small) == pytest.approx(0.3 * d(big), rel=1e-6)
    assert gestures.track(kin, q0, "nod", 0.0)[0] is None
    assert gestures.track(kin, q0, "moonwalk", 1.0)[0] is None


def test_the_hand_stays_shut_while_it_holds_something(task):
    kin, q0 = task.kin, task.poses["ready"]
    g = list(kin.cfg.joints).index("gripper")
    _, free = gestures.track(kin, q0, "chatter", 1.0)
    _, held = gestures.track(kin, q0, "chatter", 1.0, holding=True)
    assert max(abs(w[g] - q0[g]) for w in free) > 0.1
    assert all(abs(w[g] - q0[g]) < 1e-12 for w in held)


def test_a_gesture_into_a_keep_out_is_refused_whole(task):
    """From the stow (folded inside the bumper, design.md 4.4: its collision is still open), a gesture either passes
    every check or is refused: never clipped, never played unchecked."""
    kin, q0 = task.kin, task.poses["stow"]
    vmax = float(np.min(np.atleast_1d(kin.cfg.servos["speed"]))) / 1.875
    for name in gestures.GESTURES:
        times, wps = gestures.track(kin, q0, name, 1.0)
        if times is not None:                                              # it passed: the same check agrees
            pts = list(zip(times[1:], [list(w) for w in wps[1:]]))
            assert mo.check_trajectory(kin, q0, list(kin.cfg.joints), pts, max_speed=vmax)[0] is not None


def test_the_motion_keeps_the_keys_times_and_eases_between_them(task):
    kin, q0 = task.kin, task.poses["ready"]
    m, why = gestures.motion(kin, q0, "tilt", 1.0, 100.0)
    assert m is not None, why
    assert m.t[0] == 100.0 and m.t_end == pytest.approx(100.0 + gestures.GESTURES["tilt"][-1][0])
    assert np.allclose(m.sample(m.t_end + 1.0), q0)
    k = list(kin.cfg.joints).index("wrist_roll")
    assert m.sample(100.45)[k] - q0[k] == pytest.approx(math.radians(22.0), abs=1e-6)    # at its key
