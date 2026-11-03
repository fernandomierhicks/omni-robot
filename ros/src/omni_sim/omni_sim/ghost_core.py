"""ghost_ball's arithmetic, without ROS (tested in characterization/throw_track/test_ghost.py): the head's track into the
sim's odom frame, and the head's clock onto the sim's."""
from __future__ import annotations

import math

import numpy as np


def rot6(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    out = np.zeros((6, 6))
    out[:3, :3] = R
    out[3:, 3:] = R
    return out


def transform_track(x, P, crossing, cam_xy, cam_yaw):
    """The head's track (state x[6], covariance P[36], crossing[3]) in the sim's odom frame."""
    R6 = rot6(cam_yaw)
    xs = R6 @ np.asarray(x, float)
    xs[0] += cam_xy[0]
    xs[1] += cam_xy[1]
    Ps = R6 @ np.asarray(P, float).reshape(6, 6) @ R6.T
    cr = R6[:3, :3] @ np.asarray(crossing, float)
    cr[0] += cam_xy[0]
    cr[1] += cam_xy[1]
    return xs, Ps, cr


class ClockMap:
    """Head stamp -> this node's clock, by the shortest delay (see the module docstring). The minimum decays slowly so a
    drift between the two clocks (a rate error) is followed."""

    def __init__(self, latency_min: float = 0.017, decay: float = 2e-4):
        self.latency_min, self.decay = latency_min, decay
        self.offset = None

    def update(self, head_stamp: float, now: float) -> float:
        d = now - head_stamp
        self.offset = d if self.offset is None else min(self.offset + self.decay * 0.1, d)
        return head_stamp + self.offset - self.latency_min
