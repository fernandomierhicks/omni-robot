"""The Goalie's ball tracker (ball_track.py) on synthetic frames: a red ball on a grey floor, rendered by ray casting
into a registered colour and depth pair; the filter's velocity from a rolling ball. The rendered D435 case (with its
depth errors, through the robot's firmware) is simulation/v11 sil/studies/goalie.py.

    python -m pytest -q ros/src/omni_arm/test
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from omni_arm.ball_track import BallParams, BallTracker  # noqa: E402
from omni_arm.floor_vision import DepthFrame  # noqa: E402

W, H, F = 320, 180, 160.0
CAM_Z, TILT = 0.20, math.radians(15)


def cam_T():
    """The optical frame (z forward, x right, y down) in base_link: at CAM_Z, tilted TILT down, looking along +x."""
    c, s = math.cos(TILT), math.sin(TILT)
    fwd = np.array([c, 0.0, -s])
    right = np.array([0.0, -1.0, 0.0])
    down = np.cross(fwd, right)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = right, down, fwd, [0.0, 0.0, CAM_Z]
    return T


def render(ball_xyz, r=0.11, color=(220, 30, 30)):
    T = cam_T()
    K = (F, F, W / 2, H / 2)
    v, u = np.mgrid[0:H, 0:W]
    d_cam = np.stack([(u - W / 2) / F, (v - H / 2) / F, np.ones_like(u, float)], axis=-1)
    d = d_cam @ T[:3, :3].T                                     # rays in base_link (z = 1 along the optical axis)
    o = T[:3, 3]
    depth = np.full((H, W), np.nan, np.float32)
    rgb = np.full((H, W, 3), 120, np.uint8)
    t_floor = np.where(d[..., 2] < -1e-6, -o[2] / np.minimum(d[..., 2], -1e-6), np.inf)
    depth[np.isfinite(t_floor)] = t_floor[np.isfinite(t_floor)]
    c = np.asarray(ball_xyz, float) - o
    a = np.sum(d * d, axis=-1)
    b = -2 * (d @ c)
    cc = c @ c - r * r
    disc = b * b - 4 * a * cc
    hit = disc > 0
    t_ball = np.where(hit, (-b - np.sqrt(np.where(hit, disc, 0))) / (2 * a), np.inf)
    front = hit & (t_ball < t_floor)
    depth[front] = t_ball[front]                                 # t along d is the optical z (d's z is 1)
    rgb[front] = color
    return DepthFrame(0.0, depth, K, T), rgb


def test_detects_the_ball_where_it_is():
    tr = BallTracker(BallParams(color="red", diameter=0.22, min_px=20))
    for x, y in ((1.5, 0.0), (2.5, 0.4), (3.0, -0.5)):
        f, rgb = render((x, y, 0.11))
        c = tr.detect(f, rgb)
        assert c is not None and math.hypot(c[0] - x, c[1] - y) < 0.04, (x, y, c)


def test_ignores_a_red_rug_and_the_wrong_colour():
    tr = BallTracker(BallParams(color="red", diameter=0.22, min_px=20))
    f, rgb = render((2.0, 0.0, 0.11), color=(30, 30, 220))      # a blue ball
    assert tr.detect(f, rgb) is None
    f, rgb = render((2.0, 0.0, -0.105))                          # a red disc flat on the floor (a rug's spot)
    assert tr.detect(f, rgb) is None


def test_tracks_a_rolling_ball_and_its_speed():
    tr = BallTracker(BallParams(color="red", diameter=0.22, min_px=20))
    pose = (0.0, 0.0, 0.0)
    st = None
    for k in range(12):
        t = k / 15
        f, rgb = render((3.0 - 2.0 * t, 0.2 + 0.3 * t, 0.11))
        st = tr.step(t, f, rgb, pose)
    assert st["seen"] and abs(st["vx"] + 2.0) < 0.3 and abs(st["vy"] - 0.3) < 0.2
    for k in range(12, 25):                                      # gone from view: lost after BallParams.lost
        st = tr.step(k / 15, *render((30.0, 0.0, 0.11)), pose)
    assert not st["seen"]
