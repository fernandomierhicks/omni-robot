"""tag_pose.py against a rendered AprilTag at a known pose (needs OpenCV: runs in the realsense venv, skipped elsewhere)."""
import sys
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import tag_pose as T   # noqa: E402


def render_tag(size, t_cv, yaw_deg, f, w, h):
    """The 36h11 id 0 tag (black square `size` m) at t_cv (OpenCV camera frame: x right, y down, z forward), turned
    yaw_deg about the vertical, on a grey image."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    m = cv2.aruco.generateImageMarker(d, 0, 200)
    pad = 40
    img_tag = np.full((200 + 2 * pad, 200 + 2 * pad), 255, np.uint8)
    img_tag[pad:pad + 200, pad:pad + 200] = m
    half = size / 2 * (200 + 2 * pad) / 200
    obj = np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]], np.float64)   # y down in the image
    a = np.radians(yaw_deg)
    R = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
    px, _ = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], np.array(t_cv, np.float64), K, None)
    src = np.array([[0, 0], [img_tag.shape[1], 0], [img_tag.shape[1], img_tag.shape[0]], [0, img_tag.shape[0]]], np.float32)
    H = cv2.getPerspectiveTransform(src, px.reshape(4, 2).astype(np.float32))
    bg = np.full((h, w), 120, np.uint8)
    warped = cv2.warpPerspective(img_tag, H, (w, h), dst=bg, borderMode=cv2.BORDER_TRANSPARENT)
    return np.stack([warped] * 3, -1)


def test_tag_pose_recovers_position():
    f, w, h, size = 463.0, 768, 432, 0.15
    t_cv = (0.30, -0.20, 2.5)                     # right 0.3, UP 0.2 (y down is negative), 2.5 m ahead
    img = render_tag(size, t_cv, 25, f, w, h)
    r = T.pose_of(img, 0, size, f, w / 2, h / 2)
    assert r is not None
    x, y, z = r["tag_cam"]
    assert abs(x - 0.30) < 0.03 and abs(y - 0.20) < 0.03 and abs(z + 2.5) < 0.12     # our frame: y up, looks along -z
    assert abs(r["range_m"] - np.linalg.norm(t_cv)) < 0.12


def test_no_tag_returns_none():
    assert T.pose_of(np.full((432, 768, 3), 120, np.uint8), 0, 0.15, 463.0, 384, 216) is None
