"""omni_calib.detect.apriltags: the C library (pupil_apriltags) and OpenCV's aruco find the same tag with the same
corners in the same order (counter-clockwise from the tag's bottom left, as printed from the official images), and the
C library gives a decoder margin. Each backend's test skips where it isn't installed (both are in WSL's
~/.venvs/omni-ros).
    python -m pytest -q ros/src/omni_calib/test/test_detect.py
"""

from __future__ import annotations

import numpy as np
import pytest

from omni_calib import detect

# the official tag36h11 id 0 (black border included), upright: built from the C library's family table (code
# 0xd7e00984b, bit_x / bit_y) and found by it as id 0. OpenCV's marker 0 (camera_snap.FAKE_TAGS) is this turned 180 deg.
TAG36H11_0 = ["00000000", "01101010", "00111010", "00110000", "01010000", "00101100", "00001000", "00000000"]
OPENCV_36H11_0 = ["00000000", "00010000", "00110100", "00001010", "00001100", "01011100", "01010110", "00000000"]
CELL, X0, Y0 = 12, 140, 90                     # px: cell size; the black square's top-left corner in the picture


def picture(rot: int = 0, rows=TAG36H11_0) -> tuple[np.ndarray, np.ndarray]:
    """A 480 x 360 grey picture with the tag upright (then turned rot quarter turns counter-clockwise on screen), and
    its black square's corners in apriltag's order: bottom left, bottom right, top right, top left (as seen upright)."""
    img = np.full((360, 480), 255, np.uint8)
    bits = np.array([[int(b) for b in row] for row in rows], np.uint8)
    img[Y0:Y0 + 8 * CELL, X0:X0 + 8 * CELL] = np.kron(bits, np.ones((CELL, CELL), np.uint8)) * 255
    s = 8 * CELL
    # pixel edges: the square covers columns X0 .. X0 + s - 1, so its outline runs at X0 - 0.5 .. X0 + s - 0.5
    c = np.array([[X0, Y0 + s], [X0 + s, Y0 + s], [X0 + s, Y0], [X0, Y0]], float) - 0.5
    for _ in range(rot):                        # np.rot90: (u, v) -> (v, W - 1 - u) in the turned picture
        w = img.shape[1]
        img = np.rot90(img)
        c = np.column_stack([c[:, 1], w - 1 - c[:, 0]])
    return np.ascontiguousarray(img), c


def backends():
    out = []
    try:
        import pupil_apriltags  # noqa: F401
        out.append("apriltag")
    except ImportError:
        pass
    try:
        import cv2
        if hasattr(cv2, "aruco"):
            out.append("opencv")
    except ImportError:
        pass
    return out


@pytest.mark.parametrize("use", ["apriltag", "opencv"])
@pytest.mark.parametrize("rot", [0, 1, 2, 3])
@pytest.mark.parametrize("decimate", [1.0, 2.0])
def test_one_tag_its_corners_in_apriltags_order(use, rot, decimate):
    if use not in backends():
        pytest.skip(f"{use} is not installed here")
    img, want = picture(rot)
    found = detect.apriltags(img, "36h11", decimate=decimate, backend=use)
    assert [t[0] for t in found] == [0]
    _, corners, margin = found[0]
    assert corners.shape == (4, 2)
    assert np.abs(corners - want).max() < 1.0, (corners, want)
    if use == "apriltag":
        assert margin is not None and margin > 30          # a clean synthetic tag: well over tag_slam's min_margin
    else:
        assert margin is None


def test_opencvs_marker_is_the_official_tag_turned_half_round():
    """Both backends read OpenCV's own marker image as the official tag upside down: corner 0 at its top right."""
    have = backends()
    if not have:
        pytest.skip("no AprilTag detector here")
    img, upright = picture(0, OPENCV_36H11_0)
    for use in have:
        found = detect.apriltags(img, "36h11", backend=use)
        assert [t[0] for t in found] == [0]
        assert np.abs(found[0][1] - upright[[2, 3, 0, 1]]).max() < 1.0, use


def test_the_default_is_the_c_library_when_installed():
    if "apriltag" not in backends():
        pytest.skip("pupil_apriltags is not installed here")
    assert detect.apriltag_backend() == "apriltag"
    img, _ = picture()
    assert detect.apriltags(img, "tag36h11")[0][2] is not None   # "tag36h11" and "36h11" name the same family


def test_detectors_are_cached_per_setting():
    if "apriltag" not in backends():
        pytest.skip("pupil_apriltags is not installed here")
    a = detect._c_detector("36h11", 2.0, 2)
    assert detect._c_detector("36h11", 2.0, 2) is a
    assert detect._c_detector("36h11", 1.0, 2) is not a


def test_no_tag_no_detection():
    if not backends():
        pytest.skip("no AprilTag detector here")
    assert detect.apriltags(np.full((240, 320), 200, np.uint8)) == []
