"""Calibration targets: a checkerboard (its inner corners) and an AprilGrid (Kalibr's: a grid of AprilTag 36h11
tags, each tag's four corners known by id, so a partial view still says which corner is which). Points on the target
in metres, z = 0 on its face, x along the rows, y along the columns."""

from __future__ import annotations

import numpy as np


def checkerboard(cols: int = 9, rows: int = 6, square: float = 0.03) -> np.ndarray:
    """Inner corners, (cols * rows, 3), row by row."""
    x, y = np.meshgrid(np.arange(cols) * square, np.arange(rows) * square)
    return np.column_stack([x.ravel(), y.ravel(), np.zeros(x.size)])


def aprilgrid(nx: int = 6, ny: int = 6, tag: float = 0.088, spacing: float = 0.3, first_id: int = 0):
    """(ids (n,), corners (n, 3)): corner id = 4 * tag id + k (k = 0..3 counter-clockwise from the bottom left, as
    Kalibr numbers them). Default: Kalibr's 6 x 6 grid of 88 mm tags, 0.3 spacing ratio (printed at A0: VERIFY the
    printed size with calipers; the scale error goes straight into the extrinsics)."""
    step = tag * (1 + spacing)
    ids, pts = [], []
    for j in range(ny):
        for i in range(nx):
            tid = first_id + j * nx + i
            x0, y0 = i * step, j * step
            for k, (dx, dy) in enumerate(((0, 0), (tag, 0), (tag, tag), (0, tag))):
                ids.append(4 * tid + k)
                pts.append((x0 + dx, y0 + dy, 0.0))
    return np.array(ids), np.array(pts, float)


def tag_corners(size: float) -> np.ndarray:
    """A lone AprilTag's four corners in its own frame (centre at 0, z out of the face, x right, y up), the order
    apriltag's detector reports them (counter-clockwise from the bottom left)."""
    h = size / 2
    return np.array([(-h, -h, 0.0), (h, -h, 0.0), (h, h, 0.0), (-h, h, 0.0)])
