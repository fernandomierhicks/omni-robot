"""AprilTags in the sim's house and what the robot's camera detects of them (design.md 10).

TagWorld   tags (36h11, 16 cm, est: A4 prints) on the house's walls at picture height (0.9 and 1.3 m, alternating, one
           every `spacing` m along each wall face that looks into a room; the sim's walls are 0.5-0.6 m lidar-height
           boxes, so a tag stands for one on the real wall above them), plus a dock tag low on a wall (0.25 m)
TagCamera  the left camera's detections at the detector's rate (10 Hz, est for apriltag on the Orin's CPU at full
           resolution), from the robot's true pose (its rocking included) and the camera's true mount. A tag is
           detected when it is in front of the camera, inside the image, not hidden by a wall (in plan: the walls
           stand floor to ceiling), at most 75 deg off-axis and 7 m away; the chance falls as it shrinks below ~30 px
           and while the robot turns fast (motion blur). Corners carry noise of 0.25 + 4 / side px (est), and
           rarely a misdecode reports a wrong id at a wrong place (the graph must reject it).
The estimator gets the camera mount from the frames file (robot_v11.json), with its own calibration error.
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROS_SRC = Path(os.environ.get("OMNI_ROS_SRC", HERE.parents[1] / "ros" / "src"))
for pkg in ("omni_slam", "omni_calib", "omni_bringup"):
    if str(ROS_SRC / pkg) not in sys.path:
        sys.path.insert(0, str(ROS_SRC / pkg))

from omni_slam.tags import Pinhole, TagDetection, tag_corners  # noqa: E402


@dataclass
class Tag:
    tag_id: int
    R: np.ndarray          # tag -> world (z out of the face, x along the wall, y up)
    t: np.ndarray
    size: float


def _walls(m, d, min_top: float = 0.4):
    """Static boxes taller than min_top: (centre xy, half sizes xy, yaw, geom id)."""
    out = []
    static = m.body_weldid[m.geom_bodyid] == 0
    for g in range(m.ngeom):
        if not static[g] or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX:
            continue
        if d.geom_xpos[g][2] + m.geom_size[g][2] < min_top or m.geom_group[g] not in (0, 1, 3):
            continue
        R = d.geom_xmat[g].reshape(3, 3)
        out.append((d.geom_xpos[g][:2].copy(), m.geom_size[g][:2].copy(), math.atan2(R[1, 0], R[0, 0]), g))
    return out


class TagWorld:
    def __init__(self, m, d, spacing: float = 2.4, heights=(0.9, 1.3), size: float = 0.16, dock: bool = True):
        self.walls = _walls(m, d)
        self.tags: list = []
        k = 0
        for c, h, yaw, g in self.walls:
            long_axis = 0 if h[0] >= h[1] else 1
            L, W = h[long_axis], h[1 - long_axis]
            ax = np.array([math.cos(yaw), math.sin(yaw)]) if long_axis == 0 else \
                np.array([-math.sin(yaw), math.cos(yaw)])
            nrm = np.array([-ax[1], ax[0]])
            for side in (1, -1):
                n = nrm * side
                if np.dot(-c, n) < 0 and np.linalg.norm(c) > 4.0:      # an outer wall's outside: nobody sees it
                    continue
                n_tags = max(1, int(2 * L // spacing))
                for q in range(n_tags):
                    s = -L + (q + 0.5) * 2 * L / n_tags
                    p = c + ax * s + n * (W + 0.002)
                    z = heights[k % len(heights)]
                    xw = np.array([-n[1], n[0], 0.0])                # x along the wall (seen from the room: right)
                    zw = np.array([n[0], n[1], 0.0])
                    R = np.column_stack([xw, np.cross(zw, xw), zw])
                    self.tags.append(Tag(k, R, np.array([p[0], p[1], z]), size))
                    k += 1
        self.dock = None
        if dock and self.tags:                     # the dock's tag: 10 cm, 0.25 m up, beside the first wall's tag
            t0 = self.tags[0]
            p = t0.t + t0.R[:, 0] * 0.6
            p[2] = 0.25
            self.dock = Tag(500, t0.R.copy(), p, 0.10)
            self.tags.append(self.dock)

    def occluded(self, a, b, skip_eps: float = 0.01) -> bool:
        """The plan-view segment a -> b crosses a wall (not counting the last skip_eps m at b)."""
        a, b = np.asarray(a[:2], float), np.asarray(b[:2], float)
        v = b - a
        L = float(np.linalg.norm(v))
        if L < 1e-9:
            return False
        b = a + v * (L - skip_eps) / L
        for c, h, yaw, _ in self.walls:
            cy, sy = math.cos(yaw), math.sin(yaw)
            R = np.array([[cy, sy], [-sy, cy]])
            p, q = R @ (a - c), R @ (b - c)
            d = q - p
            t0, t1 = 0.0, 1.0
            hit = True
            for k in range(2):
                if abs(d[k]) < 1e-12:
                    if abs(p[k]) > h[k]:
                        hit = False
                        break
                    continue
                ta, tb = (-h[k] - p[k]) / d[k], (h[k] - p[k]) / d[k]
                t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
                if t0 > t1:
                    hit = False
                    break
            if hit:
                return True
        return False


class TagCamera:
    def __init__(self, world: TagWorld, cam: Pinhole, T_base_cam, rate: float = 10.0, seed: int = 0,
                 max_range: float = 7.0, max_angle_deg: float = 75.0, misdecode: float = 0.002):
        self.w, self.cam, self.Tbc = world, cam, np.asarray(T_base_cam, float)
        self.period = 1.0 / rate
        self.rng = np.random.default_rng(80_000 + seed)
        self.max_range, self.max_angle = max_range, math.radians(max_angle_deg)
        self.misdecode = misdecode
        self.t_next = 0.0
        self.n_frames = self.n_dets = 0

    def due(self, t: float) -> bool:
        return t >= self.t_next - 1e-9

    def observe(self, t: float, T_world_base, yaw_rate: float = 0.0) -> list:
        self.t_next = t + self.period
        self.n_frames += 1
        Twc = np.asarray(T_world_base, float) @ self.Tbc
        Rcw, pcw = Twc[:3, :3].T, Twc[:3, 3]
        out = []
        blur = 1.0 if abs(yaw_rate) < 1.5 else 0.6
        for tg in self.w.tags:
            corners_w = tg.t + tag_corners(tg.size) @ tg.R.T
            Pc = (corners_w - pcw) @ Rcw.T
            if np.any(Pc[:, 2] < 0.1):
                continue
            rng_m = float(np.linalg.norm(tg.t - pcw))
            if rng_m > self.max_range:
                continue
            view = (pcw - tg.t) / rng_m
            if math.acos(max(-1.0, min(1.0, float(view @ tg.R[:, 2])))) > self.max_angle:
                continue
            uv = self.cam.project(Pc)
            if np.any(uv[:, 0] < 5) or np.any(uv[:, 0] > self.cam.width - 5) or np.any(uv[:, 1] < 5) or \
                    np.any(uv[:, 1] > self.cam.height - 5):
                continue
            side = float(np.mean([np.linalg.norm(uv[k] - uv[(k + 1) % 4]) for k in range(4)]))
            p = 0.97 * min(1.0, max(0.0, (side - 12.0) / 20.0)) * blur
            if self.rng.random() > p or self.w.occluded(pcw, tg.t):
                continue
            sig = 0.25 + 4.0 / side
            out.append(TagDetection(tg.tag_id, uv + self.rng.normal(0, sig, uv.shape), t, tg.size,
                                    margin=min(200.0, 40 + 2 * side)))
        if self.rng.random() < self.misdecode:            # a wrong id somewhere in the image
            c = self.rng.uniform([100, 100], [self.cam.width - 100, self.cam.height - 100])
            s = self.rng.uniform(20, 60)
            uv = c + np.array([(-s, s), (s, s), (s, -s), (-s, -s)]) / 2
            fake = TagDetection(int(self.rng.integers(0, len(self.w.tags))), uv, t, 0.16, margin=45.0)
            fake.misdecode = True                         # (the sim's own mark: scoring only)
            out.append(fake)
        self.n_dets += len(out)
        return out
