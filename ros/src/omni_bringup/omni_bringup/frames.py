"""The robot's frames and their rules (design.md 10). No ROS here: robot_frames (frames_node.py) and the tests use it.

The tree (REP-105, REP-103):

    map ──> odom ──> base_link ──> laser, imu_link, cam_left ──> cam_left_optical, cam_right ──> cam_right_optical,
     │                         module_pivot ──> module (tilts: /mcu/state), arm_mount ──> (an arm's own frames, its end
     │                                                                                    effector from its driver)
     └──> dock, tag_<id> (landmarks: omni_slam's tag map)

  map -> odom        one publisher: the global localizer (slam_toolbox, or omni_slam when it owns the map). It may
                     jump when a correction comes in; odom never does.
  odom -> base_link  one publisher: the EKF (or mcu_link with no EKF): smooth, continuous local odometry
  base_link -> ...   fixed: config/robot_v11.json "frames" (generated from the sim's model, robot_export.py), each
                     overridden by a calibration result only within bounds of it (calibration can't drift a sensor
                     off the drawing: a result past its bound is refused and reported)

check_tree() and check_motion() say what breaks these rules in a recorded /tf (frame_check, tests).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# how far a calibrated sensor may sit from its drawn pose (est: printed mounts, +/- 1 mm parts, a hand-fitted bar)
BOUNDS = {"xyz_m": 0.02, "rot_deg": 3.0}
SENSOR_FRAMES = ("laser", "imu_link", "cam_left", "cam_right", "cam_left_optical", "cam_right_optical")
BODY_TO_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])   # columns: optical x, y, z in body


def quat_to_R(q) -> np.ndarray:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R) -> list:
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
    if q[3] < 0:
        q = [-c for c in q]
    return [float(c) for c in q]


def angle_between(Ra, Rb) -> float:
    """rad: the rotation from Ra to Rb."""
    c = (np.trace(np.asarray(Ra).T @ np.asarray(Rb)) - 1) / 2
    return float(math.acos(max(-1.0, min(1.0, c))))


@dataclass
class Frame:
    parent: str
    child: str
    xyz: np.ndarray
    R: np.ndarray
    note: str = ""
    source: str = "drawing"          # "drawing" (robot_export) | "calibration"
    sigma: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return dict(parent=self.parent, child=self.child, xyz=[float(c) for c in self.xyz], quat=R_to_quat(self.R),
                    source=self.source)


def load_frames(path: str | Path) -> list:
    raw = json.loads(Path(path).read_text())
    return [Frame(f["parent"], f["child"], np.array(f["xyz"], float), quat_to_R(f["quat"]), f.get("note", ""))
            for f in raw["frames"]]


def apply_calibration(frames: list, cal: dict, bounds: dict = BOUNDS) -> tuple:
    """cal: child -> {"xyz": [...], "quat": [x y z w], "sigma": {...}} (omni_calib's results, already validated).
    Returns (frames, report): each result inside the bounds replaces the drawn pose; one outside is refused."""
    out, report = [], []
    for f in frames:
        c = cal.get(f.child)
        if c is None:
            out.append(f)
            continue
        xyz, R = np.array(c["xyz"], float), quat_to_R(c["quat"])
        dx, da = float(np.linalg.norm(xyz - f.xyz)), math.degrees(angle_between(f.R, R))
        ok = dx <= bounds["xyz_m"] and da <= bounds["rot_deg"]
        report.append(dict(frame=f.child, moved_mm=round(1000 * dx, 1), turned_deg=round(da, 3), applied=ok))
        out.append(Frame(f.parent, f.child, xyz, R, f.note, "calibration", c.get("sigma", {})) if ok else f)
    return out, report


# -- checks --------------------------------------------------------------------------------------------------------

def check_tree(edges, static_children=SENSOR_FRAMES) -> list:
    """edges: iterable of (parent, child, R or None). Returns the problems found (empty = REP-105 holds)."""
    problems = []
    parents: dict = {}
    rot: dict = {}
    for parent, child, R in edges:
        if child in parents and parents[child] != parent:
            problems.append(f"{child} has two parents: {parents[child]} and {parent}")
        parents[child] = parent
        if R is not None:
            rot[(parent, child)] = R

    def chain(f):
        seen = []
        while f in parents:
            if f in seen:
                return seen + [f, "(cycle)"]
            seen.append(f)
            f = parents[f]
        return seen + [f]

    for c in parents:
        if chain(c)[-1] == "(cycle)":
            problems.append(f"a cycle through {c}")
    roots = {chain(c)[-1] for c in parents if chain(c)[-1] != "(cycle)"}
    if len(roots) > 1:
        problems.append(f"more than one tree: roots {sorted(roots)}")
    if "base_link" in parents:
        up = chain("base_link")
        if parents["base_link"] != "odom":
            problems.append(f"base_link's parent is {parents['base_link']}, not odom (REP-105)")
        if "map" in up and up.index("map") != 2:
            problems.append(f"map -> odom -> base_link broken: {' <- '.join(up)}")
    if "odom" in parents and parents["odom"] != "map":
        problems.append(f"odom's parent is {parents['odom']}, not map")
    for s in static_children:
        if s in parents and "base_link" not in chain(s):
            problems.append(f"{s} isn't on base_link: {' <- '.join(chain(s))}")
    for (p, c), R in rot.items():
        if c.endswith("_optical") and angle_between(R, BODY_TO_OPTICAL) > math.radians(0.5):
            problems.append(f"{c} isn't a REP-103 optical frame of {p} (z forward, x right, y down)")
    return problems


def check_motion(samples, v_max: float = 2.5, w_max: float = 5.0, slack_m: float = 0.02,
                 slack_rad: float = 0.05) -> dict:
    """samples: time-ordered (t, x, y, yaw) of one transform. Counts the steps no robot motion explains (jumps).
    odom -> base_link must have none; map -> odom may (each is a correction) and reports how big they were."""
    jumps, worst = [], 0.0
    for a, b in zip(samples, samples[1:]):
        dt = max(0.0, b[0] - a[0])
        d = math.hypot(b[1] - a[1], b[2] - a[2])
        dyaw = abs(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
        if d > v_max * dt + slack_m or dyaw > w_max * dt + slack_rad:
            jumps.append((b[0], d, dyaw))
        worst = max(worst, d)
    return dict(n=len(samples), jumps=len(jumps), largest_jump_m=round(max((j[1] for j in jumps), default=0.0), 4),
                first_jumps=[(round(t, 3), round(d, 4), round(math.degrees(y), 2)) for t, d, y in jumps[:5]])
