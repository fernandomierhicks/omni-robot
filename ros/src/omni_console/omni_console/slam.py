"""SLAM as the console runs it (the 'robot' step), ROS-free parts: the saved maps and the parameter files.

The launch writes two slam_toolbox parameter files from omni_bringup's config (scan topic and range filled in):
mapping, and localization without a map. To localize, the map's pose graph and the robot's start pose are appended
(as omni_bringup/stack.py does for the 'localize' step). mission_node.SlamSupervisor runs slam_toolbox's own launch
files with them.
"""

from __future__ import annotations

import math
from pathlib import Path

# s: a pose record younger than this is a restarted console's (the robot where it was, mcu_link's odom continuous),
# older is a reboot's: then localization starts at the map's start pose, as before
RESUME_S = 120.0


def _compose(a, b):
    """Planar pose b (in a's frame) in a's parent frame: (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def _inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


def pose_record(map_name: str, map_pose, odom_pose, now: float, t_mcu_us: int) -> dict:
    """What the console keeps (<state_dir>/slam_pose.json) while localized: base_link in map and in odom, at once, and
    the MCU's clock (its odometry's: a new boot starts both again)."""
    return dict(map=map_name, t=now, t_mcu_us=int(t_mcu_us), map_pose=[round(float(v), 4) for v in map_pose[:3]],
                odom_pose=[round(float(v), 4) for v in odom_pose[:3]])


def resume_pose(rec: dict, map_name: str, odom_now, now: float, t_mcu_us):
    """Where a restarted console starts localizing: the recorded map pose moved on by what the odometry drove since
    (map -> odom as recorded, applied to base_link in odom now), when the record is this map's, younger than RESUME_S
    and from the same odometry: the MCU's clock has run on since it, by no more than the wall clock (an MCU reset or a
    new sim session starts its clock and its odometry again). Else None: the map's start pose. Without it a console
    killed mid-session (a crash) relocalized at the map's origin, 1.9 m and 87 deg from the robot (Nov 5)."""
    try:
        if rec.get("map") != map_name or not 0.0 <= now - float(rec["t"]) <= RESUME_S or odom_now is None:
            return None
        ran = (int(t_mcu_us) - int(rec["t_mcu_us"])) * 1e-6
        if not 0.0 <= ran <= now - float(rec["t"]) + 2.0:
            return None
        map_odom = _compose(rec["map_pose"], _inverse(rec["odom_pose"]))
        x, y, yaw = _compose(map_odom, odom_now[:3])
    except (KeyError, TypeError, ValueError, IndexError):
        return None
    return x, y, math.atan2(math.sin(yaw), math.cos(yaw))


def saved_maps(maps_dir) -> list:
    """Names of the maps slam_toolbox can localize on (a serialized pose graph next to the image)."""
    d = Path(maps_dir)
    return sorted(p.stem for p in d.glob("*.posegraph") if (d / f"{p.stem}.data").exists())


def localization_params(template: str, map_base: str, pose) -> str:
    """The localization parameter file's text for map_base (path without extension), starting at pose."""
    x, y, yaw = (float(v) for v in pose)
    return (template.rstrip("\n") + f"\n    map_file_name: {map_base}\n"
            f"    map_start_pose: [{x:.3f}, {y:.3f}, {yaw:.4f}]\n")


def write_map_image(base, width: int, height: int, res: float, origin, data):
    """<base>.pgm + <base>.yaml as nav2 map_server / map_saver write them (trinary: free 254, wall 0, unknown 205),
    from an occupancy grid (row 0 at the origin). The console writes them itself: slam_toolbox's own save_map asks a
    map_saver that gives up when /map is slow to come."""
    rows = []
    for j in range(height - 1, -1, -1):                       # image row 0 is the top: the grid's last row
        row = data[j * width:(j + 1) * width]
        rows.append(bytes(205 if v < 0 else 0 if v >= 65 else 254 if v <= 25 else 205 for v in row))
    base = Path(base)
    with open(base.with_suffix(".pgm"), "wb") as f:
        f.write(f"P5\n# omni_console\n{width} {height}\n255\n".encode())
        f.write(b"".join(rows))
    x, y, yaw = (float(v) for v in origin)
    base.with_suffix(".yaml").write_text(
        f"image: {base.name}.pgm\nmode: trinary\nresolution: {res:.4f}\norigin: [{x:.4f}, {y:.4f}, {yaw:.4f}]\n"
        f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
