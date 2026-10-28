"""SLAM as the console runs it (the 'robot' step), ROS-free parts: the saved maps and the parameter files.

The launch writes two slam_toolbox parameter files from omni_bringup's config (scan topic and range filled in):
mapping, and localization without a map. To localize, the map's pose graph and the robot's start pose are appended
(as omni_bringup/stack.py does for the 'localize' step). mission_node.SlamSupervisor runs slam_toolbox's own launch
files with them.
"""

from __future__ import annotations

from pathlib import Path


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
