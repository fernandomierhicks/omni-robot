"""ros2 run omni_sim score_map [name]: score maps/<name>.yaml against the true house (map_quality.py) and write
studies/results/ros_map_<name>.json."""

from __future__ import annotations

import sys

from omni_bringup import maps_dir
from omni_sim import results_dir, sim_dir


def main():
    sim_dir()
    import map_quality
    args = [a for a in sys.argv[1:] if not a.startswith("--ros-args")]
    name = args[0] if args else "house"
    sys.argv = ["map_quality", str(maps_dir() / f"{name}.yaml"), "--out", str(results_dir() / f"ros_map_{name}.json")]
    map_quality.main()
