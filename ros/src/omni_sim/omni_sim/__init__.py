"""omni_sim (design.md 28): the MuJoCo sim as a ROS 2 robot. PC only; never built on the robot."""

import os
import sys
from pathlib import Path


def sim_dir() -> Path:
    """The sim version folder (bridge_core.py, house.py ...): $OMNI_SIM_DIR, set by ros/omni.sh, else ~/omni_ws/sim."""
    d = Path(os.environ.get("OMNI_SIM_DIR", Path.home() / "omni_ws" / "sim"))
    if str(d) not in sys.path:
        sys.path.insert(0, str(d))
    return d


def results_dir() -> Path:
    d = sim_dir() / "studies" / "results"
    d.mkdir(parents=True, exist_ok=True)
    return d
