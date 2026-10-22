"""omni_bringup (design.md 28): the robot's launch files and configuration, for the robot and the sim alike."""

import os
from pathlib import Path


def maps_dir() -> Path:
    """Saved maps: $OMNI_MAPS_DIR (ros/omni.sh sets it; synced with the repo's ros/maps), else ~/omni_ws/maps."""
    d = Path(os.environ.get("OMNI_MAPS_DIR", Path.home() / "omni_ws" / "maps"))
    d.mkdir(parents=True, exist_ok=True)
    return d
