"""Use the existing SO-101 mesh study at v9's raised payload height.

This runs in a separate process, keeping CAD and sim modules with equal names
apart. The v7 baseline, generated robot configuration and shared study stay intact.
"""
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
os.environ["OMNI_CAD_OUT"] = str(HERE / "out")
sys.path.insert(0, str(ROOT / "simulation" / "v11 sil" / "studies"))
import layout_trade as L

# The study's wheel is 127 mm. Raise only its arm mount by the CAD's 13 mm;
# the CAD's obstacle meshes already include the same lift.
lay = L.BY_KEY["A6"]
original_params = lay.params
lay.params = lambda: original_params().with_(plate_gap=0.109)

if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "arm-check":
        print(json.dumps(L.arm_check()))
    elif mode == "arm-poses":
        print(json.dumps(L.arm_poses(sys.argv[2] if len(sys.argv) > 2 else "stow")))
    elif mode == "arm-stl":
        L.write_arm_stl(Path(sys.argv[2]) / "so101__arm.stl")
    else:
        raise ValueError(mode)
