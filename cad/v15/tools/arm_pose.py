"""The SO-101's official parts posed by the robot's own arm kinematics (ros/src/omni_arm, the ROS-free core: the one import from
outside cad/v14, so the stow pose and the planned moves follow the robot code). Used by upper.py (the arm at stow), arm_sweep.py and
d435_riser.py (the arm's swept volume) and harness_route.py (the arm's keep-out).
(The pose helpers of cad/v11-v13's screen_study.py, without the screen study itself.)
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / 'ros' / 'src' / 'omni_arm'))
from omni_arm.kinematics import ArmConfig, ArmKinematics, DOWN       # noqa: E402,F401
import omni_arm.motion as mo                                          # noqa: E402,F401

REF = HERE.parent / 'reference'                                        # this version's copy of the SO-101 link table and meshes


def arm_parts():
    """the SO-101's official parts per kinematic body: [(body, product, 4 x 4 part-in-body (mm))] from the link table (its body frames
    are the planner's: the same joint offsets; only the whole arm sits 13 mm lower in the sim, which this relative form removes)"""
    out = []
    for bd in json.loads((REF / 'so101_links.json').read_text()):
        Fi = np.linalg.inv(np.array(bd['frame'], float))
        for p in bd['parts']:
            out.append((bd['body'], p['product'], Fi @ np.array(p['matrix'], float)))
    return out


def part_poses(kin, q, parts=None):
    """[(product, 4 x 4 (mm) in the robot frame)] of each official part at joint angles q"""
    fk = kin.fk(q)
    out = []
    for body, prod, M in parts or arm_parts():
        T = fk[body].copy()
        T[:3, 3] *= 1000.0
        out.append((prod, T @ M))
    return out
