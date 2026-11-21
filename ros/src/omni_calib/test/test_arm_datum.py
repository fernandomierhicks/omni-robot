"""omni_calib.arm_datum (design.md 4.5, a proposal): the touch plan stays within a five-joint arm's reach, the fit finds
pan's zero (the joint the ruler can't see) from touches made by a known arm. The touch-off itself on the detailed sim
arm: simulation/v11 sil/studies/arm_datums.py.
    python -m pytest -q ros/src/omni_calib/test/test_arm_datum.py
"""

from __future__ import annotations

import math
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np

SRC = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SRC / "omni_calib"), str(SRC / "omni_arm")]

from omni_arm.kinematics import ArmConfig, ArmKinematics, config_path  # noqa: E402
from omni_calib import arm_datum as AD  # noqa: E402
from omni_calib import arm_load as AL  # noqa: E402

KIN = ArmKinematics(ArmConfig(config_path("d435")))
PLATE = 0.1815
DATUMS = [AD.Datum(n, np.array([0.150, y, PLATE + 0.035]), np.array([0.01, 0.01, 0.035]))
          for n, y in (("L", 0.13), ("R", -0.13))]


def test_the_touches_are_reachable_and_see_the_sides():
    t = AD.plan(KIN, DATUMS)
    normals = {tuple(int(v) for v in np.round(tc["normal"])) for tc in t}
    assert len(t) >= 16 and (0, 0, 1) in normals and ((1, 0, 0) in normals or (0, 1, 0) in normals)
    for tc in t:
        assert KIN.clear(tc["q0"])[0] and tc["q0"][5] == AD.OPEN


def test_the_touches_find_pans_zero():
    """Touches made by an arm whose pan reads 1.5 deg off (and the others 0.5 deg): pan's zero alone fitted."""
    truth = AL.LoadModel.nominal(KIN.cfg)
    for j, z in zip(truth.joints, np.radians([1.5, 0.5, -0.5, 0.5, 0.0])):
        j.zero = float(z)
    hits = []
    for tc in AD.plan(KIN, DATUMS):
        def gap(s):                          # the sphere's distance past touching, the encoders at s along the line
            q = tc["q0"] + s * (tc["q1"] - tc["q0"])
            c = AD._sphere(KIN, AL.true_angles(truth, KIN, q))
            return tc["normal"] @ (c - tc["point"]) - AD.SPHERE_R, q
        lo, hi = -0.5, 1.5
        if gap(lo)[0] * gap(hi)[0] > 0:
            continue
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if gap(mid)[0] > 0 else (lo, mid)
        hits.append((tc, gap(lo)[1]))
    start = AL.LoadModel([AL.JointLoad(**asdict(j)) for j in truth.joints], truth.links)
    start.joints[0].zero = 0.0
    m = AD.fit(KIN, start, hits, DATUMS, free=(0,))
    assert len(hits) >= 12 and abs(math.degrees(m.joints[0].zero) - 1.5) < 0.1
