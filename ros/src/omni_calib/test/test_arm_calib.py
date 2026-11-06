"""arm_calib on synthetic data with a known truth, in the arm robot's own geometry (robot_arm_d435.json: the SO-101's
chain and the nose D435's colour camera, 1280 x 720 at 42 deg): a perturbed arm and camera, a held tag cube seen at
random still poses, recovered. Visibility here is geometry only (in the image, facing the camera); the sim study
(simulation/v11 sil/studies/arm_calib.py) adds the arm's own occlusion.
Run: python -m pytest -q ros/src/omni_calib/test"""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / "omni_calib"))
sys.path.insert(0, str(SRC / "omni_arm"))

from omni_calib import arm_calib as ac  # noqa: E402
from omni_calib.geometry import Camera, T, inv, rodrigues  # noqa: E402
from omni_calib.targets import tag_corners  # noqa: E402

RAW = json.loads((SRC / "omni_arm" / "omni_arm" / "config" / "robot_arm_d435.json").read_text())
CAM = Camera.from_fovy(42.0, 1280, 720)
RANGES = np.array([b["joint"]["range"] for b in RAW["arm"]["chain"] if b["joint"]])


def gravity(q):
    """A smooth stand-in for the gravity torques (N m) on the five joints."""
    return np.array([0.0, 0.9 * math.sin(q[1] + 0.3), 0.5 * math.cos(q[1] + q[2]), 0.15 * math.sin(q[1] + q[2] + q[3]),
                     0.0])


def truth(rng, sag=True):
    chain = RAW["arm"]["chain"]
    arm = ac.ArmChain(chain)
    lay = ac.Layout(sag=sag)
    x = np.zeros(lay.n)
    x[lay.zero] = rng.normal(0, math.radians(3.0), ac.N_J)
    L = np.zeros((ac.N_J, 5))
    L[:, :2] = rng.normal(0, math.radians(0.5), (ac.N_J, 2))
    L[:, 2:] = rng.normal(0, 0.001, (ac.N_J, 3))
    x[lay.link] = L.ravel()
    if sag:
        x[lay.sag] = np.radians([0.4, 0.4, 0.6])
    x[lay.cam] = np.r_[rng.normal(0, math.radians(1.5), 3), rng.normal(0, 0.005, 3)]
    x[lay.block] = np.r_[rng.normal(0, math.radians(3.0), 3), rng.normal(0, 0.003, 3)]
    return arm, lay, x


def observe(arm, lay, x, n, rng, noise_px=0.3, sag=True):
    T0 = ac.frame_pose(RAW, "d435_color_optical_frame")
    block0, tags = ac.held_cube(RAW)
    Tc_inv = inv(ac._pose(x[lay.cam], T0))
    Tb = ac._pose(x[lay.block], block0)
    corr = lay.link_corr(x)
    obs = []
    while len(obs) < n:
        q = np.r_[rng.uniform(-0.8, 0.8), rng.uniform(*RANGES[1]), rng.uniform(*RANGES[2]), rng.uniform(*RANGES[3]),
                  rng.uniform(*RANGES[4])]
        q = np.round(q / (2 * math.pi / 4096)) * (2 * math.pi / 4096)            # the encoder's ticks
        M = Tc_inv @ arm.fk(lay.true_q(x, q, gravity(q) if sag else None), corr) @ Tb
        seen = {}
        for i, (Tt, s) in tags.items():
            Mt = M @ Tt
            P = tag_corners(s) @ Mt[:3, :3].T + Mt[:3, 3]
            if np.any(P[:, 2] < 0.08):
                continue
            view = -Mt[:3, 3] / np.linalg.norm(Mt[:3, 3])
            if Mt[:3, 2] @ view < math.cos(math.radians(65)):
                continue
            uv = CAM.project(P)
            if not np.all(np.isfinite(uv)) or uv[:, 0].min() < 10 or uv[:, 0].max() > 1270 or uv[:, 1].min() < 10 \
                    or uv[:, 1].max() > 710:
                continue
            seen[i] = uv + rng.normal(0, noise_px, uv.shape)
        if seen:
            obs.append(dict(q=q, tags=seen))
    return obs, T0, block0, tags


def floor_poses(kin, n=30, rng=None):
    """Top-down grasps on the floor in reach, the tool 1 cm up: what a tidy commands."""
    rng = rng or np.random.default_rng(1)
    out = []
    while len(out) < n:
        p = np.array([rng.uniform(0.25, 0.35), rng.uniform(-0.1, 0.1), 0.01])
        q = kin.ik(p, yaw=rng.uniform(-1.5, 1.5))
        if q is not None:
            out.append(q[:5])
    return out


@pytest.fixture(scope="module")
def solved():
    rng = np.random.default_rng(3)
    arm, lay, x = truth(rng)
    obs, T0, block0, tags = observe(arm, lay, x, 60, rng)
    res = ac.calibrate_arm(obs, RAW["arm"]["chain"], CAM, T0, block0, tags, gravity=gravity, sigma_px=0.3)
    return arm, lay, x, res, T0


def test_fits_and_recovers_the_sag(solved):
    arm, lay, x, res, T0 = solved
    assert res["rms_px"] < 0.45
    assert np.degrees(np.abs(res["x"][lay.sag] - x[lay.sag]))[:2].max() < 0.1      # lift, elbow (wrist: little torque)


def test_tool_at_the_floor_seen_from_the_camera(solved):
    """Where the tool really goes, in the camera's frame, at floor grasps the camera never saw: calibrated within
    2.5 mm (the gripper's own drawing past the block is not seen); the drawing's model (3 deg zeros) centimetres off;
    the fit's own sigma covers the error."""
    from omni_arm.kinematics import ArmConfig, ArmKinematics
    arm, lay, x, res, T0 = solved
    kin = ArmKinematics(ArmConfig(SRC / "omni_arm" / "omni_arm" / "config" / "robot_arm_d435.json"))
    truth_res = dict(res, x=x)
    cal, nom = [], []
    qs = floor_poses(kin)
    for q in qs:
        g = gravity(q)
        p_true = ac.tool_in_camera(truth_res, x, q, kin._tool, g)
        cal.append(np.linalg.norm(ac.tool_in_camera(res, res["x"], q, kin._tool, g) - p_true))
        nom.append(np.linalg.norm(ac.tool_in_camera(res, np.zeros(lay.n), q, kin._tool) - p_true))
    assert max(cal) < 0.0025, max(cal)
    assert np.median(nom) > 0.01
    sig = ac.tool_sigma_mm(res, qs[:8], kin._tool, gravity)
    assert 0.0 < sig < 5.0 and 1000 * max(cal) < 3 * sig, (sig, max(cal))


def test_a_keyed_block_pins_the_fingers():
    """The block keyed to the gripper (its pose known to 0.3 mm / 0.3 deg) and roll's zero solved: the tool within
    0.6 mm at the floor and a fit sigma ~0.3 mm, where a gripped block leaves the gripper's drawing (~2 mm)."""
    from omni_arm.kinematics import ArmConfig, ArmKinematics
    kin = ArmKinematics(ArmConfig(SRC / "omni_arm" / "omni_arm" / "config" / "robot_arm_d435.json"))
    rng = np.random.default_rng(2)
    arm, lay, x = truth(rng)
    x[lay.block] = np.r_[rng.normal(0, math.radians(0.3), 3), rng.normal(0, 0.0003, 3)]
    obs, T0, block0, tags = observe(arm, lay, x, 60, rng)
    res = ac.calibrate_arm(obs, RAW["arm"]["chain"], CAM, T0, block0, tags, gravity=gravity, sigma_px=0.3,
                           zero_joints=(1, 2, 3, 4), prior=dict(block_t=0.0003, block_r=math.radians(0.3)))
    qs = floor_poses(kin)
    truth_res = dict(res, x=x)
    err = [np.linalg.norm(ac.tool_in_camera(res, res["x"], q, kin._tool, gravity(q))
                          - ac.tool_in_camera(truth_res, x, q, kin._tool, gravity(q))) for q in qs]
    assert max(err) < 0.0006, max(err)
    assert ac.tool_sigma_mm(res, qs[:8], kin._tool, gravity) < 0.5


def test_apply_to_config_drops_into_omni_arm(solved, tmp_path):
    from omni_arm.kinematics import ArmConfig, ArmKinematics
    arm, lay, x, res, _ = solved
    out = ac.apply_to_config(RAW, res)
    f = tmp_path / "robot_arm_cal.json"
    f.write_text(json.dumps(out))
    kin = ArmKinematics(ArmConfig(f))
    fit = ac.model(res)
    rng = np.random.default_rng(5)
    for _ in range(20):
        q = np.r_[rng.uniform(RANGES[:, 0], RANGES[:, 1]), 0.0]
        assert np.allclose(kin.fk(q)["arm_gripper"], fit(q[:5]), atol=1e-9)
    assert np.allclose(ac.frame_pose(out, "d435_color_optical_frame"), res["T_base_cam"], atol=1e-9)
    # the camera entry (MuJoCo convention) stays the depth optical frame, as the drawing's does
    for raw in (RAW, out):
        c = raw["cameras"][0]
        Tmj = T(np.asarray(c["R"]), c["pos"])
        Tmj[:3, :3] = Tmj[:3, :3] @ ac.MJ_TO_OPTICAL
        assert np.allclose(Tmj, ac.frame_pose(raw, "d435_depth_optical_frame"), atol=1e-6)
    assert set(out["arm"]["compliance"]) == {"shoulder_lift", "elbow_flex", "wrist_flex"}


def test_command_for_undoes_the_sag(solved):
    arm, lay, x, res, _ = solved
    q_kin = np.array([0.1, 0.4, 0.6, 1.0, 0.0])
    q = ac.command_for(q_kin, res, gravity)
    qt = lay.true_q(res["x"], q, gravity(q)) - res["x"][lay.zero]
    assert np.abs(qt - q_kin).max() < 1e-6


def test_store_value_passes_limits(solved, tmp_path):
    from omni_calib.results import Store
    _, _, _, res, _ = solved
    value, metrics = ac.store_value(res, tool_sigma=0.8)
    run = Store(tmp_path).add("arm_d435", "arm", value, metrics, method="synthetic, 60 poses")
    assert run["passed"], run["failures"]
    value, metrics = ac.store_value(res, tool_sigma=3.0)
    assert not Store(tmp_path).add("arm_d435", "arm", value, metrics)["passed"]
