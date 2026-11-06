"""The SO-101 calibrated against the D435 in the sim (robot_arm_d435.json: the nose until Nov 5, the riser since;
arm_calib.json is the nose's, arm_calib_riser.json the riser's) (omni_calib.arm_calib; design.md 4.5): how well a held tag
block seen by the D435's colour camera pins the arm and the camera, and what that buys at the floor, where the tidy
grasps and the camera cannot see the tool.

The truth is the robot's own chain (robot_arm_d435.json) perturbed per seed: joint zeros 3 deg, each link 1 mm and
0.5 deg, the camera 5 mm and 1.5 deg, sag past the encoders 0.4 / 0.4 / 0.6 deg per N m on lift / elbow / wrist
(est), gravity torques from MuJoCo with the block in hand. The block: a printed 40 mm cube, tag36h11 30 mm on three
faces, either gripped (its pose in the gripper unknown, 3 mm / 3 deg) or keyed to the gripper (0.3 mm / 0.3 deg).
Poses are random still arm poses, kept when the arm is clear of the robot and a tag is in the colour image, facing
the camera and not hidden: MuJoCo renders the D435's colour camera (1280 x 720, segmentation) with the arm there.
Corners are the truth's projections plus noise (iid sigma_px and a shift shared by a tag's corners, 0.6 sigma_px;
the Nov 4 bench measured 0.02-0.06 px on a still tag): the sim has no detector, so blur, focus and lighting are not
modelled. Intrinsics are held exact. Encoder ticks: 4096 a turn.

Scored at 30 top-down floor grasps 0.25-0.35 m from centre (the tool 1 cm up): a thing the camera sees at P (in its
frame, depth exact) is placed in base_link by the camera's pose, the arm goes there by IK on the model, and the error
is where the tool really ends up against P (horizontal and vertical, mm). The model is the drawing ("nominal") or the
calibrated config (apply_to_config + command_for). Not modelled: the base's creep between the look and the grasp,
depth error, servo droop and deadband (the settle-and-correct loop's job), encoder nonlinearity, backlash.

    python studies/arm_calib.py                       # the default grid, results/arm_calib.json
    python studies/arm_calib.py --seeds 2 --quick     # a smoke run
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "ros" / "src" / "omni_calib"))
sys.path.insert(0, str(REPO / "ros" / "src" / "omni_arm"))

from robot_export import arm_d435_params  # noqa: E402
from robot_model import build_xml  # noqa: E402
from omni_arm.kinematics import ArmConfig, ArmKinematics, q_for_gap  # noqa: E402
from omni_calib import arm_calib as ac  # noqa: E402
from omni_calib.geometry import Camera, inv  # noqa: E402
from omni_calib.targets import tag_corners  # noqa: E402

CONFIG = REPO / "ros" / "src" / "omni_arm" / "omni_arm" / "config" / "robot_arm_d435.json"
RAW = json.loads(CONFIG.read_text())
OUT = Path(__file__).resolve().parent / "results"
W, H, FOVY = 1280, 720, 42.0
CAM = Camera.from_fovy(FOVY, W, H)
TICK = 2 * math.pi / 4096
SAG_TRUE = np.radians([0.4, 0.4, 0.6])                  # rad per N m, lift / elbow / wrist (est)
ARM_J = ("arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex", "arm_wrist_flex", "arm_wrist_roll")
CUBE_KG = 0.025                                          # a printed 40 mm cube (est)
KEYED = dict(prior=dict(block_t=0.0003, block_r=math.radians(0.3)), zero_joints=(1, 2, 3, 4))


# -- the truth ------------------------------------------------------------------------------------------------------

def truth(seed: int, keyed: bool):
    rng = np.random.default_rng(1000 + seed)
    lay = ac.Layout(sag=True)
    x = np.zeros(lay.n)
    x[lay.zero] = rng.normal(0, math.radians(3.0), ac.N_J)
    L = np.zeros((ac.N_J, 5))
    L[:, :2] = rng.normal(0, math.radians(0.5), (ac.N_J, 2))
    L[:, 2:] = rng.normal(0, 0.001, (ac.N_J, 3))
    x[lay.link] = L.ravel()
    x[lay.sag] = SAG_TRUE
    x[lay.cam] = np.r_[rng.normal(0, math.radians(1.5), 3), rng.normal(0, 0.005, 3)]
    b = (0.0003, math.radians(0.3)) if keyed else (0.003, math.radians(3.0))
    x[lay.block] = np.r_[rng.normal(0, b[1], 3), rng.normal(0, b[0], 3)]
    return lay, x


class Scene:
    """The arm robot in MuJoCo with the tag cube in its fingers (the truth's pose), for gravity and occlusion."""

    def __init__(self, T_grip_block, tags):
        spec = mujoco.MjSpec.from_string(build_xml(arm_d435_params()))
        body = spec.body("arm_gripper")
        cube = body.add_geom()
        cube.name, cube.type = "calib_cube", mujoco.mjtGeom.mjGEOM_BOX
        cube.size = [0.0195, 0.0195, 0.0195]
        cube.pos = T_grip_block[:3, 3]
        cube.quat = _wxyz(T_grip_block[:3, :3])
        cube.rgba = [0.92, 0.92, 0.9, 1]
        cube.contype = cube.conaffinity = 0
        cube.mass = CUBE_KG
        self.tag_names = {}
        for i, (Tt, s) in tags.items():
            M = T_grip_block @ Tt
            g = body.add_geom()
            g.name, g.type = f"calib_tag{i}", mujoco.mjtGeom.mjGEOM_BOX
            g.size = [s / 2, s / 2, 0.0004]
            g.pos = M[:3, 3] + 0.0002 * M[:3, 2]
            g.quat = _wxyz(M[:3, :3])
            g.rgba = [0.05, 0.05, 0.05, 1]
            g.contype = g.conaffinity = 0
            g.density = 0.0
            self.tag_names[i] = g.name
        self.m = m = spec.compile()
        m.vis.global_.offwidth, m.vis.global_.offheight = W, H
        self.d = d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        for _ in range(300):
            mujoco.mj_step(m, d)
        self.base_q = d.qpos.copy()
        self.adr = [m.joint(j).qposadr[0] for j in ARM_J]
        self.dof = [m.joint(j).dofadr[0] for j in ARM_J]
        self.grip_adr = m.joint("arm_gripper").qposadr[0]
        self.grip_q = q_for_gap(ArmConfig(CONFIG), 0.040)
        self.tag_gid = {i: m.geom(n).id for i, n in self.tag_names.items()}
        self.cam = m.camera("d435_color").id
        self.r = mujoco.Renderer(m, H, W)
        self.r.enable_segmentation_rendering()

    def pose(self, q):
        d = self.d
        d.qpos[:] = self.base_q
        d.qvel[:] = 0
        for a, v in zip(self.adr, q):
            d.qpos[a] = v
        d.qpos[self.grip_adr] = self.grip_q
        mujoco.mj_forward(self.m, d)

    def gravity(self, q) -> np.ndarray:
        """The torque gravity puts on each arm joint at q (N m): - the bias force, still."""
        self.pose(q)
        return -np.array([self.d.qfrc_bias[k] for k in self.dof])

    def visible(self, q, ids) -> list:
        """The tags among ids that the colour camera sees whole at true joint angles q (inset points and centre on
        the tag's own pixels)."""
        self.pose(q)
        d, m = self.d, self.m
        self.r.update_scene(d, camera="d435_color")
        seg = self.r.render()[..., 0]
        pc, Rc = d.cam_xpos[self.cam], d.cam_xmat[self.cam].reshape(3, 3)
        f = (H / 2) / math.tan(math.radians(m.cam_fovy[self.cam]) / 2)
        out = []
        for i in ids:
            g = self.tag_gid[i]
            gp, gR = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3)
            s = m.geom_size[g][0] * 0.8
            pts = gp + np.array([(a, b, 0.0) for a, b in ((-s, -s), (s, -s), (s, s), (-s, s), (0, 0))]) @ gR.T
            c = (pts - pc) @ Rc
            if np.any(c[:, 2] > -1e-3):
                continue
            u = W / 2 + f * c[:, 0] / -c[:, 2]
            v = H / 2 - f * c[:, 1] / -c[:, 2]
            if np.all((u >= 0) & (u < W) & (v >= 0) & (v < H)) and \
                    np.all(seg[v.astype(int), u.astype(int)] == g):
                out.append(i)
        return out

    def close(self):
        self.r.close()


def _wxyz(R):
    x, y, z, w = ac.to_xyzw(R)
    return [w, x, y, z]


# -- data -----------------------------------------------------------------------------------------------------------

def collect(seed: int, keyed: bool, n: int, attempts: int = 60000):
    """n observations (measured q, true q, noise-free tag pixels) and the scene's visibility counts."""
    lay, x = truth(seed, keyed)
    arm = ac.ArmChain(RAW["arm"]["chain"])
    T0 = ac.frame_pose(RAW, "d435_color_optical_frame")
    block0, tags = ac.held_cube(RAW)
    Tb = ac._pose(x[lay.block], block0)
    Tc_inv = inv(ac._pose(x[lay.cam], T0))
    corr = lay.link_corr(x)
    kin = ArmKinematics(ArmConfig(CONFIG))
    sc = Scene(Tb, tags)
    rng = np.random.default_rng(2000 + seed)
    rg = kin.cfg.ranges
    obs, stats = [], dict(tried=0, clear=0, geometric=0, kept=0, tags=0, occluded=0)
    while len(obs) < n and stats["tried"] < attempts:
        stats["tried"] += 1
        q = np.r_[rng.uniform(-0.8, 0.8), [rng.uniform(*rg[k]) for k in range(1, 5)]]
        q = np.round(q / TICK) * TICK
        if not kin.clear(np.r_[q, 0.0])[0]:
            continue
        stats["clear"] += 1
        g = sc.gravity(q)
        qt = lay.true_q(x, q, g)
        M = Tc_inv @ arm.fk(qt, corr) @ Tb
        cand = {}
        for i, (Tt, s) in tags.items():
            Mt = M @ Tt
            P = tag_corners(s) @ Mt[:3, :3].T + Mt[:3, 3]
            if np.any(P[:, 2] < 0.08):
                continue
            if Mt[:3, 2] @ (-Mt[:3, 3] / np.linalg.norm(Mt[:3, 3])) < math.cos(math.radians(65)):
                continue
            uv = CAM.project(P)
            if np.all(np.isfinite(uv)) and uv[:, 0].min() > 8 and uv[:, 0].max() < W - 8 and \
                    uv[:, 1].min() > 8 and uv[:, 1].max() < H - 8:
                cand[i] = uv
        if not cand:
            continue
        stats["geometric"] += 1
        seen = sc.visible(qt, list(cand))
        stats["occluded"] += len(cand) - len(seen)
        if not seen:
            continue
        stats["kept"] += 1
        stats["tags"] += len(seen)
        tool = (arm.fk(qt, corr) @ kin._tool)[:3, 3]
        obs.append(dict(q=q, g=g, tags={i: cand[i] for i in seen}, tool=tool))
    sc_grav = sc.gravity
    return lay, x, obs, stats, sc, sc_grav, T0, block0, tags


def noisy(obs, sigma_px: float, rng):
    out = []
    for o in obs:
        t = {i: uv + rng.normal(0, sigma_px, uv.shape) + rng.normal(0, 0.6 * sigma_px, 2) for i, uv in o["tags"].items()}
        out.append(dict(q=o["q"], tags=t))
    return out


# -- scoring --------------------------------------------------------------------------------------------------------

def floor_targets(kin, n: int = 30):
    rng = np.random.default_rng(7)
    out = []
    while len(out) < n:
        p = np.array([rng.uniform(0.25, 0.35), rng.uniform(-0.1, 0.1), 0.01])
        yaw = rng.uniform(-1.5, 1.5)
        if kin.ik(p, yaw=yaw) is not None:
            out.append((p, yaw))
    return out


def grasp_errors(lay, x, gravity, T0, res=None, targets=None):
    """[(horizontal, vertical) mm] at the floor targets: the camera's view of each, the model's IK, the true tool. A
    target the model's IK can't reach fingers-down (omni_arm's 2 deg) is left out: summary() counts them. The
    floor grasps sit at the lift's range limit, so a few degrees of zero decide it."""
    arm = ac.ArmChain(RAW["arm"]["chain"])
    corr = lay.link_corr(x)
    Tc_true = ac._pose(x[lay.cam], T0)
    if res is None:
        kin, Tc_est = ArmKinematics(ArmConfig(CONFIG)), T0
    else:
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump(ac.apply_to_config(RAW, res), f)
        kin, Tc_est = ArmKinematics(ArmConfig(f.name)), res["T_base_cam"]
        Path(f.name).unlink()
    tool_T = kin._tool
    out = []
    for p, yaw in targets:
        Pc = inv(Tc_true) @ np.r_[p, 1.0]                  # what the camera measures
        p_est = (Tc_est @ Pc)[:3]
        q = kin.ik(p_est, yaw=yaw)
        if q is None:
            continue
        qc = ac.command_for(q[:5], res, gravity) if res is not None and res["sag_on"] else q[:5]
        tool = (arm.fk(lay.true_q(x, qc, gravity(qc)), corr) @ tool_T)[:3, 3]
        miss = kin.tool(np.r_[q[:5], 0.0])[:3, 3] - p_est            # the IK's own tolerance (2 mm), not calibration
        e = tool - p - miss
        out.append((1000 * math.hypot(e[0], e[1]), 1000 * abs(e[2])))
    return out


def summary(errs) -> dict:
    if not errs:
        return dict(h_med=math.nan, h_max=math.nan, v_med=math.nan, v_max=math.nan, tot_p95=math.nan, n=0)
    a = np.array(errs)
    tot = np.hypot(a[:, 0], a[:, 1])
    return dict(h_med=float(np.median(a[:, 0])), h_max=float(a[:, 0].max()), v_med=float(np.median(a[:, 1])),
                v_max=float(a[:, 1].max()), tot_p95=float(np.percentile(tot, 95)), n=len(a))


# -- the runs -------------------------------------------------------------------------------------------------------

def run_seed(args) -> list:
    seed, ns, noises, sag_off_n = args
    rows = []
    for keyed in (False, True):
        t0 = time.time()
        lay, x, obs, stats, sc, grav, T0, block0, tags = collect(seed, keyed, max(ns))
        t_collect = time.time() - t0
        kin = ArmKinematics(ArmConfig(CONFIG))
        targets = floor_targets(kin)
        nominal = summary(grasp_errors(lay, x, grav, T0, None, targets))
        tool = np.array([o["tool"] for o in obs])
        base = dict(seed=seed, block="keyed" if keyed else "gripped", stats=stats, collect_s=round(t_collect, 1),
                    tool_x=[float(tool[:, 0].min()), float(tool[:, 0].max())],
                    tool_z=[float(tool[:, 2].min()), float(tool[:, 2].max())], nominal=nominal)
        rows.append(dict(base, kind="nominal"))
        kw = KEYED if keyed else {}
        cases = [(n, 0.3, True) for n in ns] + [(sag_off_n, s, True) for s in noises if s != 0.3] + \
            [(sag_off_n, 0.3, False)]
        for n, s, sag in cases:
            o = noisy(obs[:n], s, np.random.default_rng(3000 + seed * 7 + n))
            t1 = time.time()
            res = ac.calibrate_arm(o, RAW["arm"]["chain"], CAM, T0, block0, tags, gravity=grav if sag else None,
                                   sigma_px=s, **kw)
            dt = time.time() - t1
            errs = summary(grasp_errors(lay, x, grav, T0, res, targets))
            errs["unreached"] = len(targets) - errs["n"]
            sig = ac.tool_sigma_mm(res, [kin.ik(p, yaw=y)[:5] for p, y in targets[:6]], kin._tool,
                                   grav if sag else None)
            rows.append(dict(base, kind="calibrated", n=n, sigma_px=s, sag_model=sag, rms_px=res["rms_px"],
                             solve_s=round(dt, 1), tool_sigma_mm=sig, err=errs,
                             zero_err_deg=[float(v) for v in np.degrees(res["x"][lay.zero] - x[lay.zero])],
                             cam_err=list(ac.pose_error_mm_deg(res["T_base_cam"], ac._pose(x[lay.cam], T0)))))
        sc.close()
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--quick", action="store_true", help="15 / 30 poses, one noise")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--out", default="", help="result file name (default arm_calib[_quick].json)")
    a = ap.parse_args()
    ns, noises = ((15, 30), (0.3,)) if a.quick else ((15, 30, 60, 120), (0.1, 0.3, 1.0))
    sag_off_n = 30 if a.quick else 60
    jobs = [(s, ns, noises, sag_off_n) for s in range(a.seeds)]
    t0 = time.time()
    rows = []
    with ProcessPoolExecutor(max_workers=min(a.workers, len(jobs))) as ex:
        for r in ex.map(run_seed, jobs):
            rows.extend(r)
    OUT.mkdir(exist_ok=True)
    f = OUT / (a.out or ("arm_calib_quick.json" if a.quick else "arm_calib.json"))
    f.write_text(json.dumps(dict(rows=rows, seconds=round(time.time() - t0, 1)), indent=1))
    report(rows)
    print(f"{f} ({time.time() - t0:.0f} s)")


def report(rows):
    def agg(sel, key):
        v = [r["err"][key] for r in sel if r["err"]["n"]]
        return f"{np.median(v):5.2f} [{min(v):.2f}-{max(v):.2f}]" if v else "  -  "
    for block in ("gripped", "keyed"):
        nom = [r for r in rows if r["block"] == block and r["kind"] == "nominal"]
        st = [r["stats"] for r in nom]
        print(f"\n{block}: kept {sum(s['kept'] for s in st)} of {sum(s['clear'] for s in st)} clear poses "
              f"({sum(s['tags'] for s in st) / max(1, sum(s['kept'] for s in st)):.2f} tags each; "
              f"{sum(s['occluded'] for s in st)} tags hidden by the arm)")
        print(f"  nominal: horiz med {np.median([r['nominal']['h_med'] for r in nom]):.1f} mm, max "
              f"{max(r['nominal']['h_max'] for r in nom):.1f}; vert med "
              f"{np.median([r['nominal']['v_med'] for r in nom]):.1f}, max {max(r['nominal']['v_max'] for r in nom):.1f}")
        cal = [r for r in rows if r["block"] == block and r["kind"] == "calibrated"]
        keys = sorted({(r["n"], r["sigma_px"], r["sag_model"]) for r in cal})
        for n, s, sag in keys:
            sel = [r for r in cal if (r["n"], r["sigma_px"], r["sag_model"]) == (n, s, sag)]
            print(f"  n {n:3d} px {s:.1f} sag {'on ' if sag else 'off'}: horiz med {agg(sel, 'h_med')} max "
                  f"{agg(sel, 'h_max')} | vert max {agg(sel, 'v_max')} | p95 {agg(sel, 'tot_p95')} | sigma "
                  f"{np.median([r['tool_sigma_mm'] for r in sel]):.2f} | rms {np.median([r['rms_px'] for r in sel]):.2f}"
                  f" | unreached {sum(r['err']['unreached'] for r in sel)}/{30 * len(sel)}")


if __name__ == "__main__":
    main()
