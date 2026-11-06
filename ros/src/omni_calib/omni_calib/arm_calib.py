"""The SO-101 against the nose D435 (design.md 4.5; item 9 in the README): the arm's joint zeros, its links, how far
its joints sag under their own weight past the encoders, and where the camera sits on the arm, all at once, from
AprilTags on a block the gripper holds, seen by the D435's colour camera at many still arm poses.

    chain = json.load(open("robot_arm_d435.json"))["arm"]["chain"]
    res = calibrate_arm(obs, chain, cam, T_base_cam0, block0, tags, gravity=None)
    raw2 = apply_to_config(raw, res)          # a drop-in robot_arm json: omni_arm's kinematics uses it as it is

What is solved (each a correction to the drawing, held near it by a prior, so a direction the poses cannot see stays at
the drawing instead of wandering):
  zeros       each joint's angle offset (true = measured + offset): LeRobot's calibration puts 0 at the middle of a
              range recorded by hand against printed stops, so a few degrees off is expected (prior 5 deg)
  links       each jointed body's fixed pose in its parent: 3 mm along x y z and 1 deg about x y (prior 2 mm, 1 deg);
              the turn about the joint's own axis is the zero's, so it is left out
  sag         compliance past the encoder (horn spline, brackets, printed links) on lift, elbow and wrist flex:
              true = measured + offset + c * gravity torque (rad / N m; only with a gravity(q) model)
  camera      the colour camera's optical frame in base_link (prior 2 cm, 3 deg: robot_frames' bounds)
  block       the tag block's pose in the gripper's frame (prior 1 cm, 10 deg: it is gripped, not bolted)

The arm's mount stays at the drawing: these views only see the camera relative to the arm, which is what a pick needs
(the object from the camera, the arm to it); base_link's place under both comes from the lidar and the floor.

Observations: [{"q": (5,) measured joint angles (rad, the arm's convention), "tags": {tag id: (4, 2) corner pixels in
apriltag's order}}], each taken with the arm still (the same final approach direction every time: design.md 4.5).
Intrinsics are held (calibrate them first, item 1). The residual is the corners' reprojection in px (Huber).
"""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import least_squares

from omni_calib.geometry import Camera, T, inv, log_rot, rodrigues, rot_deg
from omni_calib.targets import tag_corners

N_J = 5                                    # pan, lift, elbow, wrist flex, wrist roll (the gripper's jaw is not used)
SAG_JOINTS = (1, 2, 3)                     # lift, elbow, wrist flex: the ones gravity loads
PRIOR = dict(zero=math.radians(5.0), link_t=0.002, link_r=math.radians(1.0), sag=math.radians(2.0),
             cam_t=0.02, cam_r=math.radians(3.0), block_t=0.01, block_r=math.radians(10.0))
MJ_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])


def quat_wxyz(q) -> np.ndarray:
    w, x, y, z = q
    return quat_xyzw((x, y, z, w))


def quat_xyzw(q) -> np.ndarray:
    x, y, z, w = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_xyzw(R) -> list:
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0, 0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    return [float(v) for v in q]


# -- the chain --------------------------------------------------------------------------------------------------------

class ArmChain:
    """omni_arm's chain (robot_arm json "arm" "chain"): each body's fixed pose in its parent (pos, quat w x y z), then
    its joint's turn about axis through pos (every SO-101 joint: z through the body's origin). fk() gives the last
    body's (arm_gripper's) pose in base_link, as omni_arm.kinematics.ArmKinematics.fk(q)["arm_gripper"]."""

    def __init__(self, chain):
        self.chain = chain
        self.fixed = [T(quat_wxyz(b["quat"]), b["pos"]) for b in chain]
        self.jointed = [k for k, b in enumerate(chain) if b["joint"]]
        for k in self.jointed:
            j = chain[k]["joint"]
            if np.linalg.norm(np.asarray(j["axis"]) - (0, 0, 1)) > 1e-9 or np.any(np.asarray(j["pos"]) != 0):
                raise ValueError(f"{chain[k]['body']}: only joints about z through the body's origin")
        if len(self.jointed) != N_J:
            raise ValueError(f"{len(self.jointed)} joints in the chain, expected {N_J}")

    def fk(self, q, corr=None) -> np.ndarray:
        """arm_gripper in base_link at joint angles q (5, already the true angles); corr: [(4 x 4) per jointed body]
        applied to its fixed pose (F @ corr) before the joint's turn."""
        M = np.eye(4)
        k = 0
        for b, F in enumerate(self.fixed):
            M = M @ F
            if self.chain[b]["joint"]:
                if corr is not None:
                    M = M @ corr[k]
                c, s = math.cos(q[k]), math.sin(q[k])
                M = M @ np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
                k += 1
        return M


# -- the parameters ---------------------------------------------------------------------------------------------------

class Layout:
    """Where each correction lives in the solver's vector, and its prior."""

    def __init__(self, sag: bool, prior: dict | None = None):
        PRIOR = {**globals()["PRIOR"], **(prior or {})}
        self.names, sig = [], []

        def add(name, n, s):
            setattr(self, name, slice(len(self.names), len(self.names) + n))
            self.names.extend(f"{name}{i}" for i in range(n))
            sig.extend(s if isinstance(s, (list, tuple)) else [s] * n)
        add("zero", N_J, PRIOR["zero"])
        add("link", 5 * N_J, [PRIOR["link_r"], PRIOR["link_r"], PRIOR["link_t"], PRIOR["link_t"], PRIOR["link_t"]] * N_J)
        add("sag", len(SAG_JOINTS) if sag else 0, PRIOR["sag"])
        add("cam", 6, [PRIOR["cam_r"]] * 3 + [PRIOR["cam_t"]] * 3)
        add("block", 6, [PRIOR["block_r"]] * 3 + [PRIOR["block_t"]] * 3)
        self.sigma = np.array(sig)
        self.n = len(self.names)

    def link_corr(self, x) -> list:
        """Each jointed body's correction: a turn about its x and y (rad), then a shift (m), in its own frame."""
        L = x[self.link].reshape(N_J, 5)
        return [T(rodrigues([a, b, 0.0]), [px, py, pz]) for a, b, px, py, pz in L]

    def true_q(self, x, q, g=None) -> np.ndarray:
        """The joints' true angles from the measured ones: + zero offset, + c * gravity torque."""
        qt = np.asarray(q, float)[:N_J] + x[self.zero]
        if g is not None and self.sag.stop > self.sag.start:
            qt = qt.copy()
            qt[list(SAG_JOINTS)] += x[self.sag] * np.asarray(g)[list(SAG_JOINTS)]
        return qt


def _pose(x6, T0) -> np.ndarray:
    """A correction (rotation vector, shift) in the parent's frame on top of the drawing's T0."""
    return T(rodrigues(x6[:3]) @ T0[:3, :3], T0[:3, 3] + x6[3:])


def held_cube(raw: dict, cube: float = 0.04, tag: float = 0.03):
    """(block0, tags) for a printed cube held in the fingers, one face on the fixed finger's inner face, centred on the
    tool point along the fingers: tag36h11 ids 0, 1, 2 on the face past the fingertips (tool +x) and the two sides
    (tool +y, -y); the gripped faces carry none. block0 is the drawing's guess (in arm_gripper's frame), solved."""
    t = raw["arm"]["tool"]
    T_tool = T(quat_wxyz(t["quat"]), t["pos"])
    block0 = T_tool @ T(np.eye(3), [0.0, 0.0, cube / 2])
    h = cube / 2
    faces = {0: ((0, 1, 0), (0, 0, 1), (1, 0, 0)), 1: ((1, 0, 0), (0, 0, -1), (0, 1, 0)),
             2: ((1, 0, 0), (0, 0, 1), (0, -1, 0))}
    tags = {i: (T(np.column_stack(f), h * np.array(f[2], float)), tag) for i, f in faces.items()}
    return block0, tags


# -- the solve --------------------------------------------------------------------------------------------------------

def calibrate_arm(obs, chain, cam: Camera, T_base_cam0, block0, tags: dict, gravity=None, sigma_px: float = 0.5,
                  loss: str = "huber", f_scale: float = 2.0, fit: tuple = ("zero", "link", "sag", "cam", "block"),
                  zero_joints: tuple = (1, 2, 3), prior: dict | None = None) -> dict:
    """obs: as the module says. chain: the drawing's. cam: the colour camera's intrinsics (optical frame). T_base_cam0:
    the drawing's base_link -> colour optical (4 x 4). block0: the drawing's tag block in arm_gripper's frame (4 x 4).
    tags: {id: (T_block_tag 4 x 4, black square's side m)}. gravity: q (5,) -> torque gravity puts on each joint (N m)
    for the sag terms, or None. fit: the groups solved; the others stay at the drawing (to compare models).
    zero_joints: the joints whose zeros are solved. Not pan's or roll's by default: to these views pan's zero is the
    camera turned about the pan axis, and roll's the block turned about the roll axis (only the difference is seen;
    the camera and block take it). The lift, elbow and wrist zeros also trade exactly against a shift of the next
    link across the arm, so they come out as effective zeros in the drawing's links: the model's predictions are what
    the views pin, not each number. prior: overrides of PRIOR. A block keyed to the gripper's own faces (block_t /
    block_r tight, and roll in zero_joints: its zero is then seen) pins the fingers' place past the wrist too
    (synthetic: tool sigma ~0.3 mm); a block only gripped leaves that at the drawing (~link_t, ~2 mm).

    Returns the corrections (zeros deg, links mm / deg, sag deg per N m, the camera and block poses and how far they
    moved), rms_px, per-parameter 1-sigma from the fit's information, and x for model()."""
    arm = ArmChain(chain)
    lay = Layout(sag=gravity is not None, prior=prior)
    corners ={i: np.c_[tag_corners(s), np.ones(4)] @ Tt.T for i, (Tt, s) in tags.items()}     # in the block's frame
    G = [np.asarray(gravity(o["q"]), float) if gravity is not None else None for o in obs]
    uv = [{i: np.asarray(p, float) for i, p in o["tags"].items() if i in tags} for o in obs]
    n_res = 2 * 4 * sum(len(u) for u in uv)
    free = np.zeros(lay.n, bool)
    for g in fit:
        free[getattr(lay, g)] = True
    if "zero" in fit:
        free[lay.zero] = False
        free[[lay.zero.start + j for j in zero_joints]] = True

    def unpack(z):
        x = np.zeros(lay.n)
        x[free] = z
        return x

    def fun(z):
        x = unpack(z)
        corr = lay.link_corr(x)
        Tc_inv = inv(_pose(x[lay.cam], T_base_cam0))
        Tb = _pose(x[lay.block], block0)
        res = []
        for o, u, g in zip(obs, uv, G):
            M = Tc_inv @ arm.fk(lay.true_q(x, o["q"], g), corr) @ Tb
            for i, p in u.items():
                P = corners[i] @ M.T
                res.append(((cam.project(P[:, :3]) - p) / sigma_px).ravel())
        res.append(x[free] / lay.sigma[free])
        return np.concatenate(res)

    sol = least_squares(fun, np.zeros(int(free.sum())), loss=loss, f_scale=f_scale, x_scale="jac", method="trf")
    x = unpack(sol.x)
    r = sol.fun[:n_res] * sigma_px
    J = sol.jac
    sig = np.full(lay.n, np.nan)
    cov = None
    try:
        cov = np.linalg.inv(J.T @ J)
        sig[free] = np.sqrt(np.diag(cov))
    except np.linalg.LinAlgError:
        pass
    Tc, Tb = _pose(x[lay.cam], T_base_cam0), _pose(x[lay.block], block0)
    L = x[lay.link].reshape(N_J, 5)
    names = [chain[k]["joint"]["name"] for k in arm.jointed]
    return dict(
        x=x, layout=lay, chain=chain, sag_on=gravity is not None, free=free, cov=cov,
        zeros_deg={n: math.degrees(v) for n, v in zip(names, x[lay.zero])},
        links={chain[k]["body"]: dict(turn_deg=[math.degrees(L[i, 0]), math.degrees(L[i, 1])],
                                      shift_mm=[1000 * v for v in L[i, 2:]]) for i, k in enumerate(arm.jointed)},
        sag_deg_per_nm={names[j]: math.degrees(v) for j, v in zip(SAG_JOINTS, x[lay.sag])} if gravity else {},
        T_base_cam=Tc, T_gripper_block=Tb, T_base_cam0=np.asarray(T_base_cam0, float),
        cam_moved_mm=1000 * float(np.linalg.norm(Tc[:3, 3] - T_base_cam0[:3, 3])),
        cam_turned_deg=rot_deg(T_base_cam0[:3, :3], Tc[:3, :3]),
        rms_px=float(np.sqrt(np.mean(r ** 2))) if len(r) else math.nan, n_obs=len(obs), n_corners=n_res // 2,
        sigma={"zero_deg": [math.degrees(v) for v in sig[lay.zero]],
               "cam_rot_deg": [math.degrees(v) for v in sig[lay.cam][:3]],
               "cam_mm": [1000 * v for v in sig[lay.cam][3:]]},
        cost=float(sol.cost), status=int(sol.status))


def model(res: dict):
    """(gripper(q_measured, g=None) -> arm_gripper's 4 x 4 in base_link, the calibrated model)."""
    arm = ArmChain(res["chain"])
    lay, x = res["layout"], res["x"]
    corr = lay.link_corr(x)
    return lambda q, g=None: arm.fk(lay.true_q(x, q, g), corr)


def tool_in_camera(res: dict, x, q, T_tool, g=None) -> np.ndarray:
    """The tool point in the camera's optical frame at measured q under corrections x: what a pick needs (the object
    from the camera, the tool to it), free of the pan / camera trade."""
    arm = ArmChain(res["chain"])
    lay = res["layout"]
    M = inv(_pose(x[lay.cam], res["T_base_cam0"])) @ arm.fk(lay.true_q(x, q, g), lay.link_corr(x)) @ T_tool
    return M[:3, 3]


def tool_sigma_mm(res: dict, qs, T_tool, gravity=None, h: float = 1e-6) -> float:
    """The fit's own 1-sigma of the tool point in the camera's frame (mm, the worst axis over poses qs, e.g. floor
    grasps the camera never saw): the solve's covariance carried through the chain. The pass metric: the joint zeros
    alone are not pinned (calibrate_arm's note), the predictions are."""
    if res["cov"] is None:
        return math.nan
    idx = np.flatnonzero(res["free"])
    worst = 0.0
    for q in qs:
        g = gravity(q) if gravity is not None else None
        p0 = tool_in_camera(res, res["x"], q, T_tool, g)
        J = np.empty((3, len(idx)))
        for c, i in enumerate(idx):
            x = res["x"].copy()
            x[i] += h
            J[:, c] = (tool_in_camera(res, x, q, T_tool, g) - p0) / h
        worst = max(worst, float(np.sqrt(np.max(np.diag(J @ res["cov"] @ J.T)))))
    return 1000 * worst


def command_for(q_kin, res: dict, gravity, iters: int = 4) -> np.ndarray:
    """The joint command (measured angles) that leaves the arm at q_kin in apply_to_config's chain (zeros folded in)
    once it has sagged: q + c * g(q) = q_kin, by fixed point (the sag is a small fraction of a degree per N m)."""
    q = np.array(q_kin, float)
    if not res["sag_on"]:
        return q
    lay, x = res["layout"], res["x"]
    for _ in range(iters):
        g = np.asarray(gravity(q), float)
        q[:N_J] = np.asarray(q_kin, float)[:N_J]
        q[list(SAG_JOINTS)] -= x[lay.sag] * g[list(SAG_JOINTS)]
    return q


# -- the result into the robot's config -------------------------------------------------------------------------------

def _frames(raw) -> dict:
    return {f["child"]: f for f in raw["frames"]}


def frame_pose(raw, child: str) -> np.ndarray:
    """base_link -> child from a robot_arm json's frames."""
    fr = _frames(raw)
    M = np.eye(4)
    path = []
    while child in fr:
        path.append(fr[child])
        child = fr[child]["parent"]
    for f in reversed(path):
        M = M @ T(quat_xyzw(f["quat"]), f["xyz"])
    return M


def apply_to_config(raw: dict, res: dict, cam_frame: str = "d435_color_optical_frame", cam_link: str = "d435_link",
                    camera: str = "d435") -> dict:
    """A copy of a robot_arm json with the result in it, for omni_arm as it is (config:=<path>):
      - each jointed body's fixed pose corrected, and its zero offset folded in as a turn about its joint (F Rz(off)
        Rz(q) = F Rz(q + off)), so the measured angles go straight in; joint ranges stay in measured angles (the
        servo's own limits);
      - the camera: its link frame moved so that cam_frame lands where solved (the colour-to-depth offset kept at the
        drawing's: on the robot take the D435's own factory extrinsic), and the named camera's pos / R with it;
      - the sag (rad / N m) under arm["compliance"], which the kinematics does not read yet (command_for does)."""
    import copy
    out = copy.deepcopy(raw)
    arm = ArmChain(raw["arm"]["chain"])
    lay, x = res["layout"], res["x"]
    corr = lay.link_corr(x)
    for i, k in enumerate(arm.jointed):
        a = x[lay.zero][i]
        Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
        F = arm.fixed[k] @ corr[i] @ T(Rz, [0, 0, 0])
        w = to_xyzw(F[:3, :3])
        b = out["arm"]["chain"][k]
        b["pos"] = [float(v) for v in F[:3, 3]]
        b["quat"] = [w[3], w[0], w[1], w[2]]
    if res["sag_on"]:
        out["arm"]["compliance"] = {kk: math.radians(v) for kk, v in res["sag_deg_per_nm"].items()}
    # the camera's link: T_base_link = T_base_cam @ inv(T_link_cam), the link -> cam offset from the drawing
    T_link_cam = inv(frame_pose(raw, cam_link)) @ frame_pose(raw, cam_frame)
    T_bl = res["T_base_cam"] @ inv(T_link_cam)
    fr = _frames(out)[cam_link]
    T_parent = frame_pose(raw, fr["parent"])
    T_pl = inv(T_parent) @ T_bl
    fr["xyz"] = [float(v) for v in T_pl[:3, 3]]
    fr["quat"] = to_xyzw(T_pl[:3, :3])
    fr["note"] = fr.get("note", "") + " [calibrated: omni_calib.arm_calib]"
    # the named camera (MuJoCo convention, its depth origin = the link): keep its pose relative to the link
    for c in out["cameras"]:
        if c["name"] == camera:
            T_old = frame_pose(raw, cam_link)
            Tc_mj = T(np.asarray(c["R"], float), c["pos"])
            Tn = T_bl @ inv(T_old) @ Tc_mj
            c["pos"] = [float(v) for v in Tn[:3, 3]]
            c["R"] = [[float(v) for v in row] for row in Tn[:3, :3]]
    out["generated_by"] = str(raw.get("generated_by", "")) + " + omni_calib.arm_calib"
    return out


def store_value(res: dict, tool_sigma: float, cam_link_pose: np.ndarray | None = None) -> tuple[dict, dict]:
    """(value, metrics) for results.Store (kind "arm"): the corrections as numbers, plus the camera as a frame for
    robot_frames when its link pose is given. tool_sigma: tool_sigma_mm over the tidy's floor grasps."""
    value = dict(zeros_deg=res["zeros_deg"], links=res["links"], sag_deg_per_nm=res["sag_deg_per_nm"],
                 T_base_cam=[[float(v) for v in row] for row in res["T_base_cam"]],
                 T_gripper_block=[[float(v) for v in row] for row in res["T_gripper_block"]])
    if cam_link_pose is not None:
        value["frame"] = dict(child="d435_link", xyz=[float(v) for v in cam_link_pose[:3, 3]],
                              quat=to_xyzw(cam_link_pose[:3, :3]))
    metrics = dict(rms_px=res["rms_px"], n_obs=res["n_obs"], cam_moved_mm=res["cam_moved_mm"],
                   cam_turned_deg=res["cam_turned_deg"], tool_sigma_mm=float(tool_sigma))
    return value, metrics


def pose_error_mm_deg(Ta, Tb) -> tuple[float, float]:
    return 1000 * float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])), math.degrees(float(np.linalg.norm(
        log_rot(Ta[:3, :3].T @ Tb[:3, :3]))))
