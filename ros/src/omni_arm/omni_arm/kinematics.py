"""The SO-101 on the robot as its own code knows it (design.md 11): config/robot_arm.json, written from the sim by
simulation/v11 sil/robot_export.py (the official chain as the sim builds it, the mount, the finger pads, the wrist
camera, the stereo pods, the bin, the robot's keep-out volumes).

  ArmKinematics.fk(q)          every link's pose in base_link (4 x 4), the tool point, the jaw, the wrist camera
  ArmKinematics.ik(p, a, yaw)  joint angles that put the tool point at p with the fingers along a (unit, base_link);
                               with a straight down the wrist's roll also turns the grasp to yaw (the opening's
                               direction, rad); damped least squares on the four joints that place the tool, the roll
                               solved after. None when out of reach or out of the joints' ranges
  ArmKinematics.clear(q)       the arm's capsules against the floor and the robot's keep-out volumes

Joint angles are radians in the sim's (and LeRobot's "new calibration") convention: 0 = the middle of each joint's
range. Frames: base_link = the floor under the wheel circle's centre, x forward, z up (REP-103); the tool frame's x runs
along the fingers, its z from the fixed finger toward the moving one. No ROS here: the sim's harness uses it directly.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

CONFIG = Path(__file__).resolve().parent / "config" / "robot_arm.json"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
DOWN = np.array([0.0, 0.0, -1.0])


def quat_mat(q) -> np.ndarray:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def axis_mat(axis, a: float) -> np.ndarray:
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def tf(R=None, p=None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


class ArmConfig:
    def __init__(self, path: str | Path | None = None):
        self.raw = raw = json.loads(Path(path or CONFIG).read_text())
        a = raw["arm"]
        self.joints = tuple(a["joints"])
        self.chain = a["chain"]
        self.ranges = np.array([c["joint"]["range"] for c in self.chain if c["joint"]] + [a["jaw"]["range"]])
        self.tool = a["tool"]
        self.jaw = a["jaw"]
        self.pads = a["pads"]
        self.wrist_cam = a["wrist_cam"]
        self.capsules = a["capsules"]
        self.servos = a["servos"]
        self.keep_out = raw["keep_out"]
        self.cameras = raw["cameras"]
        self.bin = raw["bin"]
        self.poses = {k: np.array(v, float) for k, v in raw.get("poses", {}).items()}   # the layout's fixed poses
        self.arm_out = raw["arm_out"]
        self.base = raw["base"]
        self.frames = raw["frames"]


class ArmKinematics:
    def __init__(self, cfg: ArmConfig | None = None):
        self.cfg = c = cfg or ArmConfig()
        # each body: its fixed transform in its parent, then (if it has one) the joint's turn about its axis
        self._fixed = [tf(quat_mat(b["quat"]), b["pos"]) for b in c.chain]
        self._axis = [(np.array(b["joint"]["axis"]), np.array(b["joint"]["pos"])) if b["joint"] else None
                      for b in c.chain]
        self.bodies = [b["body"] for b in c.chain]
        self._tool = tf(quat_mat(c.tool["quat"]), c.tool["pos"])
        self._jaw = tf(quat_mat(c.jaw["quat"]), c.jaw["pos"])
        self._jaw_axis = np.array(c.jaw["axis"])
        wc = c.wrist_cam
        self._cam = tf(None, wc["pos"]) @ tf(quat_mat(wc["cam_quat"]), wc["cam_pos"])
        self.lo, self.hi = c.ranges[:, 0], c.ranges[:, 1]
        self._caps = [(cap["body"], np.array(cap["a"]), np.array(cap["b"]), cap["r"]) for cap in c.capsules]
        self.mount = self._fixed[0][:3, 3].copy()

    # -- forward ------------------------------------------------------------------------------------------------
    def fk(self, q) -> dict:
        """{body: 4 x 4 in base_link} plus "tool", "arm_jaw", "wrist_cam" (a MuJoCo camera: it looks along -z)."""
        q = np.asarray(q, float)
        out = {}
        T = np.eye(4)
        k = 0
        for name, F, ax in zip(self.bodies, self._fixed, self._axis):
            T = T @ F
            if ax is not None:
                axis, jp = ax
                T = T @ tf(None, jp) @ tf(axis_mat(axis, q[k])) @ tf(None, -jp)
                k += 1
            out[name] = T
        g = out["arm_gripper"]
        out["tool"] = g @ self._tool
        out["arm_jaw"] = g @ self._jaw @ tf(axis_mat(self._jaw_axis, q[5] if len(q) > 5 else 0.0))
        out["wrist_cam"] = g @ self._cam
        return out

    def tool(self, q) -> np.ndarray:
        return self._chain(q)[0]

    def _chain(self, q):
        """(the tool's 4 x 4, [(each joint's axis point, its axis direction)] in base_link)."""
        q = np.asarray(q, float)
        T = np.eye(4)
        k = 0
        frames = []
        for F, ax in zip(self._fixed, self._axis):
            T = T @ F
            if ax is not None:
                axis, jp = ax
                R = T[:3, :3]
                frames.append((R @ jp + T[:3, 3], R @ axis))
                J = np.eye(4)
                J[:3, :3] = axis_mat(axis, q[k])
                if jp.any():
                    J[:3, 3] = jp - J[:3, :3] @ jp
                T = T @ J
                k += 1
        return T @ self._tool, frames

    # -- inverse --------------------------------------------------------------------------------------------------
    def ik(self, p, approach=DOWN, yaw: float | None = None, seed=None, tol_pos=0.002, tol_ang=math.radians(2.0),
           flip_ok: bool = True):
        """Joint angles (6, the gripper's left at seed's or 0) that put the tool point at p with the fingers along
        approach, or None. yaw: the opening direction's heading in base_link (a top-down grasp only; None = keep the
        roll at the seed's); flip_ok: yaw + pi will do too (a line of motion; a grasp needs the exact one). Tries the
        seed, then a few spread starts; the first that converges."""
        p = np.asarray(p, float)
        a = np.asarray(approach, float)
        a = a / np.linalg.norm(a)
        seed = np.zeros(6) if seed is None else np.asarray(seed, float).copy()
        pan0 = math.atan2(-(p[1] - self.mount[1]), p[0] - self.mount[0])        # + pan turns clockwise (axis -z)
        starts = [seed[:4], np.array([pan0, 0.6, 0.3, 0.5]), np.array([pan0, -0.6, 0.9, 1.0]),
                  np.array([pan0, 0.6, -0.4, 1.3]), np.array([pan0, -1.2, 1.4, 0.4])]
        for s in starts:
            q6 = np.concatenate([s, [seed[4], seed[5]]])
            ok = True
            # the roll's axis passes ~8 mm from the tool point: place the tool, roll, place it again with that roll
            for _ in range(4 if yaw is not None else 1):
                q = self._solve4(p, a, q6[:4], q6[4])
                if q is None:
                    ok = False
                    break
                q6[:4] = q
                if yaw is not None:
                    r = self._roll_for(q6, yaw, flip_ok)
                    if r is None:
                        ok = False
                        break
                    if abs(r - q6[4]) < 1e-5:
                        break
                    q6[4] = r
            if not ok:
                continue
            T = self.tool(q6)
            if np.linalg.norm(T[:3, 3] - p) < tol_pos and math.acos(min(1.0, T[:3, 0] @ a)) < tol_ang:
                return q6
        return None

    def _solve4(self, p, a, q0, roll, iters=100):
        """Damped least squares on pan, lift, elbow and wrist flex (the geometric Jacobian: joint k turns the tool
        point about its axis z_k through o_k, d p = z_k x (p - o_k), and the fingers' direction, d a = z_k x a)."""
        q = np.clip(np.asarray(q0, float), self.lo[:4], self.hi[:4])
        w = 0.15                                          # m per unit of direction error
        lam = 1e-4
        e = None
        for _ in range(iters):
            T, frames = self._chain(np.concatenate([q, [roll, 0.0]]))
            pt, ap = T[:3, 3], T[:3, 0]
            e = np.concatenate([pt - p, w * (ap - a)])
            if np.linalg.norm(e[:3]) < 3e-4 and np.linalg.norm(e[3:]) < 1e-3 * w:
                return q
            J = np.empty((6, 4))
            for k in range(4):
                o, z = frames[k]
                J[:3, k] = np.cross(z, pt - o)
                J[3:, k] = w * np.cross(z, ap)
            dq = -np.linalg.solve(J.T @ J + lam * np.eye(4), J.T @ e)
            step = np.max(np.abs(dq))
            if step > 0.3:
                dq *= 0.3 / step
            q = np.clip(q + dq, self.lo[:4], self.hi[:4])
        T = self.tool(np.concatenate([q, [roll, 0.0]]))
        e = np.concatenate([T[:3, 3] - p, w * (T[:3, 0] - a)])
        return q if np.linalg.norm(e[:3]) < 2e-3 and np.linalg.norm(e[3:]) < 0.035 * w else None

    def _roll_for(self, q6, yaw: float, flip_ok: bool = True):
        """The wrist roll that turns the opening direction (tool z) to heading yaw, or to yaw + pi (the fixed finger on
        the other side: the caller places the tool point for whichever comes back, see opening()); the one nearest the
        seed's roll within the joint's range. With the fingers vertical the roll turns the opening about the vertical
        one for one (the sign from the chain)."""
        def heading(r):
            q = q6.copy()
            q[4] = r
            z = self.tool(q)[:3, 2]
            return math.atan2(z[1], z[0])
        h0 = heading(0.0)
        s = 1.0 if math.remainder(heading(0.1) - h0, 2 * math.pi) > 0 else -1.0
        best = None
        for target in ((yaw, yaw + math.pi) if flip_ok else (yaw,)):
            base = s * math.remainder(target - h0, 2 * math.pi)
            for r in (base, base - 2 * math.pi, base + 2 * math.pi):
                if self.lo[4] <= r <= self.hi[4] and (best is None or abs(r - q6[4]) < abs(best - q6[4])):
                    best = r
        if best is None:
            return None
        err = math.remainder(heading(best) - yaw, math.pi if flip_ok else 2 * math.pi)
        return best if abs(err) < 1e-3 else None

    def opening(self, q) -> np.ndarray:
        """The opening direction (tool z: from the fixed finger toward the moving one), base_link."""
        return self.tool(q)[:3, 2]

    # -- collisions -----------------------------------------------------------------------------------------------
    def capsules(self, q, fk=None):
        """[(a, b, r)] in base_link."""
        fk = fk or self.fk(q)
        out = []
        for body, a, b, r in self._caps:
            T = fk["arm_jaw"] if body == "arm_jaw" else fk[body]
            out.append((T[:3, :3] @ a + T[:3, 3], T[:3, :3] @ b + T[:3, 3], r))
        return out

    def points(self, q, n: int = 5):
        P, R = [], []
        s = np.linspace(0.0, 1.0, n)
        for a, b, r in self.capsules(q):
            P.append(a + np.outer(s, b - a))
            R.append(np.full(n, r))
        return np.vstack(P), np.concatenate(R)

    def clear(self, q, allow_bin: bool = False, floor_margin: float | None = None) -> tuple[bool, str]:
        """(ok, what it hits). allow_bin: the bin's box may be entered from above (a drop)."""
        P, r = self.points(q)
        x, y, z = P.T
        rho = np.hypot(x, y)
        for k in self.cfg.keep_out:
            kind = k["kind"]
            if kind == "floor":
                if np.any(z - r < (k["z"] if floor_margin is None else floor_margin)):
                    return False, "floor"
            elif kind == "ring":
                if np.any((rho > k["r0"] - r) & (rho < k["r1"] + r) & (z - r < k["z1"])):
                    return False, "bumper"
            elif kind == "cylinder":
                sel = (rho < k["r"]) & (z - r < k["z1"])
                if "x_max" in k:
                    sel &= x < k["x_max"]
                if "x_min" in k:
                    sel &= x >= k["x_min"]
                if np.any(sel):
                    return False, "robot"
            elif kind == "box":
                if allow_bin and "bin" in k["note"]:
                    continue
                lo, hi = np.array(k["min"]), np.array(k["max"])
                if np.any(np.all((P > lo - r[:, None]) & (P < hi + r[:, None]), axis=1)):
                    return False, k["note"].split(":")[0]
        return True, ""

    def path_clear(self, q0, q1, step: float = 0.05, **kw) -> tuple[bool, str]:
        """Joint-space straight line q0 -> q1 sampled every step rad of the largest joint's move."""
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        n = max(2, int(math.ceil(np.max(np.abs(q1[:5] - q0[:5])) / step)) + 1)
        for s in np.linspace(0.0, 1.0, n):
            ok, why = self.clear(q0 + s * (q1 - q0), **kw)
            if not ok:
                return False, why
        return True, ""

    def tool_out(self, q) -> bool:
        """The tool is outside the bumper or high: the base runs with the "arm out" caps (design.md 11)."""
        t = self.tool(q)[:3, 3]
        ao = self.cfg.arm_out
        return bool(math.hypot(t[0], t[1]) > ao["out_radius"] or t[2] > ao["out_z"])


def gap_at(cfg: ArmConfig, q: float) -> float:
    """Gap between the finger pads' inner faces near the tips (m) at gripper angle q: the sim's arm_so101.gap_at."""
    (cx, cy, _), (hx, _, _) = cfg.pads["moving"]
    fixed = cfg.pads["fixed"]
    x, y = cx - hx, cy
    xr = x * math.cos(q) - y * math.sin(q)
    return cfg.jaw["pos"][0] + xr - (fixed[0][0] + fixed[1][0])


def q_for_gap(cfg: ArmConfig, gap: float) -> float:
    lo, hi = cfg.pads["gripper_closed"], 1.6
    if gap <= gap_at(cfg, lo):
        return lo
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if gap_at(cfg, mid) < gap else (lo, mid)
    return 0.5 * (lo + hi)
