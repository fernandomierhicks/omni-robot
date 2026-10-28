"""A small factor graph for landmark SLAM (design.md 10): robot keyframes and AprilTags estimated together.

Variables   X_i = (x, y, yaw) of base_link in map at keyframe i (the base is planar; its rocking is noise here)
            L_j = (rotation vector, position) of tag j in map: unknown landmarks, found by the graph
Factors     prior      X_i ~ z: the anchor (the first keyframe, when nothing else fixes the map frame: the gauge) or
                       an absolute fix (the lidar localizer's pose in the map, with its covariance)
            between    X_i -> X_k ~ the odometry's motion between them (the EKF's odom frame), with a covariance
                       that grows with distance and turn
            tag        the four corners of tag j seen from X_i through the camera (T_base_cam, calibrated) ~ their
                       pixels: reprojection residuals, robust (Huber) so a bad detection can't drag the map
            tag prior  L_j ~ a saved tag map's pose and covariance (localization against a known map)
Solver      Levenberg-Marquardt on the sparse normal equations (scipy's sparse LU), analytic Jacobians for the
            priors and odometry, a vectorized finite difference for the corners; marginal covariances of a pose or a
            tag from the factorized information.

This is what GTSAM's Pose2 / Pose3 / GenericProjectionFactor graph would be; written here in numpy / scipy so the
sim's tests and the robot run the same code (GTSAM is `ros-jazzy-gtsam` when a bigger, incremental backend (iSAM2)
is needed: the factors map one to one).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix, diags
from scipy.sparse.linalg import splu

from omni_slam.tags import Pinhole, log_rot, rodrigues, tag_corners


def wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def rodrigues_n(r):
    """(n, 3) rotation vectors -> (n, 3, 3)."""
    th = np.linalg.norm(r, axis=1)
    k = r / np.maximum(th, 1e-12)[:, None]
    K = np.zeros((len(r), 3, 3))
    K[:, 0, 1], K[:, 0, 2], K[:, 1, 0] = -k[:, 2], k[:, 1], k[:, 2]
    K[:, 1, 2], K[:, 2, 0], K[:, 2, 1] = -k[:, 0], -k[:, 1], k[:, 0]
    s, c = np.sin(th)[:, None, None], (1 - np.cos(th))[:, None, None]
    return np.eye(3)[None] + s * K + c * (K @ K)


def se2_between(a, b):
    """b in a's frame: (dx, dy, dyaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, float(wrap(b[2] - a[2]))])


def se2_compose(a, d):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * d[0] - s * d[1], a[1] + s * d[0] + c * d[1], float(wrap(a[2] + d[2]))])


@dataclass
class Stats:
    iters: int = 0
    cost0: float = 0.0
    cost: float = 0.0
    tag_rms_px: float = math.nan
    outliers: int = 0


class TagGraph:
    def __init__(self, cam: Pinhole, T_base_cam: np.ndarray, sigma_px: float = 0.7, huber: float = 3.0,
                 cam_sigma_deg: float | None = None, cam_bound_deg: float = 1.0):
        self.cam, self.Tbc = cam, np.asarray(T_base_cam, float)
        self.sigma_px, self.huber = sigma_px, huber
        # online refinement of the camera's mount rotation (design.md 10): a 3-vector C (rotation vector) on top of the
        # calibrated T_base_cam, held to it by a prior of cam_sigma_deg and never past cam_bound_deg. Off (None) unless
        # something fixes the map independently of the camera (the lidar's poses): otherwise it isn't observable
        self.refine_cam = cam_sigma_deg is not None
        self.cam_sigma = math.radians(cam_sigma_deg or 0.5)
        self.cam_bound = math.radians(cam_bound_deg)
        self.C = np.zeros(3)
        self.cam_at_bound = False
        self.X = np.zeros((0, 3))
        self.t_pose: list = []
        self.L = np.zeros((0, 6))
        self.lm_index: dict = {}                   # tag id -> j
        self.lm_size: list = []
        self.priors: list = []                     # (i, z, sqrt_info)
        self.betweens: list = []                   # (i, k, z, sqrt_info)
        self.lm_priors: list = []                  # (j, z, sqrt_info)
        self.obs_i, self.obs_j, self.obs_uv, self.obs_t = [], [], [], []
        self.obs_w = np.zeros(0)
        self.stats = Stats()
        self._lu = None

    # -- building ------------------------------------------------------------------------------------------------
    def add_pose(self, t: float, guess, roll_pitch=(0.0, 0.0)) -> int:
        """roll_pitch: the base's attitude then (the IMU's, rad): known, not estimated; the camera sees through it
        (the sprung base pitches ~1.5 deg when it accelerates: ~15 px at 3 m)."""
        self.X = np.vstack([self.X, np.asarray(guess, float)[None]])
        self.t_pose.append(t)
        r, p = roll_pitch
        cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
        A = np.array([[cp, sp * sr, sp * cr], [0.0, cr, -sr], [-sp, cp * sr, cp * cr]])     # Ry(pitch) Rx(roll)
        self.att = np.concatenate([getattr(self, "att", np.zeros((0, 3, 3))), A[None]])
        self._lu = None
        return len(self.X) - 1

    def add_prior(self, i: int, z, cov):
        self.priors.append((i, np.asarray(z, float), np.linalg.cholesky(np.linalg.inv(np.asarray(cov, float))).T))

    def add_between(self, i: int, k: int, z, cov):
        self.betweens.append((i, k, np.asarray(z, float),
                              np.linalg.cholesky(np.linalg.inv(np.asarray(cov, float))).T))

    def has_tag(self, tag_id: int) -> bool:
        return tag_id in self.lm_index

    def lm_index_inv(self, j: int) -> int:
        if not hasattr(self, "_inv") or len(self._inv) != len(self.lm_index):
            self._inv = {v: k for k, v in self.lm_index.items()}
        return self._inv[j]

    def add_tag(self, tag_id: int, size: float, R_map_tag, t_map_tag, prior_cov=None) -> int:
        j = len(self.L)
        self.L = np.vstack([self.L, np.concatenate([log_rot(R_map_tag), t_map_tag])[None]])
        self.lm_index[tag_id] = j
        self.lm_size.append(size)
        if prior_cov is not None:
            self.lm_priors.append((j, self.L[j].copy(), np.linalg.cholesky(np.linalg.inv(prior_cov)).T))
        return j

    def add_obs(self, i: int, tag_id: int, uv, t: float):
        self.obs_i.append(i)
        self.obs_j.append(self.lm_index[tag_id])
        self.obs_uv.append(np.asarray(uv, float))
        self.obs_t.append(t)

    def base_cam(self, C=None) -> np.ndarray:
        """T_base_cam with the refined rotation."""
        M = self.Tbc.copy()
        M[:3, :3] = rodrigues(self.C if C is None else C) @ self.Tbc[:3, :3]
        return M

    def cam_pose(self, x, att=None):
        """T_map_cam (4x4) of the camera at robot pose x (and attitude att, 3x3: Ry(pitch) Rx(roll))."""
        c, s = math.cos(x[2]), math.sin(x[2])
        Tmb = np.array([[c, -s, 0, x[0]], [s, c, 0, x[1]], [0, 0, 1, 0], [0, 0, 0, 1.0]])
        if att is not None:
            Tmb[:3, :3] = Tmb[:3, :3] @ att
        return Tmb @ self.base_cam()

    def tag_in_map(self, x, R_cam_tag, t_cam_tag, att=None):
        Tmc = self.cam_pose(x, att)
        return Tmc[:3, :3] @ R_cam_tag, Tmc[:3, :3] @ t_cam_tag + Tmc[:3, 3]

    # -- residuals -------------------------------------------------------------------------------------------------
    def _tag_res(self, X, L, I, J, UV, C=None):
        """(n, 8) whitened corner residuals."""
        n = len(I)
        x = X[I]
        c, s = np.cos(x[:, 2]), np.sin(x[:, 2])
        Rmb = np.zeros((n, 3, 3))
        Rmb[:, 0, 0], Rmb[:, 0, 1], Rmb[:, 1, 0], Rmb[:, 1, 1], Rmb[:, 2, 2] = c, -s, s, c, 1.0
        Rmb = Rmb @ self.att[I]                                                # yaw, then the IMU's roll / pitch
        Rbc, tbc = rodrigues(self.C if C is None else C) @ self.Tbc[:3, :3], self.Tbc[:3, 3]
        Rmc = Rmb @ Rbc
        tmc = Rmb @ tbc + np.column_stack([x[:, 0], x[:, 1], np.zeros(n)])
        Rml = rodrigues_n(L[J, :3])
        corners = np.stack([tag_corners(self.lm_size[j]) for j in J])           # (n, 4, 3)
        P = np.einsum("nij,nkj->nki", Rml, corners) + L[J, 3:][:, None, :]
        Pc = np.einsum("nji,nkj->nki", Rmc, P - tmc[:, None, :])              # R^T (P - t)
        uv = self.cam.project(Pc)
        r = (uv - UV) / self.sigma_px
        return np.nan_to_num(r.reshape(n, 8), nan=1e3)

    def _assemble(self, X, L, C=None):
        C = self.C if C is None else C
        nX, nL = 3 * len(X), 6 * len(L)
        nC = 3 if self.refine_cam else 0
        rows, cols, vals, res = [], [], [], []
        r0 = 0

        def put(rr, cc, M):
            M = np.atleast_2d(M)
            for a in range(M.shape[0]):
                for b in range(M.shape[1]):
                    if M[a, b] != 0.0:
                        rows.append(rr + a)
                        cols.append(cc + b)
                        vals.append(M[a, b])

        for i, z, W in self.priors:
            e = X[i] - z
            e[2] = wrap(e[2])
            res.append(W @ e)
            put(r0, 3 * i, W)
            r0 += 3
        for i, k, z, W in self.betweens:
            a, b = X[i], X[k]
            cth, sth = math.cos(a[2]), math.sin(a[2])
            dp = b[:2] - a[:2]
            RT = np.array([[cth, sth], [-sth, cth]])
            pred = np.array([*(RT @ dp), wrap(b[2] - a[2])])
            e = pred - z
            e[2] = wrap(e[2])
            Ji = np.zeros((3, 3))
            Ji[:2, :2] = -RT
            Ji[:2, 2] = np.array([[-sth, cth], [-cth, -sth]]) @ dp
            Ji[2, 2] = -1.0
            Jk = np.zeros((3, 3))
            Jk[:2, :2] = RT
            Jk[2, 2] = 1.0
            res.append(W @ e)
            put(r0, 3 * i, W @ Ji)
            put(r0, 3 * k, W @ Jk)
            r0 += 3
        for j, z, W in self.lm_priors:
            e = L[j] - z
            res.append(W @ e)
            put(r0, nX + 6 * j, W)
            r0 += 6
        if nC:
            res.append(C / self.cam_sigma)
            put(r0, nX + nL, np.eye(3) / self.cam_sigma)
            r0 += 3
        n = len(self.obs_i)
        if n:
            I, J = np.array(self.obs_i), np.array(self.obs_j)
            r = self._tag_res(X, L, I, J, self.UV, C)
            nrm = np.linalg.norm(r, axis=1)
            w = np.where(nrm <= self.huber, 1.0, self.huber / np.maximum(nrm, 1e-9))     # Huber on each view
            self.obs_w = w
            sw = np.sqrt(w)[:, None]
            res.append((r * sw).ravel())
            base_rows = r0 + 8 * np.arange(n)
            h = 1e-6
            for d in range(3):
                Xp = X.copy()
                Xp[:, d] += h
                D = (self._tag_res(Xp, L, I, J, self.UV, C) - r) / h * sw
                for q in range(8):
                    rows.extend(base_rows + q)
                    cols.extend(3 * I + d)
                    vals.extend(D[:, q])
            for d in range(6):
                Lp = L.copy()
                Lp[:, d] += h
                D = (self._tag_res(X, Lp, I, J, self.UV, C) - r) / h * sw
                for q in range(8):
                    rows.extend(base_rows + q)
                    cols.extend(nX + 6 * J + d)
                    vals.extend(D[:, q])
            for d in range(nC):
                Cp = C.copy()
                Cp[d] += h
                D = (self._tag_res(X, L, I, J, self.UV, Cp) - r) / h * sw
                for q in range(8):
                    rows.extend(base_rows + q)
                    cols.extend(np.full(n, nX + nL + d))
                    vals.extend(D[:, q])
            r0 += 8 * n
        res = np.concatenate(res) if res else np.zeros(0)
        Jm = coo_matrix((vals, (rows, cols)), shape=(r0, nX + nL + nC)).tocsr()
        return res, Jm

    # -- solving ---------------------------------------------------------------------------------------------------
    def solve(self, iters: int = 10, tol: float = 1e-6) -> Stats:
        self.UV = np.array(self.obs_uv).reshape(-1, 4, 2) if self.obs_uv else np.zeros((0, 4, 2))
        X, L, C = self.X.copy(), self.L.copy(), self.C.copy()
        r, J = self._assemble(X, L, C)
        cost = float(r @ r)
        st = Stats(cost0=cost)
        lam = 1e-4
        nX, nL = 3 * len(X), 6 * len(L)
        for it in range(iters):
            H = (J.T @ J).tocsc()
            g = J.T @ r
            dg = H.diagonal()
            improved = False
            for _ in range(8):
                A = (H + diags(lam * np.maximum(dg, 1e-9) + 1e-12)).tocsc()
                try:
                    dx = -splu(A).solve(g)
                except RuntimeError:
                    lam *= 10
                    continue
                Xn = X + dx[:nX].reshape(-1, 3)
                Xn[:, 2] = wrap(Xn[:, 2])
                Ln = L + dx[nX:nX + nL].reshape(-1, 6)
                Cn = C + dx[nX + nL:] if self.refine_cam else C
                nrm = float(np.linalg.norm(Cn))
                if nrm > self.cam_bound:                    # the bound: never past it
                    Cn = Cn * self.cam_bound / nrm
                rn, Jn = self._assemble(Xn, Ln, Cn)
                cn = float(rn @ rn)
                if cn < cost:
                    X, L, C, r, J = Xn, Ln, Cn, rn, Jn
                    done = cost - cn < tol * max(cost, 1e-12)
                    cost = cn
                    lam = max(lam / 5, 1e-9)
                    improved = True
                    break
                lam *= 10
            st.iters = it + 1
            if not improved or done:
                break
        self.X, self.L, self.C = X, L, C
        self.cam_at_bound = bool(np.linalg.norm(C) >= self.cam_bound * 0.999)
        st.cost = cost
        if self.obs_i:
            rr = self._tag_res(X, L, np.array(self.obs_i), np.array(self.obs_j), self.UV) * self.sigma_px
            st.tag_rms_px = float(np.sqrt(np.mean(rr ** 2)))
            st.outliers = int(np.sum(np.linalg.norm(rr / self.sigma_px, axis=1) > self.huber))
        self.stats = st
        self._J = J
        self._lu = None
        return st

    def _factor(self):
        if self._lu is None:
            # a weak floor on every variable (1e-6 in whitened units: a sigma of 1000): a tag whose only views were
            # dropped has no factor left, and its block would make the information singular (NaN marginals)
            H = (self._J.T @ self._J).tocsc()
            self._lu = splu((H + diags(np.full(H.shape[0], 1e-6))).tocsc())
        return self._lu

    def marginal(self, cols) -> np.ndarray:
        lu = self._factor()
        n = lu.shape[0]
        E = np.zeros((n, len(cols)))
        E[cols, np.arange(len(cols))] = 1.0
        M = lu.solve(E)[cols]
        return np.where(np.isfinite(M), M, 1e6)

    def pose_cov(self, i: int) -> np.ndarray:
        return self.marginal([3 * i, 3 * i + 1, 3 * i + 2])

    def tag_cov(self, tag_id: int) -> np.ndarray:
        j = self.lm_index[tag_id]
        b = 3 * len(self.X) + 6 * j
        return self.marginal(list(range(b, b + 6)))

    def tag_pose(self, tag_id: int):
        Lj = self.L[self.lm_index[tag_id]]
        return rodrigues(Lj[:3]), Lj[3:].copy()

    def cam_correction(self) -> dict:
        """The refined camera rotation: degrees, its 1-sigma, and whether the data could see it (sigma well under
        the prior's) or it sits at its bound."""
        if not self.refine_cam:
            return dict(enabled=False)
        b = 3 * len(self.X) + 6 * len(self.L)
        try:
            s = np.sqrt(np.maximum(np.diag(self.marginal([b, b + 1, b + 2])), 0.0))
        except Exception:
            s = np.full(3, math.nan)
        return dict(enabled=True, deg=[round(math.degrees(c), 4) for c in self.C],
                    sigma_deg=[round(math.degrees(c), 4) for c in s],
                    observable=bool(np.all(s < 0.5 * self.cam_sigma)), at_bound=self.cam_at_bound)

    def obs_residual_px(self, k: int) -> float:
        """One observation's corner RMS (px) at the current estimate."""
        r = self._tag_res(self.X, self.L, np.array([self.obs_i[k]]), np.array([self.obs_j[k]]), self.obs_uv[k][None]) * self.sigma_px
        return float(np.sqrt(np.mean(r ** 2)))
