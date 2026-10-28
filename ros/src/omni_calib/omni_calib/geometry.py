"""Rotations, poses and the camera model (OpenCV's: pinhole + Brown-Conrady k1 k2 p1 p2 k3), so results drop
straight into OpenCV / ROS camera_info."""

from __future__ import annotations

import math

import numpy as np


def rodrigues(r) -> np.ndarray:
    r = np.asarray(r, float)
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.eye(3) + skew(r)
    k = r / th
    K = skew(k)
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def log_rot(R) -> np.ndarray:
    R = np.asarray(R, float)
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / 2
    if th > math.pi - 1e-6:                          # 180 deg: the axis from R + I
        A = (R + np.eye(3)) / 2
        k = A[:, int(np.argmax(np.diag(A)))]
        return th * k / np.linalg.norm(k)
    return th / (2 * math.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def skew(v) -> np.ndarray:
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], float)


def rot_deg(Ra, Rb) -> float:
    return math.degrees(float(np.linalg.norm(log_rot(np.asarray(Ra).T @ np.asarray(Rb)))))


def T(R, t) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3], M[:3, 3] = R, np.asarray(t, float).reshape(3)
    return M


def inv(M) -> np.ndarray:
    R, t = M[:3, :3], M[:3, 3]
    return T(R.T, -R.T @ t)


def mean_rotation(Rs) -> np.ndarray:
    """The chordal L2 mean of rotations (SVD of their sum)."""
    U, _, Vt = np.linalg.svd(np.sum(Rs, axis=0))
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    return U @ D @ Vt


class Camera:
    """K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dist = (k1, k2, p1, p2, k3). Points in the camera's optical frame
    (REP-103: z forward, x right, y down) -> pixels."""

    def __init__(self, fx, fy, cx, cy, dist=(0, 0, 0, 0, 0), width=1280, height=800):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self.dist = np.array(dist, float)
        self.width, self.height = width, height

    @classmethod
    def from_fovy(cls, fovy_deg: float, width: int = 1280, height: int = 800, dist=(0, 0, 0, 0, 0)):
        f = height / 2 / math.tan(math.radians(fovy_deg) / 2)
        return cls(f, f, width / 2, height / 2, dist, width, height)

    @property
    def params(self) -> np.ndarray:
        return np.array([self.fx, self.fy, self.cx, self.cy, *self.dist])

    @classmethod
    def from_params(cls, p, width=1280, height=800):
        return cls(*p[:4], p[4:9], width, height)

    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    def project(self, Pc) -> np.ndarray:
        """(n, 3) optical-frame points -> (n, 2) pixels (points behind the camera give nan)."""
        Pc = np.atleast_2d(Pc)
        z = Pc[:, 2]
        x, y = Pc[:, 0] / np.where(z > 1e-9, z, np.nan), Pc[:, 1] / np.where(z > 1e-9, z, np.nan)
        return self.distort_px(x, y)

    def distort_px(self, x, y) -> np.ndarray:
        k1, k2, p1, p2, k3 = self.dist
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        return np.column_stack([self.fx * xd + self.cx, self.fy * yd + self.cy])

    def undistort_norm(self, uv, iters: int = 8) -> np.ndarray:
        """pixels -> normalized (x, y) on z = 1 (fixed-point inversion of the distortion)."""
        uv = np.atleast_2d(uv)
        xd, yd = (uv[:, 0] - self.cx) / self.fx, (uv[:, 1] - self.cy) / self.fy
        x, y = xd.copy(), yd.copy()
        k1, k2, p1, p2, k3 = self.dist
        for _ in range(iters):
            r2 = x * x + y * y
            rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
            x = (xd - (2 * p1 * x * y + p2 * (r2 + 2 * x * x))) / rad
            y = (yd - (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)) / rad
        return np.column_stack([x, y])

    def as_dict(self) -> dict:
        return dict(fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy, dist=[float(d) for d in self.dist],
                    width=self.width, height=self.height)


def planar_pose(cam: Camera, obj: np.ndarray, uv: np.ndarray):
    """A planar target's pose in the camera (R, t), from its points (z = 0 on the target) and their pixels:
    the homography's decomposition (Zhang), as the initial guess for a refinement."""
    xy = cam.undistort_norm(uv)
    H = homography(obj[:, :2], xy)
    h1, h2, h3 = H[:, 0], H[:, 1], H[:, 2]
    s = 2.0 / (np.linalg.norm(h1) + np.linalg.norm(h2))
    if h3[2] * s < 0:
        s = -s                                      # the target in front of the camera
    r1, r2 = s * h1, s * h2
    R = np.column_stack([r1, r2, np.cross(r1, r2)])
    U, _, Vt = np.linalg.svd(R)
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    return R, s * h3


def homography(src, dst) -> np.ndarray:
    """DLT, normalized: dst ~ H src (both (n, 2))."""
    def norm(p):
        m, s = p.mean(axis=0), np.sqrt(2) / max(np.mean(np.linalg.norm(p - p.mean(axis=0), axis=1)), 1e-12)
        N = np.array([[s, 0, -s * m[0]], [0, s, -s * m[1]], [0, 0, 1]])
        return N, (np.column_stack([p, np.ones(len(p))]) @ N.T)
    Ns, s = norm(np.asarray(src, float))
    Nd, d = norm(np.asarray(dst, float))
    A = []
    for (x, y, _), (u, v, _) in zip(s, d):
        A.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        A.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, Vt = np.linalg.svd(np.array(A))
    H = Vt[-1].reshape(3, 3)
    H = np.linalg.inv(Nd) @ H @ Ns
    return H / H[2, 2]
