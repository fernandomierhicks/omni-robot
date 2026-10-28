"""Camera intrinsics and stereo extrinsics from views of a planar target (design.md 10).

calibrate_camera(views)   Zhang's closed form from the views' homographies, then every intrinsic, the distortion
                          and every view's pose refined together (sparse least squares, robust loss). Reports the
                          reprojection error (all, per view), the parameters' 1-sigma from the fit's information, and
                          how much of the image the corners covered (corners near the edges pin the distortion).
calibrate_stereo(...)     the right camera's pose in the left's from views both saw at the same trigger, intrinsics
                          held: reprojection in both cameras, the baseline, and the epipolar error (how far a right
                          corner sits off its left corner's epipolar line, px): the number the stereo depends on.

A view is (obj (n, 3) target points, uv (n, 2) their pixels): detect.py makes them from images (OpenCV), the tests
from the sim's geometry.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from omni_calib.geometry import Camera, homography, log_rot, planar_pose, rodrigues


def _v(H, i, j):
    h = H.T
    return np.array([h[i, 0] * h[j, 0], h[i, 0] * h[j, 1] + h[i, 1] * h[j, 0], h[i, 1] * h[j, 1],
                     h[i, 2] * h[j, 0] + h[i, 0] * h[j, 2], h[i, 2] * h[j, 1] + h[i, 1] * h[j, 2], h[i, 2] * h[j, 2]])


def zhang_init(views, width: int, height: int) -> Camera:
    Hs = [homography(o[:, :2], uv) for o, uv in views]
    V = []
    for H in Hs:
        V += [_v(H, 0, 1), _v(H, 0, 0) - _v(H, 1, 1)]
    V.append([0, 1, 0, 0, 0, 0])                              # zero skew
    _, _, Vt = np.linalg.svd(np.array(V))
    B11, B12, B22, B13, B23, B33 = Vt[-1]
    try:
        v0 = (B12 * B13 - B11 * B23) / (B11 * B22 - B12 ** 2)
        lam = B33 - (B13 ** 2 + v0 * (B12 * B13 - B11 * B23)) / B11
        a = math.sqrt(lam / B11)
        b = math.sqrt(lam * B11 / (B11 * B22 - B12 ** 2))
        u0 = -B13 * a * a / lam
        if not (0 < u0 < width and 0 < v0 < height and a > 0 and b > 0):
            raise ValueError
        return Camera(a, b, u0, v0, width=width, height=height)
    except (ValueError, ZeroDivisionError):
        return Camera(width, width, width / 2, height / 2, width=width, height=height)   # a guess the fit can leave


def _reproj(cam: Camera, R, t, obj):
    return cam.project(obj @ R.T + t)


def _sparse_fd(fun, S, h: float = 1e-7):
    """A dense Jacobian by forward differences, columns that share no residual perturbed together (greedy colouring
    of the sparsity pattern): one function call per colour instead of one per column."""
    S = S.tocsc().astype(bool)
    rows = [set(S[:, j].nonzero()[0]) for j in range(S.shape[1])]
    colours, used = [], []
    for j, r in enumerate(rows):
        for c, u in zip(colours, used):
            if not (u & r):
                c.append(j)
                u |= r
                break
        else:
            colours.append([j])
            used.append(set(r))

    def jac(x):
        f0 = fun(x)
        J = np.zeros((len(f0), len(x)))
        for c in colours:
            dx = np.zeros_like(x)
            steps = h * np.maximum(1.0, np.abs(x[c]))
            dx[c] = steps
            df = fun(x + dx) - f0
            for j, s in zip(c, steps):
                idx = list(rows[j])
                J[idx, j] = df[idx] / s
        return J
    return jac


def calibrate_camera(views, width: int = 1280, height: int = 800, fix_k3: bool = True, init: Camera | None = None,
                     loss: str = "huber", f_scale: float = 1.0) -> dict:
    cam0 = init or zhang_init(views, width, height)
    poses = [planar_pose(cam0, o, uv) for o, uv in views]
    n_in = 8 if fix_k3 else 9
    x0 = np.concatenate([cam0.params[:n_in]] + [np.concatenate([log_rot(R), t]) for R, t in poses])
    n_pts = [len(o) for o, _ in views]
    m = 2 * sum(n_pts)

    def unpack(x):
        p = np.concatenate([x[:n_in], [0.0] * (9 - n_in)])
        return Camera.from_params(p, width, height), x[n_in:].reshape(-1, 6)

    def fun(x):
        cam, P = unpack(x)
        res = []
        for (o, uv), pv in zip(views, P):
            res.append((_reproj(cam, rodrigues(pv[:3]), pv[3:], o) - uv).ravel())
        return np.concatenate(res)

    S = lil_matrix((m, len(x0)), dtype=int)
    r = 0
    for k, n in enumerate(n_pts):
        S[r:r + 2 * n, :n_in] = 1
        S[r:r + 2 * n, n_in + 6 * k:n_in + 6 * k + 6] = 1
        r += 2 * n
    # an exact trust-region solve on the dense Jacobian: a few hundred columns at most, and lsmr on this problem
    # (poorly scaled intrinsics against poses) crawls; the sparsity still makes the finite differences cheap
    sol = least_squares(fun, x0, jac=_sparse_fd(fun, S), method="trf", tr_solver="exact", x_scale="jac", loss=loss,
                        f_scale=f_scale)
    cam, P = unpack(sol.x)
    res = sol.fun.reshape(-1, 2)
    per_view, r = [], 0
    for n in n_pts:
        per_view.append(float(np.sqrt(np.mean(np.sum(res[r:r + n] ** 2, axis=1)))))
        r += n
    rms = float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))
    try:     # the intrinsics' sigma with the poses marginalized out (the full information's inverse, its block)
        Jf = sol.jac.toarray() if hasattr(sol.jac, "toarray") else sol.jac
        cov = np.linalg.pinv(Jf.T @ Jf)[:n_in, :n_in] * (rms ** 2 / 2)
        sigma = np.sqrt(np.maximum(np.diag(cov), 0.0))
    except np.linalg.LinAlgError:
        sigma = np.full(n_in, np.nan)
    return dict(camera=cam, poses=[(rodrigues(p[:3]), p[3:]) for p in P], rms_px=rms, per_view_px=per_view,
                sigma=dict(zip(("fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2", "k3")[:n_in], map(float, sigma))),
                coverage=coverage([uv for _, uv in views], width, height), n_views=len(views))


def coverage(uvs, width: int, height: int, nx: int = 8, ny: int = 5) -> float:
    """The fraction of an nx x ny grid over the image that has corners in it."""
    hit = np.zeros((ny, nx), bool)
    for uv in uvs:
        i = np.clip((uv[:, 0] / width * nx).astype(int), 0, nx - 1)
        j = np.clip((uv[:, 1] / height * ny).astype(int), 0, ny - 1)
        hit[j, i] = True
    return float(hit.mean())


def calibrate_stereo(views_l, views_r, cam_l: Camera, cam_r: Camera, loss: str = "huber") -> dict:
    """views_l[k], views_r[k]: the same target pose (one trigger), each (obj, uv); the same obj points in both (detect
    with ids). Returns R, t: a point in the left optical frame -> the right's (x_r = R x_l + t)."""
    poses_l = [planar_pose(cam_l, o, uv) for o, uv in views_l]
    poses_r = [planar_pose(cam_r, o, uv) for o, uv in views_r]
    rels = [(Rr @ Rl.T, tr - Rr @ Rl.T @ tl) for (Rl, tl), (Rr, tr) in zip(poses_l, poses_r)]
    from omni_calib.geometry import mean_rotation
    R0 = mean_rotation([r[0] for r in rels])
    t0 = np.median([r[1] for r in rels], axis=0)
    x0 = np.concatenate([log_rot(R0), t0] + [np.concatenate([log_rot(R), t]) for R, t in poses_l])

    def fun(x):
        Rlr, tlr = rodrigues(x[:3]), x[3:6]
        P = x[6:].reshape(-1, 6)
        res = []
        for (ol, uvl), (orr, uvr), p in zip(views_l, views_r, P):
            R, t = rodrigues(p[:3]), p[3:]
            res.append((_reproj(cam_l, R, t, ol) - uvl).ravel())
            res.append((_reproj(cam_r, Rlr @ R, Rlr @ t + tlr, orr) - uvr).ravel())
        return np.concatenate(res)

    m = sum(2 * (len(a[0]) + len(b[0])) for a, b in zip(views_l, views_r))
    S = lil_matrix((m, len(x0)), dtype=int)
    r = 0
    for k, (a, b) in enumerate(zip(views_l, views_r)):
        S[r:r + 2 * len(a[0]), 6 + 6 * k:12 + 6 * k] = 1
        r += 2 * len(a[0])
        S[r:r + 2 * len(b[0]), :6] = 1
        S[r:r + 2 * len(b[0]), 6 + 6 * k:12 + 6 * k] = 1
        r += 2 * len(b[0])
    sol = least_squares(fun, x0, jac=_sparse_fd(fun, S), method="trf", tr_solver="exact", x_scale="jac", loss=loss)
    R, t = rodrigues(sol.x[:3]), sol.x[3:6]
    res = sol.fun.reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))
    Jf = np.asarray(sol.jac)
    cov = np.linalg.pinv(Jf.T @ Jf)[:6, :6] * (rms ** 2 / 2)
    epi = epipolar_px(views_l, views_r, cam_l, cam_r, R, t)
    return dict(R=R, t=t, baseline_m=float(np.linalg.norm(t)), rms_px=rms, epipolar_px=epi,
                sigma_rot_deg=float(math.degrees(math.sqrt(max(np.trace(cov[:3, :3]), 0.0)))),
                sigma_t_mm=float(1000 * math.sqrt(max(np.trace(cov[3:, 3:]), 0.0))))


def epipolar_px(views_l, views_r, cam_l: Camera, cam_r: Camera, R, t) -> float:
    """RMS distance (px, at the right camera's focal length) of each right corner from its left corner's epipolar
    line; matches by the target point (the same obj row in both views)."""
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
    E = tx @ R
    d = []
    for (ol, uvl), (orr, uvr) in zip(views_l, views_r):
        key_r = {tuple(np.round(p, 6)): i for i, p in enumerate(orr)}
        idx = [(i, key_r[tuple(np.round(p, 6))]) for i, p in enumerate(ol) if tuple(np.round(p, 6)) in key_r]
        if not idx:
            continue
        xl = np.column_stack([cam_l.undistort_norm(uvl[[i for i, _ in idx]]), np.ones(len(idx))])
        xr = np.column_stack([cam_r.undistort_norm(uvr[[j for _, j in idx]]), np.ones(len(idx))])
        lines = xl @ E.T
        dist = np.abs(np.sum(xr * lines, axis=1)) / np.hypot(lines[:, 0], lines[:, 1])
        d.append(dist * cam_r.fy)
    d = np.concatenate(d) if d else np.array([np.nan])
    return float(np.sqrt(np.mean(d ** 2)))
