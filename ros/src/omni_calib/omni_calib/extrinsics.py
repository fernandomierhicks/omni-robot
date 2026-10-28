"""Where the sensors sit (design.md 10): LiDAR to camera, a sensor on base_link, the dock's tag to its contacts.

lidar_to_camera(views, prior)  A board seen by both: the camera gives its plane (from the board's pose), the lidar's
                               scan line crosses it, so every lidar point on the board satisfies n . (R p + t) = d.
                               Least squares over many board poses (tilted both ways, near and far: a planar lidar
                               only sees a line per pose), held near the drawing by a weak prior (sensible initial
                               value, bounded result). Reports the point-to-plane RMS, how much of the 6 DoF the views
                               pinned (the information's weakest direction) and the projected lidar points' fit.
hand_eye_se2(sensor, base)     A sensor's (x, y, yaw) on the base from the same motions seen by both: the sensor's own
                               (scan matching, tags) and the base's (wheel odometry over short legs). Planar robots:
                               T_B T_X = T_X T_S. Needs turns (they show x, y) and straight legs (they show yaw).
dock_tag(obs, T_base_contacts) The dock tag's pose relative to the charging contacts, from several dockings: each time the
                               contacts close, the camera's view of the tag ties the two; the spread says how repeatable.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import least_squares

from omni_calib.geometry import T, inv, log_rot, mean_rotation, rodrigues, rot_deg


# -- LiDAR <-> camera ---------------------------------------------------------------------------------------------

def board_plane(R_cb, t_cb):
    """The board's plane in the camera frame from its pose (board -> camera): (n, d) with n . X = d."""
    n = R_cb[:, 2]
    return n, float(n @ t_cb)


def lidar_to_camera(views, R0, t0, prior_sigma=(0.02, math.radians(3.0)), loss: str = "huber",
                    f_scale: float = 0.01) -> dict:
    """views: [(n (3,), d, pts (m, 3) lidar points on the board in the laser frame)]. R0, t0: the drawing's laser ->
    camera (optical) transform. Returns R, t (laser -> camera), rms_mm, weakest (the information's smallest eigen
    value's direction and its 1-sigma), and the change from the drawing."""
    s_t, s_r = prior_sigma
    x0 = np.zeros(6)                                            # a correction to the drawing: rotation, translation

    def fun(x):
        R = rodrigues(x[:3]) @ R0
        t = t0 + x[3:]
        res = [(P @ R.T + t) @ n - d for n, d, P in views]
        return np.concatenate(res + [x[:3] / s_r * 0.001, x[3:] / s_t * 0.001])   # the prior: weak, in metres

    sol = least_squares(fun, x0, loss=loss, f_scale=f_scale, x_scale="jac")
    R, t = rodrigues(sol.x[:3]) @ R0, t0 + sol.x[3:]
    m = sum(len(v[2]) for v in views)
    r = sol.fun[:m]
    J = sol.jac[:m]
    H = J.T @ J
    w, V = np.linalg.eigh(H)
    sig = math.sqrt(float(np.mean(r ** 2))) / math.sqrt(max(w[0], 1e-18))
    return dict(R=R, t=t, rms_mm=1000 * float(np.sqrt(np.mean(r ** 2))), n_points=m, n_views=len(views),
                weakest=dict(direction=[round(float(c), 3) for c in V[:, 0]], sigma=float(sig)),
                moved_mm=1000 * float(np.linalg.norm(t - t0)), turned_deg=rot_deg(R0, R))


def lidar_in_image(cam, R, t, pts, R_cb, t_cb, board_wh) -> float:
    """Validation: the fraction of a view's lidar points that, projected into the image, land on the board (inside its
    outline, which the board pose gives); with a wrong extrinsic the line slides off the board's edges."""
    Pc = pts @ R.T + t
    uv = cam.project(Pc)
    corners = np.array([(0, 0, 0), (board_wh[0], 0, 0), (board_wh[0], board_wh[1], 0), (0, board_wh[1], 0)], float)
    poly = cam.project(corners @ R_cb.T + t_cb)
    inside = np.array([_in_poly(p, poly) for p in uv if np.all(np.isfinite(p))])
    return float(inside.mean()) if len(inside) else 0.0


def _in_poly(p, poly) -> bool:
    x, y = p
    c = False
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1:
            c = not c
    return c


# -- a sensor on the base (planar hand-eye) --------------------------------------------------------------------------

def _se2(x, y, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def hand_eye_se2(sensor_moves, base_moves) -> dict:
    """Each move is (dx, dy, dyaw) over the same interval, in the mover's own frame at the start. Returns the sensor's
    (x, y, yaw) on the base, the residual (m), and the yaw-consistency of the pairs (their rotations must agree)."""
    rows, rhs = [], []
    for (sx, sy, sa), (bx, by, ba) in zip(sensor_moves, base_moves):
        Rb = np.array([[math.cos(ba), -math.sin(ba)], [math.sin(ba), math.cos(ba)]])
        A = np.zeros((2, 4))
        A[:, :2] = Rb - np.eye(2)
        A[:, 2:] = -np.array([[sx, -sy], [sy, sx]])
        rows.append(A)
        rhs.append(-np.array([bx, by]))
    A, b = np.vstack(rows), np.concatenate(rhs)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    yaw = math.atan2(x[3], x[2])
    A2 = A[:, :2]
    b2 = b - A[:, 2:] @ np.array([math.cos(yaw), math.sin(yaw)])
    txy, *_ = np.linalg.lstsq(A2, b2, rcond=None)
    res = A2 @ txy - b2
    sv = np.linalg.svd(A, compute_uv=False)
    dyaw = [abs(math.atan2(math.sin(s[2] - m[2]), math.cos(s[2] - m[2]))) for s, m in zip(sensor_moves, base_moves)]
    return dict(x=float(txy[0]), y=float(txy[1]), yaw=float(yaw), rms_m=float(np.sqrt(np.mean(res ** 2))),
                conditioning=float(sv[-1] / max(sv[0], 1e-12)), yaw_mismatch_deg=math.degrees(float(np.median(dyaw))))


# -- the dock ---------------------------------------------------------------------------------------------------------

def dock_tag(observations, T_base_contacts) -> dict:
    """observations: [(T_base_cam, T_cam_tag)] each taken with the contacts closed (charging). Returns the tag ->
    contacts transform (the dock's own geometry) and its spread over the dockings."""
    Ts = [inv(Tbc @ Tct) @ T_base_contacts for Tbc, Tct in observations]
    R = mean_rotation([M[:3, :3] for M in Ts])
    t = np.mean([M[:3, 3] for M in Ts], axis=0)
    spread_mm = [1000 * float(np.linalg.norm(M[:3, 3] - t)) for M in Ts]
    spread_deg = [rot_deg(R, M[:3, :3]) for M in Ts]
    return dict(T_tag_contacts=T(R, t), n=len(Ts), spread_mm=float(np.max(spread_mm)),
                spread_deg=float(np.max(spread_deg)), rms_mm=float(np.sqrt(np.mean(np.square(spread_mm)))))


def pose_error(Ta, Tb) -> tuple:
    """(mm, deg) between two poses."""
    return 1000 * float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3])), math.degrees(float(np.linalg.norm(
        log_rot(Ta[:3, :3].T @ Tb[:3, :3]))))
