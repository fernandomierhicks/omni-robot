"""v9 (design.md 27): where the robot thinks it is. A planar EKF in the style of ROS 2's robot_localization, and lidar
localization against a known map (AMCL / slam_toolbox's localization mode; full SLAM is step 2, in ROS).

  Map2D        a likelihood field: the static scene (walls, netting, posts, people standing) cut at the lidar's scan
               height, rasterized at 2 cm, and its distance transform
  ScanMatcher  de-skews a scan with the filter's pose history (the C1 takes 0.1 s per turn), then Gauss-Newton on the
               distance field with a robust loss; returns a pose and its covariance
  EKF          state x, y, yaw (world), vx, vy (body), gyro bias. Predicts with the IMU (yaw rate, horizontal
               accelerations), corrects with the wheel twist (trusted less when the over-determined wheels disagree:
               slip), the optical flow and the lidar pose. Gyro-less variants dead-reckon on the wheels.
  Localizer    runs several sensor combinations side by side on the same readings (shadow mode: the robot still drives
               on the sim's truth), so they can be compared on one run
"""

from __future__ import annotations

import math
from collections import deque

import mujoco
import numpy as np
from scipy import ndimage

from omni_control import wrap_angle


# -- the map ------------------------------------------------------------------------------------

class Map2D:
    def __init__(self, m, d, z: float, res: float = 0.02, margin: float = 0.5, exclude_root: int | None = None):
        """Rasterize every static geom (welded to the world) that the plane z cuts, in groups 0, 1, 3."""
        self.res = res
        static = m.body_weldid[m.geom_bodyid] == 0
        keep = []
        for g in range(m.ngeom):
            if not static[g] or m.geom_group[g] not in (0, 1, 3) or m.geom_type[g] in (mujoco.mjtGeom.mjGEOM_PLANE,
                                                                                       mujoco.mjtGeom.mjGEOM_MESH):
                continue
            if exclude_root is not None and m.body_rootid[m.geom_bodyid[g]] == exclude_root:
                continue
            keep.append(g)
        pts = np.array([d.geom_xpos[g][:2] for g in keep])
        rb = np.array([m.geom_rbound[g] for g in keep])
        lo = (pts - rb[:, None]).min(axis=0) - margin
        hi = (pts + rb[:, None]).max(axis=0) + margin
        self.origin = lo
        nx, ny = (np.ceil((hi - lo) / res)).astype(int)
        xs = lo[0] + (np.arange(nx) + 0.5) * res
        ys = lo[1] + (np.arange(ny) + 0.5) * res
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        P = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, z)])
        occ = np.zeros(X.size, dtype=bool)
        for g in keep:
            c, R, s = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
            near = np.abs(P[:, 0] - c[0]) <= m.geom_rbound[g] + res
            near &= np.abs(P[:, 1] - c[1]) <= m.geom_rbound[g] + res
            if not near.any():
                continue
            q = (P[near] - c) @ R                       # into the geom's frame
            t = m.geom_type[g]
            if t == mujoco.mjtGeom.mjGEOM_BOX:
                ins = np.all(np.abs(q) <= s[:3] + res / 2, axis=1)
            elif t == mujoco.mjtGeom.mjGEOM_CYLINDER:
                ins = (np.hypot(q[:, 0], q[:, 1]) <= s[0] + res / 2) & (np.abs(q[:, 2]) <= s[1])
            elif t == mujoco.mjtGeom.mjGEOM_CAPSULE:
                zc = np.clip(q[:, 2], -s[1], s[1])
                ins = np.sqrt(q[:, 0] ** 2 + q[:, 1] ** 2 + (q[:, 2] - zc) ** 2) <= s[0] + res / 2
            elif t in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
                ins = np.linalg.norm(q / np.maximum(s[:3] if t == mujoco.mjtGeom.mjGEOM_ELLIPSOID else s[0], 1e-6),
                                     axis=1) <= 1.0
            else:
                continue
            idx = np.flatnonzero(near)[ins]
            occ[idx] = True
        self.occ = occ.reshape(nx, ny)
        self.dist = ndimage.distance_transform_edt(~self.occ) * res
        self.gx, self.gy = np.gradient(self.dist, res)

    def lookup(self, xy: np.ndarray):
        """Distance to the nearest obstacle and its gradient at points (n, 2), bilinear."""
        f = (xy - self.origin) / self.res - 0.5
        i0 = np.floor(f).astype(int)
        a = f - i0
        nx, ny = self.dist.shape
        i0[:, 0] = np.clip(i0[:, 0], 0, nx - 2)
        i0[:, 1] = np.clip(i0[:, 1], 0, ny - 2)
        out = []
        for F in (self.dist, self.gx, self.gy):
            v = (F[i0[:, 0], i0[:, 1]] * (1 - a[:, 0]) * (1 - a[:, 1]) + F[i0[:, 0] + 1, i0[:, 1]] * a[:, 0] * (1 - a[:, 1])
                 + F[i0[:, 0], i0[:, 1] + 1] * (1 - a[:, 0]) * a[:, 1] + F[i0[:, 0] + 1, i0[:, 1] + 1] * a[:, 0] * a[:, 1])
            out.append(v)
        return out


class ScanMatcher:
    def __init__(self, map2d: Map2D, lidar_xy=(0.0, 0.0), huber: float = 0.04, outlier: float = 0.3, sigma: float = 0.02):
        self.map, self.off = map2d, np.asarray(lidar_xy, float)
        self.huber, self.outlier, self.sigma = huber, outlier, sigma

    def match(self, scan, history, max_iter: int = 12, init=None):
        """scan: sensors.Scan. history(t) -> (x, y, yaw) estimated at time t. Returns (pose, cov, quality) or None;
        quality = (inlier fraction, rms residual of the inliers m)."""
        ok = np.isfinite(scan.rng)
        if ok.sum() < 30:
            return None
        t, a, r = scan.t[ok], scan.ang[ok], scan.rng[ok]
        # de-skew: each point into the world with the pose at its own time, then back into the frame at the scan's end
        P = np.array([history(ti) for ti in t])
        c, s = np.cos(P[:, 2]), np.sin(P[:, 2])
        lx = self.off[0] + r * np.cos(a)
        ly = self.off[1] + r * np.sin(a)
        wx = P[:, 0] + c * lx - s * ly
        wy = P[:, 1] + s * lx + c * ly
        pe = np.array(history(scan.t_end))
        ce, se = math.cos(pe[2]), math.sin(pe[2])
        bx = ce * (wx - pe[0]) + se * (wy - pe[1])
        by = -se * (wx - pe[0]) + ce * (wy - pe[1])
        pose = pe.copy() if init is None else np.asarray(init, float).copy()
        self.last_pts = (bx, by)
        H = np.eye(3)
        used = 0
        for _ in range(max_iter):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            xy = np.column_stack([pose[0] + c * bx - s * by, pose[1] + s * bx + c * by])
            dd, gx, gy = self.map.lookup(xy)
            keep = dd < self.outlier
            used = int(keep.sum())
            if used < 30:
                return None
            w = np.where(np.abs(dd) <= self.huber, 1.0, self.huber / np.maximum(np.abs(dd), 1e-9))[keep]
            J = np.column_stack([gx, gy, gx * (-s * bx - c * by) + gy * (c * bx - s * by)])[keep]
            H = (J * w[:, None]).T @ J + 1e-6 * np.eye(3)
            step = -np.linalg.solve(H, (J * w[:, None]).T @ dd[keep])
            pose += step
            if np.linalg.norm(step[:2]) < 1e-4 and abs(step[2]) < 1e-4:
                break
        c, s = math.cos(pose[2]), math.sin(pose[2])
        dd = self.map.lookup(np.column_stack([pose[0] + c * bx - s * by, pose[1] + s * bx + c * by]))[0]
        inl = np.abs(dd) < 0.05
        quality = (float(inl.mean()), float(np.sqrt(np.mean(dd[inl] ** 2))) if inl.any() else 1.0)
        # covariance: the fit's information, inflated for correlated errors (map raster, de-skew, people), and floored
        # at 1.5 cm / 0.3 deg (what it gives with a perfect de-skew: ~1 cm): never better than its map and timing
        cov = self.sigma ** 2 * np.linalg.inv(H) * 9.0 + np.diag([0.015 ** 2, 0.015 ** 2, math.radians(0.3) ** 2])
        return pose, cov, quality

    def search(self, prior, span=1.0, yaw_span=math.radians(20), step=0.1, yaw_step=math.radians(5)):
        """Coarse re-localization around a prior (after a run of rejected fixes): the pose with the most points within
        5 cm of the map, using the last scan's de-skewed points."""
        bx, by = self.last_pts
        best, best_n = None, -1
        for dyaw in np.arange(-yaw_span, yaw_span + 1e-9, yaw_step):
            yaw = prior[2] + dyaw
            c, s = math.cos(yaw), math.sin(yaw)
            rx, ry = c * bx - s * by, s * bx + c * by
            for dx in np.arange(-span, span + 1e-9, step):
                for dy in np.arange(-span, span + 1e-9, step):
                    dd = self.map.lookup(np.column_stack([prior[0] + dx + rx, prior[1] + dy + ry]))[0]
                    n = int((dd < 0.08).sum())
                    if n > best_n:
                        best, best_n = np.array([prior[0] + dx, prior[1] + dy, yaw]), n
        return best


# -- the filter ---------------------------------------------------------------------------------

class EKF:
    def __init__(self, x0, use_imu=True, use_wheels=True, use_flow=False, use_lidar=False, flow_pos=(0.0, 0.0),
                 name: str = ""):
        self.name = name
        self.use = dict(imu=use_imu, wheels=use_wheels, flow=use_flow, lidar=use_lidar)
        self.s = np.array([*x0, 0.0, 0.0, 0.0])           # x, y, yaw, vx, vy, gyro bias
        self.P = np.diag([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, (0.1 * math.pi / 180) ** 2])
        self.flow_pos = np.asarray(flow_pos, float)
        self.w = 0.0                                        # last yaw rate used
        self.t = None
        self.hist = deque(maxlen=400)                       # (t, x, y, yaw) for de-skewing scans
        self.n_lidar = self.n_lidar_rej = self.n_reloc = self.reject_streak = 0

    def pose(self):
        return self.s[:3].copy()

    def history(self, t):
        h = self.hist
        if not h:
            return self.pose()
        if t >= h[-1][0]:
            return np.array(h[-1][1:])
        if t <= h[0][0]:
            return np.array(h[0][1:])
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, t1 = h[lo][0], h[hi][0]
        f = (t - t0) / max(t1 - t0, 1e-9)
        a, b = np.array(h[lo][1:]), np.array(h[hi][1:])
        out = a + f * (b - a)
        out[2] = a[2] + f * wrap_angle(b[2] - a[2])
        return out

    # -- predict ---------------------------------------------------------------------------
    def predict_imu(self, t, gz, ax, ay):
        if self.t is None:
            self.t = t
            return
        dt = t - self.t
        self.t = t
        s = self.s
        w = gz - s[5]
        self.w = w
        c, sn = math.cos(s[2]), math.sin(s[2])
        vx, vy = s[3], s[4]
        s[0] += (c * vx - sn * vy) * dt
        s[1] += (sn * vx + c * vy) * dt
        s[2] = wrap_angle(s[2] + w * dt)
        s[3] += (ax + w * vy) * dt
        s[4] += (ay - w * vx) * dt
        F = np.eye(6)
        F[0, 2] = (-sn * vx - c * vy) * dt
        F[0, 3], F[0, 4] = c * dt, -sn * dt
        F[1, 2] = (c * vx - sn * vy) * dt
        F[1, 3], F[1, 4] = sn * dt, c * dt
        F[2, 5] = -dt
        F[3, 4], F[3, 5] = w * dt, -vy * dt
        F[4, 3], F[4, 5] = -w * dt, vx * dt
        qa = 0.6 ** 2 * dt          # m/s^2: accelerometer noise, residual bias and tilt leaking gravity in (est)
        # position random walk: what the velocity model misses (wheel-radius error, slip, the base rocking) (est)
        sp = math.hypot(vx, vy)
        qp = (0.01 + 0.03 * sp) ** 2 * dt
        Q = np.diag([qp, qp, (0.003) ** 2 * dt, qa, qa, (0.002 * math.pi / 180) ** 2 * dt])
        self.P = F @ self.P @ F.T + Q
        self._log(t)

    def dead_reckon(self, t, vx, vy, wz):
        """Gyro-less: integrate the wheel twist."""
        if self.t is None:
            self.t = t
            return
        dt = t - self.t
        self.t = t
        s = self.s
        c, sn = math.cos(s[2]), math.sin(s[2])
        s[0] += (c * vx - sn * vy) * dt
        s[1] += (sn * vx + c * vy) * dt
        s[2] = wrap_angle(s[2] + wz * dt)
        s[3], s[4], self.w = vx, vy, wz
        self.P[:3, :3] += np.diag([(0.02 * dt) ** 2, (0.02 * dt) ** 2, (0.02 * dt) ** 2])
        self._log(t)

    def _log(self, t):
        if not self.hist or t - self.hist[-1][0] >= 0.0019:
            self.hist.append((t, *self.s[:3]))

    # -- corrections -------------------------------------------------------------------------
    def _update(self, z, h, H, R, gate=None):
        y = z - h
        S = H @ self.P @ H.T + R
        if gate is not None and float(y @ np.linalg.solve(S, y)) > gate:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.s = self.s + K @ y
        self.s[2] = wrap_angle(self.s[2])
        self.P = (np.eye(6) - K @ H) @ self.P
        return True

    def update_wheels(self, gz_last, vx, vy, wz, slip=0.0):
        """Wheel twist; trust drops with the wheels' disagreement (slip) and with speed."""
        sp = math.hypot(vx, vy)
        sv = 0.01 + 0.02 * sp + 3.0 * slip
        H = np.zeros((3, 6))
        H[0, 3] = H[1, 4] = 1.0
        H[2, 5] = -1.0                       # wheel yaw rate = gyro - bias
        z = np.array([vx, vy, wz - gz_last])
        h = np.array([self.s[3], self.s[4], -self.s[5]])
        R = np.diag([sv ** 2, sv ** 2, (0.02 + 0.05 * abs(wz) + 3.0 * slip) ** 2])
        self._update(z, h, H, R, gate=25.0)

    def update_flow(self, vx, vy):
        xf, yf = self.flow_pos
        w = self.w
        H = np.zeros((2, 6))
        H[0, 3] = H[1, 4] = 1.0
        z = np.array([vx, vy])
        h = np.array([self.s[3] - w * yf, self.s[4] + w * xf])
        sp = math.hypot(vx, vy)
        R = np.diag([(0.005 + 0.03 * sp) ** 2] * 2)
        self._update(z, h, H, R, gate=16.0)

    def update_pose(self, pose, cov):
        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        z = np.asarray(pose, float).copy()
        z[2] = self.s[2] + wrap_angle(z[2] - self.s[2])
        ok = self._update(z, self.s[:3].copy(), H, cov, gate=30.0)
        self.n_lidar += 1
        self.n_lidar_rej += not ok
        if ok and self.hist:                     # the history after a correction (for the next scan's de-skew)
            self.hist[-1] = (self.hist[-1][0], *self.s[:3])
        return ok

    def relocalize(self, pose):
        self.s[:3] = pose
        self.P[:3, :3] = np.diag([0.03 ** 2, 0.03 ** 2, math.radians(1.0) ** 2])
        self.P[:3, 3:] = 0.0
        self.P[3:, :3] = 0.0
        self.n_reloc += 1
        self.hist.clear()


class Localizer:
    """Shadow-mode estimators fed from one Sensors object. VARIANTS: name -> which sensors it uses."""

    VARIANTS = {
        "wheels": dict(use_imu=False, use_wheels=True),
        "wheels+IMU": dict(use_imu=True, use_wheels=True),
        "wheels+IMU+flow": dict(use_imu=True, use_wheels=True, use_flow=True),
        "wheels+IMU+lidar": dict(use_imu=True, use_wheels=True, use_lidar=True),
        "all": dict(use_imu=True, use_wheels=True, use_flow=True, use_lidar=True),
    }

    def __init__(self, robot, sensors, variants=None, map2d=None):
        self.r, self.sn = robot, sensors
        x0 = robot.pose()
        self.f = {k: EKF(x0, flow_pos=sensors.cfg.flow_pos, name=k, **self.VARIANTS[k])
                  for k in (variants or self.VARIANTS)}
        self.matcher = None
        if any(f.use["lidar"] for f in self.f.values()):
            m, d = robot.m, robot.d
            z = float(d.site_xpos[m.site("lidar").id][2])
            mp = map2d or Map2D(m, d, z, exclude_root=m.body_rootid[m.body("base").id])
            site = m.site("lidar").id
            off = d.site_xpos[site][:2] - d.xpos[m.body("base").id][:2]
            yaw = robot.pose()[2]
            c, s = math.cos(yaw), math.sin(yaw)
            self.matcher = ScanMatcher(mp, (c * off[0] + s * off[1], -s * off[0] + c * off[1]))
        self.gz = 0.0

    def step(self):
        rd = self.sn.step()
        if rd.imu is not None:
            t, gz, ax, ay = rd.imu
            self.gz = gz
            for f in self.f.values():
                if f.use["imu"]:
                    f.predict_imu(t, gz, ax, ay)
        if rd.wheels is not None:
            t, vx, vy, wz, slip = rd.wheels
            for f in self.f.values():
                if f.use["imu"]:
                    f.update_wheels(self.gz, vx, vy, wz, slip)
                else:
                    f.dead_reckon(t, vx, vy, wz)
        if rd.flow is not None and rd.flow[1] is not None:
            for f in self.f.values():
                if f.use["flow"]:
                    f.update_flow(*rd.flow[1])
        if rd.scan is not None and self.matcher is not None:
            for f in self.f.values():
                if not f.use["lidar"]:
                    continue
                res = self.matcher.match(rd.scan, f.history)
                if res is None:
                    continue
                pose, cov, (frac, rms) = res
                if frac < 0.5 or rms > 0.04:          # a poor fit (wrong basin, people, a moved chair): skip it
                    f.reject_streak += 1
                elif not f.update_pose(pose, cov):
                    f.reject_streak += 1
                else:
                    f.reject_streak = 0
                if f.reject_streak >= 5:              # lost: search around the estimate, then accept a good fit outright
                    guess = self.matcher.search(f.pose())
                    res = self.matcher.match(rd.scan, f.history, init=guess)
                    if res is not None and res[2][0] > 0.6 and res[2][1] < 0.03:
                        f.relocalize(res[0])
                        f.reject_streak = 0
        return rd

    def errors(self):
        """Pose error of each variant against the truth: (position m, yaw deg)."""
        tp = self.r.pose()
        return {k: (float(np.hypot(*(f.s[:2] - tp[:2]))), math.degrees(abs(wrap_angle(f.s[2] - tp[2]))))
                for k, f in self.f.items()}
