"""v9 (design.md 27): the robot's motion sensors, as the firmware and the Jetson will see them.

Every sensor reads the sim's truth and adds what the real part adds: noise from its datasheet, bias, rate,
quantization, its working limits and, for the lidar, the scan's own timing. All (est) values are marked; the
datasheet values carry a source.

  IMU      TDK ICM-42688-P on the MCU board (design.md 7), 1 kHz. Gyro noise 2.8 mdps/sqrt(Hz), accelerometer
           70 ug/sqrt(Hz) (TDK DS-000347). Residual bias after the at-rest calibration and its random walk (est).
           It reads the tilted base: on an edge, gravity leaks into the horizontal accelerations.
  Wheels   the drive encoders, 1 kHz; the kinematics turn them into a body twist (OmniKinematics or, for the swerve,
           SwerveKinematics with the measured steering angles). Slip is not visible to them.
  Flow     a downward optical-flow sensor: the PixArt PAA5100JE (Pimoroni "near optical flow"): 15-35 mm working
           range, max ~1.14 m/s at 25 mm (scales with height), 42 deg field of view (pimoroni.com). Firmware turns
           pixels into metres with a fixed nominal height unless `flow_tof` (a VL53L1X-class height sensor) corrects
           it. Out of range or too fast: no reading. The PMW3901 named in design.md 7 needs >= 80 mm; it is the
           wrong part for a sensor under the deck.
  Lidar    Slamtec RPLIDAR C1: 5 kHz samples, 10 Hz, 0.72 deg, 0.05-12 m (70% reflectivity; 6 m on black), +/-30 mm,
           15 mm resolution (slamtec.com). Rays are cast one by one as the head turns, so a moving robot's scan is
           skewed (de-skewing is the estimator's job). Returns from the robot itself (standoffs, the module) are
           masked, as the firmware will mask them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

DEG = math.pi / 180


@dataclass
class SensorConfig:
    seed: int = 0
    # IMU (ICM-42688-P)
    imu_rate: float = 1000.0
    gyro_nd: float = 2.8e-3 * DEG           # rad/s/sqrt(Hz) (datasheet)
    accel_nd: float = 70e-6 * 9.81          # m/s^2/sqrt(Hz) (datasheet)
    gyro_bias: float = 0.05 * DEG           # rad/s residual after the at-rest calibration (est; raw offset +/-0.5 dps)
    gyro_rw: float = 0.002 * DEG            # rad/s/sqrt(s) bias random walk (est)
    accel_bias: float = 0.01 * 9.81         # m/s^2 residual (est; raw offset tens of mg)
    gyro_scale: float = 0.003               # scale error (est)
    # wheels
    wheel_rate: float = 1000.0
    wheel_scale: float = 0.01               # wheel radius error: tyre / roller wear, build (est)
    # optical flow (PAA5100JE)
    flow: bool = True
    flow_rate: float = 100.0
    flow_pos: tuple = (-0.04, 0.0)          # m, body frame (under the deck, behind the battery's centre line; est)
    flow_height: float = 0.025              # m above a hard floor at rest (mid-range: 15-35 mm)
    flow_range: tuple = (0.015, 0.035)
    flow_vmax_25mm: float = 1.14            # m/s at 25 mm (pimoroni.com); scales with height
    flow_noise: float = 0.02                # fraction of speed (est)
    flow_noise_abs: float = 0.003           # m/s (est)
    flow_dropout: float = 0.02              # random frame losses (est)
    flow_tof: bool = False                  # a height sensor corrects the pixel scale
    # lidar (RPLIDAR C1)
    lidar: bool = True
    lidar_rate: float = 5000.0
    lidar_scan_hz: float = 10.0
    lidar_range: tuple = (0.05, 12.0)
    lidar_sigma: float = 0.012              # m (+/-30 mm accuracy ~ 2.5 sigma)
    lidar_res: float = 0.015                # m ranging resolution
    lidar_dropout: float = 0.03             # (est)


@dataclass
class Scan:
    t_end: float
    t: np.ndarray            # per-point time
    ang: np.ndarray          # per-point angle in the lidar frame (rad)
    rng: np.ndarray          # m, nan = no return


@dataclass
class Readings:
    """What arrived during the last step (None = nothing new)."""
    imu: tuple | None = None          # (t, gyro_z, accel_x, accel_y)
    wheels: tuple | None = None       # (t, vx, vy, wz, slip) body twist from the encoders; slip = wheel disagreement, m/s
    flow: tuple | None = None         # (t, vx, vy) at the sensor point, body frame, or None (invalid)
    scan: Scan | None = None


class Sensors:
    def __init__(self, robot, cfg: SensorConfig | None = None):
        self.r, self.m, self.d, self.p = robot, robot.m, robot.d, robot.p
        self.cfg = c = cfg or SensorConfig()
        self.rng = np.random.default_rng(c.seed)
        m = self.m
        self.base = m.body("base").id
        self.imu_site = m.site("imu").id
        self.lidar_site = m.site("lidar").id
        self.dt = m.opt.timestep
        self.bg = self.rng.normal(0, c.gyro_bias)
        self.ba = self.rng.normal(0, c.accel_bias, 2)
        self.gscale = 1 + self.rng.normal(0, c.gyro_scale)
        self.wscale = 1 + self.rng.normal(0, c.wheel_scale)
        self.t_imu = self.t_wheel = self.t_flow = 0.0
        # lidar: the head's angle advances at scan_hz; one sample every 1 / rate s
        self.n_scan = int(round(c.lidar_rate / c.lidar_scan_hz))
        self.k = 0
        self.t_lidar = 0.0
        self._sc_t, self._sc_a, self._sc_r = [], [], []
        robot_root = m.body_rootid[self.base]
        self.robot_geom = m.body_rootid[m.geom_bodyid] == robot_root
        self.ggroup = np.array([1, 1, 0, 1, 0, 0], dtype=np.uint8)     # structure, decor, netting; not group 2 (the lidar)
        self._gid = np.zeros(8, dtype=np.int32)
        self._dist = np.zeros(8)

    # -- per step ------------------------------------------------------------------------
    def step(self) -> Readings:
        c, d, out = self.cfg, self.d, Readings()
        t = d.time
        if t - self.t_imu >= 1 / c.imu_rate - 1e-9:
            self.t_imu = t
            self.bg += self.rng.normal(0, c.gyro_rw * math.sqrt(1 / c.imu_rate))
            sg = c.gyro_nd * math.sqrt(c.imu_rate / 2)
            sa = c.accel_nd * math.sqrt(c.imu_rate / 2)
            gz = self.gscale * d.sensor("imu_gyro").data[2] + self.bg + self.rng.normal(0, sg)
            acc = d.sensor("imu_accel").data[:2] + self.ba + self.rng.normal(0, sa, 2)
            out.imu = (t, float(gz), float(acc[0]), float(acc[1]))
        if t - self.t_wheel >= 1 / c.wheel_rate - 1e-9:
            self.t_wheel = t
            ws = self.r.wheel_speeds() * self.wscale
            kin = self.r.kin
            tw = kin.body_twist(ws)
            # slip: how far the wheels disagree with any rigid motion (X4: 4 speeds for 3 dof; swerve: 2n for 3)
            if hasattr(kin, "A"):
                b = np.empty(2 * kin.n)
                b[0::2] = ws * kin.R * np.cos(kin.steer)
                b[1::2] = ws * kin.R * np.sin(kin.steer)
                slip = float(np.linalg.norm(kin.A @ tw - b)) / math.sqrt(kin.n)
            else:
                slip = float(np.linalg.norm(kin.J @ tw - ws)) * self.p.wheel_radius / math.sqrt(len(ws))
            out.wheels = (t, float(tw[0]), float(tw[1]), float(tw[2]), slip)
        if c.flow and t - self.t_flow >= 1 / c.flow_rate - 1e-9:
            self.t_flow = t
            out.flow = self._flow(t)
        if c.lidar:
            out.scan = self._lidar(t)
        return out

    def _flow(self, t):
        c, d, m = self.cfg, self.d, self.m
        R = d.xmat[self.base].reshape(3, 3)
        pos_b = np.array([*c.flow_pos, c.flow_height])
        p_w = d.xpos[self.base] + R @ pos_b
        # height above whatever is under it (a rug is 12 mm closer than the floor)
        g = np.zeros(1, dtype=np.int32)
        h = mujoco.mj_ray(m, d, p_w, -R[:, 2], np.array([1, 0, 0, 0, 0, 0], np.uint8), 1, self.base, g)
        if h < 0:
            return (t, None)
        v_w = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, self.base, v_w, 0)   # [ang, lin] about the body origin
        w, vlin = v_w[:3], v_w[3:]
        v_pt = vlin + np.cross(w, p_w - d.xpos[self.base])
        v_b = R.T @ v_pt
        sp = float(np.hypot(v_b[0], v_b[1]))
        if not (c.flow_range[0] <= h <= c.flow_range[1]) or sp > c.flow_vmax_25mm * h / 0.025 or \
                self.rng.random() < c.flow_dropout:
            return (t, None)
        scale = 1.0 if c.flow_tof else h / c.flow_height       # pixels -> m with the nominal height
        # the flow also sees the base turning about the sensor's axis (the camera rotation), which pixel flow mixes in
        # at h x pitch rate; small and ignored here
        meas = v_b[:2] / scale
        meas = meas * (1 + self.rng.normal(0, c.flow_noise)) + self.rng.normal(0, c.flow_noise_abs, 2)
        return (t, (float(meas[0]), float(meas[1])))

    def _lidar(self, t):
        """Cast the samples due this step; return a Scan when the head completes a turn."""
        c, d, m = self.cfg, self.d, self.m
        n_due = int((t - self.t_lidar) * c.lidar_rate)
        if n_due <= 0:
            return None
        self.t_lidar += n_due / c.lidar_rate
        pnt = d.site_xpos[self.lidar_site].copy()
        R = d.site_xmat[self.lidar_site].reshape(3, 3)
        ks = self.k + np.arange(n_due)
        ang = 2 * math.pi * (ks % self.n_scan) / self.n_scan
        vec = (R @ np.vstack([np.cos(ang), np.sin(ang), np.zeros(n_due)])).T.copy()
        if len(self._gid) < n_due:
            self._gid = np.zeros(n_due, dtype=np.int32)
            self._dist = np.zeros(n_due)
        gid, dist = self._gid[:n_due], self._dist[:n_due]
        mujoco.mj_multiRay(m, d, pnt, vec.reshape(-1), self.ggroup, 1, -1, gid, dist, None, n_due, c.lidar_range[1])
        rng = dist.copy()
        bad = (gid < 0) | (rng < c.lidar_range[0]) | (rng > c.lidar_range[1])
        bad |= np.where(gid >= 0, self.robot_geom[np.maximum(gid, 0)], False)
        bad |= self.rng.random(n_due) < c.lidar_dropout
        rng = np.round((rng + self.rng.normal(0, c.lidar_sigma, n_due)) / c.lidar_res) * c.lidar_res
        rng[bad] = np.nan
        ts = t - (n_due - 1 - np.arange(n_due)) / c.lidar_rate
        self._sc_t.append(ts)
        self._sc_a.append(ang)
        self._sc_r.append(rng)
        out = None
        k_end = self.k + n_due
        if k_end // self.n_scan > self.k // self.n_scan:            # completed a turn
            T, A, Rr = (np.concatenate(x) for x in (self._sc_t, self._sc_a, self._sc_r))
            out = Scan(float(T[-1]), T, A, Rr)
            self._sc_t, self._sc_a, self._sc_r = [], [], []
        self.k = k_end
        return out
