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

v10 (design.md 28) adds the parts Fernando chose (design.md 7), selected with SensorConfig.parts(imu=, lidar=):
  BNO085   CEVA/Hillcrest 9-axis with on-chip fusion, read by the MCU at 400 Hz: calibrated gyro, accelerometer and
           the game rotation vector (gyro + accelerometer, no magnetometer: its yaw starts at 0 and drifts). 400 Hz
           and the game rotation vector's 0.5 deg/min heading drift, 2.5 / 1.5 deg dynamic / static error are
           reseller figures (SparkFun, for the BNO086 successor; VERIFY in CEVA's datasheet). Noise densities (est).
  A2M12    Slamtec RPLIDAR A2M12: 16 kHz samples, 10 Hz (5-15), 0.225 deg at 10 Hz, 0.2-12 m, accuracy 1% of range to
           3 m, 2% at 3-5 m, 2.5% beyond, +/-1.5 deg scan flatness (Slamtec datasheet). Range sigma = accuracy / 2.5
           (as for the C1); the flatness is a fixed tilt of this unit's scan plane, up to 1.5 deg (est).
  Scans are cut at whole turns (exactly lidar_rate / scan_hz samples each, starting at angle 0), so they map onto
  sensor_msgs/LaserScan with a constant time_increment.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

DEG = math.pi / 180


def _quat_zyx(yaw, pitch, roll):
    """Quaternion (w, x, y, z) of R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


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
    # v10: range accuracy as a fraction of range, by range band ((up to m, fraction), ...); None = lidar_sigma only
    lidar_acc_frac: tuple | None = None
    lidar_flatness: float = 0.0             # rad: this unit's scan plane tilt is drawn up to this
    # v10: cast the samples due every lidar_cast_dt s instead of every step. Each keeps its own time and its ray is
    # turned back by the robot's rotation since then; only the translation is batched (2 ms = 4 mm at 2 m/s).
    # 0 = every physics step, as v9
    lidar_cast_dt: float = 0.0
    # v10: which parts (names only; the numbers above are set by parts())
    imu_part: str = "ICM-42688-P"
    lidar_part: str = "C1"
    imu_orientation: bool = False           # BNO085: the game rotation vector (fused orientation) is reported
    grv_noise: float = 0.0                  # rad, white noise on the reported orientation (est)
    grv_tilt_bias: float = 0.0              # rad, roll / pitch reference error (est)

    @classmethod
    def parts(cls, imu: str = "ICM-42688-P", lidar: str = "C1", **kw) -> "SensorConfig":
        """A config for the named parts. ICM-42688-P / C1 are v9's (the defaults); BNO085 / A2M12 are v10's."""
        c = {}
        if imu == "BNO085":
            c.update(imu_part="BNO085", imu_rate=400.0, imu_orientation=True,
                     gyro_nd=0.01 * DEG,             # rad/s/sqrt(Hz) (est: a consumer MEMS gyro, BMI088 class 0.014)
                     accel_nd=150e-6 * 9.81,         # m/s^2/sqrt(Hz) (est)
                     # the on-chip dynamic calibration leaves a residual bias that shows as the game rotation vector's
                     # 0.5 deg/min heading drift (reseller figure, VERIFY)
                     gyro_bias=0.5 / 60 * DEG, gyro_rw=0.001 * DEG,
                     accel_bias=0.01 * 9.81,         # m/s^2 residual after its calibration (est, as v9's)
                     gyro_scale=0.002,               # (est)
                     grv_noise=0.1 * DEG, grv_tilt_bias=0.3 * DEG)
        elif imu != "ICM-42688-P":
            raise ValueError(imu)
        if lidar == "A2M12":
            c.update(lidar_part="A2M12", lidar_rate=16000.0, lidar_scan_hz=10.0, lidar_range=(0.2, 12.0),
                     lidar_acc_frac=((3.0, 0.01), (5.0, 0.02), (math.inf, 0.025)), lidar_sigma=0.0,
                     lidar_res=0.0005,               # m (est; Slamtec quote < 0.5 mm for the A2 family at short range)
                     lidar_flatness=1.5 * DEG)
        elif lidar != "C1":
            raise ValueError(lidar)
        return cls(**{**c, **kw})


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
    # v10: the full IMU report (t, quat wxyz or None, gyro xyz rad/s, accel xyz m/s^2 incl. gravity), IMU frame
    imu3: tuple | None = None


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
        # v10: the lidar's clock starts now (v9 started it at 0, so a sensor made after the house's 0.3 s settle cast
        # ~1,500-4,800 samples at once and its first scan was several turns long)
        self.t_lidar = self.d.time
        self._sc_t, self._sc_a, self._sc_r = [], [], []
        # v10: the 3-axis IMU report and the scan-plane tilt draw from their own stream, so v9's readings are unchanged
        r3 = self.rng3 = np.random.default_rng(10_000 + c.seed)
        self.bg3 = np.array([*r3.normal(0, c.gyro_bias, 2), self.bg])
        self.ba3 = np.array([*self.ba, r3.normal(0, c.accel_bias)])
        self.tilt_bias = r3.normal(0, c.grv_tilt_bias, 2) if c.grv_tilt_bias else np.zeros(2)
        self.yaw_drift = 0.0
        self.yaw0 = None
        phi, psi = r3.uniform(0, c.lidar_flatness), r3.uniform(0, 2 * math.pi)
        self.flat_tilt = (math.tan(phi), psi)
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
            g_true = d.sensor("imu_gyro").data
            gz = self.gscale * g_true[2] + self.bg + self.rng.normal(0, sg)
            acc = d.sensor("imu_accel").data[:2] + self.ba + self.rng.normal(0, sa, 2)
            out.imu = (t, float(gz), float(acc[0]), float(acc[1]))
            out.imu3 = self._imu3(t, g_true, gz, acc, sg, sa)
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
        if c.lidar and (c.lidar_cast_dt <= 0 or t - self.t_lidar >= c.lidar_cast_dt - 1e-9):
            out.scan = self._lidar(t)
        return out

    def _imu3(self, t, g_true, gz, acc_xy, sg, sa):
        """v10: the whole IMU report, in the IMU's frame. z rate and x, y accelerations are the ones in Readings.imu;
        the other axes draw from rng3. BNO085: plus the game rotation vector, whose yaw starts at 0 and drifts with
        the residual z-gyro bias and scale error; roll / pitch from gravity with a small reference error (est)."""
        c, d, r3 = self.cfg, self.d, self.rng3
        self.bg3[2] = self.bg
        self.bg3[:2] += r3.normal(0, c.gyro_rw * math.sqrt(1 / c.imu_rate), 2)
        gyro = np.array([*(self.gscale * g_true[:2] + self.bg3[:2] + r3.normal(0, sg, 2)), gz])
        acc = np.array([*acc_xy, d.sensor("imu_accel").data[2] + self.ba3[2] + r3.normal(0, sa)])
        q = None
        if c.imu_orientation:
            R = d.site_xmat[self.imu_site].reshape(3, 3)
            yaw = math.atan2(R[1, 0], R[0, 0])
            pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
            roll = math.atan2(R[2, 1], R[2, 2])
            if self.yaw0 is None:
                self.yaw0 = yaw
            self.yaw_drift += (gz - g_true[2]) / c.imu_rate          # what the fusion integrates beyond the truth
            n = r3.normal(0, c.grv_noise, 3)
            q = _quat_zyx(yaw - self.yaw0 + self.yaw_drift + n[2], pitch + self.tilt_bias[1] + n[1],
                          roll + self.tilt_bias[0] + n[0])
        return (t, q, gyro, acc)

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
        ts = t - (n_due - 1 - np.arange(n_due)) / c.lidar_rate
        a_cast = ang
        if c.lidar_cast_dt > 0:
            # v10: the batch is cast from the pose at its end; turn each ray back by the robot's rotation since its own
            # sample time, so batching costs only the translation (<= v * cast_dt: 4 mm at 2 m/s and 2 ms)
            a_cast = ang - float(d.sensor("imu_gyro").data[2]) * (t - ts)
        vec = (R @ np.vstack([np.cos(a_cast), np.sin(a_cast), np.zeros(n_due)])).T.copy()
        if len(self._gid) < n_due:
            self._gid = np.zeros(n_due, dtype=np.int32)
            self._dist = np.zeros(n_due)
        gid, dist = self._gid[:n_due], self._dist[:n_due]
        mujoco.mj_multiRay(m, d, pnt, vec.reshape(-1), self.ggroup, 1, -1, gid, dist, None, n_due, c.lidar_range[1])
        rng = dist.copy()
        tz, psi = self.flat_tilt
        if tz:
            # v10: this unit's scan plane is tilted (flatness). The rays are cast flat (a tilted ray from a site this
            # close to the deck would hit the robot, which a real mount clears); a ray tilted down meets the floor
            # (z = 0) first when that is nearer than what the flat ray hit. Walls are 0.6 m tall: no ray goes over.
            vz = tz * np.cos(ang - psi)
            down = vz < -1e-6
            r_floor = np.where(down, pnt[2] / np.maximum(-vz, 1e-6), np.inf)
            hit_floor = down & (r_floor < np.where(gid >= 0, rng, np.inf)) & (r_floor <= c.lidar_range[1])
            rng = np.where(hit_floor, r_floor, rng)
            gid = np.where(hit_floor, 0, gid)
        bad = (gid < 0) | (rng < c.lidar_range[0]) | (rng > c.lidar_range[1])
        bad |= np.where(gid >= 0, self.robot_geom[np.maximum(gid, 0)], False)
        bad |= self.rng.random(n_due) < c.lidar_dropout
        if c.lidar_acc_frac is None:
            noise = self.rng.normal(0, c.lidar_sigma, n_due)
        else:                                          # v10 (A2M12): accuracy is a fraction of range, by band
            frac = np.full(n_due, c.lidar_acc_frac[-1][1])
            for r_max, f in reversed(c.lidar_acc_frac):
                frac[rng <= r_max] = f
            noise = self.rng.normal(0, 1.0, n_due) * np.maximum(c.lidar_sigma, frac * rng / 2.5)
        rng = np.round((rng + noise) / c.lidar_res) * c.lidar_res
        rng[bad] = np.nan
        self._sc_t.append(ts)
        self._sc_a.append(ang)
        self._sc_r.append(rng)
        out = None
        k_end = self.k + n_due
        if k_end // self.n_scan > self.k // self.n_scan:            # completed a turn
            T, A, Rr = (np.concatenate(x) for x in (self._sc_t, self._sc_a, self._sc_r))
            # v10: cut exactly at the turn (angle 0); the samples past it start the next scan
            n_next = k_end % self.n_scan
            cut = len(T) - n_next
            first = max(0, cut - self.n_scan)                      # never more than one turn (after a time jump)
            out = Scan(float(T[cut - 1]), T[first:cut], A[first:cut], Rr[first:cut])
            self._sc_t, self._sc_a, self._sc_r = ([x[cut:]] for x in (T, A, Rr))
        self.k = k_end
        return out
