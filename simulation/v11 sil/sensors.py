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
  BNO085   CEVA/Hillcrest 9-axis with on-chip fusion, read by the MCU at 100 Hz (the robot's firmware since Nov 17;
           400 Hz before: parts(..., imu_rate=400) reproduces earlier runs): calibrated gyro, accelerometer and the
           game rotation vector (gyro + accelerometer, no magnetometer: its yaw starts at 0 and drifts). The game
           rotation vector's 0.5 deg/min heading drift, 2.5 / 1.5 deg dynamic / static error are reseller figures
           (SparkFun, for the BNO086 successor; VERIFY in CEVA's datasheet). Noise densities (est).
  A2M12    Slamtec RPLIDAR A2M12: 16 kHz samples, 10 Hz (5-15), 0.225 deg at 10 Hz, 0.2-12 m, accuracy 1% of range to
           3 m, 2% at 3-5 m, 2.5% beyond, +/-1.5 deg scan flatness (Slamtec datasheet). Range sigma = accuracy / 2.5
           (as for the C1); the flatness is a fixed tilt of this unit's scan plane, up to 1.5 deg (est).
  Scans are cut at whole turns (exactly lidar_rate / scan_hz samples each, starting at angle 0), so they map onto
  sensor_msgs/LaserScan with a constant time_increment.

Nov 16 (design.md 2, 4.6): the robot's lidar is the RPLIDAR A2M8 Fernando has, modelled from the bench
(characterization/lidar/README.md, test 1: timing, hardware) and Slamtec's protocol; parts(lidar="A2M8", lidar_mode=):
  A2M8     its four modes (A2M8_MODES): 2 / 4 / 8 / 4 k samples/s in Standard / Express / Boost / Stability (the device's
           default), an exact sample clock (+/-10 ppm measured); the turn rate set by the motor's PWM with no speed loop
           (A2M8_PWM_HZ, warm: 12.5 Hz at the default 660, so 320 samples a turn in Stability).
           Angles: the device reports each sample at the hit's bearing from the centre, the encoder's angle less 8 deg
           plus the triangulation parallax atan-like b / d (b = 24.0 mm; the SDK's correction), so a sample's time is
           not its reported angle's and the samples of a turn are not in angle order (measured: steps -6..+9 deg, a turn
           starting at about -6 deg). Here each ray is cast at that bearing (twice: the far-field bearing gives the range
           the parallax needs) and the turn is re-binned by angle (nearest return per bin), as the ROS driver's
           angle_compensate does; each bin keeps its sample's own time (Scan.t), so the bins near 360 deg carry the
           turn's first samples (the seam).
           Delivery: samples travel in packets (Standard 1 sample in 5 bytes; Express 32 in 84; Boost and Stability 96
           in 132) at 115200 baud; a capsule is decoded only when the next one has arrived (its angles are interpolated
           to the next one's start), so a turn reaches the host after its last sample by the rest of its capsule, the
           next capsule, the packet's transmission, and USB (fixed 0.5 ms est + 0-1 ms measured jitter): Stability
           24-49 ms + 11.5 ms (measured 24-49 ms above the fastest delivery). Scan.t_ready is that time; the scan is
           returned then, not at its last sample.
           Stamps (lidar_stamp): "clock" (default since Nov 17): as omni_cpp's rplidar_node stamps, from the device's
           sample clock (rplidar.hpp): bin n - 1's time from a line fitted to the turn's sample times against their
           encoder angles plus the turn's mean bearing offset, 1 / rate apart; the returns whose bearing lands across
           the seam are dropped. (The driver's clock adds ~0.1 ms of its own; not modelled.) "sllidar": as
           sllidar_ros2 stamps (the host's time when the previous turn was handed over; scan_time = the time between
           handovers; time_increment = scan_time / (n - 1)), so the stamp is late by a delivery and its spacing jitters
           by up to a capsule; "true": the turn's first sample's own time.
           Direction (Nov 17): the head turns clockwise seen from above, as the real one, so the bins (counter-
           clockwise from 0, REP-103) run against time: bin n - 1 first (scan_prep's time_order -1). A turn starts where
           the far-field bearing (encoder - 8 deg) is half a bin past 0, as rplidar_node cuts it, so the seam falls
           between bins. (Before Nov 17 it turned counter-clockwise, its turns cut at the encoder's 0.)
           Bins (lidar_bins, Nov 17): rplidar_node's (robot.yaml: 320, the warm turn's samples at PWM 660); 0 = one
           bin a sample (the runs before Nov 17: the same 320 in Stability at 12.5 Hz). A turn here holds a whole
           number of samples (rate / lidar_scan_hz), so at 12.5 Hz and 320 bins every turn's samples sit at the same
           place in their bins, which the real head (~327 a turn, drifting) never does: optimistic. lidar_scan_hz
           4000 / 327 is the realistic case (design.md 4.6).
           Range: quantized as the answer format carries it (measured): Standard 0.25 mm, Express 1 mm, Boost and
           Stability 1 / 2 / 4 / 8 mm under 0.512 / 2.048 / 4.096 / 16.4 m. Range noise, dropouts, the minimum range and
           the plane's tilt are the datasheet's / estimates until the bench's tests 2-6 (the A2M12's figures, VERIFY).
           Not modelled: the turn-to-turn period jitter (0.1-0.4 %), the speed wobble inside a turn (~0.03 ms) and the
           cold-start creep of the rate (~5 % over minutes).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

DEG = math.pi / 180

# Nov 16: the RPLIDAR A2M8 (characterization/lidar/README.md). Its modes: samples/s, samples a packet, bytes a packet,
# capsules decoded with the next one, range quantization ((up to m, step m), ...) of the answer format
_ULTRA_RES = ((0.512, 0.001), (2.048, 0.002), (4.096, 0.004), (16.384, 0.008), (math.inf, 0.016))
A2M8_MODES = {
    "Standard": (2000.0, 1, 5, False, ((math.inf, 0.00025),)),
    "Express": (4000.0, 32, 84, True, ((math.inf, 0.001),)),
    "Boost": (8000.0, 96, 132, True, _ULTRA_RES),
    "Stability": (4000.0, 96, 132, True, _ULTRA_RES),
}
# the turn rate against the motor's PWM, warm (20 s after start; the bench, Nov 16): no speed loop
A2M8_PWM_HZ = ((300, 4.854), (400, 7.005), (500, 9.174), (660, 12.5), (800, 15.564), (1023, 21.505))
A2M8_PARALLAX = 98361 / 4096 * 1e-3       # m: the SDK's parallax correction, angle += b/d + (b/d)^3 / 3 (rad)
A2M8_ANGLE_OFFSET = 8.0 * DEG             # the SDK's encoder-to-bearing offset (7.5 deg for a sample with no range)
A2M8_BINS = 320                            # rplidar_node's bins (robot.yaml; test_v11 checks they agree)


def a2m8_scan_hz(pwm: float) -> float:
    """The A2M8's turn rate at a motor PWM (warm), interpolated from the bench."""
    p, h = zip(*A2M8_PWM_HZ)
    return float(np.interp(pwm, p, h))


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
    lidar_blocked: tuple = ()               # ((start deg, width deg), ...) in the lidar's frame (0 = forward, + = left):
                                            # sectors the robot's own parts hide (the drawn robot: BLOCKED_CAD)
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
    # Nov 16 (the A2M8, set by parts()): packets of lidar_packet samples (0: the turn handed over at its last sample,
    # as before), lidar_packet_bytes each at lidar_baud, a capsule decoded with the next one (lidar_decode_next), USB's
    # fixed delay and jitter span (s); the parallax baseline (m; 0: angles are the encoder's) and the encoder-to-bearing
    # offset; range quantization by band ((up to m, step m), ...; None: lidar_res); the driver's stamps
    lidar_packet: int = 0
    lidar_packet_bytes: int = 0
    lidar_baud: int = 115200
    lidar_decode_next: bool = False
    lidar_usb: tuple = (0.0005, 0.001)      # s: fixed (est) and jitter span (measured p95 ~1 ms above the fastest)
    lidar_parallax: float = 0.0
    lidar_angle_offset: float = 0.0
    lidar_res_bands: tuple | None = None
    lidar_stamp: str = "true"               # "true" (the first sample's own time) | A2M8: "clock" | "sllidar" (docstring)
    lidar_bins: int = 0                     # A2M8: angle bins a turn (rplidar_node's bins); 0: one a sample (before Nov 17)
    lidar_mode: str = ""
    # v10: which parts (names only; the numbers above are set by parts())
    imu_part: str = "ICM-42688-P"
    lidar_part: str = "C1"
    imu_orientation: bool = False           # BNO085: the game rotation vector (fused orientation) is reported
    grv_noise: float = 0.0                  # rad, white noise on the reported orientation (est)
    grv_tilt_bias: float = 0.0              # rad, roll / pitch reference error (est)

    @classmethod
    def parts(cls, imu: str = "ICM-42688-P", lidar: str = "C1", lidar_mode: str = "Stability", lidar_pwm: float = 660,
              **kw) -> "SensorConfig":
        """A config for the named parts. ICM-42688-P / C1 are v9's (the defaults); BNO085 / A2M12 are v10's; A2M8 the
        robot's since Nov 16 (lidar_mode: one of A2M8_MODES, Stability the device's default; lidar_pwm: its motor)."""
        c = {}
        if imu == "BNO085":
            c.update(imu_part="BNO085", imu_rate=100.0, imu_orientation=True,     # Nov 17: the firmware's rate
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
        elif lidar == "A2M8":
            if lidar_mode not in A2M8_MODES:
                raise ValueError(f"A2M8 mode {lidar_mode!r}: one of {', '.join(A2M8_MODES)}")
            rate, k, nb, nxt, res = A2M8_MODES[lidar_mode]
            c.update(lidar_part="A2M8", lidar_mode=lidar_mode, lidar_rate=rate, lidar_scan_hz=a2m8_scan_hz(lidar_pwm),
                     lidar_range=(0.15, 12.0),       # datasheet 0.15-12 m; the device lists 12 m for every mode (VERIFY: test 6)
                     # range noise: the A2 family's datasheet figures, as the A2M12's (VERIFY: test 2)
                     lidar_acc_frac=((3.0, 0.01), (5.0, 0.02), (math.inf, 0.025)), lidar_sigma=0.0,
                     lidar_res=res[0][1], lidar_res_bands=res,
                     lidar_flatness=1.5 * DEG,       # est, as the A2M12's (VERIFY: test 5)
                     lidar_packet=k, lidar_packet_bytes=nb, lidar_baud=115200, lidar_decode_next=nxt,
                     lidar_parallax=A2M8_PARALLAX, lidar_angle_offset=A2M8_ANGLE_OFFSET, lidar_stamp="clock",
                     lidar_bins=A2M8_BINS)
        elif lidar != "C1":
            raise ValueError(lidar)
        return cls(**{**c, **kw})


# the drawn robot's lidar shadows (cad/v7, design.md 12.2: 22 deg blocked, the widest wedge 8.2 deg): the payload
# plate's rear standoffs and the head's side posts, (start deg, width deg) in the lidar's frame, from
# studies/layout_trade.py C2 (results/layout_trade.json, "any tilt")
BLOCKED_CAD = ((137.2, 8.2), (170.8, 2.8), (186.8, 2.8), (214.8, 8.2))


@dataclass
class Scan:
    t_end: float
    t: np.ndarray            # per-point time
    ang: np.ndarray          # per-point angle in the lidar frame (rad)
    rng: np.ndarray          # m, nan = no return
    # Nov 16 (the A2M8; None: the earlier model's, i.e. t[0], 1 / rate, n / rate, t_end): the driver's view of the
    # turn (its stamp, time between samples and turn time) and when the host has it
    stamp: float | None = None
    time_increment: float | None = None
    scan_time: float | None = None
    t_ready: float | None = None


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
        self.n_bins = c.lidar_bins if c.lidar_packet and c.lidar_bins > 0 else self.n_scan    # the A2M8's /scan bins
        self.k = 0
        # v10: the lidar's clock starts now (v9 started it at 0, so a sensor made after the house's 0.3 s settle cast
        # ~1,500-4,800 samples at once and its first scan was several turns long)
        self.t_lidar = self.d.time
        self._sc_t, self._sc_a, self._sc_r, self._sc_e = [], [], [], []
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
        # Nov 16 (the A2M8's packets): its own random stream, turns waiting for delivery, the last handover
        self.rng_l = np.random.default_rng(20_000 + c.seed)
        self._pending: list = []
        self._prev_ready = None
        self._cap_g = int(self.rng_l.integers(0, max(1, c.lidar_packet)))   # the device's sample count at a turn's end
        # Nov 17: the first sample of a turn, where the far-field bearing (encoder - 8 deg) is half a bin past 0
        self.k_cut = math.ceil((c.lidar_angle_offset + math.pi / self.n_bins) / (2 * math.pi / self.n_scan) - 1e-9)

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
        elif c.lidar and self._pending and self._pending[0].t_ready <= t + 1e-12:    # the A2M8: a turn delivered
            out.scan = self._pending.pop(0)
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

    def _cast(self, pnt, R, a, n):
        """Rays at angles a (lidar frame) from pnt: (geom ids, distances), copies."""
        vec = (R @ np.vstack([np.cos(a), np.sin(a), np.zeros(n)])).T.copy()
        if len(self._gid) < n:
            self._gid = np.zeros(n, dtype=np.int32)
            self._dist = np.zeros(n)
        gid, dist = self._gid[:n], self._dist[:n]
        mujoco.mj_multiRay(self.m, self.d, pnt, vec.reshape(-1), self.ggroup, 1, -1, gid, dist, None, n,
                           self.cfg.lidar_range[1])
        return gid.copy(), dist.copy()

    def _lidar_packets(self, t):
        """Nov 16, the A2M8 (module docstring): parallax-corrected bearings, quantized ranges, turns re-binned by angle
        and handed over when the host would have them, with the driver's stamps."""
        c, d = self.cfg, self.d
        n_due = int((t - self.t_lidar) * c.lidar_rate)
        if n_due > 0:
            self.t_lidar += n_due / c.lidar_rate
            pnt = d.site_xpos[self.lidar_site].copy()
            R = d.site_xmat[self.lidar_site].reshape(3, 3)
            ks = self.k + np.arange(n_due)
            # Nov 17: the head turns clockwise seen from above, as the real one (its angles clockwise); here the
            # angles are the laser frame's, counter-clockwise (REP-103), so a reported bearing is minus the device's
            enc = 2 * math.pi * (ks % self.n_scan) / self.n_scan            # the head's encoder angle, clockwise
            ts = t - (n_due - 1 - np.arange(n_due)) / c.lidar_rate
            turn = (-float(d.sensor("imu_gyro").data[2]) * (t - ts)) if c.lidar_cast_dt > 0 else 0.0
            far = -enc + c.lidar_angle_offset                                # the device's encoder - 8 deg, ccw
            gid, dist = self._cast(pnt, R, far + turn, n_due)                # the far-field bearing: its range
            x = c.lidar_parallax / np.maximum(dist, 0.05)
            ang = np.where((gid >= 0) & (dist >= 0.05), far - x - x ** 3 / 3, -enc + 7.5 * DEG)
            hit = gid >= 0
            gid2, rng = self._cast(pnt, R, ang + turn, n_due)                # at the reported bearing
            gid = np.where(hit, gid2, -1)
            ang = np.mod(ang, 2 * math.pi)
            tz, psi = self.flat_tilt
            if tz:                                                           # the plane's tilt (as the earlier model)
                vz = tz * np.cos(ang - psi)
                down = vz < -1e-6
                r_floor = np.where(down, pnt[2] / np.maximum(-vz, 1e-6), np.inf)
                hit_floor = down & (r_floor < np.where(gid >= 0, rng, np.inf)) & (r_floor <= c.lidar_range[1])
                rng = np.where(hit_floor, r_floor, rng)
                gid = np.where(hit_floor, 0, gid)
            bad = (gid < 0) | (rng < c.lidar_range[0]) | (rng > c.lidar_range[1])
            bad |= np.where(gid >= 0, self.robot_geom[np.maximum(gid, 0)], False)
            bad |= self.rng_l.random(n_due) < c.lidar_dropout
            for a0, width in c.lidar_blocked:
                bad |= np.mod(np.degrees(ang) - a0, 360.0) < width
            frac = np.full(n_due, c.lidar_acc_frac[-1][1])
            for r_max, f in reversed(c.lidar_acc_frac):
                frac[rng <= r_max] = f
            rng = rng + self.rng_l.normal(0, 1.0, n_due) * np.maximum(c.lidar_sigma, frac * rng / 2.5)
            res = np.full(n_due, c.lidar_res_bands[-1][1])
            for r_max, s in reversed(c.lidar_res_bands):
                res[rng < r_max] = s
            rng = np.round(rng / res) * res
            rng[bad] = np.nan
            self._sc_t.append(ts)
            self._sc_a.append(ang)
            self._sc_r.append(rng)
            self._sc_e.append(enc)
            # a turn starts where the far-field bearing is half a bin past the 0 mark (omni_cpp rplidar.hpp's cut:
            # the seam falls between bins, bin n - 1 first)
            n, k0 = self.n_scan, self.k_cut
            k_end = self.k + n_due
            if (k_end - k0) // n > (self.k - k0) // n:                    # a turn completed
                T, A, Rr, E = (np.concatenate(v) for v in (self._sc_t, self._sc_a, self._sc_r, self._sc_e))
                cut = len(T) - (k_end - k0) % n
                first = max(0, cut - n)
                if cut - first == n:                                         # not the samples before the first cut
                    self._pending.append(self._turn(T[first:cut], A[first:cut], Rr[first:cut], E[first:cut]))
                self._sc_t, self._sc_a, self._sc_r, self._sc_e = ([v[cut:]] for v in (T, A, Rr, E))
            self.k = k_end
        if self._pending and self._pending[0].t_ready <= t + 1e-12:
            return self._pending.pop(0)
        return None

    def _turn(self, T, A, Rr, E):
        """One turn's samples -> a Scan re-binned by angle (nearest return per bin, its own time kept), with when the
        host has it and the driver's stamp. The head turns clockwise: bin n - 1 first, the array against time."""
        c, ns, n = self.cfg, self.n_scan, self.n_bins          # samples a turn, bins a turn
        inc = 2 * math.pi / n
        b = np.round(np.mod(A, 2 * math.pi) / inc).astype(int) % n
        out = np.full(n, np.inf)
        ok = np.isfinite(Rr)
        # rplidar_node's view (rplidar.hpp): each sample's place in time order from its encoder angle and the turn's
        # mean bearing offset (-8 deg plus the parallax); a return whose bearing lands across the seam is dropped
        off = float(np.mean(np.mod(-A - E + math.pi, 2 * math.pi) - math.pi))     # cw bearing - encoder
        x0 = np.mod(E[0] + off - inc / 2 + math.pi, 2 * math.pi) - math.pi        # the first sample, near the cut
        x = (x0 + np.unwrap(E) - E[0]) / inc - 0.5
        if c.lidar_stamp == "clock":
            ok &= np.abs((n - 1 - b) - x) <= n / 2
        np.minimum.at(out, b[ok], Rr[ok])
        t_bin = T[0] + (n - 1 - np.arange(n)) * ns / n / c.lidar_rate         # empty bins: the nominal time
        kept = ok & (Rr == out[b])
        t_bin[b[kept]] = T[kept]
        rng = np.where(np.isfinite(out), out, np.nan)
        # delivery: the rest of the last sample's packet, the next packet for a capsule, then its bytes and USB. Where
        # the turn ends in its capsule follows the device's own sample count, whose turns wander by ~1 sample (the
        # bench: 0.5-1.4 a turn; the sim's are exact and would lock the turn to the capsules)
        K = c.lidar_packet
        self._cap_g += ns + int(round(self.rng_l.normal(0, 0.8)))
        rest = (K - 1 - self._cap_g % K) + (K if c.lidar_decode_next else 0)
        t_ready = (T[-1] + rest / c.lidar_rate + c.lidar_packet_bytes * 10 / c.lidar_baud
                   + c.lidar_usb[0] + self.rng_l.uniform(0, c.lidar_usb[1]))
        if c.lidar_stamp == "sllidar":
            stamp = self._prev_ready if self._prev_ready is not None else t_ready - ns / c.lidar_rate
            scan_time = t_ready - stamp
            dt = scan_time / (n - 1)                                          # sllidar's time_increment
        elif c.lidar_stamp == "true":
            stamp, scan_time = float(T[0]), ns / c.lidar_rate
            dt = scan_time / n
        elif c.lidar_stamp == "clock":                                        # omni_cpp rplidar_node: the sample clock
            dt, c0 = np.polyfit(x, T - T[0], 1)                               # the line rplidar_node fits a turn
            stamp, scan_time = float(T[0] + c0), float(dt * n)
        else:
            raise ValueError(f"lidar_stamp {c.lidar_stamp!r}: 'clock', 'true' or 'sllidar'")
        self._prev_ready = t_ready
        return Scan(float(T[-1]), t_bin, np.arange(n) * inc, rng, stamp=float(stamp),
                    time_increment=float(dt), scan_time=float(scan_time), t_ready=float(t_ready))

    def _lidar(self, t):
        """Cast the samples due this step; return a Scan when the head completes a turn."""
        c, d, m = self.cfg, self.d, self.m
        if c.lidar_packet:
            return self._lidar_packets(t)
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
        for a0, width in c.lidar_blocked:              # the robot's own standoffs: inside the minimum range, no return
            bad |= np.mod(np.degrees(ang) - a0, 360.0) < width
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
