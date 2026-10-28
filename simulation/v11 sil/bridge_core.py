"""v10 (design.md 28): the MuJoCo side of the ROS 2 bridge, with no ROS in it (the node in ros/omni_bridge wraps it).

One SimCore owns the v7 house (house.py, base house.BASES["v7"], 0.5 ms physics), the robot's sensors as the real
parts report them (SensorConfig.parts: BNO085 IMU, RPLIDAR A2M12) and what the MCU would send the Jetson:

  imu      every IMU report (400 Hz): game rotation vector, calibrated gyro, accelerometer (with gravity), IMU frame
  odom     wheel odometry (odom_hz, est 50 Hz): the encoders' body twist averaged over the period and integrated
           into a pose in the odom frame (its origin = where the robot stood when the bridge started), with a
           covariance that grows with speed and with the wheels' disagreement (slip), as v9's EKF trusts them
  scan     one turn of the lidar (A2M12: 1,600 samples, each with its own time; the scan is skewed when moving)
  truth    the base's true pose and twist in the world (the house) frame, for scoring only
  shadow   v9's own estimators (estimator.Localizer: EKF + known-map matcher) on the same readings, for comparison

Commands come in as a body-frame twist (ROS /cmd_vel) through Driver.step -> Robot.drive_velocity, so the
firmware's speed, acceleration and jerk limits and the terrain-aware caps still apply; with no command for
cmd_timeout the firmware ramps to a stop (design.md 3: stops are always ramped). mode "tour" drives v9's house
tour (studies/localization.py TOUR, then the wall push) on the sim's truth instead, as v9 did.

v11 (design.md 30), firmware=True (the default for mode "cmd_vel"): the robot is sil_robot.SilRobot, its C++ firmware
in the loop. Commands then come over the firmware's USB port (the bridge's McuPort; mcu_link and the mode manager on
the ROS side), and the firmware sends the odometry and the IMU: this core emits only scans and the truth. The firmware
has no terrain assist (the edge map isn't ported: design.md 30), and the tour still needs firmware=False (it drives on
the truth through the Python controller, as v9 and v10's scored runs did).
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "studies"))

import house  # noqa: E402
from estimator import Localizer  # noqa: E402
from omni_control import wrap_angle  # noqa: E402
from sensors import Sensors, SensorConfig  # noqa: E402

# v9's house tour (studies/localization.py), the same waypoints and wall push
TOUR = [(1.5, 1.25, 0), (3.45, 1.25, 30), (3.45, 3.0, 90), (4.6, 3.25, 0), (3.45, 3.0, -60), (3.45, 1.25, -90),
        (0.45, 1.25, 180), (0.45, 3.2, 90), (0.45, 1.25, 45), (-3.0, 1.25, 180), (-3.0, 3.3, 90), (-3.0, 1.25, 0),
        (-3.0, -0.6, -90), (-3.1, -2.4, -135), (-0.8, -2.0, 0), (2.5, -2.0, 60), (2.5, -0.2, 90), (-1.0, 0.0, 180),
        (-3.0, 1.25, 0)]


@dataclass
class CoreConfig:
    base: str = "v7"
    assist: bool = True                  # terrain-aware firmware (design.md 25.4); v9's tour ran without it
    wheel_model: str = "rollers"         # crossings need the explicit rollers (CLAUDE.md)
    seed: int = 0
    imu: str = "BNO085"
    lidar: str = "A2M12"
    flow: bool = False                   # the PAA5100JE is optional (design.md 27); not on the ROS side
    odom_hz: float = 50.0                # MCU -> Jetson odometry rate (est)
    truth_hz: float = 50.0
    cmd_timeout: float = 0.5             # s without /cmd_vel -> ramp to a stop (est; the MCU's heartbeat rule)
    shadow: tuple = ("wheels+IMU", "wheels+IMU+lidar")   # v9 estimators run alongside (() = none)
    mode: str = "cmd_vel"                # or "tour"
    start: tuple = house.START           # (x, y, yaw deg) in the house
    tour_rounds: int = 1
    lidar_cast_dt: float = 0.002         # sensors.SensorConfig.lidar_cast_dt (speed; every sample keeps its own time)
    # firmware rates in physics steps (speed; design.md 28.2): the base controller (the MCU's velocity loop, 1 kHz as
    # design.md 4), the floor hooks (friction field, rolling resistance) and the electrical bookkeeping. Contacts,
    # motor drivers and the physics itself still run every 0.5 ms step. 1 = every step (v9 and the studies).
    fw_div: int = 2
    floor_div: int = 2
    post_div: int = 2
    sensor_kw: dict = field(default_factory=dict)   # overrides, e.g. lidar_flatness=0 for a levelled unit
    firmware: bool = False               # v11: the C++ firmware in the loop (sil_robot.SilRobot); not with mode "tour"
    person: bool = False                 # a person walking in front of the robot's start (the console's Follow / Tag)
    timing: str = "ideal"                # design.md 10: "real" = the MCU's drifting clock, CAN reply timing (SilRobot)
    # design.md 11: "arm" = the arm robot (robot_export.ARM: the SO-101 on its nose, the stereo pods looking down, the
    # lidar at the rear, the bin) with n_objects things from arm_objects.SETS[objects] on the floor ahead of its start
    payload: str = "catch"
    objects: str = "toys"
    n_objects: int = 6
    joint_err_deg: float = 1.0           # the arm's servos off their calibration (arm_bus)
    # the Launcher's live world (sim_world.World): the person, the clutter ("off", "toys", "all" = toys and larger
    # obstacles) and the house's thrower switched on and off while the sim runs
    live_world: bool = False
    clutter: str = "off"
    thrower: bool = True


@dataclass
class Events:
    """What became due in the last step (None / empty = nothing)."""
    t: float = 0.0
    imu: tuple | None = None             # (t, quat wxyz | None, gyro xyz, accel xyz)
    odom: tuple | None = None            # (t, (x, y, yaw), (vx, vy, wz), pose_var (3,), twist_var (3,))
    scan: object | None = None           # sensors.Scan
    truth: tuple | None = None           # (t, (x, y, z), quat wxyz, (vx, vy, wz) body)
    shadow: dict = field(default_factory=dict)   # variant -> (x, y, yaw) in the world, at truth times
    phase: str | None = None             # tour: "tour", "wall", "done" when it changes


ARM_AREA = ((-1.9, 1.0), 0.8)           # the arm world's things: within 0.8 m of this, in the hallway and the living room


def toy_objects(c: CoreConfig) -> tuple:
    """The things on the floor of the arm's world (design.md 11): n_objects from arm_objects.SETS[objects], scattered
    in ARM_AREA (params.floor_objects entries)."""
    import arm_objects
    names = list(arm_objects.SETS[c.objects]) * 3
    rng = np.random.default_rng(c.seed + 900)
    names = [names[k] for k in rng.permutation(len(names))[:c.n_objects]]
    return arm_objects.scatter(names, ARM_AREA[0], ARM_AREA[1], seed=c.seed, keep_out=(house.START[:2], 0.5))


def arm_kw(c: CoreConfig) -> dict:
    """SimParams overrides for the arm robot in the house, its things on the floor (design.md 11)."""
    from robot_export import ARM
    return dict(ARM, floor_objects=toy_objects(c))


MODEL_FIELDS = ("base", "assist", "wheel_model", "seed", "person", "payload", "objects", "n_objects", "live_world")


def model_params(c: CoreConfig):
    """The house and the robot SimCore builds for c (only c's MODEL_FIELDS count): ros/tools/mujoco_view.py builds the
    same from the bridge's hello. live_world: the person and every piece of clutter are in the model, parked outside
    the house until sim_world.World puts them in."""
    kw = dict(person=True) if c.person or c.live_world else {}
    base = c.base
    if c.base == "drawn":
        # the robot as cad/v7 draws it (the baseline since Oct 25, design.md 12.2): robot_export.V11 with the CAD's
        # masses and its 150 mm scan plane
        from robot_export import V11, drawn_params
        base = "v7"
        kw.update({k: v for k, v in V11.items() if k not in ("wheel_model", "limits", "base_max_speed")})
        if c.payload != "arm":
            kw.update(mass_model="cad", extra_masses=drawn_params().extra_masses)
    if c.payload == "arm":
        kw.update(arm_kw(c))
    if c.live_world:
        import sim_world
        kw["floor_objects"] = sim_world.parked_objects(c)
    return house.house_params(base, assist=c.assist, wheel_model=c.wheel_model, **kw)


def model_hello(c: CoreConfig) -> dict:
    """c's MODEL_FIELDS, for the viewer (JSON)."""
    return {k: getattr(c, k) for k in MODEL_FIELDS}


class SimCore:
    def __init__(self, cfg: CoreConfig | None = None):
        self.cfg = c = cfg or CoreConfig()
        sensor_kw = dict(c.sensor_kw)
        if c.base == "drawn":                            # the lidar's shadows on the drawn robot (model_params)
            from sensors import BLOCKED_CAD
            sensor_kw.setdefault("lidar_blocked", BLOCKED_CAD)
        self.drv = house.Driver(model_params(c))
        if tuple(c.start) != tuple(house.START):
            self.drv.teleport(c.start)
        self.firmware = c.firmware and c.mode != "tour"
        if self.firmware:
            from sil_robot import SilRobot
            self.drv.r = SilRobot(self.drv.m, self.drv.d, self.drv.p, seed=c.seed, lidar=True, timing=c.timing,
                                  sensor_kw=dict(lidar_cast_dt=c.lidar_cast_dt, **sensor_kw))
        self.m, self.d, self.r = self.drv.m, self.drv.d, self.drv.r
        self.dt = self.m.opt.timestep
        self.bus_sim = None
        if c.payload == "arm":                          # the SO-101's servo bus (arm_bus; the bridge serves it on TCP)
            from arm_bus import ServoBusSim
            self.bus_sim = ServoBusSim(self.r.arm, joint_err_deg=c.joint_err_deg, seed=c.seed)
        self.r.floor_div, self.r.post_div = c.floor_div, c.post_div
        self._k = 0
        if self.firmware:
            self.sn, self.loc = self.r.sn, None
        else:
            self.sn = Sensors(self.r, SensorConfig.parts(c.imu, c.lidar, seed=c.seed, flow=c.flow,
                                                         lidar_cast_dt=c.lidar_cast_dt, **sensor_kw))
            self.loc = Localizer(self.r, self.sn, variants=list(c.shadow)) if c.shadow else None
        self.p = self.drv.p
        # frames: base_link = the MuJoCo base body; laser, imu = their sites (fixed on the base)
        self.base_id = self.m.body("base").id
        self.t0 = self.d.time
        self.pose0 = self.r.pose().copy()             # odom frame origin, in the world
        self.odom = np.zeros(3)
        self.odom_var = np.zeros(3)
        self._tw_acc, self._tw_n, self._slip_max = np.zeros(3), 0, 0.0
        self._next_odom = self.d.time + 1 / c.odom_hz
        self._next_truth = self.d.time
        self.cmd = np.zeros(3)
        self.t_cmd = -math.inf
        self.n_cmd = 0
        # tour state
        self.phase = "tour" if c.mode == "tour" else None
        self._wp, self._t_wp, self._round = 0, self.d.time, 0
        self.dist = 0.0
        self._last_xy = self.r.pose()[:2].copy()
        self.walker = None
        self.world = None
        if c.live_world:
            from sim_world import World
            self.world = World(self, person=c.person, clutter=c.clutter, thrower=c.thrower)
        elif c.person:
            self.walker = Walker(self.m, self.d, self.r.pose(), self.base_id)

    # -- static frames ---------------------------------------------------------------------------
    def frame(self, site: str):
        """(xyz, quat wxyz) of a site in the base body frame."""
        m, d = self.m, self.d
        R = d.xmat[self.base_id].reshape(3, 3)
        sid = m.site(site).id
        p = R.T @ (d.site_xpos[sid] - d.xpos[self.base_id])
        Rs = R.T @ d.site_xmat[sid].reshape(3, 3)
        return p, _mat2quat(Rs)

    # -- commands --------------------------------------------------------------------------------
    def set_cmd(self, vx: float, vy: float, wz: float):
        self.cmd[:] = (vx, vy, wz)
        self.t_cmd = self.d.time
        self.n_cmd += 1

    def _drive(self):
        c, r, drv, d = self.cfg, self.r, self.drv, self.d
        if self.firmware:                               # the firmware takes its commands over its USB port
            r.step()
            return
        self._k += 1
        if self._k % c.fw_div:                          # the controller runs at its own rate: hold its command
            r.step()
            return
        if self.phase in ("tour", "wall", "back"):
            self._tour()
            return
        if self.phase == "done" or d.time - self.t_cmd > c.cmd_timeout:
            v, w = np.zeros(2), 0.0                     # firmware: ramp to a stop within its limits
        else:
            v, w = self.cmd[:2].copy(), float(self.cmd[2])
        # house.Driver.step without its tilt / airborne bookkeeping (a second wheel_loads() every step)
        pose = r.pose()
        cy, sy = math.cos(pose[2]), math.sin(pose[2])
        v_world = (cy * v[0] - sy * v[1], sy * v[0] + cy * v[1])
        r.edges = drv._edges if c.assist else []
        if not c.assist:
            r.ctl.max_speed = self.p.base_max_speed
            r.terrain_yaw = None
        h = self.dt * c.fw_div
        if r.terrain_yaw is not None or abs(w) < 1e-3:
            r.drive_velocity(v_world, None, pose[2], dt=h)    # firmware holds or aligns the heading
        else:
            r._hold_yaw = None
            r.drive_velocity(v_world, w, pose[2], dt=h)
        r.step()

    def _tour(self):
        r, d = self.r, self.d
        pose = r.pose()
        h = self.dt * self.cfg.fw_div
        if self.phase == "tour":
            wp = TOUR[self._wp]
            target = np.array([wp[0], wp[1], math.radians(wp[2])])
            r.drive_to(pose, target, dt=h)
            r.step()
            arrived = np.linalg.norm(pose[:2] - target[:2]) < 0.03 and abs(wrap_angle(pose[2] - target[2])) < 0.05
            if arrived or d.time - self._t_wp >= 8.0:
                self._wp += 1
                self._t_wp = d.time
                if self._wp >= len(TOUR):
                    self._round += 1
                    self._wp = 0
                    if self._round >= self.cfg.tour_rounds:
                        self._set_phase("wall")
        elif self.phase == "wall":                      # v9: 3 s pushing the hallway's north wall at 0.6 m/s
            r.drive_velocity((0.0, 0.6), 0.0, pose[2], dt=h)
            r.step()
            if d.time - self._t_wp >= 3.0:
                self._set_phase("back")
        elif self.phase == "back":
            r.drive_velocity((0.0, -0.4), 0.0, pose[2], dt=h)
            r.step()
            if d.time - self._t_wp >= 1.5:
                self._set_phase("done")

    def _set_phase(self, ph):
        self.phase = ph
        self._t_wp = self.d.time
        self._phase_changed = True

    # -- the step ----------------------------------------------------------------------------------
    def step(self) -> Events:
        c, d, r = self.cfg, self.d, self.r
        self._phase_changed = False
        if self.walker is not None:
            self.walker.step(d.time)
        self._drive()
        if self.firmware:
            rd = r.readings                             # SilRobot stepped its sensors; IMU and odometry go via the MCU
        else:
            rd = self.loc.step() if self.loc is not None else self.sn.step()
        ev = Events(t=d.time)
        if self._phase_changed:
            ev.phase = self.phase
        xy = r.pose()[:2]
        self.dist += float(np.linalg.norm(xy - self._last_xy))
        self._last_xy = xy.copy()
        if self.firmware:
            if rd.scan is not None:
                ev.scan = rd.scan
            if d.time >= self._next_truth - 1e-9:
                self._next_truth += 1 / c.truth_hz
                ev.truth = self._truth()
            return ev
        if rd.imu3 is not None:
            ev.imu = rd.imu3
        if rd.wheels is not None:
            _, vx, vy, wz, slip = rd.wheels
            self._tw_acc += (vx, vy, wz)
            self._tw_n += 1
            self._slip_max = max(self._slip_max, slip)
            # integrate the pose at the encoder rate (as the MCU does)
            o = self.odom
            cy, sy = math.cos(o[2]), math.sin(o[2])
            h = 1 / self.sn.cfg.wheel_rate
            o[0] += (cy * vx - sy * vy) * h
            o[1] += (sy * vx + cy * vy) * h
            o[2] = wrap_angle(o[2] + wz * h)
        if d.time >= self._next_odom - 1e-9 and self._tw_n:
            self._next_odom += 1 / c.odom_hz
            tw = self._tw_acc / self._tw_n
            sp = math.hypot(tw[0], tw[1])
            sv = 0.01 + 0.02 * sp + 3.0 * self._slip_max            # m/s, as estimator.EKF.update_wheels
            sw = 0.02 + 0.05 * abs(tw[2]) + 3.0 * self._slip_max     # rad/s
            T = 1 / c.odom_hz
            self.odom_var += np.array([(sv * T) ** 2, (sv * T) ** 2, (sw * T) ** 2])
            ev.odom = (d.time, tuple(self.odom), tuple(tw), self.odom_var.copy(), np.array([sv * sv, sv * sv, sw * sw]))
            self._tw_acc[:], self._tw_n, self._slip_max = 0.0, 0, 0.0
        if rd.scan is not None:
            ev.scan = rd.scan
        if d.time >= self._next_truth - 1e-9:
            self._next_truth += 1 / c.truth_hz
            ev.truth = self._truth()
            if self.loc is not None:
                ev.shadow = {k: f.pose() for k, f in self.loc.f.items()}
        return ev

    def _truth(self):
        d = self.d
        v = d.sensor("base_linvel").data
        yaw = self.r.pose()[2]
        cy, sy = math.cos(yaw), math.sin(yaw)
        wz = float(d.sensor("imu_gyro").data[2])
        return (d.time, tuple(d.xpos[self.base_id]), tuple(d.xquat[self.base_id]),
                (cy * v[0] + sy * v[1], -sy * v[0] + cy * v[1], wz))

    def world_from_odom(self):
        """The odom frame's pose in the world (x, y, yaw)."""
        return self.pose0.copy()


class Walker:
    """The person (params.person) walks in front of the robot's start: stands 1.6 m ahead for 8 s (time to be
    found), then out to the far point at 0.5 m/s, waits 3 s, back to 1.2 m, waits 3 s, and again; swaying up to 0.4 m
    to the side on the way (keeping 0.45 m from the walls). The far point stops 0.6 m short of the first wall straight
    ahead (at most 4.5 m)."""

    WAIT0, SPEED, NEAR, START_D = 8.0, 0.5, 1.2, 1.6

    def __init__(self, m, d, pose, base_id: int, t0: float = 0.0):
        import mujoco
        self.d, self.t0 = d, t0                         # t0: when it steps in (the Launcher's person, later on)
        self.mid = m.body("person").mocapid[0]
        self.x0, self.y0, self.yaw = float(pose[0]), float(pose[1]), float(pose[2])
        self.u = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        gid = np.array([-1], dtype=np.int32)
        hit = mujoco.mj_ray(m, d, np.array([self.x0, self.y0, 0.5]), np.array([*self.u, 0.0]),
                            np.array([1, 1, 0, 1, 0, 0], np.uint8), 1, base_id, gid)
        self.far = float(min(4.5, (hit if hit > 0 else 9.0) - 0.6))
        self.far = max(self.far, self.NEAR + 0.8)
        # the sway stays 0.45 m clear of the walls either side of the path (checked at its middle)
        mid = np.array([self.x0, self.y0]) + self.u * (self.NEAR + self.far) / 2
        side = []
        for sgn in (1, -1):
            n = np.array([-self.u[1], self.u[0]]) * sgn
            h = mujoco.mj_ray(m, d, np.array([*mid, 0.5]), np.array([*n, 0.0]), np.array([1, 1, 0, 1, 0, 0], np.uint8),
                              1, base_id, gid)
            side.append(h if h > 0 else 9.0)
        self.sway = float(np.clip(min(side) - 0.45, 0.0, 0.4))
        self.leg = (self.far - self.NEAR) / self.SPEED
        self.period = 2 * self.leg + 6.0

    def along(self, t: float) -> float:
        if t < self.WAIT0:
            return self.START_D
        tt = t - self.WAIT0
        first = (self.far - self.START_D) / self.SPEED
        if tt < first:
            return self.START_D + self.SPEED * tt
        tt = (tt - first) % self.period
        if tt < 3.0:
            return self.far
        tt -= 3.0
        if tt < self.leg:
            return self.far - self.SPEED * tt
        tt -= self.leg
        if tt < 3.0:
            return self.NEAR
        return self.NEAR + self.SPEED * (tt - 3.0)

    def step(self, t: float):
        t = t - self.t0
        a = self.along(t)
        side = self.sway * math.sin(2 * math.pi * t / 9.0) if t > self.WAIT0 else 0.0
        x = self.x0 + a * self.u[0] - side * self.u[1]
        y = self.y0 + a * self.u[1] + side * self.u[0]
        self.d.mocap_pos[self.mid] = (x, y, 0.0)
        c, s = math.cos((self.yaw + math.pi) / 2), math.sin((self.yaw + math.pi) / 2)     # facing the robot
        self.d.mocap_quat[self.mid] = (c, 0.0, 0.0, s)


def _mat2quat(R):
    """wxyz quaternion of a rotation matrix."""
    import mujoco
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R).reshape(-1))
    return q
