"""The robot between firmware and physics (v3).

Firmware-side code talks to Robot through sensors and four commands: wheel
speeds, flywheel speed, gate open/closed and module tilt. Every physics step:

  pre_step():  speed loops (+ load-aware torque allocation), tilt servo,
               torque/voltage/current limits -> MuJoCo controls
  mj_step()
  post_step(): applied torques -> phase and bus currents -> battery sag, heat
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np

import contact_model
import floor
from models import Battery, DriveActuator, FeedDrive, FlywheelDrive, GateServo
from omni_control import BaseController, OmniKinematics, SwerveKinematics, world_to_body, wrap_angle, yaw_from_quat
from params import SimParams
from robot_model import feed_kv, geometry, mass_properties, wheel_loads
from traction import Base

def cal_file(p, write: bool = False) -> Path:
    """Launcher calibration for this feed (the J-bend and kicker feeds launch differently). With viscoelastic contacts
    "launcher_cal_<feed>_ve.json", falling back to the elastic (v6 rounds 1-2) table if that feed has none."""
    base = "launcher_cal" if p.feed == "j_bend" else f"launcher_cal_{p.feed}"
    here = Path(__file__)
    if p.contact_model == "viscoelastic":
        ve = here.with_name(f"{base}_ve.json")
        if write or ve.exists():
            return ve
    return here.with_name(f"{base}.json")


class LauncherCal:
    """(flywheel speed, tilt) -> ball exit speed and elevation, measured by launcher_bench.py.

    Stored per tilt angle; between tilts the speed table is interpolated and the
    elevation offset (measured minus geometric) is interpolated too.
    """

    def __init__(self, p: SimParams):
        self.p = p
        r = p.flywheel_diameter / 2
        self.tilts = np.array([0.0])
        self.w = [np.array([100.0, 1000.0])]
        self.v = [self.w[0] * r * 0.7]
        self.d_elev = [np.zeros(2)]
        self.calibrated = False
        if cal_file(p).exists():
            cal = json.loads(cal_file(p).read_text())
            if (abs(cal.get("flywheel_diameter", 0) - p.flywheel_diameter) < 1e-6
                    and abs(cal.get("launch_angle_deg", 0) - p.launch_angle_deg) < 1e-6):
                rows = sorted(cal["tilts"], key=lambda t: t["tilt_deg"])
                self.tilts = np.radians([t["tilt_deg"] for t in rows])
                self.w = [np.array(t["w"]) for t in rows]
                self.v = [np.array(t["v"]) for t in rows]
                self.d_elev = [np.radians(np.array(t["elev_deg"])) - (math.radians(p.launch_angle_deg) - math.radians(t["tilt_deg"]))
                               for t in rows]
                self.calibrated = True

    def _blend(self, tilt: float, fn):
        if len(self.tilts) == 1:
            return fn(0)
        k = int(np.clip(np.searchsorted(self.tilts, tilt) - 1, 0, len(self.tilts) - 2))
        f = float(np.clip((tilt - self.tilts[k]) / (self.tilts[k + 1] - self.tilts[k]), 0.0, 1.0))
        return (1 - f) * fn(k) + f * fn(k + 1)

    def speed_for(self, v_exit: float, tilt: float) -> float:
        return self._blend(tilt, lambda k: float(np.interp(v_exit, self.v[k], self.w[k])))

    def elevation_at(self, v_exit: float, tilt: float) -> float:
        geo = math.radians(self.p.launch_angle_deg) - tilt
        return geo + self._blend(tilt, lambda k: float(np.interp(v_exit, self.v[k], self.d_elev[k])))

    def tilt_for(self, v_exit: float, elevation: float) -> float:
        """Tilt that gives this elevation at this exit speed (fixed point: the offset depends weakly on tilt)."""
        tilt = math.radians(self.p.launch_angle_deg) - elevation
        for _ in range(3):
            tilt = math.radians(self.p.launch_angle_deg) - elevation + \
                self._blend(tilt, lambda k: float(np.interp(v_exit, self.v[k], self.d_elev[k])))
        return tilt


class TiltServo(GateServo):
    """Position servo on the module pivot: slew-limited command; MuJoCo's position actuator does the rest."""


class Robot:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, p: SimParams):
        self.m, self.d, self.p = model, data, p
        self.g = geometry(p)
        nw = p.n_wheels
        self.nw = nw
        if p.is_swerve:        # v8: steered modules (design.md 26)
            self.kin = SwerveKinematics(p.wheel_angles_deg, p.wheel_base_radius, p.wheel_radius,
                                        math.radians(p.steer_range_deg), hold_speed=p.steer_hold_speed)
        else:
            self.kin = OmniKinematics(p.wheel_angles_deg, p.wheel_base_radius, p.wheel_radius,
                                      roller_angle_deg=45.0 if p.omni.kind == "mecanum" else 0.0)
        self.ctl = BaseController(self.kin, max_speed=p.base_max_speed, max_accel=p.base_max_accel,
                                  max_jerk=p.base_max_jerk, max_yaw_rate=p.base_max_yaw_rate,
                                  max_yaw_accel=p.base_max_yaw_accel, accel_table=p.accel_table,
                                  brake_margin=p.brake_margin,
                                  max_wheel_speed=p.wheel_speed_frac * p.motor.noload_rpm * 2 * math.pi / 60)
        # output-side back-EMF constant (V per output rad/s): the no-load speed at the live bus voltage
        self.ke_out = p.motor.ke * p.motor.gear
        self.drives = [DriveActuator(p.motor, p.drive_kp, p.drive_ki) for _ in range(nw)]
        self.fly = [FlywheelDrive(p.fly_motor, p.esc_spec) for _ in range(2)]
        self.gate = GateServo(speed=p.gate_slew)        # trapdoor (rad) or kicker plunger (m)
        self.tilt_servo = TiltServo(speed=p.tilt_speed, i_stall=3.0)
        self.battery = Battery(p.pack, soc=p.battery_soc)
        self.cal = LauncherCal(p)
        # physics side, not firmware: viscoelastic ball contacts and tire slides (None = elastic), stepped in pre_step
        self.contacts = contact_model.attach(model, data, p)
        self.floor = floor.attach(model, data, p)          # rolling resistance, varying wheel friction ([] = v6's floor)

        self.a_wheel = [model.actuator(f"wheel{i}").id for i in range(1, nw + 1)]
        self.a_fly = [model.actuator(f"flywheel_{s}").id for s in "LR"]
        # trigger: the trapdoor / plunger position servo, or (roller feed) the two feed-wheel gearmotors
        self.a_feed = [model.actuator(f"feed_{s}").id for s in "LR"] if p.feed_wheels else []
        self.a_gate = None if p.feed_wheels else model.actuator("gate").id
        # the feed gearmotors + drivers (catalog.FEED_MOTORS; "ideal" = v6 rounds 1-2's velocity actuators)
        self.feed_drives = [FeedDrive(p.feed_spec, feed_kv(p)) for _ in self.a_feed]
        self.j_feed = [(model.joint(f"feed_{s}").qposadr[0], model.jnt_dofadr[model.joint(f"feed_{s}").id])
                       for s in "LR"] if p.feed_wheels else []
        self.a_tilt = model.actuator("tilt").id
        self.a_sweep = model.actuator("sweeper").id if p.hopper in ("sweeper", "stirrer") else None
        # indexer (v6): position-controlled wheel; index_step() moves one pocket on
        self.a_index = model.actuator("indexer").id if p.hopper == "indexer" else None
        self.j_index = model.joint("indexer").qposadr[0] if p.hopper == "indexer" else None
        self.index_target = self.index_cmd = 0.0
        self.want_ball = False             # firmware: keep the launch pocket loaded from the indexer
        self.index_empty = False           # a whole turn brought nothing: the hopper is empty
        self.index_jams = 0
        self._ix_settled_t = None
        self._ix_steps_empty = 0
        self._ix_t_step = 0.0
        self._ix_backoff = False
        self.sweep = False                 # hopper sweeper arm running
        self.j_drive_v = [model.jnt_dofadr[model.joint(f"drive{i}").id] for i in range(1, nw + 1)]
        self.j_fly_v = [model.jnt_dofadr[model.joint(f"flywheel_{s}").id] for s in "LR"]
        self.exit_site = model.site("launch_exit").id
        self.funnel_site = model.site("funnel_center").id
        self.module = model.body("catch_module").id
        self.susp_k = p.roller_stiffness
        self._load_cache = {}
        # v7 terrain-aware firmware: the mapped edges (a perfect map from the floor features unless terrain_map is given)
        self.edges = [] if not p.terrain_assist else list(p.terrain_map if p.terrain_map is not None else floor.edges(p))
        self.terrain_yaw = None
        self._hold_yaw = None
        self._fw_div = max(1, int(round(0.01 / model.opt.timestep)))
        self.align_ok = True               # the game clears it while a catch is on (facing the ball comes first)
        # turn to meet only the square edges the wheels can't climb sideways (0.7 x the roller radius: v6 ~6 mm, the
        # 36 mm rollers ~13 mm)
        self.align_height = p.edge_align_height if p.edge_align_height is not None else 0.7 * p.omni.roller_diameter / 2
        if p.omni.kind == "tire" and p.edge_align_height is None:
            self.align_height = math.inf       # v8 swerve: every tyre already rolls straight at the edge
        self._terrain_cap = p.base_max_speed
        # v7 cross-axis pods: preloaded centring springs (applied as joint torque, like a pair of springs on stops)
        self.swing_dofs = np.array([model.jnt_dofadr[model.joint(f"swing{i}").id] for i in range(1, nw + 1)])             if p.swing != "none" else None
        self.swing_q = np.array([model.joint(f"swing{i}").qposadr[0] for i in range(1, nw + 1)])             if p.swing != "none" else None

        # load-aware allocation uses the traction model with the robot's own mass properties
        self.mass, cg = mass_properties(model, p)
        self.traction = Base(p.wheel_angles_deg, p.wheel_base_radius, float(cg[2]), float(cg[0]), float(cg[1]), p.mu)
        # v8 swerve: steering servos (slew-limited to the servo's speed through steer_gear) and, unless one is given, a
        # firmware acceleration table from this base's own limits: swerve_accel_frac of the lower of the quasi-static tip-over limit
        # and 0.8 mu g (every tyre can push along the acceleration), capped at swerve_accel_cap
        self.steer_servos, self.a_steer, self.j_steer = [], [], []
        if p.is_swerve:
            sv = p.steer_spec
            self.steer_servos = [GateServo(speed=sv.speed / p.steer_gear, i_stall=2.4, v=12.0) for _ in range(nw)]
            self.a_steer = [model.actuator(f"steer{i}").id for i in range(1, nw + 1)]
            self.j_steer = [model.joint(f"steer{i}").qposadr[0] for i in range(1, nw + 1)]
            if p.accel_table is None:
                dirs = np.arange(0.0, 181.0, 15.0)
                vals = []
                for a in dirs:
                    tips = [self.traction.tip_accel((math.cos(math.radians(s * a)), math.sin(math.radians(s * a))))
                            for s in (1, -1)]
                    vals.append(min(p.swerve_accel_cap, p.swerve_accel_frac * min(min(tips), 0.8 * p.mu * 9.81)))
                self.ctl.accel_table = (tuple(dirs), tuple(vals))
        self.tau_ff = np.zeros(nw)

        self.wheel_cmd = np.zeros(nw)
        self.a_body_cmd = None             # set by studies that command wheels directly
        self.fly_target = 0.0
        # kicker feed: the pocket is open toward the nip, so idle flywheels brake to a stop instead of
        # coasting (a ball bouncing forward during a catch must meet stopped wheels). The roller feed's stopped
        # feed wheels close the pocket, so its flywheels may coast or keep running between shots.
        self.fly_brake = p.feed == "kicker"
        self.gate_open = False
        self._feed_armed, self._feed_t, self._feed_clear = True, 0.0, 0.0
        self.tilt_target = 0.0
        self.i_total = 0.0

    # -- sensors ------------------------------------------------------------
    def pose(self):
        pos = self.d.sensor("base_pos").data
        return np.array([pos[0], pos[1], yaw_from_quat(self.d.sensor("base_quat").data)])

    def wheel_speeds(self):
        return np.array([self.d.sensor(f"wheel{i}_speed").data[0] for i in range(1, self.nw + 1)])

    def wheel_loads(self):
        """Normal load on each wheel (N), from the force each pod carries (v7: any suspension), 0 once it lifts."""
        return wheel_loads(self.m, self.d, self.p, self._load_cache)

    def flywheel_speeds(self):
        return np.array([self.d.sensor(f"flywheel_{s}_speed").data[0] for s in "LR"])

    def tilt(self) -> float:
        return float(self.d.sensor("tilt_pos").data[0])

    def tilt_settled(self, tol=math.radians(1.0)) -> bool:
        return abs(self.tilt() - self.tilt_target) < tol and abs(self.tilt_servo.pos_cmd - self.tilt_target) < 1e-6

    def feed_exit_blocked(self) -> bool:
        """Chute: the beam where the ball leaves the feed wheels sees a ball."""
        r = self.d.sensor("feed_exit_beam").data[0]
        return 0 <= r < 2 * self.g.w

    def ball_held(self) -> bool:
        r = self.d.sensor("catch_beam").data[0]
        return 0 <= r < 2 * self.g.w

    def ball_aboard(self, body: str = "ball") -> bool:
        """The ball is caught: in the launch pocket (break-beam) or, with the indexer, in one of its wheel pockets
        (on hardware: a beam per pocket station or a weight/IR check; here the ball's position)."""
        if self.ball_held() or self.p.hopper != "indexer":
            return self.ball_held()
        mb = self.module
        q = self.d.xmat[mb].reshape(3, 3).T @ (self.d.xpos[self.m.body(body).id] - self.d.xpos[mb])
        z0 = self.g.tube_top + self.p.indexer_plate
        return bool(math.hypot(q[0], q[1]) < self.p.funnel_throat_radius and z0 < q[2] < self.g.throat_z
                    and float(np.linalg.norm(self.d.sensor("ball_vel").data - self.d.sensor("base_linvel").data)) < 0.5)

    def launch_exit_world(self) -> np.ndarray:
        return self.d.site_xpos[self.exit_site].copy()

    def funnel_center_world(self) -> np.ndarray:
        return self.d.site_xpos[self.funnel_site].copy()

    def index_step(self, n: int = 1):
        """Indexer: turn the wheel n pockets on (the next pocket over the exit hole)."""
        self.index_target += n * 2 * math.pi / self.p.indexer_pockets
        self._ix_t_step = self.d.time

    def index_angle(self) -> float:
        return float(self.d.qpos[self.j_index]) if self.j_index is not None else 0.0

    def index_settled(self, tol=math.radians(3.0)) -> bool:
        return self.j_index is None or (abs(self.index_cmd - self.index_target) < 1e-9 and
                                        abs(self.index_angle() - self.index_target) < tol)

    def _index_service(self):
        """Indexer firmware: while want_ball and the launch pocket is empty, move one pocket over the exit, then dwell
        long enough for a ball to drop through (stepping on early shears a falling ball in the exit hole). A stalled
        wheel backs off a quarter step and tries again; a whole turn with nothing dropping means the hopper is empty."""
        t, p = self.d.time, self.p
        if self.ball_held():
            self._ix_steps_empty, self.index_empty = 0, False
        settled = self.index_settled()
        if settled:
            self._ix_settled_t = self._ix_settled_t if self._ix_settled_t is not None else t
            self._ix_backoff = False
        else:
            self._ix_settled_t = None
            if t - self._ix_t_step > p.indexer_jam_time and not self._ix_backoff:
                # jammed: back off a quarter pocket, then go for the same pocket again
                self.index_jams += 1
                self._ix_backoff = True
                self._ix_t_step = t
                self.index_cmd -= 0.25 * 2 * math.pi / p.indexer_pockets
                return
            if self._ix_backoff and t - self._ix_t_step > p.indexer_jam_time:
                self._ix_backoff = False
                self._ix_t_step = t
        if not self.want_ball or self.ball_held() or self.index_empty or not settled:
            return
        if t - self._ix_settled_t < p.indexer_dwell:
            return
        if self._ix_steps_empty >= 2 * p.indexer_pockets:      # two turns with nothing dropping in
            self.index_empty = True
            return
        self.index_step()
        self._ix_steps_empty += 1
        self._ix_t_step = t

    def trigger_done(self) -> bool:
        """Roller feed: this trigger's ball has been passed on (the feed wheels stopped); others: never early."""
        return bool(self.a_feed) and self.gate_open and not self._feed_armed

    def flywheels_ready(self, tol=0.02) -> bool:
        w = self.flywheel_speeds()
        return self.fly_target > 0 and bool(np.all(np.abs(w - self.fly_target) < tol * self.fly_target))

    # -- commands -----------------------------------------------------------
    def drive_to(self, pose, target, v_ff=(0.0, 0.0), v_end: float = 0.0):
        if self.terrain_yaw is not None and self.align_ok:   # v7: crossing a square edge, hold an aligned heading
            target = (target[0], target[1], self.terrain_yaw)
        self.wheel_cmd = self.ctl.goto_step(self.m.opt.timestep, pose, target, v_ff=v_ff, v_end=v_end)
        self.a_body_cmd = world_to_body(self.ctl.a_cmd, pose[2])

    def drive_velocity(self, v_world, w, yaw, a_cap=None):
        """w = None (v7): the firmware picks the yaw rate: hold the heading, or turn to cross a square edge aligned."""
        if w is None:
            if self._hold_yaw is None:
                self._hold_yaw = yaw
            if self.terrain_yaw is not None:
                self._hold_yaw = self.terrain_yaw
            w = float(np.clip(6.0 * wrap_angle(self._hold_yaw - yaw), -self.ctl.max_yaw_rate, self.ctl.max_yaw_rate))
        self.wheel_cmd = self.ctl.velocity_step(self.m.opt.timestep, v_world, w, yaw, a_cap=a_cap)
        self.a_body_cmd = world_to_body(self.ctl.a_cmd, yaw)

    # -- v7: terrain-aware firmware ----------------------------------------------
    def edge_speed(self, height: float) -> float:
        for h, v in self.p.edge_speeds:
            if height <= h:
                return v
        return 0.5

    def _terrain(self):
        """Mapped floor edges (design.md 25): cap the speed so the base meets an edge no faster than edge_speed(height),
        and when the path is about to cross a square edge (or is crossing it) turn the base so the edge's normal sits at
        a multiple of 90 deg in the body frame: every X4 wheel then meets it at 45 deg, half rolling, and none slides
        into it along its axle on its small rollers."""
        ctl, p = self.ctl, self.p
        self._fw_tick = getattr(self, "_fw_tick", 0) + 1
        if self._fw_tick % self._fw_div:                   # the firmware runs this at 100 Hz, not every physics step
            ctl.max_speed = self._terrain_cap
            return
        if not hasattr(self, "_E"):
            E = np.array([e[:6] for e in self.edges], dtype=float)
            self._E = dict(a=E[:, :2], ab=E[:, 2:4] - E[:, :2], h=E[:, 4], sq=(E[:, 5] > 0.5) & (E[:, 4] > self.align_height),
                           vz=np.array([self.edge_speed(h) for h in E[:, 4]]),
                           n_ang=np.arctan2(E[:, 3] - E[:, 1], E[:, 2] - E[:, 0]) + math.pi / 2)
        E = self._E
        pose = self.pose()
        pos = pose[:2]
        cap = ctl.max_speed if p.slow_zones else p.base_max_speed
        v = ctl.v_cmd
        sp = float(np.linalg.norm(v))
        u = v / sp if sp > 0.05 else np.zeros(2)
        reach0 = p.wheel_base_radius + p.wheel_radius + 0.03      # the outline of the wheels
        a = p.brake_margin * ctl.worst_accel()

        def dist(pts):                                          # (k points) x (n edges)
            rel = pts[:, None, :] - E["a"][None]
            t = np.clip((rel * E["ab"][None]).sum(-1) / np.maximum((E["ab"] ** 2).sum(-1), 1e-12)[None], 0.0, 1.0)
            return np.linalg.norm(rel - t[..., None] * E["ab"][None], axis=-1)
        d_now = dist(pos[None])[0]
        # along the path: as far as the braking distance down to each edge's speed (the longest of them), + 0.1 m
        reach = float(np.max(np.maximum(0.0, (sp * sp - E["vz"] ** 2) / (2 * a)))) + 0.1
        s = np.linspace(0.0, max(reach, sp * 0.5 + 0.1), 14)
        dpath = dist(pos + np.outer(s, u))                      # (14, n)
        brake_d = np.maximum(0.0, (sp * sp - E["vz"] ** 2) / (2 * a)) + 0.1
        near = (d_now < reach0) | np.any((dpath < reach0) & (s[:, None] <= brake_d[None]), axis=0)
        if np.any(near):
            cap = min(cap, float(np.min(E["vz"][near])))
        want = None
        if p.edge_align and sp > 0.05:
            soon = E["sq"] & ((d_now < reach0) | np.any((dpath < reach0) & (s[:, None] <= sp * 0.5 + 0.1), axis=0))
            if np.any(soon):
                k = int(np.argmin(np.where(soon, d_now, np.inf)))
                n_ang = float(E["n_ang"][k])
                q = round(wrap_angle(pose[2] - n_ang) / (math.pi / 2))
                want = wrap_angle(n_ang + q * math.pi / 2)
        ctl.max_speed = cap
        self._terrain_cap = cap
        self.terrain_yaw = want

    # -- allocation -------------------------------------------------------------
    def _allocate(self):
        """Null-space wheel torques that shift traction onto the loaded wheels (4+ wheels).

        Plain speed loops share the force like a pseudo-inverse. The firmware knows
        the commanded acceleration, so it can predict the load transfer and add
        torques that cancel in the net force and yaw torque but move the work to
        the wheels with the most grip.
        """
        self.tau_ff[:] = 0.0
        if self.p.allocation != "load_aware" or self.nw <= 3 or self.a_body_cmd is None:
            return
        a = float(np.linalg.norm(self.a_body_cmd))
        if a < 1e-3:
            return
        u = self.a_body_cmd / a
        b = self.traction
        f = b.forces(a, u, "optimal")
        f_null = f - b.G_pinv @ (b.G @ f)          # remove the part the speed loops already supply
        self.tau_ff[:] = f_null * self.mass * self.p.wheel_radius

    def _slow_zones(self):
        """Firmware speed cap from the mapped slow zones (v6 round 3): if a zone lies within the base's braking
        distance (plus its radius) along its current velocity, cap the speed at that zone's limit."""
        ctl, p = self.ctl, self.p
        v = ctl.v_cmd
        sp = float(np.linalg.norm(v))
        cap = p.base_max_speed
        pos = self.pose()[:2]
        u = v / sp if sp > 1e-6 else np.zeros(2)
        a = p.brake_margin * ctl.worst_accel()
        for x0, y0, x1, y1, vz in p.slow_zones:
            reach = max(0.0, (sp * sp - vz * vz) / (2 * a)) + 0.25
            pts = pos + np.outer(np.linspace(0.0, reach, 12), u)
            if np.any((pts[:, 0] >= min(x0, x1) - 0.25) & (pts[:, 0] <= max(x0, x1) + 0.25) &
                      (pts[:, 1] >= min(y0, y1) - 0.25) & (pts[:, 1] <= max(y0, y1) + 0.25)):
                cap = min(cap, vz)
        ctl.max_speed = cap

    # -- physics glue ---------------------------------------------------------
    def pre_step(self):
        dt, d, m = self.m.opt.timestep, self.d, self.m
        if self.contacts is not None:
            self.contacts.pre_step()
        for hook in self.floor:
            hook.pre_step()
        if self.swing_dofs is not None:
            d.qfrc_applied[self.swing_dofs] = -self.p.swing_preload * np.clip(d.qpos[self.swing_q] / 0.005, -1.0, 1.0)
        v_bus = self.battery.v
        if self.p.live_wheel_limit:
            # the controller's wheel-speed ceiling follows the battery: no-load speed at the bus voltage now
            self.ctl.max_wheel_speed = self.p.wheel_speed_frac * v_bus / self.ke_out
        if self.p.slow_zones:
            self._slow_zones()
        if self.edges:
            self._terrain()
        self._allocate()
        cmd = self.wheel_cmd
        if self.steer_servos:
            # v8 swerve: point each module at its target; while it turns, scale its wheel by cos(error) so a module
            # pointing the wrong way doesn't push
            meas = np.array([d.qpos[j] for j in self.j_steer])
            self.kin.steer = meas
            for k, sv in enumerate(self.steer_servos):
                d.ctrl[self.a_steer[k]] = sv.step(float(self.kin.target[k]), dt)
            if self.p.steer_cos_scale:
                cmd = cmd * np.maximum(0.0, np.cos(self.kin.target - meas))
        for k, drv in enumerate(self.drives):
            ctrl, (lo, hi) = drv.command(cmd[k], d.qvel[self.j_drive_v[k]], v_bus, dt)
            d.ctrl[self.a_wheel[k]] = ctrl + self.tau_ff[k] / drv.kp      # force range still clips the total
            m.actuator_forcerange[self.a_wheel[k]] = (lo, hi)
        for k, fly in enumerate(self.fly):
            d.ctrl[self.a_fly[k]] = fly.step(self.fly_target, d.qvel[self.j_fly_v[k]], v_bus, dt, brake=self.fly_brake)
        open_ = self.gate_open
        target = self.p.gate_open_target
        if self.a_feed:
            # roller feed, one ball per trigger: (optionally) seat the ball on the back stop by running backward for
            # feed_seat_time, then turn until the ball has gone feed_clear_travel past the beam (or feed_time), then
            # hold until the trigger is released and pressed again
            if not open_:
                self._feed_armed, self._feed_t, self._feed_clear = True, 0.0, 0.0
            elif self._feed_armed:
                self._feed_t += dt
                if self._feed_t <= self.p.feed_seat_time:
                    target = -0.5 * target
                elif self.p.feed == "chute":
                    # chute: stop once the ball has passed the beam at the feed wheels' exit (blocked, then clear),
                    # so it always leaves them at full speed and the next ball waits for its own trigger
                    if self.feed_exit_blocked():
                        self._feed_clear = 1.0
                    elif self._feed_clear > 0.5 or self._feed_t > self.p.feed_time:
                        self._feed_armed = False
                else:
                    self._feed_clear = 0.0 if self.ball_held() else self._feed_clear + dt
                    if self._feed_clear > self.p.feed_clear_travel / self.p.feed_speed or \
                            self._feed_t > self.p.feed_time + self.p.feed_seat_time:
                        self._feed_armed = False
            open_ = open_ and self._feed_armed
        g_cmd = self.gate.step(target if open_ else 0.0, dt)
        if self.a_gate is not None:
            d.ctrl[self.a_gate] = g_cmd
        for a, fd, (jq, jv) in zip(self.a_feed, self.feed_drives, self.j_feed):
            # both hinge axes are signed so + feeds the ball forward; 0 = hold the wheel where it stopped
            ctrl, (lo, hi) = fd.command(g_cmd, float(d.qpos[jq]), float(d.qvel[jv]), dt)
            d.ctrl[a] = ctrl
            m.actuator_forcerange[a] = (lo, hi)
        lo, hi = self.p.tilt_limits
        d.ctrl[self.a_tilt] = self.tilt_servo.step(float(np.clip(self.tilt_target, lo, hi)), dt)
        if self.a_index is not None:
            self._index_service()
            step = self.p.indexer_speed * dt
            self.index_cmd += float(np.clip(self.index_target - self.index_cmd, -step, step))
            d.ctrl[self.a_index] = self.index_cmd
        if self.a_sweep is not None:
            d.ctrl[self.a_sweep] = self.p.sweeper_speed * float(self.sweep)     # True/1 forward, -1 reverse

    def post_step(self):
        dt, d = self.m.opt.timestep, self.d
        v_bus = self.battery.v
        i = 0.0
        for k, drv in enumerate(self.drives):
            i += drv.account(d.actuator_force[self.a_wheel[k]], d.qvel[self.j_drive_v[k]], v_bus, dt)
        i += sum(f.i_bus for f in self.fly)
        if self.a_feed:
            # feed motors on the 12 V rail through a DC-DC converter (~90%)
            fm = self.p.feed_spec
            for a, fd, (jq, jv) in zip(self.a_feed, self.feed_drives, self.j_feed):
                i_ph = fd.account(d.actuator_force[a], d.qvel[jv], dt)
                i += i_ph * fm.v_supply / v_bus / 0.9 * min(1.0, (fm.r * i_ph + fm.ke * abs(d.qvel[jv])) / fm.v_supply)
        else:
            moving = abs(self.gate.pos_cmd - (self.p.gate_open_target if self.gate_open else 0.0)) > 1e-3
            i += self.gate.bus_current(moving, v_bus)
        for k, sv in enumerate(self.steer_servos):          # v8: steering servos on the 12 V rail
            i += sv.bus_current(abs(sv.pos_cmd - float(self.kin.target[k])) > 1e-3, v_bus)
        tilt_moving = abs(self.tilt_servo.pos_cmd - self.tilt_target) > 1e-3
        i += self.tilt_servo.bus_current(tilt_moving, v_bus)
        i += self.p.electronics_power / v_bus
        self.i_total = i
        self.battery.step(i, dt)

    def step(self):
        self.pre_step()
        mujoco.mj_step(self.m, self.d)
        self.post_step()

    def telemetry(self) -> dict:
        return dict(v_bus=self.battery.v, i_bus=self.i_total, soc=self.battery.soc,
                    t_motor=max(dr.thermal.t_w for dr in self.drives),
                    fly_rpm=float(np.mean(self.flywheel_speeds())) * 60 / (2 * math.pi),
                    tilt_deg=math.degrees(self.tilt()))
