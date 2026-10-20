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

from models import Battery, DriveActuator, FlywheelDrive, GateServo
from omni_control import BaseController, OmniKinematics, world_to_body, yaw_from_quat
from params import SimParams
from robot_model import geometry, mass_properties
from traction import Base

def cal_file(p) -> Path:
    """Launcher calibration for this feed (the J-bend and kicker feeds launch differently)."""
    return Path(__file__).with_name("launcher_cal.json" if p.feed == "j_bend" else f"launcher_cal_{p.feed}.json")


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
        self.kin = OmniKinematics(p.wheel_angles_deg, p.wheel_base_radius, p.wheel_radius)
        self.ctl = BaseController(self.kin, max_speed=p.base_max_speed, max_accel=p.base_max_accel,
                                  max_jerk=p.base_max_jerk, max_yaw_rate=p.base_max_yaw_rate,
                                  accel_table=p.accel_table,
                                  max_wheel_speed=0.95 * p.motor.noload_rpm * 2 * math.pi / 60)
        self.drives = [DriveActuator(p.motor, p.drive_kp, p.drive_ki) for _ in range(nw)]
        self.fly = [FlywheelDrive(p.fly_motor, p.esc_spec) for _ in range(2)]
        self.gate = GateServo(speed=p.gate_slew)        # trapdoor (rad) or kicker plunger (m)
        self.tilt_servo = TiltServo(speed=p.tilt_speed, i_stall=3.0)
        self.battery = Battery(p.pack)
        self.cal = LauncherCal(p)

        self.a_wheel = [model.actuator(f"wheel{i}").id for i in range(1, nw + 1)]
        self.a_fly = [model.actuator(f"flywheel_{s}").id for s in "LR"]
        self.a_gate = model.actuator("gate").id
        self.a_tilt = model.actuator("tilt").id
        self.j_drive_v = [model.jnt_dofadr[model.joint(f"drive{i}").id] for i in range(1, nw + 1)]
        self.j_fly_v = [model.jnt_dofadr[model.joint(f"flywheel_{s}").id] for s in "LR"]
        self.exit_site = model.site("launch_exit").id
        self.funnel_site = model.site("funnel_center").id
        self.module = model.body("catch_module").id
        self.susp_k = p.roller_stiffness

        # load-aware allocation uses the traction model with the robot's own mass properties
        self.mass, cg = mass_properties(model, p)
        self.traction = Base(p.wheel_angles_deg, p.wheel_base_radius, float(cg[2]), float(cg[0]), float(cg[1]), p.mu)
        self.tau_ff = np.zeros(nw)

        self.wheel_cmd = np.zeros(nw)
        self.a_body_cmd = None             # set by studies that command wheels directly
        self.fly_target = 0.0
        # kicker feed: the pocket is open toward the nip, so idle flywheels brake to a stop instead of
        # coasting (a ball bouncing forward during a catch must meet stopped wheels)
        self.fly_brake = p.feed == "kicker"
        self.gate_open = False
        self.tilt_target = 0.0
        self.i_total = 0.0

    # -- sensors ------------------------------------------------------------
    def pose(self):
        pos = self.d.sensor("base_pos").data
        return np.array([pos[0], pos[1], yaw_from_quat(self.d.sensor("base_quat").data)])

    def wheel_speeds(self):
        return np.array([self.d.sensor(f"wheel{i}_speed").data[0] for i in range(1, self.nw + 1)])

    def wheel_loads(self):
        """Normal load on each wheel (N): roller-compliance spring plus the wheel's own weight (0 once it lifts)."""
        w = self.p.omni.mass * 9.81
        q = np.array([self.d.sensor(f"susp{i}").data[0] for i in range(1, self.nw + 1)])
        return np.where(q > 2e-6, self.susp_k * q + w, 0.0)

    def flywheel_speeds(self):
        return np.array([self.d.sensor(f"flywheel_{s}_speed").data[0] for s in "LR"])

    def tilt(self) -> float:
        return float(self.d.sensor("tilt_pos").data[0])

    def tilt_settled(self, tol=math.radians(1.0)) -> bool:
        return abs(self.tilt() - self.tilt_target) < tol and abs(self.tilt_servo.pos_cmd - self.tilt_target) < 1e-6

    def ball_held(self) -> bool:
        r = self.d.sensor("catch_beam").data[0]
        return 0 <= r < 2 * self.g.w

    def launch_exit_world(self) -> np.ndarray:
        return self.d.site_xpos[self.exit_site].copy()

    def funnel_center_world(self) -> np.ndarray:
        return self.d.site_xpos[self.funnel_site].copy()

    def flywheels_ready(self, tol=0.02) -> bool:
        w = self.flywheel_speeds()
        return self.fly_target > 0 and bool(np.all(np.abs(w - self.fly_target) < tol * self.fly_target))

    # -- commands -----------------------------------------------------------
    def drive_to(self, pose, target, v_ff=(0.0, 0.0), v_end: float = 0.0):
        self.wheel_cmd = self.ctl.goto_step(self.m.opt.timestep, pose, target, v_ff=v_ff, v_end=v_end)
        self.a_body_cmd = world_to_body(self.ctl.a_cmd, pose[2])

    def drive_velocity(self, v_world, w, yaw, a_cap=None):
        self.wheel_cmd = self.ctl.velocity_step(self.m.opt.timestep, v_world, w, yaw, a_cap=a_cap)
        self.a_body_cmd = world_to_body(self.ctl.a_cmd, yaw)

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

    # -- physics glue ---------------------------------------------------------
    def pre_step(self):
        dt, d, m = self.m.opt.timestep, self.d, self.m
        v_bus = self.battery.v
        self._allocate()
        for k, drv in enumerate(self.drives):
            ctrl, (lo, hi) = drv.command(self.wheel_cmd[k], d.qvel[self.j_drive_v[k]], v_bus, dt)
            d.ctrl[self.a_wheel[k]] = ctrl + self.tau_ff[k] / drv.kp      # force range still clips the total
            m.actuator_forcerange[self.a_wheel[k]] = (lo, hi)
        for k, fly in enumerate(self.fly):
            d.ctrl[self.a_fly[k]] = fly.step(self.fly_target, d.qvel[self.j_fly_v[k]], v_bus, dt, brake=self.fly_brake)
        d.ctrl[self.a_gate] = self.gate.step(self.p.gate_open_target if self.gate_open else 0.0, dt)
        lo, hi = self.p.tilt_limits
        d.ctrl[self.a_tilt] = self.tilt_servo.step(float(np.clip(self.tilt_target, lo, hi)), dt)

    def post_step(self):
        dt, d = self.m.opt.timestep, self.d
        v_bus = self.battery.v
        i = 0.0
        for k, drv in enumerate(self.drives):
            i += drv.account(d.actuator_force[self.a_wheel[k]], d.qvel[self.j_drive_v[k]], v_bus, dt)
        i += sum(f.i_bus for f in self.fly)
        moving = abs(self.gate.pos_cmd - (self.p.gate_open_target if self.gate_open else 0.0)) > 1e-3
        i += self.gate.bus_current(moving, v_bus)
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
