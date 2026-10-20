"""The robot between firmware and physics.

Firmware-side code (behaviors, controllers) talks to Robot through sensors and
three commands: wheel speeds, flywheel speed, gate open/closed. Robot runs the
electromechanical models every physics step:

  pre_step():  speed loops, torque/voltage/current limits -> MuJoCo controls
  mj_step()
  post_step(): applied torques -> phase and bus currents -> battery sag, heat
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np

from kiwi_control import BaseController, KiwiKinematics, yaw_from_quat
from models import Battery, DriveActuator, FlywheelDrive, GateServo
from params import SimParams
from robot_model import geometry

CAL_FILE = Path(__file__).with_name("launcher_cal.json")


class LauncherCal:
    """Flywheel speed -> ball exit speed and elevation, measured by studies/launcher_bench.py."""

    def __init__(self, p: SimParams):
        self.w = np.array([100.0, 1000.0])
        r = p.flywheel_diameter / 2
        self.v = self.w * r * 0.85                          # placeholder until calibrated
        self.elev = np.full(2, math.radians(p.launch_angle_deg))
        if CAL_FILE.exists():
            cal = json.loads(CAL_FILE.read_text())
            if abs(cal.get("flywheel_diameter", 0) - p.flywheel_diameter) < 1e-6:
                self.w, self.v = np.array(cal["w"]), np.array(cal["v"])
                self.elev = np.radians(np.array(cal["elev_deg"]))

    def speed_for(self, v_exit: float) -> float:
        return float(np.interp(v_exit, self.v, self.w))

    def elevation_at(self, v_exit: float) -> float:
        return float(np.interp(v_exit, self.v, self.elev))


class Robot:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, p: SimParams):
        self.m, self.d, self.p = model, data, p
        self.g = geometry(p)
        self.kin = KiwiKinematics(p.wheel_angles_deg, p.wheel_base_radius, p.wheel_radius)
        self.ctl = BaseController(self.kin, max_speed=p.base_max_speed, max_accel=p.base_max_accel,
                                  max_yaw_rate=p.base_max_yaw_rate,
                                  max_wheel_speed=0.95 * p.motor.noload_rpm * 2 * math.pi / 60)
        self.drives = [DriveActuator(p.motor, p.drive_kp, p.drive_ki) for _ in range(3)]
        self.fly = [FlywheelDrive(p.fly_motor, p.esc_spec) for _ in range(2)]
        self.gate = GateServo(speed=p.gate_speed)
        self.battery = Battery(p.pack)
        self.cal = LauncherCal(p)

        self.a_wheel = [model.actuator(f"wheel{i}").id for i in (1, 2, 3)]
        self.a_fly = [model.actuator(f"flywheel_{s}").id for s in "LR"]
        self.a_gate = model.actuator("gate").id
        self.j_drive_v = [model.jnt_dofadr[model.joint(f"drive{i}").id] for i in (1, 2, 3)]
        self.j_fly_v = [model.jnt_dofadr[model.joint(f"flywheel_{s}").id] for s in "LR"]
        self.exit_site = model.site("launch_exit").id
        self.module = model.body("catch_module").id

        self.wheel_cmd = np.zeros(3)
        self.fly_target = 0.0
        self.gate_open = False
        self.i_total = 0.0

    # -- sensors ------------------------------------------------------------
    def pose(self):
        pos = self.d.sensor("base_pos").data
        return np.array([pos[0], pos[1], yaw_from_quat(self.d.sensor("base_quat").data)])

    def wheel_speeds(self):
        return np.array([self.d.sensor(f"wheel{i}_speed").data[0] for i in (1, 2, 3)])

    def flywheel_speeds(self):
        return np.array([self.d.sensor(f"flywheel_{s}_speed").data[0] for s in "LR"])

    def ball_held(self) -> bool:
        r = self.d.sensor("catch_beam").data[0]
        return 0 <= r < 2 * self.g.w

    def launch_exit_world(self) -> np.ndarray:
        return self.d.site_xpos[self.exit_site].copy()

    def flywheels_ready(self, tol=0.02) -> bool:
        w = self.flywheel_speeds()
        return self.fly_target > 0 and bool(np.all(np.abs(w - self.fly_target) < tol * self.fly_target))

    # -- commands -----------------------------------------------------------
    def drive_to(self, pose, target, v_ff=(0.0, 0.0)):
        self.wheel_cmd = self.ctl.goto_step(self.m.opt.timestep, pose, target, v_ff=v_ff)

    def drive_velocity(self, v_world, w, yaw):
        self.wheel_cmd = self.ctl.velocity_step(self.m.opt.timestep, v_world, w, yaw)

    # -- physics glue ---------------------------------------------------------
    def pre_step(self):
        dt, d, m = self.m.opt.timestep, self.d, self.m
        v_bus = self.battery.v
        for k, drv in enumerate(self.drives):
            ctrl, (lo, hi) = drv.command(self.wheel_cmd[k], d.qvel[self.j_drive_v[k]], v_bus, dt)
            d.ctrl[self.a_wheel[k]] = ctrl
            m.actuator_forcerange[self.a_wheel[k]] = (lo, hi)
        for k, fly in enumerate(self.fly):
            d.ctrl[self.a_fly[k]] = fly.step(self.fly_target, d.qvel[self.j_fly_v[k]], v_bus, dt)
        d.ctrl[self.a_gate] = self.gate.step(-self.p.gate_open_angle if self.gate_open else 0.0, dt)

    def post_step(self):
        dt, d = self.m.opt.timestep, self.d
        v_bus = self.battery.v
        i = 0.0
        for k, drv in enumerate(self.drives):
            i += drv.account(d.actuator_force[self.a_wheel[k]], d.qvel[self.j_drive_v[k]], v_bus, dt)
        i += sum(f.i_bus for f in self.fly)
        moving = abs(self.gate.pos_cmd - (-self.p.gate_open_angle if self.gate_open else 0.0)) > 1e-3
        i += self.gate.bus_current(moving, v_bus)
        i += self.p.electronics_power / v_bus
        self.i_total = i
        self.battery.step(i, dt)

    def step(self):
        """pre_step + mj_step + post_step, for callers that own no other logic."""
        self.pre_step()
        mujoco.mj_step(self.m, self.d)
        self.post_step()

    def telemetry(self) -> dict:
        return dict(v_bus=self.battery.v, i_bus=self.i_total, soc=self.battery.soc,
                    t_motor=max(dr.thermal.t_w for dr in self.drives),
                    fly_rpm=float(np.mean(self.flywheel_speeds())) * 60 / (2 * math.pi))
