"""v11 (design.md 30): the robot with its real firmware in the loop (software in the loop, SIL).

robot.Robot ran the firmware's logic in Python every physics step. SilRobot keeps the physics and the parts' own
behaviour, and hands every decision the MCU makes to the C++ firmware core (firmware/lib/omni_fw, loaded through
firmware/sil/omni_fw_sil.py), ticked every 1 ms of sim time like the Teensy's loop:

  sim -> firmware (FwHwInputs)                         firmware -> sim (FwHwOutputs)
  CAN frames from the four DM-2325s (DmMotor below)    CAN frames to them: enable / disable / clear, speed setpoints
  BNO085 reports (sensors.py, 400 Hz)                  ESC throttles (bidirectional DShot)
  ESC eRPM telemetry, quantized as DShot's period      feed-wheel speed commands (RoboClaw), 0 = position hold
  feed-wheel speeds (RoboClaw, 100 Hz est)             tilt servo goal (STS3250)
  tilt servo position (STS3250, 4096 counts/turn)
  the two break-beams, e-stop, bumper, battery volts

The parts' own loops stay models, run every physics step: the DM-2325's speed loop (models.DriveActuator), the ESC's
duty cycle and current limit (EscMotor), the RoboClaw's speed / hold loop (models.FeedDrive), the servo's slew.
What's new in the parts: the DM-2325's CAN port (enable state, latched errors, one feedback frame per command with
12-bit velocity / 16-bit position, a comm-loss timeout) and DShot's eRPM resolution.

The Jetson side talks to the firmware only through bytes (fw.rx / fw.tx): see sil_link.py and the ROS bridge.
"""

from __future__ import annotations

import math
import os
import struct
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
FW_DIR = Path(os.environ.get("OMNI_FIRMWARE_DIR", HERE.parents[1] / "firmware"))
sys.path.insert(0, str(FW_DIR / "sil"))

from models import FlywheelDrive, _clip  # noqa: E402
from omni_fw_sil import CanFrame, Firmware, param  # noqa: E402
from robot import Robot  # noqa: E402
from sensors import Sensors, SensorConfig  # noqa: E402


def _f2u(x, lo, hi, bits):
    x = min(max(x, lo), hi)
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo) + 0.5)


class DmMotor:
    """A Damiao DM-2325 as its CAN port behaves (firmware/lib/omni_fw/dm_motor.h has the protocol; this is an
    independent implementation of the motor's side). Velocity mode. Starts disabled; a latched error (8-E) needs a
    clear before it enables again. Each frame addressed to it gets one feedback reply. With comm_timeout > 0, no frame
    for that long disables it with error D (Damiao's "CAN timeout" setting; its default and behaviour are VERIFY)."""

    def __init__(self, motor_id: int, master_base: int, ranges, comm_timeout: float = 0.1):
        self.id, self.master = motor_id, master_base + motor_id
        self.p_max, self.v_max, self.t_max = ranges
        self.comm_timeout = comm_timeout
        self.enabled = False
        self.err = 0
        self.setpoint = 0.0
        self.t_rx = -math.inf
        self.n_frames = 0

    def on_frame(self, f: CanFrame, t: float, pos: float, vel: float, tau: float, t_coil: float):
        if f.id != 0x200 + self.id:
            return None
        data = bytes(f.data[:f.len])
        self.t_rx = t
        self.n_frames += 1
        if f.len == 8 and data[:7] == b"\xff" * 7:
            code = data[7]
            if code == 0xFC and self.err in (0, 1):
                self.enabled, self.err = True, 1
            elif code == 0xFD:
                self.enabled = False
                self.err = 0 if self.err in (0, 1) else self.err
            elif code == 0xFB and self.err >= 8:
                self.enabled, self.err = False, 0
        elif f.len == 4:
            self.setpoint = struct.unpack("<f", data)[0]
        return self.feedback(pos, vel, tau, t_coil)

    def step(self, t: float):
        if self.enabled and self.comm_timeout > 0 and t - self.t_rx > self.comm_timeout:
            self.enabled, self.err = False, 0xD

    def feedback(self, pos, vel, tau, t_coil) -> CanFrame:
        pos = (pos + self.p_max) % (2 * self.p_max) - self.p_max
        p = _f2u(pos, -self.p_max, self.p_max, 16)
        v = _f2u(vel, -self.v_max, self.v_max, 12)
        q = _f2u(tau, -self.t_max, self.t_max, 12)
        f = CanFrame()
        f.id, f.len = self.master, 8
        d = (self.id | self.err << 4, p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (q >> 8), q & 0xFF,
             int(min(255, max(0, t_coil))), int(min(255, max(0, t_coil))))
        for i, b in enumerate(d):
            f.data[i] = b
        return f


class EscMotor(FlywheelDrive):
    """The flywheel motor on its AM32 ESC, driven by the MCU's throttle (the rpm loop moved into the firmware).
    Telemetry: bidirectional DShot sends the electrical period in microseconds as a 9-bit mantissa and 3-bit exponent,
    so eRPM comes in steps of ~0.2%."""

    def step_throttle(self, throttle: float, w_true: float, v_bus: float, dt: float, brake: bool) -> float:
        self.throttle = throttle
        i = (throttle * v_bus - self.ke * w_true) / self.m.r_m
        if throttle <= 0 and not brake:
            i = 0.0                                   # DShot 0: disarmed, the wheel coasts
        i = _clip(i, -self.esc.current_limit, self.esc.current_limit)
        duty = _clip((self.ke * w_true + self.m.r_m * i) / v_bus, 0.0, 1.0)
        self.i_phase = i
        self.i_bus = duty * i / 0.95 if i > 0 else duty * i * 0.95
        self.e_loss += self.m.r_m * i * i * dt
        return self.m.kt * i - self.friction(w_true)

    @staticmethod
    def erpm_telemetry(w: float, pole_pairs: int) -> float:
        erpm = abs(w) * pole_pairs * 60 / (2 * math.pi)
        if erpm < 100:
            return 0.0
        period = 60e6 / erpm                          # us per electrical revolution
        e = max(0, int(math.floor(math.log2(period))) - 8)
        period = round(period / (1 << e)) * (1 << e)  # 9 significant bits
        return 60e6 / period


class SilRobot(Robot):
    """Robot whose MCU decisions come from the C++ firmware. fw_div: physics steps per firmware tick (2 at 0.5 ms)."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, p, *, seed: int = 0, lidar: bool = False,
                 boot_offset: float = 12.345, dm_comm_timeout: float = 0.1, sensor_kw: dict | None = None):
        super().__init__(model, data, p)
        if p.n_wheels != 4 or p.is_swerve:
            raise ValueError("the firmware core is built for the X4 base")
        self.fw = Firmware()
        self.fw_div = max(1, int(round(0.001 / model.opt.timestep)))
        self.boot_offset = boot_offset            # s: the MCU booted this long before sim time 0
        sc = SensorConfig.parts("BNO085", "A2M12", seed=seed, flow=False, **(sensor_kw or {}))
        sc.lidar = lidar                          # the catch scene needs no scans
        self.sn = Sensors(self, sc)
        rng = (param("DM_PMAX"), param("DM_VMAX"), param("DM_TMAX"))
        self.dm = [DmMotor(i + 1, int(param("DM_MASTER_BASE")), rng, dm_comm_timeout) for i in range(self.nw)]
        self.j_drive_q = [model.jnt_qposadr[model.joint(f"drive{i}").id] for i in range(1, self.nw + 1)]
        self.fly = [EscMotor(p.fly_motor, p.esc_spec) for _ in range(2)]
        self.pole_pairs = int(param("FLY_POLE_PAIRS"))
        self.rng = np.random.default_rng(50_000 + seed)
        self.can_rx = []                          # replies waiting for the next tick
        self.throttle = [0.0, 0.0]
        self.fly_brake_cmd = False
        self.feed_cmd = 0.0
        self.tilt_cmd = 0.0
        self.estop = False
        self.bumper = False
        self.imu_pending = None
        self.readings = None                      # the last Sensors.step() (scans for the bridge)
        self._k = 0
        self._t_feed = self._t_tilt = -1.0
        self.n_ticks = 0
        self.fw_out = None
        self.can_cut_until = -1.0                 # fault injection: CAN1 is cut until this sim time (tests)

    # -- the firmware tick -------------------------------------------------------------------------------------
    def mcu_time_us(self) -> int:
        return int(round((self.d.time + self.boot_offset) * 1e6))

    def _fw_tick(self):
        d, inp = self.d, self.fw.inputs
        t = d.time
        inp.t_us = self.mcu_time_us()
        n = min(len(self.can_rx), 16)
        for i in range(n):
            inp.can_rx[i] = self.can_rx[i]
        inp.n_can_rx = n
        self.can_rx = self.can_rx[n:]
        if self.imu_pending is not None:
            _, q, g, a = self.imu_pending
            inp.imu_new = 1
            for i in range(4):
                inp.imu_quat[i] = q[i]
            for i in range(3):
                inp.imu_gyro[i] = g[i]
                inp.imu_accel[i] = a[i]
            self.imu_pending = None
        else:
            inp.imu_new = 0
        w_fly = self.flywheel_speeds()
        for k in range(2):                       # telemetry every 0.5 ms: always a fresh frame at the 1 ms tick
            inp.fly_new[k] = 1
            inp.fly_erpm[k] = EscMotor.erpm_telemetry(w_fly[k], self.pole_pairs)
        inp.feed_new = 0
        if t - self._t_feed >= 0.01 - 1e-9 and self.j_feed:       # RoboClaw speed read at 100 Hz (est)
            self._t_feed = t
            inp.feed_new = 1
            for k, (_, jv) in enumerate(self.j_feed):
                inp.feed_speed[k] = float(d.qvel[jv])
        inp.tilt_new = 0
        if t - self._t_tilt >= 0.005 - 1e-9:     # STS3250 position read at 200 Hz (est), 4096 counts per turn
            self._t_tilt = t
            inp.tilt_new = 1
            inp.tilt_pos = round(self.tilt() / (2 * math.pi / 4096)) * (2 * math.pi / 4096)
        inp.beam_catch = int(self.ball_held())
        inp.beam_feed_exit = int(self.p.feed == "chute" and self.feed_exit_blocked())
        inp.estop = int(self.estop)
        inp.bumper = int(self.bumper)
        inp.v_bus = float(self.battery.v + self.rng.normal(0, 0.02))       # ADC through a divider (est)
        inp.rc_ok = 0
        out = self.fw.tick()
        self.fw_out = out
        self.n_ticks += 1
        for i in range(out.n_can_tx if t >= self.can_cut_until else 0):
            f = out.can_tx[i]
            for k, mot in enumerate(self.dm):
                if f.id == 0x200 + mot.id:
                    a = self.a_wheel[k]
                    r = mot.on_frame(f, t, float(d.qpos[self.j_drive_q[k]]), float(d.qvel[self.j_drive_v[k]]),
                                     float(d.actuator_force[a]), self.drives[k].thermal.t_w)
                    if r is not None:
                        self.can_rx.append(r)
        self.throttle = [out.fly_throttle[0], out.fly_throttle[1]]
        self.fly_brake_cmd = bool(out.fly_brake)
        self.feed_cmd = float(out.feed_speed_cmd[0])
        self.tilt_cmd = float(out.tilt_cmd)
        self.tilt_target = self.tilt_cmd           # for Robot.post_step's servo current
        self.wheel_cmd = np.array([out.dbg_wheel_cmd[i] for i in range(self.nw)])

    # -- physics glue (Robot.pre_step without the firmware) ---------------------------------------------------
    def pre_step(self):
        dt, d, m = self.m.opt.timestep, self.d, self.m
        if self._k % self.fw_div == 0:
            self._fw_tick()
        self._k += 1
        if self.contacts is not None:
            self.contacts.pre_step()
        self._tick += 1
        if self._tick % self.floor_div == 0:
            for hook in self.floor:
                hook.pre_step()
        if self.swing_dofs is not None:
            d.qfrc_applied[self.swing_dofs] = -self.p.swing_preload * np.clip(d.qpos[self.swing_q] / 0.005, -1.0, 1.0)
        v_bus = self.battery.v
        t = d.time
        for k, (drv, mot) in enumerate(zip(self.drives, self.dm)):
            mot.step(t)
            a = self.a_wheel[k]
            w = d.qvel[self.j_drive_v[k]]
            if mot.enabled:
                ctrl, (lo, hi) = drv.command(mot.setpoint, w, v_bus, dt)
                d.ctrl[a] = ctrl
                m.actuator_forcerange[a] = (lo, hi)
            else:                                   # disabled: no torque, the wheel is back-driven freely
                drv.integ = 0.0
                d.ctrl[a] = w
                m.actuator_forcerange[a] = (0.0, 0.0)
        for k, fly in enumerate(self.fly):
            d.ctrl[self.a_fly[k]] = fly.step_throttle(self.throttle[k], d.qvel[self.j_fly_v[k]], v_bus, dt,
                                                      self.fly_brake_cmd)
        for a, fd, (jq, jv) in zip(self.a_feed, self.feed_drives, self.j_feed):
            ctrl, (lo, hi) = fd.command(self.feed_cmd, float(d.qpos[jq]), float(d.qvel[jv]), dt)
            d.ctrl[a] = ctrl
            m.actuator_forcerange[a] = (lo, hi)
        if self.a_gate is not None:
            d.ctrl[self.a_gate] = self.gate.step(self.p.gate_open_target if self.feed_cmd else 0.0, dt)
        lo, hi = self.p.tilt_limits
        d.ctrl[self.a_tilt] = self.tilt_servo.step(float(np.clip(self.tilt_cmd, lo, hi)), dt)

    def post_step(self):
        super().post_step()
        rd = self.sn.step()
        if rd.imu3 is not None:
            self.imu_pending = rd.imu3
        self.readings = rd

    def close(self):
        self.fw.close()
