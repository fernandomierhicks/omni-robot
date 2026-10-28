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

Timing (design.md 10), timing="real" (the default "ideal" reproduces earlier results: every input carries the tick's
time, the MCU's clock runs at exactly the sim's rate):
  the MCU clock    boot offset + a crystal error (clock_ppm, est +/-20 ppm) and a slow wander
  CAN1             the bus as it really runs at 1 Mbit/s: the four speed frames go out in order at the tick, each motor
                   samples its position as it turns its command around and replies, replies win arbitration (lower
                   ids), so the fourth reply lands ~0.8 ms after the tick (the bus is ~85% busy at 1 kHz). Each reply
                   carries its arrival time (the board's RX interrupt) and the position at the motor's sample time
  BNO085           each sample keeps its time, quantized to the SH-2 timestamps' 100 us
  time references  SYNC_OUT, CAM_TRIG fire exactly when the firmware scheduled them (hardware compares); the left
                   camera's STROBE follows each trigger by trig_to_exp with the exposure's length. sync_edges /
                   cam_triggers keep each edge's true sim time for the Jetson side's GPIO model (sil_link)
"""

from __future__ import annotations

import math
import os
import struct
import sys
from collections import deque
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


CAN_BIT = 1e-6                     # s at 1 Mbit/s


def can_frame_time(n_bytes: int) -> float:
    """A standard (11-bit) data frame on the wire: 47 + 8n bits, plus stuffing (~1 in 10 of the stuffable bits, est)."""
    return (47 + 8 * n_bytes + (34 + 8 * n_bytes) // 10) * CAN_BIT


class DmBus:
    """CAN1's timing (design.md 10). The tick queues its frames at t; each motor turns its command around in
    turnaround s (sampling its position then) and replies; a reply waiting goes before the next command (its id is
    lower), otherwise frames go in order. Returns, per frame, (end of the command, sample time, reply arrival)."""

    def __init__(self, turnaround: float = 15e-6, board_delay: float = 5e-6):
        self.turnaround, self.board_delay = turnaround, board_delay

    def schedule(self, t: float, cmd_lens: list) -> list:
        bus = t + self.board_delay
        pending, out = [], [None] * len(cmd_lens)
        i = 0
        while i < len(cmd_lens) or pending:
            ready = [p for p in pending if p[0] <= bus + 1e-12]
            if ready:
                r = min(ready, key=lambda p: p[0])
                pending.remove(r)
                bus += can_frame_time(8)
                out[r[1]] = (r[2], r[0], bus)            # sampled as it turned the command around
            elif i < len(cmd_lens):
                bus += can_frame_time(cmd_lens[i])
                pending.append((bus + self.turnaround, i, bus))
                i += 1
            else:
                bus = min(p[0] for p in pending)
        return out


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
                 boot_offset: float = 12.345, dm_comm_timeout: float = 0.1, sensor_kw: dict | None = None,
                 timing: str = "ideal", clock_ppm: float = 20.0, exposure: float = 0.003, trig_to_exp: float = 20e-6):
        super().__init__(model, data, p)
        if p.n_wheels != 4 or p.is_swerve:
            raise ValueError("the firmware core is built for the X4 base")
        if timing not in ("ideal", "real"):
            raise ValueError(timing)
        self.fw = Firmware()
        self.fw_div = max(1, int(round(0.001 / model.opt.timestep)))
        self.boot_offset = boot_offset            # s: the MCU booted this long before sim time 0
        # design.md 10: the MCU's clock and the edges it makes (see the module docstring)
        self.timing = timing
        self.real = timing == "real"
        self.clock_ppm = clock_ppm if self.real else 0.0    # the Teensy's crystal (est: +/-20 ppm, VERIFY)
        self.wander_ppm, self.wander_period = (1.0, 600.0) if self.real else (0.0, 600.0)   # temperature (est)
        self.bus = DmBus()
        self.exposure, self.trig_to_exp = exposure, trig_to_exp
        self.sync_edges = deque(maxlen=64)        # true sim times of the SYNC_OUT edges (the Jetson's GPIO sees them)
        self.n_sync = 0                           # ... how many so far
        self.cam_triggers = deque(maxlen=512)     # (trigger number, true sim time) of each CAM_TRIG pulse
        self._sync_next = self._cam_next = 0
        self._n_cam = 0
        self._strobe = None                       # (rise us, width us, due us) of the exposure in progress
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
        # fault injection (faults.py): one motor's CAN cut, the battery's voltage forced, the IMU or the flywheels'
        # telemetry silent, each until a sim time (math.inf: until cleared)
        self.motor_cut_until = [-1.0] * 4
        self.v_bus_override = None
        self.imu_cut_until = -1.0
        self.fly_telem_cut_until = -1.0

    # -- the firmware tick -------------------------------------------------------------------------------------
    def mcu_s(self, t: float) -> float:
        """The MCU's clock (s since its boot) at sim time t."""
        s = t + self.boot_offset
        if not self.real:
            return s
        w = self.wander_ppm * 1e-6 * self.wander_period / (2 * math.pi)
        return s * (1 + self.clock_ppm * 1e-6) + w * math.sin(2 * math.pi * t / self.wander_period)

    def mcu_time_us(self, t: float | None = None) -> int:
        return int(round(self.mcu_s(self.d.time if t is None else t) * 1e6))

    def sim_time_of(self, t_mcu_us: float) -> float:
        """The sim time at which the MCU's clock reads t_mcu_us."""
        target = t_mcu_us * 1e-6
        t = target - self.boot_offset
        for _ in range(3):
            t -= (self.mcu_s(t) - target) / (1 + self.clock_ppm * 1e-6)
        return t

    def _time_refs(self, inp, now_us: int):
        """The board's timers: the edges the firmware scheduled last tick, fired on time; the strobe that follows a
        trigger, reported once the exposure has ended."""
        inp.sync_fired_us = inp.cam_fired_us = inp.strobe_rise_us = 0
        inp.strobe_width_us = 0
        if self._sync_next and now_us >= self._sync_next:
            inp.sync_fired_us = self._sync_next
            self.sync_edges.append(self.sim_time_of(self._sync_next))
            self.n_sync += 1
        if self._strobe is not None and now_us >= self._strobe[2]:
            inp.strobe_rise_us, inp.strobe_width_us = self._strobe[0], self._strobe[1]
            self._strobe = None
        if self._cam_next and now_us >= self._cam_next:
            inp.cam_fired_us = self._cam_next
            self.cam_triggers.append((self._n_cam, self.sim_time_of(self._cam_next)))
            self._n_cam += 1
            rise = self._cam_next + int(round(self.trig_to_exp * 1e6))
            width = int(round(self.exposure * 1e6))
            self._strobe = (rise, width, rise + width)

    def _fw_tick(self):
        d, inp = self.d, self.fw.inputs
        t = d.time
        inp.t_us = self.mcu_time_us()
        n = min(len(self.can_rx), 16)
        for i in range(n):
            inp.can_rx[i] = self.can_rx[i]
        inp.n_can_rx = n
        self.can_rx = self.can_rx[n:]
        if self.imu_pending is not None and t < self.imu_cut_until:
            self.imu_pending = None                  # fault injection: the BNO085 went quiet
        if self.imu_pending is not None:
            ts, q, g, a = self.imu_pending
            inp.imu_n = 1
            s = inp.imu[0]
            # the sample's own time: the INT stamp (us) less the SH-2 delay, in its 100 us steps
            s.t_us = (inp.t_us - 100 * int(round((inp.t_us - self.mcu_time_us(ts)) / 100))) if self.real else 0
            for i in range(4):
                s.quat[i] = q[i]
            for i in range(3):
                s.gyro[i] = g[i]
                s.accel[i] = a[i]
            self.imu_pending = None
        else:
            inp.imu_n = 0
        self._time_refs(inp, inp.t_us)
        w_fly = self.flywheel_speeds()
        for k in range(2):                       # telemetry every 0.5 ms: always a fresh frame at the 1 ms tick
            inp.fly_new[k] = 0 if t < self.fly_telem_cut_until else 1
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
        v = self.battery.v if self.v_bus_override is None else self.v_bus_override
        inp.v_bus = float(v + self.rng.normal(0, 0.02))                    # ADC through a divider (est)
        inp.rc_ok = 0
        out = self.fw.tick()
        self.fw_out = out
        self.n_ticks += 1
        self._sync_next, self._cam_next = int(out.sync_next_us), int(out.cam_next_us)
        n_tx = out.n_can_tx if t >= self.can_cut_until else 0
        sched = self.bus.schedule(t, [out.can_tx[i].len for i in range(n_tx)]) if self.real else None
        for i in range(n_tx):
            f = out.can_tx[i]
            for k, mot in enumerate(self.dm):
                if f.id == 0x200 + mot.id and t >= self.motor_cut_until[k]:
                    a = self.a_wheel[k]
                    q, w = float(d.qpos[self.j_drive_q[k]]), float(d.qvel[self.j_drive_v[k]])
                    if sched is not None:
                        q += w * (sched[i][1] - t)          # the position when the motor sampled it
                    r = mot.on_frame(f, t, q, w, float(d.actuator_force[a]), self.drives[k].thermal.t_w)
                    if r is not None:
                        if sched is not None:
                            r.t_rx_us = self.mcu_time_us(sched[i][2])
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
        for k, (a, jv) in enumerate(zip(self.a_fly, self.j_fly_v)):
            d.ctrl[a] = self.fly[k].step_throttle(self.throttle[k], d.qvel[jv], v_bus, dt, self.fly_brake_cmd)
        for a, fd, (jq, jv) in zip(self.a_feed, self.feed_drives, self.j_feed):
            ctrl, (lo, hi) = fd.command(self.feed_cmd, float(d.qpos[jq]), float(d.qvel[jv]), dt)
            d.ctrl[a] = ctrl
            m.actuator_forcerange[a] = (lo, hi)
        if self.a_gate is not None:
            d.ctrl[self.a_gate] = self.gate.step(self.p.gate_open_target if self.feed_cmd else 0.0, dt)
        if self.a_tilt is not None:
            lo, hi = self.p.tilt_limits
            d.ctrl[self.a_tilt] = self.tilt_servo.step(float(np.clip(self.tilt_cmd, lo, hi)), dt)
        if self.arm is not None:        # payload "arm": the arm's servos, commanded outside the firmware
            self.arm.step(dt)

    def post_step(self):
        super().post_step()
        rd = self.sn.step()
        if rd.imu3 is not None:
            self.imu_pending = rd.imu3
        self.readings = rd

    def close(self):
        self.fw.close()
