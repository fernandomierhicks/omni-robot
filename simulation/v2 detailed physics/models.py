"""Electromechanical models: drive actuators, flywheel motors + ESC, gate servo, battery.

All models are stepped once per physics step by robot.py. Parameters come from
catalog.py; every number a bench test can measure lives there, not here.
"""

from __future__ import annotations

import math

import numpy as np

from catalog import BatteryPack, DriveMotor, Esc, FlywheelMotor, Thermal

CU_TEMPCO = 0.0039     # copper resistance rise per K


class ThermalNode2:
    """Winding and case temperatures (two-node RC network)."""

    def __init__(self, th: Thermal, t_amb: float = 25.0):
        self.th, self.t_amb = th, t_amb
        self.reset()

    def reset(self):
        self.t_w = self.t_c = self.t_amb

    def step(self, p_loss: float, dt: float):
        th = self.th
        q_wc = (self.t_w - self.t_c) / th.r_wc
        q_ca = (self.t_c - self.t_amb) / th.r_ca
        self.t_w += (p_loss - q_wc) / th.c_winding * dt
        self.t_c += (q_wc - q_ca) / th.c_case * dt

    def derate(self) -> float:
        """Current-limit scale: 1 below t_max-10 C, falling linearly to 0 at t_max."""
        return float(np.clip((self.th.t_max - self.t_w) / 10.0, 0.0, 1.0))


class DriveActuator:
    """AK-series style actuator: FOC driver with a PI speed loop, gearbox, limits.

    The speed loop's proportional term is applied inside MuJoCo by a general
    actuator (force = kp * (ctrl - joint speed)), which is integrated implicitly and
    so stays stable at any gain. This class supplies the integral term through
    ctrl, and the torque limits through the actuator's force range:
      - current limit (driver peak current, thermally derated),
      - voltage limit: the bus voltage minus back-EMF must push the current
        through the winding, so motoring torque fades with speed and sags with
        the battery,
      - gearbox efficiency.
    After each step, account() turns the torque MuJoCo applied into phase
    current, bus current, copper loss and temperature.
    """

    def __init__(self, spec: DriveMotor, kp: float = 2.0, ki: float = 40.0, t_amb: float = 25.0):
        self.s, self.kp, self.ki = spec, kp, ki
        self.thermal = ThermalNode2(spec.thermal, t_amb)
        self.k_out = spec.kt * spec.gear * spec.gear_eff      # output N m per phase amp
        self.reset()

    def reset(self):
        self.integ = 0.0
        self.thermal.reset()
        self.i_phase = self.i_bus = self.p_loss = 0.0

    def resistance(self) -> float:
        return self.s.r_eq * (1 + CU_TEMPCO * (self.thermal.t_w - 25.0))

    def torque_limits(self, w_out: float, v_bus: float) -> tuple[float, float]:
        """(min, max) output torque available at this speed and bus voltage."""
        s = self.s
        i_cap = s.i_peak * self.thermal.derate()
        emf = s.ke * abs(w_out) * s.gear
        r = self.resistance()
        i_motoring = float(np.clip((v_bus - emf) / r, 0.0, i_cap))
        i_braking = float(np.clip((v_bus + emf) / r, 0.0, i_cap))
        t_mot, t_brk = self.k_out * i_motoring, self.k_out * i_braking
        return (-t_brk, t_mot) if w_out >= 0 else (-t_mot, t_brk)

    def command(self, w_cmd: float, w_out: float, v_bus: float, dt: float):
        """Speed command -> (ctrl for the MuJoCo actuator, force range)."""
        lo, hi = self.torque_limits(w_out, v_bus)
        self.integ = float(np.clip(self.integ + self.ki * (w_cmd - w_out) * dt, lo, hi))
        return w_cmd + self.integ / self.kp, (lo, hi)

    def account(self, tau_out: float, w_out: float, v_bus: float, dt: float, driver_eff: float = 0.95):
        """Electrical side of the torque actually applied this step."""
        s = self.s
        i = abs(tau_out) / self.k_out
        r = self.resistance()
        p_cu = r * i * i
        p_mech = tau_out * w_out                       # >0 motoring, <0 regenerating
        p_elec = p_cu + (p_mech / s.gear_eff if p_mech > 0 else p_mech * s.gear_eff)
        self.i_phase, self.p_loss = i, p_cu
        self.i_bus = p_elec / driver_eff / v_bus if p_elec > 0 else p_elec * driver_eff / v_bus
        self.thermal.step(p_cu, dt)
        return self.i_bus


class FlywheelDrive:
    """Drone outrunner on an AM32 ESC with closed-loop rpm control on the MCU.

    ESC: the DShot throttle sets the average phase voltage (throttle * bus).
    With complementary PWM (AM32 default) the current may reverse, so dropping
    the throttle brakes actively. The ESC clamps phase current.
    MCU: PI on the eRPM telemetry (bidirectional DShot), with a feed-forward
    throttle from the back-EMF constant; telemetry arrives every telemetry_period.
    """

    def __init__(self, motor: FlywheelMotor, esc: Esc, kp: float = 0.004, ki: float = 0.08):
        self.m, self.esc, self.kp, self.ki = motor, esc, kp, ki
        self.ke = 1 / (motor.kv * 2 * math.pi / 60)
        self.w_10v = 10 / self.ke
        self.reset()

    def reset(self):
        self.integ = 0.0
        self.w_meas = 0.0
        self.t_since_telemetry = 0.0
        self.throttle = 0.0
        self.i_phase = self.i_bus = 0.0
        self.e_loss = 0.0

    def friction(self, w: float) -> float:
        """Bearing + iron loss torque, split Coulomb/viscous, matched to I0 at 10 V."""
        t0 = self.m.kt * self.m.i0
        return math.copysign(0.4 * t0 + 0.6 * t0 * abs(w) / self.w_10v, w) if w else 0.0

    def step(self, w_target: float, w_true: float, v_bus: float, dt: float) -> float:
        """Returns the motor torque on the flywheel for this step."""
        self.t_since_telemetry += dt
        if self.t_since_telemetry >= self.esc.telemetry_period:
            self.t_since_telemetry = 0.0
            self.w_meas = w_true
        if w_target <= 0:
            self.throttle, self.integ = 0.0, 0.0
        else:
            err = w_target - self.w_meas
            ff = (self.ke * w_target + self.m.r_m * self.friction(w_target) / self.m.kt) / v_bus
            self.integ = float(np.clip(self.integ + self.ki * err * dt, -0.2, 0.2))
            self.throttle = float(np.clip(ff + self.kp * err + self.integ, 0.0, 1.0))
        v_phase = self.throttle * v_bus
        i = (v_phase - self.ke * w_true) / self.m.r_m
        if w_target <= 0:
            i = 0.0                                   # DShot 0: ESC disarmed, the wheel coasts
        # otherwise complementary PWM: a duty below the back-EMF ratio brakes actively
        i = float(np.clip(i, -self.esc.current_limit, self.esc.current_limit))
        # when the ESC limits current it does so by lowering the duty cycle
        duty = float(np.clip((self.ke * w_true + self.m.r_m * i) / v_bus, 0.0, 1.0))
        self.i_phase = i
        self.i_bus = duty * i / 0.95 if i > 0 else duty * i * 0.95
        self.e_loss += self.m.r_m * i * i * dt
        return self.m.kt * i - self.friction(w_true)


class GateServo:
    """Hobby servo driving the trapdoor directly: position command (rad), slew-limited
    to its no-load speed (rad/s). Stall current is for a 25 kg cm servo at 6 V."""

    def __init__(self, speed: float = 8.0, i_stall: float = 2.5, v: float = 6.0):
        self.speed, self.i_stall, self.v = speed, i_stall, v
        self.pos_cmd = 0.0

    def step(self, target: float, dt: float) -> float:
        d = float(np.clip(target - self.pos_cmd, -self.speed * dt, self.speed * dt))
        self.pos_cmd += d
        return self.pos_cmd

    def bus_current(self, moving: bool, v_bus: float) -> float:
        return (0.5 * self.i_stall if moving else 0.01) * self.v / v_bus / 0.9


class Battery:
    """Thevenin pack model: OCV(SoC) - i*R0(SoC, T) - one RC polarization branch."""

    def __init__(self, pack: BatteryPack, soc: float = 1.0, t_c: float = 25.0):
        self.p = pack
        self.soc0, self.t_c = soc, t_c
        self._soc_pts = np.array(pack.cell.ocv_soc)
        self._v_pts = np.array(pack.cell.ocv_v) * pack.series
        self.reset()

    def reset(self):
        self.soc = self.soc0
        self.v_rc = 0.0
        self.i = 0.0
        self.v = self.ocv()
        self.v_min = self.v
        self.i_peak = 0.0
        self.tripped = False

    def ocv(self) -> float:
        return float(np.interp(self.soc, self._soc_pts, self._v_pts))

    def r0(self) -> float:
        soc_factor = 1.0 + 0.6 * (1.0 - self.soc) ** 3          # resistance rises when nearly empty
        temp_factor = math.exp(0.012 * (25.0 - self.t_c))       # and when cold
        return self.p.r0 * soc_factor * temp_factor

    def step(self, i_load: float, dt: float) -> float:
        p = self.p
        r1 = p.cell.r1 * p.series / p.parallel
        tau = p.cell.tau1
        self.v_rc += (i_load * r1 - self.v_rc) * (dt / tau)
        self.soc = float(np.clip(self.soc - i_load * dt / (p.capacity * 3600), 0.0, 1.0))
        self.i = i_load
        self.v = self.ocv() - i_load * self.r0() - self.v_rc
        self.v_min = min(self.v_min, self.v)
        self.i_peak = max(self.i_peak, i_load)
        if i_load > p.i_bms:
            self.tripped = True
        return self.v
