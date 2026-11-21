"""Feetech STS3215 serial-bus servos as the robot's driver meets them, for params.arm_model = "detailed" (design.md 4.5):
each servo's memory table (EEPROM and RAM), its firmware (the position loop with its dead zone, start-up force, torque
limit, the protections and the status it reports), its sensors (the 12-bit magnetic encoder, the current sense, the
voltage ADC, the case thermistor, each with its own error) and the half-duplex bus (PING, READ, WRITE, REG_WRITE /
ACTION, SYNC_READ, SYNC_WRITE). arm_detailed.py is the physics under it (motor, gearbox, horn, links); the robot's own
omni_cpp driver talks to ServoBus byte for byte, in-process through omni_cpp/sil/arm_sil or over TCP (arm_bus.ServoPort).

Every number is an estimate: no STS3215 has been on the bench yet (Nov 20). The memory table is the STS/SMS table that
LeRobot's Feetech driver lists; the defaults, the units and the firmware's behaviour are Feetech's documentation as
recalled plus reseller data. VERIFY each against a register dump and a held-weight test on the first servo:
  - Present Load (60) is taken to be the PWM duty the firmware drives the motor with, in 0.1 %, bit 10 the sign (a
    current-like reading only at rest: when the motor turns, back-EMF takes part of the duty)
  - Present Current (69) the motor current in 6.5 mA, bit 15 the sign (the STS3235 / STS3250 pages don't list it)
  - P (21) and D (22) scale the duty per tick of error (KP, KD below); the dead zone (26, 27) gives zero duty inside it
  - overload (36 % for 35 x 10 ms) drops the output to the protective torque (34 %); overheat (13) and over-current
    (28 for 38 x 10 ms) unload the servo when Unloading Condition (19) has their bit
  - EEPROM-area writes act at once; they survive a power cycle only when written with Lock (55) = 0
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

TICKS = 4096
TICK = 2 * math.pi / TICKS                     # rad per tick
BAUD = 1_000_000

# the memory table: name -> (address, size); EEPROM below 40 (LeRobot's STS_SMS_SERIES_CONTROL_TABLE)
REG = dict(
    firmware_major=(0, 1), firmware_minor=(1, 1), model_number=(3, 2), id=(5, 1), baud=(6, 1), return_delay=(7, 1),
    response_level=(8, 1), min_limit=(9, 2), max_limit=(11, 2), max_temp=(13, 1), max_volt=(14, 1), min_volt=(15, 1),
    max_torque=(16, 2), phase=(18, 1), unload_cond=(19, 1), led_alarm=(20, 1), p_gain=(21, 1), d_gain=(22, 1),
    i_gain=(23, 1), startup_force=(24, 2), cw_dead=(26, 1), ccw_dead=(27, 1), protect_current=(28, 2),
    resolution=(30, 1), homing_offset=(31, 2), mode=(33, 1), protect_torque=(34, 1), protect_time=(35, 1),
    overload_torque=(36, 1), speed_p=(37, 1), overcurrent_time=(38, 1), speed_i=(39, 1),
    torque_enable=(40, 1), acceleration=(41, 1), goal_position=(42, 2), goal_time=(44, 2), goal_speed=(46, 2),
    torque_limit=(48, 2), lock=(55, 1), present_position=(56, 2), present_speed=(58, 2), present_load=(60, 2),
    present_voltage=(62, 1), present_temperature=(63, 1), async_flag=(64, 1), status=(65, 1), moving=(66, 1),
    present_current=(69, 2))
EEPROM_END = 40
# factory defaults (est; LeRobot's SO-101 follower setup changes P, D and the gripper's limits: arm_driver's own
# configure writes the robot's values)
DEFAULTS = dict(firmware_major=3, firmware_minor=10, model_number=777, baud=0, return_delay=0, response_level=1,
                min_limit=0, max_limit=4095, max_temp=70, max_volt=140, min_volt=40, max_torque=1000, phase=0,
                unload_cond=44, led_alarm=47, p_gain=32, d_gain=32, i_gain=0, startup_force=16, cw_dead=1,
                ccw_dead=1, protect_current=500, resolution=1, homing_offset=0, mode=0, protect_torque=20,
                protect_time=200, overload_torque=80, speed_p=10, overcurrent_time=200, speed_i=10,
                torque_enable=1, acceleration=0, goal_time=0, goal_speed=0, torque_limit=1000, lock=1)
# error / status bits
E_VOLTAGE, E_ANGLE, E_HEAT, E_CURRENT, E_OVERLOAD = 1, 2, 4, 8, 32
SIGNED = dict(homing_offset=11, present_speed=15, present_load=10, present_current=15, goal_speed=15)

# firmware scaling (est): duty per tick of error per unit of P (integer firmware arithmetic: duty in 0.1 % = P x the
# error in ticks); per tick-per-ms of the error's change per unit of D (its change over D_WINDOW). At P 16 on
# arm_detailed's motor: ~31 N m/rad on the stall line, so ~1 deg of droop under 1 N m (what SO-100/101 users see), and
# Present Load steps 1.6 % a tick. A per-tick step 10x this (the official model's kp 998 N m/rad) made every load
# reading jump ~1 N m and hunted on the gearbox's play (Nov 20 trial)
KP, KD = 0.0010, 0.0010
D_WINDOW = 0.004                               # s
FW_DT = 0.001                                  # s: the firmware's control tick (est 1 kHz)
SPEED_WINDOW = 0.010                           # s: the speed it reports, from the position over this window
I_LSB, V_LSB = 0.0065, 0.1                     # A, V per count


def to_sm(v: int, bit: int) -> int:
    return (abs(v) & ((1 << bit) - 1)) | ((1 << bit) if v < 0 else 0)


def from_sm(v: int, bit: int) -> int:
    return -(v & ((1 << bit) - 1)) if v & (1 << bit) else v & ((1 << bit) - 1)


@dataclass
class SensorTruth:
    """One servo's sensors as built (est): what its readings get wrong."""
    enc_a1: float = 0.0          # rad: once-per-turn encoder error (magnet off the shaft's axis)
    enc_p1: float = 0.0
    enc_a2: float = 0.0          # rad: twice-per-turn error (magnetization, the chip's own)
    enc_p2: float = 0.0
    enc_noise: float = 0.35      # ticks rms: the last bit flickers
    cal_err: float = 0.0         # rad: LeRobot's hand calibration off the joint's true middle (homing offset error)
    mech_zero: int = 0           # ticks: where the horn sits on the raw encoder (the homing offset hides it)
    cur_gain: float = 1.0        # current sense gain
    cur_offset: float = 0.0      # A
    cur_noise: float = 0.008     # A rms
    volt_gain: float = 1.0       # voltage ADC gain
    temp_offset: float = 0.0     # C: the thermistor's

    @staticmethod
    def draw(rng: np.random.Generator, cal_err_deg: float = 1.0) -> "SensorTruth":
        return SensorTruth(enc_a1=math.radians(rng.uniform(0.05, 0.25)), enc_p1=rng.uniform(0, 2 * math.pi),
                           enc_a2=math.radians(rng.uniform(0.03, 0.15)), enc_p2=rng.uniform(0, 2 * math.pi),
                           cal_err=math.radians(rng.normal(0.0, cal_err_deg)), mech_zero=int(rng.integers(-900, 900)),
                           cur_gain=rng.normal(1.0, 0.03), cur_offset=rng.normal(0.0, 0.010),
                           volt_gain=rng.normal(1.0, 0.015), temp_offset=rng.normal(0.0, 1.0))


class Sts3215:
    """One servo: its memory, its firmware and its sensors. The physics calls update() every firmware tick with the
    true output-shaft angle, the motor current, the supply voltage and the case temperature; it returns the PWM duty
    (-1..1, + turns toward more ticks)."""

    def __init__(self, sid: int, sensors: SensorTruth, rng: np.random.Generator, eeprom: dict | None = None,
                 angle0: float = 0.0):
        self.sid, self.s, self.rng = sid, sensors, rng
        self.mem = bytearray(128)
        for k, v in DEFAULTS.items():
            self._put(k, v)
        self._put("id", sid)
        # LeRobot's calibration left the homing offset that puts the joint's middle near tick 2048
        self._put("homing_offset", sensors.mech_zero + round(sensors.cal_err / TICK))
        for k, v in (eeprom or {}).items():                  # what survived the last power cycle
            self._put(k, v)
        self.saved = {k: self.get(k) for k in REG if REG[k][0] < EEPROM_END}
        self.duty = 0.0
        self.i_meas = 0.0
        self.pos = self._encoder(angle0)
        self._put("goal_position", self.pos)
        self._put("present_position", self.pos)            # it reports from power-up, before its first tick
        self._put("present_voltage", 120)
        self._put("present_temperature", 25)
        self.hist = [(0.0, self.pos)]                       # (t, position): the reported speed's window
        self.e_prev = 0.0
        self.e_hist: list = []
        self.i_acc = 0.0
        self.t = 0.0
        self.t_overload = self.t_overcurrent = 0.0
        self.overloaded = False
        self.unloaded = False
        self.errors = 0
        self.reg_write = None                               # REG_WRITE's (address, data) until ACTION

    # -- memory ---------------------------------------------------------------------------------------------------
    def get(self, name: str) -> int:
        a, n = REG[name]
        v = self.mem[a] if n == 1 else self.mem[a] | self.mem[a + 1] << 8
        return from_sm(v, SIGNED[name]) if name in SIGNED else v

    def _put(self, name: str, v: int):
        a, n = REG[name]
        v = to_sm(int(v), SIGNED[name]) if name in SIGNED else int(v) & ((1 << (8 * n)) - 1)
        self.mem[a] = v & 0xFF
        if n == 2:
            self.mem[a + 1] = (v >> 8) & 0xFF

    def write_bytes(self, addr: int, data: bytes) -> bool:
        """WRITE / SYNC_WRITE / ACTION: False (an angle-limit error in the status) for a read-only address."""
        ro = range(REG["present_position"][0], 71)
        if any(a in ro for a in range(addr, addr + len(data))):
            return False                                    # read-only: ignored (est)
        self.mem[addr:addr + len(data)] = data
        if addr < EEPROM_END and self.get("lock") == 0:      # unlocked: saved for the next power cycle
            self.saved = {k: self.get(k) for k in REG if REG[k][0] < EEPROM_END}
        if addr <= REG["torque_enable"][0] < addr + len(data) and self.get("torque_enable"):
            self.unloaded = False                           # torque on again clears an unload
            self.overloaded = False
            self.errors &= ~(E_OVERLOAD | E_HEAT | E_CURRENT)
            self.i_acc = 0.0
            if not (addr <= REG["goal_position"][0] < addr + len(data)):
                self._put("goal_position", self.pos)        # (est: it holds where it is)
        return True

    def read_bytes(self, addr: int, n: int) -> bytes:
        return bytes(self.mem[addr:addr + n])

    # -- sensors --------------------------------------------------------------------------------------------------
    def _encoder(self, angle: float) -> int:
        """Present position (ticks, after the homing offset) of the output shaft's true angle."""
        s = self.s
        a = angle + s.enc_a1 * math.sin(angle + s.enc_p1) + s.enc_a2 * math.sin(2 * angle + s.enc_p2)
        raw = 2048 + s.mech_zero + a / TICK + self.rng.normal(0.0, s.enc_noise)
        return int(round(raw - self.get("homing_offset"))) % TICKS

    # -- firmware ---------------------------------------------------------------------------------------------------
    def update(self, t: float, angle: float, i_motor: float, volts: float, temp_c: float) -> float:
        dt = max(1e-6, t - self.t)
        self.t = t
        s = self.s
        self.pos = pos = self._encoder(angle)
        self.hist.append((t, pos))
        while len(self.hist) > 2 and self.hist[1][0] <= t - SPEED_WINDOW:
            self.hist.pop(0)
        t0, p0 = self.hist[0]
        speed = (pos - p0) / max(t - t0, FW_DT)
        self.i_meas = s.cur_gain * i_motor + s.cur_offset + self.rng.normal(0.0, s.cur_noise)
        v_meas = volts * s.volt_gain
        t_meas = temp_c + s.temp_offset
        # protections
        err = 0
        if not (self.get("min_volt") * V_LSB <= v_meas <= self.get("max_volt") * V_LSB):
            err |= E_VOLTAGE
        if t_meas >= self.get("max_temp"):
            err |= E_HEAT
        torque_on = self.get("torque_enable") == 1
        if torque_on and abs(self.i_meas) > self.get("protect_current") * I_LSB:
            self.t_overcurrent += dt
            if self.t_overcurrent >= self.get("overcurrent_time") * 0.01:
                err |= E_CURRENT
        else:
            self.t_overcurrent = 0.0
        if torque_on and abs(self.duty) * 100 >= self.get("overload_torque") * min(1.0, self.get("torque_limit") / 1000):
            self.t_overload += dt
            if self.t_overload >= self.get("protect_time") * 0.01:
                self.overloaded = True
        else:
            self.t_overload = 0.0
            if self.overloaded and abs(self.duty) * 100 < 0.5 * self.get("protect_torque"):
                self.overloaded = False                     # (est: the overload clears once the load is gone)
        if self.overloaded:
            err |= E_OVERLOAD
        if err & self.get("unload_cond") & (E_HEAT | E_CURRENT | E_VOLTAGE):
            self.unloaded = True
        # an unload latches its cause until the torque is switched on again
        self.errors = err | ((self.errors & (E_HEAT | E_CURRENT | E_VOLTAGE)) if self.unloaded else 0)
        # the position loop
        duty = 0.0
        if torque_on and not self.unloaded:
            lo, hi = self.get("min_limit"), self.get("max_limit")
            goal = self.get("goal_position")
            if lo < hi and not lo <= goal <= hi:
                goal = min(max(goal, lo), hi)
                self.errors |= E_ANGLE                      # (reported, still moves to the limit; est)
            e = float(goal - pos)
            if -self.get("ccw_dead") <= e <= self.get("cw_dead"):
                e_eff = 0.0
            else:
                e_eff = e
            self.e_hist.append(e)                            # the error's change over D_WINDOW, per firmware tick
            n = int(round(D_WINDOW / FW_DT))
            if len(self.e_hist) > n + 1:
                self.e_hist.pop(0)
            de = (e - self.e_hist[0]) / max(1, len(self.e_hist) - 1)
            self.e_prev = e
            u = self.get("p_gain") * KP * e_eff + self.get("d_gain") * KD * de
            if self.get("i_gain") and e_eff:
                self.i_acc = float(np.clip(self.i_acc + self.get("i_gain") * 1e-5 * e_eff, -0.3, 0.3))
                u += self.i_acc
            if e_eff and abs(u) < self.get("startup_force") / 1000:
                u = math.copysign(self.get("startup_force") / 1000, e_eff)
            lim = min(self.get("torque_limit"), self.get("max_torque")) / 1000
            if self.overloaded:
                lim = min(lim, self.get("protect_torque") / 100)
            # the current limit: the duty that keeps the motor current at the protection current (est: per tick)
            i_lim = self.get("protect_current") * I_LSB
            if abs(self.i_meas) > i_lim and abs(self.duty) > 0:
                lim = min(lim, abs(self.duty) * i_lim / abs(self.i_meas))
            duty = float(np.clip(u, -lim, lim))
        else:
            self.e_prev = 0.0
            self.e_hist.clear()
        self.duty = duty
        # the present values
        self._put("present_position", pos)
        self._put("present_speed", int(round(speed)))
        self._put("present_load", int(round(duty * 1000)))
        self._put("present_voltage", min(255, max(0, int(round(v_meas / V_LSB)))))
        self._put("present_temperature", min(255, max(0, int(round(t_meas)))))
        self._put("present_current", int(round(self.i_meas / I_LSB)))
        self._put("status", self.errors)
        self._put("moving", int(abs(speed) > 20 or abs(self.get("goal_position") - pos) > 2))
        return duty

    def status_error(self) -> int:
        return self.errors & 0xFF


# -- the bus ------------------------------------------------------------------------------------------------------

def _chk(body) -> int:
    return (~sum(body)) & 0xFF


def status_packet(sid: int, err: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, err]) + params
    return b"\xff\xff" + body + bytes([_chk(body)])


@dataclass
class BusStats:
    packets: int = 0
    errors: int = 0
    replies: int = 0
    bytes_in: int = 0
    bytes_out: int = 0
    busy_us: float = 0.0              # the wire's time on the instructions and replies (1 Mbaud, the return delays)
    by_instr: dict = field(default_factory=dict)


class ServoBus:
    """write(bytes) the driver's packets; read(n) the servos' replies (arm_bus.ServoBusSim's interface, so
    arm_bus.ServoPort and arm_sil serve it the same way)."""

    def __init__(self, servos: dict):
        self.servos = servos                               # id -> Sts3215
        self.rx, self.tx = bytearray(), bytearray()
        self.stats = BusStats()

    # (arm_bus.ServoBusSim's names, for the harnesses)
    @property
    def n_errors(self) -> int:
        return self.stats.errors

    def write(self, data: bytes):
        self.rx += data
        self.stats.bytes_in += len(data)
        self.stats.busy_us += 10.0 * len(data) / (BAUD / 1e6)
        self._parse()

    def read(self, n: int = 256) -> bytes:
        out = bytes(self.tx[:n])
        del self.tx[:n]
        return out

    def _reply(self, sv: Sts3215, params: bytes = b"", always: bool = False):
        if not always and sv.get("response_level") == 0:
            return
        p = status_packet(sv.sid, sv.status_error(), params)
        self.tx += p
        self.stats.replies += 1
        self.stats.bytes_out += len(p)
        self.stats.busy_us += 10.0 * len(p) / (BAUD / 1e6) + 2.0 * sv.get("return_delay") + 20.0   # + its turnaround

    def _parse(self):
        b = self.rx
        while True:
            k = b.find(b"\xff\xff")
            if k < 0:
                del b[:max(0, len(b) - 1)]
                return
            del b[:k]
            if len(b) < 6:
                return
            sid, n = b[2], b[3]
            if sid == 0xFF:
                del b[:1]
                continue
            if n < 2:
                del b[:2]
                self.stats.errors += 1
                continue
            if len(b) < 4 + n:
                return
            body = bytes(b[2:3 + n])
            chk = b[3 + n]
            del b[:4 + n]
            if _chk(body) != chk:
                self.stats.errors += 1
                continue
            self.stats.packets += 1
            self.stats.by_instr[body[2]] = self.stats.by_instr.get(body[2], 0) + 1
            self._handle(sid, body[2], body[3:])

    def _targets(self, sid: int) -> list:
        return list(self.servos.values()) if sid == 0xFE else [self.servos[sid]] if sid in self.servos else []

    def _handle(self, sid: int, instr: int, p: bytes):
        S = self.servos
        if instr == 0x01 and sid in S:                                             # PING
            self._reply(S[sid], always=True)
        elif instr == 0x02 and sid in S and len(p) == 2:                           # READ
            self._reply(S[sid], S[sid].read_bytes(p[0], p[1]), always=True)
        elif instr == 0x03 and len(p) >= 2:                                        # WRITE
            for sv in self._targets(sid):
                sv.write_bytes(p[0], p[1:])
                if sid != 0xFE:
                    self._reply(sv)
        elif instr == 0x04 and len(p) >= 2:                                        # REG_WRITE
            for sv in self._targets(sid):
                sv.reg_write = (p[0], p[1:])
                if sid != 0xFE:
                    self._reply(sv)
        elif instr == 0x05:                                                        # ACTION
            for sv in self._targets(sid):
                if sv.reg_write is not None:
                    sv.write_bytes(*sv.reg_write)
                    sv.reg_write = None
        elif instr == 0x83 and len(p) >= 2:                                        # SYNC_WRITE
            addr, size = p[0], p[1]
            for i in range(2, len(p) - size, size + 1):
                if p[i] in S:
                    S[p[i]].write_bytes(addr, p[i + 1:i + 1 + size])
        elif instr == 0x82 and len(p) >= 2:                                        # SYNC_READ
            addr, size = p[0], p[1]
            for s in p[2:]:
                if s in S:
                    self._reply(S[s], S[s].read_bytes(addr, size), always=True)
