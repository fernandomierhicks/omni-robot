"""The SO-101's six STS3215s as their serial bus behaves (design.md 11), driving arm_so101.ArmServos: the servos' side
of the protocol, written independently of the Jetson's (ros/src/omni_arm/omni_arm/feetech.py), like sil_robot's DmMotor.

Each servo keeps the registers the Jetson uses (feetech.REG): torque enable, acceleration, goal position / time /
speed, torque limit, and answers PING, READ, WRITE, SYNC_WRITE and SYNC_READ with status packets. Present position,
speed, load, voltage and temperature come from the physics. The servos are calibrated as LeRobot's tool leaves them:
each joint's range symmetric about tick 2048, angle 0 at 2048 (the sim's convention), drive_mode 0.

What the registers do to the arm:
  goal_position   ArmServos' target (clipped to the joint's range), slewed at the no-load speed (or goal_speed if set)
  torque_enable   0: the joint goes limp (no torque either way)
  torque_limit    scales the stall torque (0..1000 = 0..100%)
Timing: the replies are ready at the next exchange (the bridge's port runs once per firmware tick, 1 ms); a real
1 Mbaud bus takes ~0.1 ms per status packet (VERIFY with the Waveshare board), so a 6-servo SYNC_READ is ~0.8 ms.

Not modelled: the servo's own P gain / deadband (its position loop is MuJoCo's, as arm_so101), overload protection,
the EEPROM lock, baud changes, temperature (reads 30 C).
"""

from __future__ import annotations

import math
import socket

import numpy as np

import arm_so101 as A

TICKS = 4096
REG = dict(id=5, torque_enable=40, acceleration=41, goal_position=42, goal_time=44, goal_speed=46, torque_limit=48,
           lock=55, present_position=56, present_speed=58, present_load=60, present_voltage=62,
           present_temperature=63, moving=66, present_current=69)


def _chk(body) -> int:
    return (~sum(body)) & 0xFF


def _status(sid: int, err: int, params: bytes = b"") -> bytes:
    body = bytes([sid, len(params) + 2, err]) + params
    return b"\xff\xff" + body + bytes([_chk(body)])


def _sm(v: int, bit: int) -> int:
    return (abs(v) & ((1 << bit) - 1)) | ((1 << bit) if v < 0 else 0)


class ServoBusSim:
    """write(bytes) the Jetson's packets in; read(n) the servos' replies out. step() after ArmServos.step."""

    def __init__(self, arm: "A.ArmServos", ids=(1, 2, 3, 4, 5, 6), v_bus: float = 12.0, joint_err_deg: float = 0.0,
                 seed: int = 0):
        self.arm = arm
        self.ids = list(ids)
        self.v_bus = v_bus
        # each servo's calibration error: the joint really sits err off the angle its ticks say (est 1 deg rms for a
        # hand calibration; the gripper's doesn't matter to the pick)
        self.err = np.random.default_rng(seed + 41).normal(0.0, math.radians(joint_err_deg), len(self.ids))
        self.err[5] = 0.0
        lo, hi = A.joint_ranges().T
        self.half = np.maximum(np.abs(lo), np.abs(hi)) * TICKS / (2 * math.pi)
        self.mem = {sid: bytearray(80) for sid in self.ids}
        for k, sid in enumerate(self.ids):
            m = self.mem[sid]
            m[REG["id"]] = sid
            m[REG["torque_limit"]:REG["torque_limit"] + 2] = (1000).to_bytes(2, "little")
            m[REG["acceleration"]] = 0
            g = self._ticks(k, arm.target[k] - self.err[k])
            m[REG["goal_position"]:REG["goal_position"] + 2] = g.to_bytes(2, "little")
            m[REG["torque_enable"]] = 1
        self.rx = bytearray()
        self.tx = bytearray()
        self.n_packets = self.n_errors = 0

    # -- conversions (the sim's calibration: the range's middle at 2048) --------------------------------------------
    def _ticks(self, k: int, rad: float) -> int:
        return int(round(2048 + rad * TICKS / (2 * math.pi)))

    def _rad(self, k: int, ticks: int) -> float:
        return (ticks - 2048) * 2 * math.pi / TICKS

    # -- the byte stream ---------------------------------------------------------------------------------------------
    def write(self, data: bytes):
        self.rx += data
        self._parse()

    def read(self, n: int = 256) -> bytes:
        out = bytes(self.tx[:n])
        del self.tx[:n]
        return out

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
            if n < 2:
                del b[:2]
                self.n_errors += 1
                continue
            if len(b) < 4 + n:
                return
            body = bytes(b[2:3 + n])
            chk = b[3 + n]
            del b[:4 + n]
            if _chk(body) != chk:
                self.n_errors += 1
                continue
            self.n_packets += 1
            self._handle(sid, body[2], body[3:])

    def _handle(self, sid: int, instr: int, p: bytes):
        if instr == 0x01 and sid in self.mem:                                      # PING
            self.tx += _status(sid, 0)
        elif instr == 0x02 and sid in self.mem and len(p) == 2:                    # READ
            self._refresh(sid)
            self.tx += _status(sid, 0, bytes(self.mem[sid][p[0]:p[0] + p[1]]))
        elif instr == 0x03 and len(p) >= 2:                                        # WRITE
            for s in (self.ids if sid == 0xFE else [sid] if sid in self.mem else []):
                self.mem[s][p[0]:p[0] + len(p) - 1] = p[1:]
                self._apply(s)
                if sid != 0xFE:
                    self.tx += _status(s, 0)
        elif instr == 0x83 and len(p) >= 2:                                        # SYNC_WRITE
            addr, size = p[0], p[1]
            for i in range(2, len(p) - size, size + 1):
                s = p[i]
                if s in self.mem:
                    self.mem[s][addr:addr + size] = p[i + 1:i + 1 + size]
                    self._apply(s)
        elif instr == 0x82 and len(p) >= 2:                                        # SYNC_READ
            addr, size = p[0], p[1]
            for s in p[2:]:
                if s in self.mem:
                    self._refresh(s)
                    self.tx += _status(s, 0, bytes(self.mem[s][addr:addr + size]))

    def _word(self, sid: int, reg: str) -> int:
        a = REG[reg]
        return self.mem[sid][a] | self.mem[sid][a + 1] << 8

    def _apply(self, sid: int):
        """The registers -> the arm's goal, speed and torque."""
        k = self.ids.index(sid)
        m = self.mem[sid]
        tgt = self.arm.target.copy()
        tgt[k] = self._rad(k, self._word(sid, "goal_position")) + self.err[k]
        self.arm.set_pose(tgt)
        sp = self._word(sid, "goal_speed") & 0x7FFF
        self.arm.speed[k] = A.SERVO_SPEED if sp == 0 else min(A.SERVO_SPEED, sp * 2 * math.pi / TICKS)
        lim = (self._word(sid, "torque_limit") / 1000.0) if m[REG["torque_enable"]] else 0.0
        self.arm.torque_scale[k] = min(1.0, max(0.0, lim))

    def _refresh(self, sid: int):
        """The physics -> the present-value registers."""
        k = self.ids.index(sid)
        arm, m = self.arm, self.mem[sid]
        q = float(arm.d.qpos[arm.qadr[k]])
        w = float(arm.d.qvel[arm.vadr[k]])
        tau = float(arm.d.actuator_force[arm.a[k]])
        pos = int(round(2048 + (q - self.err[k]) * TICKS / (2 * math.pi)))
        m[REG["present_position"]:REG["present_position"] + 2] = max(0, min(TICKS - 1, pos)).to_bytes(2, "little")
        spd = _sm(int(round(w * TICKS / (2 * math.pi))), 15)
        m[REG["present_speed"]:REG["present_speed"] + 2] = spd.to_bytes(2, "little")
        load = _sm(int(round(1000 * tau / A.SERVO_STALL)), 10)
        m[REG["present_load"]:REG["present_load"] + 2] = load.to_bytes(2, "little")
        m[REG["present_voltage"]] = int(round(self.v_bus * 10)) & 0xFF
        m[REG["present_temperature"]] = 30
        m[REG["moving"]] = int(abs(w) > 0.05)
        cur = int(round((A.SERVO_I_IDLE + (A.SERVO_I_STALL - A.SERVO_I_IDLE) * min(1.0, abs(tau) / A.SERVO_STALL)) / 0.0065))
        m[REG["present_current"]:REG["present_current"] + 2] = cur.to_bytes(2, "little")


class ServoPort:
    """The bus board's USB port as a localhost TCP server (the ROS bridge; like omni_sim's McuPort): omni_arm's driver
    connects to it as it opens /dev/ttyUSB0 on the robot. exchange() once per firmware tick."""

    def __init__(self, bus: ServoBusSim, port: int = 5763, log=None):
        self.bus, self.port = bus, port
        self.log = log or (lambda m: None)
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(1)
        self.srv.setblocking(False)
        self.conn = None

    def exchange(self):
        if self.conn is None:
            try:
                self.conn, _ = self.srv.accept()
                self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.conn.setblocking(False)
                self.log(f"servo bus port {self.port}: the arm driver connected")
            except BlockingIOError:
                return
        try:
            while True:
                d = self.conn.recv(65536)
                if not d:
                    raise ConnectionError
                self.bus.write(d)
        except BlockingIOError:
            pass
        except (ConnectionError, OSError):
            self.log(f"servo bus port {self.port}: the arm driver disconnected")
            self.conn.close()
            self.conn = None
            return
        out = self.bus.read(1 << 16)
        if out:
            try:
                self.conn.sendall(out)
            except OSError:
                self.conn.close()
                self.conn = None

    def close(self):
        if self.conn is not None:
            self.conn.close()
        self.srv.close()
