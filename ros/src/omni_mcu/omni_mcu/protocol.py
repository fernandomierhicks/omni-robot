"""The Jetson side of the Jetson <-> MCU link (design.md 5, 30). Mirror of firmware/lib/omni_fw/protocol.h; no ROS here,
so the sim's in-process harness and the link node share it. test_v11.py checks it against the firmware's C++ through
the SIL library.

Frame:   COBS( type | seq | body | crc16 ) 0x00      crc16 = CRC-16/CCITT-FALSE over type | seq | body, little-endian
  CMD   0x01  Jetson -> MCU (100 Hz; also the heartbeat)
  STATE 0x81  MCU -> Jetson (200 Hz)
  IMU   0x82  MCU -> Jetson (every BNO085 sample, 400 Hz; protocol 4: stamped with the sample's own time)
  INFO  0x83  MCU -> Jetson (10 Hz): protocol version, firmware build id, the RC receiver
  TIME  0x84  MCU -> Jetson (protocol 4): an edge of a time reference: the 1 Hz sync pulse, a camera trigger, the
              left camera's measured exposure (design.md 10)

Time: the MCU stamps everything with its own clock (microseconds since boot), where it happened. TimeSync
(clock_sync.ClockSync) maps it onto the Jetson's clock: a line t_host = a t_mcu + b from the sync pulses the Jetson
also stamps on a GPIO (HostLink.add_sync_edge), else from the CMD / STATE echo (NTP-style: the fastest exchange of
each second).
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, fields

from omni_mcu.clock_sync import ClockSync

MSG_CMD, MSG_STATE, MSG_IMU, MSG_INFO, MSG_TIME = 0x01, 0x81, 0x82, 0x83, 0x84
TE_SYNC, TE_CAM_TRIGGER, TE_CAM_STROBE = 1, 2, 3
TIME_KINDS = {TE_SYNC: "sync", TE_CAM_TRIGGER: "cam_trigger", TE_CAM_STROBE: "cam_strobe"}
PROTOCOL_VERSION = 4             # protocol.h PROTOCOL_VERSION: bump both with any body change
REQ_DISARM, REQ_AUTO = 0, 1
PROFILE_ADULT, PROFILE_KID = 0, 1
DRIVE_IDLE, DRIVE_VEL_BODY, DRIVE_GOTO = 0, 1, 2
CMD_HOLD_YAW = 1
MODES = {0: "BOOT", 1: "DISARMED", 2: "AUTO", 3: "MANUAL", 4: "ESTOP"}
FAULTS = {1: "LINK_LOST", 2: "ESTOP", 4: "BUMPER", 8: "MOTOR_ERR", 16: "MOTOR_SILENT", 32: "LOW_BATT",
          64: "IMU_SILENT", 128: "FLY_TELEM"}
FEED_STATES = {0: "IDLE", 1: "SEAT", 2: "RUN", 3: "DONE"}

CMD_FMT = struct.Struct("<Q4B6fQ3fQ2fH")
STATE_FMT = struct.Struct("<3Q4BH4B9f3f2f3f2B2HfHH")
IMU_FMT = struct.Struct("<Q4f3f3f")
INFO_FMT = struct.Struct("<QH12s2B8h4B4B")
TIME_FMT = struct.Struct("<QB3xII")
assert (CMD_FMT.size, STATE_FMT.size, IMU_FMT.size, INFO_FMT.size, TIME_FMT.size) == (74, 116, 48, 48, 20), \
    "protocol.h changed"


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(raw: bytes) -> bytes:
    out = bytearray([0])
    code_at, code = 0, 1
    for b in raw:
        if b == 0:
            out[code_at] = code
            code_at = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_at] = code
                code_at = len(out)
                out.append(0)
                code = 1
    out[code_at] = code
    return bytes(out)


def cobs_decode(enc: bytes) -> bytes | None:
    out = bytearray()
    i, n = 0, len(enc)
    while i < n:
        code = enc[i]
        i += 1
        if code == 0 or i + code - 1 > n:
            return None
        out += enc[i:i + code - 1]
        i += code - 1
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def frame(msg_type: int, seq: int, body: bytes) -> bytes:
    raw = bytes([msg_type, seq & 0xFF]) + body
    c = crc16(raw)
    return cobs_encode(raw + bytes([c & 0xFF, c >> 8])) + b"\x00"


@dataclass
class Command:
    """What the Jetson asks of the MCU (CmdBody). Angles rad, speeds m/s, times are MCU microseconds."""
    req_mode: int = REQ_AUTO
    profile: int = PROFILE_ADULT
    drive_mode: int = DRIVE_IDLE
    flags: int = 0
    a: float = 0.0                # VEL_BODY vx | GOTO x
    b: float = 0.0                # VEL_BODY vy | GOTO y
    c: float = 0.0                # VEL_BODY wz | GOTO yaw
    v_end: float = 0.0
    a_cap: float = 0.0
    speed_cap: float = 0.0
    t_brake_us: int = 0
    coast_s: float = 0.0
    a_brake: float = 0.0
    tilt: float = 0.0
    t_tilt_us: int = 0
    tilt_after: float = 0.0
    fly_rpm: float = 0.0
    fire_seq: int = 0

    def pack(self, t_host_ns: int) -> bytes:
        return CMD_FMT.pack(int(t_host_ns), self.req_mode, self.profile, self.drive_mode, self.flags,
                            self.a, self.b, self.c, self.v_end, self.a_cap, self.speed_cap, int(self.t_brake_us),
                            self.coast_s, self.a_brake, self.tilt, int(self.t_tilt_us), self.tilt_after, self.fly_rpm,
                            self.fire_seq & 0xFFFF)


@dataclass
class State:
    t_mcu_us: int
    echo_t_host_ns: int
    echo_rx_us: int
    mode: int
    profile: int
    drive_mode: int
    feed_state: int
    faults: int
    dm_err: tuple
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    slip: float
    roll: float
    pitch: float
    v_cmd_x: float
    v_cmd_y: float
    w_cmd: float
    fly_rpm: tuple
    fly_target_rpm: float
    tilt: float
    tilt_cmd: float
    beams: int
    fly_ready: int
    fire_seq_done: int
    fire_rejected: int
    v_bus: float
    cmd_count: int
    rx_errors: int
    t_host: float = math.nan      # filled by HostLink: t_mcu_us on the Jetson's clock (s)

    @classmethod
    def unpack(cls, body: bytes) -> "State":
        v = STATE_FMT.unpack(body)
        return cls(*v[:8], tuple(v[8:12]), *v[12:24], tuple(v[24:26]), *v[26:])

    @property
    def ball_held(self) -> bool:
        return bool(self.beams & 1)

    @property
    def mode_name(self) -> str:
        return MODES.get(self.mode, str(self.mode))

    def fault_names(self) -> list:
        return [n for b, n in FAULTS.items() if self.faults & b]


@dataclass
class Imu:
    t_mcu_us: int
    quat: tuple          # w x y z
    gyro: tuple
    accel: tuple
    t_host: float = math.nan

    @classmethod
    def unpack(cls, body: bytes) -> "Imu":
        v = IMU_FMT.unpack(body)
        return cls(v[0], tuple(v[1:5]), tuple(v[5:8]), tuple(v[8:11]))


@dataclass
class Info:
    t_mcu_us: int
    protocol: int        # the firmware's PROTOCOL_VERSION
    build: str           # its build id (firmware/fw_build_id.py)
    rc_ok: bool          # the RC receiver's link is up
    rc_manual: bool      # its switch asks for MANUAL
    rc: tuple            # 8 channels, -1..1
    t_rotor: tuple = (0, 0, 0, 0)    # protocol 3: each drive motor's winding and driver temperature, deg C
    t_mos: tuple = (0, 0, 0, 0)
    t_host: float = math.nan

    @classmethod
    def unpack(cls, body: bytes) -> "Info":
        v = INFO_FMT.unpack(body)
        return cls(v[0], v[1], v[2].split(b"\0", 1)[0].decode("ascii", "replace"), bool(v[3]), bool(v[4]),
                   tuple(c / 1000.0 for c in v[5:13]), tuple(v[13:17]), tuple(v[17:21]))


@dataclass
class TimeEvent:
    """A time reference's edge (TimeBody): kind TE_*, seq, the MCU time and, mapped, the Jetson's."""
    t_mcu_us: int
    kind: int
    seq: int
    value: int                   # CAM_STROBE: the exposure's length, us
    t_host: float = math.nan

    @classmethod
    def unpack(cls, body: bytes) -> "TimeEvent":
        return cls(*TIME_FMT.unpack(body))

    @property
    def kind_name(self) -> str:
        return TIME_KINDS.get(self.kind, str(self.kind))


TimeSync = ClockSync             # the link's clock map (clock_sync.py; offset + drift, pulses or echoes)


@dataclass
class LinkStats:
    frames: int = 0
    errors: int = 0
    states: int = 0
    imus: int = 0
    infos: int = 0
    cmds: int = 0
    times: int = 0


class HostLink:
    """Bytes <-> messages for the Jetson side. feed() with what arrived (and when, on the Jetson's clock); encode()
    the next CMD with the Jetson's send time."""

    def __init__(self, **sync_kw):
        self.buf = bytearray()
        self.seq = 0
        self.sync = TimeSync(**sync_kw)
        self.state: State | None = None
        self.info: Info | None = None
        self.stats = LinkStats()

    def encode(self, cmd: Command, t_host_ns: int) -> bytes:
        self.seq = (self.seq + 1) & 0xFF
        self.stats.cmds += 1
        return frame(MSG_CMD, self.seq, cmd.pack(t_host_ns))

    def add_sync_edge(self, t_host_ns: int):
        """The Jetson's GPIO stamped the MCU's SYNC_OUT edge (this clock)."""
        self.sync.add_host_edge(t_host_ns)

    def feed(self, data: bytes, t_host_ns: int) -> list:
        out = []
        self.buf += data
        while True:
            k = self.buf.find(0)
            if k < 0:
                break
            enc = bytes(self.buf[:k])
            del self.buf[:k + 1]
            if not enc:
                continue
            raw = cobs_decode(enc)
            if raw is None or len(raw) < 4 or crc16(raw[:-2]) != (raw[-2] | raw[-1] << 8):
                self.stats.errors += 1
                continue
            t, body = raw[0], raw[2:-2]
            self.stats.frames += 1
            if t == MSG_STATE and len(body) == STATE_FMT.size:
                s = State.unpack(body)
                self.sync.add(s.echo_t_host_ns, s.echo_rx_us, s.t_mcu_us, t_host_ns)
                s.t_host = self.sync.to_host_s(s.t_mcu_us)
                self.state = s
                self.stats.states += 1
                out.append(s)
            elif t == MSG_IMU and len(body) == IMU_FMT.size:
                m = Imu.unpack(body)
                m.t_host = self.sync.to_host_s(m.t_mcu_us)
                self.stats.imus += 1
                out.append(m)
            elif t == MSG_INFO and len(body) == INFO_FMT.size:
                m = Info.unpack(body)
                m.t_host = self.sync.to_host_s(m.t_mcu_us)
                self.info = m
                self.stats.infos += 1
                out.append(m)
            elif t == MSG_TIME and len(body) == TIME_FMT.size:
                m = TimeEvent.unpack(body)
                if m.kind == TE_SYNC:
                    self.sync.add_mcu_pulse(m.seq, m.t_mcu_us)
                m.t_host = self.sync.to_host_s(m.t_mcu_us)
                self.stats.times += 1
                out.append(m)
            else:
                self.stats.errors += 1
        if len(self.buf) > 4096:                  # garbage with no delimiter
            self.buf.clear()
            self.stats.errors += 1
        return out


@dataclass
class Request:
    """A Command with its times on the Jetson's clock (omni_interfaces/McuCommand in Python): what the mode manager
    asks for; to_command() converts it with the link's time sync."""
    req_mode: int = REQ_DISARM
    kid: bool = False
    drive_mode: int = DRIVE_IDLE
    hold_yaw: bool = False
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    v_end: float = 0.0
    a_cap: float = 0.0
    speed_cap: float = 0.0
    t_brake: float = 0.0          # s, Jetson clock; 0 = none
    coast_s: float = 0.0
    a_brake: float = 0.0
    tilt: float = 0.0
    t_tilt: float = 0.0           # s, Jetson clock; 0 = none
    tilt_after: float = 0.0
    fly_rpm: float = 0.0
    fire_seq: int = 0


def to_command(r: Request, sync: TimeSync) -> Command:
    return Command(req_mode=r.req_mode, profile=PROFILE_KID if r.kid else PROFILE_ADULT, drive_mode=r.drive_mode,
                   flags=CMD_HOLD_YAW if r.hold_yaw else 0, a=r.a, b=r.b, c=r.c, v_end=r.v_end, a_cap=r.a_cap,
                   speed_cap=r.speed_cap, t_brake_us=sync.to_mcu_us(r.t_brake) if r.t_brake else 0,
                   coast_s=r.coast_s, a_brake=r.a_brake, tilt=r.tilt,
                   t_tilt_us=sync.to_mcu_us(r.t_tilt) if r.t_tilt else 0, tilt_after=r.tilt_after, fly_rpm=r.fly_rpm,
                   fire_seq=r.fire_seq & 0xFFFF)


def command_fields() -> list:
    return [f.name for f in fields(Command)]
