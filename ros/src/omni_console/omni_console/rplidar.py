"""rplidar: a Slamtec RPLIDAR on a serial port, ROS-free and SDK-free, for the console's Lidar page (camera_snap
serves it) and bench checks. What the ROS driver (sllidar_ros2) hides is the point: the device's model, firmware,
hardware revision, serial number and health, its scan modes, the motor's PWM, and the raw turn rate, sample count and
framing errors.

Protocol (Slamtec's interface protocol and SDK): a request is A5 CMD [len payload checksum]; an answer starts with a
7-byte descriptor A5 5A len(30 bits) mode(2 bits) type. The scan modes the device lists (GET_LIDAR_CONF) answer in
one of three formats, all decoded here:

    0x81  standard samples, 5 bytes each (the Standard mode, the SCAN command)
    0x82  express capsules, 84 bytes, 32 samples (Express on the A2)
    0x84  ultra capsules, 132 bytes, 96 samples (Boost, Stability, Sensitivity)

Angles are the lidar's own: degrees clockwise seen from above, 0 at its forward mark (the ROS driver flips them
counter-clockwise for /scan). Ranges in mm, 0 = no return. The motor: A2 and later run it by a PWM (0-1023, 660
default) with DTR cleared; the A2M8's adapter runs the motor whatever the PWM while DTR is raised (Nov 13,
ros/tools/lidar_motor_test.py: raised + PWM 0 kept 9.5 turns/s, cleared + PWM 0 stopped), and pyserial raises DTR
when it opens a port. A1 adapters switch the motor by DTR alone (the SDK's: cleared runs it, raised stops it).

    python rplidar.py [/dev/ttyUSB0] [--seconds 3] [--mode Boost]      # info, health, modes, a few turns
"""
from __future__ import annotations

import os
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field

# requests
STOP, RESET, SCAN, EXPRESS_SCAN, GET_INFO, GET_HEALTH, GET_SAMPLERATE, GET_CONF, SET_PWM = (
    0x25, 0x40, 0x20, 0x82, 0x50, 0x52, 0x59, 0x84, 0xF0)
# GET_LIDAR_CONF keys
CONF_COUNT, CONF_US_PER_SAMPLE, CONF_MAX_DIST, CONF_ANS_TYPE, CONF_TYPICAL, CONF_NAME = (
    0x70, 0x71, 0x74, 0x75, 0x7C, 0x7F)
ANS_STANDARD, ANS_EXPRESS, ANS_ULTRA = 0x81, 0x82, 0x84
BAUDS = (115200, 256000, 1000000)        # A1 / A2M8; A2M12, A3, S1; S2: tried in this order
DEFAULT_PWM = 660
SOFT_START_PWM = 200                     # the motor starts here and ramps to its PWM over the spin-up (gentler inrush)
NO_DATA_S = 1.0                          # s without a byte before a scan says "no data" (a turn is ~80 ms; Boost
                                         # pauses ~0.3-0.5 s before its first capsule after a mode change, Nov 12)
HEALTH = ("good", "warning", "error")


def model_name(model: int) -> str:
    """0x28 -> A2M8, 0x2C -> A2M12 (the A series: high nibble the series, low the sub-model); others as a number."""
    major, sub = model >> 4, model & 0xF
    return f"A{major}M{sub}" if 1 <= major <= 3 else f"model 0x{model:02X}"


class LidarError(RuntimeError):
    pass


def request(cmd: int, payload: bytes = b"") -> bytes:
    if not payload:
        return bytes((0xA5, cmd))
    pkt = bytes((0xA5, cmd, len(payload))) + payload
    cs = 0
    for b in pkt:
        cs ^= b
    return pkt + bytes((cs,))


# ---------------------------------------------------------------- the three answer formats

class StandardParser:
    """0x81: 5 bytes a sample. byte 0: quality << 2 | !S << 1 | S; byte 1: angle_q6 low 7 bits << 1 | check bit 1;
    byte 2: angle_q6 high; bytes 3-4: range in mm * 4. A byte that breaks the pattern is skipped (counted bad)."""
    size = 5

    def __init__(self):
        self.buf = b""
        self.bad = 0

    def feed(self, data: bytes) -> list[tuple[float, float, int, bool]]:
        buf, out, i = self.buf + data, [], 0
        while len(buf) - i >= 5:
            b0, b1 = buf[i], buf[i + 1]
            s = b0 & 1
            if s == (b0 >> 1) & 1 or not b1 & 1:
                self.bad += 1
                i += 1
                continue
            ang = ((b1 >> 1) | (buf[i + 2] << 7)) / 64.0
            dist = (buf[i + 3] | (buf[i + 4] << 8)) / 4.0
            out.append((ang, dist, b0 >> 2, bool(s)))
            i += 5
        self.buf = buf[i:]
        return out


class _CapsuleParser:
    """Capsules: sync nibbles 0xA, 0x5 in the first two bytes' high halves, their low halves the XOR of the rest.
    A capsule's samples need the next capsule's start angle, so each is decoded when the next arrives."""
    size = 0

    def __init__(self):
        self.buf = b""
        self.bad = 0
        self.prev: bytes | None = None

    def feed(self, data: bytes) -> list[tuple[float, float, int, bool]]:
        buf, out, i, n = self.buf + data, [], 0, self.size
        while len(buf) - i >= n:
            c = buf[i:i + n]
            if c[0] >> 4 != 0xA or c[1] >> 4 != 0x5:
                self.bad += 1
                i += 1
                continue
            cs = 0
            for b in c[2:]:
                cs ^= b
            if cs != ((c[0] & 0xF) | ((c[1] & 0xF) << 4)):
                self.bad += 1
                self.prev = None
                i += 1
                continue
            i += n
            start = c[2] | (c[3] << 8)
            if start & 0x8000:                    # the first capsule of a new scan: nothing to pair with
                self.prev = None
            if self.prev is not None:
                out.extend(self.decode(self.prev, c))
            self.prev = c
        self.buf = buf[i:]
        return out

    @staticmethod
    def _angles(prev: bytes, cur: bytes, k: int):
        """The first sample's raw angle and the step between samples (q16 degrees), k samples a capsule."""
        a_cur = ((cur[2] | (cur[3] << 8)) & 0x7FFF) << 2
        a_prev = ((prev[2] | (prev[3] << 8)) & 0x7FFF) << 2
        diff = a_cur - a_prev
        if a_prev > a_cur:
            diff += 360 << 8
        return a_prev << 8, (diff << 8) // k

    def decode(self, prev: bytes, cur: bytes):
        raise NotImplementedError


def _wrap_q6(a: int) -> int:
    if a < 0:
        a += 360 << 6
    if a >= 360 << 6:
        a -= 360 << 6
    return a


class ExpressParser(_CapsuleParser):
    """0x82: start angle (q6, bit 15 = new scan), 16 cabins of 5 bytes: two ranges (mm in bits 15-2, the angle
    offsets' high bits in 1-0) and a byte of the two offsets' low nibbles (q3 degrees)."""
    size = 84

    def decode(self, prev, cur):
        raw, inc = self._angles(prev, cur, 32)
        out = []
        for pos in range(16):
            o = 4 + 5 * pos
            d1, d2, off = struct.unpack_from("<HHB", prev, o)
            for d, a_off in ((d1, (off & 0xF) | ((d1 & 3) << 4)), (d2, (off >> 4) | ((d2 & 3) << 4))):
                ang = _wrap_q6((raw - (a_off << 13)) >> 10)
                sync = ((raw + inc) % (360 << 16)) < inc
                raw += inc
                out.append((ang / 64.0, float(d >> 2), 47 if d >> 2 else 0, sync))
        return out


def _varbitscale(scaled: int) -> tuple[int, int]:
    for base, lvl, target in ((3328, 4, 1 << 14), (1792, 3, 1 << 12), (1280, 2, 1 << 11), (512, 1, 1 << 9), (0, 0, 0)):
        if scaled >= base:
            return target + ((scaled - base) << lvl), lvl
    return 0, 0


def _signed10(v: int) -> int:
    v &= 0x3FF
    return v - 0x400 if v & 0x200 else v


class UltraParser(_CapsuleParser):
    """0x84: start angle, 32 cabins of 32 bits, three samples each: a major range (12 bits, variable-bit scaled)
    and two signed 10-bit predictions off it and the next cabin's major. The angle offset follows the range
    (the SDK's _ultraCapsuleToNormal)."""
    size = 132

    def decode(self, prev, cur):
        raw, inc = self._angles(prev, cur, 96)
        cab = struct.unpack_from("<32I", prev, 4)
        nxt0 = struct.unpack_from("<I", cur, 4)[0]
        out = []
        for pos in range(32):
            x = cab[pos]
            major, lvl1 = _varbitscale(x & 0xFFF)
            major2, lvl2 = _varbitscale((cab[pos + 1] if pos < 31 else nxt0) & 0xFFF)
            p1, p2 = _signed10(x >> 12), _signed10(x >> 22)
            base1, base2 = major, major2
            if not major and major2:
                base1, lvl1 = major2, lvl2
            d = [major << 2,
                 0 if p1 in (-512, 511) else ((p1 << lvl1) + base1) << 2,
                 0 if p2 in (-512, 511) else ((p2 << lvl2) + base2) << 2]
            for dq2 in d:
                sync = ((raw + inc) % (360 << 16)) < inc
                off = int(7.5 * 3.1415926535 * (1 << 16) / 180.0)
                if dq2 >= 50 * 4:
                    k2 = 98361 // dq2
                    off = int(8 * 3.1415926535 * (1 << 16) / 180) - (k2 << 6) - (k2 * k2 * k2) // 98304
                ang = _wrap_q6((raw - int(off * 180 / 3.14159265)) >> 10)
                raw += inc
                mm = max(dq2, 0) / 4.0
                out.append((ang / 64.0, mm, 47 if mm else 0, sync))
        return out


PARSERS = {ANS_STANDARD: StandardParser, ANS_EXPRESS: ExpressParser, ANS_ULTRA: UltraParser}


# ---------------------------------------------------------------- turns

@dataclass
class Turn:
    """One revolution: samples in the order measured. t: host monotonic time it completed."""
    seq: int
    t: float
    angle: list[float] = field(default_factory=list)       # deg, clockwise
    dist: list[float] = field(default_factory=list)        # mm, 0 = no return
    quality: list[int] = field(default_factory=list)

    def stats(self) -> dict:
        r = sorted(d for d in self.dist if d > 0)
        n = len(self.dist)
        out = dict(samples=n, valid=round(len(r) / n, 3) if n else 0.0)
        if r:
            out.update(min_m=round(r[0] / 1000, 3), median_m=round(r[len(r) // 2] / 1000, 3), max_m=round(r[-1] / 1000, 3))
            k = min(range(n), key=lambda j: self.dist[j] if self.dist[j] > 0 else 1e9)
            out["nearest_deg"] = round(self.angle[k], 1)
        return out


class TurnAssembler:
    """Samples into turns at each start flag. A first partial turn (the scan started mid-revolution) is dropped."""

    def __init__(self, on_turn, seq: int = 0):
        """seq: the last turn number handed out (a restarted scan carries on from it, so readers waiting for a
        newer turn than theirs aren't stuck)."""
        self.on_turn, self.cur, self.started, self.seq = on_turn, Turn(0, 0.0), False, seq

    def add(self, samples, t: float):
        for ang, dist, q, start in samples:
            if start:
                if self.started and len(self.cur.angle) > 10:
                    self.seq += 1
                    self.cur.seq, self.cur.t = self.seq, t
                    self.on_turn(self.cur)
                self.cur, self.started = Turn(0, 0.0), True
            self.cur.angle.append(ang)
            self.cur.dist.append(dist)
            self.cur.quality.append(q)


def decode_bytes(ans_type: int, data: bytes) -> list[Turn]:
    """Every whole turn in a recording of a scan's bytes (after its descriptor): fixtures and offline checks."""
    p, turns = PARSERS[ans_type](), []
    asm = TurnAssembler(turns.append)
    asm.add(p.feed(data), 0.0)
    return turns


# ---------------------------------------------------------------- the device

class RPLidar:
    """One RPLIDAR on a serial port (pyserial or anything with its read / write / dtr / timeout). The constructor
    finds the baud rate (GET_INFO at each of BAUDS), stops any scan and reads the info, health and scan modes;
    start() spins the motor and scans in a thread that keeps the latest turn."""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int | None = None, opener=None):
        if opener is None:
            import serial

            def opener(p, b):                            # DTR cleared from the open on (raised, an A2 motor runs)
                s = serial.Serial(None, b, timeout=0.3)
                s.port, s.dtr = p, False
                s.open()
                return s
        self.port, self.opener = port, opener
        self.ser = None
        self.lock = threading.Lock()                     # the port: commands vs the reader
        self.cond = threading.Condition()
        self.turn: Turn | None = None
        self.thread: threading.Thread | None = None
        self.stop_ev = threading.Event()
        self.running: str | None = None                  # the scan mode's name while scanning
        self.starting = False                            # a (re)start under way (a mode change: the scan carries on)
        self.spinning = False                           # the motor on without a scan (Spin on the page)
        self.motor_t: float | None = None                # when the motor last started (monotonic)
        self.pwm = 0
        self.dtr_stops = False                           # an A1: DTR switches the motor (else it stays cleared)
        self.error: str | None = None
        self.dead: str | None = None                     # the port failed (unplugged, a USB reset): reopen
        self.bytes_in = self.bad = 0
        self.hist: list[tuple[float, int, float]] = []   # (t, samples, valid share) of recent turns
        self.last_bytes = b""
        self.path = os.path.realpath(port) if os.path.exists(port) else port     # the tty behind a by-id link
        bauds, seen = ([baud] if baud else BAUDS), []
        for b in bauds:
            self.ser = opener(port, b)
            try:
                self._motor(False)
                self._send(STOP)
                time.sleep(0.05)
                self._flush()
                self.info = self._info()
                self.baud = b
                break
            except LidarError:
                seen.append(self.last_bytes)
                self.ser.close()
                self.ser = None
        if self.ser is None:
            raise LidarError(f"no RPLIDAR answer on {port} at {', '.join(map(str, bauds))} baud{probe_hint(seen)}")
        if self.info["model"] >> 4 == 1:                 # an A1: its motor off is DTR raised
            self.dtr_stops = True
            self._motor(False)
        self.health = self._health()
        self.rates = self._samplerate()
        self.modes = self._modes()

    def gone(self) -> str | None:
        """Why this port is no use any more (None if it's fine): it failed, its tty was removed or the adapter came
        back as another one (a USB reset re-enumerates it: ttyUSB0 -> ttyUSB1), or it no longer answers the OS."""
        if self.dead:
            return self.dead
        if self.path.startswith("/dev/"):
            now = os.path.realpath(self.port) if os.path.exists(self.port) else None
            if now is None:
                return f"{self.path} is gone (the adapter dropped off USB)"
            if now != self.path:
                return f"the adapter dropped off USB and came back as {now} (it was {self.path})"
        if not self.running:                             # no reader thread on the port: ask the driver
            try:
                getattr(self.ser, "in_waiting", 0)
            except Exception as e:
                return f"the port stopped answering ({type(e).__name__}: {e})"
        return None

    # -- the wire
    def _send(self, cmd: int, payload: bytes = b""):
        self.ser.write(request(cmd, payload))

    def _flush(self):
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def _read(self, n: int, timeout: float = 1.0) -> bytes:
        buf, t_end = b"", time.monotonic() + timeout
        while len(buf) < n and time.monotonic() < t_end:
            buf += self.ser.read(n - len(buf))
        return buf

    def _answer(self, cmd: int, payload: bytes = b"", timeout: float = 1.0) -> tuple[int, bytes]:
        """A single answer: (its type, its body)."""
        self._send(cmd, payload)
        d = self.last_bytes = self._read(7, timeout)
        if len(d) < 7 or d[0] != 0xA5 or d[1] != 0x5A:
            raise LidarError(f"no answer to 0x{cmd:02X} ({len(d)} bytes: {d.hex()})")
        size = struct.unpack("<I", d[2:6])[0] & 0x3FFFFFFF
        body = self._read(size, timeout)
        if len(body) < size:
            raise LidarError(f"short answer to 0x{cmd:02X}")
        return d[6], body

    def _motor(self, on: bool, pwm: int = DEFAULT_PWM):
        try:
            self.ser.dtr = (not on) if self.dtr_stops else False    # A2+: raised runs the motor regardless (top)
        except Exception:
            pass
        self._send(SET_PWM, struct.pack("<H", int(pwm) if on else 0))
        self.pwm = int(pwm) if on else 0

    def _spin_up(self, pwm: int, seconds: float):
        """The motor on, from SOFT_START_PWM up to pwm over `seconds` (a stalled motor's start draws the most; a
        gentler start may keep a weak 5 V supply from dipping. Unmeasured: the A2M8's draw wasn't logged)."""
        pwm = max(0, min(int(pwm), 1023))
        lo = min(pwm, SOFT_START_PWM)
        self.motor_t = time.monotonic()
        self._motor(True, lo)
        steps = max(1, round(seconds / 0.1))
        for i in range(1, steps + 1):
            time.sleep(seconds / steps)
            self._send(SET_PWM, struct.pack("<H", round(lo + (pwm - lo) * i / steps)))
        self.pwm = pwm

    def _info(self) -> dict:
        _, b = self._answer(GET_INFO, timeout=0.4)
        if len(b) != 20:
            raise LidarError("bad GET_INFO answer")
        return dict(model=b[0], name=model_name(b[0]), firmware=f"{b[2]}.{b[1]:02d}", hardware=b[3],
                    serial=b[4:].hex().upper())

    def _health(self) -> dict:
        _, b = self._answer(GET_HEALTH)
        st, code = b[0], struct.unpack("<H", b[1:3])[0]
        return dict(status=HEALTH[st] if st < 3 else str(st), code=code)

    def _samplerate(self) -> dict:
        try:
            _, b = self._answer(GET_SAMPLERATE)
            std, exp = struct.unpack("<HH", b[:4])
            return dict(standard_us=std, express_us=exp)
        except (LidarError, struct.error):
            return {}

    def _conf(self, key: int, mode: int | None = None) -> bytes:
        payload = struct.pack("<I", key) + (b"" if mode is None else struct.pack("<H", mode))
        _, b = self._answer(GET_CONF, payload, timeout=0.5)
        if len(b) < 4 or struct.unpack("<I", b[:4])[0] != key:
            raise LidarError("bad GET_LIDAR_CONF answer")
        return b[4:]

    def _modes(self) -> list[dict]:
        """The scan modes the device lists; firmware without GET_LIDAR_CONF: Standard and (if it says) Express."""
        self.has_conf = False
        try:
            n = struct.unpack("<H", self._conf(CONF_COUNT)[:2])[0]
            typical = struct.unpack("<H", self._conf(CONF_TYPICAL)[:2])[0]
            out = []
            for m in range(n):
                us = struct.unpack("<I", self._conf(CONF_US_PER_SAMPLE, m)[:4])[0] / 256
                out.append(dict(id=m, name=self._conf(CONF_NAME, m).split(b"\0")[0].decode(errors="replace"),
                                us=round(us, 1), rate=round(1e6 / us) if us else 0,
                                max_m=round(struct.unpack("<I", self._conf(CONF_MAX_DIST, m)[:4])[0] / 256, 1),
                                answer=self._conf(CONF_ANS_TYPE, m)[0], typical=m == typical))
            self.has_conf = True
            return out
        except (LidarError, struct.error, IndexError):
            self._send(STOP)
            time.sleep(0.05)
            self._flush()
            out = [dict(id=0, name="Standard", us=self.rates.get("standard_us"), answer=ANS_STANDARD, typical=True)]
            if self.rates.get("express_us"):
                out.append(dict(id=1, name="Express", us=self.rates["express_us"], answer=ANS_EXPRESS, typical=False))
            for m in out:
                m["rate"] = round(1e6 / m["us"]) if m["us"] else 0
                m["max_m"] = None
            return out

    # -- scanning
    def mode(self, name: str | None) -> dict:
        if name is None:
            return next((m for m in self.modes if m["typical"]), self.modes[0])
        for m in self.modes:
            if m["name"].lower() == str(name).lower():
                return m
        raise LidarError(f"no scan mode {name!r} (this one has {', '.join(m['name'] for m in self.modes)})")

    def start(self, mode: str | None = None, pwm: int = DEFAULT_PWM, spin_up_s: float = 1.0):
        self.starting = True
        try:
            self._start(mode, pwm, spin_up_s)
        finally:
            self.starting = False

    def _start(self, mode, pwm, spin_up_s):
        self.stop()
        m = self.mode(mode)
        if m["answer"] not in PARSERS:
            raise LidarError(f"{m['name']} answers in format 0x{m['answer']:02X}, which this driver can't read")
        with self.lock:
            self._spin_up(pwm, spin_up_s)
            self._flush()
            if m["answer"] == ANS_STANDARD and m["id"] == 0:
                self._send(SCAN)
            else:                                          # working mode, flags, parameter
                self._send(EXPRESS_SCAN, struct.pack("<BHH", m["id"] if self.has_conf else 0, 0, 0))   # 0: legacy express
            d = self._read(7, 2.0)
            if len(d) < 7 or d[:2] != b"\xA5\x5A":
                self._send(STOP)
                self._motor(False)
                raise LidarError(f"the {m['name']} scan didn't start ({d.hex()})")
        self.running, self.error, self.bytes_in, self.bad, self.hist = m["name"], None, 0, 0, []
        self.stop_ev.clear()
        self.thread = threading.Thread(target=self._run, args=(PARSERS[d[6]](),), daemon=True)
        self.thread.start()

    def spin(self, on: bool, pwm: int = DEFAULT_PWM, spin_up_s: float = 1.0):
        """The motor alone, no scan: does it turn, and does the USB survive its start? (A scan stops first.)"""
        if self.running:
            self.stop()
        with self.lock:
            if on:
                self._spin_up(pwm, spin_up_s)
            else:
                self._motor(False)
        self.spinning = on

    def set_pwm(self, pwm: int):
        """The motor's speed while it runs (A2 and later; the turn rate follows in a second or two)."""
        with self.lock:
            self._send(SET_PWM, struct.pack("<H", max(0, min(int(pwm), 1023))))
            self.pwm = max(0, min(int(pwm), 1023))

    def _run(self, parser):
        asm = TurnAssembler(self._publish, self.turn.seq if self.turn else 0)
        last = time.monotonic()                            # the last byte (a read times out after 0.3 s)
        while not self.stop_ev.is_set():
            try:                                           # what's waiting (else one byte, up to the timeout), so a
                n = getattr(self.ser, "in_waiting", 0)     # turn's time is when its bytes came, not a big read's end
                data = self.ser.read(min(max(n, 1), 8192))
            except Exception as e:                         # unplugged, a USB reset: the session reopens the port
                self.error = self.dead = f"{type(e).__name__}: {e}"
                with self.cond:
                    self.cond.notify_all()
                return
            if not data:
                if time.monotonic() - last > NO_DATA_S:
                    self.error = "no data from the lidar (motor stopped? cable?)"
                continue
            last = time.monotonic()
            self.bytes_in += len(data)
            asm.add(parser.feed(data), time.monotonic())
            self.bad = parser.bad

    def _publish(self, turn: Turn):
        with self.cond:
            self.turn = turn
            self.error = None
            st = turn.stats()
            self.hist = [h for h in self.hist if turn.t - h[0] < 5.0] + [(turn.t, st["samples"], st["valid"])]
            self.cond.notify_all()

    def wait(self, after: int = 0, timeout: float = 2.0) -> Turn | None:
        with self.cond:
            self.cond.wait_for(lambda: (self.turn is not None and self.turn.seq > after) or self.stop_ev.is_set(), timeout)
            t = self.turn
        return t if t is not None and t.seq > after else None

    def rate(self) -> dict:
        """Turn rate (Hz), samples a turn and a second, valid share: over the last 5 s of turns."""
        h = self.hist
        if len(h) < 2:
            return {}
        dt = h[-1][0] - h[0][0]
        n = sum(x[1] for x in h[1:])
        return dict(hz=round((len(h) - 1) / dt, 2) if dt > 0 else None, samples_per_s=round(n / dt) if dt > 0 else None,
                    samples_per_turn=round(n / (len(h) - 1)), valid=round(sum(x[2] for x in h) / len(h), 3))

    def stop(self):
        if self.thread is not None:
            self.stop_ev.set()
            with self.cond:
                self.cond.notify_all()
            self.thread.join(2.0)
            self.thread = None
        self.running, self.spinning = None, False
        with self.lock:
            if self.ser is not None:
                self._send(STOP)
                time.sleep(0.02)
                self._motor(False)
                time.sleep(0.05)
                self._flush()

    def refresh_health(self) -> dict:
        """Only while not scanning (a request would end the scan)."""
        if self.running:
            return self.health
        with self.lock:
            self.health = self._health()
        return self.health

    def reset(self):
        """The device's own reboot (it prints its banner; the next request waits it out)."""
        self.stop()
        with self.lock:
            self._send(RESET)
            time.sleep(1.0)
            self._flush()
            self.info, self.health = self._info(), self._health()

    def close(self):
        try:
            self.stop()
        finally:
            if self.ser is not None:
                try:
                    self.ser.close()
                finally:
                    self.ser = None


def probe_hint(seen: list[bytes]) -> str:
    """What the bytes that came back instead of an answer say (one reply per baud tried)."""
    if seen and all(seen) and all(not any(b) for b in seen):
        return (": only 0x00 bytes at every baud, so the lidar's TX line is held low. The lidar side is unpowered or "
                "its cable to the USB adapter is loose (the adapter itself is fine)")
    if seen and not any(seen):
        return ": nothing came back at all (the lidar unpowered, or its cable to the adapter?)"
    return ""


# ---------------------------------------------------------------- the console's session (camera_snap serves it)

RETRY_S = 3.0
REAPPEAR_S = 3.0          # after a USB reset, how long to wait for the adapter to come back before retrying
EVENTS = 40


def find_port() -> str | None:
    """The lidar's serial port: a CP210x adapter by id (the A-series' USB adapter), else the first ttyUSB."""
    import glob
    for pat in ("/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*Silicon_Labs*", "/dev/ttyUSB*"):
        hit = sorted(glob.glob(pat))
        if hit:
            return hit[0]
    return None


def holders(path: str) -> list[str]:
    """Other processes with the port open (Linux /proc): a serial port isn't exclusive, and two readers garble it."""
    import os
    try:
        real = os.path.realpath(path)
        me = os.getpid()
        out = []
        for pid in os.listdir("/proc"):
            if not pid.isdigit() or int(pid) == me:
                continue
            try:
                fds = os.listdir(f"/proc/{pid}/fd")
            except OSError:
                continue
            for fd in fds:
                try:
                    if os.path.realpath(f"/proc/{pid}/fd/{fd}") == real:
                        with open(f"/proc/{pid}/comm") as f:
                            out.append(f"{f.read().strip()} (pid {pid})")
                        break
                except OSError:
                    continue
        return out
    except OSError:
        return []


class LidarSession:
    """The lidar as the Lidar page sees it: opened at the first request (info, health, modes; motor off), scanning
    when asked, and stopped and closed after idle_s seconds without a request, so the motor never runs for nobody
    and the ROS driver can have the port back. A port that failed or re-enumerated (a USB reset, often the motor's
    start dipping the 5 V) is noticed at the next request and opened again; what happened is kept as events."""

    def __init__(self, port: str | None = None, baud: int | None = None, idle_s: float = 20.0, fake: bool = False):
        self.port_arg, self.baud, self.idle_s, self.fake = port, baud, idle_s, fake
        self.opener = (lambda p, b: FakeSerial(p, b)) if fake else None
        self.lock = threading.Lock()
        self.dev: RPLidar | None = None
        self.port: str | None = port
        self.error: str | None = None
        self.used = self.failed = 0.0
        self.events: deque = deque(maxlen=EVENTS)        # (wall time, level, text): what happened, for the page
        self._said: str | None = None                    # the last device error logged (each said once)
        self.last_test: dict | None = None               # self_test()'s result

    def _event(self, text: str, level: str = "info"):
        self.events.append((time.time(), level, text))

    def _open(self) -> RPLidar:
        now = self.used = time.monotonic()
        if self.dev is not None:
            why = self.dev.gone()
            if why is None:
                return self.dev
            self._lost(why)
        if self.error and now - self.failed < RETRY_S:
            raise LidarError(self.error)
        try:
            self.port = "fake" if self.fake else (self.port_arg or find_port())
            if not self.port:
                raise LidarError("no lidar serial port (no CP210x adapter, no /dev/ttyUSB*): is it plugged in?")
            others = [] if self.fake else holders(self.port)
            if others:
                ros = any("lidar" in o or "ros" in o for o in others)
                raise LidarError(f"the lidar's port is in use by {', '.join(others)}: "
                                 + ("the robot's ROS lidar driver has it. Stop the robot's stack (Robot > System > Stop) to "
                                    "test the lidar here; while it runs, its /scan shows at the bottom of this page"
                                    if ros else "stop that program first (one reader at a time)"))
            self.dev = RPLidar(self.port, self.baud, opener=self.opener)
        except Exception as e:
            err = str(e) if isinstance(e, LidarError) else f"{type(e).__name__}: {e}"
            if err != self.error:
                self._event(f"can't open the lidar: {err}", "error")
            self.error, self.failed = err, now
            raise LidarError(self.error) from e
        self.error = self._said = None
        d = self.dev
        self._event(f"opened {d.info['name']} (firmware {d.info['firmware']}) on {d.path} at {d.baud} baud; "
                    f"health {d.health['status']}" + (f" (code 0x{d.health['code']:x})" if d.health["code"] else ""),
                    "info" if d.health["status"] == "good" else "warn")
        return d

    def _lost(self, why: str):
        """The port went (a USB reset, unplugged): log why, drop it, and let the next open try at once."""
        d, self.dev = self.dev, None
        what = f"scanning ({d.running})" if d and d.running else "with the motor spinning" if d and d.spinning else ""
        hint = ""
        if d and d.motor_t is not None and time.monotonic() - d.motor_t < 5.0:
            hint = (f", {time.monotonic() - d.motor_t:.1f} s after the motor started: a dip in the lidar's 5 V at "
                    "spin-up is the usual cause (feed it from a Pi port or a powered hub, or plug in the adapter's "
                    "second, power micro-USB)")
        self._event(f"lost the lidar{' while ' + what if what else ''}: {why}{hint}", "error")
        if d is not None:
            try:
                d.close()
            except Exception:
                pass
        self.error, self.failed = None, 0.0
        if not self.fake and not self.port_arg:          # give a re-enumerating adapter a moment to come back
            t_end = time.monotonic() + REAPPEAR_S
            while find_port() is None and time.monotonic() < t_end:
                time.sleep(0.2)

    def describe(self, open_: bool = True) -> dict:
        with self.lock:
            if open_:
                try:
                    self._open()
                except LidarError:
                    pass
            d = self.dev
            out = dict(port=self.port, open=d is not None, error=self.error, idle_s=self.idle_s, running=None,
                       spinning=False, pwm=0)
            if d is not None:
                t = d.turn
                out.update(baud=d.baud, info=d.info, health=d.health, rates=d.rates, modes=d.modes, running=d.running,
                           spinning=d.spinning, pwm=d.pwm, path=d.path, bad_bytes=d.bad, bytes_in=d.bytes_in,
                           rate=d.rate() if d.running else {}, error=d.error if d.running else None,
                           turn_age_ms=round((time.monotonic() - t.t) * 1e3) if t and d.running else None)
                if d.running and d.error and d.error != self._said:
                    self._event(d.error, "warn")
                self._said = d.error if d.running else None
            out["events"] = [dict(t=round(t, 3), level=lv, text=tx) for t, lv, tx in self.events]
            out["self_test"] = self.last_test
            return out

    def set(self, run: bool | None = None, mode: str | None = None, pwm: int | None = None, reset: bool = False,
            health: bool = False, spin: bool | None = None, reopen: bool = False) -> dict:
        """One request from the page. If the port fails under it (a USB reset since the last request), the lidar is
        opened again and the request tried once more."""
        with self.lock:
            if reopen:
                self._event("reconnect asked")
                self.close()
                self.error, self.failed = None, 0.0
            for attempt in (0, 1):
                d = self._open()
                try:
                    self._apply(d, run, mode, pwm, reset, health, spin)
                    break
                except LidarError as e:
                    self._event(str(e), "warn")
                    raise
                except Exception as e:                     # the port itself failed
                    self._lost(d.gone() or f"{type(e).__name__}: {e}")
                    if attempt:
                        raise LidarError(f"the lidar dropped out again ({type(e).__name__}: {e}); see Events") from e
            self.used = time.monotonic()                   # the spin-up isn't idle time
        return self.describe(open_=False)

    def _apply(self, d: RPLidar, run, mode, pwm, reset, health, spin):
        spin_up = 0.1 if self.fake else 1.0
        if reset:
            d.reset()
            self._event(f"reset the device: health {d.health['status']}")
        if health:
            h = d.refresh_health()
            self._event(f"health {h['status']}" + (f" (code 0x{h['code']:x})" if h["code"] else ""),
                        "info" if h["status"] == "good" else "warn")
        if run is False:
            if d.running or d.spinning:
                self._event("stopped (motor off)")
            d.stop()
        elif run or (mode and d.running and d.mode(mode)["name"] != d.running):
            p = pwm if pwm is not None else (d.pwm or DEFAULT_PWM)
            d.start(mode or d.running, p, spin_up_s=spin_up)
            self._event(f"scanning in {d.running} at PWM {d.pwm}")
        elif spin is not None:
            d.spin(spin, pwm if pwm is not None else (d.pwm or DEFAULT_PWM), spin_up_s=spin_up)
            self._event(f"motor spinning at PWM {d.pwm}, no scan" if spin else "motor off")
        elif pwm is not None and (d.running or d.spinning):
            d.set_pwm(pwm)

    def scan(self, after: int = 0, timeout: float = 2.0) -> dict:
        """The newest turn after `after` (waits up to timeout for it): compact lists, angles in 0.01 deg, mm."""
        with self.lock:
            d = self._open()
            if not d.running:
                raise LidarError("not scanning: press Start")
        until = time.monotonic() + timeout
        t = d.wait(after, timeout)
        while t is None and (d.starting or d.running) and time.monotonic() < until:
            time.sleep(0.05)                               # a mode change restarted the scan: its next turn
            t = d.wait(after, max(0.0, until - time.monotonic()))
        self.used = time.monotonic()
        if t is None:
            raise LidarError(d.error or (f"no turn in {timeout:.0f} s" if d.running else "the scan stopped"))
        return dict(seq=t.seq, age_ms=round((time.monotonic() - t.t) * 1e3, 1), mode=d.running, stats=t.stats(),
                    rate=d.rate(), a=[round(x * 100) for x in t.angle], r=[round(x) for x in t.dist], q=t.quality)

    def self_test(self, seconds: float = 3.0, path=None) -> dict:
        """The boot check (camera_snap runs it at its start; GET /lidar/test again): open the lidar, read its model and
        health, scan for `seconds`, count the turns and samples, then stop it (motor off, the port let go). ok: True
        passed, False failed (detail says why), None skipped (the ROS driver has the port, or the Lidar page is
        scanning). Kept as last_test and, with path, written there as JSON (mission_node's preflight reads it)."""
        res = dict(t=round(time.time(), 1), ok=False, detail="")
        d = None
        with self.lock:
            if self.dev is not None and (self.dev.running or self.dev.spinning):
                res.update(ok=None, detail="the Lidar page was scanning: its numbers are live there")
            else:
                try:
                    d = self._open()
                    res.update(model=d.info["name"], firmware=d.info["firmware"], health=d.health["status"],
                               port=d.path, baud=d.baud)
                    d.start(None, DEFAULT_PWM, spin_up_s=0.1 if self.fake else 1.0)
                except LidarError as e:
                    held = "in use by" in str(e)
                    res.update(ok=None if held else False, detail=str(e))
                    d = None
                except Exception as e:                     # the port failed under it
                    self._lost(f"{type(e).__name__}: {e}")
                    res.update(detail=f"the lidar dropped out ({type(e).__name__}: {e})")
                    d = None
        if d is not None:
            t_end = time.monotonic() + seconds
            while time.monotonic() < t_end and d.running:
                time.sleep(0.1)
            rate, turn, err = d.rate(), d.turn, d.error
            with self.lock:
                try:
                    d.stop()
                except Exception:
                    pass
                self.close()                               # let the port go: the ROS driver may want it
            hz = rate.get("hz") or 0.0
            n = rate.get("samples_per_turn") or (len(turn.dist) if turn else 0)
            res.update(hz=round(hz, 2), samples=int(n), valid=rate.get("valid"),
                       range_m=turn.stats().get("median_m") if turn else None)
            why = [w for w, bad in ((f"health {res['health']}", res["health"] != "good"),
                                    (f"{hz:.1f} turns/s (motor or USB power?)", hz < 5.0),
                                    (f"{n} samples a turn", n < 100),
                                    (err or "", bool(err) and hz < 5.0)) if bad and w]
            res.update(ok=not why, detail="; ".join(why) if why else "spun, scanned and stopped")
        self._event(f"self-test: {'passed' if res['ok'] else 'skipped' if res['ok'] is None else 'failed'}"
                    + (f" ({res['detail']})" if res["detail"] else ""),
                    "info" if res["ok"] is not False else "error")
        self.last_test = res
        if path:
            import json
            try:
                tmp = f"{path}.tmp"
                with open(tmp, "w") as f:
                    json.dump(res, f)
                os.replace(tmp, path)
            except OSError as e:
                self._event(f"couldn't write the self-test result to {path}: {e}", "warn")
        return res

    def close_if_idle(self, now: float):
        with self.lock:
            if self.dev is not None and now - self.used > self.idle_s:
                self._event(f"no page for {self.idle_s:.0f} s: motor off, port let go")
                self.close()

    def close(self):
        d, self.dev = self.dev, None
        if d is not None:
            try:
                d.close()
            except Exception:
                pass


# ---------------------------------------------------------------- a stand-in for tests and the page on a PC

class FakeSerial:
    """An A2M8 on a serial line, in a 4 x 3 m room with a 0.15 m post: answers info, health, sample rate and its
    scan modes (Standard 2 k/s, Express 4 k/s), and streams standard samples or express capsules at its rate
    while the motor runs: 10 Hz at PWM 660, proportional to it, with DTR cleared; at 10 Hz whatever the PWM while DTR
    is raised (as the real A2M8's adapter, Nov 13). Opened with DTR raised, as pyserial opens a port."""
    MODES = (("Standard", 500.0, ANS_STANDARD), ("Express", 250.0, ANS_EXPRESS))

    def __init__(self, port: str = "fake", baud: int = 115200, answer_baud: int = 115200):
        self.port, self.baud, self.answer_baud = port, baud, answer_baud
        self.timeout, self.dtr = 0.3, True
        self.out = bytearray()
        self.scanning: int | None = None               # the answer type while scanning
        self.pwm = 0
        self.t_last = time.monotonic()
        self.phase = 0.0                                # turns, fractional
        self.k = 0
        self.acc = 0.0
        self.closed = False
        self.unplugged = False                          # True: every call fails as a vanished USB device's does
        self.prev_start: int | None = None

    def _check(self):
        if self.unplugged:
            raise OSError(5, "Input/output error")

    # the room, seen from (0.5, 0.2) m off its centre: a range in mm at a clockwise angle (deg)
    @staticmethod
    def range_mm(deg: float) -> float:
        import math
        a = -math.radians(deg)                          # clockwise -> the room's counter-clockwise
        x0, y0, dx, dy = 0.5, 0.2, math.cos(a), math.sin(a)
        best = 12.0
        for edge, d in ((2.0 - x0, dx), (-2.0 - x0, dx), (1.5 - y0, dy), (-1.5 - y0, dy)):
            if abs(d) > 1e-9 and edge / d > 0:
                best = min(best, edge / d)
        px, py = 1.2 - x0, -0.6 - y0                    # the post
        b = px * dx + py * dy
        c = px * px + py * py - 0.075 ** 2
        if b > 0 and b * b - c > 0:
            best = min(best, b - math.sqrt(b * b - c))
        return 0.0 if best > 8.0 else round(best * 1000, 1)

    def write(self, data: bytes):
        self._check()
        data = bytes(data)
        if self.baud != self.answer_baud or len(data) < 2 or data[0] != 0xA5:
            return len(data)
        cmd = data[1]
        payload = data[3:3 + data[2]] if len(data) > 2 else b""
        desc = lambda size, typ, cont=False: bytes((0xA5, 0x5A)) + struct.pack("<I", size | (1 << 30 if cont else 0)) + bytes((typ,))  # noqa: E731
        if cmd == STOP:
            self.scanning = None
        elif cmd == RESET:
            self.scanning = None
        elif cmd == SET_PWM:
            self.pwm = struct.unpack("<H", payload[:2])[0]
        elif cmd == GET_INFO:
            self.out += desc(20, 0x04) + bytes((0x28, 26, 1, 5)) + bytes(range(16))
        elif cmd == GET_HEALTH:
            self.out += desc(3, 0x06) + bytes((0, 0, 0))
        elif cmd == GET_SAMPLERATE:
            self.out += desc(4, 0x15) + struct.pack("<HH", 500, 250)
        elif cmd == GET_CONF:
            key = struct.unpack("<I", payload[:4])[0]
            m = struct.unpack("<H", payload[4:6])[0] if len(payload) >= 6 else 0
            name, us, ans = self.MODES[min(m, len(self.MODES) - 1)]
            body = {CONF_COUNT: struct.pack("<H", len(self.MODES)), CONF_TYPICAL: struct.pack("<H", 0),
                    CONF_US_PER_SAMPLE: struct.pack("<I", int(us * 256)), CONF_MAX_DIST: struct.pack("<I", 12 * 256),
                    CONF_ANS_TYPE: bytes((ans,)), CONF_NAME: name.encode() + b"\0"}[key]
            self.out += desc(4 + len(body), 0x20) + struct.pack("<I", key) + body
        elif cmd == SCAN:
            self.out += desc(5, ANS_STANDARD, True)
            self.scanning, self.t_last, self.prev_start = ANS_STANDARD, time.monotonic(), None
        elif cmd == EXPRESS_SCAN:
            mode = payload[0] if payload else 0
            ans = self.MODES[mode][2] if mode < len(self.MODES) else ANS_EXPRESS
            self.out += desc(5 if ans == ANS_STANDARD else 84, ans, True)
            self.scanning, self.t_last, self.prev_start = ans, time.monotonic(), None
        return len(data)

    def _generate(self):
        now = time.monotonic()
        dt, self.t_last = now - self.t_last, now
        if self.scanning is None or not (self.dtr or self.pwm > 0):
            return
        hz = 10.0 * (DEFAULT_PWM if self.dtr else self.pwm) / DEFAULT_PWM
        per = 2000 if self.scanning == ANS_STANDARD else 4000
        self.acc += min(dt, 0.5) * per + (1 if self.k == 0 else 0)      # samples due, fractions carried
        n = int(self.acc) if self.scanning == ANS_STANDARD else int(self.acc) // 32 * 32
        self.acc -= n
        if self.scanning == ANS_STANDARD:
            for _ in range(n):
                prev = self.phase % 1.0
                self.phase += hz / per
                start = (self.phase % 1.0) < prev or self.k == 0
                ang = (self.phase % 1.0) * 360.0
                d = self.range_mm(ang)
                q = 47 if d else 0
                aq6 = int(ang * 64) & 0x7FFF
                self.out += bytes(((q << 2) | (0 if start else 2) | (1 if start else 0), ((aq6 & 0x7F) << 1) | 1, aq6 >> 7)) \
                    + struct.pack("<H", int(d * 4) & 0xFFFF)
                self.k += 1
        else:                                            # express capsules, 32 samples each, no angle offsets
            for _ in range(n // 32):
                a0 = (self.phase % 1.0) * 360.0
                start_q6 = int(a0 * 64) & 0x7FFF
                body = struct.pack("<H", start_q6 | (0x8000 if self.k == 0 else 0))
                step = hz / per
                for pos in range(16):
                    ds = [self.range_mm(((self.phase + (2 * pos + j) * step) % 1.0) * 360.0) for j in (0, 1)]
                    body += struct.pack("<HHB", int(ds[0]) << 2, int(ds[1]) << 2, 0)
                cs = 0
                for b in body:
                    cs ^= b
                self.out += bytes((0xA0 | (cs & 0xF), 0x50 | (cs >> 4))) + body
                self.phase += 32 * step
                self.k += 32

    def read(self, n: int = 1) -> bytes:
        self._check()
        t_end = time.monotonic() + (self.timeout or 0)
        while True:
            self._generate()
            if self.out or time.monotonic() >= t_end:
                break
            time.sleep(0.01)
        b = bytes(self.out[:n])
        del self.out[:n]
        return b

    @property
    def in_waiting(self) -> int:
        self._check()
        self._generate()
        return len(self.out)

    def reset_input_buffer(self):
        self._check()
        self._generate()
        self.out.clear()

    def close(self):
        self.closed = True
        self.scanning = None


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int)
    ap.add_argument("--mode", help="scan mode by name (default: the device's typical one)")
    ap.add_argument("--pwm", type=int, default=DEFAULT_PWM)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--fake", action="store_true")
    a = ap.parse_args()
    lid = RPLidar(a.port, a.baud, opener=(lambda p, b: FakeSerial(p, b)) if a.fake else None)
    try:
        print(f"{lid.info['name']} firmware {lid.info['firmware']} hardware {lid.info['hardware']} serial "
              f"{lid.info['serial']} on {a.port} at {lid.baud} baud; health {lid.health['status']} (code {lid.health['code']})")
        for m in lid.modes:
            print(f"  mode {m['id']} {m['name']:<12} {m['rate']:>6}/s  max {m['max_m']} m  answer 0x{m['answer']:02X}"
                  f"{'  (typical)' if m['typical'] else ''}")
        lid.start(a.mode, a.pwm)
        t_end = time.monotonic() + a.seconds
        while time.monotonic() < t_end:
            time.sleep(1.0)
            t = lid.turn
            print(f"  {lid.running}: {lid.rate()}  last turn {t.stats() if t else None}  bad bytes {lid.bad}")
    finally:
        lid.close()


if __name__ == "__main__":
    main()
