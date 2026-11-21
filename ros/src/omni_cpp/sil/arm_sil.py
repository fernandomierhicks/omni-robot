"""The robot's C++ arm driver (omni_cpp's ArmDriverCore, the code the arm_driver node runs) for the v11 sim: built from
the sources on first use into a small C library (arm_sil.cpp) and loaded with ctypes, as the firmware's SIL library is
(firmware/sil). g++ on Linux / WSL, zig c++ on Windows (pip package ziglang in the sim venv). Standard library only.

    drv = CppArmDriver(port, "…/robot_arm_d435.json")   # port: write(bytes), read(n) -> bytes (the sim's ServoBusSim)
    drv.connect()                                        # [] = every servo answered: torque on, holding
    drv.command(MOVE, times, waypoints)                  # an ArmMotion, as arm_brain sends it
    st = drv.cycle(t)                                    # read, play, write: the arm's state (ArmHw)
    ld = drv.load_state()                                # what it makes of the load (design.md 4.5: ArmLoad)
"""

from __future__ import annotations

import ctypes as C
import hashlib
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]           # ros/src/omni_cpp
MOVE, HOLD, RELAX = 0, 1, 2                          # omni_interfaces/ArmMotion
SOURCES = ["sil/arm_sil.cpp", "src/arm_driver_core.cpp", "src/feetech.cpp", "src/arm_kin.cpp", "src/arm_motion.cpp",
           "src/arm_load.cpp", "src/json_value.cpp"]

_WRITE = C.CFUNCTYPE(None, C.POINTER(C.c_uint8), C.c_uint32)
_READ = C.CFUNCTYPE(C.c_uint32, C.POINTER(C.c_uint8), C.c_uint32)


def build() -> Path:
    """The library for these sources (rebuilt when any source or header changes), in ~/.cache/omni_cpp/<hash>."""
    srcs = [PKG / s for s in SOURCES]
    heads = sorted((PKG / "include" / "omni_cpp").glob("*.hpp"))
    h = hashlib.sha1(b"".join(p.read_bytes() for p in srcs + heads) + sys.platform.encode()).hexdigest()[:12]
    name = "omni_arm_sil.dll" if sys.platform == "win32" else "libomni_arm_sil.so"
    out = Path(os.environ.get("OMNI_NATIVE_CACHE", Path.home() / ".cache" / "omni_cpp")) / h / name
    if out.exists():
        return out
    out.parent.mkdir(parents=True, exist_ok=True)
    flags = ["-std=c++17", "-O2", "-shared", "-Wall", "-I", str(PKG / "include")]
    if sys.platform == "win32":
        cmd = [sys.executable, "-m", "ziglang", "c++", "-target", "x86_64-windows-gnu", *flags]
    else:
        cmd = ["g++", "-fPIC", *flags]
    tmp = out.with_suffix(out.suffix + ".tmp")
    r = subprocess.run(cmd + [str(s) for s in srcs] + ["-o", str(tmp)], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"arm SIL build failed:\n{' '.join(cmd)}\n{r.stdout}\n{r.stderr}")
    os.replace(tmp, out)
    return out


_LIB = []


def lib():
    if not _LIB:
        L = C.CDLL(str(build()))
        L.arm_error.restype = C.c_char_p
        L.arm_create.restype = C.c_void_p
        L.arm_create.argtypes = [C.c_char_p, C.c_char_p, C.c_char_p]
        L.arm_destroy.argtypes = [C.c_void_p]
        L.arm_connect.argtypes = [C.c_void_p, _WRITE, _READ, C.c_double, C.POINTER(C.c_int)]
        L.arm_command.argtypes = [C.c_void_p, C.c_int, C.c_int, C.POINTER(C.c_double), C.c_uint32,
                                  C.POINTER(C.c_double), C.c_uint32, C.c_int]
        L.arm_read.argtypes = [C.c_void_p, C.c_double]
        L.arm_drive.argtypes = [C.c_void_p, C.c_double]
        d6, i6, ip, dp = C.c_double * 6, C.c_int * 6, C.POINTER(C.c_int), C.POINTER(C.c_double)
        L.arm_state.argtypes = [C.c_void_p, d6, d6, d6, d6, i6, dp, ip, i6, ip, i6, ip]
        L.arm_bus_stats.argtypes = [C.c_void_p, C.c_int * 4]
        L.arm_load_state.argtypes = [C.c_void_p, d6, d6, d6, d6, d6, d6, d6, i6, i6, d6, d6, d6, C.c_double * 3,
                                     C.c_int * 3]
        L.arm_set_gravity.argtypes = [C.c_void_p, C.c_double, C.c_double, C.c_double]
        L.arm_set_load_model.argtypes = [C.c_void_p, C.c_char_p]
        L.arm_settings.argtypes = [C.c_void_p, C.c_int, C.c_int, C.c_int, C.c_double, C.c_int, C.c_double, C.c_int,
                                   C.c_int, C.c_int, C.c_int, C.c_int, C.c_double]
        L.arm_config_notes.restype = C.c_char_p
        L.arm_config_notes.argtypes = [C.c_void_p]
        L.arm_estimate.argtypes = [C.c_void_p, C.c_double * 48, C.c_double * 3, C.c_double * 24, d6, d6,
                                   C.c_double * 3]
        _LIB.append(L)
    return _LIB[0]


@dataclass
class ArmHw:
    """What the driver knows (omni_interfaces/ArmHardware plus the joint states)."""
    t: float = 0.0
    q: list = field(default_factory=lambda: [0.0] * 6)
    dq: list = field(default_factory=lambda: [0.0] * 6)
    load: list = field(default_factory=lambda: [0.0] * 6)
    goal: list = field(default_factory=lambda: [0.0] * 6)
    temp_c: list = field(default_factory=lambda: [0] * 6)
    volts: float = 0.0
    connected: bool = False
    torque: bool = False
    have_goal: bool = False
    moving: bool = False
    ok: bool = False
    tool_out: bool = False
    missing: tuple = ()
    errors: tuple = ()


@dataclass
class ArmLoad:
    """The driver's load estimate (design.md 4.5; omni_interfaces/ArmServoState)."""
    q_enc: list
    current: list
    gravity: list            # N m on each joint
    motor: list              # N m the servo's motor gives
    external: list           # N m from outside (0 inside the friction band at rest)
    band: list               # N m: the band's half width (0 moving)
    filtered: list           # the detector's
    strained: list = field(default_factory=list)
    status: list = field(default_factory=list)       # the servos' error bits
    volts: list = field(default_factory=list)        # each servo's supply
    goal_vel: list = field(default_factory=list)     # the played goal's
    goal_acc: list = field(default_factory=list)
    tool_force: list = field(default_factory=list)   # N, base_link
    reflexes: int = 0
    reflex_active: bool = False
    has_current: bool = False


class CppArmDriver:
    def __init__(self, port, config: str | Path, calibration: str | Path = "", load: str | Path = ""):
        self.port = port
        self.L = lib()
        self.h = self.L.arm_create(str(config).encode(), str(calibration).encode(), str(load).encode())
        if not self.h:
            raise RuntimeError(f"arm driver: {self.L.arm_error().decode()}")

        def write(data, n):
            self.port.write(C.string_at(data, n))

        def read(buf, cap):
            b = self.port.read(cap)
            C.memmove(buf, b, len(b))
            return len(b)

        self._cb = (_WRITE(write), _READ(read))       # kept alive while the library holds them

    def __del__(self):
        if getattr(self, "h", None):
            self.L.arm_destroy(self.h)
            self.h = None

    def _check(self, rc: int, what: str) -> int:
        if rc < 0:
            raise RuntimeError(f"arm driver {what}: {self.L.arm_error().decode()}")
        return rc

    def connect(self, timeout_s: float = 0.0) -> list:
        """Ping every servo; all there: torque on, hold where the arm is. Returns the missing ids."""
        miss = (C.c_int * 6)()
        n = self._check(self.L.arm_connect(self.h, self._cb[0], self._cb[1], timeout_s, miss), "connect")
        return list(miss[:n])

    def command(self, cmd: int, times=(), waypoints=(), linear: bool = False, guard: bool = True) -> bool:
        """An ArmMotion (MOVE / HOLD / RELAX). waypoints: n rows of 6 joint angles. guard False: no reflex for this
        motion. False if refused (why())."""
        flat = [float(v) for row in waypoints for v in row]
        n = len(times)
        T = (C.c_double * max(1, n))(*[float(t) for t in times])
        W = (C.c_double * max(1, len(flat)))(*flat)
        return bool(self._check(self.L.arm_command(self.h, cmd, int(linear), T, n, W, len(flat), int(guard)),
                                "command"))

    def load_state(self) -> ArmLoad:
        d6, i6 = C.c_double * 6, C.c_int * 6
        a = [d6() for _ in range(7)]
        st, sb, f3, c3 = i6(), i6(), (C.c_double * 3)(), (C.c_int * 3)()
        vj, gv, ga = d6(), d6(), d6()
        self.L.arm_load_state(self.h, *a, st, sb, vj, gv, ga, f3, c3)
        return ArmLoad(*[list(x) for x in a], strained=[bool(v) for v in st], status=list(sb), volts=list(vj),
                       goal_vel=list(gv), goal_acc=list(ga), tool_force=list(f3), reflexes=c3[0],
                       reflex_active=bool(c3[1]), has_current=bool(c3[2]))

    def set_gravity(self, g):
        self.L.arm_set_gravity(self.h, *[float(v) for v in g])

    def set_load_model(self, path: str | Path = ""):
        self._check(self.L.arm_set_load_model(self.h, str(path).encode()), "load model")

    def settings(self, arm_torque: int = 1000, grip_torque: int = 500, reflex: bool = True, backoff: float = 0.05,
                 soft_torque: int = 350, tau: float = 0.04, n: int = 4, write_servo_settings: bool = True,
                 model_angles: bool = True, reflex_nominal: bool = False, hold_trim: bool = True,
                 deflect: float = 0.012):
        """The driver's settings (the node's parameters). reflex_nominal: the reflex fires on the nominal model too."""
        self.L.arm_settings(self.h, arm_torque, grip_torque, int(reflex), backoff, soft_torque, tau, n,
                            int(write_servo_settings), int(model_angles), int(reflex_nominal), int(hold_trim),
                            float(deflect))

    def config_notes(self) -> list:
        return [n for n in self.L.arm_config_notes(self.h).decode().split("\n") if n]

    def estimate(self, q, w, duty, volts, temp, current, goal_vel, goal_acc, g=(0.0, 0.0, -9.81)) -> dict:
        """The driver's load model on one sample (no bus): {grav, motor, ext, band, q_true, q_enc_goal, force}."""
        vals = [float(v) for row in (q, w, duty, volts, temp, current, goal_vel, goal_acc) for v in row]
        out, t6, e6, f3 = (C.c_double * 24)(), (C.c_double * 6)(), (C.c_double * 6)(), (C.c_double * 3)()
        self.L.arm_estimate(self.h, (C.c_double * 48)(*vals), (C.c_double * 3)(*g), out, t6, e6, f3)
        o = list(out)
        return dict(grav=o[0:6], motor=o[6:12], ext=o[12:18], band=o[18:24], q_true=list(t6), q_enc_goal=list(e6),
                    force=list(f3))

    def why(self) -> str:
        return self.L.arm_error().decode()

    def read(self, t: float = 0.0) -> ArmHw:
        self._check(self.L.arm_read(self.h, t), "read")
        return self.state(t)

    def drive(self, t: float):
        self._check(self.L.arm_drive(self.h, t), "drive")

    def cycle(self, t: float) -> ArmHw:
        """The driver's cycle: read every servo, sample the motion at t, write the goals. Returns the state read."""
        st = self.read(t)
        self.drive(t)
        return st

    def state(self, t: float = 0.0) -> ArmHw:
        d6, i6 = C.c_double * 6, C.c_int * 6
        q, dq, load, goal, temp, miss, err = d6(), d6(), d6(), d6(), i6(), i6(), i6()
        volts, flags, nm, ne = C.c_double(), C.c_int(), C.c_int(), C.c_int()
        self.L.arm_state(self.h, q, dq, load, goal, temp, C.byref(volts), C.byref(flags), miss, C.byref(nm), err,
                         C.byref(ne))
        f = flags.value
        return ArmHw(t=t, q=list(q), dq=list(dq), load=list(load), goal=list(goal), temp_c=list(temp),
                     volts=volts.value, connected=bool(f & 1), torque=bool(f & 2), have_goal=bool(f & 4),
                     moving=bool(f & 8), ok=bool(f & 16), tool_out=bool(f & 32), missing=tuple(miss[:nm.value]),
                     errors=tuple(err[:ne.value]))

    def bus_stats(self) -> dict:
        s = (C.c_int * 4)()
        self.L.arm_bus_stats(self.h, s)
        return dict(timeouts=s[0], tx=s[1], rx=s[2], parse_errors=s[3])
