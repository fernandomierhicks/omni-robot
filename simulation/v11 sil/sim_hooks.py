"""The per-physics-step hooks in C++ (Nov 26; cext/sim_hooks.cpp): the contact model's slides and ball check, the floor
friction field, rolling resistance and the arm's servos, one call a step on MuJoCo's own model and data. Python's hook
objects keep their state (sim_hooks builds C structs pointing at their numpy arrays), so studies that change a servo's
goal or a floor's crr still work. Opt-in (SimCore's cpp_hooks, the ROS bridge): the sums run in another order, so a
run is not bit-identical to the Python hooks' (test_v11 checks they agree to rounding).

Build: zig c++ on Windows (the sim venv's ziglang, as the firmware SIL), g++ in WSL, against the installed MuJoCo's
headers; rebuilt when the source or the MuJoCo version changes (cext/build/<platform>/sim_hooks_<version>.*).
"""

from __future__ import annotations

import ctypes as C
import os
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE / "cext" / "sim_hooks.cpp"
ABI = 1

i32p, f64p, u8p = C.POINTER(C.c_int), C.POINTER(C.c_double), C.POINTER(C.c_uint8)


class Contacts(C.Structure):
    _fields_ = [("is_ball", u8p), ("n_slides", C.c_int), ("slide_q", i32p), ("slide_v", i32p), ("slide_k", f64p),
                ("x_d", f64p), ("decay", C.c_double)]


class Floor(C.Structure):
    _fields_ = [("n_pairs", C.c_int), ("pairs", i32p), ("geoms", i32p), ("mu0", C.c_double), ("n_modes", C.c_int),
                ("k", f64p), ("phi", f64p), ("amp", C.c_double), ("n_patches", C.c_int), ("patches", f64p)]


class Rolling(C.Structure):
    _fields_ = [("n_wheels", C.c_int), ("load_adr", i32p), ("load_site", i32p), ("load_body", i32p), ("drive", i32p),
                ("drive0", f64p), ("crr", f64p), ("R", C.c_double), ("r", C.c_double), ("pin_mu", C.c_double),
                ("pin_r", C.c_double), ("roller_start", i32p), ("roller_dofs", i32p), ("roller0", f64p),
                ("load_out", f64p)]


class Arm(C.Structure):
    _fields_ = [("n", C.c_int), ("act", i32p), ("vadr", i32p), ("cmd", f64p), ("target", f64p), ("speed", f64p),
                ("torque_scale", f64p), ("stall", C.c_double), ("no_load", C.c_double)]


class Hooks(C.Structure):
    _fields_ = [("abi", C.c_int), ("contacts", C.POINTER(Contacts)), ("floor", C.POINTER(Floor)),
                ("rolling", C.POINTER(Rolling)), ("arm", C.POINTER(Arm)), ("floor_div", C.c_int), ("tick", C.c_int64)]


def lib_path() -> Path:
    ver = mujoco.mj_version()
    sub, ext = ("win64", "dll") if sys.platform == "win32" else ("linux", "so")
    return HERE / "cext" / "build" / sub / f"sim_hooks_{ver}.{ext}"


def _include_dir() -> Path:
    root = Path(mujoco.__file__).resolve().parent / "include"
    for h in root.rglob("mujoco.h"):
        if h.parent.name == "mujoco":
            return h.parent.parent                  # #include "mujoco/mujoco.h"
    raise RuntimeError(f"no MuJoCo headers under {root}")


def build(force: bool = False) -> Path:
    out = lib_path()
    if not force and out.exists() and out.stat().st_mtime >= max(SRC.stat().st_mtime, Path(__file__).stat().st_mtime):
        return out
    out.parent.mkdir(parents=True, exist_ok=True)
    flags = ["-std=c++17", "-O2", "-shared", "-fno-exceptions", "-fno-rtti", "-Wall", "-Wextra",
             "-Wno-unused-parameter", f"-I{_include_dir()}"]
    if sys.platform == "win32":
        cmd = [sys.executable, "-m", "ziglang", "c++", "-target", "x86_64-windows-gnu", *flags]
    else:
        cmd = ["g++", "-fPIC", "-fvisibility=hidden", *flags]
    tmp = out.with_suffix(out.suffix + f".{os.getpid()}.tmp")
    r = subprocess.run(cmd + [str(SRC), "-o", str(tmp)], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"sim_hooks build failed:\n{' '.join(cmd)}\n{r.stdout}\n{r.stderr}")
    os.replace(tmp, out)
    for junk in list(out.parent.glob("*.pdb")) + list(out.parent.glob("*.lib")):
        junk.unlink(missing_ok=True)
    return out


_LIB = None


def lib():
    global _LIB
    if _LIB is None:
        L = C.CDLL(str(build()))
        L.hooks_abi.restype = C.c_int
        want = ABI * 10000000 + int(mujoco.mj_version())
        if L.hooks_abi() != want:
            raise RuntimeError(f"sim_hooks ABI {L.hooks_abi()} != {want} (rebuild: delete {lib_path()})")
        L.hooks_pre.restype = C.c_int
        L.hooks_pre.argtypes = [C.c_void_p, C.c_void_p, C.POINTER(Hooks), C.c_double, C.c_int]
        L.hooks_arm.restype = None
        L.hooks_arm.argtypes = [C.c_void_p, C.c_void_p, C.POINTER(Hooks), C.c_double]
        _LIB = L
    return _LIB


def _i32(a) -> np.ndarray:
    return np.ascontiguousarray(a, dtype=np.int32)


def _f64(a) -> np.ndarray:
    return np.ascontiguousarray(a, dtype=np.float64)


def _p(a, t):
    return a.ctypes.data_as(t)


class CppHooks:
    """For a Robot / SilRobot: pre(dt) does what Robot.pre_step's contact, floor and arm hooks did."""

    def __init__(self, robot):
        import floor as floor_mod
        from arm_so101 import ArmServos, SERVO_STALL, SERVO_SPEED
        self.r, self.m, self.d = robot, robot.m, robot.d
        self.L = lib()
        self.keep = []                                  # the arrays the structs point at
        h = self.h = Hooks(abi=ABI, floor_div=int(robot.floor_div), tick=int(robot._tick))
        cm = robot.contacts
        self.contacts = cm
        if cm is not None:
            is_ball = np.ascontiguousarray(cm.is_ball, dtype=np.uint8)
            sq = _i32([s[0] for s in cm.slides] or [0])
            sv = _i32([s[1] for s in cm.slides] or [0])
            sk = _f64([s[2] for s in cm.slides] or [0.0])
            cm.x_d = _f64(cm.x_d) if len(cm.x_d) else np.zeros(1)
            self.keep += [is_ball, sq, sv, sk]
            self.c_contacts = Contacts(_p(is_ball, u8p), len(cm.slides), _p(sq, i32p), _p(sv, i32p), _p(sk, f64p),
                                       _p(cm.x_d, f64p), float(cm._decay))
            h.contacts = C.pointer(self.c_contacts)
        self.ff = next((x for x in robot.floor if isinstance(x, floor_mod.FloorFriction)), None)
        self.rr = next((x for x in robot.floor if isinstance(x, floor_mod.RollingResistance)), None)
        others = [x for x in robot.floor if x is not self.ff and x is not self.rr]
        if others:
            raise RuntimeError(f"sim_hooks: no C++ twin for {others}")
        if self.ff is not None:
            f = self.ff
            pairs, geoms = _i32(f.pairs), _i32(f.geoms)
            k = _f64(f.k) if f.modes else np.zeros(2)
            phi = _f64(f.phi) if f.modes else np.zeros(1)
            patches = _f64(f.patches) if len(f.patches) else np.zeros(5)
            self.keep += [pairs, geoms, k, phi, patches]
            self.c_floor = Floor(len(pairs), _p(pairs, i32p), _p(geoms, i32p), float(f.mu0), int(f.modes or 0),
                                 _p(k, f64p), _p(phi, f64p), float(f.amp if f.modes else 0.0), len(f.patches),
                                 _p(patches, f64p))
            h.floor = C.pointer(self.c_floor)
        if self.rr is not None:
            rr = self.rr
            from robot_model import wheel_loads
            wheel_loads(self.m, self.d, rr.p, rr._cache)                    # fills rr._cache["ids"]
            ids = rr._cache["ids"]
            adr, site, body = (_i32([x[i] for x in ids]) for i in range(3))
            drive, drive0 = _i32(rr.drive), _f64(rr.drive0)
            start = _i32(np.r_[0, np.cumsum([len(ix) for ix in rr.roller_dofs])])
            rdofs = _i32(np.concatenate([np.asarray(ix, int) for ix in rr.roller_dofs]) if start[-1] else [0])
            r0 = _f64(np.concatenate(rr.roller0) if start[-1] else [0.0])
            self._rr_first = not hasattr(rr, "_crr")
            if self._rr_first:
                rr.update_crr()                                              # (Python's first call does this too)
            self.load = np.zeros(len(ids))
            self.keep += [adr, site, body, drive, drive0, start, rdofs, r0]
            self.c_rolling = Rolling(len(ids), _p(adr, i32p), _p(site, i32p), _p(body, i32p), _p(drive, i32p),
                                     _p(drive0, f64p), _p(rr._crr, f64p), float(rr.R), float(rr.r),
                                     float(rr.p.roller_pin_mu), float(rr.p.roller_pin_radius), _p(start, i32p),
                                     _p(rdofs, i32p), _p(r0, f64p), _p(self.load, f64p))
            h.rolling = C.pointer(self.c_rolling)
        self.arm = robot.arm if type(getattr(robot, "arm", None)) is ArmServos else None
        if self.arm is not None:
            a = self.arm
            act, vadr = _i32(a.a), _i32(a.vadr)
            self.keep += [act, vadr]
            self.c_arm = Arm(len(act), _p(act, i32p), _p(vadr, i32p), None, None, None, None, float(SERVO_STALL),
                             float(SERVO_SPEED))
            self._arm_refs = None
            h.arm = C.pointer(self.c_arm)
        self.hp = C.pointer(h)
        self.ma, self.da = self.m._address, self.d._address

    def _arm_arrays(self):
        """Re-point the arm struct when ArmServos rebinds an array (set_pose, a study's speed)."""
        a = self.arm
        for name in ("cmd", "target", "speed", "torque_scale"):
            v = getattr(a, name)
            if v.dtype != np.float64 or not v.flags.c_contiguous:
                setattr(a, name, _f64(v))
        refs = (a.cmd, a.target, a.speed, a.torque_scale)
        if self._arm_refs is None or any(x is not y for x, y in zip(refs, self._arm_refs)):
            self._arm_refs = refs
            ca = self.c_arm
            ca.cmd, ca.target, ca.speed, ca.torque_scale = (_p(x, f64p) for x in refs)

    def pre(self, dt: float, with_arm: bool = True) -> None:
        h = self.h
        if self.rr is not None and (h.tick + 1) % h.floor_div == 0:         # the floor hooks run this step
            if self._rr_first:
                self._rr_first = False                                      # update_crr ran in __init__ (its tick 0)
            else:
                old = self.rr._crr
                self.rr.update_crr()
                if self.rr._crr is not old:
                    self.c_rolling.crr = _p(self.rr._crr, f64p)
        if with_arm and self.arm is not None:
            self._arm_arrays()
        ball = self.L.hooks_pre(self.ma, self.da, self.hp, dt, 1 if with_arm else 0)
        self.r._tick = int(h.tick)
        cm = self.contacts
        if cm is not None and (ball or cm._touched):
            cm.ball_laws()

    def arm_step(self, dt: float) -> None:
        if self.arm is not None:
            self._arm_arrays()
            self.L.hooks_arm(self.ma, self.da, self.hp, dt)
