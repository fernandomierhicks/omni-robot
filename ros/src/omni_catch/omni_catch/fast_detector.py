"""The ball detector with its pixel loops in C++ (design.md 7: "a C++ or GPU detector, timed on the PC").

FastDetector is omni_catch.detector.Detector with the two per-pixel steps replaced by native/blob.cpp: the colour test
with the components and their moments in a window, and the coarse view's components. Everything else (the tracking
window, the candidates, which blob wins) is the Python class's own code, and the C++ reproduces the numpy / scipy
arithmetic (labels in the same order, the same float32 hue), so the two report the same blobs (test_v11 checks).

The library is built on first use from the source next to this file: g++ on Linux (WSL, the Jetson), zig c++ on
Windows (the sim's venv has the ziglang package), into ~/.cache/omni_catch/<hash of the source>/, so a changed source
builds a new library and nothing is written into the package. available() says whether it built; Perception falls
back to the Python detector if not.
"""

from __future__ import annotations

import ctypes
import hashlib
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

from omni_catch.detector import Detector, DetectorConfig

SRC = Path(__file__).resolve().parent / "native" / "blob.cpp"
_LIB = []          # [ctypes.CDLL] or [None] once tried


def lib_path() -> Path:
    h = hashlib.sha1(SRC.read_bytes()).hexdigest()[:12]
    cache = Path(os.environ.get("OMNI_NATIVE_CACHE", Path.home() / ".cache" / "omni_catch")) / h
    return cache / ("blob.dll" if sys.platform == "win32" else "blob.so")


def build(force: bool = False) -> Path:
    out = lib_path()
    if out.exists() and not force:
        return out
    out.parent.mkdir(parents=True, exist_ok=True)
    flags = ["-std=c++17", "-O3", "-shared", "-fno-exceptions", "-fno-rtti", "-Wall"]
    if sys.platform == "win32":
        cmd = [sys.executable, "-m", "ziglang", "c++", "-target", "x86_64-windows-gnu", *flags]
    else:
        cmd = ["g++", "-fPIC", "-march=native", *flags]
    tmp = out.with_name(out.name + f".{os.getpid()}.tmp")
    r = subprocess.run(cmd + [str(SRC), "-o", str(tmp)], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"native detector build failed:\n{' '.join(cmd)}\n{r.stdout}\n{r.stderr}")
    os.replace(tmp, out)
    for junk in list(out.parent.glob("*.pdb")) + list(out.parent.glob("*.lib")):
        junk.unlink(missing_ok=True)
    return out


def load():
    if _LIB:
        return _LIB[0]
    try:
        lib = ctypes.CDLL(str(build()))
        dp, ip = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int32)
        u8 = ctypes.POINTER(ctypes.c_uint8)
        lib.bd_components.argtypes = [u8, ctypes.c_int, ctypes.c_int, ctypes.c_long, ctypes.c_long, ctypes.c_int,
                                      ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float,
                                      ctypes.c_float, ctypes.c_float, ctypes.c_int, dp, ctypes.c_int]
        lib.bd_components.restype = ctypes.c_int
        lib.bd_coarse.argtypes = [u8, ctypes.c_int, ctypes.c_int, ctypes.c_long, ctypes.c_long, ctypes.c_float,
                                  ctypes.c_float, ctypes.c_float, ctypes.c_float, ip, ctypes.c_int]
        lib.bd_coarse.restype = ctypes.c_int
        _LIB.append(lib)
    except (OSError, RuntimeError, FileNotFoundError) as e:
        print(f"omni_catch: the native detector isn't available ({str(e).splitlines()[0]}): using the Python one",
              file=sys.stderr)
        _LIB.append(None)
    return _LIB[0]


def available() -> bool:
    return load() is not None


def _view(a: np.ndarray):
    """(pointer, h, w, row stride, pixel stride) of an H x W x 3 uint8 array or view (channels contiguous)."""
    if a.dtype != np.uint8 or a.ndim != 3 or a.shape[2] < 3 or a.strides[2] != 1:
        a = np.ascontiguousarray(a[..., :3], dtype=np.uint8)
    return a, a.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), a.shape[0], a.shape[1], a.strides[0], a.strides[1]


class FastDetector(Detector):
    MAX = 4096

    def __init__(self, cfg: DetectorConfig | None = None):
        super().__init__(cfg)
        self.lib = load()
        if self.lib is None:
            raise RuntimeError("the native detector didn't build")
        self._out = np.zeros(6 * self.MAX)
        self._cout = np.zeros(5 * self.MAX, dtype=np.int32)

    def _components(self, fr, u0, v0, u1, v1):
        import math
        u0, v0 = max(0, int(u0)), max(0, int(v0))
        u1, v1 = min(fr.width, int(math.ceil(u1))), min(fr.height, int(math.ceil(v1)))
        if u1 <= u0 or v1 <= v0:
            return []
        c = self.cfg
        keep, ptr, h, w, rs, ps = _view(fr.roi(u0, v0, u1, v1))
        n = self.lib.bd_components(ptr, h, w, rs, ps, u0, v0, fr.width, fr.height, c.hue_deg[0], c.hue_deg[1],
                                   c.min_sat, c.min_val, c.min_blob_px,
                                   self._out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), self.MAX)
        del keep
        out = []
        for i in range(max(0, n)):
            uc, vc, b, cnt, ok, _ = self._out[6 * i:6 * i + 6]
            if ok:
                out.append((float(uc), float(vc), float(b) * c.size_gain if c.size_gain != 1.0 else float(b), int(cnt)))
        return out

    def _coarse(self, fr):
        c = self.cfg
        keep, ptr, h, w, rs, ps = _view(fr.coarse(c.coarse_stride))
        n = self.lib.bd_coarse(ptr, h, w, rs, ps, c.hue_deg[0], c.hue_deg[1], c.min_sat, c.min_val,
                               self._cout.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)), self.MAX)
        del keep
        o = self._cout
        return [(k + 1, int(o[5 * k]), int(o[5 * k + 1]), int(o[5 * k + 2]), int(o[5 * k + 3]), int(o[5 * k + 4]))
                for k in range(max(0, n))]
