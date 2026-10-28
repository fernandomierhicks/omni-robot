"""Build the firmware core as a shared library for the sim (software in the loop, design.md 30).

  Windows  zig c++ (pip package `ziglang` in the sim venv: no Visual Studio needed) -> build/win64/omni_fw_sil.dll
  Linux    g++ (WSL)                                                                -> build/linux/omni_fw_sil.so

Rebuilds only when a source is newer than the library. Standard library only, so the sim and the ROS side can both
call build() before loading.   python build_sil.py [--force] [--pins]   (--pins rewrites firmware/PINS.md)
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

SIL = Path(__file__).resolve().parent
FW = SIL.parent
CORE = FW / "lib" / "omni_fw"
SOURCES = sorted(CORE.glob("*.cpp")) + [SIL / "sil_api.cpp"]
HEADERS = sorted(CORE.glob("*.h"))


def lib_path() -> Path:
    if sys.platform == "win32":
        return SIL / "build" / "win64" / "omni_fw_sil.dll"
    return SIL / "build" / "linux" / "omni_fw_sil.so"


def stale(out: Path) -> bool:
    if not out.exists():
        return True
    t = out.stat().st_mtime
    return any(p.stat().st_mtime > t for p in SOURCES + HEADERS + [Path(__file__), FW / "fw_build_id.py"])


def build(force: bool = False, quiet: bool = True) -> Path:
    out = lib_path()
    if not force and not stale(out):
        return out
    out.parent.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(FW))
    from fw_build_id import build_id                 # FW_BUILD, the id the INFO message reports
    flags = ["-std=c++17", "-O2", "-shared", "-fno-exceptions", "-fno-rtti", "-Wall", "-Wextra",
             "-Wno-unused-parameter", f"-I{CORE}", f'-DFW_BUILD="{build_id(CORE)}"']
    if sys.platform == "win32":
        cmd = [sys.executable, "-m", "ziglang", "c++", "-target", "x86_64-windows-gnu", *flags]
    else:
        cmd = ["g++", "-fPIC", *flags]
    tmp = out.with_suffix(out.suffix + ".tmp")
    cmd += [str(s) for s in SOURCES] + ["-o", str(tmp)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"SIL build failed:\n{' '.join(cmd)}\n{r.stdout}\n{r.stderr}")
    if r.stderr.strip() and not quiet:
        print(r.stderr)
    os.replace(tmp, out)                     # a sim holding the old library keeps its copy (Linux); Windows: retry
    for junk in out.parent.glob("*.pdb"):
        junk.unlink(missing_ok=True)
    for junk in out.parent.glob("*.lib"):
        junk.unlink(missing_ok=True)
    return out


def write_pins():
    sys.path.insert(0, str(SIL))
    from omni_fw_sil import board_report
    text = ("# Teensy 4.1 pin map\n\nGenerated from `lib/omni_fw/board_teensy41.h` by `sil/build_sil.py --pins` "
            "(the header's static_asserts fail the build on a clash, a pin that can't do the job, or a missing "
            "signal). design.md section 30.\n\n" + board_report())
    (FW / "PINS.md").write_text(text)
    print(f"wrote {FW / 'PINS.md'}")


if __name__ == "__main__":
    p = build(force="--force" in sys.argv, quiet=False)
    print(f"built {p}")
    if "--pins" in sys.argv:
        write_pins()
