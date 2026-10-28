"""The firmware's build id: the first 8 hex digits of a SHA-1 over lib/omni_fw (each file's name and its text, line
endings normalized). The Teensy build (PlatformIO, `extra_scripts = pre:fw_build_id.py`) and the SIL build
(sil/build_sil.py) compile it in as FW_BUILD; the MCU reports it in its INFO message. The Jetson computes the same id
from its own copy of firmware/ (omni_console/status.py firmware_build_id) and warns when they differ: the robot runs
firmware other than the software expects.

  python fw_build_id.py          prints this checkout's id
"""

import hashlib
from pathlib import Path


def build_id(core=None) -> str:
    core = Path(core) if core else Path(__file__).resolve().parent / "lib" / "omni_fw"
    h = hashlib.sha1()
    for p in sorted(core.glob("*.h")) + sorted(core.glob("*.cpp")):
        h.update(p.name.encode())
        h.update(p.read_bytes().replace(b"\r\n", b"\n"))
    return h.hexdigest()[:8]


try:                                            # as a PlatformIO extra script
    Import("env")                               # noqa: F821  (SCons)
except NameError:
    if __name__ == "__main__":
        print(build_id())
else:
    _id = build_id(Path(env.subst("$PROJECT_DIR")) / "lib" / "omni_fw")     # noqa: F821
    env.Append(CPPDEFINES=[("FW_BUILD", env.StringifyMacro(_id))])         # noqa: F821
    print(f"firmware build id {_id}")
