"""FW_BUILD for the bench build: the same id as the Teensy's (firmware/fw_build_id.py over lib/omni_fw), so the robot
computer's firmware check compares the core it runs, not this board layer."""
import importlib.util
from pathlib import Path

Import("env")  # noqa: F821  (SCons)

_fw = Path(env.subst("$PROJECT_DIR")).resolve().parents[1]           # noqa: F821  -> firmware/
_spec = importlib.util.spec_from_file_location("fw_build_id_mod", _fw / "fw_build_id.py")
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
_id = _mod.build_id(_fw / "lib" / "omni_fw")
env.Append(CPPDEFINES=[("FW_BUILD", env.StringifyMacro(_id))])     # noqa: F821
print(f"firmware build id {_id} (bench: UNO R4)")
