# omni-robot — working instructions

## Read only what the task needs

**Reading all of `design.md` is not required for every session.** For a local bug fix, test, UI change or tooling task, start with the affected code, its README and any scoped `AGENTS.md`. Read the relevant sections of [design.md](design.md) when the task depends on robot geometry, hardware selection, control limits, architecture, performance claims or project priorities. Read the full baseline for broad design reviews or cross-subsystem design changes.

**Design upkeep:** when a session changes the baseline or establishes an important discovery, update the relevant `design.md` section before finishing. Update `Start here` if status changes and the condensed decision log for lasting decisions/major findings. Routine fixes and passing tests do not need a design entry. Rewrite conclusions in place; detailed procedures, run tables and debugging belong in existing subsystem READMEs/results. Follow the maintenance rules at the top of `design.md`; preserve removed substantive content in `design full history.md`, append-only. Do not start parallel design documents or edit superseded documents in `archive/`.

Keep this file for actionable working rules and pointers, not subsystem descriptions or session history. Put rules needed only in one subtree in its scoped `AGENTS.md`.

## Current entry points

The active simulator is **`simulation/v11 sil`**; the arm baseline is `robot_export.ARM` / `arm_params()`, the coarse v10 model. **The arm robot's only camera is the nose RealSense D435** (`arm_camera:=d435`, the default; `ARM_D435`): run and test the arm robot with it only. The stereo pair belongs to the parked catch robot; use `arm_camera:=stereo` only to reproduce earlier results. Active editable CAD is **`cad/v10 human editable`**. Older sim/CAD versions reproduce earlier work; do not treat their README status as a current adoption decision. Robot computer selection remains open; do not assume a Jetson deployment.

| Task | Start with |
| --- | --- |
| Robot design / build priorities | Relevant sections of [design.md](design.md) |
| Simulation / model / picking | [v11 README](simulation/v11%20sil/README.md), `params.py`, `catalog.py`, `robot_export.py` |
| CAD | [cad/solidworks.md](cad/solidworks.md) (main CAD reference), [cad/AGENTS.md](cad/AGENTS.md), then the active version's README |
| ROS launch / navigation / bridges | [ros/README.md](ros/README.md), `ros/omni.sh`, affected package/config; use design.md for current hardware assumptions |
| Arm / face / calibration | READMEs in `ros/src/omni_arm`, `omni_face`, `omni_calib` |
| Console / supervisor / web UI | [console instructions](ros/src/omni_console/AGENTS.md) and affected source |
| AI control (Claude / Codex via MCP) | [omni_mcp README](ros/tools/omni_mcp/README.md), [ai_robot_architecture.md](ai_robot_architecture.md); its own venv `C:\Users\ferna\.venvs\omni-mcp` |
| Firmware / SIL | [firmware README](firmware/README.md), [pin map](firmware/PINS.md) |
| Parts database | [database README](database/README.md) |
| Pi / split-computer experiments | [rpi.md](rpi.md), [Pi README](characterization/pi_camera/README.md) |
| Camera characterization | The relevant `characterization/<bench>/README.md`; observe the pause below |

Old `§1–31` references generally mean `design full history.md`; legacy `design.md 10–16` means its Appendix A. Appendix B preserves the full design before the Nov 1 compression. Consult history only when needed to trace a decision or reproduce an old study.

## Environment and running processes

- Windows Python: `C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe`. Each venv has a requirements file: `simulation/requirements.txt` (sim), `ros/requirements-robot.txt` (robot nodes' pip-only packages, WSL / robot computer), `ros/tools/omni_mcp/requirements.txt`, `characterization/realsense/requirements.txt`. PATH `python` is PlatformIO's interpreter; never install into it. RealSense bench tools use `C:\Users\ferna\.venvs\realsense` separately.
- ROS 2 Jazzy runs in `Ubuntu-24.04` WSL. Use `ros/omni.sh <step>` to mirror/build in `~/omni_ws` on ext4; never build on `/mnt/c`. WSL Python is `~/.venvs/omni-ros`. Render MuJoCo on Windows (`ros/tools/mujoco_view.py`), not WSL.
- Alongside Fernando's stack, use another `ROS_DOMAIN_ID` and separate ports (e.g. `mcu_port:=5771`). Stop only your own launch/processes by PID; do not use `omni.sh stop`, `stop-console` or broad `pkill` against shared sessions. His console may own 8080/9090.
- Pi deployment/network details live in `rpi.md`. On the 4 GB Pi, keep package builds sequential with `-j2`; do not start an unrestricted parallel ROS build. Use Ethernet and synchronized clocks for meaningful split-computer timing/load measurements.
- Flash firmware only in attended sessions. Do not resume the paused real-camera ball benches (`ball_view`, `throw_track`, `motion_test`) without Fernando. Their resumption gate is stereo global-shutter colour cameras and a mobile robot; this does not prohibit authorized arm/D435 work.
- Preserve real recordings outside Dropbox (`C:/Users/ferna/throw_data/`, Pi `~/throw_data`); do not mix large raw captures into source folders.

## Code and model invariants

- Robot packages must not import `omni_sim`; keep reusable robot cores ROS-free. Keep `firmware/lib/omni_fw` free of Arduino includes.
- Change protocol definitions in `firmware/lib/omni_fw/protocol.h` and `ros/src/omni_mcu/omni_mcu/protocol.py` together; bump both `PROTOCOL_VERSION`s for body changes (the MCU drops CMDs of another version). `pio run` builds the arm variant and stops until the AK40-10 driver exists; check compiles with `pio run -e catch`. Preserve event timestamps and frame conventions.
- One implementation per robot node: where an `omni_cpp` node exists, there is no Python twin node (deleted Nov 7). The v11 sim runs the robot's C++ arm driver (`omni_cpp/sil/arm_sil.py`, built on first use); Python cores it still imports (protocol, modes, catch cores, arm kinematics/motion planning) and their `omni_cpp` twins change together; run their parity tests. Drive TELEOP through the mode manager's prioritized inputs (`/cmd_vel/joy`, `/cmd_vel/manual`, `/cmd_vel`), never a new writer on an existing one. `ModeMux` owns the fire counter for the controlling source; never pass a source's `fire_seq` straight to the MCU.
- After simulation params/catalog/model changes, rerun v11 `robot_export.py` and check generated JSON/header files. Arm SIL uses `fw_params_arm.h`; catch SIL uses `fw_params.h`. Preserve earlier variants/results for reproduction; changed geometry is not validated by old scores. Terrain crossings must use `wheel_model="rollers"`.
- Keep the shared `database/` free of omni-robot imports; values need provenance/sources. Mark estimates and VERIFY items as unconfirmed, never measured.
- If launch steps/options change, keep `ros/omni.sh`, console `supervisor.SIM_STEPS`, `launcher.ps1` and `launcher_full.ps1` consistent. Avoid `clients` as an rclpy Node attribute (it is a Node property).

## Verification

Run checks relevant to the changed subsystem, using the correct interpreter/environment. Do not run every historical simulator for an unrelated change.

| Change | Checks |
| --- | --- |
| v11 model / SIL / exports | From `simulation/v11 sil`: `python -m pytest -q test_v11.py`, plus affected study/tests |
| ROS-free package cores | `python -m pytest -q ros/src/<package>/test`; `omni_cpp/test` checks Python/C++ parity; `omni_mcu/test` the protocol |
| ROS-dependent launch / nodes | Relevant package tests in WSL; affected step through `ros/omni.sh` |
| AI control / omni_mcp | `C:\Users\ferna\.venvs\omni-mcp\Scripts\python.exe -m pytest -q ros/tools/omni_mcp/test`, console `test_ai.py`, arm `test_color_vision.py`; end to end `ros/tools/ai_smoke.py` against the arm step |
| Firmware | `pio run -e catch` in `firmware/`, affected SIL/parity checks and pin checks |
| Database | From `database`: `python actuator_db.py validate` and `python -m pytest -q` |
| CAD | Follow `cad/AGENTS.md` and the active version's validation procedure |
| Characterization | Bench-local tests/README and its specified venv; synthetic checks do not authorize restarting paused live work |

For documentation-only changes, check references, consistency and formatting; simulation runs are unnecessary. When working on a historical sim version, run that version's own tests and preserve its intended configuration.
