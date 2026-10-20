# omni-robot (play-catch kiwi robot)

Read `design.md` first. It is the single source of truth: the current design, its expected performance from simulation, a repo map, run commands, next steps and the decision log. Superseded documents live in `archive/`; don't edit them or start parallel design docs. When the design changes, update the matching `design.md` section, its decision log and the "Start here" status.

- Python: use `C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe` (MuJoCo, build123d). `python` on PATH is PlatformIO's interpreter; never install into it.
- The current sim is `simulation/v2 detailed physics`. Check `python -m pytest -q` there after changes. Part data lives in `catalog.py`, the fitted configuration in `params.py`.
- `simulation/v3 compact` is a proposed compact redesign (4-wheel X base, tilting catch/launch module; `design.md` section 20). It stays "proposed" until Fernando adopts it; its tests are `test_v3.py`.
- `simulation/v5 unlimited` is an exploration on top of v3b (`design.md` section 21): no firmware limits by default (`limits="firmware"` restores them), a per-pixel vision model, and a hopper mode. Its tests are `test_v5.py`.
- `simulation/v6 shallow throw` builds on v5 for throwers 6-7 m away (`design.md` section 22): chute feed (floor sloping into the nip of two driven feed wheels, J-bend to the flywheels, one ball per trigger; round 1 roller feed kept), padded pocket, 35° catch tilt, honest stereo track start, ceiling-aware returns, tube/indexer hopper options. Round 3 (`design.md` section 23): GM3506 feed motors modeled from datasheets, viscoelastic contacts (`contact_model.py`), randomized worlds (`randomize.py`), rolling resistance / friction fields / thresholds / carpet (`floor.py`), a motor study. Proposed; its tests are `test_v6.py`.
- `database/` is the shared actuator database (`design.md` section 24): motors, servos, controllers and sources as CSV, with `actuator_db.py` (validate, gaps, sim export). Every value carries provenance and a source; run `python actuator_db.py validate` and `python -m pytest -q` there after edits. It will move up to `Robotics/database/`, so keep it free of omni-robot imports. `studies/actuator_choice.py` applies it to this robot.
- CAD is in `cad/`. Dimensions come from the sim through `dims.py`. After CAD changes, run `build.py` (it must still report 0 clashes) and `render.py`.
- Parts are 3D printed wherever possible (PLA, PETG, TPU; ~256 mm bed).
- Values marked (est) or VERIFY are unconfirmed. Don't present them as measured.
