# Operator console working rules

For local console changes, start with the affected source/tests; read design.md only when changing the robot's design, architecture, operating limits or mission semantics. `web/` uses plain ES modules with no build step; vendored libraries are documented in `web/vendor/README.md`.

- Add features through verbs/settings in `missions.py` and the existing `/console/command` service, not new service types.
- The robot sends data; the browser decodes and draws. Map, scan and MCU samples use raw rosbridge subscriptions (`cbor-raw`, `web/js/cdr.js`). Rosbridge shares a subscription per topic: subscribe raw everywhere or nowhere. Do not add images or other high-rate topics through rosbridge; use the separate camera preview path.
- Drain busy inputs with `Feed.drain`, not a callback for every sample. Do not run console nodes on sim time: the high-rate clock creates substantial Python overhead.
- Keep the supervisor's `SIM_STEPS`, `ros/omni.sh` and both root Launchers consistent. Preserve the supervisor/page-as-launcher flow and runtime mode switching.
- Test ROS-free cores with `python -m pytest -q ros/src/omni_console/test` from the repo root using the sim venv. Supervisor/rclpy checks run in WSL. For affected end-to-end flows use `ros/tools/console_smoke.py`; its `--tabs --faults --catch --cpu` and `--slam` options cover broader regressions when warranted.
- Use a separate ROS domain/ports alongside Fernando's stack. Do not stop his supervisor or console. Remove only maps created by your own tests from both `ros/maps` and `~/omni_ws/maps`, because the launch script copies maps back; preserve existing user maps.
