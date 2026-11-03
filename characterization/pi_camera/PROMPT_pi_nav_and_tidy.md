# Prompt: does `nav_profile:=pi` let a Pi 4 run the arm robot? (route, explore, navigate-then-tidy)

Paste everything below the line into a new Claude Code session in the omni-robot repo.

---

Read `design.md` first (section 4.10, "The whole arm robot on the Pi 4, over Ethernet", and the `nav_profile:=pi` bullet under it), then `rpi.md` and `AGENTS.md`. Do not start parallel design docs; update design.md when you finish.

## Goal

Find out, by simulation, whether the **leaner navigation and SLAM numbers (`nav_profile:=pi`)** let a **Raspberry Pi 4 run the whole arm robot correctly**: it must navigate a route, explore, and then drive to a cluster of floor objects and tidy them with the arm. The Pi runs the robot's code (`side:=robot`), the PC runs the world (`side:=sim`: MuJoCo plus the firmware in the loop). "Correctly" means the robot does the job, not just that the CPU number is below 100%.

## What is already known (Nov 1; do not re-derive)

- Everything on the Pi with the default Nav2/SLAM: 3.5–3.6 of 4 cores, busiest core pinned, **patrol aborts after 2–3 waypoints** ("Transform data too old", 23 aborts), EKF missed its 100 Hz rate 714 times. The arm tier alone (`nav:=false`) takes 2.1–2.25 cores and the tidy works (2 picks per 100 s).
- Biggest processes (% of a core): Nav2 container 95–100, `arm_node` 77–90 (Python rclpy overhead, not arm work), EKF 34, `mission_node` 25–60, `mcu_link` 26, face 27, `scan_prep` 19, slam_toolbox 11 parked and up to 99 driving.
- `nav_profile:=pi` (`ros/src/omni_bringup/omni_bringup/stack.py`: `PI`, `PROFILE_EXTRAS`) is a **hypothesis, never run on the Pi**: 1 m/s cap (MPPI, velocity smoother, console `speed_cap`), MPPI 400 × 24 at 10 Hz, 5 m local costmap at 0.075 m, costmaps at 2 / 0.5 Hz, slam_toolbox every 0.5 s / 0.4 m with no loop closing, EKF at 30 Hz. It only edits `nav2.yaml` and `slam_localization.yaml`: **`slam_mapping.yaml` (used by Explore with a new map) is not covered** and mapping runs loop closing, so decide whether to extend the profile before testing Explore (see below).
- The sim runs at ~0.7× real time on the PC (one MuJoCo thread), so sensor-driven loads are understated by up to ~30%. Use `clock:=wall` as the earlier runs did and say so in the results. Do not "fix" the sim's speed in this task.

## Setup you must check first (all done once on Nov 1; verify, do not assume)

1. Pi: `ssh omni-cam-eth` works (`~/.ssh/config` alias, Pi on the cable at 192.168.0.101, PC 192.168.0.100). The Pi has **no internet** on the cable (no pip/apt); Wi-Fi is the other interface.
2. PC admin firewall rules exist: `Get-NetFirewallHyperVRule -Name WSL-MCU-TCP,WSL-ROS2-DDS-ETH`. You cannot create them (no admin); ask Fernando if missing.
3. **Deploy the current code to the Pi first**, it changed after the last deploy (`mission_node.py` has the new `speed_cap`; `stack.py` the profiles): `OMNI_PI=omni-cam-eth OMNI_PI_PKGS="omni_interfaces omni_mcu omni_catch omni_perception omni_cpp omni_camera omni_slam omni_calib omni_arm omni_face omni_console omni_bringup" bash characterization/pi_camera/pi.sh build` (sequential, ~1.5 min). Also `scp ros/tools/pi_keepup.py omni-cam-eth:omni_ws/tools/`.
4. Clocks: with `clock:=wall` the Pi and the PC must agree to a few ms: `python3 ros/tools/clock_offset.py serve` in WSL and `probe` on the Pi (rpi.md). Windows' time service is stopped; the Pi's clock was stepped by hand on Nov 1 and drifts. Re-measure; step the Pi with `sudo date -s` / `clock_settime` if off by more than ~5 ms.
5. **Stale Fast DDS shared memory hangs lifecycle nodes** (slam_toolbox never configures, Nav2 never gets a map frame): `characterization/pi_camera/arm_split.sh` clears `/dev/shm/fastrtps*` on the Pi before and after every run. If a run ever shows "Waiting on external lifecycle transitions" forever or `Timed out waiting for transform from base_link to map`, suspect this before suspecting the profile.
6. Fernando's own stack may be running in WSL (domain 0, ports 5761/5763, 8080, 9090). Never `omni.sh stop`; the scripts here use domain 43 and ports 5771/5773 and stop only their own processes. Never `pkill` broadly on the Pi (the ball view may be running there).

## The harness (exists; extend it)

- `bash characterization/pi_camera/arm_split.sh LABEL [launch args]` runs both halves, samples the Pi (host + per-process CPU every 5 s, `pi_keepup.py --no-ros --raw`) and drives phases from the PC with `ros/tools/arm_load.py` (MANUAL, TIDY, PATROL; each phase writes start/end status). Env: `PHASES`, `PHASE` (s), `NAV_PROFILE` (full | lean | pi), `CLK`, `FACE`, `PROFILE=1` (cProfile of `arm_node`). Extra launch args go straight through: `face:=false`, `nav:=false`, `ekf_hz`, `arm_hz`, `arm_camera:=d435`.
- `python ros/tools/arm_load_report.py results/pi_arm_<tag>.json results/logs/arm_load_<tag>.json` prints CPU per phase and per process.
- Known harness limits: `arm_load.py` records only the patrol's start/end status, not per-waypoint outcomes; it has no Explore or navigate-then-tidy phase; the Pi monitor does not record topic rates (it hung in rclpy start-up on an overloaded Pi, so it is `--no-ros`). **Extend `arm_load.py` (and the report) as below. Keep it ROS-free where it can be; it runs in WSL with rclpy.**

## What to build

1. **A route phase with a scorecard.** Patrol the eight house spots (`ros/tools/soak.py` `SPOTS`, the same route as `mode_load.py`) at the profile's speed for at least 3 laps, or better, send the goals one by one through the console's `goto` verb so each has a result. Record per goal: reached or aborted, time, final distance error to the goal (sim truth from `/sim/origin` + the pose, or the bridge's truth), the abort reason from the Pi's log (`Transform data too old`, `Goal failed`, costmap or controller errors), and the distance driven. Add the speed actually reached.
2. **A navigate-then-tidy phase ("NAVTIDY").** Find where the floor objects are (the sim's truth: `arm_objects.positions`, or the bridge's `/sim/objects` topic, or the objects' count in the bridge log), `goto` a spot a metre or so short of the nearest cluster through Nav2, wait for arrival, then call the console's `tidy` verb. Score: arrived (distance to the target), tidy picks and failures (`/arm/state` `tidy`: `picked`, `failed`, `skipped`), time per pick, and whether Nav2 sat idle during the tidy (the base is capped to 1 m/s while the arm is out: `mode_manager` `ARM_OUT_CAPS`). Repeat for two clusters so a second leg starts from the first tidy's end. **The Pi must switch cleanly from driving to picking and back**, and report CPU separately for the drive leg, the tidy leg and the transition.
3. **An Explore phase.** Explore with a new map exercises slam_toolbox in *mapping* mode (loop closing on, bigger work). Decide: (a) explore on the existing map (`explore` with `new_map:false`, localization mode, what `pi` covers) first; (b) then, if (a) passes, extend `PI` in `stack.py` to `slam_mapping.yaml` (the same four levers; add the replacement strings, extend `test_profiles.py`, run `python3 -m pytest -q ros/src/omni_bringup/test` in WSL) and run a new-map explore. Score: area covered, time, map quality against the house truth (`simulation/v11 sil/map_quality.py` if it applies), and aborts.
4. **Controls.** Run every scored scenario with `NAV_PROFILE=full` as well (the same harness and clock) to confirm the baseline fails the way it did, and with `lean`, so the profile is shown to be the difference and not luck. Run each scenario at least twice (the sim is not fully deterministic). Optionally run `full` on the PC alone (`omni.sh arm` with no split) as the "what correct looks like" reference for goal accuracy and time.

## Pass criteria (judge them, then state where they fail)

For `nav_profile:=pi` on the Pi 4 with the face on, `arm_camera:=d435`, the arm idle while driving:
- Route: ≥ 7 of 8 goals reached per lap, no more than 1 abort per lap, mean goal error ≤ 25 cm, top speed within 0.8–1.0 m/s, no `Transform data too old` in the Pi log.
- Navigate-then-tidy: arrives within 30 cm of the target, tidy picks ≥ 80% of what `full` on the PC picks in the same time, no arm or Nav2 crash, the drive and the arm never both heavy.
- Pi: median < 85% of all cores and the busiest core p95 < 95% in every phase, no throttling (`throttled` 0x0), memory available > 1 GB.
- Control rates (read them from the Pi log or a light rate check, not by adding a Python subscriber to a high-rate topic): the controller holds 10 Hz, scans reach ≥ 9 Hz of the sim's, the EKF does not log "Failed to meet update rate" more than a few times a minute.

## If it fails: levers, in order (change one at a time, record each)

1. Put `arm_node` to sleep while Nav2 drives (it costs 70–90% idle): publish joint states at ~5 Hz and TF at ~2 Hz while stowed, cache the forward kinematics, or cut `arm_hz`; this is a code change in `ros/src/omni_arm/omni_arm/arm_node.py` (the profile in `logs/*_arm_node.prof`: publishing, executor bookkeeping, `kinematics._chain` / `fk`). Keep tests green (`python -m pytest -q ros/src/omni_arm/test`).
2. `face:=false` (and no display on the Pi) while driving.
3. Replace slam_toolbox localization with AMCL (`localizer:=amcl`, design.md 4.9) and/or drop the EKF; replace MPPI by Regulated Pure Pursuit (the default "forward" driving mode already faces the path).
4. Lower the cap (0.8 m/s), shrink the local costmap further, run the depth layer at 2 Hz or off while driving.
Report the **navigation and driving performance hit** for each: top speed, goal accuracy, time per lap, aborts, stopping distance at the cap, and what the robot can no longer do (strafing, tight avoidance, 2 m/s).

## Rules

- No hardware: this is simulation and the Pi as a computer. Do not flash anything.
- Do not touch `cad/v7`. Do not run `omni.sh stop` or `stop-console`. Kill only your own processes by PID.
- Results go in `characterization/pi_camera/results/` (the scripts do this); put the numbers that matter in design.md 4.10 next to the Nov 1 bullet, with run labels, mark estimates `(est)` and anything unconfirmed `VERIFY`, and add a decision-log row. Do not present the sim's 0.7× real-time caveat as solved.
- Ask Fernando before anything outward-facing (committing, pushing) and before the admin-only steps.
- At the end give a short verdict first: does the Pi 4 run the arm robot with `nav_profile:=pi`, yes / no / only if X, then the table of numbers, then what to scale back next.
