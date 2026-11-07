# ros/: the robot's ROS 2 software (monorepo)

This folder is the robot's software, the code that will run on the robot computer, kept in the same repo as the design, the CAD and the MuJoCo sims. Today it runs in WSL2 against the MuJoCo sim; `omni_sim` stands in for the robot's drivers. The robot computer is still open (design.md 4.9: a compact x86 mini PC on Ubuntu 24.04 is proposed; a Pi 4 runs the stack with `nav_profile:=pi`, design.md 4.10). Older comments and READMEs say "the Jetson" for the robot computer; read that as whichever computer is chosen. Numbers and decisions: [../design.md](../design.md) sections 2.1 (software), 4.6 (localization and ROS) and 4.10 (the Pi). Old "design.md 28 / 30" references mean `design full history.md` §28 (the navigation stack) and §30 (v11: the catch code, the MCU link, the firmware in the loop).

**v11:** the sim runs the robot's real MCU firmware (`../firmware`, C++, software in the loop). The ROS side talks to it exactly as it will to the Teensy: `mcu_link` connects to the firmware's USB port (TCP from the bridge in the sim, `/dev/ttyACM0` on the robot). The arm robot (`omni.sh arm`, design.md 2) is the current build; the parked catch game runs on the robot's own code: `catch_perception` (camera frames → ball track), `catch_behavior` and `mode_manager`.

> **Jetson caveat (only if a Jetson is chosen later):** JetPack 6 is Ubuntu 22.04 and ROS 2 Jazzy needs 24.04, so Jazzy would run in Docker there. Check NVIDIA's release notes first.

## Layout

```
ros/
├── omni.sh                ← one command per step, run in WSL (Launcher.cmd in the repo root calls it)
├── setup/install_wsl.sh   ← one-time: ROS 2 Jazzy, slam_toolbox, robot_localization, Nav2, foxglove_bridge, MuJoCo venv
├── src/
│   ├── omni_interfaces/   ← messages: McuState, McuCommand, McuInfo, BallTrack, SparseImage, CatchStatus,
│   │                        TagDetection(s); SetMode,
│   │                        ConsoleCommand                                                              → robot
│   ├── omni_console/      ← the operator console: supervisor.py (serves web/, the page, and starts / stops the
│   │                        steps: the page is the launcher; systemd/ for the Jetson), mission_node (missions,
│   │                        settings, places, zones, programs, logs, status; runs slam_toolbox in the robot
│   │                        step), console_web (the page alone) + ROS-free cores (missions, program, people,
│   │                        calibration, zones, throws, health, session_log, cdr, tflite ...)         → robot
│   ├── omni_bringup/      ← launch + config (EKF, slam_toolbox, Nav2, teleop, RViz), save_map; frames.py:
│   │                        robot_frames (the frame tree, calibrated within bounds), frame_check (REP-105) → robot
│   ├── omni_perception/   ← scan_prep: drop the A2M12's floor returns, de-skew scans (scan_time.py: sample
│   │                        times, ScanClock); camera_time.py: frames stamped from the MCU's trigger;
│   │                        image_preview: a small, slow JPEG copy of a camera for the console            → robot
│   ├── omni_mcu/          ← mcu_link + protocol.py (no ROS): the MCU link, odometry, IMU; clock_sync.py: the
│   │                        MCU clock map from the SYNC_OUT pulses (gpio_edges.py) or the echo         → robot
│   ├── omni_cpp/          ← the robot's always-on nodes in C++ (the only implementation since Nov 7): mcu_link
│   │                        (frames from the firmware's own protocol.h), mode_manager, scan_prep, heading_node;
│   │                        the catch nodes catch_perception, catch_behavior; arm_driver (the SO-101's servo bus);
│   │                        test/test_cores.py checks the C++ cores against the Python cores the sim still uses → robot
│   ├── omni_camera/       ← the camera head (design.md 4.9): libcamera_capture (a CSI camera as
│   │                        /cam/<side>/image_raw, sensor-stamped; needs Raspberry Pi's libcamera, skipped
│   │                        without it), camera_head.launch.py (+ catch_perception, intra-process)  → camera head
│   ├── omni_calib/        ← the calibration workflow's solvers and checks (README.md: the procedure)  → robot
│   ├── omni_params/       ← described ROS 2 parameters for the Python nodes (omni_cpp/params.hpp's twin)  → robot
│   ├── omni_slam/         ← tag_slam (AprilTag landmarks in a factor graph next to slam_toolbox, the
│   │                        localization health), tag_detector                                          → robot
│   ├── omni_face/         ← the face on the 5 in display and the voice: face_node + display.py (a window
│   │                        with sound on Windows in the sim: tools/face_view.py)                       → robot
│   ├── omni_catch/        ← catch_perception, catch_behavior, mode_manager + their ROS-free cores      → robot
│   │                        (flight, tracker, detector, frames, perception, intercept, game, modes, robot_config;
│   │                         config/robot_v11.json is generated by simulation/v11 sil/robot_export.py)
│   └── omni_sim/          ← the MuJoCo bridges (house, catch), the firmware's USB port (mcu_port), viewer,
│                            scoring (catch_scorer too), Nav2 goal runs, joystick relay                  → PC only
├── maps/                  ← saved maps (house = the main one, with its slam_toolbox pose graph)
├── foxglove/omni_layout.json   ← Foxglove layout: 3D (map, scan, true house, robot), status, a Teleop panel
├── tools/mujoco_view.py        ← MuJoCo's 3D view of the ROS sim, drawn on Windows (the bridge starts it)
├── tools/joystick_sender.py    ← Windows side of the joystick relay (Extreme 3D; the 3Dconnexion is ignored)
├── tools/console_smoke.py      ← the operator console checked end to end in Edge (Playwright)
└── tools/cpu_sample.py         ← CPU per process group on the robot side (WSL, the Jetson): what the console costs
```

`omni_sim` depends on the robot packages, never the reverse. On the robot: `colcon build --packages-skip omni_sim`.

## Where it builds

The repo stays in Dropbox. `omni.sh` mirrors `ros/src` into `~/omni_ws/src`, the sim version (`simulation/v11 sil`, set by `OMNI_SIM_VERSION`; `"v10 ros bridge"` runs v10's) into `~/omni_ws/sim` and `firmware/` into `~/omni_ws/firmware`, all on WSL's ext4. It builds there (the SIL library with g++ on first use) and copies maps and results back.

Reasons:
- the Dropbox folder is ~110× slower for small files under WSL;
- Dropbox locks fresh files;
- build output doesn't belong in Dropbox.

Nothing is ever built on `/mnt/c`, so Dropbox needn't be paused. On the Jetson, clone the repo from GitHub and build in place.

## Run it

**Without a terminal:** double-click `Launcher.cmd` in the repo root, pick the options (MuJoCo 3D view, Foxglove,
Fast, Kid, map), then **Console (start here)**. The page opens at `http://localhost:8080` once its server is up, and
the robot's software starts by itself (the *Robot (all modes)* step), as it will on the robot at boot.
- **Home:** a card per mode (Play catch, Drive, Follow me / Tag, Program it, Go to, Explore, Patrol, Thrower test).
  Catch in the sim plays in its own world: Home's *Switch to the catch world* (and *Back to the house*).
- **Robot > System:** every step with its options (Start, Stop, Restart), the running step's log, the robot computer,
  and in the sim the fault injection. *Stop console* in the Launcher stops the page server and its step.
- Every tab has a "How to use" box. For debugging, *Open Foxglove*.

The step buttons still work on their own (each serves the page itself; then *Open console*). The bottom panel
shows each job's log.

**The operator console** (`omni_console`; design.md section 2) is served by the robot itself.
- **Connection.** In the sim the page is at `http://localhost:8080`; on the robot it is `http://omni.local:8080`, on the PC or an iPad. It talks to ROS through rosbridge on port 9090 and to the supervisor over plain HTTP (`/api/*`, `/logs/`). One-time install: `sudo apt install ros-jazzy-rosbridge-suite`.
- **Topics.** The page uses `/console/*` (a status at 10 Hz, the path, settings, events, the last throw) and reads three topics as the publisher's raw bytes (rosbridge `cbor-raw`, decoded in the browser): `/map`, the scan (`/scan_filtered`) and `/mcu/samples` (the Base tab). It sends commands through `/console/command` and drives through `/console/cmd_vel`. The sim's fault injection goes on `/sim/fault`.
- **Cameras tab** (Robot ▸ Cameras): the cameras on the robot computer, from `omni_console/camera_snap.py` there (plain HTTP on :8086, no ROS, never rosbridge). Today that is the Pi's Arducam plus the D435's colour, left-IR and colourized depth images; click the depth image for a range. **Snapshots** fetch single JPEGs, on demand or one every 1–10 s. **Live video** streams each picture whose *Live* is on (`/stream`: multipart JPEGs, each with its info). Stream one picture for the fastest rate: on the Pi 4 at 640 px, the Arducam runs 30 / 56 / 99 fps in its modes and the D435 30 fps. Each camera's capture mode (resolution and rate), the Arducam's focus and the D435's IR projector are set from the tab (`/set`). **AprilTags** (`omni_console/apriltag_view.py`: the C apriltag library, else OpenCV) are found on the robot computer; the overlay and table show the pose, `omni_slam.tags`' verdict (usable, ambiguous or refused) and the lock's scatter over 5 s. Results: `characterization/apriltag/README.md`. Start it with `bash ros/tools/cam_snap.sh start` (Git Bash; `OMNI_PI=omni-cam-eth` over the cable, then Server `http://192.168.0.101:8086`; `stop`, `status`, `fake` for test patterns with two tags on the PC). A camera opens at its first request and closes 20 s after the last, or after its last live stream ends. The server address is set in the tab: by default the page's own host, or `omni-cam.local` when the page is on localhost.
- **Safety.** The robot stops 0.3 s after the page stops sending, for example when the dead man is released, the tab is closed or the Wi-Fi drops.
- **Saved data** (`~/.omni` on the robot): settings `console.json`, programs `programs.json`, calibration results `calibration.json`, health counters `health.json`, throws `throws.jsonl`, session logs `logs/sessions/*.jsonl`, the steps' logs `logs/steps/`; `slam_pose.json` (the last localized pose, its
odometry and the MCU's clock: a console restarted within 2 min on the same MCU boot localizes from there, not the map's
start) and `slam_toolbox.pgid`
(its slam_toolbox's process group: a console that died without stopping it leaves it running in its own session, and the
next console stops it first). Per map in `maps/<map>.places.json`: places, routes, no-go zones and floor edges.
- **On the Jetson:** `omni_console/systemd/README.md` (the supervisor at boot, `omni.local`, shutdown from the page).

**With a terminal** (WSL, or PowerShell with `wsl -d Ubuntu-24.04 -- bash`):

```bash
O="/mnt/c/Dropbox/Personal Projects/Robotics/omni-robot/ros/omni.sh"
bash "$O" map          # drive the house (stick or Foxglove's Teleop panel), slam_toolbox mapping
bash "$O" save-map     # (second terminal) -> ros/maps/house
bash "$O" localize     # slam_toolbox localization on it
bash "$O" nav          # + Nav2: goals from the console's Go To tab (or Foxglove's 3D panel / RViz)
bash "$O" robot        # everything at once for the console: Nav2 + slam_toolbox run by the console (new map,
                       # save, load), Manual, Go To, Explore, Patrol, Thrower; web_video_server on :8081
bash "$O" arm          # the arm robot (the current build): 'robot' plus the SO-101, its nose D435 (the default and
                       # only arm camera; arm_camera:=stereo reproduces earlier runs) and the Tidy tab;
                       # objects:=toys|clutter|colours, props:=true map:=house_furnished; depth_layer:=raw
                       # (default) | height (proposed: omni_arm's fused height map, speed by certainty, the
                       # AI's floor catalog) | floor (Nov 5's per-frame grid) | off
                       # (the sim's D435 has realistic depth errors; d435_noise:=ideal is the Nov 4 model)
bash "$O" catch        # v11: the catch game (thrower, cameras, the robot's catch code and firmware)
bash "$O" catch-score tosses:=24   # the same, scored: studies/results/ros_catch.json
bash "$O" sim          # just the bridge; also: auto-map, tour, goals, score-map, build, test, stop, status
bash "$O" console --autostart robot person:=true   # the page server + launcher (Launcher's "Console"); stop-console
```

With `firmware:=true` (the default) every step but `tour` and `auto-map` runs the robot's C++ firmware inside the sim. `mcu_link` then publishes `/odom`, `/imu` and odom → base_link from the MCU, and `mode_manager` (TELEOP) forwards a twist to it, ramped and capped by the firmware. `firmware:=false` is v10's bridge.

**Parameters: what can be tuned.** The robot nodes' tunable values are ROS 2 parameters, each declared with a
description, a range for numbers, and read-only (set at start) or live (changes at once). Their values for the robot
are in `src/omni_bringup/config/robot.yaml`, one section per node; the launch loads it into each node and puts its
launch-specific values (ports, modes, sim time) on top.
```bash
ros2 param list /mode_manager                      # what a node has
ros2 param describe /mode_manager pick_timeout     # what it means, its range, read-only or not
ros2 param set /arm_driver grip_torque 400         # live ones only; ROS refuses values out of range
ros2 param dump /arm_driver                        # copy tuned values back into robot.yaml
```
Foxglove's Parameters panel shows and sets the same. In C++ declare new ones with `omni_cpp/params.hpp`
(`params::number / integer / flag / text`, `params::Live` for live ones), in Python with `omni_params`
(`Params(node).number / flag / text / array`, `live=` for live ones), add the value to `robot.yaml`;
`omni_bringup/test/test_robot_params.py` checks the file against the declarations. Not parameters: Nav2 / SLAM / EKF
(their own yaml), the console's operator settings (the page), the firmware's limits (compiled in: the ceiling a host
can only ask under).

**Who drives (TELEOP).** Each source has its own topic and the mode manager takes the highest-priority one that is
fresh (0.5 s): `/cmd_vel/joy` (100, the joystick), `/cmd_vel/manual` (90: Foxglove's Teleop panel, the console's stick
when it bypasses Nav2's collision monitor), `/cmd_vel` (50: Nav2's collision monitor; the console's guarded driving
goes through it). `/cmd_vel/lock` (Bool) true stops TELEOP; `/cmd_vel/active` names the source in charge. Publish to
the right topic: two writers never share one. With the arm robot a missing or stale `/arm/out` caps the base as if the
arm were out (`mode_manager` parameter `arm`, true in the arm step).

Arguments are `name:=value` after the step:

| Argument | What it does |
| --- | --- |
| `viewer` | MuJoCo's 3D window |
| `foxglove`, `rviz`, `joy`, `console` | turn those on or off (`console`: rosbridge + the operator console, on for the interactive steps) |
| `mcu_port` | the sim firmware's USB port on localhost (5761); another value with another `ROS_DOMAIN_ID` runs a second stack alongside |
| `fast` | the firmware at 1 kHz and the lidar in 2 ms batches (≈ real time); on for the interactive steps, off for the scored ones |
| `map`, `seed`, `flatness`, `slam_range`, `floor_filter`, `deskew`, `rtf` | as in [../simulation/v10 ros bridge/README.md](../simulation/v10%20ros%20bridge/README.md) |
| `firmware`, `kid` | v11: the C++ firmware in the loop (default true); the firmware's kid profile |
| `dist`, `floor`, `returns`, `tosses`, `pass_game` | the catch steps: thrower distance (m), `flat` / `home`, return throws, throws before `catch-score` ends, two throwers taking turns (the robot returns to the other one) |
| `person` | the house steps: a person walking ahead of the robot's start, for Follow / Tag |
| `web` | `false` when the supervisor serves the page (it passes it) |
| `arm_hz` | arm step: omni_cpp's `arm_driver` rate (servo bus, joint states, TF; default 50 Hz) beside omni_arm's `arm_brain` (planning and the tidy). The Python `arm_node` and the `arm_impl`, `glue` and `catch_impl` switches were deleted Nov 7: every robot node runs its C++ version |
| `nav_profile` | `lean`: Nav2 and slam_toolbox set for a smaller computer (design.md 4.9); `pi`: a Pi 4 robot computer at 0.6 m/s, slam_toolbox corrected less often, the EKF at 30 Hz, arm_node at 20 Hz, the collision monitor without its odom shift (design.md 4.10); the default when a Raspberry Pi runs the robot's nodes (`full` elsewhere) |
| `localizer` | localization, navigation, tour, goals: `slam` (default): slam_toolbox's localization on `maps/<map>.posegraph`; `amcl`: Nav2's AMCL (omni motion model, `config/amcl.yaml`) on `maps/<map>.yaml` via map_server. In the robot and arm steps `amcl` replaces the console's slam_toolbox (no Explore or map switching); it localized worse than slam_toolbox in the house (design.md 4.10) |
| `catch_sleep` | `true` (default): the catch nodes sleep outside CATCH (no `/mcu/state`, cameras or fast timers) and wake on `/robot_mode` |
| `compose` | `true` (default): Nav2's servers and heading_node in one `component_container_isolated` (`nav2_container`); `all`: + mcu_link, mode_manager, scan_prep; `false`: a process each |

**The catch game's topics** (design.md 30):

| Topic | From → to | What |
| --- | --- | --- |
| `/cam/left/sparse`, `/cam/right/sparse` | catch_bridge → catch_perception | the cameras' frames (`SparseImage`: the ball's pixels; `source:=image` takes `sensor_msgs/Image` instead) |
| `/mcu/state` | mcu_link → all | the MCU's STATE at 200 Hz: odometry, attitude, tilt, flywheels, beams, mode, faults |
| `/catch/track` | catch_perception → catch_behavior | the filter's state and covariance (`BallTrack`) |
| `/catch/cmd` → `/mcu/cmd` | catch_behavior → mode_manager → mcu_link | the request (`McuCommand`), sent at 100 Hz as the MCU's heartbeat |
| `/catch/status`, `/catch/ball`, `/catch/target`, `/catch/perception` | | state, markers, processing times |
| `/set_mode` (service) | → mode_manager | `IDLE`, `TELEOP`, `CATCH` or `THROWER`, kid on / off |
| `/thrower/cmd` | omni_console → mode_manager | the Thrower tab's tilt, flywheel rpm, fire (`McuCommand`), in THROWER |
| `/mcu/info` | mcu_link → omni_console | the MCU's INFO (`McuInfo`, 10 Hz): protocol version (6), firmware build id, RC channels, motor temperatures |
| `/mcu/clock`, `/mcu/time_ref` | mcu_link | the clock map's state (JSON, 1 Hz); camera triggers and strobes (`TimeReference`) on the Jetson's clock (design.md 10) |
| `/tags/detections`, `/tag_slam/*`, `/localization/health` | tag_detector (the bridge in the sim) → tag_slam → omni_console | AprilTags, the tag map and pose, the localization's level and speed scale |
| `/mcu/state_10hz`, `/mcu/samples` | mcu_link → omni_console, the page | the STATE every 20th; the 200 Hz samples batched (`Float32MultiArray`, only while someone listens) |
| `/sim/catch_result`, `/sim/ball`, `/ground_truth/odom` | catch_bridge | the truth, for scoring and views |

**Speed:** with `fast`, the whole interactive stack runs at ≈ 0.95–1.0× real time on this PC (~1.0 standing, 0.94–0.98 driving, ~0.9 pushing a wall, 0.97 with RViz too).
- **The MuJoCo view is drawn on Windows** (`tools/mujoco_view.py`, native OpenGL, ≥ 20 frames/s). The bridge streams the state to it over localhost and starts it through WSL's interop.
- Drawing it inside WSL was the problem: WSLg's GPU path took ~0.8 s a frame for MuJoCo's many small OpenGL calls, and software rendering took the sim's CPU.
- What still draws in WSL (RViz) uses software OpenGL on 4 threads (`LP_NUM_THREADS=4`).

**Foxglove:** use the Foxglove app (a current version: foxglove_bridge 3.6 speaks the SDK protocol). *Open connection → Foxglove WebSocket → `ws://localhost:8765`*. The first time, *Layout → Import from file → `ros/foxglove/omni_layout.json`*.


## PC regression checks

The [Nov 3 desktop stack regression](../simulation/v11%20sil/studies/results/pc_stack_2026-11-03.md) covers manual driving and goals, SLAM/map switching, exploration, patrol, arm poses, navigate-then-tidy and injected stops. It records both successes and the remaining Nav2 container shutdown issue.

Use `tools/console_smoke.py --slam --tabs --faults` against the supervisor's `robot` step with `person:=true`. It resets an already-enabled simulated person for Follow and supplies the return-map initial pose after Explore. For repeatable automation, pass `OMNI_FACE_ARGS="--mute --listen none"` when launching, so microphone input cannot steer a scripted test.

For the arm, pass camera, implementation and clock options directly to `omni.sh arm`, then run `tools/arm_load.py <out.json> --phases NAVTIDY,ROUTE --legs 1 --laps 1 --tidy-timeout 600` in the same ROS domain. The timeout is wall time: a 240 s budget can cut off a six-object tidy when the simulator runs below real time. Reports distinguish a started task, timeout, task pick count and physical bin count; picking is skipped after failed navigation.
