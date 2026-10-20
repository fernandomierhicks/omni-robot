# Play-Catch Robot — Design Document

Oct 16, 2026 · Fernando

> **Archived.** Superseded by [../design.md](../design.md), which carries this content forward with the Oct 17 simulation results and design changes.

## Overview and goals

We are building a kiwi omni-wheel robot that tracks a tossed ball, drives under it, catches it in a funnel and throws it back with a flywheel launcher. Most tuning and training happens in a realistic MuJoCo simulation; the real robot is used for attended calibration and validation sessions.

**Goals**

- Fun to play catch with, including with kids (a dedicated kid mode).
- Technically impressive: stop-and-catch first, moving catches later.
- Improvable by Claude between sessions, mainly through overnight simulation runs.
- A general mobile platform: lidar SLAM, RC driving, a speaker, and swappable top modules (catch module now, robot arm later).

**Using this document in Claude Code:** treat each section's decisions as the current baseline. Numbers marked provisional are inputs to the simulation spec sweep and may change. Open questions and items to verify are collected at the end.

## Requirements and time budget

v1 targets stop-and-catch of gentle underhand throws; all numbers below are provisional and will be settled by the simulation spec sweep.

| Parameter | v1 target | Rationale |
| --- | --- | --- |
| Throw distance | 2–4 m, underhand | Gentle play, kid-friendly |
| Apex above release | 1–2 m | Gives about 1.0–1.3 s of flight |
| Detect + fit trajectory | by ~0.25 s after release | Leaves ~0.7 s to move |
| End-to-end latency | < 100 ms | Camera exposure to motor command |
| Base acceleration | 4–5 m/s² | Near omni-roller traction and tipping limits |
| Peak speed | ~1.4 m/s | Bang-bang stop-and-catch over a 0.7 s move only reaches a·t/2 |
| Landing prediction error | ±5 cm | Combined with ±5 cm positioning fits a 30 cm funnel |
| Catch zone | ~1 m diameter (v1), 1.5–2 m with moving catches | Reach is ~0.37 m when the robot must stop under the ball |
| Ball | 70–75 mm hollow plastic or foam training ball, matte, bright color | Low bounce for funnel settling, safe for kids, easy color detection |

Key insight: requiring a full stop under the ball halves reach versus catching while still moving. Stop-and-catch is chosen first because it is easier to perceive, control and log; moving catches are a later upgrade. The real error budget is about ±11 cm (funnel radius minus ball radius), and perception accuracy drives it more than raw speed.

## Drive base

A three-wheel kiwi base driven by CubeMars AK40-10 KV170 integrated actuators on a 7S Li-ion pack.

| Item | Decision | Rationale |
| --- | --- | --- |
| Layout | Kiwi (3 omni wheels at 120°) | Equal agility in every direction, all wheels always touch the ground, no suspension; mecanum loses speed and grip sideways |
| Wheels | Double-row omni, 100–125 mm | Smooth rolling between rollers; practical acceleration limit ~3–5 m/s² before slip |
| Motors | 3× [CubeMars AK40-10 V3.0 KV170](https://store.cubemars.com/) | Motor, 10:1 gearbox, FOC driver and encoders in one unit over CAN; 24 V, ~370 rpm rated, 1.3 N·m rated / 4.1 N·m peak, ~190 g |
| Battery | 7S Li-ion with BMS (~25.9 V nominal, ~5 Ah, ~130 Wh) | Stays in the preferred 24–28 V range; 7S gives ~10–20% more speed than 6S; Li-ion is safer than LiPo |
| Mass target | 6–9 kg total | Sizing basis for torque and tipping |

**Sizing check:** worst-case wheel force is about 25 N (8 kg at 4 m/s²), about 1.6 N·m on a 125 mm wheel. That is above the AK40-10's rated torque but within peak, which is acceptable for bursts under 0.5 s. A thermal check under repeated cycles is still needed. At 370 rpm, a 125 mm wheel gives ~2.4 m/s, well above the 1.4 m/s stop-and-catch need.

**Rejected motor options:** AK60-6 (fits, but ~$300+ each and heavier), AK45-10 (needs ~200 mm wheels for speed at 24 V), AK45-36 (52 rpm, far too slow), RobStride 01/02 (need ~36–48 V for speed), Damiao DM-J6006 (too slow at 24 V).

**Tipping:** the funnel and launcher raise the center of gravity. Tipping starts at roughly g × (half base width ÷ CG height), so keep the battery and motors low and the base wide (e.g. 45 cm wide with CG at 40 cm tips near 5.5 m/s²).

## Compute and real-time control

A Jetson Orin runs perception, planning, SLAM and logging; a separate microcontroller runs motor control and is the safety authority.

**Main computer: Jetson Orin.** Start with the Orin Nano Super dev kit ($399) or an Orin NX 16GB module (~$999 plus a ~$150–250 carrier). Both share the same module footprint, so a carrier board can take the Nano now and the NX later. Prices reflect NVIDIA's July 2026 increase ([CNX Software](https://www.cnx-software.com/2026/07/22/nvidia-increases-the-price-of-jetson-modules-and-devkits-by-up-to-101/?amp=1)).

- Why Jetson over a Raspberry Pi 5: more MIPI camera lanes, CUDA for neural-net detection and visual SLAM (Isaac ROS), and headroom for an arm and more cameras later.
- Why not AGX Orin or AGX Thor: overkill and now $3,499 / $5,499 dev kits. The cheaper Thor T3000/T2000 modules expected Q1 2027 ([ServeTheHome](https://www.servethehome.com/nvidia-announces-expanded-jetson-thor-lineup-with-mid-range-t3000-and-t2000-modules/)) are a possible future upgrade.
- Considered: x86 mini PC (cameras only over USB, no CUDA), RK3588 boards (weak robotics software support), VOXL 2 (drone-oriented, expensive).
- Runs full Ubuntu (JetPack) with a desktop; switch to headless boot when deployed to free ~1 GB of RAM.

**Real-time microcontroller: STM32G4/H7 class with native CAN-FD** (Teensy 4.1 acceptable).

- Runs the 500 Hz–1 kHz base control loop and talks to the three AK40-10s over CAN.
- Enforces velocity, acceleration, current and flywheel-RPM limits; reads bumpers, e-stop and the RC receiver directly; stops the motors if the Jetson heartbeat stops.
- The Jetson only requests desired velocities, so a bad high-level change cannot exceed firmware limits.

## Software architecture and development workflow

ROS 2 for the overall system, with one low-latency catch path, a simple custom link to the microcontroller, and development done remotely on the Jetson from a Windows PC.

**Architecture**

- ROS 2 provides the RPLidar driver, slam_toolbox, Nav2, MoveIt (future arm), rosbag logging and Foxglove visualization, so none of that is rebuilt.
- **Fast catch path:** camera capture, ball detection, stereo triangulation, trajectory fitting and landing prediction run in one process (composable nodes or one C++ node), passing data in memory. It publishes only the target "go to (x, y) by time t" plus diagnostics.
- Camera frames carry hardware timestamps, and Jetson and microcontroller clocks are synchronized, so fits use real capture times.
- **Microcontroller link:** small custom protocol over CAN or UART (velocity commands, heartbeat, status), not micro-ROS, to keep the safety MCU simple.
- Robot software runs in Docker so the PC simulation and the Jetson run identical code and ROS 2 versions.

**Development workflow**

- Windows PC is the front end: VS Code Remote-SSH into the Jetson for editing, building (colcon, natively on ARM) and running.
- Claude Code runs on the Jetson, so it can edit, build, read logs, run tests and restart nodes directly on the hardware.
- The MCU programmer (ST-Link or USB DFU) plugs into the Jetson, so flashing, serial console and GDB all work remotely. Flashing is allowed only in attended sessions and is blocked in unattended configurations; hard safety limits live in a separate firmware module changed only deliberately, together.
- One git repo on the Jetson, pushed to GitHub. Tunable parameters (YAML) are kept separate from algorithm code.
- **Visualization:** Foxglove on Windows connects to foxglove_bridge over WebSocket, so no ROS networking or Linux is needed on the PC.
- **Simulation on the PC:** WSL2 + Ubuntu 24.04 + ROS 2 + Gazebo with GPU acceleration; MuJoCo runs natively on Windows. Native Ubuntu on the PC is unnecessary because all hardware connects to the Jetson.
- Remote desktop (Ubuntu screen sharing or NoMachine) is available for occasional GUI use; it needs an HDMI dummy plug when headless.

**Improvement loop (sim-first):** every change, parameter or code, must pass the simulation regression suite before it reaches the robot. Real sessions score catch rate, positioning error and return accuracy; changes are kept only if they beat the current best, with automatic rollback. Firmware, safety limits and test-area geometry are outside what Claude changes unattended.

## Cameras and perception

Two hardware-synchronized OV9782 color global-shutter cameras, mounted onboard as a wide-baseline stereo pair.

| Item | Decision | Rationale |
| --- | --- | --- |
| Placement | Onboard, on or beside the funnel rim, angled forward and up | Works anywhere (mobile platform goal); fixed external cameras would tie the robot to one spot |
| Configuration | Stereo, 15–25 cm baseline | A 7 cm ball at 3 m is ~20 px wide; mono size-based depth gives ~8 cm error, stereo ~1–2 cm, and earlier fits save time |
| Sensor | [OV9782](https://www.ovt.com/sensors/OV9782): 1280×800 color at 120 fps, global shutter, frame sync | Rolling shutter smears a fast ball; color simplifies detection |
| Exposure | < 1 ms | Ball moves ~5 cm per frame at 6 m/s and 120 fps; short exposure needs good lighting |

**Pipeline:** color-blob detection of a uniquely colored ball (CPU), stereo triangulation, then a ballistic filter with drag that outputs a landing point and its uncertainty. The robot commits to moving once the uncertainty is small enough. Each frame is corrected for the robot's own motion using odometry and IMU. A small neural-net detector comes later if lighting or backgrounds get messy.

**Alternatives considered:** Arducam OV9281 stereo kit (about $174, but mono, 90 fps on Jetson, and only guaranteed on official dev kits); Stereolabs ZED X (about $599 plus a GMSL2 capture card; 60 fps is low for catching, but excellent for visual SLAM later).

**Caution:** Jetson camera support depends on drivers for the exact JetPack version and carrier board. Confirm OV9782 driver support for the chosen carrier before buying; the official dev kit is the safest choice.

## Localization

Short-term motion tracking for the catch, lidar SLAM for long-term position, and three ways to find the thrower.

- **During a catch (about 1 s):** the ball is measured in the robot's camera frame, so only the robot's own motion over the flight matters. Wheel odometry and an IMU are fused on the microcontroller.
- **Slip handling:** omni wheels slip hardest during the acceleration bursts that matter most. A downward-facing PMW3901-class optical-flow sensor measures ground motion directly and is unaffected by wheel slip.
- **Long-term position:** RPLidar with slam_toolbox builds a map, keeps the robot in the play area, avoids obstacles and returns it to a home spot. This loop is standard ROS 2 and not latency-critical.
- **Finding the thrower:** (1) back-extrapolate the ball trajectory to its release point; (2) a person or pose detector on the Jetson at a modest rate, aiming return throws at chest height; (3) in a test setup, an AprilTag on a fixed target gives a precise, unambiguous aim point and a drift check.
- **Hardware added:** IMU on the microcontroller board (ICM-42688 or BNO085 class), optical-flow sensor, RPLidar (mounted low with a clear 360° view under any top module).

## Catching

A 30–35 cm funnel with a light rigid rim and a stretchy mesh cone, feeding straight into the launcher.

- **Geometry:** walls about 30–40° from vertical, narrowing to a throat slightly larger than the ball. The ball arrives at 5–6 m/s, tilted 20–30° from vertical, so walls must be steep enough to redirect it downward. Rim height 0.5–0.7 m (lower keeps the center of gravity down and adds flight time).
- **Capture and settling:** bounce-out is the main failure mode, so rigid plastic walls are avoided. A mesh or fabric cone (like a closed-bottom basketball net) absorbs energy, is light, and is soft if a child leans into it. The low-bounce ball choice helps.
- **Catch confirmation:** an IR break-beam at the throat gives an exact catch/miss score for logging and tuning.
- **Integrated reload:** the throat feeds by gravity directly into the launcher's loading chamber, so no transfer mechanism is needed.
- **Misses:** a person picks up the ball during play; a pickup mechanism (low roller or scoop) is a possible future upgrade.
- **Camera occlusion:** the funnel may block the view of the ball in the final moments; acceptable because the landing prediction is committed by then. Cameras mount on or beside the rim, angled up and forward.

## Throwing

A dual counter-rotating flywheel launcher at a fixed 50–55° launch angle, aimed left and right by rotating the whole base.

- **Why flywheels:** wheel speed sets ball speed directly (gentle lob vs firmer throw is just RPM), consistent shot to shot, fast cycling with no re-cocking, and a slight speed difference adds stabilizing backspin. Springs/catapults are slow to re-cock and drift with wear; pneumatics add a compressor, tank, noise and pressure hazards.
- **Motors:** small brushless outrunners on ESCs with RPM telemetry (e.g. bidirectional DShot). The microcontroller closes the speed loop and enforces a firmware RPM cap.
- **Aiming:** yaw comes free from the kiwi base spinning in place, so there is no turret. Pitch is fixed at 50–55° in v1, since a high, slow arc is easiest for people, especially kids, to catch; distance is set by wheel speed. A pitch actuator is a later option.
- **Feeding:** the ball drops from the funnel throat into a chamber held by a servo-driven gate. The gate opens only when the robot is aimed, both wheels are at speed and safety checks pass, so every shot is a deliberate, gated event.
- **Consistency:** the biggest factor is how the ball enters the wheels, so the chamber guides it the same way every time. A wheel-speed-to-distance calibration table is built and refined from logged throws.

## CAD workflow

SolidWorks is the master assembly; Claude generates most custom parts as STEP files from Python (build123d), placed in SolidWorks through wrapper parts with named reference geometry.

- **Why this split:** Fernando knows SolidWorks well, and its mates solver, interference detection and motion checks are better for mechanisms. Claude is most reliable generating parts in code (fast, verifiable, version-controlled) rather than building native parts feature by feature through the SolidWorks API.
- **Parts in code:** build123d (Python, OpenCascade kernel) with all key dimensions in one parameters file. Exports STEP for SolidWorks and STL/3MF for Bambu Studio. Lives in the same git repo as the robot code.
- **Wrapper parts:** each generated part sits inside a native SolidWorks part that contains the imported STEP body plus named reference planes and axes (e.g. "MotorShaftAxis", "MountFace"). References are defined from the wrapper's own origin and default planes, never from faces of the imported body. Higher-level assemblies mate only to these references.
- **Updates:** the STEP is imported with 3D Interconnect (linked to its source file), so a regenerated STEP updates in place and mates hold. To verify on the first wrapper part.
- **Coordinate-frame contract:** each part has an agreed origin and axes (e.g. "origin at center of motor mounting face, Z along the shaft"). The code always builds the part in that frame, so redesigned parts land in the right place. If a mating location truly moves, an API script updates the wrapper's reference offsets.
- **Mates:** Claude creates mates via the SolidWorks API when running on the Windows PC, using the named references for reliable selection.
- **Native parts:** Fernando models mechanism-heavy parts natively when that is faster; they mix freely with wrapped STEP parts.
- **Simulation link:** robot geometry and masses flow to MuJoCo via the SolidWorks-to-URDF exporter (or from the build123d parameters).

## Simulation

MuJoCo is the primary simulator and the main training environment; Gazebo is added later for full-system navigation testing.

- **Why MuJoCo:** accurate, fast contact physics (ball flight with drag, base acceleration and slip, funnel contact, launch), much faster than real time, runs natively on Windows and on the Jetson. Its GPU version (MJX) can run thousands of robots in parallel for large-scale training on the PC.
- **Why Gazebo later:** best for exercising the full ROS 2 stack (lidar, SLAM, Nav2, node wiring), not needed for the first catch work.
- **Modeling choices:**
  - Base: three wheels with direction-dependent friction plus a motor model from the AK40-10 torque/speed limits, rather than simulating every roller.
  - Launcher: modeled by its output (exit speed and angle with measured scatter), not flywheel-ball contact.
  - Perception: early on, synthetic detections with realistic noise, latency and dropped frames instead of rendered images. The detector itself is trained and tested on real recorded images.
- **First job, the spec sweep:** sweep acceleration, funnel size, perception latency, camera noise, throw distance and apex, and map catch rate across them to settle the provisional requirements.
- **Sim-to-real calibration:** once hardware exists, measure and feed back motor step responses, end-to-end latency and jitter, ball drag (filmed throws), funnel bounce (drop tests at varied speeds and angles) and launcher scatter (50–100 logged shots). Randomize each parameter within its measured range during training.
- **Key trust metric:** the gap between simulated and real catch rate for the same settings. A small gap means overnight simulation results can be trusted; a large gap focuses the next real session on finding the wrong part of the model.

## Real-world test area

Real sessions are short, attended calibration and validation runs, so the test area stays simple; unattended overnight training on hardware is not planned.

- **Area:** about 4×5 m with soft netting walls and foam bumpers along the bottom edge.
- **Throwing source:** a person throwing, or optionally a simple launcher (a copy of the robot's flywheel launcher on a pan-tilt base) for repeatable, logged pitches.
- **Referee camera:** a fixed overhead camera records every trial as ground truth for catches and onboard accuracy.
- **Logging per trial:** throw parameters (if launched), full ball track and landing prediction over time, robot path, break-beam result, return-throw settings and outcome, plus video. Stored as rosbag per session plus a summary table.
- **Metrics:** catch rate by distance from start, positioning error at ball arrival, prediction convergence time, return accuracy, cycle time, faults. The same metrics are computed in simulation for the sim-vs-real gap.
- **Session outputs:** calibration data for the simulation, and validation of simulation-tuned settings before they are adopted.
- **Power:** sessions are sized to one battery (7S, about 130 Wh, roughly 4–5 h with the Jetson at 15–25 W plus motor bursts). A charging dock is not needed for this plan.

## Safety

Safety is layered so that no high-level code change can make the robot exceed hard limits; kids are the design case.

**Hazards:** an 8 kg robot at 1.5 m/s carries about 9 J (enough to hurt a small foot or knock over a toddler), pinch points at omni wheels and flywheels, the launched ball, tipping, and the battery.

**Layers, most independent of software first:**

1. **Physical e-stop:** mushroom button on the robot cutting motor and payload power through a relay, independent of any code.
2. **Wireless e-stop:** ELRS receiver paired with the RadioMaster TX15, wired to the same power cutoff, failsafe set to stop on link loss.
3. **Firmware limits (microcontroller):** caps on speed, acceleration, motor current and flywheel RPM, plus the Jetson heartbeat watchdog. The Jetson can request less but never more; only Fernando flashes this firmware. AK40-10 current limits are set as a backstop; their comm-loss timeout is to be confirmed in the manual.
4. **Bumpers and guards:** soft foam skirt around the base with contact switches wired to the microcontroller; guards over wheels and the flywheel intake.
5. **Lidar people-awareness (Jetson software):** slow down when anyone is within about 1.5 m and keep target positions away from people. Useful but not trusted alone.

**Modes:**

- **Kid mode:** hard-capped speed (around 0.8 m/s) and acceleration, gentle lob throws only, launcher aimed at chest height or below.
- **Adult mode:** full performance envelope.
- **Rule:** settings trained in simulation must pass a supervised adult-only session before they are allowed in kid mode.

**Audible cues:** the speaker chirps before the robot starts moving and gives a distinct warning tone before the launcher fires.

**Battery:** Li-ion pack with BMS, main fuse, low-voltage cutoff; charge only while supervised, ideally in a fire-resistant bag or box.

## Future features and modular payload interface

The base is a general mobile platform with a standard payload interface on top, so the catch module can be swapped for a robot arm or removed entirely.

**Payload interface (the first CAD coordinate-frame contract):**

- **Mechanical:** hole grid (e.g. 25 mm spacing), two locating pins for repeatable placement (so camera and launcher calibration survive removal), and four quick-release fasteners (captive thumbscrews or cam latches).
- **Electrical:** one connector with e-stop-switched 24 V power, CAN, and USB or Ethernet to the Jetson.
- **Identity:** each module announces itself over CAN; software loads the matching behavior, safety limits and mass model (catch mode, arm mode, or bare base).
- **Payload budget:** size the base for about 3–4 kg of payload; firmware lowers acceleration limits automatically based on the attached module.

**What lives where:** the base carries everything every personality needs (lidar, IMU, optical flow, speaker, RC receiver, Jetson, battery, microcontroller). Mission-specific parts ride on the module: stereo cameras and launcher on the catch module; the arm module brings its own wrist or gripper camera. The lidar sits low with a clear 360° view under any module's overhang.

**Planned features:**

- **Speaker:** USB or I2S amp and speaker on the Jetson for personality (sounds, voice, celebrating catches) and safety cues.
- **RC driving:** the ELRS receiver used for the wireless e-stop also carries stick inputs to the microcontroller over a serial link. A TX15 switch selects manual or autonomous. In manual mode the microcontroller drives the base directly under the same firmware limits, even with the Jetson off; link loss means stop.
- **Robot arm module:** a second personality on the same payload interface, with its power cut by the e-stop chain.

## Rough bill of materials

The core robot plus catch module comes to roughly $1,700–3,000, with the Jetson tier driving most of the spread. Only the AK40-10 and Jetson prices are verified; the rest are typical street prices to firm up when parts are chosen.

| Item | Est. cost (USD) |
| --- | --- |
| Jetson: Orin Nano Super dev kit ($399) or Orin NX 16GB (about $999 module plus $150–250 carrier) | 400–1,200 |
| 3× CubeMars AK40-10 V3.0 KV170 ($135.90 each) | 410 |
| 3× double-row omni wheels, 100–125 mm | 100–180 |
| Microcontroller board with CAN-FD, IMU, optical-flow sensor | 70–120 |
| RPLidar (C1 / A-series class) | 70–100 |
| 2× OV9782 global-shutter color cameras + cables | 150–250 |
| 7S Li-ion pack with BMS + charger | 150–200 |
| Power: fuses, e-stop relay, DC-DC converters, wiring | 80–120 |
| ELRS receiver (TX15 already owned), speaker + amp | 50 |
| Launcher: 2 brushless motors + telemetry ESCs, gate servo | 100–150 |
| Funnel mesh, aluminum plate, fasteners, filament | 100–200 |
| **Total** | **1,700–3,000** |

Sources: [CubeMars store](https://store.cubemars.com/), [Jetson pricing, July 2026](https://www.cnx-software.com/2026/07/22/nvidia-increases-the-price-of-jetson-modules-and-devkits-by-up-to-101/?amp=1).

## Phased roadmap

Seven phases, ordered so the riskiest piece (perception) is proven before the robot is built; each phase ends with an exit check.

1. **Simulation and design (buy only the Jetson).** Build the MuJoCo model, run the spec sweep, set up the dev environment (Jetson + WSL2 + Foxglove + Claude Code), and lock CAD coordinate-frame contracts including the payload interface.
   - Exit: provisional requirements confirmed or revised from sweep results.
2. **Perception bench.** Cameras and lidar on the Jetson, no robot. Track hand-tossed balls and measure landing-prediction accuracy and how early it converges.
   - Exit: prediction within about ±5 cm early enough to leave ~0.7 s of motion time.
3. **Base and safety.** Motors, microcontroller firmware with limits and watchdog, physical and wireless e-stop, bumpers, RC driving. Measure base dynamics and feed them back into the simulation.
   - Exit: all safety layers tested; RC driving works with the Jetson off.
4. **Catch module.** Funnel and cameras on the robot; stop-and-catch with adult tosses; funnel drop tests to calibrate contact.
   - Exit: real catch rate tracked against simulation.
5. **Launcher and return throws.** Calibrate throw distance and scatter; validate kid mode.
   - Exit: consistent, catchable return throws; kid mode signed off in an adult-only session.
6. **Improvement loop.** Simulation-driven tuning with real validation sessions; lidar SLAM/Nav2; speaker personality.
   - Exit: small, stable sim-vs-real gap.
7. **Arm module.** Second personality on the same payload interface.

## Open questions and items to verify

- [ ] Jetson tier: Orin Nano Super dev kit vs Orin NX 16GB, and which carrier board.
- [ ] OV9782 driver support for the chosen JetPack version and carrier.
- [ ] Flashing path for the chosen Jetson (SD card image vs SDK Manager on an x86 Ubuntu host).
- [ ] AK40-10 thermal behavior under repeated 1.6 N·m acceleration bursts.
- [ ] AK40-10 comm-loss timeout and current-limit settings (manual).
- [ ] Exact ball model (70–75 mm, low bounce, bright color).
- [ ] Funnel angle and mesh design from simulation plus drop tests.
- [ ] Flywheel diameter, motor and ESC sizing for the throw speed range.
- [ ] Payload interface spec: hole pattern, connector, power budget.
- [ ] Confirm SolidWorks 3D Interconnect STEP update keeps wrapper references and mates on a first test part.
- [ ] ROS 2 and Ubuntu version alignment between the Jetson and the PC Docker image.
