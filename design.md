# Play-Catch Robot — Design Baseline

Oct 17, 2026 · Fernando (with Claude) · supersedes the Oct 16 design doc and the Oct 17 component study (both in [archive/](archive/))

## Start here (for a new session)

This is the single source of truth for the design. Each section states the **current baseline**, its **expected performance** (from the v2 MuJoCo simulation unless noted) and **why** it was chosen. Conventions used throughout:

- **(est)**: an engineering estimate waiting for bench data.
- **VERIFY**: a dimension or spec to confirm on the real part (searchable in `cad/` and the sim catalog).
- **Proposed**: not yet agreed with Fernando.

**Where things stand:** Phase 1 (simulation and design) is well under way.

- Three MuJoCo sims run: v1 is the hello world, v2 is the detailed physics model (the baseline), v3 is a **proposed compact redesign**, summarized in section 20. Its defaults are round 2 (v3b): a 4-wheel X base with the deck in the axle plane, and a tilting catch/launch module with a plunger feed. CG 95 mm, rim 387 mm.
- **v5 unlimited** (section 21) is the v3b robot with every firmware limit removed, a per-pixel vision model and a multi-ball hopper mode. It is an exploration of the envelope, not a proposal to drop safety limits. Headline: 81% → 89% caught (92% with a fast camera pipeline); the funnel only works as a hopper with a single-file magazine tube. Section 21.4 estimates the real cameras: 86–92% expected, 72% pessimistic. A thrower farther than ~4 m catches less, not more. Swerve is the base upgrade path, worth +2–4 points.
- **v6 shallow throw** (section 22) targets throwers 6-7 m away with the expected cameras and the X4 base. Firmware adult mode catches 96% of shallow throws (v5: 57%). Three changes:
  - a **roller feed**: two worm-geared feed wheels hold the ball and pass it to the flywheels, one per trigger, with zero misfires;
  - a padded pocket and catch tilt up to 35°;
  - an honest stereo track start (perception software).
- v6 catches 99–100% at 1.5–6 m with human-like scatter, and returns land a median 3 cm from the thrower at 6.5 m (sim upper bound). Multi-ball: the magazine tube works. A tennis-style indexer wheel feeds but catches only 20%, so a catch path that bypasses it is the open design question.
- Indoors under an 8 ft ceiling, a ceiling-aware return (~35–40°, ~4,500 rpm) still reaches a thrower 7 m away, with 0 ceiling hits and a median miss of 7 cm (section 22.5). The round-1 feed pocket only fed balls within about ±0.5 mm of 74 mm. Round 2's **chute pocket** (floor sloping into the feed-wheel nip, J-bend into the flywheels) feeds every 72–76 mm ball at every tilt, with the same catch rate and zero misfires (section 22.4).
- **v6 round 3** (section 23) replaces guesses with models and ranges:
  - feed wheels on direct-drive gimbal motors that hold the ball with current (no worm): the integrated **LK-TECH MF4005** or a GM3506;
  - viscoelastic contacts;
  - randomized worlds: **86% caught across 192 worlds** (per-world median 89%, 10–90% 72–97%), with stereo calibration the dominant unknown;
  - floors: an omni wheel can't climb a square rug edge sideways, and a missed ball under a wheel or a threshold hop can flip the robot (fixed by a skirt and slow zones);
  - a motor study: keep four wheels, but the AK40-10 isn't needed; the DJI M2006 catches the same for less.
- **Actuator choice, cost first** (section 24), on a new shared **actuator database** (`database/`: CSV + `actuator_db.py`, every part with sources and provenance):
  - cheapest compliant motor + controller set **~$714 on a 6S pack**: M2006 + C610 drive, brushed Pololu 25D feed on DRV8874s, STS3250 tilt, F90 flywheels on an 8S AM32 4-in-1;
  - preferred integrated set ~$937 on 7S (AK40-10 drive, 25D + RoboClaw feed);
  - brushed gearmotors are out for the drive: they overheat in continuous play;
  - the M2006 catches the same on 6S as on 7S, so the unverified C610-on-7S question goes away. It is out of stock at DJI: buy and bench one pair first.
- A full printable CAD design exists for v2, from build123d to STEP and STL, with 0 clashes. No CAD for v3 yet.
- A component trade study confirmed most choices and changed a few.
- No hardware has been bought or built. Perception is ground truth in v2; v3 onward model it (v5-v6: per-pixel vision).
- **Safety finding that applies to v2 today:** a zero-speed command on every wheel at 1.4 m/s (a naive fault stop) rocks the v2 kiwi 23° on a μ 0.95 floor and tips it over at μ 1.2. Stops must be ramped (sections 3 and 14).

**Repo map**

```
omni-robot/
├── design.md                       ← this file
├── archive/                        ← superseded docs (Oct 16 design doc, Oct 17 component study)
├── simulation/
│   ├── README.md                   ← Python venv setup, version index
│   ├── requirements.txt
│   ├── v1 hello world/             ← simple kiwi + catch loop (velocity-servo motors, launcher modeled by its output)
│   ├── v2 detailed physics/        ← CURRENT SIM
│   │   ├── catalog.py              ← datasheet values for every candidate part ((est) marks estimates)
│   │   ├── params.py               ← fitted parts + geometry + masses + limits (single source for sim AND cad)
│   │   ├── models.py               ← drive actuator, flywheel motor + ESC, gate servo, battery
│   │   ├── robot_model.py          ← MJCF builder (scene, base, omni wheels, funnel, gate, J-bend, flywheels)
│   │   ├── robot.py                ← firmware-facing robot: sensors in, commands out, electrical bookkeeping
│   │   ├── catch_game.py           ← toss → catch → aim → launch loop
│   │   ├── run.py                  ← viewer / headless / video
│   │   ├── launcher_bench.py, launcher_cal.json   ← flywheel rpm → exit speed calibration
│   │   ├── test_v2.py              ← 7 checks (~5 s)
│   │   ├── studies/                ← trade studies; results/ holds JSON, plots, logs
│   │   └── catch_demo.mp4, drive_demo.mp4
│   └── v3 compact/                 ← PROPOSED compact robot (section 20): same files as v2, plus
│       ├── traction.py             ← quasi-static N-wheel traction / tip model (also used for load-aware allocation)
│       ├── omni_control.py         ← N-wheel kinematics, jerk-limited base controller
│       ├── render.py, renders/     ← module poses
│       ├── test_v3.py              ← 13 checks (~10 s)
│       └── studies/                ← wheel_count.py, funnel_tilt.py, camera_view.py
│   └── v5 unlimited/               ← v3b with no firmware limits, vision.py, hopper (section 21; there is no v4)
│       ├── vision.py               ← per-pixel blob detector, stereo + known-size ranging, image-space EKF
│       ├── shoot_game.py           ← hopper mode: shoot at targets standing or on the move
│       ├── test_v5.py              ← 25 checks (~20 s)
│       └── studies/                ← envelope, vision_sweep, catch_sweep, hopper_rig, launcher_max, shoot_sweep,
│                                     camera_estimate, thrower_distance, base_types
│   └── v6 shallow throw/           ← 6-7 m shallow throws, chute feed, padded pocket, indexer (section 22) + round 3 (section 23)
│       ├── contact_model.py        ← round 3: Hunt-Crossley ball contacts, viscoelastic tire slides
│       ├── floor.py                ← round 3: rolling resistance, friction field/patches, thresholds, carpet
│       ├── randomize.py            ← round 3: the 17 guessed parameters, their ranges, randomized worlds
│       ├── test_v6.py              ← 43 checks (~45 s)
│       └── studies/                ← shallow_sweep, catch_rig, return_throws, hopper_rig, shoot_sweep, thrower_distance,
│                                     feed_tolerance, indoor; round 3: feed_motor, randomized, floor_wheels, motor_study;
│                                     actuator_choice (section 24: reads database/, ranks by cost, writes results/actuator_choice.md)
├── database/                       ← shared actuator database (section 24; moves up to Robotics/database/ later)
│   ├── motors.csv, servos.csv, controllers.csv, sources.csv   ← the data (edit in Excel); gaps.csv is generated
│   ├── actuator_db.py              ← schema, validator, DC model, requirement checks, costs, sim export, gaps
│   └── README.md, test_actuator_db.py
└── cad/
    ├── README.md                   ← print list, fasteners, assembly notes
    ├── dims.py                     ← CAD dimensions + coordinate-frame contracts (imports the sim's params)
    ├── base.py, catch.py           ← printed parts (drive base / catch module)
    ├── vendor.py, vendor/          ← purchased parts (official CubeMars AK40-10 STEP + drawing, envelopes)
    ├── build.py, render.py         ← export STEP/STL/assembly + clash check; MuJoCo renders + turntable
    └── out/                        ← step/, stl/, assembly.step, PRINT_LIST.md, renders/, funnel_net_pattern.svg
```

Outside the repo:

- **Python venv:** `C:\Users\ferna\.venvs\omni-robot-sim`. It holds MuJoCo 3.16 and build123d 0.13. The only base Python on this PC is PlatformIO's, so never install packages into PlatformIO's penv.
- **Sister project:** `C:\Dropbox\Personal Projects\Robotics\wheeled-leg-robot\simulation\mujoco\v4_twin_279mm_baseline`, a mature sim with motor, battery and thermal models, optimizers and a digital-twin link. Reuse its conventions.

**Common commands** (run from the folder shown; `$py = "$env:USERPROFILE\.venvs\omni-robot-sim\Scripts\python.exe"`):

| Folder | Command | What it does |
| --- | --- | --- |
| `simulation/v6 shallow throw` | `& $py watch_floor.py` (`--list`; `cross --obstacle "rug edge 8 mm" --dir 45`; `game`; `flip`) | Watch the robot on thresholds, rug edges and the home floor (section 23.4) |
| `simulation/v2 detailed physics` | `& $py run.py` | Catch demo in the MuJoCo viewer (`--drive` for the omni demo, `--wheel-model rollers` for explicit rollers) |
| same | `& $py run.py --headless -n 30` | Catch statistics without graphics |
| same | `& $py -m pytest -q` | Regression checks |
| same | `& $py launcher_bench.py` | Recalibrate the launcher table |
| same/studies | `& $py wheel_slip.py` (and the others) | Re-run a trade study |
| `cad` | `& $py build.py; & $py render.py --turntable` | Rebuild all CAD outputs and renders |

**Suggested next steps (in order):**

0. Decide on the v3 compact proposals (section 20). Whichever base is kept, make firmware stops ramped, never a step to zero speed.
1. Perception: a first stereo noise/latency/dropout model and tracking filter now exist in the v3 sim (section 20, round 3), and they show perception timing is the biggest lever on catch rate. Next: run the spec sweep over latency, noise and frame rate (section 12), then calibrate the model on the bench (Phase 2).
2. Make the base acceleration limit direction-aware and jerk-limited (section 3). Done in the v3 sim (section 20, item 8): port it to v2 if v3 isn't adopted.
3. Bench work: order parts and print a test wheel pod with adapter and a flywheel hub with tire. Measure roller μ on the real floor, the foam ball's properties, and the launcher calibration. Resolve the VERIFY items.
4. Bring the CAD into SolidWorks as wrapper parts; add wiring, bumper switches, speaker, the ELRS receiver and the MCU board once chosen.
5. From v5 (section 21): set the camera spec by what the sweep says matters (latency, frame rate, calibration, global shutter, hardware sync), film throws to measure ball spin. (v5's open item, a retaining gate at the pocket exit, is answered by v6's roller feed.)
6. From v6 (section 22):
   - decide on the roller feed, the padded pocket and the 35° catch tilt;
   - port the honest track start into the perception code;
   - bench-test a feed-wheel pair on GM3506 gimbal motors (round 3, section 23.1): speed, current-limited holding, a rigid preloaded stop;
   - measure the net's rolling resistance and the ball's mass and drag;
   - choose a multi-ball path: the tube insert now, or an indexer whose catch path bypasses the wheel;
   - decide on the chute pocket (section 22.4) and bench-test a printed chute + J with two feed gearmotors and the exit beam;
   - for indoor play, use the ceiling-aware return planner and spin-test the flywheels to ≥ 5,000 rpm (section 22.5).
7. From v6 round 3 (section 23):
   - bench the stereo pair's calibration stability first: it is the biggest unknown in the randomized study;
   - drop-test the foam ball on the floor and the pad at three heights; film throws for spin;
   - rework the battery into two rows;
   - design the bumper skirt (down to ~20 mm) and the wheel-top guards; map thresholds as ≤ 1 m/s zones; avoid square rug edges;
   - in the tracking filter, model calibration error as a slowly varying bias.
8. From the actuator choice (section 24):
   - decide between the cheapest set (6S, M2006) and the integrated set (7S, AK40-10);
   - for the cheapest set, buy one M2006 + C610 pair (supply: out of stock at DJI) and bench it on 6S at ~0.4 N m RMS for winding temperature and backlash;
   - look at a counterbalance spring on the tilt pivot (it would let a $26-49 STS3215 do the tilt);
   - add torque sharing to the X4's four speed loops (it cuts drive heating for any motor);
   - fill the database gaps (`database/gaps.csv`), prices first.

![design overview](cad/out/renders/overview.png)

## 1. Overview and goals

A kiwi omni-wheel robot tracks a tossed ball, drives under it, catches it in a funnel and throws it back with a flywheel launcher. Most tuning and training happens in a realistic MuJoCo simulation; the real robot is used for attended calibration and validation sessions.

**Goals**

- Fun to play catch with, including with kids (a dedicated kid mode).
- Technically impressive: stop-and-catch first, moving catches later.
- Improvable by Claude between sessions, mainly through overnight simulation runs.
- A general mobile platform: lidar SLAM, RC driving, a speaker, and swappable top modules (catch module now, robot arm later).
- Built mostly from 3D-printed parts (PLA, PETG, TPU on a ~256 mm Bambu bed) plus purchased actuators and electronics.

## 2. Requirements and expected performance

v1 targets stop-and-catch of gentle underhand throws. Requirements are still provisional; the right-hand columns show what the current design achieves in simulation.

| Parameter | Requirement | Expected (sim v2) | Notes |
| --- | --- | --- | --- |
| Throw distance (to robot) | 2–4 m underhand | Thrower at 3.0 m | |
| Apex above release | 1–2 m | 1.0–2.0 m tested | Gives 0.7–1.0 s after the 0.25 s perception delay |
| Detect + fit trajectory | by ~0.25 s after release | 0.25 s delay, 20 Hz updates (ground truth) | **Perception noise not modeled yet** |
| End-to-end latency | < 100 ms | not modeled | |
| Base acceleration | ~~4–5 m/s²~~ → **2.5 m/s² limit (proposed)** | Slip-free: ~2.4 m/s² worst direction, up to 4–7 toward a wheel (μ 0.95) | Traction-limited; see section 3 |
| Peak speed | ~1.4 m/s | 2.6 m/s full battery, 2.0 m/s at 10% charge | Firmware cap 1.4 m/s |
| Landing prediction error | ±5 cm | n/a (ground truth) | |
| Catch zone | ~1 m diameter | 100% within 0.65 m radius (30/30), with perfect perception | The funnel tolerates ~±12 cm of positioning error |
| Positioning at ball arrival | — | median 0.4 cm, max 1.9 cm funnel offset (20 tosses) | |
| Return throw | to the thrower | median 14 cm from the thrower's hands (20 throws) | 52° lob to chest height |
| Cycle time | — | ~4.5 s per toss in sim (toss → catch → aim → launch → reset) | |
| Ball | ~~70–75 mm hollow plastic~~ → **74 mm foam pickleball, ~22 g (proposed)** | Drag ≈ 25% of weight at 6 m/s | Drag must be in every predictor |
| Run time | one session per battery | ~3.9 h at 28 W average | |

Key insights:

1. **Stop-and-catch is easy in sim.** Requiring a full stop under the ball halves reach versus a moving catch, but in simulation it caught everything inside a 0.65 m radius. Even acceleration limits from 2.0 to 3.5 m/s² all scored 30/30; only 1.5 m/s² lost catches (26/30). Perception accuracy, not drive performance, will drive the real catch rate.
2. **The real error budget is about ±11 cm** (funnel radius minus ball radius).

## 3. Drive base

A three-wheel kiwi base on CubeMars AK40-10 actuators and 96 mm goBILDA omni wheels, running from a 7S Li-ion pack.

| Item | Baseline | Why |
| --- | --- | --- |
| Layout | Kiwi: 3 wheels at 60° / 180° / 300° (0° = forward = launcher side), 180 mm from center | Equal agility in every direction, always 3 wheels on the ground |
| Wheels | **goBILDA 96 mm omni, 14 mm bore, 50A rollers, part 3624-0014-0096** ($21.99, 119 g) | Lightest and grippiest of the candidates; lowers deck and CG. 125–127 mm wheels add ~1 kg and raise the CG for no catch benefit |
| Motors | 3 × **CubeMars AK40-10 V3.0 KV170** with driver ($135.90, 190 g): Kt 0.056 N·m/A, 10:1, 978 mΩ, 7.3 A peak (4.1 N·m), 435 rpm no-load at 24 V, CAN | Lightest actuator that meets speed; torque is never the limit |
| Wheel mount | Wheel overhung on the motor output via a printed adapter (3 × M2.5 to the output on PCD 27, 3 × M4 through the wheel's 32 mm pattern); motor bolts to a printed pod through its rear 4 × M2.5 on PCD 47 | Simple; VERIFY the AK40-10 output bearing's radial and moment rating |
| Battery | 7S1P Molicel P50B 21700 (5 Ah, ~126 Wh), BMS ≥ 40 A | See section 10 |
| Mass | **5.56 kg** total (sim with CAD masses), CG 151 mm high and ~21 mm forward (launcher, lidar) | Lighter than the old 6–9 kg planning range |

**Expected performance**

| Metric | Value | Source |
| --- | --- | --- |
| Top speed | 2.63 m/s (29 V), 2.0 m/s (22 V, 10% charge) | motor + battery model |
| Slip-free acceleration (μ 0.95) | ~2.4 m/s² worst in sim, rear diagonals 120° / 240° (quasi-static model 2.8); ~3.4 forward, 3.1–3.4 sideways, 4.7–7.4 toward a wheel | `studies/wheel_slip.py` |
| Same vs roller friction | worst direction 1.6 / 2.0 / 2.4 / 3.1 m/s² at μ 0.6 / 0.75 / 0.95 / 1.1 | same |
| Tip-over | ~5.9 m/s² toward a wheel-triangle edge (quasi-static) | model |
| 0.37 m stop-and-catch move | ~0.97 s to settle within 1 cm; 0.6 m in ~1.2 s | `studies/drive_components.py`, `drive_reach.py` |
| Motor heating | winding ≈ +2–3 °C over ambient in continuous catch duty (thermal constants est) | thermal model |
| Ground clearance | 22 mm (motor bottom) | geometry |

**Traction, not torque, limits acceleration.**

- Accelerating between two wheels is pushed by those two wheels alone, each needing 0.58·m·a, while load transfer unloads them.
- Quasi-static limit per wheel: a_max = μ·N₀·g / ((2/3)|t·u| + μ(2/3)(h/L)(u·r)); take the minimum over the three wheels. N₀ is the static load share (it includes the CG offset), t the wheel's rolling direction, r its radial direction, and u the acceleration direction.
- The sim matches this model in shape, and runs ~0.5 m/s² below it in the worst directions. The instant acceleration step rocks the robot, so jerk limiting should recover most of that.
- The explicit-roller wheel model agrees with the fast sphere model within ±25%. It also adds the real double-row 2–3% speed ripple and about ±2° of heading wander.
- **Proposed firmware behavior:**
  - cap at 2.5 m/s²,
  - make the limit direction-aware with the formula above (30–50% more acceleration in good directions),
  - add jerk limiting.

**Tipping.** The support polygon is the wheel triangle, so the tipping lever is its inradius: half the wheel-circle radius (90 mm here), not half the base width. The Oct 16 doc's "45 cm wide, CG at 40 cm, tips near 5.5 m/s²" example actually tips at about 2.2–2.5 m/s². Keep the battery and motors low (they are).

- Accelerating toward a wheel, or braking while driving forward (launcher first), the kiwi is right at its tip limit: the two loaded wheels do the pushing and gain grip as the third one unloads, so traction doesn't cap the acceleration before tipping does.
- **Dynamic check (v3 sim, `studies/wheel_count.py`, kiwi at v2's 180 mm):** cruising at 1.4 m/s, then every wheel commanded to zero speed at once, the base rocks 23° on a μ 0.95 floor and **tips over at μ 1.2**. At the 2.5 m/s² ramped limit nothing tips.
- So: stops are always ramped within the acceleration limit, including watchdog and fault stops; the e-stop cuts power so the wheels coast; the AK40-10's own comm-loss behavior must coast or ramp, not lock (VERIFY in its manual).

**Rejected drive options** (sim + datasheets, `studies/drive_components.py`):

| Option | Why rejected |
| --- | --- |
| AK45-10, DM-J4310-2EC | Too slow at 24 V: top speed 1.1–1.6 m/s with low-battery speed under 1.2 m/s |
| AK60-6 V3 | 2.6× the cost for torque the floor can't use; needs ≥127 mm wheels for clearance |
| SteadyWin GIM6010-8 | Budget option at ~$105, but 200 g heavier per motor and needs ≥100 mm wheels |
| RobStride 01/02, Damiao DM-J6006 | Need 36–48 V, or too slow at 24 V (from the Oct 16 doc) |
| Nexus 100 / 127 mm aluminum omnis | 3–4× heavier, lower μ (est) |

## 4. Compute and real-time control

A Jetson Orin runs perception, planning, SLAM and logging; a separate microcontroller runs motor control and is the safety authority. (Unchanged from Oct 16.)

**Main computer: Jetson Orin.** Start with the Orin Nano Super dev kit ($399) or an Orin NX 16GB module (~$999 plus a ~$150–250 carrier). Both share the same module footprint, so a carrier board can take the Nano now and the NX later. Prices reflect NVIDIA's July 2026 increase ([CNX Software](https://www.cnx-software.com/2026/07/22/nvidia-increases-the-price-of-jetson-modules-and-devkits-by-up-to-101/?amp=1)).

- Why Jetson over a Raspberry Pi 5: more MIPI camera lanes, CUDA for neural-net detection and visual SLAM (Isaac ROS), and headroom for an arm and more cameras later.
- Why not AGX Orin or AGX Thor: overkill, and now $3,499 / $5,499 dev kits. The cheaper Thor T3000/T2000 modules expected Q1 2027 ([ServeTheHome](https://www.servethehome.com/nvidia-announces-expanded-jetson-thor-lineup-with-mid-range-t3000-and-t2000-modules/)) are a possible future upgrade.
- Considered: x86 mini PC (cameras only over USB, no CUDA), RK3588 boards (weak robotics software support), VOXL 2 (drone-oriented, expensive).
- Runs full Ubuntu (JetPack) with a desktop; switch to headless boot when deployed to free ~1 GB of RAM.
- Placement: in the 65 mm layer between the deck and the payload plate, rear half (VERIFY the carrier hole pattern).

**Real-time microcontroller: STM32G4/H7 class with native CAN-FD** (Teensy 4.1 acceptable).

- Runs the 500 Hz–1 kHz base control loop and talks to the three AK40-10s over CAN.
- Enforces velocity, acceleration, current and flywheel-RPM limits; reads bumpers, e-stop and the RC receiver directly; stops the motors if the Jetson heartbeat stops.
- Closes the flywheel rpm loop from bidirectional-DShot eRPM telemetry and drives the gate servo.
- The Jetson only requests desired velocities, so a bad high-level change cannot exceed firmware limits.
- Placement: under the center plate beside the battery, with the power board (relay, DC-DC converters) opposite.

## 5. Software architecture and development workflow

ROS 2 for the overall system, with one low-latency catch path, a simple custom link to the microcontroller, and development done remotely on the Jetson from a Windows PC. (Unchanged from Oct 16.)

**Architecture**

- ROS 2 provides the RPLidar driver, slam_toolbox, Nav2, MoveIt (future arm), rosbag logging and Foxglove visualization, so none of that is rebuilt.
- **Fast catch path:** camera capture, ball detection, stereo triangulation, trajectory fitting with drag, and landing prediction run in one process (composable nodes or one C++ node), passing data in memory. It publishes only the target "go to (x, y) by time t" plus diagnostics.
- Camera frames carry hardware timestamps, and Jetson and microcontroller clocks are synchronized, so fits use real capture times.
- **Microcontroller link:** a small custom protocol over CAN or UART (velocity commands, heartbeat, status), not micro-ROS, to keep the safety MCU simple.
- Robot software runs in Docker so the PC simulation and the Jetson run identical code and ROS 2 versions.
- The sim already separates "firmware-facing" robot code (`robot.py`: sensors in, wheel/flywheel/gate commands out) from the world. Keep that boundary when the code moves to ROS.

**Development workflow**

- Windows PC is the front end: VS Code Remote-SSH into the Jetson for editing, building (colcon, natively on ARM) and running.
- Claude Code runs on the Jetson, so it can edit, build, read logs, run tests and restart nodes directly on the hardware.
- The MCU programmer (ST-Link or USB DFU) plugs into the Jetson, so flashing, serial console and GDB all work remotely. Flashing is allowed only in attended sessions and is blocked in unattended configurations; hard safety limits live in a separate firmware module changed only deliberately, together.
- One git repo on the Jetson, pushed to GitHub. Tunable parameters are kept separate from algorithm code. Today they live in `simulation/v2 detailed physics/params.py` and `catalog.py`, to become YAML.
- **Visualization:** Foxglove on Windows connects to foxglove_bridge over WebSocket, so no ROS networking or Linux is needed on the PC.
- **Simulation on the PC:** MuJoCo runs natively on Windows in the venv above. WSL2 + Ubuntu 24.04 + ROS 2 + Gazebo come later.
- Remote desktop (Ubuntu screen sharing or NoMachine) is available for occasional GUI use; it needs an HDMI dummy plug when headless.

**Improvement loop (sim-first):** every change, parameter or code, must pass the simulation regression suite before it reaches the robot (today: `test_v2.py` plus headless catch statistics). Real sessions score catch rate, positioning error and return accuracy; changes are kept only if they beat the current best, with automatic rollback. Firmware, safety limits and test-area geometry are outside what Claude changes unattended.

## 6. Cameras and perception

Two hardware-synchronized OV9782 color global-shutter cameras, on the funnel rim, looking forward and tilted 30° up.

| Item | Baseline | Why |
| --- | --- | --- |
| Placement | Printed pads on the front rim arcs at ±33.7°, 600 mm up | Works anywhere (mobile platform goal) |
| Configuration | Stereo, **190 mm baseline**, parallel optical axes | Stereo gives ~1–2 cm depth vs ~8 cm for mono size-based depth |
| Sensor | [OV9782](https://www.ovt.com/sensors/OV9782): 1280×800 color at 120 fps, global shutter, frame sync | Rolling shutter smears a fast ball |
| Exposure | < 1 ms | Ball moves ~5 cm per frame at 6 m/s and 120 fps |

**Pipeline:**

1. Color-blob detection of the brightly colored foam ball (CPU).
2. Stereo triangulation.
3. A ballistic filter **with quadratic drag**, whose output is a landing point and its uncertainty, computed where the ball crosses the funnel rim height (0.60 m), not the floor.

The 22 g foam ball has drag ≈ 25% of its weight at 6 m/s, so a vacuum-ballistic fit misses by tens of centimeters. The sim's predictor (`ball_flight.py`, MuJoCo's drag law, Cd 0.5 (est)) matches the simulated ball within 1 cm. The robot commits once uncertainty is small; each frame is corrected for robot motion using odometry and IMU. A small neural-net detector comes later if lighting or backgrounds get messy.

**Alternatives considered:** Arducam OV9281 stereo kit (about $174, but mono, 90 fps on Jetson, and only guaranteed on official dev kits); Stereolabs ZED X (about $599 plus a GMSL2 capture card; 60 fps is low for catching, but excellent for visual SLAM later).

**Caution:** Jetson camera support depends on drivers for the exact JetPack version and carrier board. Confirm OV9782 driver support for the chosen carrier before buying; the official dev kit is the safest choice.

## 7. Localization

Short-term motion tracking for the catch, lidar SLAM for long-term position, and three ways to find the thrower. (Unchanged from Oct 16.)

- **During a catch (about 1 s):** the ball is measured in the robot's camera frame, so only the robot's own motion over the flight matters. Wheel odometry and an IMU are fused on the microcontroller.
- **Slip handling:** omni wheels slip hardest during the acceleration bursts that matter most. A downward-facing PMW3901-class optical-flow sensor measures ground motion directly and is unaffected by wheel slip. The sim shows slip starts around 2.4 m/s² in the worst direction.
- **Long-term position:** RPLidar C1 with slam_toolbox builds a map, keeps the robot in the play area, avoids obstacles and returns it to a home spot. The lidar sits at the front of the deck under the payload plate (scan plane ~150 mm). Its only fixed obstructions are the three thin payload standoffs at 0° / 120° / 240°, which software masks out.
- **Finding the thrower:** (1) back-extrapolate the ball trajectory to its release point; (2) a person or pose detector on the Jetson at a modest rate, aiming return throws at chest height; (3) in a test setup, an AprilTag on a fixed target gives a precise, unambiguous aim point and a drift check.
- **Hardware added:** IMU on the microcontroller board (ICM-42688 or BNO085 class), optical-flow sensor, RPLidar C1.

## 8. Catching

A 33 cm fabric-net funnel feeds a padded catch pocket closed by a servo trapdoor; an IR break-beam confirms the catch.

| Element | Baseline | Why / evidence |
| --- | --- | --- |
| Funnel | Ø330 rim at 600 mm, walls 35° from vertical, Ø84 throat at 424 mm. Fabric net laced to a printed collar and a 4-piece printed rim, held by 6 × Ø8 carbon struts | Steep walls redirect a ball arriving at 5–6.5 m/s, 20–30° off vertical |
| Net material | Grippy, soft mesh (sim: μ 1.0, high rolling resistance (est)) | A slippery cone lets off-center balls swirl and climb out. With a grippy, lossy net, 6/6 test arrivals were held (6.5 m/s, 20° tilt, up to 0.11 m off-center) |
| Catch pocket | Vertical-walled square tube (78 mm inside, 2 mm clearance) one ball diameter (85 mm) below the throat | With the gate at the throat, balls skated across and climbed back out of the cone |
| Gate | **Trapdoor** hinged on a Ø3 pin at the side wall, driven directly by a 25 kg·cm servo on the hinge axis; opens 95° into a wall window so it lies flush | A sliding shutter needs ~250 mm of side travel and drags the ball sideways. The padded impact (~0.6 N·m) is within servo holding torque |
| Gate pad | 3 mm EVA foam (restitution 0.15 (est)) | A hard gate bounces the ball back out of the pocket |
| Catch confirmation | IR break-beam (5 mm pair) across the held ball | Exact catch/miss score for logging and tuning; doubles as "ball loaded" |
| Misses | A person picks up the ball; a pickup mechanism is a possible future upgrade | |
| Camera occlusion | The funnel may block the ball in the final moments; acceptable because the landing prediction is committed by then | |

## 9. Throwing

A gravity-fed, gated, left/right dual-flywheel launcher at a fixed 52°, aimed by rotating the base.

**Ball path:**

1. The trapdoor opens.
2. The ball drops through a J-bend (50 mm centerline radius) that turns it up the 52° launch line.
3. It rolls into the nip between two Ø70 flywheels.
4. The flywheels throw it.

The flywheels sit left and right with axes perpendicular to the launch line (38° from vertical). Equal speeds launch without spin.

| Item | Baseline | Why / evidence |
| --- | --- | --- |
| Feed | Gravity through the J-bend; needs ≥15 cm of drop, so the lidar/Jetson layer is 65 mm (was 105) | A 52° launch can't be gravity-fed into over-under wheels; the J reaches the nip with margin in every sim shot |
| Flywheels | Printed PETG hub + TPU 95A tire, **Ø70 × 28 mm**, 8 × M4 steel bolt ring: 8.0e-5 kg·m², 120 g each; 10 mm squeeze on the ball (~15 N (est)) | Smallest shot dip and tightest scatter of 50–80 mm in the bench study |
| Motors | 2 × **T-Motor F90 2806.5 1300KV** (iFlight XING 2806.5 1300KV equivalent), M5 shaft | Huge margin: 14% throttle at max speed on a 22 V battery |
| ESC | **TBS Lucid 8S 60A 4-in-1, AM32, bidirectional DShot** (2 of 4 channels) | Must be 8S-rated (a full 7S pack is 29.4 V); eRPM telemetry for the MCU rpm loop |
| Aiming | Yaw from the base spinning in place; pitch fixed at 52°; distance set by flywheel rpm through `launcher_cal.json` | High, slow arcs are easy for kids to catch |
| Gating | The gate opens only when aimed, both wheels at speed (±2%) and safety checks pass | Every shot is a deliberate event |
| Spin | None (left/right wheels at equal speed) | Backspin would need over-under wheels; not needed for gentle lobs |

**Expected performance** (bench in sim, `launcher_bench.py`):

| Flywheel rpm | 1000 | 2000 | 3000 | 4000 | 5000 |
| --- | --- | --- | --- | --- | --- |
| Exit speed (m/s) | 2.4 | 5.1 | 7.7 | 10.2 | 12.6 |
| Elevation (°) | 43 | 50 | 51.4 | 51.8 | 51.9 |

- **Transfer:** about 0.70 of rim speed. Repeat shots agree within ±0.1–0.2 m/s; azimuth scatter is within ±2.6°.
- **Recovery:** each shot dips the flywheels ~20%, and they recover in 30–40 ms; the calibration table absorbs the dip.
- **Spin-up:** to 3 m-throw speed in 0.13 s, with a 4 A peak bus current.
- **Requirement:** 5.5 m/s for a 1.5 m return throw, 6.8 m/s at 3 m and 8.8 m/s at 5 m, all to chest height. Design max is 10.1 m/s (+15%), which needs ~3950 rpm.
- **Return accuracy in the full game:** median 14 cm from the thrower's hands.
- **Consistency** comes from feeding the ball the same way every time (the J and gate) and from the calibration table refined with logged throws.

## 10. Power

| Item | Baseline | Expected |
| --- | --- | --- |
| Pack | 7S1P Molicel P50B 21700 (5 Ah, 25.2 V nominal, 29.3 V full), BMS ≥ 40 A, main fuse, low-voltage cutoff | 612 g, R0 ≈ 137 mΩ incl. BMS and wiring (est) |
| Worst burst | Hardest base move + both flywheels spinning up | 12.6 A, 29.3 → 27.5 V |
| Average play | 28 W (Jetson + sensors ~22 W dominate) | ~3.9 h per charge |
| Low charge | 10% SoC, 15 °C | 22.4 V loaded; top speed still 1.97 m/s |

7S1P is enough. A 7S2P pack doubles the mass for runtime nobody needs in attended sessions. Sessions are sized to one battery; a charging dock is not needed.

## 11. Mechanical design and CAD

Python (build123d) generates every printed part as STEP (for SolidWorks) and STL (for Bambu Studio). Dimensions are imported from the simulation (`cad/dims.py` imports `params.py` and `geometry()`), so the CAD and the sim can't drift. SolidWorks remains the master assembly for mechanisms and fine work, using wrapper parts with named reference geometry. That workflow is unchanged from Oct 16: wrapper parts reference their own origin, STEP files are imported with 3D Interconnect, and each part follows the coordinate-frame contracts below.

**Coordinate-frame contracts** (`cad/dims.py`)

| Frame | Definition |
| --- | --- |
| ROBOT | Origin on the floor under the base center; x forward (launcher side), y left, z up |
| PAYLOAD | Origin at the payload-plate top center (z = 185 mm); axes parallel to ROBOT. The first contract, shared by every module |
| POD | Origin at the wheel center on the axle; +x along the axle toward the robot center |
| MOTOR | As in the CubeMars STEP: axis +Y, output face y = −14.0, rear face y = +26.2 |

**Current printable design** (38 printed pieces from 32 unique parts, ~2.3 kg of PETG/PLA/TPU, all ≤ 248 mm, 0 clashes; see `cad/out/PRINT_LIST.md`)

- **Base:**
  - 3 wheel pods, each one PETG print: motor bulkhead, top flange and outer wheel guard.
  - 3 wheel adapters.
  - A Ø240 center plate with 6 half-lapped 60° deck sectors, making a Ø450 deck at 108–114 mm.
  - 6 TPU clip-on bumper arcs, each bridging a sector joint, with a skirt to 45 mm above the floor.
  - A strapped battery tray under the plate and a lidar riser.
- **Payload interface:**
  - Ø240 PETG plate on 3 × 65 mm M4 standoffs (at 0° / 120° / 240°, r 100).
  - A 25 mm grid of M4 heat-set inserts.
  - Two asymmetric Ø4 dowels at (87.5, 37.5) and (−37.5, −62.5), so a module fits only one way.
  - Four captive M4 thumbscrews into grid inserts at (±50, ±50).
  - A Ø28 center hole for the power/CAN/USB connector.
- **Catch module:**
  - Launcher base plate carrying both tilted flywheel-motor pedestals.
  - Two channel halves (pocket, drop tube and J), split on the centerline and printed on their outer walls so the ball path comes out smooth.
  - Trapdoor flap, servo tower, flywheel hubs and tires.
  - Funnel collar, 4 rim arcs, 2 camera brackets.
  - A 1:1 sewing pattern for the net (`out/funnel_net_pattern.svg`).
- **Purchased:** the CubeMars AK40-10 STEP and drawing are in `cad/vendor/`; other parts are envelopes marked VERIFY.

Payload budget: size the base for about 3–4 kg of payload; firmware lowers acceleration limits automatically based on the attached module.

## 12. Simulation

MuJoCo is the primary simulator and the main training environment; Gazebo is added later for full-system navigation testing.

| Version | Content | Speed |
| --- | --- | --- |
| v1 hello world | Kiwi base (sphere omni wheels), velocity-servo motors, funnel, launcher modeled by its output, catch loop, 10 tests | ~9× real time |
| **v2 detailed physics** (current) | See the model list below; 7 tests; 5 trade studies | ~4× real time at a 0.5 ms step |
| v3 compact (proposed) | v2's models on a compact robot: N compliant wheels (+ rocker), jerk-limited direction-aware control, tilting catch/launch module with a plunger feed, base-mounted cameras, a stereo perception model, moving catches; 20 tests; 7 studies (section 20) | ~3× real time; ~1× with the perception model |
| v5 unlimited (exploration) | v3b with a "physics" limits profile (live back-EMF speed ceiling, full traction table, no jerk cap), a per-pixel vision model, extra balls, magazine tube / sweeper hopper, shooting game; 25 tests; 6 new studies (section 21) | ~2.5× real time; ~2× with the vision model |
| v6 shallow throw (proposed) | v5 plus a roller feed (driven feed wheels, one ball per trigger), padded pocket, 35° catch tilt, honest stereo track start, landing prior option, stirrer and tennis-style indexer hoppers, misfire scoring (section 22). Round 3 (section 23): datasheet feed-motor and driver models, viscoelastic contacts, randomized worlds, rolling resistance, floor friction fields, thresholds and carpet; 43 tests | ~1.5× real time (~2× with viscoelastic contacts) |

v2 models:

- **AK40-10:** FOC speed loop with current and back-EMF limits at the live bus voltage, gearbox, two-node thermal model.
- **Battery:** OCV, R0 and an RC branch, with BMS trip.
- **Omni wheels:** a fast sphere model, plus an explicit double-row roller model for validation.
- **Launcher:** physical flywheels on sprung slides, drone motor + AM32 ESC models, closed-loop rpm.
- **Gate:** the trapdoor servo.
- **Ball and funnel:** foam pickleball with drag, a lossy net.
- **Scene:** a nicer scene with video and HUD.

**Modeling choices:**

- The base uses three wheels with direction-dependent traction (the sphere model) plus the motor model, rather than every roller.
- The launcher is now physical, calibrated into `launcher_cal.json` the same way the real one will be.
- Perception is still ground truth plus delay. Next: synthetic detections with realistic noise, latency and dropped frames. The detector itself is trained and tested on real recorded images.

**MuJoCo lessons learned** (read before changing the models):

1. Frame sensors with `objtype="body"` report the inertial (CoM) frame; use `xbody`.
2. Soft-contact stiffness scales with the constraint's effective mass, so a ball pinched between two wheels gets huge forces (250 N). Model the squeeze compliance as sprung slides and keep the contact stiff.
3. Default soft friction acts like a weak damper. Friction-critical contacts (wheels on floor, ball on flywheels, ball on funnel, ball on gate) are explicit `<pair>`s with `solreffriction 0.002`.
4. Flywheel shots above ~4000 rpm are erratic at a 1 ms step; v2 uses 0.5 ms.
5. Thin collision panels let a 6.5 m/s ball tunnel through. Funnel panels are 10 mm thick with stiff contact.
6. The sphere-wheel contact body needs real mass (≥ 0.06 kg). A 20 g sphere drifts in yaw at a 2 ms step.
7. The roller model's bearing drag must be scaled by (r_roller/R)².
8. Keep Python out of the inner loop: the drag predictor uses plain floats, and the throw solver runs once per state change.
9. ESC model details: complementary-PWM braking; current limiting acts through the duty cycle.
10. build123d quirks: extrusion follows the face normal (winding order), and touching faces don't fuse (overlap by 0.5 mm).

**First job, the spec sweep (not yet run):** sweep acceleration, funnel size, perception latency, camera noise, throw distance and apex, and map catch rate across them to settle the provisional requirements. This needs the perception model first.

**Sim-to-real calibration:** once hardware exists, measure and feed back:

- motor step responses (refit `catalog.py`; a characterized motor is a new catalog entry),
- end-to-end latency and jitter,
- ball drag (film throws), mass and restitution,
- funnel bounce (drop tests),
- roller-floor μ,
- the launcher calibration (50–100 logged shots, rewrite `launcher_cal.json`).

Randomize each parameter within its measured range during training. **Key trust metric:** the gap between simulated and real catch rate for the same settings.

## 13. Real-world test area

Real sessions are short, attended calibration and validation runs, so the test area stays simple; unattended overnight training on hardware is not planned. (Unchanged from Oct 16; the sim models this same area.)

- **Area:** about 4×5 m with soft netting walls and foam bumpers along the bottom edge. Measure the floor's roller friction first: it sets the acceleration limit.
- **Throwing source:** a person throwing, or optionally a simple launcher (a copy of the robot's flywheel launcher on a pan-tilt base) for repeatable, logged pitches.
- **Referee camera:** a fixed overhead camera records every trial as ground truth for catches and onboard accuracy.
- **Logging per trial:** throw parameters (if launched), full ball track and landing prediction over time, robot path, break-beam result, return-throw settings and outcome, plus video. Stored as rosbag per session plus a summary table.
- **Metrics:** catch rate by distance from start, positioning error at ball arrival, prediction convergence time, return accuracy, cycle time, faults. The same metrics are computed in simulation (`catch_game.py` already reports catch rate, positioning error, return accuracy, battery and motor temperature).
- **Session outputs:** calibration data for the simulation, and validation of simulation-tuned settings before they are adopted.

## 14. Safety

Safety is layered so that no high-level code change can make the robot exceed hard limits; kids are the design case. (Unchanged from Oct 16, apart from the notes on the new mechanisms.)

**Hazards:** a ~5.6 kg robot at 1.4 m/s carries about 5.5 J (enough to hurt a small foot or knock over a toddler), pinch points at omni wheels, flywheels and the trapdoor, the launched ball, tipping, and the battery.

**Layers, most independent of software first:**

1. **Physical e-stop:** a 22 mm mushroom button on the rear-left deck sector (135°), cutting motor and payload power through a relay, independent of any code.
2. **Wireless e-stop:** ELRS receiver paired with the RadioMaster TX15, wired to the same power cutoff, failsafe set to stop on link loss.
3. **Firmware limits (microcontroller):**
   - caps on speed, acceleration (direction-aware, see section 3), motor current and flywheel RPM;
   - every stop is ramped, never a step to zero wheel speed: a stepped stop at 1.4 m/s can tip the kiwi (section 3);
   - a Jetson heartbeat watchdog;
   - the Jetson can request less but never more, and only Fernando flashes this firmware;
   - AK40-10 current limits set as a backstop; their comm-loss timeout is to be confirmed in the manual.
4. **Bumpers and guards:** TPU bumper skirt around the base (contact switches to add), printed guards over the wheels' outer faces and tops, enclosed flywheels (guard over the intake to add).
5. **Lidar people-awareness (Jetson software):** slow down when anyone is within about 1.5 m and keep target positions away from people. Useful but not trusted alone.

**Modes:**

- **Kid mode:** hard-capped speed (around 0.8 m/s) and acceleration, gentle lob throws only, launcher aimed at chest height or below.
- **Adult mode:** full performance envelope.
- **Rule:** settings trained in simulation must pass a supervised adult-only session before they are allowed in kid mode.

**Audible cues:** the speaker chirps before the robot starts moving and gives a distinct warning tone before the launcher fires.

**Battery:** Li-ion pack with BMS, main fuse, low-voltage cutoff; charge only while supervised, ideally in a fire-resistant bag or box.

## 15. Future features and modular payload interface

The base is a general mobile platform with the standard payload interface on top (section 11), so the catch module can be swapped for a robot arm or removed entirely. (Unchanged from Oct 16.)

- **Electrical:** one connector with e-stop-switched 24 V power, CAN, and USB or Ethernet to the Jetson.
- **Identity:** each module announces itself over CAN; software loads the matching behavior, safety limits and mass model (catch mode, arm mode, or bare base).
- **What lives where:**
  - The base carries everything every personality needs: lidar, IMU, optical flow, speaker, RC receiver, Jetson, battery, microcontroller.
  - Mission-specific parts ride on the module: stereo cameras and launcher on the catch module; the arm module brings its own wrist or gripper camera.
- **Planned features:**
  - **Speaker:** USB or I2S amp and speaker on the Jetson for personality (sounds, voice, celebrating catches) and safety cues.
  - **RC driving:** the ELRS receiver used for the wireless e-stop also carries stick inputs to the microcontroller over a serial link. A TX15 switch selects manual or autonomous. In manual mode the microcontroller drives the base directly under the same firmware limits, even with the Jetson off; link loss means stop.
  - **Robot arm module:** a second personality on the same payload interface, with its power cut by the e-stop chain.

## 16. Bill of materials

Core robot plus catch module: roughly $1,600–2,800, with the Jetson tier driving most of the spread. The AK40-10, goBILDA wheel and Jetson prices are verified; the rest are typical street prices.

| Item | Part | Est. cost (USD) |
| --- | --- | --- |
| Jetson | Orin Nano Super dev kit ($399), or Orin NX 16GB (~$999 + $150–250 carrier) | 400–1,200 |
| Drive actuators | 3 × CubeMars AK40-10 V3.0 KV170 with driver ($135.90) | 408 |
| Omni wheels | 3 × goBILDA 3624-0014-0096 ($21.99) | 66 |
| MCU, IMU, optical flow | STM32G4/H7 board with CAN-FD, ICM-42688/BNO085, PMW3901 | 70–120 |
| Lidar | RPLIDAR C1 | 70–100 |
| Cameras | 2 × OV9782 global-shutter color modules + cables | 150–250 |
| Battery | 7 × Molicel INR21700-P50B, 7S BMS ≥ 40 A, charger | 150–200 |
| Power | Fuses, e-stop relay, DC-DC converters, wiring, 22 mm e-stop | 80–120 |
| RC / audio | ELRS receiver (TX15 already owned), speaker + amp | 50 |
| Launcher | 2 × T-Motor F90 2806.5 1300KV, TBS Lucid 8S 60A 4-in-1 ESC, 25 kg·cm servo (e.g. DS3225MG) + 25T horn | 170–200 |
| (v6 round 3, proposed) Feed + tilt | 2 × LK-TECH MF4005-V2 integrated FOC motors ($100; DIY alternative: GM3506 + AS5048A + SimpleFOC Mini, $69), 12 V buck converter (~$10), Feetech STS3250 tilt servo ($74) in place of a PWM servo. Full motor + controller BOM: section 23.5 | ~284 |
| (v6 round 3, proposed) Cheaper drive | 4 × DJI M2006 P36 + C610 ($94 at Oct 20 prices) instead of 4 × AK40-10 ($136), on a 6S pack: −$168. Cheapest compliant motor + controller set ~$714 vs ~$937 integrated (section 24) | −168 |
| Catch hardware | IR break-beam (5 mm), 6 × Ø8 carbon tubes, Ø3 hinge pin, EVA foam, mesh fabric | 30–50 |
| Printed parts | ~2.3 kg filament (PETG, PLA, TPU 95A) | 50–80 |
| Fasteners | M3/M4 heat-set inserts, M2.5/M4/M5 screws and nuts, 3 × 65 mm M4 standoffs, Ø4 dowels, 4 × M4 thumbscrews (full list in `cad/README.md`) | 40–60 |
| **Total** | | **~1,750–2,900** |

Sources: [CubeMars AK40-10 V3.0](https://www.cubemars.com/product/ak40-10-v3-0-kv170-robotic-actuator.html), [goBILDA 96 mm omni](https://gobilda.com/96mm-omni-wheel-14mm-bore-50a-durometer), [Jetson pricing, July 2026](https://www.cnx-software.com/2026/07/22/nvidia-increases-the-price-of-jetson-modules-and-devkits-by-up-to-101/?amp=1), [T-Motor F90](https://pyrodrone.com/collections/motors/products/t-motor-f90-2806-5-long-range-motor), [TBS Lucid 8S](https://www.rotorama.com/product/tbs-lucid-8s-4v1), [Molicel P45B datasheet](https://www.kronium.cz/uploads/p45b-datasheet.pdf).

## 17. Phased roadmap

1. **Simulation and design (buy only the Jetson)**. *In progress.*
   - Done: v1/v2 sims, component study, printable CAD.
   - Remaining: perception model and spec sweep, dev environment (Jetson + WSL2 + Foxglove + Claude Code), SolidWorks wrapper parts.
   - Exit: provisional requirements confirmed or revised from sweep results.
2. **Perception bench.** Cameras and lidar on the Jetson, no robot. Track hand-tossed foam balls and measure landing-prediction accuracy and how early it converges.
   - Exit: prediction within about ±5 cm early enough to leave ~0.7 s of motion time.
3. **Base and safety.** Print pods, deck and adapters; motors, microcontroller firmware with limits and watchdog, physical and wireless e-stop, bumpers, RC driving. Measure base dynamics and roller μ, and feed them back into the sim.
   - Exit: all safety layers tested; RC driving works with the Jetson off.
4. **Catch module.** Funnel, pocket, trapdoor and cameras on the robot; stop-and-catch with adult tosses; funnel drop tests to calibrate contact.
   - Exit: real catch rate tracked against simulation.
5. **Launcher and return throws.** Bench-calibrate the flywheels (rewrite `launcher_cal.json`), measure scatter, validate kid mode.
   - Exit: consistent, catchable return throws; kid mode signed off in an adult-only session.
6. **Improvement loop.** Simulation-driven tuning with real validation sessions; lidar SLAM/Nav2; speaker personality.
   - Exit: small, stable sim-vs-real gap.
7. **Arm module.** Second personality on the same payload interface.

## 18. Open questions and items to verify

**Decisions**

- [ ] Jetson tier: Orin Nano Super dev kit vs Orin NX 16GB, and which carrier board.
- [ ] Adopt the proposed changes: 2.5 m/s² direction-aware acceleration limit, 74 mm / 22 g foam ball, 96 mm wheels.
- [ ] MCU board choice (STM32G4/H7 with CAN-FD).
- [ ] ROS 2 and Ubuntu version alignment between the Jetson and the PC Docker image.
- [ ] v3 compact proposals (section 20): 4-wheel X base; rocker vs rigid chassis; tilting catch/launch module with a 55° funnel; cameras on the payload-plate front; tilt servo choice.
- [ ] v3b proposals: deck in the axle plane (moves the payload standoffs to 0/90/180/270°, a payload-interface change); plunger feed; flywheel brake-on-stop; direction-aware acceleration cap up to ~4 m/s².
- [ ] v6 proposals: roller feed in place of the plunger; padded pocket walls; catch tilt cap 35° (tilt range −30…+40°); honest track start; throw line out to 6–7 m; multi-ball by the tube insert or an indexer with a separate catch path.
- [ ] v6 round 3 proposals (section 23): GM3506 feed motors on a 12 V rail; M2006 + C610 drive instead of AK40-10 (after the voltage check); a serial-bus tilt servo with an encoder; bumper skirt to ~20 mm + wheel-top guards; threshold slow zones; a two-row battery pack (the one-row pack overlaps the motors in v3b's layout).

**Hardware to verify**

- [ ] OV9782 driver support for the chosen JetPack version and carrier.
- [ ] Flashing path for the chosen Jetson (SD card image vs SDK Manager on an x86 Ubuntu host).
- [ ] AK40-10: is the output's Ø15 feature a recess (assumed) or a boss? Does the driver accept a full 7S pack (29.4 V)? Radial and moment load on the output bearing with the overhung wheel. Comm-loss timeout and current-limit settings. Thermal constants (sim says the duty is light).
- [ ] goBILDA 96 mm omni: width (38 mm assumed) and 32 mm-pattern hole phase; behavior on the real floor's thresholds and carpet edges.
- [ ] Roller-floor μ on the play floor (0.95 assumed): sets the acceleration limit.
- [ ] Foam ball: diameter, mass, Cd (film throws), restitution on the floor and on the EVA pad.
- [ ] Launcher transfer on the bench (~0.70 assumed), squeeze force, TPU-on-foam grip; 2806.5 mount pattern (19 × 19 assumed) and M5 shaft fit.
- [ ] Gate servo horn pattern and tab spacing; Jetson, RPLIDAR C1 and OV9782 hole patterns (all marked VERIFY in `cad/`).
- [ ] Funnel net fabric choice (grippy, soft) and drop tests at varied speeds and angles.
- [ ] Confirm SolidWorks 3D Interconnect STEP updates keep wrapper references and mates on a first test part.
- [ ] AK40-10 behavior on CAN loss: coast, ramp or hold zero speed (holding can tip a kiwi, section 3).
- [ ] (v3) Roller vertical stiffness (30 N/mm assumed) and chassis/floor flatness: they decide rocker vs rigid.
- [ ] (v3) Ball rebound off the gate pad at 6 m/s (drop test): the tilted funnel needs an effective restitution ≈ 0.05 (sim) or a lower tilt gain.
- [ ] (v3) OV9782 module + 30–40 cm 22-pin FFC at 1280×800 / 120 fps on the chosen carrier.
- [ ] (v3b) RPLIDAR C1 scan-plane height vs the ~15 mm window between the wheel tops and the payload plate; plunger servo speed and horn/link geometry (0.4 m/s over 50 mm assumed); AM32 brake-on-stop on the Lucid ESC.
- [ ] (v5) Ball spin of real tosses (film them): 10 rad/s doubles the landing error. Stereo calibration quality reached in practice (0.02° assumed; 0.5° drops catches to 58%). End-to-end camera latency (50 ms assumed; 22–26 ms raw V4L2 / 40–50 ms Argus estimated, section 21.4): measure with an LED and a photodiode. Does the OV9782 driver for the chosen carrier offer 1280×800 at 120 fps (some listings give 60), and does the module expose the frame-sync pin? Printed-flywheel spin test behind a guard before any shot above 5,000 rpm.

- [ ] (v6 chute) Feed gearmotor at ~760 rpm on Ø40 wheels (or Ø60 at ~510) that still self-locks; J-bend and launch-run friction (μ 0.25 assumed); second IR beam at the feed-wheel exit; exit-speed calibration per ball batch (±3.5% across 72-76 mm).
- [ ] (v6) Ball diameter spread of a real batch (the chute takes 72-76 mm; the round-1 tangent fit needed ±0.5 mm); the room's ceiling height; printed-flywheel spin test to ≥ 5,000 rpm (indoor 7 m returns run at ~4,500).
- [ ] (v6) ~~Feed wheels: N20 worm gearmotor speed (~380 rpm assumed) and torque (0.3 N·m), self-locking under a ball push~~ (round 3: GM3506 gimbal motors, see below); feed-wheel preload (6 N) on a rigid stop; TPU-on-foam grip. Net rolling resistance (12 mm assumed; 4 mm lets the indexer feed). Pocket pad restitution on the side walls. Foam "practice dodgeball" mass, size, Cd if it replaces the pickleball foam ball.
- [ ] (v6 round 3) GM3506 + AS5048A + SimpleFOC Mini on a 12 V rail: bench a feed-wheel pair for 764 rpm, the hold stiffness against a dropped ball (sim: ≤ 3 mm back-drive at the tire), heating while holding a 6-ball magazine; the kt from ke assumption (est).
- [ ] (v6 round 3) Drop-test the foam ball at 3 heights (e.g. 5 cm, 1.25 m, 2.5 m: ~1, 5, 7 m/s) on the floor and on the catch pad: restitution 0.55 / 0.38 and 0.10 / 0.04 assumed. Squeeze-force vs rate on a flywheel (static 15 N, 2× at high rate assumed).
- [ ] (v6 round 3) Stereo calibration stability on the robot, measured over a session and after ball impacts: the randomized study's dominant factor (0.04° → 97%, 0.3° → 78%).
- [ ] (v6 round 3) Roller rolling resistance on the play floor and on carpet (Crr 0.02 / 0.06 assumed); real thresholds and rug edges in the play area.
- [ ] (v6 round 3) ~~DJI C610 input voltage range against a full 7S pack (29.4 V)~~ (section 24: run the M2006 on a 6S pack instead); M2006 P36 backlash and noise; motor and ESC supply (both out of stock at DJI on Oct 20).
- [ ] (Section 24) M2006 winding temperature on the bench at ~0.4 N m RMS (sim est 70 °C; 127 °C with the reseller's torque constant). AK40-10 maximum supply voltage (needed for the 7S integrated set). Feetech STS3250 continuous torque rating. LK-TECH MF4005 supply range (a listing says 7.4-32 V). Pilotix / Sequre 8S ESCs: bidirectional DShot with AM32. Is a tilt counterbalance spring practical?
- [ ] (Section 24) Decision: cheapest set on 6S (M2006 drive) or integrated set on 7S (AK40-10 drive).

## 19. Decision log

| Date | Decision | Reason |
| --- | --- | --- |
| Oct 16 | Kiwi base, AK40-10, 7S Li-ion, Jetson + STM32, ROS 2, stereo OV9782, MuJoCo-first | Original design doc |
| Oct 17 | Wheels 125 mm → **96 mm goBILDA** | Lighter, grippier, lower CG; catch performance unchanged |
| Oct 17 | Base acceleration target 4–5 → **2.5 m/s², direction-aware** (proposed) | Traction-limited kiwi; catches insensitive above 2 m/s² |
| Oct 17 | Ball → **74 mm foam pickleball ~22 g** (proposed) | Fernando's request; safe and soft; strong drag modeled |
| Oct 17 | **Catch pocket + EVA-padded trapdoor + J-bend gravity feed** | Sim failures: bounce-out, slide-out, no gravity path to over-under wheels at 52° |
| Oct 17 | Launcher: **left/right Ø70 flywheels, F90 2806.5 1300KV, 8S AM32 ESC** | Bench study; 7S needs an 8S-rated ESC |
| Oct 17 | Lidar/Jetson layer 105 → 65 mm | Drop height for the J-bend |
| Oct 17 | Printed-first mechanical design (PLA, PETG, TPU) | Fernando's preference; 2.3 kg printed, all parts fit a 256 mm bed |
| Oct 17 | Firmware stops always ramped; e-stop coasts (proposed) | A stepped zero-speed stop at 1.4 m/s tips the kiwi on grippy floors (v3 sim) |
| Oct 17 | v3 compact robot: X4 base, tilting catch/launch module, 55° funnel, base-mounted cameras (**proposed**, section 20) | Fernando's questions; v3 sim studies |
| Oct 17 | v3b: deck in the axle plane, plunger feed instead of the J, braked idle flywheels, return-throw bias learning (**proposed**, section 20) | CG 142 → 95 mm, rim 494 → 387 mm; J's 0.4 s feed → 0.1 s |
| Oct 17 | Adult mode: 2 m/s cap; moving catches only with a 20 cm clear path (stop included); arena stays 5 × 4 m | Fernando |
| Oct 17 | Moving catch = braking curve to the slowest on-time end speed, replanned at 20 Hz; brake at 3 m/s² with the ball aboard (**proposed**) | 83 → 86% caught with modeled perception; the safety gate costs ~0 |
| Oct 17 | v5 sim: no-limits envelope, per-pixel vision model, hopper mode (exploration; v3b's firmware limits stay the design) | Fernando's questions; section 21 |
| Oct 17 | Hopper mode needs a single-file magazine tube (proposed); the bare funnel stops after 2 balls | Two balls arch over the Ø84 throat (v5 hopper_rig) |
| Oct 17 | Camera spec (proposed): stereo pair triggered from the MCU, stiff stereo bar, raw V4L2 path if Argus > ~50 ms, driver must offer 1280×800 at 120 fps | Expected pipeline 86–92%; drifting calibration (0.3°: 85%) and a slow stacked pipeline (72%) are the risks (section 21.4) |
| Oct 17 | Throw line at ~2.5 m; long throws as high lobs (proposed) | Flight time comes from the apex, not distance; scatter and flat arrivals drop catches beyond 4 m (section 21.4) |
| Oct 17 | Keep the X4 omni for catching; 4-module swerve is the upgrade path for a general platform (proposed) | Traction-limited omni; swerve +2–4 points; mecanum and diff drive are worse here (section 21.4) |
| Oct 18 | v6: roller feed (worm-geared feed wheels as gate and active pass, one ball per trigger), padded pocket walls, catch tilt to 35°, honest stereo track start (**proposed**, section 22) | 6–7 m shallow throws 57% → 96% (firmware adult); 0 misfires in hopper shooting; returns 3 cm at 6.5 m (sim) |
| Oct 18 | Multi-ball: keep the magazine tube; a tennis-style indexer only with a catch path that bypasses it (proposed) | Bare funnel domes after 2 balls (stirrer, vibrator: unreliable); indexer feeds but catches 20% |
| Oct 19 | Feed wheels stay tangent to the nominal ball; ball spec ±0.5 mm until the pocket is made size-tolerant (proposed) | Pinching the resting ball for size tolerance perches caught balls on the tires (returns 3 → 45 cm) (section 22.4) |
| Oct 19 | **Chute pocket** replaces the roller pocket (proposed): floor sloping into the stopped feed wheels' nip, slick J-bend, 1.6 m/s feed, feed-exit beam | Fernando's choice; feeds every 72-76 mm ball at every tilt; catching, misfires unchanged; bursts 6/6 with no double feeds (section 22.4) |
| Oct 19 | Indoors: ceiling-aware return elevation (apex 0.25 m under the ceiling); 7 m returns in an 8 ft room at ~35-40° and ~4,500 rpm (proposed) | 0 ceiling hits, returns 7 cm median; the 50° lob hits the ceiling 96 times in 97 (section 22.5) |
| Oct 19 | Feed wheels on direct-drive gimbal motors holding with current instead of a worm: **LK-TECH MF4005-V2 (integrated FOC + encoder, CAN; Fernando prefers integrated devices)**, or the GM3506 + SimpleFOC Mini as the DIY fallback, on a 12 V rail (proposed) | Feeds 72-76 mm at every tilt; a fast catch back-drives them ≤ 3 mm; tightest bursts. The micro gearmotor and N20 worm can't do 764 rpm with torque; the 25D doesn't package (section 23.1) |
| Oct 19 | Sim: viscoelastic contacts (Hunt-Crossley ball, standard-linear-solid tires), rolling resistance, collidable bumper + skirt + wheel guards are the v6 defaults | More honest physics; catching and returns unchanged or better (sections 23.2, 23.4) |
| Oct 19 | Measure first: stereo calibration stability, then throw spin and camera latency; ball properties can wait (proposed) | Randomized worlds: calibration 0.04 → 0.3° costs 19 points; ball size/mass/drag/bounce within ±2 (section 23.3) |
| Oct 19 | Bumper skirt down to ~20 mm and guards over the wheel tops; slow to ≤ 1 m/s over mapped thresholds; no square rug edges in the play area (proposed) | A missed ball in a wheel slot, or a hop over a threshold onto it, flipped the robot; an omni wheel can't climb a square 8 mm edge sideways (section 23.4) |
| Oct 19 | Keep 4 wheels; the AK40-10 is not needed: DJI M2006 + C610 proposed, pending its voltage rating; tilt actuator → a serial-bus servo with an encoder (proposed) | The kiwi loses 11 points; every 4-motor set catches the same; the AK40-10 uses a third of its torque and runs out of speed on a low battery (section 23.5) |
| Oct 20 | Shared actuator database in `database/` (CSV + `actuator_db.py`; moves to `Robotics/database/` later) | Fernando's request: one sourced, provenance-marked part list for every project, with checks, costs and sim export (section 24) |
| Oct 20 | Cost-first actuator set (proposed): **6S pack**, M2006 + C610 drive, brushed Pololu 25D + DRV8874 feed, Feetech STS3250 tilt, F90 + 8S AM32 4-in-1 flywheels, ~$714; integrated fallback on 7S (AK40-10, 25D + RoboClaw) ~$937 | Catches the same at 10% charge on 6S as on 7S; the C610 stays inside its 24 V rating; brushed drive motors overheat in continuous play (est 157 °C); the 25D is the cheapest feed motor that holds (0.7 mm) (section 24) |

## 20. v3 compact robot study (proposed)

**Everything in this section is Proposed.** The sim is `simulation/v3 compact` ([README](simulation/v3%20compact/README.md)). Perception is still ground truth.

Two rounds of questions from Fernando:

- **Round 1 (v3a):** do four omni wheels raise the acceleration limit? Can one pitch joint on the whole funnel + launcher shorten it, and also set the launch angle? How far can the cameras be from the Jetson? At the traction limit, can the robot tip over?
- **Round 2 (v3b, the sim's defaults):** lower the deck into the wheel-axle plane; replace the J-bend gravity feed with an active one; does the robot ever yaw to shoot back; how sensitive is catching to throws far from the baseline?
- **Round 3:** a perception model, and catching while moving (2 m/s, only with a 20 cm clear path, 5 × 4 m arena).

![v3 poses](simulation/v3%20compact/renders/v3_poses.png)

**The robots**

| | v2 baseline | v3a (round 1) | v3b (round 2) |
| --- | --- | --- | --- |
| Base | Kiwi, 3 × AK40-10 at r 180 mm, deck Ø450 above the wheels | X drive, 4 × AK40-10 at r 150 mm, deck Ø370 above the wheels, front pair on a ±3° rocker | Same wheels and rocker; **deck in the axle plane**, notched for wheels and motors; battery as a flat 7×21700 row under it |
| Catch module | Fixed, 35° cone 176 mm deep | Pitches −20°…+35° on one joint; 55° cone 86 mm deep | Same funnel; pitches −30°…+25° |
| Feed | Trapdoor + 145 mm gravity drop through a J | Trapdoor + 118 mm drop through a J | **Plunger**: the old gate servo pushes the ball 50 mm up an inclined pocket floor into the flywheels |
| Launch elevation | Fixed 52° | 35–60° (40° in the module minus tilt) | 30–60° (30° in the module minus tilt) |
| Rim height / plate-to-rim | 600 / 415 mm | 494 / 298 mm | **387 / 265 mm** |
| Mass, CG height | 5.56 kg, 151 mm | 5.85 kg, 142 mm | **5.34 kg, 95 mm** |
| Cameras | Funnel rim, ~1 m of cable | Payload-plate front, ~40 cm | Payload-plate front, ~34 cm |
| Catch game (perfect perception) | 30/30, returns median 14 cm | 60/60, returns median 13–14 cm | 60/60 (funnel offset ≤ 3.1 cm), returns median 13–14 cm |

### Round 3: modeled perception and the moving catch

Fernando's decisions for this round: adult mode may drive at 2 m/s; moving catches only when the path is clear by 20 cm; the arena stays 5 × 4 m.

**Perception model** (`perception.py`; all figures are estimates until the OV9782 pair is on the bench):

- Stereo detections at 120 fps, only when the ball is in both fields of view and not hidden by the robot, the funnel net or the thrower (ray cast).
- 5% dropped frames; 0.5 px centroid and 0.3 px disparity noise (depth error grows with distance squared); 3 mm camera-pose error; 50 ± 15 ms latency.
- A drag-aware Kalman filter (0.5 m/s² process noise for spin and drag error) reports the landing point and its uncertainty.
- It replaces "exact ball state after 0.25 s". A usable estimate arrives after 8 detections plus latency, about 0.12 s after release.

**What the catch module tolerates in motion** (`studies/moving_rig.py`; 6.4 m/s ball, 24° from vertical, 17 landing offsets up to 7 cm):

| Base moving at | 0.5 m/s | 1.0 m/s | 1.5 m/s | 2.0 m/s |
| --- | --- | --- | --- | --- |
| With the ball's travel | 17/17 | 17/17 | 17/17 | 17/17 |
| Sideways | 17/17 | 17/17 | 17/17 | 16/17 |
| Against it | 17/17 | 17/17 | 13/17 | 12/17 |

- Moving with the ball makes it arrive more vertically relative to the funnel (6° at 2 m/s).
- **Braking with the ball aboard** (from 1.5 m/s, in every direction, funnel at 0° or 15°, dump starting with the braking): held at 1, 2, 3 and 4 m/s². The game brakes at 3 m/s² after a catch, with a 0.05 s coast after a moving catch.

**The moving catch** (`intercept.py`):

- When the stop controller can't get there in time, the robot passes through the meeting point *on time* at the slowest possible speed: accelerate fully, brake fully, and end at v_f instead of rest.
- This is the same braking-curve controller with an end speed, replanned at 20 Hz: getting early lowers v_f to 0 (a plain stop-and-catch), getting late raises it.
- It commits only once the landing prediction's 1σ is under 10 cm.
- **The clear-path gate:** the path, the coast and the braking with the ball aboard must keep the robot's outline 20 cm from the netting and from the thrower. If that fails, v_f is capped at the fastest speed that passes.
- **Two earlier versions lost to the plain stop controller:**
  - chasing a virtual target that slides at the meeting velocity: the robot waited for a target that started behind it, then couldn't catch it;
  - constant-acceleration guidance: it arrives at twice the average speed, past the cap and with too little room to stop.

**Results** (`studies/catch_sweep.py`, 5 × 4 m arena, waiting spot 1.5 m from the back netting):

- **Grid:** thrower 1.5–3 m away; apex 0.5 m (line drive) to 3 m; landing 0.3, 0.6 or 1 m from the waiting spot.
- **Size:** 6 tosses per cell, the same throws for every robot (216 each).

| Robot | Caught | Robot not in place / bounced out | Funnel offset of catches (median) |
| --- | --- | --- | --- |
| Stop, 1.4 m/s, perception model | 82% | 38 / 1 | 5.7 cm |
| Stop, 2 m/s, perception model | 83% | 35 / 2 | 5.6 cm |
| **Moving, 2 m/s, perception model** | **86%** | 29 / 2 | 4.7 cm |
| Moving without the clear-path gate (reference only) | 86% | 27 / 3 | 4.6 cm |
| Stop, exact state after 0.25 s | 70% | 54 / 10 | 2.7 cm |
| Moving, exact state after 0.25 s | 83% | 36 / 0 | 0.7 cm |

- **Perception timing matters more than catching on the move.**
  - With the modeled pipeline (estimate at ~0.12 s), plain stop-and-catch already gets 83%.
  - Under the old 0.25 s assumption, the same robot gets 70%, and the moving catch is worth +13 points there.
  - With modeled perception it adds +3 points: 1 m-apex throws landing 1 m away go from 0–17% to 0–83%, and catches are more precise. Of 55 moving catches, 38 held.
- **The 20 cm safety rule costs almost nothing here** (86% with or without it). It caps 20 throws, mostly toward the back netting, but those would mostly not have been caught anyway.
- **What's left is physics.** Flights of 1 s or less landing 1 m away leave the robot ~0.65–0.85 s to cover 1 m at 2 m/s. Only a faster pipeline, a longer reach (cap above 2 m/s) or a bigger funnel would help.
- **Perception model accuracy:** catches land a median 5–6 cm from the funnel center, vs 0.7–2.7 cm with exact data. The filter's own landing 1σ at commit is ~17 cm on high lobs, which leave the cameras' view near their top and descend unseen above the robot. It is conservative: actual errors are smaller.
- **Proposed:** adult mode uses moving catches with the gate; kid mode stays stop-and-catch at a low speed cap.

### Round 2 (v3b)

**5. Deck in the axle plane: CG height 142 → 95 mm, rim 494 → 387 mm, no loss of ground clearance.**

- **Layout:**
  - The deck is a center piece plus four lobes between 110 mm-wide slots for the wheels and motors, starting 75 mm out.
  - The motors (bottom at 21 mm) still set the ground clearance. The battery becomes a flat row of 7 × 21700 (~150 × 72 × 23 mm) under the deck, level with them.
  - Everything above the deck drops 63 mm. Mass falls to 5.34 kg (lighter notched deck, shorter module).
- **Packaging changes (VERIFY in CAD):**
  - The Jetson moves to the center; its v2 spot collides with a rear motor.
  - The payload-plate standoffs move to 0/90/180/270° because the diagonals hold the motors. **This changes the payload interface contract** (section 11).
  - The lidar sits on a ~23 mm riser so its scan plane (~100 mm) clears the wheel tops (96 mm) and passes under the plate (116 mm). Check that window against the RPLIDAR C1's real scan height.
  - The e-stop moves to the rear lobe.
- **Effect on the base** (`studies/wheel_count.py`, μ 0.95):

| Base | CG height | Slip-free accel, worst / mean | Tip-over (model, worst) | Panic stop from 1.4 m/s, μ 0.95 / 1.2 |
| --- | --- | --- | --- | --- |
| Kiwi r150, high deck | 150 mm | 2.7 / 4.1 m/s² | 4.1 m/s² | falls / falls |
| Kiwi r150, axle deck | 100 mm | 3.3 / 5.4 m/s² | 6.4 m/s² | 12° / 17° |
| X4 + rocker, high deck (v3a) | 142 mm | 3.1 / 4.9 m/s² | 6.5 m/s² | 11° / 13° |
| **X4 + rocker, axle deck (v3b)** | **95 mm** | **3.5 / 5.3 m/s²** | **9.9 m/s²** | **4° / 6°** |
| X4 rigid, axle deck | 95 mm | 4.2 / 6.2 m/s² | 10.0 m/s² | 0.2° / 0.4° |

- On the lowered deck even a kiwi survives a panic stop.
- The rigid X4 still loses half its diagonal traction with one wheel 1 mm proud (1.8 m/s²); the rocker holds ≥ 3.5 m/s² with 0–2 mm. So the rocker stays.

**6. Active feed: a plunger replaces the J. The module is 33 mm shorter, and the ball leaves 0.10 s after the command (J: 0.4 s).**

- **Design:**
  - The pocket floor is inclined at the module's launch angle (30°, rising toward the flywheels), so a caught ball rolls back into the corner against a plunger face standing perpendicular to the floor.
  - The old gate servo drives the plunger 50 mm up the floor through a horn and link, into the flywheel nip 50 mm ahead.
  - The trapdoor is gone.
- **Four things failed in the sim first, and shaped the design:**
  - *Wedge jam.* A paddle hinged at the pocket corner wedge-jams the ball against the padded floor, because the wedge half-angle falls below the friction angle as it swings. A plunger keeps a 90° corner and can't.
  - *Launch path under the funnel.* The launch line must be shallower than the funnel wall (30° vs 35° from horizontal) and pass under the funnel's collar and 10 mm wall. The throat is raised until it does.
  - *Push distance.* The plunger must still be pressing when the wheels take the ball. With the nip 70 mm away, a 0.4 m/s plunger let go first and exit speed scattered 3.6%; at 50 mm the scatter is 0.0%. The 0.4 m/s is set so a hobby servo can do it (est).
  - *Live flywheels.* The pocket is open toward the nip, so idle flywheels must brake to a stop (AM32 brake-on-stop) rather than coast. Coasting wheels fired out balls that bounced forward during catches, which cut catches to 73%.
- **Catch tilt limit:** the floor must still slope back during a catch, so the catch tilt stays below the 30° launch angle (cap 25°, enough at 60% gain for arrivals up to ~42°).
- **Results:**
  - Calibration: 72/72 shots fed at tilts −30°…0°, 7.7 m/s at 3000 rpm (same ~0.70 transfer as v2), elevation within ~1.5° of geometric.
  - Funnel rig: 75/75 with the v3 funnel setting; upright cones get 84–88% with this pocket.
  - Catch game: 60/60.
- **Return throws: median 13–14 cm** (same as v3a), after three fixes:
  - **Spin bias.** The ball leaves with ~2 rad/s of spin (J: ~10). MuJoCo's air model includes Magnus lift and the planner doesn't, so each feed has its own range bias (+0.17 to +0.25 m here).
  - **Bias learning.** The game learns that bias from logged landings, as the real robot will (section 9). A learning gain of 0.25 works; at 0.5 it chased throw-to-throw scatter and the median rose to 19–20 cm.
  - **Fire only when settled.** The firmware waits for the flywheels to hold speed for 0.1 s and the base to be still (< 3 cm/s) before firing. Without this, returns were 35 cm.

**7. Does it yaw to shoot back? Not needed: 0.0° in all 60 tosses.**

- The robot catches facing the incoming ball. In still air that ball's horizontal path runs straight from the thrower's hand, so at the catch the robot already faces the thrower.
- Aiming is just tilting to the launch elevation and spinning up the flywheels: median 0.34 s.
- Yaw would be needed with wind or spin curving the ball, or a thrower who moves after throwing. The base can yaw at 4 rad/s anyway.

**8. Throw sensitivity: reach is the limit, and the tilt matters for long, fast throws** (`studies/toss_sweep.py`).

- **Grid:**
  - thrower distance 1.5–4.5 m;
  - apex 0.5 m (a flat line drive) to 3 m (a high lob) above a 1.0 m release;
  - landing 0.3, 0.6 or 1.0 m from where the robot waits.

  Each cell gets 6 tosses, for 3 robots: 864 tosses with perfect perception.
- **The throws at the rim:**
  - line drives (apex 0.5 m) arrive 22–49° from vertical at 4.8–6.6 m/s, 0.78 s after release;
  - apex 1 m arrives 14–37°, 1.0 s;
  - apex 2 m arrives 9–25°, 6.5–6.9 m/s, 1.3 s;
  - lobs (apex 3 m) arrive 6–18° at 7.2–7.5 m/s, 1.5 s.

| Landing offset | v3b catches |
| --- | --- |
| 0.3 m | 100%, except line drives from 3.5–4.5 m (67%: they arrive at 42–49°, beyond what the 25° tilt cap can meet) |
| 0.6 m | Line drives 0% (0.78 s is too short to move 0.6 m); apex ≥ 1 m: 83–100% |
| 1.0 m | Apex ≤ 1 m: 0%; apex 2 m: 67–83%; apex 3 m: 83–100% |

- **Overall** (final run, jerk-aware base planner, same throws for every robot):

| Robot | Caught | Misses: robot not in place / bounced out |
| --- | --- | --- |
| v3b as first reported (catch tilt up to 25°, brakes as soon as the ball is in) | 71% | 79 / 5 |
| Same, funnel held upright | 62% | 80 / 29 |
| + catch fixes (catch tilt ≤ 15°, coast 0.2 s after the ball enters) | 71% | 79 / 5 |
| **+ direction-aware acceleration limit (the v3b defaults)** | **73%** | 71 / 7 |

  An earlier run with a 4 m/s² flat cap got 74%, but 4 m/s² exceeds the worst-direction slip limit (3.5), so it isn't safe.

- **What it says:**
  - With perfect perception the robot covers about 0.3 m in 0.8 s, 0.6 m in 1.0 s and 1 m in 1.3 s. Faster, flatter throws have to land closer.
  - The upright funnel's extra misses are long, fast lobs. At 4.5 m with a 2 m apex it held 1/6 of the balls landing 0.3 m away; the tilted funnel held 6/6.
  - The one place the upright funnel does better is long line drives landing close (3.5–4.5 m, 0.5 m apex: 12/12 vs 8/12). Those arrive 42–49° from vertical, and with the tilt capped at 25° the inclined pocket floor bounced some out. Worth revisiting the tilt gain for flat arrivals.
  - **Reach is physics, not tuning.** With the 0.25 s perception delay, a line drive leaves 0.53 s to move and stop: about 0.3 m even at 4.5 m/s². A ball landing 0.6 m away needs ~1 s of flight to reach and *stop*. Beyond that the robot would have to catch on the move (roadmap), and every 0.1 s of perception latency shrinks the reach.
  - **Direction-aware acceleration limit** (`params.ACCEL_TABLE_V3B`): 85% of the slip-free acceleration measured in each direction, mirrored left/right and capped at 4.5 m/s². That is 3.0–4.5 m/s² instead of a flat 2.5, with no slip in 0.6 m test moves in 12 directions.
    - It cut reach misses from 79 to 71: 2 m-apex throws landing 1 m away went from 67–83% to 100%.
    - It first made positioning *worse* (worst funnel offset 4.4 → 11.8 cm). The cause: the stop was planned with the current direction's limit while the base yawed into a weaker one, and the jerk limit's 0.17 s acceleration reversal wasn't in the stopping-distance plan.
    - The fix plans the stop with the table's worst value and subtracts the distance covered while the acceleration reverses. The worst offset is now 2.2–3.1 cm, better than the flat limit's 4.4.
    - This is the "direction-aware, jerk-limited" firmware limit proposed in section 3, now tested in sim.
  - **Catch fixes** for line drives caught while the base is still moving: at a 25° catch tilt, the kicker's pocket floor slopes back only 5°, and braking right after the catch rolled the ball out. Capping the catch tilt at 15° and coasting 0.2 s after the ball enters helped on the hard cells (169 → 177/180) with the earlier planner, but shows no net gain over the full grid with the jerk-aware planner, which arrives calmer.
  - Kid throws are short and loopy, which is the easy corner of this map. Adult line drives are the hard one.

### Round 1 (v3a)

**1. Four wheels: yes, but mostly through load transfer and tip margin, and only with a rocker or suspension.**

- *Theory* (`traction.py`):
  - Without load transfer, any symmetric omni base has the same worst-direction limit, μg/2 (4.7 m/s² at μ 0.95). That is reached when the motion lines up with one wheel's rolling direction (kiwi) or with a diagonal (X4).
  - What 4 wheels change:
    - On a diagonal move, the two wheels that push gain and lose no load; only the two idle wheels trade load.
    - The tip lever is 0.71 L instead of 0.5 L.
    - The extra wheel leaves freedom to shift torque to the loaded wheels.
- *Sim, slip-free acceleration* (`studies/wheel_count.py`, μ 0.95, jerk-limited ramps):

| Base | Worst direction | Mean over directions | Tip-over (model, worst) |
| --- | --- | --- | --- |
| Kiwi at r 150 (compact) | 2.7 m/s² | 3.7 m/s² | 4.0 m/s² |
| Kiwi at r 180 (v2 size) | 3.0 m/s² | 4.8 m/s² | 4.9 m/s² |
| X4 at r 150, rigid | 4.2 m/s² | 5.9 m/s² | 6.3 m/s² |
| X4 at r 150, rigid, load-aware torque allocation | 4.3 m/s² | 6.2 m/s² | 6.3 m/s² |
| **X4 at r 150, front rocker (v3)** | **3.0 m/s²** | **4.8 m/s²** | 6.3 m/s² |

- *Four wheels are statically indeterminate.*
  - With stiff rubber rollers (30 N/mm (est)), one wheel 1 mm proud of the others (floor dip or build error) takes about half the load off the two wheels beside it (13 → 6 N). The rigid X4's diagonal limit drops from 4.2 to 2.0 m/s², and at 2 mm a wheel lifts off (1.0 m/s²).
  - A rocker keeps 3.0–3.5 m/s² whatever the unevenness. 8 N/mm sprung pods give 3.6 flat and 2.5 m/s² at 2 mm.
  - So the rocker is proposed; rigid only if the real floor and chassis are flat to well under 0.5 mm.
- *Load-aware allocation is optional.* With plain speed loops, the unloaded wheel micro-slips and the other three keep tracking, so the base redistributes force by itself. Explicit allocation adds only ~5%.
- Costs: one more AK40-10 ($136) and pod, +0.3 kg, the rocker. A bonus: four-wheel odometry is over-determined, so a slipping wheel can be detected.
- Every option clears the 2.5 m/s² firmware cap, the compact kiwi only barely (2.7 m/s² worst).

**2. Tilting funnel + launcher: yes. The funnel is half as deep, provided the tilt is partial and it "dumps".**

- Required tosses (2–4 m, apex 1–2 m) arrive 12–36° from vertical at 5–6.7 m/s.
- *Funnel rig* (`studies/funnel_tilt.py`): 6.4 m/s arrivals at 12°, 24° and 36°, landing 0–10 cm off-center, 75 trials per configuration.

| Funnel | Caught |
| --- | --- |
| v2 35° cone, upright | 93% (84% at 36°, 79% at 10 cm off-center) |
| 50° upright | 92% (76% at 36°) |
| 50° fully aligned | 55%. Centered balls hit the gate at full speed and bounce out; others stall on the far wall, which is nearly flat when the funnel is tilted |
| 50° fully aligned + dump | 100% |
| **55° (86 mm deep), tilted to 60% of the arrival angle + dump** | **100%** |
| 60° (71 mm), 65° (57 mm) + dump | 96%, 95% |

- How it works:
  - The robot faces the incoming ball and tilts to ~60% of its arrival angle, so the ball still glances off one wall and loses energy there.
  - 30 ms after the ball enters, the funnel swings upright, so a ball resting on the shallow far wall rolls into the throat.
  - The pocket floor must absorb a straight-in 6 m/s impact. The sim needed the gate pad's restitution at 0.05 (est, e.g. 10 mm slow-recovery foam) or a tilt gain of 0.3.
  - Spring-loaded one-way flaps at the throat were tried and rejected: they jam the ball and can't close before the rebound.
- *Launcher on the same joint.* Calibrated at 4 tilts × 9 rpm: 72/72 shots fed, exit speed as v2 (7.8 m/s at 3000 rpm), elevation within ~1° of geometric above 2000 rpm. The return lob at 50° lands a median 13–14 cm from the thrower's hands.
- *In the full game with perfect perception* (positioning errors ≤ 4.5 cm), an upright 55° funnel also catches 30/30. The tilt's margin shows at the larger errors and steeper arrivals that real perception will produce.
- *Tilt actuator.* It holds the module at ~0.4 N·m and saturates a 3 N·m servo during fast swings: a 35 kg·cm-class servo or a geared motor (est).
- Height: plate-to-rim drops 28% (415 → 298 mm). The tilt itself costs ~2 cm of clearance under the module.

**3. Cameras: keep them within ~30–40 cm of the Jetson, fixed to the base.**

- MIPI CSI-2 over ribbon (FFC/FPC) is commonly specified to ~30 cm, and well-made cables reach ~50 cm ([The Imaging Source](https://www.theimagingsource.com/en-us/support/knowledge-base/mipi-csi-2-cameras-fpc-cable-length/)). 22-pin 0.5 mm Jetson cables are sold up to 40 cm ([Kurokesu](https://www.kurokesu.com/main/tag/ffc/)).
- Longer runs need a serializer: GMSL2, FPD-Link/V3Link, or Arducam's LAN extension (10 m). None of these lists the OV9782 ([Arducam LAN kit](https://www.arducam.com/blog/?p=95847), [Arducam GMSL](https://docs.arducam.com/GMSL-Camera-Solution/GMSL-Camera-for-NVIDIA/Introduction/)).
- The OV9782 at 1280×800 / 120 fps RAW10 is ~1.2 Gbit/s over 2 lanes, so cable quality matters.
- v2's rim cameras are ~1 m from the Jetson, and on a tilting module the cable would also flex through the pivot and the extrinsics would change with the tilt.
- *Visibility* (`studies/camera_view.py`; 15 tosses per mount, ray-cast at 120 fps against the robot, funnel net and thrower):

| Mount | Cable to Jetson | Stereo, first 0.5 s (the fit) | Stereo, whole flight | Last stereo frame before the rim |
| --- | --- | --- | --- | --- |
| **Payload-plate front, fixed** | ~40 cm | 100% | 56% | ~0.5 s |
| Side posts at rim height, fixed (0.40 m baseline) | ~70 cm | 100% | 74% | ~0.3 s |
| Funnel rim (v2), tilts | ~1 m | 92% (worst 70%) | 36% | ~0.75 s |

- Proposed: plate front, with the Jetson's CSI connectors facing forward to stay at ~30 cm. Side posts track longer but need a serializer or USB3 cameras.

**4. Tip-over: yes for a kiwi, no for the X4.**

- At the ramped 2.5 m/s² limit, nothing tips.
- *Panic stop:* at 1.4 m/s, every wheel commanded to zero speed at once (a naive fault stop: traction-limited braking):

| Base | μ 0.95 | μ 1.2 |
| --- | --- | --- |
| Compact kiwi | falls over | falls over |
| v2-size kiwi | rocks 23° | falls over |
| X4 rigid | ≤ 6° | ≤ 10° |
| X4 + rocker | ≤ 11° | ≤ 13° |

- The kiwi is worst braking while driving forward. That is acceleration toward its rear wheel, where the loaded front pair keeps gripping while the rear wheel unloads.
- The X4's quasi-static tip limit (6.3 m/s²) is above its slip limit in every direction.
- This applies to v2 now (sections 3 and 14): ramped stops in firmware, a coasting e-stop, and the AK40-10's comm-loss behavior checked.

**What v3 does not cover yet:** CAD for the X4 base, notched deck, rocker and tilting module with the plunger; perception noise (the spec sweep, section 12); the tilt servo, plunger linkage and pocket pad as real parts; the explicit-roller wheel model on four wheels.

## 21. v5: no limits, a detailed vision model, and a hopper (exploration)

The sim is `simulation/v5 unlimited` ([README](simulation/v5%20unlimited/README.md)). It is the v3b robot (section 20). Fernando asked three questions: what can it do with every speed and safety limit removed; can a more detailed camera model (color blob + known size, besides stereo) show how sensitive catching is to the cameras; and can the compact module work as a multi-ball hopper that shoots while driving. **Removing the limits is an envelope study, not a proposal:** v3b's firmware limits stay the design, and `limits="firmware"` reproduces them.

![v5 hopper options](simulation/v5%20unlimited/renders/v5_hopper.png)

### 1. The physical envelope (`studies/envelope.py`, μ 0.95)

Only physics is left: motor back-EMF at the live battery voltage, the driver's peak current, roller traction, tip-over and the 40 A BMS.

| | v3b firmware | Physics only |
| --- | --- | --- |
| Top speed | 1.4 m/s (2.0 adult) | **3.44 m/s** along the X's sides, 2.45 m/s on a diagonal (full charge); 2.9 / 2.1 at 20% charge |
| Acceleration | 3.0–4.5 m/s² (85% of v3's measurement, capped) | **3.4–5.0 m/s²** (worst front-right diagonal, best forward) |
| 0.3 / 0.6 / 1.0 / 2.5 m move, settled within 1 cm | 0.93 / 1.29 / 1.58 / 2.69 s | **0.64 / 0.89 / 1.13 / 1.79 s** (no jerk limit, no overshoot) |
| Turn 180° | 1.08 s at v3b's 15 rad/s² (slower still with its 4 rad/s cap) | 0.73 s (100 rad/s², ~16 rad/s, motor-limited) |
| Peak bus current | 2.6 A | 10.7 A (BMS trips at 40 A) |

- **Traction was already nearly used.** The real slip-free limit is only ~10% above v3b's table, which was 85% of it. v3's "9.9 m/s² forward" was a test artifact: its 1 m/s, 40 m/s³ ramp can't exceed 6.3 m/s².
- **The gains are speed and jerk.** No jerk limit was fastest, with no overshoot.
- **It can't tip.** Stopping from 3.4 m/s: the ramped stop takes 1.25 m with 0.2° of tilt. A naive all-wheels-zero stop takes 0.7–1.1 m with at most 8.8° (μ 1.2), and no fall.
- **The ball stays in the pocket** when braking from 2.5 m/s at every rate the floor allows (4.3–5.4 m/s²).

**Catch game** (`studies/catch_sweep.py`, the v3 grid, 216 identical throws per robot, vision model):

| Robot | Caught | Misses: not in place / bounced out |
| --- | --- | --- |
| Firmware, stop, 1.4 m/s | 81% | 36 / 5 |
| Firmware, moving, 2 m/s, 20 cm gate (v3 adult mode) | 84% | 27 / 8 |
| Physics, stop | 88% | 18 / 8 |
| **Physics, moving** (only rule: don't hit the netting or the thrower) | **89%** | 15 / 9 |
| Physics, moving, exact ball state after 0.25 s | 90% | 21 / 0 |
| Physics, moving, fast vision (200 fps, 20 ms latency, 90° lenses) | **92%** | 8 / 10 |

- No limits: 1 m-apex throws landing 1 m away go from 0% to 83–100%. Moving catches reach 2.9 m/s.
- What's left is line drives landing 1 m away (0–33%: physics) and bounce-outs, which become the largest miss category with fast vision. The next gains are in the catch module, not the drive.

**Launcher with no rpm cap** (`studies/launcher_max.py`, 40° launch). Range is to 0.15 m height with drag.

| rpm | 3,000 | 5,000 | 7,500 | 10,000 | 20,000 |
| --- | --- | --- | --- | --- | --- |
| Exit speed (m/s) | 7.2 | 11.2 | 14.6 | 18.1 | 32 |
| Range (m) | 4.5 | 8.6 | 12.0 | 15.0 | 24 |
| Spin-up from rest | 0.14 s | 0.23 s | 0.35 s | 0.47 s | 1.1 s |

- Drag makes range grow slowly. The 22 g ball's terminal speed is ~13 m/s.
- Above ~10,000 rpm the contact model is erratic, so treat those rows as indicative.
- Printed flywheel stress (est) is low (tire 1.6 MPa at 10,000 rpm), but VERIFY with a guarded spin test.

### 2. The vision model (`vision.py`, `studies/vision_sweep.py`)

Every frame the ball is rasterized on each camera's pixel grid: motion blur over the exposure, optional rolling shutter, 0.4 px edge noise, and occlusion ray-cast per pixel (the net shows half the ball through). A blob detector reports centroid and minor-axis radius. An EKF uses those pixels directly, with the exact projection of a sphere, so stereo, known-size ranging or both come from the same update.

The estimator doesn't know about calibration error (0.02°), stereo sync offset, the real ball size (±1%) or odometry drift (1%/m). Detector precision on its own is 0.28/√r px (r = blob radius in px): 0.04 px at 0.5 m, 0.13 px at 4 m.

Results: one change at a time, 144 identical tosses each, moving catches, physics limits. Catch rates carry about ±2.5% noise, so the landing error is the steadier signal.

| Change | Caught | Landing error 0.3 s out, median / 90% |
| --- | --- | --- |
| **Baseline**: OV9782 1280×800, 120 fps, 1 ms, 50 ms latency, stereo + size | 89% | 1.7 / 3.7 cm |
| v3's Gaussian model | 87% | 4.2 / 8.9 cm (2.5× too pessimistic) |
| Stereo only / one camera with size ranging only | 92% / 89% | 1.7 / 4.2, 2.1 / 4.5 cm |
| 60 / 200 fps | 90 / 88% | 1.6 / 1.7 cm (first estimate 0.18 / 0.12 s) |
| Latency 20 / 100 / 150 ms | 92 / 85 / **82%** | ~1.8 cm |
| 640×400 / 1920×1200 | 86 / 88% | 2.0 / 1.8 cm |
| Baseline 10 / 30 cm | 85 / 90% | 1.9 / 1.6 cm |
| FOV 90° / 50° | 91 / 88% | **1.3** / 2.5 cm |
| Exposure 0.5 / 4 / 8 ms | 89 / 90 / 91% | 1.7–1.9 cm |
| Calibration 0.2° / **0.5°** | 90 / **58%** | 1.6 / 2.5 cm (90%: 5.7 / 10.3) |
| Stereo sync 2 / **5 ms** | 90 / 86% | 2.4 / 4.0 cm |
| **Rolling shutter 20 ms** (row-stamped) | **72%** | 6.8 / 28.5 cm |
| Odometry drift 5%/m | 90% | 3.3 cm |
| 20% dropped frames / 5% clutter blobs | 83 / 88% | 2.0 / 1.6 cm |
| **Ball spin 10 rad/s** (tosses have none by default) | 92% | **2.9** / 7.5 cm |
| Fast pipeline: 200 fps + 20 ms + FOV 90° | **95%** | 1.3 / 3.1 cm |

- **What matters:** latency, calibration, a global shutter, hardware sync, a wide FOV, and real-world ball spin.
- **What barely matters:** resolution, exposure up to 8 ms, stereo baseline, the detector type, and the net.
- **Known-size ranging** adds little to a working stereo pair. But one camera alone with size ranging is almost as good (2.1 cm), so losing one camera degrades gracefully.
- The filter's σ is ~5–8× its actual error, which delays moving-catch commitment. A lower process noise helps spin-free tosses but is overconfident with spin, so it stays at 0.5 m/s² until throws are filmed.

### 3. Multi-ball hopper (`studies/hopper_rig.py`, `shoot_game.py`, `studies/shoot_sweep.py`)

- **The bare funnel holds 9–10 balls but fires only 2.** The pocket ball and the one standing in the throat feed fine; nothing lands behind the plunger. Then two balls arch over the Ø84 throat (two 74 mm balls can't pass at once) and feeding stops. That happens on a slick surface too, so it's geometry.
- **Agitation** (tilt flicks, base yaw shimmy) only partly breaks the arch: 1–2 s per ball, often stalls. A wire sweeper arm on the net feeds 6/6 upright but throws balls out when tilted.
- **Proposed: a single-file magazine tube** dropped into the funnel for hopper mode (12-sided printed tube, Ø82 inside, 33 cm above the throat, holds 6).
  - Stationary bursts: **6/6 fired at every launch tilt (30–60°), one ball per 0.36 s (2.8/s)**, 2–3% exit-speed spread, no double feeds. These were measured with a 30 cm tube; at 30 cm the 6th ball sticks out 2 cm and hard turns threw it out, hence 33 cm.
  - A 0.8 m/s plunger gives 4/s (3.5% spread); 1.2 m/s scatters 15%.
  - A 45 cm, 8-ball tube double-fed at the 60° launch.
- **The weak point is the open pocket while driving.** In hopper mode the flywheels run, so a waiting ball rolled forward by a hard turn or braking gets fired unaimed.
  - Braking the flywheels while turning, plus a settle hold, still leaves 19% misfeeds when stopping and turning between targets, and ~30% at 2 m/s.
  - A 4–6 mm retaining ridge was worse (balls don't seat at 60° launch).
  - Next step: a sprung retaining gate at the pocket exit that the plunger pushes open.
- **Shooting at targets 2–6 m away** (8 random layouts, range bias learned as it goes): standing 23 cm median miss (90%: 57 cm); on the move at 0.5–1 m/s 34–37 cm; at 2 m/s 47 cm. Range error dominates (lateral 6–15 cm): range goes with exit speed squared, so the feed's 2–3% speed spread alone gives ~5% range error.

### 4. Expected camera performance, thrower distance and base type

Fernando's questions: what should the real stereo pair do; does a farther thrower give the robot more time; what limits the base, and is another base type better as the long-term platform. Studies: `camera_estimate.py`, `thrower_distance.py`, `base_types.py`. All use the v5 vision model. Each catch rate is from 216 throws and carries about ±2.5% of noise.

**Why the cameras have unknowns.** The OV9782's sensor specs are published: a color global shutter, 1280×800 at 120 fps (VGA at 180), 3 µm pixels and a frame-sync input ([OmniVision](https://www.ovt.com/sensors/OV9782), [Luxonis](https://docs.luxonis.com/projects/hardware/en/latest/pages/articles/sensors/ov9782/)). The unknowns are all at the system level:

- **Latency** depends on the Jetson capture path, not on the sensor.
- **Frame rate** depends on the driver. NVIDIA ships no OV9782 driver; the vendors' drivers are built per JetPack, and some Arducam module listings give only 60 fps at full resolution.
- **Calibration** depends on the mount and how often it's redone.
- **Segmentation noise** depends on lighting and background.
- **Spin** is a property of the throws, not the cameras.

**Latency budget at 120 fps** (camera capture to the filter, built from published measurements):

| Stage | Time |
| --- | --- |
| Exposure midpoint, then the global-shutter readout | ~1 ms + ~7 ms |
| MIPI → VI → memory | ~1 ms |
| ISP and Argus buffering | +2–3 frames: 17–25 ms. NVIDIA calls 2–3 frames expected; Xavier users measure 48–60 ms from the capture stamp to the app ([forum](https://forums.developer.nvidia.com/t/latency-analysis-with-libargus/203673)) |
| *or* raw V4L2 with `low_latency_mode`, no ISP | measured 15 ms glass-to-software at 120 fps (OV2311 on a Nano, [forum](https://forums.developer.nvidia.com/t/latency-reduction-for-csi-mipi-camera-on-jetson-nano/298282)); the color threshold then runs on the raw Bayer ROI |
| Blob detection on the ROI, stereo pairing, EKF | 2–4 ms |
| Linux jitter, CAN to the MCU | 2–5 ms |
| **Total** | **raw V4L2 ~22–26 ms; Argus ~40–50 ms; GStreamer appsink + OpenCV conversions 60–90 ms** |

For comparison, RidgeRun measures 36–42 ms glass-to-glass, display included, on an AGX Orin with a raw path, and 73–116 ms with Argus on TX2/Xavier ([RidgeRun](https://www.ridgerun.com/post/glass-to-glass-assessment-of-the-holoscan-sensor-bridge-on-a-nvidia-agx-orin), [wiki](https://developer.ridgerun.com/wiki/index.php/Jetson_glass_to_glass_latency)).

**Scenarios** (each a consistent whole pipeline; spin 6 rad/s per axis (est) unless noted):

| Scenario | Physics limits, moving | Firmware adult mode | First estimate |
| --- | --- | --- | --- |
| v5 assumed baseline (50 ms, 0.02°, no spin) | 89% | 84% | 0.14 s |
| **Best:** raw V4L2, both sensors triggered from one MCU timer, metal/carbon stereo bar (24 ms, 0.04°) | 91% (93% with 90° lenses) | **90%** | 0.08 s |
| **Expected:** Argus, MCU trigger, printed bracket recalibrated per session (45 ms, 0.1°, 1% clutter, 5% drops) | **92%** | **86%** | 0.12 s |
| Expected, but the driver only offers 60 fps (60 ms) | 89% | | 0.18 s |
| Expected, but calibration drifted to 0.3° | 85% | | |
| Expected, but software sync (3 ms) | 92% | | |
| **Pessimistic:** GStreamer/OpenCV (80 ms), software sync, knocked bracket (0.3°), 10% drops, 5% clutter, 10 rad/s spin | **72%** | | 0.19 s |

- **The realistic band is 86–92%,** as good as v5 assumed, because 45 ms is a little better than its 50 ms. The difference between the best and expected rows is within noise for physics limits; with firmware limits the faster pipeline is worth about +4 points.
- **The cliffs are a drifting calibration and a slow, stacked pipeline,** not the sensor. Camera spec:
  - hardware-triggered sync from the MCU, which also gives hardware timestamps in the MCU clock;
  - a stiff stereo bar, not separate printed pads;
  - the raw V4L2 path if Argus measures above ~50 ms;
  - confirm the driver offers 1280×800 at 120 fps before buying.
- **Exposure:** 2–4 ms is fine (motion blur stretches only the long axis of the blob), and it gives more light indoors than the 1 ms assumed.

**Thrower distance: farther doesn't give more time.** Flight time is set by the apex: a 1 m apex flies 1.0 s from 1.5 m and from 6 m alike. What distance changes is the thrower's scatter, which grows with it, and the arrival angle: at a 1 m apex, 16° from vertical at 1.5 m and 45° at 6 m, beyond the 15° catch-tilt cap. Results with expected cameras, physics limits (firmware in brackets):

| Thrower distance | 1.5 m | 2 m | 3 m | 4 m | 5 m | 6 m |
| --- | --- | --- | --- | --- | --- | --- |
| Fixed landing offsets (0.3 / 0.6 / 1 m) | 92% (83) | 94% (83) | 92% (92) | 83% (78) | 81% (74) | 78% (68) |
| Human scatter, σ 7% × 5% of the distance (est) | 94% (99) | 96% (99) | **99% (96)** | 92% (92) | 79% (81) | 61% (69) |
| Median landing offset (human) | 11 cm | 14 cm | 22 cm | 30 cm | 34 cm | 45 cm |

- **Close catches aren't harder for lack of time.** With human-like scatter, 1.5–4 m all score 92–99%; the sweet spot is 2–3 m. Beyond 4 m both effects compound: the landing scatter is larger and the arrivals are flatter.
- **Proposed:** mark a throw line at ~2.5 m; for longer throws ask for high lobs (apex ≥ 2 m: 83–94% even at 5–6 m). The 5 × 4 m arena fits this.

**What limits the base: traction, then speed.**

- **Torque isn't the limit:** an AK40-10 can push ~85 N through a 96 mm wheel, but each wheel gets only ~12 N from the floor.
- **Omni and mecanum wheels can only push along one fixed direction.** In the worst direction two of the four wheels push, each at 45°: μg/2 before load transfer, whatever the wheel count. That gives 3.4–5.0 m/s².
- **Top speed** is back-EMF: a 2.43 m/s rim speed, 3.44 m/s along the X's sides.
- **Tipping** isn't a limit on the axle deck (9.9 m/s²).
- **With expected cameras the base causes only 15 of 216 misses** (`base_types.py`: 92% caught, 15 not in place, 2 bounced out).

Base types, compared on the same 5.3 kg robot with its CG at 95 mm and a ~150 mm footprint. Reach is a bang-bang move and stop, averaged over directions. The catch game uses the sim where it can represent the base:

| Base | Worst / best accel | Reach 0.3 / 0.6 / 1 m (mean, s) | Catch game | Notes |
| --- | --- | --- | --- | --- |
| **X4 omni + rocker (v3b)** | 3.4 / 5.0 m/s² | 0.55 / 0.77 / 1.00 | **92%** | Baseline |
| Kiwi omni (3 wheels) | 3.6 / ~6 | 0.51 / 0.72 / 0.93 | (not run) | Same traction, but tips at 6.9 m/s² and falls in a panic stop on a high deck (section 20); one wheel fewer |
| Mecanum | same as X4 × roller μ | 0.64 / 0.90 / 1.16 at μ 0.7 | **84%** at μ 0.7 | Kinematically the X4 (rollers at 45°). PU/hard rollers grip less, vibrate more and handle carpet worse. Only gains: rectangular packaging, load capacity |
| **Swerve** (4 steered modules, rubber μ 1.0) | ~8 every direction (tip-limited), 2.6 m/s, ~50 ms to steer | 0.43 / 0.59 / 0.75 | 90% at 8 m/s²; **94% at 6 m/s²** (96% with best cameras) | At 8 m/s² the base rocks 5–10° into the catch and bounce-outs rise (17 vs 9). Use it at ~6 m/s², or widen or lower it |
| Differential drive (2 rubber wheels + casters) | ~6.7 forward, 0 sideways | 0.85 / 1.02 / 1.20 (turn there and back); 0.64 / 0.81 / 0.99 if it doesn't turn back | (not run: reduced-order only) | Fastest straight ahead (0.42 s for 0.3 m), but a sideways landing costs two ~0.3 s turns. It must face the ball to tilt the funnel and face the thrower to shoot back |

- **For catching, the base is not where the misses are now.** Swerve removes most reach misses (1 m landings 79% → 90%). The rest are bounce-outs, which come from the catch module.
- **As the long-term platform, swerve is the ultimate base:**
  - full μg in every direction;
  - rubber tires, so it handles carpet, thresholds and outdoors;
  - odometry with no roller slip, and stiff lateral resistance for an arm.
- **What swerve costs:** 4 steering actuators with encoders, more mass and more firmware. Steering ±90° plus drive reversal avoids slip rings.
- **Mecanum and diff drive are not upgrades** for this robot. Kiwi saves a motor but tips sooner.
- **Proposed:** keep the X4 omni for the catch robot now. If the general-platform goal wins (arm, outdoors, carpet), a 4-module swerve with a lower CG or a wider (~r 200 mm) footprint is the step up; it is worth +2–4 points to the catch game.
- **Sim caveat:** "swerve-like" is the X4 model with tire grip, a flat 8 or 6 m/s² limit and +50 ms latency for steering. Real module dynamics and steering-motor sizing are not modeled.
- **One event to know:** in one missed line drive the robot, chasing at 1.9 m/s, drove over the ball on the floor and rocked 40°. Ball-under-wheel is a real hazard for a fast base; the bumper skirt should push balls, not ride over them.

**What v5 does not cover:** CAD for the tube or a pocket gate; a measured spin model for thrown balls; the tilt servo's real speed under a full tube; flywheel structural limits beyond the estimate.

## 22. v6: shallow throws from 6-7 m, an active feed, multi-ball (proposed)

The sim is `simulation/v6 shallow throw` ([README](simulation/v6%20shallow%20throw/README.md)). It is the v3b robot of v5 (X4 omni base, axle-plane deck, tilting module) with the **expected camera pipeline** (Argus, MCU-triggered pair, printed bracket recalibrated per session, 6 rad/s ball spin; section 21.4). Fernando asked for three things:

- catch more of the shallow throws from a thrower 6-7 m away, where the tilting funnel should matter;
- no misfires: a motor or servo should actively pass the ball to the launcher, not gravity or an open pocket;
- ideally a funnel and launcher that can fire several balls without clogging. Tennis-ball machines (a rotating plate of one-ball holes) were suggested as the model.

**Everything in this section is Proposed.** Catch rates come from 216 throws per robot (about ±1.5 points at 95%).

![v6 module](simulation/v6%20shallow%20throw/renders/v6_module.png)

### 1. Catch rate: 57% -> 96% for 6-7 m shallow throws

**Throws** (`studies/shallow_sweep.py`): thrower at 6, 6.5 and 7 m; apex 0.75, 1, 1.5 or 2 m above a 1.0 m release; human-like landing scatter of 7% of the distance along the throw and 5% across (v5's estimate). They arrive 27-71 deg from vertical (median 45) at a median 7.3 m/s, after 1.1 s, landing a median 46 cm from the waiting spot.

| Robot (each row adds to the one above) | Firmware adult mode | Physics limits | Misses (firmware): not in place / bounced out |
| --- | --- | --- | --- |
| v5 robot | 57% | 60% | 59 / 33 |
| v5 robot, exact ball state after 0.12 s (diagnostic) | 62% | | 0 / 82 |
| **v6 catch module**: roller feed, padded pocket, catch tilt up to 35 deg | 75% | 78% | 48 / 5 |
| v6 module, exact ball state (diagnostic) | 100% | | 0 / 1 |
| **+ honest track start** (perception software) | **96%** | **99%** | 4 / 5 |
| + landing prior (planner, optional) | 97% | 97% | 2 / 4 |

The two diagnostic rows separate the problems: with exact ball data the v5 robot is always in place but a third of the balls bounce out (the catch module), and with the cameras it is also often out of place (perception).

**Why balls bounced out** (`studies/catch_rig.py`, a static robot, 6.5 and 8 m/s arrivals 30-60 deg from vertical, up to 8 cm off-center):

- A flat, fast ball goes through the throat almost at full speed and hits the pocket's bare PETG back and side walls, which bounce it at the ball's own restitution (0.45). Then it climbs out over the rim. The net itself is fine.
- v5 capped the catch tilt at 15 deg because the plunger pocket's floor tips forward above the 30 deg launch angle and the ball rolls into the flywheels.

| Catch module (rig) | Caught |
| --- | --- |
| v5 (plunger, bare walls, tilt cap 15) | 80% |
| v5, tilt cap 25 | 88% |
| v5 + padded pocket walls | 89% |
| Roller feed, bare walls, tilt cap 35 | 89% |
| **v6: roller feed + padded pocket + tilt cap 35** (60% of the arrival angle) | **99%** |
| v6 at 80% of the arrival angle, cap 40 | 100% |
| v6 at 40% | 88% |
| v6 with a slack cloth net (lumped model) | 92% |
| v6 with the tennis-style indexer (see 3) | 20% |

- **Padding plus tilt together** do it: padding alone or tilt alone each get ~89%.
- **A slack cloth net didn't help in the sim**, but the model is crude: whole panels move outward on spring-dampers, including at the throat collar, where real cloth is sewn tight, so the ball catches the pocket wall's edge. Keep the net taut and grippy near the throat; a slack upper net would need a real cloth model (MuJoCo flex) or a drop test.
- **Ball:** the 74 mm foam ball is kept. With the pocket padded the ball's own bounce matters little, so a softer "practice dodgeball" foam is optional. Its mass and drag change the launcher calibration and the predictor, so measure it first.

**Why the robot was out of place: the stereo track started overconfident.** A track starts from two frames 8 ms apart. v5 gave it a fixed ±5 cm and ±2 m/s. That is right at 2 m, but at 7 m the per-frame stereo depth noise is ~9 cm, so the first velocity is ~15 m/s off along the line of sight. The filter took half the flight to recover, with estimates ~10 sigma out (one landing estimate 16 m short), and the robot drove off after them. v6 starts the track with the covariance the stereo geometry actually gives: depth error growing with range squared, size ranging, and the velocity from two such positions. Same cameras, a software fix. It also helps closer throws (below).

**The landing prior** fuses the landing estimate with "throws are aimed at me" (40 cm sigma, which on hardware would be learned from the thrower's logged landings). It is within noise here, so it stays off by default.

**Close throws don't get worse; they get better** (v5's `thrower_distance.py` on the v6 robot):

| Thrower distance, human-like scatter | 1.5 m | 2 m | 3 m | 4 m | 5 m | 6 m |
| --- | --- | --- | --- | --- | --- | --- |
| v5, physics (firmware) | 94% (99) | 96% (99) | 99% (96) | 92% (92) | 79% (81) | 61% (69) |
| **v6, physics (firmware)** | **100% (100)** | **100% (100)** | **100% (100)** | **100% (100)** | **100% (99)** | **100% (99)** |

- With fixed landing offsets the remaining misses are physics: line drives (apex 0.5 m, 61-83%) and 1 m landing offsets (62-88%).
- **v5's advice to ask for high lobs beyond 4 m is no longer needed.** The throw line can move out to 6-7 m.

### 2. The trigger: roller feed (feed wheels as gate and active pass)

The plunger is replaced by two small TPU feed wheels (Ø40 × 12 mm, est), one each side just ahead of the resting ball, axes perpendicular to the pocket floor (small flywheels). Each runs on a worm gearmotor (N20 class, ~380 rpm, 0.3 N·m, est) on a sprung mount preloaded onto a stop (6 N, est). The plunger's rest position becomes a fixed padded back stop, and the flywheel nip moves 28 mm forward to clear the feed wheels.

- **Stopped**, a worm can't be back-driven, so the wheels close the pocket: no ball reaches the flywheels at any tilt, standing or driving. The flywheels no longer need to brake between shots.
- **Turning**, they pull the ball in and pass it to the flywheels at their surface speed (0.8 m/s), so every shot starts the same way.
- **One ball per trigger**, in firmware: stop the feed wheels once the ball is 30 mm past the break-beam, so the next ball, which drops in ~0.1 s later, waits for its own trigger. Feeding below ~0.4 m/s stalls on the steepest launch.
- **The catch tilt can go past the launch angle** (cap 35 deg; the tilt joint now ranges −30…+40 deg).

| | v5 plunger | v6 roller feed |
| --- | --- | --- |
| Single shots (bench) | 7.7 m/s at 3000 rpm, ~2 rad/s spin | 7.5 m/s at 3000 rpm (same ~0.70 transfer), ~6-8 rad/s spin, repeat shots identical; `launcher_cal_roller.json` |
| Return throws to a thrower 6.5 m away (`return_throws.py`) | median 27 cm from the hands | **median 3 cm** (90%: 9 cm) |
| Hopper shooting, misfires (a ball fired without a trigger) standing / 1 m/s / 2 m/s | 9 / 5 / 7 of 48 | **0 / 0 / 0** |
| Hopper shooting, median miss standing / 1 m/s / 2 m/s | 23 / 34 / 37 cm | **13 / 22 / 26 cm** |
| 6 shots at 6 targets | 9.5 s | 5.0 s |
| Magazine burst, 6 balls | 6/6, 0.36 s apart, ±2-6% speed | 6/6 at every tilt, **0.19-0.23 s apart**, ±1.7-5%, no double feeds |

- **Feed-wheel mounts must sit hard on their stops.** In the sim, a soft stop let the 6 N preload push them 3-4 mm inward. The wheels then caught a dropping ball 21 mm above the floor, and return throws scattered by half a metre. On hardware: a rigid stop, and the preload set so the wheels just touch a seated ball.
- **Rejected tweaks:** a reverse "seat" pulse before feeding made returns worse (44 cm); a settle wait before firing isn't needed.
- **The sim's launcher is very repeatable**, so 3 cm is an upper bound on return accuracy. Real tire wear, ball-to-ball spread and spin variation will add to it, and the logged-landing bias learning (section 9) still applies.
- **Cost:** two N20 worm gearmotors, two printed TPU wheels and an H-bridge channel each, in place of the plunger servo. Module mass about the same (robot 5.32 kg, CG 9.7 cm, rim 389 mm).

### 3. Several balls: tube, stirrers, and the tennis-machine indexer

`studies/hopper_rig.py`, 6 balls (10 for the indexer), burst fire at 30, 45 and 60 deg launch:

| Multi-ball option | Fired | Notes |
| --- | --- | --- |
| Bare funnel | 2/6 at every tilt | Three balls dome over the Ø84 throat |
| Stirrer arm next to the throat, vibration motor, slick lower net | 2-6/6, depends on tilt | No setting works at every tilt |
| **Magazine tube** (v5's single-file insert) | **6/6 at every tilt** | 0.2 s per ball, holds 6 through tilts and hard driving |
| **Tennis-style indexer** (Fernando's idea) | 2/6 with the net's 12 mm rolling resistance (est); 6/6, 6/6, 2/4 with 4 mm; 10 balls: 3/10, 10/10, 2/5 | No jams, no double feeds, ±1-3% speed. Loose balls spill when the module tilts to 60 deg launch |

**The indexer as modeled:**

- The funnel ends on a Ø206 mm wheel with 4 one-ball pockets (78 mm deep: one ball fits, two don't), running inside a fixed padded wall on a padded base plate.
- The plate has one exit hole at the front, over the pocket tube; the pocket, feed wheels and flywheels move 60 mm forward to sit under it. A fixed hood over the exit station stops the hopper refilling that pocket.
- Balls in the hopper rest on the moving wheel, so no dome over a fixed hole can stand, and each empty pocket takes a ball as it passes under them.
- **Its firmware matters:**
  - index one pocket only when the launch pocket is empty;
  - **dwell ~0.4 s over the exit** (stepping on while a ball is still falling shears it in the exit hole and jams the wheel);
  - back off and retry on a stall;
  - call the hopper empty after two turns with nothing dropping in.
- The indexer's surfaces need a slick skin over the foam (μ ~0.2, est): at the pad's 0.8 a ball dragged along the plate and the wall jams the wheel.

**But it catches only 20%** in the rig, against 99% for the v6 pocket:

- Balls land on top of the wheel and bounce off the hub and paddle tops. A pocket only accepts a ball centred within about ±5 mm radially.
- It also costs height: rim 389 → 455 mm, pivot +28 mm, CG 9.7 → 11.2 cm, tip-over 9.4 → 7.6 m/s² (still above the traction limit).

Tennis machines never catch; their balls trickle in.

**So, proposed:**

- Keep the v6 catch module (throat → padded pocket → feed wheels) for play-catch.
- Multi-ball stays a hopper mode: the magazine tube insert works now.
- An indexer is worth building only if the catch path bypasses it. Two options to try next:
  - a separate indexer magazine behind the funnel that feeds the same launch pocket through a side port;
  - an indexer that replaces the funnel for shooting sessions (a swap-in hopper bin).
- In either case, measure the real net's rolling resistance first: it decides whether balls roll into the pockets at all.

### 4. The feed-wheel to flywheel handoff (`studies/feed_handoff.py`)

Fernando asked how the worm-driven feed wheels hand the ball to the flywheels without holding it while the flywheels rip it away, and whether there is a ramp or a gravity-fed stretch between them.

- **The worm is only the feed motor's gearbox.** The ball is gripped first by the two TPU feed wheels and then by the two flywheels. All four are side wheels in the plane of the launch line.
- **The two grips don't overlap.** With the 74 mm ball the feed wheels push for the first 42 mm of travel and the flywheels grab at 52 mm (the nip is at 78 mm). The ball can't be held by one pair while the other pulls, so the self-locking worm is never back-driven.
- **No gravity feed.** Across the ~10 mm gap nothing drives the ball: it coasts uphill along the 30-60° launch line on its own 0.8 m/s. That sets a minimum feed speed, √(2·g·sin(launch)·gap) ≈ 0.45 m/s at the 60° launch, which is why 0.3 m/s stalled there.

| Ball, on hardware fitted to 74 mm | Feed wheels push | Flywheels grip | Unpowered gap | Exit at 3000 rpm |
| --- | --- | --- | --- | --- |
| 72 mm | not touched: **no feed** | | | |
| 74 mm | 0-42 mm | 52-65 mm | 10 mm | 7.6 m/s |
| 76 mm | 1-44 mm | 49-61 mm | 5 mm | 7.1 m/s |

**The open problem is ball tolerance.** If the wheels are placed to grip a 72 mm ball at rest (`feed_fit_diameter`), every ball from 72 to 76 mm feeds and the gap stays 3-13 mm. But then a caught ball, dropping in from above, perches on the tire edges about 15 mm above the floor. The pinch plus the back stop hold it there by friction, and return throws scatter to 45 cm. A reverse "seat" pulse doesn't free it.

So a fixed side-wheel pocket either pinches the resting ball and perches it, or doesn't pinch and misses undersized balls. The perch height is about half the tire width plus √(2·r·intrusion).

**Round 2: the chute pocket (`feed="chute"`, now the v6 default).** Fernando chose the sloped floor:

- The pocket floor descends toward the feed wheels at −40° in the module frame. At every launch tilt it still slopes 10-40° down toward them, and during catches more steeply.
- The ball rolls into the nip of the stopped feed wheels and rests there under its own weight, so any ball wider than the wheels' 66 mm gap touches both. Nothing pinches it at rest.
- Turning, the wheels push it into a J-bend (40 mm radius) that turns it up the launch line into the flywheels: still an active pass.
- The rim stays at 387 mm (the launch line now starts lower); the pivot rises 27 mm and the CG to 9.9 cm.

Three things failed first and shaped it:

- **The ball must coast up the J.** After the feed wheels let go, the ball climbs the bend unpowered. At 0.8 m/s a 72 mm ball stalled at the 60° launch, so the chute feeds at **1.6 m/s** (≈760 rpm on Ø40 wheels; VERIFY a worm gearmotor at that speed, or use Ø60 wheels).
- **The J must be slick.** On the catch pad (μ 0.8) a ball sliding round the bend lost 1 m/s and stalled. The next fed ball then pushed it into the flywheels, a double feed. The J and launch run are smooth printed PETG (μ 0.25, est); caught balls never reach them.
- **Stop the feed by a beam at the feed-wheel exit.** No time or distance after the pocket beam fits every ball size: too early brakes the ball in the wheels, too late feeds the next one. A second IR beam where the ball leaves the wheels stops them once it has passed.

Results (`feed_tolerance.py` and the studies above):

| | Roller pocket (round 1) | **Chute pocket (round 2)** |
| --- | --- | --- |
| Balls 72-76 mm, at 30 / 45 / 60° launch | 73 mm at 45° doesn't feed; 75-76 mm get 65-81 rad/s of spin and up to −6° of elevation | **all feed**; elevation within 0.2° across sizes, 18-23 rad/s spin |
| Exit speed by ball size at 3000 rpm | 7.0-7.9 m/s, erratic | 7.2-7.8 m/s, steady (±3.5%: calibrate per ball batch) |
| Feed-wheel and flywheel grips | never together (9-27 ms apart) | never together (51-69 ms apart) |
| Catch rig (fast, flat arrivals) | 99% | 99% |
| 6-7 m shallow throws, firmware adult / physics | 96 / 99% | 95 / 98% |
| Close throws 1.5-6 m, human scatter (physics) | 100% | 96-100% |
| Return throws at 6.5 m, median / 90% | 3 / 9 cm | 14 / 19 cm (a steady ~10 cm bias the learning removes slowly) |
| Magazine bursts, 6 balls (and 8, overfilled) | 6/6, 0.19-0.23 s, ±1.7-5% | **6/6 at every tilt, 0.18 s, ±0.5-1.6%, no double feeds**; 8 balls: the same |
| Hopper shooting misfires, stand / 1 / 2 m/s | 0 / 0 / 0 | 0 / 0 / 0 |
| Hopper shooting miss, stand / 1 / 2 m/s | 13 / 22 / 26 cm | 5 / 15 / 37 cm |
| Indoors, 8 ft: caught, ceiling hits, return miss | (roller) 90%, 0, 7 / 50 cm | 93%, 0, 13 / 23 cm |

- **The chute is the proposal.** It feeds every ball size the roller pocket missed, with no loss in catching or misfires and tighter bursts. Its return throws are ~10 cm less accurate in the sim (more, steadier spin).
- The bare funnel still domes (1/6) and the indexer was only modeled on the round-1 pocket.

### 5. Indoors: a one-storey ceiling (`studies/indoor.py`)

The question: in a standard one-storey room, can the robot still throw back 7 m?

- **The 50° return lob can't.** At 7 m it peaks at about 3.0 m.
- **A flatter return can.** In an 8 ft (2.44 m) room, 35-40° keeps the apex near 2.0-2.3 m. It needs 10.8-11.2 m/s, about 4,400-4,600 rpm, above v2's 10.1 m/s design maximum but inside the launcher's 5,000 rpm calibration.
- **The planner:** `GameConfig.ceiling` picks the steepest elevation from 50° down to 30° whose apex stays 0.25 m below the ceiling.
- **The sim room:** the ceiling is a surface the balls bounce off (`SimParams.ceiling_height`).
- **The thrower's lobs must clear it too:** an apex up to 1.25 m above the release at 8 ft, 1.5 m at 9 ft.

| Room, return planner | Caught | Returns hitting the ceiling | Return miss, median / 90% | Return elevation, speed |
| --- | --- | --- | --- | --- |
| 8 ft, **ceiling-aware** | 90% | **0 of 97** | **7 / 50 cm** | 32-50° (38° at 7 m), median 10.1 m/s, up to 5,000 rpm |
| 8 ft, fixed 50° | 91% | 96 of 97 | 282 / 406 cm | |
| 9 ft, **ceiling-aware** | 96% | **0 of 138** | **5 / 38 cm** | 35-50° (43° at 7 m), median 10.1 m/s |
| 9 ft, fixed 50° | 93% | 117 of 133 | 203 / 338 cm | |

- **Yes, 7 m works indoors.** 8 returns in 235 asked for more than the 5,000 rpm calibration covers, all at 6.5-7 m; within it, misses are 7 cm median at 8 ft and 5 cm at 9 ft.
- **Catches are a little lower in an 8 ft room** (90% vs 96%). The thrower's lobs must stay flat, and flat throws are the hard ones.
- **Cost:** the flat return arrives at the hands at about 6.5 m/s, 30-40° below horizontal. That is a firm throw for adults, not for kid mode, which should stay at shorter distances.
- **The flywheels now run near 4,500 rpm**, so the guarded spin test of the printed flywheels (section 18) becomes a prerequisite.

**What v6 does not cover:** CAD for the feed wheels, padded pocket, tube or indexer; a real cloth model of the net; ~~the feed motors' real speed, torque and self-locking~~ (round 3, section 23.1); a measured feed-wheel preload and stop; ~~a ball-size-tolerant pocket~~ (the chute, 22.4); launcher scatter beyond the sim's repeatability (round 3 randomizes the ball and tires, 23.3).

## 23. v6 round 3: real feed motors, viscoelastic contacts, randomized worlds, real floors, a motor study (proposed)

The sim is still `simulation/v6 shallow throw` ([README](simulation/v6%20shallow%20throw/README.md)); round 3 changes its defaults. Fernando asked for five things:

- a real motor for the chute's feed wheels; a worm shouldn't be needed if the motor controller holds them with current;
- viscoelastic contacts in place of the elastic ones, for more realistic bounce, grip and shot scatter;
- the studies run with random draws of the guessed parameters, giving catch-rate ranges and showing which guesses matter;
- the roller wheel model validated on four wheels, plus floor friction variation, thresholds and carpet edges;
- with all of that in place, a motor study: is every motor and controller right, are there cheaper ones, and are four AK40-10s needed?

**Everything in this section is Proposed.** Part numbers and specs below come from the makers' pages (October 2026); anything marked (est) is not on a datasheet.

### 1. Feed motors: a GM3506 gimbal motor holds the ball with current (`studies/feed_motor.py`)

The chute feeds at 1.6 m/s at the tire, ~764 rpm on the 40 mm wheels. Round 2 assumed an ideal velocity loop (0.3 N m that holds like a worm); the N20 worm assumed in round 1 turns at only 380 rpm. Each candidate is now modeled from its datasheet (`catalog.FEED_MOTORS`, `models.FeedDrive`):

- five datasheet points give R, kt, ke and the gearbox drag;
- the driver sets the rail and the current limit;
- the reflected rotor inertia is the joint's armature;
- the firmware runs a speed loop while feeding and, stopped, a position hold with the driver's current limit. That is Fernando's "hold current": nothing has to self-lock.

| Motor (driver) | Price per wheel | Torque left at 764 rpm | Feeds 72/74/76 mm at 30/45/60° launch | Caught balls pushed back into the nip | Back-driven by a fast catch (max) | 6-ball magazine burst, 30 / 60° |
| --- | --- | --- | --- | --- | --- | --- |
| Ideal (round 2) | | 0.19 N m | 9/9 | 0 | 0 | 6/6, 6/6 |
| N20 worm, 380 rpm (est) | ~$18 | 0 (too slow) | 9/9, but only half speed; exit ±4-5% | 0 | 1.4 mm | 6/6, **5/6** |
| Pololu #5210 30:1 micro HPCB 12 V (DRV8874) | $39 | 0.016 N m | **2/9** | 0 | 5.5 mm | **4/6, 5/6** |
| Pololu #4842 25D 9.7:1 HP 12 V (DRV8874) | $69 | 0.115 N m | 9/9 | 0 | 0.7 mm | 6/6, 6/6 |
| **Pololu #4842 25D 9.7:1 HP 12 V, brushed, on a RoboClaw 2x7A** (4 A limit) | **$97** (half a board) | **0.115 N m** | **9/9** | **0** | **0.8 mm** | **6/6 (±0.4%), 6/6 (±0.6%)** |
| iFlight GM2804 gimbal + AS5048A (SimpleFOC Mini) | $63 | 0.045 N m | 9/9 | 0 (deepest 58% of the way) | 8.7 mm | 6/6, 6/6 |
| **iFlight GM3506 gimbal + AS5048A (SimpleFOC Mini)** | **$69** | **0.079 N m** | **9/9** | **0** | **3.1 mm** | **6/6 (±0.5%), 6/6 (±0.8%)** |
| **LK-TECH MF4005-V2, integrated FOC driver + 18-bit encoder, CAN** (driver limits est) | **$100** | **0.12 N m (est)** | **9/9** | **0** | **4.5 mm** | **6/6 (±0.4%), 6/6 (±0.4%)** |

- **Proposed, either of two (Fernando prefers integrated devices and is happy with brushed):** the brushed **Pololu 25D + RoboClaw 2x7A** (~$194 for both wheels, datasheet motor, best hold), or the **LK-TECH MF4005-V2**.
- **The MF4005-V2:** Motor, FOC driver and 18-bit encoder are one unit; the µC sends speed or position setpoints over CAN, as it does to the AK40-10s. It performs like the GM3506 in every test (its torque and current limit are estimates from a reseller listing, so VERIFY on the bench). Rated 12 V: it shares the 12 V rail.
- **The GM3506, direct drive under FOC**, is the cheaper do-it-yourself route ($69). But the SimpleFOC Mini is only a 3-phase driver: the FOC loop and the encoder reading run on a µC (the STM32 or a small dedicated one), and the encoder magnet and calibration are set up by hand.
- **About the GM3506:** It feeds every ball size at every tilt, a fast flat catch back-drives the stopped wheels at most 3 mm at the tire, and bursts are the tightest of all (0.18 s apart, ±0.5-0.8%). The TPU tire goes on a printed hub on its hollow shaft; the Ø40 × ~25 mm motor fits the ~30 mm envelope the chute geometry assumed. No gearbox: no backlash, no wear, and the encoder reads the wheel directly.
- **Brushed works: the Pololu 25D holds best of all** (0.7-0.8 mm), feeds every ball and fires the tightest bursts with the GM3506 and MF4005. Brushless isn't needed: the feed wheels turn ~0.1 s per shot, so brush wear is negligible, and holding a resting ball takes ~0.05 A (est).
  - On a **RoboClaw 2x7A** (one board for both wheels, the encoders wired to it), the RoboClaw closes the speed and position loops itself: integrated control, like the drive's RoboClaw option.
  - **Packaging is not the problem I first thought.** Its ~70 mm body below each wheel stays inside the module's tilt clearance: up to 110 mm doesn't change the rim or pivot height (`SimParams.feed_motor_reach`). It is ~25 g heavier per wheel than the gimbal motors. VERIFY against the J-bend and the chute walls in CAD.
- **The micro metal gearmotor and the N20 worm are out:** too little torque at speed, or too little speed.
- **Holding costs almost nothing at rest.** A ball resting in the nip needs ~0.004 N m (est, ~0.07 A). The current only rises while feeding and during a catch's impact (peaks 2.3 A, for milliseconds).
- **The driver:** SimpleFOC Mini v1 (DRV8313, 2.5 A per phase) is rated 8-24 V, so the feed motors run from a 12 V rail (a small buck converter), not the 7S pack. On 12 V the GM3506 tops out at ~2.1 A, inside the driver's rating.
- **Catching is unchanged** (catch rig 103/104 for every motor that feeds). In the full 6-7 m game the final round-3 robot catches 96% in firmware adult mode and 100% with physics limits (round 2: 95 / 98%; table in 23.2).

### 2. Viscoelastic contacts (`contact_model.py`)

- **Ball contacts: Hunt-Crossley.** The damping grows with the compression, so a hard hit loses a bigger share of its energy than a soft one, and the force starts and ends at zero (no "sticky" release). Each material is fitted in MuJoCo, by drop tests, to its restitution at 1 and 7 m/s (est until drop-tested): the foam ball on hard surfaces 0.55 / 0.38, on the catch pad 0.10 / 0.04.
- **Tires: a standard linear solid** on the flywheel and feed-wheel squeeze slides: the static spring plus a Maxwell arm (2× stiffer at high rates, 3 ms relaxation, est). Through a shot the arms carry up to 10 N on top of the 28 N static spring and take ~0.2 J, which the flywheel motors make up.
- **Restitution on a hard floor, by impact speed:**

| Impact speed (m/s) | 1 | 2 | 4 | 6 | 7 | 10 |
| --- | --- | --- | --- | --- | --- | --- |
| Round 2 "e = 0.45" (as MuJoCo actually ran it) | 0.47 | 0.50 | 0.51 | | 0.52 | 0.52 |
| **Round 3 foam ball** | **0.55** | **0.52** | **0.46** | **0.40** | **0.38** | **0.32** |
| Round 3 catch pad | 0.10 | 0.09 | 0.07 | 0.05 | 0.04 | 0.03 |

  The old contact bounced harder the faster it was hit (MuJoCo's soft-contact impedance and gravity during the contact), the opposite of foam.

- **What changed, from the same studies:**

| | Round 2 (elastic, ideal feed) | Round 3 (viscoelastic, GM3506) |
| --- | --- | --- |
| Launcher exit speed, same rpm | | +0.4 to +2% (more grip from the stiffer squeeze); recalibrated, `launcher_cal_chute_ve.json` |
| Catch rig, fast flat arrivals | 99.0% | 99.0% |
| 6-7 m shallow throws, firmware adult / physics | 95% (6 not in place, 5 bounced) / 98% | **96% (8 / 0) / 100%** |
| Return throws at 6.5 m, median / 90% | 14 / 19 cm | **12 / 17 cm** |

- **So the elastic contacts were not hiding an error.** The pads were already very lossy, and the bounce-outs that mattered came from geometry (round 2's padding and tilt).
- **Within noise, round 3 catches and throws back as round 2 did**, with no bounce-outs in this sweep. The round-3 column is the final defaults (GM3506 feed, viscoelastic contacts, rolling resistance, collidable bumper). Intermediate runs ranged 96-98% caught and 8-12 cm returns, so treat ±2 points and ±3 cm as noise.
- **Shot scatter** in a deterministic sim still only comes from how each catch leaves the ball, so real scatter is a ball-to-ball and tire-to-tire matter: that is what 23.3 randomizes.
- **Cost:** ~25% more compute per sim second.

### 3. Randomized worlds (`randomize.py`, `studies/randomized.py`)

Each world draws all 17 guessed parameters at once, on a Latin hypercube over plausible ranges (all est):

- ball: diameter 72-76 mm, mass 18-26 g, Cd 0.42-0.58, foam stiffness, restitution ×0.75-1.25, grip ×0.75-1.25;
- pad restitution ×0.5-2; net grip 0.7-1.2 and rolling resistance 4-20 mm; tire grip 0.8-1.3; floor grip 0.7-1.1;
- cameras: latency 30-70 ms, calibration 0.04-0.3°, edge noise 0.4-0.9 px, 2-10% dropped frames, 0-3% clutter; ball spin 3-10 rad/s.

The robot doesn't know the draw: it is built for the 74 mm ball, predicts with the catalog ball (74 mm, 22 g, Cd 0.5), uses the nominal launcher table and an acceleration table that assumes a 0.95 floor. That is the robot before anything is measured. Catching: the round-3 robot in firmware adult mode, the 6-7 m shallow throws, 192 worlds × 36 throws. Returns: 96 worlds × 8 return throws at 6.5 m.

**Catch rate is a range, not a number:**

| | Caught |
| --- | --- |
| Nominal guesses (same throw seeds, 288 throws) | 93% (other seeds 94-98%: throw sampling alone moves it ±2) |
| **All 192 worlds (6,912 throws)** | **86%** |
| Per world: median / 10-90% / worst | **89% / 72-97% / 50%** |
| Worlds below 80% | 46 of 192 |

Misses: 752 not in place, 207 bounced out.

**Which guesses matter** (catch rate in the bottom vs top third of each range; logistic-regression effect per full range, cluster-bootstrapped by world):

| Parameter | Range | Bottom → top third | Effect (log-odds) | Verdict |
| --- | --- | --- | --- | --- |
| **Stereo calibration error** | 0.04-0.3° | **92% → 78%** | −2.3 ± 0.2 | **Dominant.** A direct A/B on the same throws: 97% at 0.04°, 78% at 0.3° |
| Net rolling resistance | 4-20 mm | 83% → 88% | +0.9 ± 0.2 | A lossy net helps catching (the indexer wanted the opposite) |
| Ball spin | 3-10 rad/s | 88% → 84% | −0.6 ± 0.2 | Film real throws |
| Camera latency | 30-70 ms | 87% → 85% | −0.4 ± 0.2 | |
| Dropped frames | 2-10% | 84% → 88% | +0.6 ± 0.2 | **Not a real benefit:** no effect at 0.1° calibration in a direct A/B (93.8 vs 94.1%). It grows with calibration error (+2 / +4 / +7 points): the filter treats a fixed calibration bias as white noise, so more frames make it more confident in a biased estimate |
| Floor grip | 0.7-1.1 | 85% → 87% | +0.3 ± 0.2 | Small, at the firmware's acceleration table |
| Ball diameter, mass, Cd, stiffness, restitution, grip; pad restitution; net and tire grip; edge noise; clutter | | within ±2 points | \|effect\| ≤ 0.3 | **Don't matter for catching** within these ranges |

**Return throws** (96 worlds): median miss 14 cm per throw (90%: 37 cm); world medians 7 / 14 / 32 cm (10-50-90%). Only tire grip shows a clear effect (bottom third 25 cm, top third 14 cm). The ball's mass and size don't; the learned range bias most likely absorbs them.

- **What to measure first, in order:** how well the stereo calibration holds on the robot (and how fast it drifts), the spin of real throws, and the camera latency. The ball's properties can wait: a 72-76 mm, 18-26 g batch catches the same.
- **A filter change worth making:** model the calibration error as a slowly varying bias, not per-frame noise. That would recover part of the calibration loss.
- **Caveat:** this run predates 23.4's rolling resistance and collidable bumper (the wheels rolled freely and the bumper was decoration). Neither moves the catch rate measurably on a flat floor (23.4).

### 4. Floors and wheels (`floor.py`, `studies/floor_wheels.py`)

**The roller model holds up on four wheels.** Every v3-v6 result used the fast sphere wheel; v2 had validated the explicit double-row rollers only on the kiwi. On the round-3 X4:

| | Sphere model | Explicit rollers |
| --- | --- | --- |
| Slip-free acceleration at body 30 / 45 / 60 / 90 / 135° (m/s²) | 4.1 / 3.8 / 3.7 / 4.9 / 4.7 | 4.1 / 4.3 / 3.7 / 4.5 / 5.5 |
| At 0° / 180° (the test tops out near 6.3) | ≥ 6.3 / 6.0 | 6.2 / 5.1 |
| Cruising 1.5 m/s: speed ripple, heading drift over 1.5 s | 0%, 0.0° | 0.1-0.4%, ≤ 0.2° |
| 6-7 m catch game, firmware adult (72 throws) | 93% | 94% |

- The two models agree within 18% in every direction (the rollers a little grippier on the diagonals), and in the game.
- The roller model costs ~2× the compute, so the sphere stays the default **on a flat floor**.

**Rolling resistance.** The rubber rollers lose energy as they roll (Crr 0.02 on a hard floor, 0.06 on carpet, est). It is now on by default, as joint drag that follows each wheel's load and the surface under it: Crr·N·R on the hub, Crr·N·r on the roller freedom. Cruising at 1 m/s draws 3-5% more battery current on a hard floor and 10-23% more on carpet (the 22 W of electronics included). It must not be MuJoCo rolling friction: in the elliptic friction cone, a wheel that always rolls keeps its rolling torque at the limit and loses sliding grip, so the base ran 2-7% slower than its wheels.

**Thresholds and carpet edges, crossed with a ball in the pocket** (6, 12, 18 mm thresholds with 20 mm ramps; 8 and 12 mm square rug edges; 8 mm carpet trim with a 25 mm bevel; at 0.5, 1 and 2 m/s; moving at body 0°, 45° and 90°). Results from the roller model; the sphere model can't represent a roller meeting a step sideways:

| Obstacle | Crossed (0 / 45 / 90°) | Ball kept | Peak body tilt at 0.5 / 1 / 2 m/s | Wheel-odometry error at 2 m/s |
| --- | --- | --- | --- | --- |
| Threshold 6 mm | all | all | 3 / 4 / 6° | 13 cm |
| Threshold 12 mm | all | all | 4 / 10 / 18° | 29 cm |
| Threshold 18 mm | all | all | 8 / 15 / **28°** | 49 cm |
| Rug edge 8 mm, square | **stuck at 45°** at 0.5 and 1 m/s; crosses at 2 m/s | all | 5 / 16 / 11° | 64 cm |
| Rug edge 12 mm, square | **stuck at 45° at every speed** | all | 4 / 8 / 16° | |
| Carpet trim 8 mm, 25 mm bevel | all | all | 3 / 6 / 10° | 11 cm |

- **An omni wheel can't climb a square edge sideways.** At body 45°, two of the X4's wheels meet the edge purely on their Ø16 mm rollers; a step at the roller's radius (8 mm) stops them, and the sphere model wrongly crosses it. Ramped edges (thresholds, carpet trim) are fine in every direction.
- **Proposed:** keep square rug edges out of the play area, or cross them only at body 0° or 90°. Thresholds up to 12 mm are fine; at 18 mm and 2 m/s the base pitches 28°, so slow to ≤ 1 m/s over thresholds (the lidar map can mark them).
- **Odometry is the casualty:** a crossing at 2 m/s costs 10-50 cm of wheel odometry. This is why the optical-flow sensor and the IMU are in the design (section 7); the X4's over-determined odometry can flag the slipping wheel.

**Floor friction that varies, and a home floor** (the catch game, firmware adult, 6-7 m throws; the acceleration table still assumes 0.95):

| Floor (72 throws each, ±3 points) | Sphere model | Roller model |
| --- | --- | --- |
| Uniform 0.95 | 96% | 97% |
| Friction field, σ 0.1 / σ 0.2 (0.5 m patches) | 94% / 99% | |
| Two slippery 0.5 stripes across the catch area | 97% | |
| Home floor: σ 0.1 field + 12 mm threshold 0.5 m in front + 8 mm rug behind | **86%** (one flip) | **86%** |
| Same, firmware crosses the threshold at ≤ 1 m/s | 92% (no flip, peak tilt 10°) | 82% |

- **Varying friction doesn't matter** at the firmware's acceleration table (it plans at 85% of the 0.95 floor's limit).
- **A threshold or a rug inside the catch area costs about 10 points**: the base hops or pitches up to ~24° while chasing at 1.5-2 m/s. Slowing over it removes the flip but not the lost reach.
- **Proposed:** wait at least ~1 m from thresholds and rug edges (keep the catch area flat), and cross mapped thresholds at ≤ 1 m/s elsewhere (`SimParams.slow_zones`).
- **The 180° row is the second way to flip** (the first is below): crossing that threshold at 1.7 m/s, the base hopped (all four wheel loads at zero for ~60 ms) and landed on a missed ball beside it.

- **A missed ball under a wheel can flip the robot.** In one moving catch at 2 m/s, a missed ball bounced into a wheel slot and the wheel climbed it: the robot turned over (roller model; it rocked 37° in the sphere model). v5 saw a 40° rock from the same thing. In the sim the TPU bumper ring was decoration. It now collides (`bumper_collides`, the default), together with:

  - a flexible skirt down to ~19 mm. The 34-62 mm ring alone meets a resting ball above its 37 mm centre, so a pitched-up base can still ride over it; the skirt touches only balls, since it flexes over thresholds and rugs;
  - guards over the wheel tops, as v2's pods have.

  On the same throw the base rocks 25° and recovers (sphere model: 4°). **Proposed:** the real bumper skirt must reach ~20 mm above the floor and close the wheel slots (v2's CAD stops at 45 mm), and the wheels need top guards. The planner could also avoid a missed ball's landing spot, which the vision tracker knows.

### 5. Motor study: right-sized? cheaper? four AK40-10s? (`studies/motor_study.py`)

**What each actuator actually does in play** (round-3 robot, catch + return throws from 3 m and 6.5 m, 24 tosses per row):

| Actuator | Demand in play (peak / 99% / RMS) | Fitted part | Verdict |
| --- | --- | --- | --- |
| Drive wheel torque | firmware 1.3 / 0.8-0.9 / 0.2-0.3 N m; physics 2.0 / 1.1-1.2 / 0.3-0.4 N m | AK40-10: 3.7 N m peak, 1.4 rated | **2-3× more torque than used.** The floor takes ~0.6 N m per wheel (12 N at the rim); the peaks above that are short transients, most likely the four speed loops working against each other on the over-determined X4 (not traction). Winding +0.5 °C |
| Drive wheel speed | firmware 376 rpm (2 m/s at 6.5 m), physics 406 rpm | AK40-10: 435 rpm at 24 V, **399 at 22 V** | **Speed is the tight one:** on a low battery 2 m/s along a diagonal needs all of it, with no torque left |
| Flywheels | 5,000-5,250 rpm, 25 A spin-up (the ESC limit), 5 A RMS | F90 2806.5 1300KV + Lucid 8S | Right. ~14% throttle at the top speed; spin-up 0.14 s to 3,000 rpm, 0.21 s to 4,500 |
| Tilt | **3.0 N m (saturated at the sim's cap)**, 1.05 RMS, 6.5 rad/s | "35 kg cm class" servo (3 N m, 5 rad/s in the sim) | **Undersized**; see D |
| Feed wheels | 2.3 A peak, 0.24 A RMS | GM3506 + SimpleFOC Mini | Right (23.1) |
| Battery | 12 A peak, 1.2 A mean | 7S1P P50B, 40 A BMS | Unchanged |

**Drive alternatives** (the 6-7 m catch game, 72 throws per row, so ±2.5 points):

| Drive set | Firmware adult / physics | Peak drive torque used (of the motor's) | Robot mass | Ground clearance | Set price |
| --- | --- | --- | --- | --- | --- |
| **4 × AK40-10** (baseline) | 96% / 99% | 1.5 / 3.0 (3.7) N m | 5.43 kg | 22 mm | $544 |
| **4 × DJI M2006 P36 + C610** | 93% / 99% | 1.6 / 2.7 (2.7) N m | 5.10 kg | **36 mm** | **$320** |
| 4 × Pololu 37D 24 V 19:1 + DRV8874 (brushed) | 97% / 97% | 1.0 / 1.0 (1.0, saturated) | 5.53 kg | 30 mm | $292 |
| 4 × SteadyWin GIM6010-8 | 96% / 96% | 1.7 / 3.7 (9.8) | 6.22 kg | 14 mm | $420 |
| **3 × AK40-10 (kiwi)** | **85%** / 92% | 1.3 / 3.0 (3.7) | 4.92 kg | 22 mm | $408 |

- **Four wheels: yes.** The kiwi saves one motor ($136, 0.5 kg) and loses 11 points in firmware adult mode (11 throws not reached in 72). Most likely its flat 2.5 m/s² limit and weaker diagonals cost the reach (on the axle deck it does survive a panic stop, section 20).
- **Four AK40-10s: no.** Every 4-motor alternative catches the same within noise. The AK40-10's torque is never used, and its speed is the binding limit.
- **Proposed: the DJI M2006 P36 + C610** ($80 a wheel, ~$224 less for the set):
  - 15% more top speed (2.3 m/s at the rim on a 22 V battery, against 2.0);
  - torque still left at 2 m/s on a low battery (0.9 N m against the AK40-10's ~0);
  - 330 g lighter, and 36 mm of ground clearance instead of 22 (a 24 mm motor);
  - CAN, with an FOC ESC that takes current commands at 1 kHz; the MCU closes the speed loop, as it does now.
- **Before switching, VERIFY:**
  - the C610's input voltage against a full 7S pack (29.4 V). RoboMaster packs are 6S; if it is a 6S part, the pack becomes 6S. A 6S pack also allows a cheaper 6S flywheel ESC;
  - the P36's backlash and noise;
  - the M2006's thermal constants (the sim's 33-38 °C winding uses estimates);
  - supply.
- **The Pololu 37D is the budget fallback** ($73 a wheel with its H-bridge). It saturates at its ~1 N m stall torque, and it is the heaviest of the 4-motor sets but one.
- **Packaging:** the 65-68 mm M2006 and 37D reach in to r ≈ 60 mm along the diagonals and overlap v3b's one-row 150 × 72 mm battery by 4-13 mm. Even the AK40-10 overlaps it by ~1 mm, so it is a v3b packaging issue regardless. A 4 + 3 two-row pack (~86 × 43 × 23 mm) clears all of them (AK40-10 17 mm, M2006 10 mm, 37D 2 mm). VERIFY in CAD.

**Flywheel motors** (bench, from rest): the F90 2806.5 reaches 3,000 / 4,500 rpm in 0.14 / 0.21 s, and a shot dips it 23-24% for ~140 ms. The iFlight XING 2806.5 is identical in the sim and $4 cheaper. A 2207 1750KV is slower (0.19 / 0.28 s); a 3115 900KV is faster (0.10 / 0.14 s) but heavier and pricier. Keep the 2806.5 class. The ESC must be 8S-rated because a full 7S pack is 29.4 V; the Lucid 8S 4-in-1 ($90, two channels spare) stays unless the pack goes 6S.

**Tilt servo** (D, 72 throws per row, firmware adult):

| Tilt actuator at the pivot | Caught (not in place / bounced) |
| --- | --- |
| Sim default (3 N m, 5 rad/s) | 93% (4 / 1) |
| DS3235 at 8.4 V (3.4 N m stall, 0.11 s/60° = 9.5 rad/s), ~$25, PWM | 92% (5 / 1) |
| Feetech STS3250 at 12 V (4.9 N m stall, 0.133 s/60° = 7.9 rad/s), $74, 12-bit absolute encoder on a TTL bus | 89% (7 / 1) |

- **Catching doesn't separate them** (±3 points of noise).
- **The choice is about holding and knowing the angle.** The tilt sits at the servo's torque limit in fast swings and holds ~1 N m RMS through play. That is a third of a hobby servo's stall torque, all the time. The tilt angle is also the launch elevation, so the firmware should read it back.
- **Proposed:** a serial-bus servo with an absolute encoder (STS3250 class) on the 12 V rail it shares with the feed motors. The DS3235 is the cheap fallback; it needs an 8.4 V rail and reports nothing.

**Integrated wheel drives, and Dynamixels:**

- **Dynamixel servos can't drive the wheels.** The fastest 12 V X-series (XM430-W210, XW540-T140) turn ~72-77 rpm with no load; the wheels need ~400 rpm at 2 m/s. Geared up 5:1 by a belt, they would have no torque left at speed. The P series is fast enough but costs $1,000+ each.
- **The AK40-10 is already the integrated choice:** motor, encoder, FOC driver, and speed and position modes over CAN.
- **The M2006 + C610 is integrated hardware but not integrated control.** The motor has its own rotor-position sensor, so sensored FOC is smooth down to standstill, with 0-8191 counts per motor turn × 36 at the wheel. But the C610 only takes a current command over CAN and reports angle, speed and torque at 1 kHz; the speed or position loop runs on the µC.
- **The Pololu 37D + RoboClaw 2x7A** is integrated in the same sense as the AK40-10: the encoder is built into the motor, and the RoboClaw closes speed and position PID itself, with an autotune tool. It takes 6-34 V (7S full is 29.4 V) and drives two motors per board.
- **A Teensy 4.1 could drive four brushed motors itself.** Its i.MX RT1062 has four hardware quadrature decoders, and a 1 kHz loop is easy. That needs four H-bridges and your own loop tuning, which is the custom route.

**Motor and controller BOM** (every actuator; USD from the makers' or resellers' pages, October 2026). *Superseded by section 24*, which re-checked the prices (the M2006 is now $55, the XING $40.99), ran the drives on a low battery and on 6S, and ranks every class by cost:

| Item | Option | Qty | Each | Total |
| --- | --- | --- | --- | --- |
| Drive | **CubeMars AK40-10 V3.0 with driver** (integrated, CAN) | 4 | 135.90 | **543.60** |
| | DJI M2006 P36 + C610 (host runs the speed loop) | 4 | 41 + 39 | 320.00 (motor listings run $75-95: up to ~536) |
| | Pololu 37D 24 V 19:1 with encoder #4691 + RoboClaw 2x7A (1 per 2 motors) | 4 + 2 | 60.95 / 79.95 | 403.70 |
| Flywheels | T-Motor F90 2806.5 1300KV (or iFlight XING 2806.5, $26) | 2 | 30 | 60.00 |
| Flywheel ESC | TBS Lucid 8S 60A 4-in-1, AM32 | 1 | 90 | 90.00 |
| Tilt | **Feetech STS3250** (12 V, encoder, TTL bus) | 1 | 73.99 | **73.99** |
| | DSServo DS3235 (PWM, no feedback) + 8.4 V BEC | 1 | ~25 + ~10 | ~35 |
| Feed | **LK-TECH MF4005-V2 (CAN)** (integrated) | 2 | 100 | **200.00** |
| | **Pololu 25D 9.7:1 HP 12 V with encoder + 1 RoboClaw 2x7A** (brushed, RoboClaw runs the loops) | 2 + 1 | 56.95 / 79.95 | **193.85** |
| | iFlight GM3506 + AS5048A + SimpleFOC Mini | 2 | 53.63 + 15 | 137.26 |
| Power for feed + tilt | 12 V buck converter, ~3 A | 1 | ~10 | ~10 |

| Set | Drive | Others (flywheels, ESC, tilt, feed, 12 V) | Total |
| --- | --- | --- | --- |
| **All integrated (proposed): AK40-10, F90, Lucid, STS3250, MF4005** | 543.60 | 433.99 | **~978** |
| Integrated, brushed drive: 37D + RoboClaw, the rest as above | 403.70 | 433.99 | ~838 |
| DJI drive (host loops): M2006 + C610, the rest as above | 320-536 | 433.99 | ~754-970 |
| Cheapest DIY: M2006 + C610, XING, Lucid, DS3235, GM3506 + SimpleFOC | 320 | ~324 | ~644 |

The motors and controllers come to ~$650-980 of the robot. The drive set is the swing item (±$220), and the integrated feed and tilt parts add ~$110 over the DIY ones.

**Controllers everywhere:**

| Actuator | Controller | Fit |
| --- | --- | --- |
| Drive | AK40-10's integrated driver over CAN (or C610 over CAN) | Fine. VERIFY the comm-loss behavior (coast, not hold) |
| Flywheels | Lucid 8S AM32 over bidirectional DShot, MCU rpm loop | Fine |
| Feed | 2 × LK MF4005 (integrated FOC, CAN), or 2 × SimpleFOC Mini with the FOC loops on a µC | Integrated: FOC lives in the motor. With the SimpleFOC route the STM32G4 can run both loops (3 PWM + an SPI encoder each), or a small dedicated board can |
| Tilt | Hobby PWM servo, no position feedback | **Change:** a serial-bus servo with an absolute encoder (Feetech STS3250, 12 V rail). The launch elevation is the tilt angle, so the firmware should read it, not assume it |

## 24. Actuator choice, cost first, every actuator class; a shared actuator database (proposed)

Fernando asked for two things: a reusable actuator database for all his robotics projects, and a re-run of this robot's actuator choice with cost as a primary criterion and every actuator class considered, not only brushless. Prices were re-checked on Oct 19-20, 2026. **Everything here is Proposed.**

### 1. The actuator database (`database/`)

- **Where:** `omni-robot/database/` for now; it moves one level up (`Robotics/database/`) once it settles, so it imports nothing from omni-robot. [README](database/README.md) has the full schema and the rules.
- **What:** plain CSV you can edit in Excel. `motors.csv` (32 rows: brushed spur and planetary gearmotors, BLDC in- and outrunners, gimbal motors, closed-loop steppers, a hub motor, integrated actuators), `servos.csv` (6), `controllers.csv` (16: H-bridges, smart brushed controllers, ESCs, FOC drivers and controllers, an MCU host, DC-DC converters), `sources.csv` (51).
- **Rules:** SI units in the column names (`torque_stall_Nm`, `speed_noload_rpm`, `price_usd`). Speeds and torques are at the output shaft. Blank means unknown. Every row has a default provenance (datasheet / reseller / forum / est / measured) plus per-value overrides, its source ids and the date checked. An empty `meas_*` column group waits for bench data and wins over the datasheet when filled (sim-to-real).
- **`actuator_db.py`:** loader and validator; the DC model from the five datasheet points (as `catalog.FeedMotor`); requirement checks (PASS / FAIL / UNKNOWN); cost of a complete function (part + share of a controller + power conversion); $ per N m at speed; N m per kg; motor-controller compatibility; export to this sim's `DriveMotor`, `FeedMotor`, `FlywheelMotor` and `Esc`; and a gaps report (`gaps.csv`, 49 parts with blanks to fill). 15 tests, including the export round trip against `catalog.py`.
- **Not used yet:** the Tyto Robotics dynamometer database (database.tytorobotics.com) renders client-side; it is listed as source S042 for flywheel-motor curves later. The sister project's early `bom.yaml` / `COMPONENTS.md` / `NewWheelMotor.md` were mined for the owned parts (AK45-10, MTO5065, ODESC 3.6) and the 5010 class.

### 2. The study (`studies/actuator_choice.py`, `studies/motor_study.py`, `studies/feed_motor.py`)

`actuator_choice.py` reads the database, checks every candidate against this robot's requirement records, ranks them by function cost and joins the sim results; its output is `studies/results/actuator_choice.md`. Two sim changes made the drive check honest:

- **Low battery:** `SimParams.battery_soc` (default 1.0). Earlier drive runs started on a full pack; the requirement is at the minimum voltage, so the new rows start at 10% (7S: 22-23 V under load).
- **A 6S pack** (`6S1P-P50B` in the catalog), for the parts rated 24 V.
- `motor_study.py` B now also reports each drive's **session steady-state winding temperature** (the hottest motor's mean copper loss × its thermal resistance, est), and runs only named rows (`motor_study.py BD 10% STS`), keeping the others.

**Requirements** (duty in play, 23.5): drive peak 1.3 N m firmware (2.0 physics), RMS 0.2-0.36 N m (0.38-0.41 at 10% charge), **376 rpm in firmware adult mode** (2 m/s on a diagonal; 406 with physics limits) at the low-battery voltage, encoder; flywheels to ~5,250 rpm with ~0.15-0.2 s spin-up; tilt ≥ 3 N m peak, ~1 N m RMS, ~6-8 rad/s, angle feedback; feed wheels 764 rpm with ≥ 0.06 N m left, back-driven ≤ 5 mm by a catch.

### 3. Drive wheels (per wheel; 6-7 m catch game, 72 throws per row, ±3 points)

| Cost per wheel | Option | Integration, firmware | Caught at 10% charge, firmware / physics | Session winding (est) | Verdict |
| --- | --- | --- | --- | --- | --- |
| $73 | Pololu 37D 24 V 19:1 + DRV8874 (brushed) | MCU runs PWM, encoder, PID | 90% / 92% (7S); 89% / 90% (6S) | **157 °C** | **Fails:** 0.93 N m stall < 1.3 peak, overheats |
| **$94** | **DJI M2006 P36 + C610, on 6S** | sensored FOC ESC; MCU runs the speed loop over CAN | **93% / 97%** (6S full: 93% / 99%) | 70 °C | **Cheapest that meets them** |
| $94 | DJI M2006 + C610 on 7S | same | 93% / 100% | 69 °C | C610 at 29.4 V unverified |
| $101 | Pololu 37D + RoboClaw 2x7A (1 per 2) | RoboClaw runs the loops | as the 37D | 157 °C | Fails, as above |
| $105 | SteadyWin GIM6010-8 (GDS68 driver) | integrated, CAN | 96% / 96% (full pack) | | Passes; +0.8 kg robot, 14 mm clearance |
| $136 | **CubeMars AK40-10** | **integrated, CAN** | **94% / 97%** | 32 °C | **Preferred integrated** |

Rejected on the datasheet: Waveshare DDSM115 hub servo (200 rpm, 765 g, a tyred wheel, not an omni), closed-loop NEMA 17 steppers ($36-70, 0.48 N m holding that falls with speed), Dynamixel X-series (72-77 rpm), goBILDA 5203 (a 12 V motor: 0.15 N m left at 400 rpm on 12 V), MTO5065 + ODESC (450 g a wheel, DIY Hall/FOC setup, $110), Damiao DM-J4310 and AK45-10 (200 / 180 rpm). RC-car sensored motor + ESC systems were not seeded: their ESCs give the MCU no speed or position feedback and are poor at crawling speeds, and they still need a ~25:1 gearbox.

- **Brushed is out for the drive (it stays fine for the feed).** The 37D's 8 Ω winding turns the play RMS (0.35-0.42 N m) into ~10 W of copper loss per motor, continuously. The 157-193 °C steady state rests on estimated thermal constants, but a 37 mm can motor won't shed 10 W. It also saturates at 0.96 N m and catches 6-7 points less on a low battery. The feed wheels differ: they turn ~0.1 s per shot.
- **The M2006 doesn't need 7S.** On a 6S pack it catches the same as on 7S, full or at 10%, and its C610 ESC stays inside its 24 V rating (DJI publishes no maximum; RoboMaster robots run 6S).
- **Its thermal margin needs a bench check.** DJI rates it 1 N m continuous against 0.41 N m RMS in play, and the sim estimates ~70 °C. With a reseller's lower torque constant (0.18 N m/A at the output, 1.8 N m at 10 A; `M2006-lowkt`) it still catches 93% / 97%, but the estimate rises to ~127 °C.
- **Price and supply:** the DJI store now asks $55 for the motor (was $41) and $39 for the C610, and both were out of stock on Oct 20.
- **The AK40-10 needs 7S:** on 6S at 10% its no-load speed falls to ~350 rpm (by its constants, not simulated).
- **Drive RMS rises on a low battery** for every motor (0.29 → 0.38-0.41 N m firmware). That is most likely the four speed loops fighting on the over-determined X4 (23.5). A torque-sharing allocation in the base controller would cut heating whichever motor is chosen.

### 4. Feed wheels, tilt, flywheels

| Feed wheel (per wheel, 12 V rail) | Cost | Feeds 9/9 | Worst back-drive | Verdict |
| --- | --- | --- | --- | --- |
| iFlight GM3506 + AS5048A + SimpleFOC Mini | $68.63 | yes | 3.1 mm | Passes; DIY FOC on the MCU |
| **Pololu 25D 9.7:1 HP 12 V + DRV8874** | **$68.89** | yes | **0.7 mm** | **Cheapest that meets them** (brushed, MCU loops) |
| LK-TECH MS4005-V3 (new, $75, integrated) | $75 | yes | 5.8 mm | Fails the hold; ~0.04 N m left at 764 rpm |
| **Pololu 25D + RoboClaw 2x7A (1 per 2)** | **$96.93** | yes | 0.8 mm | **Preferred integrated** (the RoboClaw runs the loops) |
| LK-TECH MF4005-V2 (integrated) | $100 | yes | 4.5 mm | Passes; its listing says 7.4-32 V: it might run from the pack (VERIFY) |
| iFlight GM2804 / Pololu micro 30:1 | $63 / $39 | yes / 2 of 9 | 8.7 / 5.5 mm | Fail |

| Tilt actuator | Cost | Caught (72 throws, ±3) | Verdict |
| --- | --- | --- | --- |
| DSServo DS3235 + BEC | ~$30 | 92% (at 8.4 V) | Fails: its datasheet stops at 7.4 V (the 8.4 V figures are from listings); no angle feedback |
| Feetech STS3215 (new, 12 V) | $26-49 | **96%** | Fails on paper: 2.9 N m, 4.7 rad/s, and its 1 N m rating equals the tilt's 1.05 N m RMS (no thermal margin) |
| **Feetech STS3250** | **$73.99** | 89% | **Cheapest that meets them**; no rated torque published (VERIFY) |
| LK-TECH MG4010E-i10 (new) | $165 + driver | 94% | Passes; nothing gained for the price |
| Dynamixel XM430-W210 | $310 | | Passes; 4× the price |

- **Catching doesn't separate the tilt actuators** (89-96%, within noise). The choice is holding torque and the angle readback.
- **A counterbalance spring on the tilt pivot would make the $26-49 STS3215 enough.** The ~1 N m RMS is mostly the module's weight held off-centre; a printed or wire torsion spring that carries it would drop the RMS far below the STS3215's rating. Worth a CAD look before buying the servo.

**Flywheels:** a 6S pack spins up exactly as 7S does (0.14 / 0.21 s to 3,000 / 4,500 rpm): the 25 A ESC current limit sets it, not the voltage. The F90 2806.5 is the cheapest of the class ($30 on Oct 17; $35.50 at another shop), and the XING is now $40.99 (was $26). For the ESC, an 8S AM32 4-in-1 works on either pack: the Pilotix 65A (~$56, converted from €48) or Sequre Blueson A2 8S (~$60, bidirectional DShot to VERIFY) undercut the Lucid (~$73, from €63; design.md 16 had $90).

### 5. System choices

| Choice | Finding |
| --- | --- |
| **Pack: 6S or 7S** | 6S keeps the C610 (24 V) and any 6S part inside its rating; flywheels identical; one cell fewer (~70 g, ~$15 est). It rules out the AK40-10 (too slow on a low 6S pack). So: **6S with the M2006, 7S with the AK40-10** (VERIFY the AK40-10's maximum voltage) |
| 12 V rail | Needed either way: the 25D feed motors, the STS3250, the GM3506's SimpleFOC Mini (8-24 V) are 12 V parts. One ~3 A buck (~$10 est). The MF4005 may not need it (VERIFY its range) |
| Sharing controllers | One RoboClaw 2x7A per two brushed motors ($40 a motor) vs a DRV8874 each ($11.94): the RoboClaw buys onboard loops for $28 a motor. The flywheel 4-in-1 has two spare channels |
| MCU buses | Cheapest set: one CAN bus for four C610s (one command frame drives all four; 4 feedback frames per ms ≈ 0.5 Mbit/s), 2 DShot timers, the feed's 2 PWM + 2 encoder timers + 2 current-sense ADCs, one half-duplex UART for the servo. Preferred set: four AK40-10s on CAN (plus two MF4005s if chosen: at 1 kHz six nodes need ~1.3 Mbit/s, over classic CAN's 1 Mbit/s, so a second FDCAN or 500 Hz); the RoboClaw on a UART. An STM32G474 has 3 FDCANs and enough timers for either; a Teensy 4.1 (4 hardware quadrature decoders, 3 CAN) also could |

### 6. Motor + controller BOM

| Function | Cheapest compliant set (6S) | Cost | Preferred integrated set (7S) | Cost |
| --- | --- | --- | --- | --- |
| Drive | 4 × DJI M2006 P36 ($55) + 4 × C610 ($39) | $376 | 4 × CubeMars AK40-10 ($135.90) | $543.60 |
| Flywheels | 2 × T-Motor F90 2806.5 1300KV ($30) | $60 | same | $60 |
| Flywheel ESC | Pilotix 65A 8S 4-in-1 (~$56 est) | $56 | same | $56 |
| Tilt | Feetech STS3250 | $73.99 | same | $73.99 |
| Feed | 2 × Pololu 25D #4842 ($56.95) + 2 × DRV8874 ($11.94) | $137.78 | 2 × Pololu 25D + 1 × RoboClaw 2x7A ($79.95) | $193.85 |
| 12 V rail | buck converter (est) | $10 | same | $10 |
| **Total** | | **~$714** | | **~$937** |

Against 23.5's sets: its all-integrated set was ~$978 (MF4005 feed, Lucid at $90) and its cheapest DIY set ~$644 (old M2006 and XING prices, a DS3235 with no feedback, GM3506 + SimpleFOC).

### 7. Recommendation

**Build the cheapest compliant set (~$714) on a 6S pack. It is $223 under the integrated set.**

- **Drive: DJI M2006 + C610.** It catches the same as the AK40-10 within noise, on a low 6S pack too.
- **Feed: the brushed Pololu 25D on DRV8874s,** with the speed and hold loops on the MCU, which already runs the M2006's speed loops.
- **Tilt: the STS3250,** unless a tilt counterbalance makes the STS3215 enough.
- **Flywheels: F90s on an 8S AM32 4-in-1.**

**Conditions:**

- buy one M2006 + C610 pair first (both out of stock at DJI on Oct 20; check resellers);
- bench it on 6S at 0.4 N m RMS for its winding temperature, and check its backlash.

**Fallback:** if the M2006 can't be had, or runs hot, use the preferred integrated set on 7S (~$937) after confirming the AK40-10's maximum voltage. Fernando's preference for integrated devices costs $223 here, nearly all of it in the drive.
