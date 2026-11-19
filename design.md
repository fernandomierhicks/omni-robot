# Omni Robot — Design Baseline

Rewritten Nov 1, 2026; compressed Nov 7 and again Nov 17 to a current-design summary. Current focus: the arm robot. Catch development is paused.

**Rules of the road: keep this a short design baseline.**

- A session that changes the baseline or establishes an important finding updates the matching section here, `Start here` if the status changes, and the decision log for a lasting decision. Rewrite in place: the current choice, why, the decisive evidence, what is uncertain. No session reports or successive versions.
- Separate **adopted**, **proposed**, **implemented but unvalidated** and **paused**; a software default, a CAD export and Fernando's adoption are different facts. Label simulation, hardware measurements, estimates **(est)** and physical checks **VERIFY**. Older scores do not validate a changed robot.
- Keep this around 4,000 words. Procedures, run tables, CAD rounds, bench numbers and debugging live in the subsystem READMEs and study results (section 6); link them here.
- Before removing substantive material, append it or a dated snapshot to [design full history.md](design%20full%20history.md) (append-only). Do not start parallel design documents or edit `archive/`.

**History.** [design full history.md](design%20full%20history.md) keeps every earlier version. The latest full snapshot is **"November 17, 2026 — design.md snapshot before the compression to a current-design summary"**: detailed bench results, the lidar timing and SLAM studies, the v15 pod review, power budget, wheel noise/wobble, the full decision log. The Nov 7 snapshot and Appendix B hold earlier ones. `§1–31` in code means the history's sections; legacy `design.md 10–16` its Appendix A. Section numbers below are unchanged since Nov 7, so `design.md 4.x` citations still resolve.

## Start here

- **Active baseline (sim):** `robot_export.ARM` / `arm_params()` in `simulation/v11 sil`, a coarse model of the arm robot: X4, 140 mm omni wheels on 687-bearing rollers, sprung forks pivoting level with the axle, AK40-10 drives, SO-101 with a rear bin, D435 on the arm riser. **Nothing is built**; results are simulation unless marked.
- **CAD:** `cad/v14` is the last adopted whole-robot CAD (layout F). **`cad/v15` (the mid deck) is the working CAD:** a Nov 14 proposal on which Fernando's later choices (colours, the HRB pack, the bed-fit rule) are drawn and whose deck dovetails he has coupon-tested; not formally adopted. Not ready to print as a robot; the arm's stow collision is open (4.4, 7).
- **Hardware so far (bench):** the Teensy 4.1 with the arm firmware (protocol 9) drives AK40-10s on CAN from the robot's ROS nodes on a Pi 4, through the console's motor test (2.1, 4.7), and reads the BNO085 (`/imu` at 100 Hz, Nov 17). The RPLIDAR A2M8 runs on the Pi under the robot's own driver (4.6).
- **Supply:** HRB 6S 3300 mAh (baselined Nov 16); the v11 sim and `cad/v14` still carry the earlier OVONIC 6S 5200.
- **Compute and cameras:** robot computer **open** (4.9). **The riser D435 is the arm robot's only camera** (Fernando, Nov 5); it sees no wall tags, so a tag camera (IMX662 proposed, 4.12) or another tilt is still to choose (4.11). A Pi 4 runs a slow working robot with `nav_profile:=pi` (4.10).
- **Paused by Fernando:** the catch payload and real-camera ball benches, until stereo global-shutter colour cameras and a mobile robot exist.
- **Software (sim):** web console (health-gated), AI control through `omni_mcp`, people detection, tidy and fun missions on the full ROS stack (3).
- **Next:** CAD/model gaps, one wheel/pod on the bench (`cad/v13` `pod_jig.py`), arm navigation and picking on the candidate computer, then the rolling arm platform (8).

## 1. Goal

A mostly printed indoor home robot with omnidirectional driving, lidar SLAM, navigation, RC and browser control, a face and voice, and swappable payloads. **The immediate build is the SO-101 arm robot that tidies floor objects.** The parked catch payload tracks a 74 mm foam ball, catches it and returns it to a thrower 6–7 m away; the platform stays useful for both without making the catch robot a prerequisite.

Run the robot's own code in simulation before physical validation. First real runs and flashes are attended; measurements feed back into the models.

## 2. Baseline design

Dimensions are the current arm simulation and CAD (nominal, estimated mass), not a built robot.

| Subsystem | Current design and status |
| --- | --- |
| Base | X4, wheel centres r 146 mm, bumper r 225 mm (v14 Ø450; v15 Ø465), payload plate 181.5 mm high; arm model 9.63 kg, CG z 126 mm (est). |
| Wheels | 140 mm, two rows of six 39 mm rollers, **each on two 687 bearings round a printed sleeve clamped between the hub's spokes** (Fernando, Nov 13); TPU treads with 0.2 mm tip relief; PLA hub on the motor's output bearing and one 6804. Fit from printed coupons; strength, wear, grip untested. |
| Suspension | Sprung forks, −3/+15 mm, **pivot level with the axle** (4.7). v15: two springs in series per bore; spring rate ASSUMED. |
| Drive | 4 × CubeMars AK40-10 V3.0 KV170, CAN, **servo mode** (4.7). Bench: top speed 368 rpm at 24 V; wheel-speed cap 37.2 rad/s. |
| Battery | **HRB 6S 3300 mAh, 73.3 Wh, 558 g, 136 × 46 × 43 mm** ([components/battery](components/battery/README.md)); connector, C rating, sag VERIFY. Ramped stop at 21 V (provisional); ≈29 min typical (est). |
| Arm | SO-101 at x 140 mm on a 37 mm riser; finger pads, wrist camera (part TBD), rear bin; floor reach ~0.26–0.35 m. |
| Cameras | **D435 on the riser's front**, landscape, 30° down, centre (190.8, 0, 193.0) mm (`ARM_D435`). IMX662 forward camera proposed (4.12). |
| Lidar / IMU | **RPLIDAR A2M8** (Fernando, Nov 16: the unit he has) at the front, scan plane z 163 mm, read by the robot's own `omni_cpp` `rplidar_node` (4.6). BNO085 on the MCU over SPI (PS0/PS1 strapped high), game RV + gyro + accelerometer at 100 Hz: the wheeled-leg robot's proven driver, on hardware Nov 17 (`/imu` from `mcu_link` on the Pi at 100 Hz; firmware/README.md). |
| Computer / MCU | Computer open (4.9). Teensy 4.1: safety, 1 kHz control, odometry, CAN; board-free core shared with the SIL. |
| Power | **Nov 17 (Fernando): one latching main switch right before the 30 A main fuse is also the e-stop** and cuts the whole robot, computer included; no separate e-stop circuit, contactor or power board. Discrete DC/DC converters (his picks): 19 V 10 A (computer; bench-test its 22 V input floor), 12 V 8 A (arm), 5 V 5 A. The AK40-10s take the pack directly, unfused (they limit their own current). One common ground. INA226 (2 mOhm, total current) and INA3221 (arm 12 V, 19 V, 5 V); a powered USB hub. **Nov 18 (Fernando: power down), drawn in `cad/v15` (proposal, `tools/electrical.py`):** the battery's path (anti-spark switch for the main button, inline fuse, INA226, XT30 distribution block) and the 12 V and 5 V converters on the first floor, screwed into the deck's own accessory pilots (deck, pods, fins unchanged); the 19 V module fits nowhere on the deck and sits on floor 2 by the computer, which moved under the bin. No battery or drive lead leaves the first floor; 6 leads cross the lidar plane, all in the side fins' shadows. Main switch DC rating and inrush VERIFY. **Nov 18 (Fernando): the ELRS receiver lives on the Teensy carrier** (pcb/v1 J5, a 1x4 socket, the receiver on a male header; its pad pitch VERIFY), no lead of its own. |
| Teensy carrier | `pcb/v1` (rev C schematic, under review; 95 × 62 placement): 2-layer, all through-hole, JST XH breakout of every header pin; Teensy on USB power; CAN modules in sockets; pack divider; buzzer, RGB LED, NeoPixel; I2C to the current monitors. In `cad/v15` (Nov 18) on floor 2 beside the arm riser, plugs up. [Block diagram](pcb/v1/robot_block_diagram.html). |
| Interface | Robot-served web console, gamepad, RC as override; 5 in face, microphone, speaker (TBD). |
| Printing | PLA/PETG/TPU, no inserts, M3 thread-forming; every printed part fits 252 mm square (256 bed); rules in `cad/solidworks.md` 18. |
| Colours | **Baselined Nov 16 on `cad/v15`** (Fernando): Jade White structure and arm (the colour vision keys on saturated hue, so a white arm is ignored), Dark Gray suspension and fins, Orange bumper/bin/lidar pedestal/riser, Black hubs, TPU gray `#8e9089` treads. Source: `cad/v15/tools/palette.py`, v15 README "Filament colours". |

**Safety/control (SIL; partly on the bench):** ramped stops; 0.5 s link and MCU watchdogs. In AUTO, a pack under 21 V, a motor error or a silent motor that the base needs latches `FAULT_STOP` until DISARM at ≥ 22.2 V; RC manual is not stopped. Invalid CMDs are dropped. Kid profile 0.8 m/s (off only at rest), adult 2 m/s, arm-out 1 m/s. No bump sensing (4.13). The arm refuses unchecked motions and holds (never torque-off) on bad servo feedback. Thresholds are estimates.

**Parked catch kit:** net funnel, padded pocket, chute feed, IR beams, tilt, two Ø70 flywheels; v7-based model.

### 2.1 Software

- **Firmware** (`firmware/`): board-free core `lib/omni_fw`, COBS/CRC USB **protocol 9** (STATE carries each wheel's commanded/measured speed and current; a non-answering motor reports `DM_NO_REPLY`). The motor test (`DRIVE_WHEELS`) gives each wheel its own speed and ramp, driven from the console's Test › Motors. `pio run -e arm` / `-e catch`; `arm_bench` is a bench-only build (no e-stop, fixed 24 V). Details: [firmware README](firmware/README.md).
- **ROS 2 Jazzy** (`ros/`): MCU link, mode manager, lidar driver, scan prep, EKF, slam_toolbox, Nav2/MPPI, arm, face, console, people. **One implementation per node, C++ (`omni_cpp`) where it exists**; Python cores the sim imports keep C++ parity tests. TELEOP by priority `/cmd_vel/joy` > `/cmd_vel/manual` > `/cmd_vel`. Tunables are described ROS parameters in `robot.yaml`; geometry comes from `robot_export.py`.
- **Arm:** `omni_arm` (kinematics, motion, floor/wrist vision, coverage, tidy/carry/place/tower); `omni_cpp` `arm_driver` owns the servo bus. No MoveIt.
- **Console** (`omni_console`; Fernando, Nov 7): workspaces Home, Map, payload, See, Test, Robot; driving by hand and Call are modes on Home. **Health-gated (Nov 13):** one availability answer from caps and preflight greys every start with the robot's reason. Served by the robot computer under systemd; runs on the Pi 4 bench (`ros/tools/robot_console.sh`). Service requirements: logind `RemoveIPC=no`, `KillSignal=SIGINT` ([systemd README](ros/src/omni_console/systemd/README.md)).
- **AI control (sim):** Claude or Codex drive by task through `ros/tools/omni_mcp`: the robot offers primitives and senses, the model plans (Fernando). Lease, take-over, deadlines, request ids. Hardware not started.

## 3. Expected performance and evidence

**No hardware validation of the robot exists.** Rows are simulation (riser D435, the robot's C++ arm driver and firmware) unless marked. Full tables: the history's Nov 17 snapshot and v11 `studies/results/`.

| Evidence | Result and limitation |
| --- | --- |
| Tidy at scale, MuJoCo pick harness (Nov 7; 6 seeds × toys/clutter × arena/house) | **112/120 binned (93%)**, 88 s per object. "Binned" counts releases over the bin. `studies/arm_pick.py`. |
| Fun missions (Nov 7) | Fetch 3/3 put down 1–5 mm off; block tower 3 high on 5/6 seeds; Goalie 13/13 on-target saves. |
| Same missions on the full ROS stack (Nov 8–9, PC) | Tidy 5/6; tower 3 high; Goalie (rolled ball) 23/25; Fetch to a spot 3/3; **Fetch to a person unreliable (2/5, open, 7)**. |
| Navigation, full Nav2/SLAM (Nov 3–5, house) | 8/8 and 16/16 goals, arrival 0.04–0.14 m; navigate-then-tidy 6/6. [Report](simulation/v11%20sil/studies/results/pc_robustness_2026-11-05.md). |
| SLAM with the A2M8 (Nov 17, house tour, robot's driver stamps) | 3.7 / 8.3 cm median / p95, yaw p95 1.5° (4.6). |
| Near-floor obstacles (Nov 5) | Default `raw` depth layer drives over 7/12 toys; proposed `height` layer avoids all (4.5). |
| AI control (Nov 4–9) | All `ai_smoke.py` checks pass; find 108/140 right, 0 wrong. |
| Drive and endurance (Nov 1–2, chart-fit motor) | 50 ft hallway 9.0 s at the 2 m/s cap; collinear pivot doubles start acceleration (4.7). Runtime ≈29 min typical on the HRB (est). |
| Earlier v7 catch | 93.1% sim, 88.0% SIL over 216 throws. Earlier configuration. |

## 4. Major findings from trades and simulations

### 4.1 Perception limits the catch (parked)

Latency, frame rate and stereo calibration stability dominate (drift 0.04° → 97% catches, 0.5° → 58%, sim). Use a stiff bar, global shutter, synchronized exposure. History §§22–23.

### 4.2 Base

**X4 with sprung pods** (Fernando, Oct 31): Kiwi caught and braked worse, rigid X4 lost catches on rugs and with a wheel 1 mm low, a rocker pitched twice as much. History §25.

### 4.3 Catch module and launcher (parked)

Pocket, chute feed and 35° tilt took static catches 80% → 99%; ceiling-aware returns reach 7 m under an 8 ft ceiling (sim).

### 4.4 Layout and CAD

- **Layout F** (Fernando, Nov 3; `cad/v12`–`v14`): frame chassis, pods out from below, electronics and battery at the rear under the bin, lidar forward with fins as wedges in their own shadow (20.9° blind), arm/riser/bus board one module.
- **Mid deck (`cad/v15`, working CAD, not formally adopted):** a 20 mm deck at z 80–100 in four dovetailed pieces; pods hang under it and drop out downward; fins generated from a height map; battery under the deck's centre (about 10 mm off the CG, est); four service-stand pads. Wheel positions and the second floor unchanged. Open: the arm's nearest floor grasp is ~0.2 mm off the bigger bumper; the arm keep-out ring sits under the deck rim; bumper redesign not drawn. [cad/v15 README](cad/v15/README.md).
- **Harness (Fernando, Nov 11; proposal):** leads cross the lidar plane only through slots in the side fins' shadows; zip ties through PLA slots, spare holes; 18 AWG motor leads, CAN daisy chain with both terminators on the carrier. `cad/v13` README "Wiring harness".
- **Printed-part rules** (Fernando's reviews; `cad/solidworks.md` 18): pilots along the print axis, no captive nuts (face hex recess), every hole through, no dowels, real mating connectors modelled, 252 mm bed fit checked on the solid.
- **CAD/sim gap:** the sim's arm base and stow pose don't match the CAD (v10's stow collides). Reconcile before trusting waypoints on the assembly.

### 4.5 Arm

- SO-101 torque limits payload before tipping (0.03–0.09 kg at reach, est); floor perception, calibration and grasp confirmation dominate.
- **Proposed `depth_layer:=height`:** a fused height map so the base sees toys and shoes the default layer misses; needs the real D435.
- **Calibration (synthetic):** tag cube in the gripper, ~60 poses, floor error to p95 0.28 mm.
- **The wrist camera measures the held block** (Nov 7): offset, skew and ride-up within ~1 mm in sim; regrips follow it. VERIFY on the real camera. "In the bin" still means "released over the bin".

### 4.6 Localization, ROS and timing

- Lidar/EKF localization with slam_toolbox; AprilTags for recovery and docking (C apriltag library, decimate 2; OpenCV's AprilTag markers are the official tags turned 180°, so print official images).
- **A2M8 timing (bench, Nov 16):** exact sample clock, open-loop turn rate (12.5 Hz at PWM 660, drifting from cold), parallax-shifted angles, capsule modes deliver a turn 12–49 ms late. sllidar_ros2's late stamps roughly doubled SLAM error and took yaw p95 to 9.7° (sim).
- **The robot's own driver, `omni_cpp` `rplidar_node` (Fernando, Nov 17):** stamps every sample from the device's sample clock; on the real lidar within 0.03 ms median. SLAM in sim back to 3.7 / 8.3 cm, yaw p95 1.5°. **Bins ≈ samples a turn (`bins: 320`):** slam_toolbox scores matches over all bins, so empty bins hurt. slam_toolbox is started by `omni_bringup lifecycle_up`.
- **A2M8 range (bench test 2, Nov 17):** range noise in Stability is ~2 mm at 1.2 m, 5 mm at 2.2 m, 12 mm at 5 m and 37 mm at 7 m, 2–3× below the sim's datasheet model, with no dropouts out to 7 m; within 2.5 cm of a rough tape. Boost is slightly less noisy but drops 8–18 % of far or slanted returns, Express and Standard more. **Stability stays the mode** (test 3 to confirm). Each mode's scan is rotated against Stability's (Boost −1.5°, Express/Standard −3.1°), so base_link → laser's yaw is calibrated per mode.
- Open: calibrate the driver's `latency`, real-room scan fill (test 3), the lidar's 0° at the mount, the sim's range noise and dropouts from tests 2–3. `hardware.launch.py lidar:=true` is off by default. Details: [lidar bench](characterization/lidar/README.md).

### 4.7 Drive, supply and printing

- **AK40-10** (confirmed Nov 2 by the sim motor study): right-sized for 10–20 kg; cheaper motors overheat sooner. **Servo mode, not MIT mode** (Nov 7): MIT's range table silently scales commands if mis-set.
- **Bench (Nov 13, hardware):** the motor caps its own acceleration at ~148 rad/s²; tops out at 368 rpm on 24 V (16% under the datasheet's no-load); acts ~5.5 ms after a command; torque per amp ≈ 0.48 N m/A inferred (VERIFY with a hub mass); no bus voltage on CAN; the status temperature is the driver board's. The sim's AK40-10 is refit to the bench; `MAX_WHEEL_SPEED` 37.2 rad/s. The adult worst case (2 m/s diagonal while turning) is past what a nominal pack gives (est). [components/ak40-10](components/ak40-10/README.md), [wheel bench](simulation/v11%20sil/studies/results/wheel_bench_2026-11-13.md), [CAN bench](simulation/v11%20sil/studies/results/can_bench_2026-11-13.md).
- **Fork pivot level with the axle** (Nov 2): a high pivot turns drive torque into hinge load; collinear doubles start acceleration and cuts cruise power (sim).
- **Wheel noise and wobble (Nov 11–12, sim):** roller play on the axles is the click, so rollers run on 687s with no play (−10 to −18 dB est); tread tip relief reduces the row-handover thump. [Noise](simulation/v11%20sil/studies/results/wheel_noise_2026-11-11.md), [wobble](simulation/v11%20sil/studies/results/wheel_wobble_2026-11-12.md).
- **Power budget (est):** ~123 W typical, ~420 W bursts at the 21 V stop; the computer is not fed straight from the pack. Pack capacity and resistance are claims; wheel acceptance includes fit, strength, wear, grip and noise.

### 4.8 Alternatives not adopted

| Alternative | Lasting reason |
| --- | --- |
| Kiwi, rigid X4, rocker, swerve | X4 + pods for home floors; rocker pitches more, swerve adds size. |
| Optical-flow odometry | Height/speed/rug limits; worse than the gyro from a rolling-shutter camera. |
| MoveIt / Gazebo / Foxglove as main UI | Exact arm planner, MuJoCo and the console kept; Foxglove for debugging. |
| Wrist or portrait-riser D435 | Wrist: weight, minimum range; portrait fits nowhere clear of the arm and lidar. |
| AK40-10 MIT mode; sllidar_ros2 | Silent command scaling; late scan stamps. |
| Bump switches or software contact sensing | 4.13. |

### 4.9 Robot computer and cameras

- **Open (Amazon-only, compact; Fernando):** leading candidates BMAX B8A Pro 8745HS and GMKtec M5 Ultra 7730U; alternatives in the [compute trade](components/computer/compute_purchase_trade_2026-11-02.md). The stack is CPU-bound; a Jetson only for a named CUDA workload. DC input, fit, Linux peripherals, load VERIFY.
- The Teensy owns robot I/O (no GPIO on these PCs). D435, Teensy, SO-101 adapter and the lidar's adapter fill four USB ports; an IMX662 needs a hub.

### 4.10 Pi in the loop; real-ball benches paused

- **Pi 4 (sim, Nov 1–2):** the default stack saturates it; `nav_profile:=pi` (0.6 m/s, lighter SLAM/MPPI) is a slow working arm robot (15/16 goals, 5/6 toys) and the default on a Pi. `rpi.md`.
- The Pi 4 is today's hardware bench: console, motor test, lidar driver, webcam. Real-camera ball benches are paused (Fernando, Nov 1).

### 4.11 Riser D435 for the arm (the arm robot's only camera)

- **Baselined Nov 5 (Fernando):** landscape, 30° down, in a riser pocket; 11–18 mm from the arm in every move, no lidar blind angle added; floor from 0.31 m; 2–3× the nose mount's floor at 1.5–2 m. Riser tidy 24/24 (sim).
- **No tags at 30°**: a tag camera or another tilt is needed. Open: tilt, overhangs, real plug and grazing returns. `studies/d435_mount*.py`.

### 4.12 Forward IMX662 camera at the nose (proposal, Nov 6)

A generic IMX662 USB 2 camera under the lidar, 20° up: tags from 88% of house poses (riser D435 0%), and people (YOLOX nano fused with lidar legs on `/people`, implemented in sim) for Follow and omni_mcp. Real frames, CPU and the ROS node on hardware open. [Report](simulation/v11%20sil/studies/results/imx662_2026-11-06.md).

### 4.13 Bump sensing without switches (sim study, Nov 9)

Rug edges and thresholds look like bumps to the IMU and motor currents; with a mapped floor-edge model a force observer finds fixed contacts, but light movable things are missed. **Decision (Fernando, Nov 9): no bump sensing for now**: no software contact sensing, no rim switches. [Report](simulation/v11%20sil/studies/results/bump_detect_2026-11-09.md).

## 5. Simulation

**`simulation/v11 sil` is the robot-code simulator.** MuJoCo owns physics; the MCU core runs at 1 kHz as SIL; ROS-free cores or the full ROS stack drive it. Use `arm_params()` / `ARM_D435` for arm work. Defaults: realistic D435 depth errors (`d435_noise:=ideal` reproduces earlier runs), the A2M8 lidar stamped as the robot's driver does (`lidar:=a2m12`, `lidar_stamp:=sllidar` reproduce earlier runs), the bench-refit AK40-10 with its 5.5 ms latency, the CAD's 687 rollers. The sim's pack is still the OVONIC (HRB not yet in the catalog). A model change needs regenerated exports and new scores. Limits: unmeasured friction, compliance and motor parameters; simplified floors and optics.

## 6. Where things live

| What | Where |
| --- | --- |
| Working rules, environments, invariants | `AGENTS.md`; scoped `AGENTS.md` in `cad/`, `ros/src/omni_console/` |
| Sim model and studies | `simulation/v11 sil/`: `params.py`, `catalog.py`, `robot_export.py` (generates `robot.yaml` geometry, `fw_params_arm.h`, `robot_arm.json`); `studies/`, results in `studies/results/` |
| CAD: working | `cad/v15/` (mid deck): `regen_all.sh`, `tools/` (map in `tools/README.md`), `step_export/`, `solidworks/V15_robot.SLDASM`, colours `tools/palette.py` |
| CAD: last adopted, history | `cad/v14/` (layout F, 687 wheel; scripts and STEPs); `cad/v13/` (harness README, jigs, coupons, studies); `cad/v10 human editable/` (native SolidWorks); `cad/README.md` lists every version |
| CAD rules | `cad/solidworks.md` (main reference; printed-part rules in 18), `cad/AGENTS.md` |
| Firmware | `firmware/`: `lib/omni_fw` core, `src/main.cpp`, `sil/`, `PINS.md`, `bench/` (UNO R4 benches) |
| Electronics | `pcb/v1/`: the Teensy carrier (KiCad 10, generated by `tools/gen_sch.py`, checked by `tools/check.py`) and the robot's block diagram `robot_block_diagram.html` |
| Robot software | `ros/`: `omni.sh`, `README.md`; `omni_cpp` (mcu_link, mode_manager, scan_prep, rplidar_node, arm_driver), `omni_bringup` (`config/robot.yaml`, `hardware.launch.py`), `omni_console`, `omni_arm`, `tools/omni_mcp`, `tools/` (smokes, `robot_console.sh`, `wheel_capture.py`) |
| Parts and purchases | `components/`: `BOM.md`, `ak40-10/`, `battery/`, `computer/`; `database/` (shared sourced data) |
| Benches | `characterization/lidar`, `apriltag`, `pi_camera`, `realsense` (ball benches paused); Pi setup `rpi.md` |
| Open small tasks | `todo.md` |
| Launchers | `Launcher.cmd`, `launcher.ps1`, `launcher_full.ps1` |

## 7. Open decisions and VERIFY

- **CAD:** adopt `cad/v15` or not; the SO-101's stow collision and arm/head heights; face location; bumper redesign; spring rate; strength of fins, riser, deck; dovetail and pilot fits on prints.
- **Drive/power:** AK40-10 torque per amp and the loaded curve; its CubeMars settings (acceleration cap, ERPM limit, comm-loss timeout); top speed on the pack; real connectors and leads; the 19 V converter's input floor; the HRB's sag; the main switch's DC rating and inrush (few latching e-stop buttons carry 20–30 A DC: a rated disconnect, or the button driving an anti-spark switch), and whether everyday power-off should shut the computer down first.
- **Arm perception:** reach and pose error on the real arm, wrist correction, grasp confirmation, a bin check; the `height` layer on the real D435; the D435's tilt and a tag camera.
- **Fetch to a person:** `ai_smoke.py --only fetchme` delivers 2/5; `/people` rarely confirms the person during Fetch's look round. Start with the people node's confirmation count.
- **Lidar:** driver `latency`, real-room scan fill against slam_toolbox's thresholds, 0° at the mount, mode.
- **Computer:** choose from 4.9 by task results with the real D435 and voice; power and cooling; `unattended-upgrades` policy.
- **Console with no page:** the robot stays armed after the last page closes (motion stops within 0.3 s); decide whether to disarm after N s.
- **Physical calibration:** roller friction, springs, encoder signs, IMU, lidar tilt and timing, sync pulse, frames.
- **Parked catch:** cameras, trigger, calibration stability, feed motor, packaging.

## 8. Path to the physical robot

One unit before four; feed each measurement into the sim and rerun with the same robot code.

1. **Reconcile the build definition:** v15 vs v14, deck/pod mounting, arm/head heights, stow and path clearance; rerun picking, stops, crossings.
2. **Compute/perception:** score routes and navigate-then-tidy on the candidate computer with the real D435 and face loads.
3. **One wheel/pod and drive:** print fits, then a wheel and pod (grip, wear, springs, crossings) on the pod jig; AK40-10 settings through the console's bring-up, current-limited. Fit the models before ordering a set.
4. **Power and rolling base:** fuse/switch/`BUS_EN`, rails; wheels up, battery, slow floor driving with arm mass; compare with the sim.
5. **Navigation, then picking:** calibrate sensors and frames, map, soak; calibrate the arm; verify stow and waypoints; navigate-then-tidy with logs.
6. **Catch later**, by explicit resumption.

Physical sessions are attended: current-limited and wheels up first, low speed before adult profiles.

## 9. Decision log (key choices)

Every dated entry is in the history's Nov 7 and Nov 17 snapshots.

| Date | Decision |
| --- | --- |
| Oct 17–25 | Printed-first X4 home robot, Teensy MCU with a firmware core shared with the SIL, ROS 2 stack, robot-served console; SO-101 tidy payload with an exact planner. |
| Oct 31 | X4 with sprung pods and big-roller 140 mm PLA/TPU wheels (Fernando). |
| Nov 1 | Focus on the arm robot; catch and real-ball benches paused. 6S pack, 21 V stop. |
| Nov 2 | AK40-10 drive confirmed (sim); fork pivot level with the axle; `nav_profile:=pi` on a Pi; computer from a compact Amazon shortlist (still open). |
| Nov 3 | Layout F (Fernando). |
| Nov 4 | AI control as a toolkit: the robot gives primitives and senses, the model plans (Fernando). |
| Nov 5 | **The D435 on the arm riser is the arm robot's only camera** (Fernando). |
| Nov 7 | One implementation per robot node, C++ where it exists; tunables as described ROS parameters; console by intent; AK40-10 in servo mode. |
| Nov 9 | No bump sensing (Fernando). |
| Nov 11 | Harness routed only through fin shadows; Fernando's power rails. |
| Nov 13 | 687-bearing roller wheel (`cad/v14`); printed-part rules (no captive nuts, holes through, no dowels); the robot knows its health and greys what can't work (Fernando); first wheel on the Teensy and the sim's motor refit to the bench. |
| Nov 14 | `cad/v15` mid-deck proposal; it has become the working CAD. |
| Nov 16 | HRB 6S 3300 pack; RPLIDAR A2M8 (the unit he has); colours on v15 (Fernando); every printed part fits 252 mm, checked on the solid. |
| Nov 17 | The robot's own lidar driver `rplidar_node` replaces sllidar_ros2 (Fernando); lidar bins ≈ samples a turn. The BNO085 on the Teensy uses the wheeled-leg robot's proven SPI driver (PS0/PS1 strapped, reset-as-wake), at 100 Hz: the IMU only feeds heading and the EKF, nothing balances on it; the v11 sim's BNO085 reports at 100 Hz too (was 400; `imu_rate=400` reproduces earlier runs, not rerun). |
| Nov 17 | A2M8 bench test 2 (walls 1–7 m): mm-level range noise, Stability without dropouts stays the scan mode; each mode rotates the scan (to −3.1°), so the lidar's yaw is calibrated in the robot's mode. |
| Nov 17 | **Power simplified (Fernando):** one latching main switch before the fuse is the e-stop and cuts everything; no contactor, power board or motor fuses (the AK40-10s limit their own current). Teensy carrier `pcb/v1` started: all through-hole, JST XH, CAN modules, Teensy on USB power. |
| Nov 18 | **Power down (Fernando), drawn in `cad/v15`:** the battery's path and the 12 V / 5 V converters on the first floor (the deck's free area holds exactly two converter-sized blocks, one per side lens), the 19 V converter by the computer on floor 2, the computer under the bin, the carrier beside the arm riser; the deck, pods and fins unchanged. The harness re-routed: 6 lidar-plane crossings, all in fin shadows, none of them battery or drive leads. |
