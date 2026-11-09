# Omni Robot — Design Baseline

Rewritten Nov 1, 2026; compressed again Nov 7. Current focus: the v10 arm robot. Catch development is paused.

**Rules of the road — keep this a design baseline.**

- Every session that changes the baseline or establishes an important finding must update this file before finishing. Update the matching section, `Start here` if the status changes, and the condensed decision log when a lasting decision or major finding changes the project direction.
- Rewrite the existing statement in place. Keep the current choice, why it was made, the decisive evidence and what remains uncertain. Do not append session reports, debugging narratives or successive versions of the same conclusion.
- Separate **adopted**, **proposed**, **implemented but unvalidated**, and **paused**. Software defaults, a CAD export and Fernando's adoption are different facts; say which applies. Do not silently promote a proposal.
- Attach results to the robot configuration and test conditions that produced them. Label simulation, hardware measurements, estimates **(est)** and physical checks **VERIFY**. Older scores do not validate a changed robot.
- Keep detailed procedures, run tables, CAD revisions and implementation notes in the existing subsystem READMEs and study results; link them here. Keep the main document around 4,000 words. Replace detail with a conclusion and source when it grows.
- Before removing substantive material, append it or a complete dated snapshot to [design full history.md](design%20full%20history.md). Preserve earlier history and its section numbers. Do not start parallel design documents or edit superseded documents in `archive/`.

**History.** [design full history.md](design%20full%20history.md) keeps every earlier version: Appendix B (before the Nov 1 compression) and **"November 7, 2026 — design.md snapshot before the compression to ~4,000 words"** (run tables, CAD rounds, Pi runs, mount and tag studies, the full decision log). `§1–31` in code means the history's sections; legacy `design.md 10–16` its Appendix A. Section numbers below are unchanged.

## Start here

- **Active baseline:** `robot_export.ARM` / `arm_params()`, the v11 sim's coarse model of `cad/v10 human editable`: X4, 140 mm omni wheels, sprung forks pivoting level with the axle, AK40-10 drives, SO-101 and bin, the D435 on the arm riser. Nothing is built; results are simulation unless marked.
- **Supply:** OVONIC Roam 6S 5200 mAh LiPo (estimates). The AK40-10 driver (servo mode) is written and SIL-tested, not run on a motor (2.1).
- **CAD in progress, not ready to print:** layout F proposals in `cad/v12`/`v13`; the CAD fork pivot, thin wheel webs and the arm's stow collision are open (4.4, 7).
- **Compute and cameras:** computer open (4.9). **The riser D435 is the arm robot's only camera** (Fernando, Nov 5): floor from 0.31 m, no wall tags, so a tag camera (IMX662 proposed, 4.12) or another tilt is still to be chosen (4.11). A Pi 4 drives a slow working robot with `nav_profile:=pi` (4.10).
- **Catch and real-camera ball benches are paused by Fernando** until stereo global-shutter colour cameras and a mobile robot exist.
- **Software (sim only):** console by intent with preflight and bring-up, AI control through `omni_mcp`, people detection, fun missions. MuJoCo with the robot's own arm and firmware code: tidy 112/120 binned; Fetch 3/3; a 3-block tower on 5 of 6 seeds; Goalie 13/13 on-target saves (3). All of it ran on the full ROS stack on the PC on Nov 8 (sim, single runs; 3).
- **Next:** CAD/model gaps, one wheel/pod on the bench, arm navigation and picking on the candidate computer, then the rolling arm platform (8).

## 1. Goal

A mostly printed indoor home robot with omnidirectional driving, lidar SLAM, navigation, RC and browser control, a face and voice, and swappable payloads. **The immediate build is the SO-101 arm robot for tidying floor objects.** The parked catch payload tracks a 74 mm foam ball, catches it and returns it to a thrower 6–7 m away. Keep the shared platform useful for both without making the catch robot a prerequisite for the arm.

Use the robot's own code in simulation before physical validation. Real first runs and flashes are attended; measured results must feed back into the models.

## 2. Baseline design

Dimensions describe the **current arm simulation** (nominal CAD, estimated mass), not a built robot.

| Subsystem | Current design and status |
| --- | --- |
| Base | X4, wheel centres r 146 mm, bumper r 225 mm, payload plate 181.5 mm high; arm model 9.63 kg, CG z 126 mm (est). |
| Wheels | 140 mm, two rows of six 36 mm rollers; PLA hubs, TPU treads, two 6804s. Fit, strength, wear, grip untested. |
| Suspension | Sprung forks, −3/+15 mm. **Pivot level with the axle** (sim, Nov 2; 4.7); the CAD fork must follow. |
| Drive | 4 × CubeMars AK40-10 V3.0 KV170 (CAN, 10:1), confirmed Nov 2 by the sim motor study. 6S torque/thermal, comm loss, bearing load VERIFY. |
| Battery | OVONIC Roam 6S1P 5200 mAh, 115 Wh, 706 g; ramped stop at 21 V (provisional); 20% reserve in runtime estimates. |
| Arm | SO-101 at x 140 mm on a 37 mm riser; finger pads, wrist camera (physical one TBD), rear bin, checked waypoints; floor reach ~0.26–0.35 m. |
| Cameras | **D435 on the riser's front**, landscape, 30° down, centre (190.8, 0, 193.0) mm (`ARM_D435`; nose kept as `ARM_D435_NOSE`). IMX662 forward camera proposed (4.12). |
| Lidar / IMU | A2M12 at the front (scan plane z 163 mm); BNO085 on the MCU. |
| Computer / MCU | Computer open (4.9). Teensy 4.1: safety, 1 kHz control, odometry, CAN; board-free core shared with SIL. |
| Interface | Robot-served console (2.1), gamepad, RC as override; 5 in face, microphone, speaker (TBD). |
| Electronics / printing | Proposed Teensy carrier with `BUS_EN`; fused switching and rails to review. PLA/PETG/TPU, no inserts, M3 thread-forming; `cad/solidworks.md`. |

**Safety/control (SIL only):** stops ramped; 0.5 s link watchdog. In AUTO, a pack under 21 V, a motor error or a silent motor latches `FAULT_STOP` until DISARM at ≥ 22.2 V; RC manual is not stopped and is left with a ramp. Invalid CMDs are dropped. Kid profile 0.8 m/s, on at once, off only at rest; adult 2 m/s; stale `/arm/out` caps the base as arm-out (1 m/s). MCU watchdog 0.5 s. No bumpers yet. The arm refuses unchecked motions and holds (never torque-off) on bad servo feedback. Thresholds are estimates.

**Parked catch kit:** net funnel, padded pocket, chute feed, IR beams, tilt, two Ø70 flywheels; feed actuator unresolved; v7-based model.

### 2.1 Software

- **Firmware:** board-free `firmware/lib/omni_fw`; COBS/CRC USB protocol 6, heartbeat, timestamped events, sync pulse. `pio run -e arm` / `-e catch`. **AK40-10 driver (Nov 7): servo mode, not MIT mode** — ERPM velocity on the motor's own loop, zero current to coast, its status broadcast for odometry. MIT's range table, if mis-set, silently scales commands (the wheeled-leg robot's AK45-10s ran 2.5× off) and the AK40-10's couldn't be checked. One CAN frame per tick round-robin. Wheel speed capped by the 60000 ERPM limit (44.9 rad/s). CubeMars-tool settings are on the bring-up list; pole pairs and the limit VERIFY.
- **ROS 2 Jazzy:** MCU link, mode manager, scan prep, EKF, slam_toolbox, Nav2/MPPI, arm, face, console, people. One implementation per node, C++ (`omni_cpp`) where it exists (Nov 7); Python cores the sim imports keep C++ parity tests; the sim's tidy runs the C++ arm driver. TELEOP sources by priority: `/cmd_vel/joy` > `/cmd_vel/manual` > `/cmd_vel`. Tunables are described ROS parameters in `robot.yaml`; geometry is generated by `robot_export.py`.
- **Arm:** `omni_arm` (kinematics, motion, floor/wrist vision, coverage, tidy/carry/place/tower); `omni_cpp` `arm_driver` owns the servo bus. No MoveIt.
- **Console (adopted Nov 7, Fernando; run on the ROS stack Nov 8, sim):** workspaces Home, Drive, Map, payload, See, Test, Robot; features declared by the robot, so a payload needs no page edit; every button a verb the AI and voice share; one activity history; a status bar with the control ladder (e-stop > RC > stick > task > AI) and the speed cap. Preflight refuses only starts that would fail (MCU, protocol, battery, localization, arm, lidar). Call: camera, drive, push-to-talk, and Listen to the robot's microphone after it announces it; a self-signed https port enables the phone's microphone.
- **AI control (sim; [ai_robot_architecture.md](ai_robot_architecture.md)):** Claude or Codex drive by task through `ros/tools/omni_mcp`; the robot offers primitives and senses, the model plans (Fernando). Lease, take-over, deadlines, request ids. Hardware not started.

## 3. Expected performance and evidence

**No v10/6S hardware validation exists.** All rows are simulation unless marked; single runs per seed unless stated.

| Configuration / evidence | Result and limitation |
| --- | --- |
| v10 arm geometry | The 37 mm riser restores fingers-down floor reach (0.26–0.35 m); CAD's 231.5 mm arm base gave none. |
| Riser D435 tidy at scale, MuJoCo pick harness (Nov 7; 6 seeds × {toys, clutter} × {arena, house}, 5 objects, 420 s; C++ arm driver and firmware, D435 depth errors) | **112/120 binned (93%)**; 88 s per object. Misses: releases counted beside the bin (the count is releases, not bin truth), one failed grasp, one never reached. `studies/arm_pick.py`, `arm_pick_d435.json`. Earlier stereo 102/120 is the catch robot's camera. |
| Fun missions, same harness (Nov 7) | **Fetch** 3/3 put down upright 1–5 mm from the spot. **Block tower** (3 × 40 mm cubes, 6 seeds): 5 stood 3 high, one 2 high (the depth look read its top at 40 mm for 81); layers 3–17 mm off; 72–81 s a tower. Before the gripper measurement (below): 4/6, 4–22 mm, 119–139 s. **Goalie** (22 cm ball from 3 m at 1.5–3 m/s): 13/13 on-target shots saved over seeds 1–2, against 3/14 parked. `studies/arm_carry.py` (`arm_carry_*_held.json`), `goalie.py`. |
| The same missions on the ROS stack (Nov 8, PC, riser D435, one run each, scored on the sim's truth) | Fetch: the asked cube carried 0.8 m and put down. Block tower: 3 high, layers within ~10 mm. Tidy: 5/6 binned in 222 sim-seconds. Dance and Messenger complete (with the camera it waited for a camera-confirmed person, lidar-only it went to the walking person). Goalie runs, tracker on and off, but no ball exists on the ROS sim: saves remain MuJoCo-only. Call's clip, face speech and https work; its microphone needs `arecord`, absent in WSL. Limit: Fetch turns but never drives closer, so a 30 mm cube ~1.4 m away stays unfound. |
| PC robustness, full Nav2/SLAM (Nov 3–5; both arm cameras, house) | 8/8 and 16/16 route goals, true arrival 0.04–0.14 m, localization ≤ 0.14 m; navigate-then-tidy 6/6 (D435); console crash recovery 0.09 m. The stereo robot pushed floor toys (no depth layer). [Report](simulation/v11%20sil/studies/results/pc_robustness_2026-11-05.md). |
| Near-field obstacles (Nov 5; D435 noise, nose mount, 3 seeds) | Default `raw` layer drives over 7/12 floor toys and pushed a shoe 1.24 m; proposed `height` layer 48/48 goals, 6/6 shoes avoided, routes ~15–30% longer (4.5). |
| AI control (Nov 4–5) | All `ai_smoke.py` checks pass on both arm cameras (stop 2.8 s after the server dies, take-over, gotos within 9 mm, colour-shape pick); find offline 108/140 right, 0 wrong. |
| Endurance and drive (Nov 1–2; AK40 fits, 10/15/20 kg) | ~4.2 m/s forward / 3.2 diagonal (back-EMF); 50 ft hallway 9.0 s at the 2 m/s cap; ~71 min touring to 21 V at 10 kg **(est thermal)**; cruise 68–76 W → roughly an hour of mixed use **(est)**. Collinear pivot: start 2.6 → 6.2 m/s², slope 18 → 28° (4.7). |
| Earlier v7 catch / navigation | 93.1% sim game, 88.0% SIL over 216 throws; 8/8 goals, 7–9 cm localization, a two-hour 43-lap soak with no crash or leak. Earlier configuration. |

Sources: v11 `studies/`, `ros/tools/soak.py`, `arm_load.py`, `characterization/pi_camera/results/`; full tables in the history.

## 4. Major findings from trades and simulations

### 4.1 Perception limits the catch (parked)

Latency, frame rate and stereo calibration stability dominate: simulated drift 0.04° → 97% catches, 0.3° → 78%, 0.5° → 58%. Use a stiff common bar, global shutter and synchronized exposure. Live D435 trials used a larger ball and hit lighting and false-trigger limits; they did not validate tracking the 74 mm ball at 6 m. History §§22–23; `characterization/throw_track/README.md`.

### 4.2 Base

**X4 with sprung pods** (Fernando, Oct 31). Kiwi was worse at catching and braking; rigid X4 lost 16 catch points on a diagonal rug and was sensitive to a wheel 1 mm low; a rocker pitched twice as much on thresholds. Big rollers matter at sideways edges. These are older wheel results, not validation of the 140 mm wheel. `studies/base_trade.py`, history §25.

### 4.3 Catch module and launcher (parked)

Pocket, chute feed and 35° tilt took static catches from 80% to 99%; keep one-ball feeding. Ceiling-aware returns reach 7 m under an 8 ft ceiling at 35–40° / 4,500 rpm (sim). No catch hardware is needed before the arm robot.

### 4.4 Layout and CAD

- Keep the lidar forward and its layer clear; check clearances with the SO-101's real meshes (its shoulder overhangs the front edge, so no face in front of the arm).
- **CAD/sim gap:** the sim's arm base is 218.5 mm on a 37 mm riser; v10's stow pose collides. Reconcile before trusting the sim's waypoints on the assembly.
- **Pod (`cad/v11`, proposed):** wheel + AK40-10 module, motor outboard (its XT30 mates radially), fork hinged level with the axle, bed springs and hard stops. Motor bearing and spring rates VERIFY.
- **Layout F (chosen Nov 3; STEP-only, `cad/v12`, `v13`):** Ø450, frame chassis, pods out from below, electronics and battery drawer at the rear under the bin, face behind the bin, lidar fins as wedges in their own shadow (20.9° blind), arm/riser/bus board one module. Rules from Fernando's reviews: pilots along the print axis (else nuts or a full Ø2.9 pilot), nothing under 4 mm or sharper than ~75°, dovetails 0.1 mm over, model real connectors. Checks: pairwise overlaps, SolidWorks interference, hole audit. Nothing printed or load-tested.

### 4.5 Arm

- SO-101 torque limits payload before tipping (0.03–0.09 kg at reach, est). Floor perception, calibration and grasp confirmation dominate.
- **Near-floor navigation (Nov 5):** the default `raw` layer sees no toy and lost a shoe. **Proposed `depth_layer:=height`:** a fused height map (floor/step/low/obstacle/drop/unknown on relief over the local floor), speed by certainty. Needs the real D435 on house floors, furniture, drops and CPU. The sim can't score pushing toys.
- **Calibration (synthetic, `omni_calib.arm_calib`):** a tag cube in the gripper at ~60 poses: floor error 7–33 mm → p95 0.28 mm keyed; sag terms matter; reach sits at the lift's limit.
- **Carry and stack (sim):** a keep-out ring round the tower, slide off before rising, carry high, close across faces, regrip when askew. **The block's place in the gripper is measured (Nov 7):** the wrist camera rides on the gripper (joint calibration cancels) and sees the block's near side; on the bottom face's plane its near edge is exact, giving the offset across the pads, the skew and how far it rode up. The centre sits half the block's side from the fixed finger (the gap reads 45.7 mm for a 40 mm cube). In the sim the cubes sat 0–6 mm off centre and rode 2.5 mm up; it reads them within ~1 mm. Regrips now follow the measured skew: the gap rule had regripped every block. Tower 4/6 → 5/6 seeds, 25–40% faster; Fetch 5–10 → 1–5 mm (3). VERIFY with the real wrist camera.
- "In the bin" still means "released over the bin".

### 4.6 Localization, ROS and timing

- Keep lidar/EKF localization; AprilTags help recovery, gaps and docking. Single-view relocalization can pick a mirror pose.
- **Real tag bench (Nov 4, Pi 4, still tag):** locks every frame on the Arducam, the D435 IR (projector off; on, it spoils tags) and colour; corner noise 6–15× below the sim's model; `omni_slam.graph`'s `sigma_px` 0.7 keeps a margin for motion. **Proposed:** the C apriltag library instead of OpenCV aruco (3× faster, better orientation); keep `min_margin` ≥ 30. Motion blur, range sweeps and the tag family are open. [Bench](characterization/apriltag/README.md).
- **Recovery (sim):** SLAM restarts reject stale transforms; the console reaps a leftover slam_toolbox and resumes from the last localized pose on the same MCU boot (0.09 m after a kill). A saved map still assumes its start unless given a pose; composed Nav2 can hang at shutdown.

### 4.7 Drive, supply and printing

- **AK40-10 (Oct 30, confirmed Nov 2):** the model is fitted to the maker's 24 V chart (2.13 Ω effective, 0.12 N·m friction, 4.1 N·m peak; the datasheet DC model was ~3× optimistic). Speed above 2 m/s pays only on long straights. At 20 kg route RMS torque exceeds rating (1.5–1.9 vs 1.3 N·m). Of cheaper motors only the 5 N·m DM-2325 comes close and overheats sooner. Right-sized for growth to 15–20 kg. `studies/drive_char.py`, `results/motor_char/`.
- **Fork pivot level with the axle (Nov 2):** the drawn 43 mm-high pivot turns ≈0.6 of drive torque into hinge load (bouncing, wheels fighting); collinear: start 2.6 → 6.2 m/s², slope 18 → 28°, cruise power 55 → 41 W at 10 kg. Acceleration numbers are criterion-sensitive (±30%). `studies/pivot_quick.py`.
- Pack capacity and mass are claims; R0 and OCV are estimates. Do not carry old 7S results to 6S. Wheel acceptance includes roller clearance, fit, loaded strength, wear and grip.

### 4.8 Alternatives not adopted

| Alternative | Lasting reason / status |
| --- | --- |
| Kiwi, rigid X4, rocker, swerve | X4 + pods for home-floor stability; rocker pitches more, swerve adds size. |
| Optical-flow odometry | Height/speed/rug limits; from a rolling-shutter camera worse than the gyro (4.12). |
| MoveIt / Gazebo / Foxglove as main UI | Exact arm planner, MuJoCo and the console retained; Foxglove for debugging. |
| Wrist or portrait-riser D435 | Wrist: weight, minimum range; portrait on the riser fits nowhere clear of the arm and lidar (4.11). |
| AK40-10 MIT mode | Unverifiable range table silently scales commands; servo mode used (2.1). |

### 4.9 Robot computer and cameras

- **Computer open (Amazon-only, compact; Fernando):** BMAX B8A Pro 8745HS ($436.99 captured) for performance, Getorli AT103 i5-13420H ($379.99) for budget, GMKtec M5 Ultra 7730U ($449.99) for documented interfaces; all 16 GB, replaceable RAM, four A + one C. DC input, audio jacks and Getorli's microphone VERIFY. [Compute trade](components/computer/compute_purchase_trade_2026-11-02.md).
- **Basis:** Teensy keeps low-level control and safety; navigation and grasp corrections stay local; heavy AI optional on the desktop. The stack is CPU-bound; a Jetson only for a named CUDA workload.
- **Ports:** D435 (USB 3), Teensy, SO-101 adapter and A2M12 adapter fill four ports; the proposed IMX662 needs a fifth (a hub). No general GPIO on these PCs: the Teensy owns robot I/O; pins 24/25/26 reserved for camera trigger/sync.
- **Catch head deferred:** ~$500 **(est)** for a Pi 5 + colour AR0234 pair or an OAK; pair throughput VERIFY.

### 4.10 Pi in the loop; real-ball benches paused

- **Pi 4 (Nov 1–2, sim at ~0.7× real time, wired):** the default stack saturates it; `nav_profile:=pi` (0.6 m/s, slam with loop closing every 0.25 m, lighter MPPI and costmaps, `arm_node` 20 Hz) gives a slow working arm robot: 15/16 goals, 0.16 m localization, 5/6 toys binned, the busiest core pinned while driving. It is the default on a Pi. Slam cuts cost localization; AMCL did worse. The C++ arm driver cut the arm's idle load ~7× on the PC. Real D435, voice and repeats not run. `characterization/pi_camera/results/`, `rpi.md`.
- **Real-camera ball benches paused by Fernando (Nov 1)** until stereo global-shutter colour cameras and a mobile robot exist; then test the 74 mm ball thrown from 6 m.

### 4.11 Riser D435 for the arm (the arm robot's only camera; on the arm riser since Nov 5)

- **Baselined Nov 5 (Fernando):** landscape, 30° down, centre (190.8, 0, 193.0) mm in a riser pocket through a floor notch (`cad/v13`). 11–18 mm from the arm in every move, no lidar blind angle added, inside the bumper; floor from 0.31 m.
- **Why:** height gives 2–3× the nose's floor at 1.5–2 m. Portrait saw more but fits nowhere clear of arm, lidar and bumper.
- **Tidy:** riser 30° 24/24 against the nose's 22/24 after a fake-rug fix; the tidy covers its region (`coverage.py`). The wrist camera, not the D435, refines grasps.
- **Tilt:** 35° looks closer but loses far obstacles; **no tags at 30°** — a tag camera or another tilt is needed.
- **Open:** tilt and tag camera, coverage reruns and a house run, overhangs, the adapter over the pocket, real plug and grazing returns. `cad/v13/README.md`, `studies/d435_mount*.py`.

### 4.12 Forward IMX662 camera at the nose (proposal, Nov 6; sim and CAD study only)

- **What:** a generic IMX662 USB 2 camera (90 × 58°, rolling shutter) under the lidar, 20° up, for tags, people, the room; fits at every pose checked. [Report](simulation/v11%20sil/studies/results/imx662_2026-11-06.md).
- **Tags:** 88% of house poses (riser D435 0%); decodes at 2 m/s with 4 ms exposure, pose 2–3× worse at a 33 ms readout; spins ≥ 120°/s blur out. Optical flow is not worth it.
- **People (implemented, sim):** YOLOX nano fused with the lidar's legs, tracked on `/people` for Follow and omni_mcp; every person at 2–4 m, few under 1.5 m; the lidar ranges, the camera rejects chair legs. Follow at 1.5–2 m. [Study](simulation/v11%20sil/studies/results/people_detect/summary.md).
- **Open:** real frames and readout, the ROS node unrun, CPU, identity.

## 5. Simulation

**v11 sil is the current robot-code simulator.** MuJoCo owns physics; the MCU code runs at 1 kHz as SIL; ROS-free robot cores or the ROS stack drive it. Use `arm_params()` / `ARM_D435` for arm work, `ARM_V7` for earlier layouts, `drawn_params()` for v7 catch references. A model change needs regenerated exports and new scores. D435 depth has realistic errors by default (`depth_noise.py`, estimates until measured; `d435_noise:=ideal` reproduces earlier runs). Limits: unmeasured friction, compliance and motor parameters; simplified floors; renders omit real optics and lighting. Compare identical real/sim sessions by logs.

## 6. Where things live

| Location | Purpose |
| --- | --- |
| `simulation/v11 sil/` | `params.py`, `catalog.py`, `robot_export.py`, SIL, worlds; `studies/` for scored comparisons. |
| `cad/v10 human editable/`, `cad/v11`–`v13` | Native SolidWorks robot; pod and layout-F proposals. `cad/solidworks.md` is the modelling contract. `cad/v9`, `v7` frozen. |
| `firmware/` | Teensy core, drivers, SIL build, pin map, generated headers. |
| `ros/` | `omni.sh`, robot packages, console, bridges, `tools/omni_mcp`; package READMEs hold procedures. |
| `characterization/`, `rpi.md` | Computer and camera benches (ball benches paused). |
| `database/` | Shared sourced parts data; no omni-robot imports. |
| `Launcher.cmd`, `launcher_full.ps1` | Fernando's launchers. |

Environments, commands and invariants: `AGENTS.md` and the subsystem READMEs.

## 7. Open decisions and VERIFY

- **CAD:** redraw the fork pivot level with the axle; wheel web strength; arm/head heights and the SO-101's stow collision; face location; battery drawer vs charge port; no strength analysis of fins, riser, keel or the adapter over the D435; dovetail clearance and thin pilot walls to print and adjust.
- **Drive/power:** CubeMars settings (servo mode, broadcast, timeout), comm-loss stop, pole pairs and ERPM limit, max supply voltage, output-bearing load, speed-loop gains, motor heat in a printed hub; measured LiPo capacity/sag, fuse, switch, rails.
- **Arm perception:** floor reach and pose error on the real arm, wrist correction, grasp confirmation, a bin check after release; the `height` layer on the real D435; the D435's tilt and a tag camera; where a block sits in the real gripper (the wrist camera's image, VERIFY).
- **People and AI:** the people node on the candidate computer (it ran in ROS on the PC Nov 8, sim); real IMX662 frames; label the real house's rooms; phase 5 hardware; voice; Remote Control; Nav2's 8–9° goal heading tolerance (follow with a turn).
- **Computer:** choose from 4.9 by task results with the real D435 and voice; DC power and cooling.
- **Physical calibration:** roller friction, springs, encoder signs, IMU, lidar tilt and timestamps, sync pulse, frames.
- **Parked catch:** cameras, trigger, calibration stability, feed motor, packaging.

## 8. Path to the physical robot

One unit before four; feed each measurement into the sim and rerun with the same robot code.

1. **Reconcile the build definition:** wheel webs, deck/pod mounting, arm/head heights, stow and path clearance; rerun picking, stops, crossings.
2. **Compute/perception:** on the candidate computer, score routes and navigate-then-tidy in real time with camera and face loads; choose by task results.
3. **One wheel/pod and drive:** fit samples, then a wheel and pod (grip, wear, springs, crossings); an AK40-10 set up in the CubeMars tool through the console's bring-up, on a current-limited supply. Fit the models before ordering a set.
4. **Power and rolling base:** fuse/switch/`BUS_EN`, rails; wheels up, battery, then slow floor driving with arm mass; compare with the sim.
5. **Navigation, then picking:** calibrate sensors and frames, map, soak; calibrate the arm (tag cube); verify stow and waypoints; navigate-then-tidy with logs.
6. **Catch later**, by explicit resumption.

Physical sessions are attended: current-limited and wheels up first, low speed before adult profiles.

## 9. Decision log (lasting choices only)

The full dated log is in the history's Nov 7 snapshot.

| Date | Decision / major finding |
| --- | --- |
| Oct 17–25 | Printed-first X4 with big-roller sprung pods, ramped stops; A2M12, BNO085, Teensy; shared firmware/SIL core, ROS stack, robot-served console; SO-101 tidy payload with exact planner and wrist correction. Catch studies kept for the parked payload. |
| Oct 27–31 | Nav2 to 2 m/s; C++ cores to save compute; AK40-10 replaces DM-2325; sprung pods and a 140 mm PLA/TPU wheel retained; native SolidWorks v10. |
| Nov 1 | Focus on the arm robot; catch and real-ball benches paused until global-shutter colour stereo and a mobile robot. `ARM` baselined to coarse v10 (37 mm riser); 6S pack, 21 V stop. |
| Nov 2 | AK40-10 confirmed (sim); fork pivot level with the axle (torque jacking). Amazon-only compact computer shortlist (not adopted). `nav_profile:=pi` the Pi default. Pod motor outboard of the hub. |
| Nov 3 | Pod with bed springs and hard stops; layout F chosen (Fernando); C++ `arm_driver`. |
| Nov 4 | AI control as a toolkit, not scripted missions (Fernando). CAD print rules from rounds 13–14. Real AprilTag bench; the apriltag library proposed. |
| Nov 5 | **The D435 is the arm robot's only camera** (Fernando), baselined on the riser (landscape, 30°) after portrait failed the CAD check. Realistic D435 noise default; `height` layer proposed. Lidar supports as wedges in their own shadow. |
| Nov 6 | Fake-rug fix (riser tidy 24/24); the tidy covers its region (Fernando). Forward IMX662 proposed. |
| Nov 7 | One implementation per robot node, C++ where it exists; tunables as described ROS parameters; console by intent (Fernando). People detector, fun missions, AK40-10 servo-mode driver (sim/SIL). The wrist camera measures the held block. Design compressed to ~4,000 words. |
| Nov 8 | Nov 7 work tested on the ROS stack and merged. Fetch and Messenger now act only on a real match: a near-miss score or a lidar blob had reported false successes. |

