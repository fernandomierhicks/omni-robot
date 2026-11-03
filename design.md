# Omni Robot — Design Baseline

Rewritten Nov 1, 2026. Current focus: the v10 arm robot. Catch development is paused.

**Rules of the road — keep this a design baseline.**

- Every session that changes the baseline or establishes an important finding must update this file before finishing. Update the matching section, `Start here` if the status changes, and the condensed decision log when a lasting decision or major finding changes the project direction.
- Rewrite the existing statement in place. Keep the current choice, why it was made, the decisive evidence and what remains uncertain. Do not append session reports, debugging narratives or successive versions of the same conclusion.
- Separate **adopted**, **proposed**, **implemented but unvalidated**, and **paused**. Software defaults, a CAD export and Fernando's adoption are different facts; say which applies. Do not silently promote a proposal.
- Attach results to the robot configuration and test conditions that produced them. Label simulation, hardware measurements, estimates **(est)** and physical checks **VERIFY**. Older scores do not validate a changed robot.
- Keep detailed procedures, run tables, CAD revisions and implementation notes in the existing subsystem READMEs and study results; link them here. Keep the main document around 4,000 words. Replace detail with a conclusion and source when it grows.
- Before removing substantive material, append it or a complete dated snapshot to [design full history.md](design%20full%20history.md). Preserve earlier history and its section numbers. Do not start parallel design documents or edit superseded documents in `archive/`.

**History and old references.** The complete pre-compression document is preserved verbatim in **Appendix B, “design.md snapshot before the Nov 1, 2026 compression.”** Earlier code references to `§1–31` refer to the history's original sections; legacy `design.md 10–16` means Appendix A (timing, arm, layout, offloading, CAD v8, electronics, base trade). Use Appendix B for the detailed Oct 28–Nov 1 reports formerly in this file. Numbered sections below describe current status.

## Start here

- **Active baseline:** `robot_export.ARM` / `arm_params()`, the v11 simulation's coarse model of `cad/v10 human editable`: X4, 140 mm omni wheels, hinged sprung forks, AK40-10 drives, SO-101 and bin. The Launcher and arm bridges use it. `ARM_V7` preserves the Oct 25 arm robot. Nov 3 PC stack regression: 4.5–4.6 (simulation only).
- **Selected supply:** OVONIC Roam 6S 5200 mAh XT90-S LiPo. Its resistance, discharge curve and runtime remain estimates. The CubeMars hardware driver is still missing.
- **CAD is in progress, not ready to print.** Native pod motion is checked; thin wheel sections, deck/pod integration and the arm's stow collision remain open. Sim and CAD also disagree on arm/head height (2, 4.4).
- **Compute and floor camera remain open:** Amazon-only compact shortlist: BMAX B8A Pro 8745HS ($436.99 captured discount), Getorli AT103 i5-13420H ($379.99 coupon) or M5 Ultra 7730U ($449.99), plus Teensy; no purchase adopted (4.9). Keep desktop AI offload and a separate colour GS head later. Pi 4 default navigation failed; `nav_profile:=pi` (0.6 m/s) is baselined for a Pi driving the robot (the default on a Pi) and gives a slow working arm robot, ~0.16 m localization, CPU pinned while driving (4.10). Nose D435 is an implemented arm option; physical validation remains open.
- **Catch and real-camera ball benches are paused by Fernando.** Resume the ball work only with the stereo global-shutter colour cameras and a mobile robot. Earlier catch scores remain reference evidence.
- **Drive baselined Nov 2:** AK40-10 confirmed after the sim motor study (4.7); the fork pivot moves level with the axle in the sim baseline. The CAD fork still has the 43 mm pivot and must follow (7).
- **Next:** resolve the CAD/model gaps, test one wheel/pod, validate arm navigation and picking on the candidate computer, then build the rolling arm platform (8).

## 1. Goal

A mostly printed indoor home robot with omnidirectional driving, lidar SLAM, navigation, RC and browser control, a face and voice, and swappable payloads. **The immediate build is the SO-101 arm robot for tidying floor objects.** The parked catch payload tracks a 74 mm foam ball, catches it and returns it to a thrower 6–7 m away. Keep the shared platform useful for both without making completion of the catch robot a prerequisite for the arm.

Use the robot's own code in simulation before physical validation. Real first runs and flashes are attended; measured results must feed back into the models.

## 2. Baseline design

Dimensions below describe the **current arm simulation**, with nominal CAD dimensions and estimated mass, not a completed physical robot. Existing bench hardware includes the D435, Pi 4, Camera Module 3 and bearings; the mobile robot has not been built.

| Subsystem | Current design and status |
| --- | --- |
| Base | Four-wheel X, wheel centres on r 146 mm; deck/guards r 217 mm, bumper outer r 225 mm. Payload plate 181.5 mm high. Arm model mass 9.63 kg and CG z 126 mm, from the earlier CAD estimate plus D435, not weighed. |
| Wheels | 140 mm, two rows of six 36 mm rollers, rows 72 mm apart; PLA hubs/cores and replaceable medium-TPU treads. M3 × 50 axle screws, captive nuts, two owned 6804 bearings per wheel. Derived from the v9 bench wheel; physical fit, strength, wear and grip untested. |
| Suspension | Retained sprung, damped double-sided forks; 3 mm droop + 15 mm bump. **Pivot baselined Nov 2 level with the axle** (80.6 mm arm, (y, z) = (80.6, 0) mm; about 12.9° swing, 1.4 mm fore-aft): the drawn CAD pivot (68.2, 43) mm causes torque jacking (4.7). The CAD fork still needs the change. Opposite-hand pods on opposite diagonals; native CAD pivot uses 627 bearings / 7 mm shaft. Spring/damping remain estimates. |
| Drive | 4 × CubeMars AK40-10 V3.0 KV170, integrated FOC, CAN, dual absolute magnetic encoders, 10:1. **Confirmed Nov 2** by the sim motor study over cheaper candidates (4.7). Real torque/speed/thermal, max supply voltage, comm-loss behaviour, output-bearing load and speed-loop gains on 6S VERIFY; firmware still needs the CubeMars driver. |
| Battery | OVONIC Roam 6S1P 5200 mAh LiPo, XT90-S, 22.2 V nominal / 25.2 V full, 115.44 Wh nominal; catalog mass 706 g. Provisional low-battery ramped stop 21 V. No assumed BMS; 20% energy reserve for runtime estimates. Mount, fuse and electrical fit VERIFY. |
| Arm | SO-101 at x 140 mm, 37 mm riser, arm base z 218.5 mm; finger pads, fingertips, wrist-vision model (physical wrist camera TBD), rear bin and checked-waypoint planner. Current fingers-down floor reach roughly 0.26–0.35 m from centre. CAD height and stow pose need reconciliation. |
| Head | Stereo bar at the arm's −40° index; camera bar/cheeks 5 mm above the v7 arrangement in the sim. Face/audio over the bin in the arm model; v10 CAD retains the rear mast arrangement. Final mechanical reconciliation pending. |
| Floor camera | Stereo remains the software default. Proposed optional D435 on the nose, body centre (190, 0, 111.5) mm, tilted 30° down; `ARM_D435` / `arm_camera:=d435`. Physical mount and comparative picking/navigation validation pending. |
| Lidar / IMU | Adopted A2M12 at the front (x 140 mm, scan plane z 163 mm in ARM); BNO085 on the MCU. Keep forward obstacle sight clear. |
| Main computer | Open; compact and Amazon-only. Proposed BMAX B8A Pro 8745HS for performance, Getorli AT103 i5-13420H within $400, or M5 Ultra 7730U for documented interfaces. All 16 GB/512 GB, replaceable RAM, four A + C. DC/audio qualifications and physical performance remain open (4.9). |
| MCU | Teensy 4.1 for now; safety authority, 1 kHz base/payload control, odometry, CAN1 drive / CAN3 FD payload. Board-free C++ core shared by hardware and SIL. |
| Interface | Robot-served operator console and supervisor; Manual, navigation/missions and Tidy; gamepad/RC. 5 in 800×480 face; microphone and speaker required, onboard analog audio preferred provisionally, exact hardware TBD. |
| Electronics | Proposed Teensy carrier PCB, protected inputs, CAN transceivers, trigger shifter and `BUS_EN`; off-the-shelf fused power switching, current sensing and regulated rails. Review against 6S and the selected computer before ordering. |
| Printing / fasteners | PLA, PETG, TPU; approximately 256 mm bed. No heat-set inserts. M3 thread-forming into 2.8 mm holes, ≥6 mm engagement; 3.4 mm through holes. Pan/flat heads, hex-drive shallow M3 pan heads 6.5 × 2.5 mm; fasteners magenta. |

**Safety/control:** all stops ramped; 0.5 s link watchdog leads to ramped stop then disarm. Firmware fault stop (implemented, SIL only): in AUTO, a pack under 21 V for 1 s, a drive-motor error or a silent motor gives a ramped stop and disarm, latched (`FAULT_STOP`) until the computer requests DISARM (the console's mission ends) with the pack ≥22.2 V; RC manual is not stopped. Thresholds are estimates. The arm refuses unchecked motions and malformed/unsafe external trajectories; unhealthy servo feedback ends a Tidy with the arm holding (never torque-off). Adult profile up to 2 m/s; kid 0.8 m/s, 60% acceleration. Acceleration is direction-aware; arm-out requests cap speed at 1 m/s and acceleration around 2.1 m/s². These are controller settings, not measured physical capability or fixed motor-selection pass criteria. The proposed Pi navigation profile also caps speed at 1 m/s. The final power-cut/switch implementation remains open.

**Parked catch kit:** Ø330 net funnel, padded pocket, chute with driven feed wheels and J-bend, IR catch/feed beams; −30…+40° tilt with STS3250. Two Ø70 flywheels with F90 1300KV motors and bidirectional-DShot AM32 ESC. Feed actuator unresolved (brushed 25D versus current-holding gimbal motor). Trigger interlocks require AUTO, flywheels within 3%, no bumper and safe launch elevation. Reassess packaging and power on the new base before adoption; the preserved catch model is v7-based.

### 2.1 Software

- **Firmware/link:** board-free `firmware/lib/omni_fw`, Teensy drivers in `src/main.cpp`; COBS/CRC USB protocol 5, command heartbeat, state/IMU, build/RC/temperature info and timestamp events. Event-time MCU stamps and hardware sync pulse map clocks; USB echo is the fallback. CubeMars, bidirectional DShot and CRSF hardware support remain to finish; written drivers are unvalidated on real parts.
- **Robot ROS 2 Jazzy:** MCU link, mode management, scan preparation, EKF, slam_toolbox, Nav2/MPPI, arm, face and console. `omni_cpp` glue and catch cores are defaults; Python twins remain references. Nav2 composition and sleeping catch nodes reduce CPU. `omni_sim` is PC-only; robot packages keep ROS-free cores.
- **Arm:** `omni_arm` owns the servo bus, kinematics, waypoint motion, floor/wrist vision and tidy behaviour. No MoveIt. Generated robot configuration supplies geometry, poses and keep-outs.
- **UI:** the browser decodes/draws raw robot data; images use the separate preview path. Supervisor starts/stops steps. Face uses pygame, Kokoro-first TTS and offline voice-command recognition. Sessions use omni-log/1 for replay.
- **Configuration:** `robot_export.py` generates arm JSONs and `fw_params_arm.h`; arm SIL uses that header because wheel geometry is compiled in. `fw_params.h` remains the catch variant. Regenerate after params/catalog/model changes.

## 3. Expected performance and evidence

**No complete v10/6S hardware validation exists.** Keep these configurations separate:

| Configuration / evidence | Result and limitation |
| --- | --- |
| Current v10 arm geometry, sim | Lowering the riser to 37 mm restores fingers-down floor reach; the CAD's 231.5 mm arm-base height gave none. Current reach roughly 0.26–0.35 m. |
| D435 arm option, single arena sim run | 4/4 toys in the bin; task counted 5 picks, unexplained. ROS depth/cloud and both Nav2 depth layers exercised. Not the 120-object comparison or a clutter-navigation score. |
| Current arm on Pi 4, Ethernet | Default stack 3.5–3.6 of 4 cores; patrol aborts. Arm tier without Nav2/SLAM 2.1–2.25 cores and picks. `nav_profile:=pi` (0.6 m/s, Nov 2, P5–P8): 13–16/16 goals, localization median 0.13–0.19 m, 4–6/6 toys binned; with the collision monitor's scan fix P9 15/16, 0.16 m, 5/6, laps 10% faster; 3.6–3.9 cores driving, busiest core pinned (PC `full`: 0.08 m) (4.10). Sim ran at 0.7× real time; real D435 driver, voice and tags excluded. |
| V10 arm, short 6S cruising probes | 68–76 W at 0.75–1 m/s commanded; about 73–82 min from 80% of nominal pack energy **(est)**. Assumed electronics and stowed-arm loads; no full discharge run. Budget roughly an hour for mixed use pending measurement. |
| V10 arm, AK40 drive study, sim (Nov 1–2; drawn 43 mm fork pivot, 10/15/20 kg, ~23,000 runs) | Physics limits: ~4.2 m/s forward / 3.2 diagonal (back-EMF); 50 ft hallway 9.0 s at 2 m/s, 5.3 s uncapped; forward start 2.6 m/s² at 10 kg (fork), braking ~7; slope start ~18°; creep step ~65 mm head-on, ~12.5 mm worst direction; house transitions to 18 mm always cross. Firmware missions: all routes done at every mass; house tour 47–48 s at any cap ≥ 2 m/s. Endurance (10 kg, uncapped): 71 min continuous touring to the 21 V stop, 102 Wh, winding peak ~97 °C **(est thermal)**; the other three endurance cases did not finish. Eight cheaper motors as datasheet fits (4.7). Single runs; acceleration criterion-sensitive (±30%). `results/motor_char/RUNLOG.md`, `REPORT.md`. |
| V10 arm, collinear fork pivot, sim (Nov 2 quick check, AK40, 10/20 kg) | Against the drawn pivot: forward start 2.6 → 6.2 m/s² (10 kg), slope start 18 → 28°, at-speed wheel bounce gone, 2 m/s cruise drive power 55 → 41 W (10 kg) and 102 → 62 W (20 kg); braking, hallway time and crossings unchanged or better. The basis of the Nov 2 pivot baseline; CAD not yet redrawn. `studies/pivot_quick.py`. |
| Earlier arm studies | 102/120 objects in bin; 48 ramped stops at 1–2 m/s, no tip. Reference results, not a rerun of the current CAD/6S robot. |
| Earlier v7 drawn catch robot | 93.1% sim game / 88.0% SIL over 216 throws at 6–7 m; full ROS 41/48. Home-floor base trade 89.8% versus 90.7% flat. Earlier pack/configuration. |
| Earlier v7 navigation/localization | 8/8 goals in 80 s, 1.95 m/s reached; slam_toolbox/EKF 7–9 cm median error. Two-hour soak: 43 laps, no crash/freeze/leak, max localization error 19 cm. |

Sources: v11 `studies/arm_pick.py`, `arm_study.py`, `sil_catch.py`, `base_trade.py`, `drive_char.py`, `drive_mission.py`, `pivot_quick.py`; `ros/src/omni_sim/omni_sim/nav_goals.py`; `ros/tools/soak.py`, `arm_load.py`; Pi reports in `characterization/pi_camera/results/`. Appendix B preserves the full result tables and qualifications.

## 4. Major findings from trades and simulations

### 4.1 Perception limits the catch (parked)

Latency, frame rate and stereo calibration stability dominate: simulated drift 0.04° → 97% catches, 0.3° → 78%, 0.5° → 58%. Use a stiff common camera bar, global shutter and synchronized exposure. Drag matters; honest stereo initialization improved the long-throw result. These are design drivers, not real-camera guarantees.

D435 trials mostly used a 160.7 mm ball, not the robot's 74 mm ball. Live work implicated lighting/backlighting and short-exposure signal; it did not validate moving-camera tracking of the small thrown ball at 6 m. The trigger also produced handling false positives. Resume only under Fernando's stated camera/mobile-robot gate (4.10). Sources: history §§22–23; `characterization/throw_track/README.md`.

### 4.2 Base

**Keep X4 with sprung pods** (Fernando retained pods Oct 31). Earlier comparisons found kiwi worse at catching and braking stability; rigid X4 lost 16 catch points on a diagonal rug and was sensitive to a wheel 1 mm low. A front rocker matched many crossings/catches, but pods roughly halved pitch (8° versus 17° on a 12 mm threshold at 2 m/s). Rocker is parked.

Big rollers matter at sideways edges; smaller 32 mm rollers lost 13 points on a diagonal rug. X4 stalls concentrate at diagonals; driving forward crossed steps up to 40 mm in the older study. Guards/skirt and low speed over mapped thresholds address ball-under-wheel and hop hazards. **These are older wheel/base results, not validation of the new 140 mm wheel.** Sources: `studies/base_trade.py`, history §25 / Appendix A §16.

### 4.3 Catch module and launcher (parked)

The padded pocket, chute feed and 35° catch tilt together improved static catches from 80% to 99%; a bare funnel cannot reliably hold multiple balls. Keep one-ball feeding; a magazine tube is the plausible later multi-ball route. Ceiling-aware returns reached 7 m under an 8 ft ceiling around 35–40° / 4,500 rpm in simulation. Sources: history §§22–23. No catch hardware build is required before the arm robot.

### 4.4 Layout and CAD

Keep the lidar forward with an unobstructed layer: shadows cost obstacle sight more than localization. Check arm clearances with the SO-101's actual meshes; link capsules missed its base/bracket.

**CAD/sim gap:** the current sim keeps arm-base height 218.5 mm with a 37 mm riser, and raises the head 5 mm instead of CAD's 13 mm. The earlier drawn height loses floor reach. Reconcile riser/adapter/arm origin and head with the actual assembly; narrow head clearances and face/mast height need verification. These are intentional simulation departures, not completed CAD changes.

**V10 status:** `cad/v10 human editable/ASSEMBLIES.md` describes the native pod and whole-robot subassemblies. Pod hinge, shocks and wheel/roller spin have checks. The current printable motor is a free-turning **two-part, one-687-bearing** fixture, not a powered actuator. The CAD README flags thin spoke webs/nose slivers and says not ready to print. Deck/pod interference and missing foot-bolt coverage remain unchecked. Its SO-101 stow pose collides; the delivered CAD pose is `high`. Per-joint limits do not establish collision-free combined motion. Resolve this before claiming the sim's stow/waypoints are valid for the assembly.

**Compact wheel + motor pod (proposed, `cad/v11/`):** a self-contained module dropped into a cutout in the flat robot plate. The plate top is 3 mm above the axle, as v10's deck, so the hinge is at plate height. Two side frames stand on the plate, each held by 3 vertical M3 screws from the top. The single printed Ø140 hub rides on the AK40-10's output bearing and one 6804. The AK40-10 sits outboard because its XT30 mates radially (pins plus leads to r about 36, outside the r 30 hub bore; Fernando chose this Nov 2); the connector points back, 4.7 mm from anything rotating. The one-piece fork bolts straight to the motor's rear face and hinges on a Ø7 shaft level with the axle (80.6 mm, the sim baseline). Suspension: one ISO 10243 die spring per side (Ø10 x 38; green 6.8 N/mm for 10-12.5 kg, blue 11.9 for 12.5-15 kg) on an M5 button-head plunger pressing a fork pad 30 mm from the pivot, with no damper. That gives a wheel rate of 1.9 / 3.3 N/mm; frame ledges stop the fork at -3/+15 mm. RC shocks were too soft and off-the-shelf torsion springs too weak (about 0.3 vs 2-4 N·m). build123d checks show no interference at droop, nominal and bump. The SolidWorks assembly (`V11_pod.SLDASM`, imported parts) matches those poses to 0.01 mm and drags within its travel. Open: the motor output bearing's rating and rear-hole thread (VERIFY), die-spring envelope (VERIFY), a spoke neck of about 1.9 mm, a heavy fork, and the mirror-hand pod. The motor model is our own solid (the maker's STEP is surfaces only). Details: `cad/v11/README.md`. Rule since: model real mating connectors and leads before packaging (`cad/solidworks.md` section 16).

Native design intent follows `cad/solidworks.md`: named controls, readable features, constrained sketches, genuine arcs, native patterns/mirrors. Detailed revisions/checks stay in the CAD README and validation outputs. v9 is the frozen source bench wheel; v7 stays untouched for earlier studies.

### 4.5 Arm

SO-101 servo torque limits payload before base tipping: earlier estimates 0.03–0.09 kg at rated torque at full reach. Ramped stops and arm-out caps remain essential. Exact top-down IK plus checked waypoints suits the five-joint task; floor perception, calibration error and grasp confirmation dominate. The wrist-vision model corrects local error; physical wrist camera remains TBD, and a heavy D435 there is a poor fit. Offload optional learned perception/free chat if useful, but keep motion and safety usable without Wi-Fi. Sources: `omni_arm/README.md`, `studies/arm_study.py`, `arm_pick.py`, history Appendix A §§11–13.

**PC stack regression, Nov 3 (sim, current ARM, seed 0, six toys, full Nav2/SLAM):** navigate-then-tidy finished with 6/6 physically binned and zero failed picks under both Python/D435/wall clock and C++ driver/stereo/sim clock. These are different conditions, not a camera or speed comparison. Simulation ran below real time; no Pi, real sensors or CAD validation. Raw D435 depth was retained: a single median-filter probe scored 5/6 versus raw 6/6, despite reporting six picks. Details and qualifications: [PC regression](simulation/v11%20sil/studies/results/pc_stack_2026-11-03.md).

### 4.6 Localization, ROS and timing

Keep lidar/EKF localization; AprilTags help recovery, gaps and docking, not replacement of a good known-map localizer. Single-view tag relocalization can choose a mirror pose. Pod roll/pitch must enter geometric models. Filter floor returns and de-skew turning scans. Nav2 costmaps must cover the controller horizon. Preserve event timestamps, hardware clock sync, REP-105 frames and monotonic durations. The console draws raw data in the browser to reduce robot CPU. Details: `omni_calib/README.md`, `omni_bringup`, history Appendix A §10.

**PC regression, Nov 3:** current ARM/Python/D435/wall clock reached 8/8 house goals (0.113 m mean true arrival error). ARM/stereo explored 115.4 m² and returned within 0.073 m of its start in 130.7 wall seconds. SLAM restart rejects cached/old map transforms and reactivates an inactive lifecycle node; console stop/takeover tracks launch descendants, including separately grouped SLAM children, preventing orphan publishers and self-termination. In the final single-localizer recovery check the PID stayed unchanged and the next goal arrived 0.061 m from truth. Loading a saved map still assumes its start unless the operator supplies a pose. The installed composed Nav2 can crash or require forced termination during shutdown; this remains open, separate from completed driving/picking results. See the PC regression report.

### 4.7 Drive, supply and printing

AK40-10 was selected Oct 30 for documented maker specifications over the cheaper DM-2325; older simulated routes/crossings/catches were broadly comparable. Useful performance versus cost and sustained duty decides the final set, not an arbitrary 2 m/s pass line. Bench torque/speed/current/temperature and comm-loss behaviour across **25.2 V down to the provisional 21 V stop**.

**Drive (AK40-10, baselined Nov 2; sim):** the model is fitted to the maker's 24 V torque-speed chart (`studies/motor_fit.py`; RMS 1.9% speed, 1.6% current): effective resistance 2.13 Ω, 0.12 N·m friction, 4.1 N·m peak; the datasheet DC model was ~3× optimistic on resistance (`AK40-10-dc`). On 6S the robot reaches ~4.2 m/s forward / 3.2 m/s diagonal (back-EMF-limited). The 50 ft hallway takes 9.0 s at the 2 m/s cap and 5.3 s uncapped; the house tour only 48 → 46 s, so speed above 2 m/s pays only on long straights. At 10 kg the motors run below rating; at 20 kg route RMS torque is 1.5–1.9 N·m (rated 1.3), windings ~60 °C on routes, derate after ~30 s of a 14° climb (thermal **est**). Of the cheaper candidates (datasheet fits on one rule, `-ds`), only the DM-2325 at its 5 N·m listing comes close, and it overheats far sooner at 20 kg and has conflicting specs and a different body; GIM3510 matches but costs more; EduLite 05, GIM4305, DM-J3507, M2006 and the 2 N·m DM-2325 fall short on speed, slopes, steps or heat. The AK40 is right-sized for growth to 15–20 kg and ramps, generous for a 10 kg flat-floor robot. The ARM model also has plain rollers (pin μ 0.3 **est**), mass-aware pod damping, `arm_params(mass_kg, cg_dz)` and a smooth roller hull (the old 14-flat hull was an artifact). Study: `studies/drive_char.py`, `drive_mission.py`, `results/motor_char/` (RUNLOG, REPORT).

**Fork pivot level with the axle (baselined Nov 2; sim):** the AK40 rides the fork, so with the drawn pivot 43 mm above the axle drive torque τ leaves ≈0.6τ on the hinge (torque jacking): ~18 N load shift per 2 N·m on a ~25 N wheel, wheels bouncing at speed and fighting each other. With the pivot collinear (`studies/pivot_quick.py`, AK40, matched springs): forward start 2.6 → 6.2 m/s² at 10 kg, sideways 4.1 → 5.6; slope start 18 → 28° forward and 8 → 24° sideways; at-speed bouncing gone; 2 m/s cruise drive power 55 → 41 W at 10 kg and 102 → 62 W at 20 kg; braking, hallway time and crossings unchanged or slightly better. Remaining limits are grip and the 20 mm step / plush rug taken at 45°. Speed-loop gains still matter (¼ gain: −19 to −29% cruise power; real CubeMars gains unknown). Springs: soft clears diagonal steps, stiff stops bottoming at 20 kg; damping and preload matter little. Acceleration numbers are criterion-sensitive (±30%); compare, don't quote as limits.

The selected pack is catalog key `6S1P-OVONIC-5200` ([maker listing](https://www.ampow.com/products/ovonic-roam-series-6s-lipo-battery-5200mah-6s1p-150c-22-2v-long-range-lipo-battery-with-xt90-s-anti-spark-plug-for-7-10-inch-long-range-drone-6s-hd-cinelifter-multirotor-x-class)). Capacity/mass are claims; R0, polarization and OCV are estimated, and advertised C-rating is not a measured current envelope. Measure capacity/sag and physical mounting. Do not carry old 7S runtime/results onto 6S.

Wheel acceptance includes roller clearance at both axial stops, fit, loaded strength, wear and grip. Assembly, screw access/thread engagement and wall checks matter beyond static clashes. Keep dimensions/revision history in the active CAD documentation; no blanket full-robot pass follows from wheel-only checks.

### 4.8 Alternatives not adopted

| Alternative | Lasting reason / status |
| --- | --- |
| Kiwi, rigid X4, rocker, swerve | X4 + pods chosen for home-floor stability; rocker simpler but pitches more, swerve adds size/actuators. |
| Optical-flow floor odometry | Height/speed/rug limits; not a baseline sensor. |
| MoveIt / Gazebo / Foxglove as main UI | Exact arm planner, MuJoCo physics and operator console retained; Foxglove for debugging. |
| Wrist D435 | Weight, minimum range, body size and cable conflict with close floor grasps; nose option instead. |

### 4.9 Robot computer and cameras

**Computer open; Amazon-only compact proposal, not adopted:** Fernando rejected Lenovo/business Tiny size and now requires Amazon. The Nov 2 saved Amazon results change the shortlist: BMAX B8A Pro / 8745HS, 16 GB DDR5/512 GB at $436.99 after 5% discount is the performance choice; Getorli AT103 / i5-13420H, 16 GB DDR4/512 GB at $379.99 after coupon is the budget choice; GMKtec M5 Ultra / 7730U, 16 GB DDR4/512 GB at $449.99 is the better-documented interface alternative. All have replaceable RAM and four A plus one C USB sockets. Prices are captured offers, not checkout guarantees. BMAX's maker specifies 19 V/3.42 A while Amazon metadata says 120 W; Getorli DC rating, dimensions and microphone input remain VERIFY. M5 specifies 19 V/3.42 A and CTIA audio. Regulate the selected DC rail from 6S. No selected computer or runtime is physically qualified.

**Resource basis:** retain Teensy for low-level control/safety; keep navigation and final grasp corrections local, with optional heavy AI on the desktop over Wi-Fi. Nav2/SLAM, depth processing and voice chiefly need CPU scheduling/per-thread reserve; D435 computes depth internally. Sixteen GB is a reasonable initial capacity; replaceable RAM preserves expansion. GPU gaming claims do not establish useful acceleration for this stack. Public CPU records indicate substantially more reserve from 8745HS than 7730U, not a measured robot speedup. Earlier M8 and direct/used-market alternatives remain historical comparison evidence outside this PDF's top three.

**Catch head deferred.** Budget about $500 **(est total)** later for Pi 5 4 GB + Arducam B0492R colour AR0234 pair, or an optional RVC2 OAK-FFC head. Proposed 0.30–0.40 m baseline needs CAD/optics/calibration checks. Arducam publishes ~93–94 fps at 1280×720 per eye, 38 at full resolution; IMX296 publishes 60 fps full resolution. Complete pair throughput/latency remains VERIFY. Send Pi detections over wired Ethernet; raw stereo exceeds GbE. Jetson is conditional on a named local CUDA/neural workload, not needed automatically for this CPU stack.

**Interfaces:** D435 direct USB 3, Teensy native USB (no FTDI), SO-101 USB/half-duplex TTL adapter and A2M12 USB/UART adapter consume four ports; USB-C stays spare. Hub is optional expansion ($30 est). OAK is not baseline; wrist camera TBD. M5 CTIA jack supports a compatible microphone and powered speaker via breakout. BMAX headset jack pinout and Getorli microphone support need verification; reserve USB audio if needed. HDMI face power is separate. These PCs expose no general GPIO/UART header; Teensy owns robot GPIO/CAN/trigger wiring. MCU USB-echo clock mapping is the arm fallback, not validation of <50 µs precision. Catch trigger/frame matching and PC/Pi clock mapping remain unfinished. Teensy pins 24/25/26 reserve camera trigger/clock pulse/strobe; BUS_EN pin 34 remains proposed. Two NICs or 2.5GbE do not establish hardware PTP support.

No new simulations, benchmarks or camera benches ran. Qualify Ubuntu drivers, real D435/navigation/picking/face/voice, data-age tails, heat and DC watts on the purchased unit. Catching remains paused. Sourced details, saved-offer prices and estimated compute/controller totals: [Nov 2 compute trade](components/computer/compute_purchase_trade_2026-11-02.md), latest Amazon PDF audit. Existing measurements: `studies/compute_choice.py` and `characterization/pi_camera/README.md`.

### 4.10 Pi in the loop; real-ball benches paused

**Whole arm robot on Pi 4, Nov 1:** Ethernet runs used 3.5–3.6 cores, saturated the busiest core and aborted patrols with stale transforms; arm-only used 2.1–2.25. Much of `arm_node` load is idle publishing/executor overhead. A servo-bus busy-wait was fixed. The sim ran at 0.7× real time and excluded real D435 capture, voice and tags, so these are provisional loads.

**`nav_profile:=pi` on the Pi 4, Nov 2 (sim, wired, `clock:=wall`, face on, nose D435, sim ~0.7× real time; single runs per setting): a slow but working arm robot at 0.6 m/s, with the CPU pinned while driving. Baselined for a Pi driving the robot: the launch's default profile resolves to `pi` on a Raspberry Pi (`full` elsewhere; a named profile is kept).** Shipped profile: 0.6 m/s cap; slam_toolbox corrects every 0.5 s / 0.25 m / 0.25 rad with loop closing kept; MPPI 400 × 24 at 10 Hz; local costmap 5 m at 7.5 cm, 2 Hz; global costmap 1 Hz; costmap waits 3 s planner / 1 s controller; EKF 30 Hz; `arm_node` 20 Hz; Nav2 bond timeout 30 s; the collision monitor takes each scan as it is, not shifted to "now" through odom. Shipped run P9: 15/16 route goals, true localization median 0.16 m (one 1.05 m slip, at spot 7 as in P6/P7), 2 laps in 429 s, navigate-then-tidy binned 5 of 6 toys. The four runs before the scan fix (P5–P8): 13–16/16 goals, localization median 0.13–0.19 m (max 0.4–1.3 m), 2 laps in 434–502 s, 4–6 of 6 toys binned. Pi: ~2.8 cores holding, 3.6–3.8 on the route with the busiest core p95 100% (slam_toolbox ~0.7, Nav2 ~0.86, `arm_node` ~0.5), ~2.9 tidying; >2.6 GB free, no throttling, no "Transform data too old"; a few EKF rate misses (20 in P9). Most of P5–P8's 0–3 "Nav2 gave up" goals a run were the collision monitor: with `base_shift_correction` it looks `odom`→`base_link` up at the current time, the loaded Pi's odom often came >0.2 s late, and it stopped the robot as an "invalid source" ~100 times a run (123 in P8, 1 in P9). The PC's `full` run had as many (123 in C4) without failing a goal: there the firmware sim's mapped MCU stamps scatter ±0.17 s (`clock:=wall`; a real Teensy's clock is linear), so the monitor's wait for odom is now 0.5 s for every profile (was 0.2), keeping the shift where CPU allows: C8 16/16, 1 failed lookup, route 518 s against 557 s; its ~20 remaining stops are scans over 1 s old while the sim lagged, kept as a stop. How it got there: the first `pi` profile (1 m/s, corrections every 0.4 m, no loop closing) failed twice before running (4 s lifecycle bond lapsed while `bt_navigator` took 25 s to activate; a 2 cm correlation grid crashed slam_toolbox), then localized 0.66 m median (P3). On the PC the same profile gave 0.34 m against `full`'s 0.08 m (C3 / C4): its slam cuts were the loss. Full slam fixes that on the PC (0.10 m, C5) but took 1.0–1.4 Pi cores while driving and lost the robot by 2 m (P4). Loop closing alone recovers most of it (0.18 m on the PC, C7), and at 0.6 m/s corrections every 0.25 m cost the Pi about what 0.4 m did at 1 m/s. AMCL (`localizer:=amcl`, now wired into the robot and arm steps) localized worse than either on the PC (median ~0.35 m, slips to 2 m; C6). `arm_node`'s ~0.67 core holding still is its rclpy executor at 50 Hz: neither skipping image decoding (~1%), throttling `/arm/joint_states` nor dropping the image subscriptions moved it; 20 Hz did (~0.5, tidy still bins; P7). Not run: Explore, `lean` on the Pi, repeats per setting, the real D435 driver and voice. Remaining levers: the face display off, a faster computer (the PC holds ~0.1 m with full slam). **`arm_node` in C++ (Nov 3, implemented, unvalidated on the Pi and the real bus; `arm_impl:=cpp`, default still `py`):** `omni_cpp`'s `arm_driver` owns the servo bus, joint states, TF and `/arm/out` at 50 Hz; `omni_arm`'s `arm_brain` (node name `arm_node`) plans, runs the tidy and sends whole motions on `/arm/motion`, and wakes only on requests (its tidy subscriptions exist only while tidying). Parity tests cover the STS3215 packets, calibration, forward kinematics and motion player. On the PC (WSL, sim, `clock:=wall`, 50 Hz, single runs): holding still the Python node used 0.32-0.36 core, the pair 0.05 (brain 0.04, driver 0.01), TF at 50 Hz; the tidy binned 6/6 with the same object order under either (sim clock, where CPU is not comparable: both Python nodes then spent ~0.87 core on the bridge's `/clock` stream). Not run: the Pi, the real serial bus. The first-look wall-clock stall did not recur in the Nov 3 PC regression: Python/D435 navigate-then-tidy finished with 6/6 toys binned (4.5). The harness now has a configurable 600 s wall budget; its former 240 s limit had prematurely stopped another run at 2/6. Runs: `characterization/pi_camera/results/pi_arm_P1–P8_*`, `arm_report_P{3..8}.json`, `logs/arm_load_{P3..P8,C3..C7}_*`; harness `arm_split.sh` + `ros/tools/arm_load.py` (ROUTE, NAVTIDY); plan `PROMPT_pi_nav_and_tidy.md`; setup `rpi.md`.

**Real-camera ball benches paused by Fernando Nov 1:** bucket lab, throw tracking and motion-test tools/data remain available; no bench gate was adopted into ROS catch defaults. Keep the observed lighting limits and false-positive problem (4.1). Do not resume until stereo global-shutter colour cameras and a mobile robot exist; then test the actual 74 mm ball thrown from 6 m at 2 ms. Detailed experiments live in their READMEs; Appendix B preserves the audits.

### 4.11 Nose D435 for the arm (proposed)

Use the owned D435 on the deck nose for floor picking and Nav2 near-field depth; physical wrist camera remains TBD despite the existing wrist-vision model. In current ARM its box centre is (190, 0, 111.5) mm; old placement at z 105 mm predates the lifted deck. The modeled streamed mode covers approximately the arm's near-floor reach; minimum range/noise are partly estimated.

`arm_camera:=d435` selects `robot_arm_d435.json`; the bridge supplies depth/cloud and Nav2 gets a separate depth layer so lidar clearing does not erase camera-only obstacles. Sim/ROS plumbing works, but robust picking versus stereo and navigation through clutter are unscored. Pod/braking pitch can mark false floor obstacles; the arm can mark itself. Physical mounting, lidar/bumper clearance, USB 3, real depth noise and pitch-aware frames need verification. Source/mount concept: `cad/other/d435_nose`, `omni_bringup/config/d435.yaml`, `sil_pick.py --camera d435`.

## 5. Simulation

**v11 sil is the current robot-code simulator.** MuJoCo owns physics; MCU code runs at 1 kHz as SIL, and ROS-free robot cores or the ROS stack drive it. Windows renders; ROS runs in WSL. Earlier versions remain for reproducing their studies, not as competing current baselines.

Use `arm_params()` / `ARM_D435` for current arm work; `ARM_V7` for earlier arm layouts, `drawn_params()` for v7 CAD-mass catch references. A model/geometry change requires regenerated exports and new scores before transferring conclusions.

Limits: unmeasured friction, compliance, ball and motor parameters; simplified floor and mass distribution; rendered imagery omits important optics/lighting effects. Model knobs are hypotheses until calibrated. Compare identical real/sim sessions with logs, not unrelated aggregate scores. Technical model pitfalls and old version details are in the simulation README and history.

## 6. Where things live

| Location | Purpose |
| --- | --- |
| `simulation/v11 sil/` | `params.py`, `catalog.py`, `robot_export.py`, SIL, picking/catching worlds; `studies/` for scored comparisons. |
| `cad/v10 human editable/` | Active native SolidWorks arm robot; README, ASSEMBLIES, DUMMY_MOTOR, validation. `cad/solidworks.md` is the modeling contract. |
| `cad/v9/`, `cad/v7/` | Frozen wheel source / preserved Oct 25 study geometry; not the active editable assembly. |
| `firmware/` | Teensy core, drivers, SIL build, pin map and generated parameter headers. |
| `ros/` | `omni.sh`, robot packages, console, bridges and test tools; package READMEs hold procedures. |
| `characterization/pi_camera/`, `rpi.md` | Candidate-computer experiments and network/setup. Other camera benches are paused as above. |
| `database/` | Shared sourced parts CSVs and validation; no omni-robot imports. |
| `Launcher.cmd` | Fernando's console/3D/face launcher; `launcher_full.ps1` retains studies and extra steps. |

Python: `C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe`, never PlatformIO's PATH interpreter. ROS: `ros/omni.sh` in Ubuntu-24.04 WSL; builds in `~/omni_ws`, never `/mnt/c`. Tests and commands: subsystem READMEs / `AGENTS.md`. Model changes require export regeneration; Python/C++ reference twins and protocol definitions change together. Keep both Launchers working.

Run alongside Fernando's stack on a separate ROS domain and port; stop only your own processes. Flash only attended. Use the separate realsense venv for its bench tools; do not restart paused work merely because tooling exists.

## 7. Open decisions and VERIFY

- **Mechanics:** redraw the CAD fork with its pivot level with the axle (sim baseline, 4.7) and recheck pod motion, shock geometry and clearances; print readiness/strength of wheel webs; real fits and loaded wear; deck/pod interfaces/fasteners; arm/head heights and mast; stow collision and complete waypoint clearance. Final mass/CG and D435 mount.
- **Drive/power:** real CubeMars driver and CAN settings; comm-loss stop; AK40 max supply voltage and output-bearing radial load; speed-loop gains; motor heat inside a printed hub (PETG/ASA near the motor); 6S torque/speed/current/thermal curves, bearing loads/backlash; measured LiPo capacity/sag, mount, fuse/switch and regulated rails.
- **Arm perception:** D435 versus stereo on the same object suite, floor reach and pose error, wrist correction, grasp confirmation; near-field navigation without false obstacles or self-obstruction.
- **Computer:** shortlist/purchase choice from 4.9 remains proposed; qualify full arm navigation/picking with real D435/voice, scheduling/data-age tails, DC power and cooling. Pi profile and idle arm load remain open. Catch pair modes/sync/optics are not validated. Carrier/`BUS_EN` remains proposed.
- **Physical calibration:** roller friction/rolling resistance, pod springs/damping, encoder signs, IMU, lidar scan tilt/timestamps, timing pulse and sensor frames. Datasheet values are not bench results.
- **Parked catch choices:** cameras/trigger/calibration stability, feed motor and payload packaging; no new ball-bench work until its gate is met.

## 8. Path to the physical robot

**Working arm-first sequence, pending bench results.** Measure the biggest unknown cheaply, one unit before four; feed each result into the sim and rerun the affected study. Use the same robot code. Current computer-in-loop work is useful; a separate Teensy-in-sim project adds less than testing its real drivers on real parts.

1. **Reconcile the build definition.** Resolve thin wheel regions, deck/pod mounting, arm/head heights, stow and full-path collision checks. Keep the active CAD and `ARM` configuration consistent; regenerate exports and rerun picking, stops and crossings. Do not require the parked catch build.
2. **Prove the compute/perception path.** On the candidate computer, score route, exploration and navigate-then-tidy with camera/face loads and honest real-time conditions. Compare Pi profiles if useful. Score D435 versus stereo and clutter navigation. Select the computer and camera from task results, not CPU scaling alone.
3. **Bench one wheel/pod and drive.** Print fit samples, then one wheel and pod after CAD gates. Measure grip, rolling resistance, wear, spring/damping and crossings. Implement CubeMars CAN; on a current-limited supply check encoder signs, timeout, motor response and thermal behaviour over the selected pack voltage range. Fit the models before ordering a set.
4. **Power and rolling base.** Verify fuse/switch/`BUS_EN`, rails, inrush and voltage/current readings. Wheels up first, then battery, then low-speed floor driving with representative arm mass/CG. Compare scripted acceleration, ramped stops, spins and transitions with the sim; increase speed only while the model remains credible.
5. **Navigation, then picking.** Calibrate odometry/IMU/lidar and frames; map the house, navigate at low speed, measure localization and run a soak. Calibrate arm servos and cameras; verify stow and waypoints before floor grasps. Progress to navigate-then-tidy, logging outcomes and replaying discrepancies.
6. **Catch later, by explicit resumption.** Bench the synchronized camera pair and actual ball, then calibration drift/latency, static catcher/feed and guarded launcher, before moving catches. Preserve the earlier camera and launch gates in Appendix B; they are not the arm build's critical path.

Physical sessions are attended: current-limited supply/wheels up before floor motion, low speed before adult profiles, guarded flywheels if catch resumes. Replace estimated parameters with sourced measurements; resolve unexplained sim/real gaps before proceeding.

## 9. Decision log (lasting choices only)

Detailed chronology is in the history; this table records choices that still define the project.

| Date | Decision / major finding |
| --- | --- |
| Oct 17–20 | Printed-first X4, low deck and ramped stops; big-roller sprung-pod base adopted for home floors. A2M12, BNO085 and Teensy selected. |
| Oct 18–19 | Catch studies established chute/pocket/tilt, drag-aware tracking and ceiling-aware returns; preserve for the parked payload. |
| Oct 21–22 | Shared firmware/SIL core and ROS stack; robot-served console; hardware clock sync and calibration/frames. |
| Oct 23–25 | SO-101 tidy payload, exact planner and wrist correction; shared payload layout and drawn v7 arm kit adopted. |
| Oct 27–29 | Nav2 up to 2 m/s adopted; C++ glue/catch cores, sleeping catch nodes and composition reduce compute. Final computer remains open. |
| Oct 30 | AK40-10 V3.0 KV170 replaces DM-2325 for better documented specifications; actual performance still to bench. |
| Oct 31 | Retain sprung pods; 140 mm wheel prototype with PLA/TPU and owned 6804s. Test before sets. Native editable SolidWorks v10 requested. |
| Nov 1 | Focus shifts to arm robot; catch and real-ball camera benches paused until global-shutter colour stereo and mobile robot exist. Nose D435 proposed for arm. |
| Nov 1 | `ARM` baselined to coarse v10; 37 mm riser / head +5 mm preserve floor reach, with CAD reconciliation outstanding. Earlier robot preserved as `ARM_V7`. |
| Nov 1 | Pi 4 default arm navigation fails; arm tier alone fits. `nav_profile:=pi` implemented as an untested hypothesis. |
| Nov 1 | OVONIC 6S 5200 mAh replaces assumed 7S in v11; 21 V provisional stop, 20% runtime reserve, no fictional BMS trip. |
| Nov 1 | Native assembly notes flag a colliding arm stow and unfinished deck/pod checks; keep these explicit before physical validation. Full design snapshot archived; baseline rewritten with maintenance rules. |
| Nov 1 | ARM sim drive model fitted to the AK40's maker curve (Fernando: fix the baseline now); plain-roller pins, smooth roller hull, mass-aware pod damping. Motor characterization study started; fork torque jacking found (4.7). |
| Nov 2 | AK40-10 confirmed by the sim motor study; fork pivot moved level with the axle in the sim baseline (torque jacking); the CAD fork to follow. |
| Nov 2 | Arm-first compute trade proposes used x86 + Teensy under the $400 PC cap, optional desktop AI and a separate colour GS head later; no adoption or new runs. Published camera modes distinguish Pi/AR0234 ~94 fps, full-resolution IMX296 60 fps and RVC2 OV9782 module 120 fps; complete catch pair remains unvalidated (4.9). |
| Nov 2 | Corrected interface count: four baseline USB endpoints; OAK optional, wrist camera TBD, analog mic/speaker provision. Hub becomes an optional expansion allowance. Mini-PC GPIO pulse sync, stereo trigger/frame integration, physical launch and BUS_EN allocation remain open; native Teensy USB and arm/lidar USB adapters fit the port plan (4.9). |
| Nov 2 | Fernando rejects Lenovo Tiny size; compact M5 Ultra 7730U / M8 shortlist replaces business Tiny recommendation. M5 at quoted $420 is under consideration, not adopted; RAM is replaceable and USB/timing/power need physical qualification (4.9). |
| Nov 2 | Fernando clarifies OAK is not baseline, wrist camera is TBD, and both microphone/speaker are required. Analog mini-PC audio is a provisional route; four baseline USB devices fit M5/M8 without a hub. |
| Nov 2 | Amazon follow-up retains M5/M8 price shortlist; exact selected CPU/RAM/audio/power and M8 case revision need qualification. External DC power is a purchasing filter; current AC-powered EQR6 is excluded. Expanded conditional candidates and dated price evidence live in the compute trade (4.9). |
| Nov 2 | Broader compact-PC search adds PELADN WO4/WO7, used NUC 12 Pro and $409 direct AM18 as conditional alternatives. No superior complete sub-$400 Amazon offer verified; AM18 USB-C power rating/protocol conflicts remain unresolved. No hardware or sim validation (4.9). |
| Nov 2 | Amazon-only PDF audit supersedes broader shortlist: BMAX B8A Pro 8745HS $436.99 performance stretch; Getorli AT103 i5-13420H $379.99 coupon budget option; M5 Ultra 7730U $449.99 documented-interface alternative. Captured offers, unadopted; Getorli DC/mic and BMAX contradictory power metadata remain VERIFY (4.9). |
| Nov 2 | `nav_profile:=pi` on the Pi 4: as first written it failed (Nav2 bond, slam grid), then localized 0.66 m off at 1 m/s. Reshaped to 0.6 m/s with slam loop closing kept, costmap waits fixed and `arm_node` at 20 Hz: a slow working arm robot, ~0.15 m localization, CPU pinned while driving (4.10). AMCL localized worse in the house. Most "Nav2 gave up" goals were the collision monitor stopping on late odom (its scan shift off in `pi`, P9). Baselined for a Pi driving the robot: the default profile on a Pi. Every profile now waits 0.5 s for the monitor's odom lookup (C8). |
| Nov 2 | Compact v11 wheel + motor (proposed): the AK40-10's XT30(2+2) plug mates radially (pins plus leads to r about 36), which ruled out burying the motor in the r 30 hub bore. Fernando chose to move the motor 27.5 mm outboard: the plug sits outside the hub, the hub rides on the motor output bearing and the left 6804, and the width is 84 mm. Output-bearing load is VERIFY (4.4). |
| Nov 3 | v11 suspension pod drawn (proposed): a drop-in module with two side frames on the plate, the hinge at plate height level with the axle, and the fork bolted to the AK40-10's rear face. ISO 10243 die springs on button-head plungers, no damper (RC shocks too soft, torsion springs too weak). The SolidWorks assembly moves through droop..bump. Motor bearing, mirror hand and plate open (4.4). |
| Nov 2 | Software review follow-up: latched firmware fault stop (LOW_BATT 1 s / MOTOR_ERR / MOTOR_SILENT in AUTO; protocol v5 adds `FAULT_STOP`); arm planner/trajectory fallbacks removed (the Tidy's way out of the bin clears the face keep-out only when planned from the commanded pose, not the lagging measured one); servo error replies count as unhealthy; console battery now 6S/21 V. SIL-verified only; firmware not flashed. |
| Nov 3 | PC stack regression: 8/8 house goals and 6/6 toys in both tested arm configurations; stale-TF/inactive-SLAM recovery and targeted console takeover fixed. Extend the tidy harness budget and score physical bin truth. Nav2 container shutdown remains an unresolved shutdown limitation (4.5–4.6). |
