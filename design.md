# Play-Catch Robot — Design Baseline

Condensed Oct 21, 2026 · Fernando (with Claude). This file holds only what a new session needs: the current design, its expected performance, important findings from designs that were not adopted, the sim strategy and options, where things live, next steps and the decision log.

**The full history is in [design full history.md](design%20full%20history.md)** (every study, table and the long decision log, 2,806 lines). Section numbers written "§N" below point there. Superseded documents are in `archive/`. Don't edit either; don't start parallel design docs.

Conventions: **(est)** = engineering estimate awaiting bench data. **VERIFY** = confirm on the real part. **Proposed** = not yet agreed with Fernando. No hardware has been bought or built; everything below is simulation and CAD.

## 1. Goal

A kiwi-style omni robot tracks a tossed 74 mm foam ball with stereo cameras, drives under it, catches it in a funnel on a tilting module and throws it back with a flywheel launcher. It is also a general mobile platform (lidar SLAM, RC driving, swappable top module, an arm later). Fun with kids (kid mode), mostly 3D printed (PLA / PETG / TPU, ~256 mm Bambu bed). Most tuning happens in MuJoCo; real sessions are short, attended calibration and validation runs.

## 2. Current design (the baseline)

The current robot is the **v7 base + v6 round-3 catch module + §24 actuators + v11 firmware / ROS code**, drawn to the fastener level in `cad/v7`. Status: Proposed, except where it says Fernando decided.

| Subsystem | Baseline | Notes |
| --- | --- | --- |
| Base | 4-wheel X (not the original 3-wheel kiwi), deck in the axle plane. Ø434 mm in CAD, 7.7 kg, CG 121 mm | v3b/§20: CG 95 mm, rim 387 mm in the sim; the CAD's masses are not in the sim yet |
| Wheels | **Printed 127 mm omni wheels, two rows of five 36 mm TPU-tread rollers** (Fernando, Oct 20), 515 g each in CAD | Crosses square rug edges an omni wheel otherwise can't (§25) |
| Suspension | Each wheel + motor on a **sprung, damped pod**: 15 mm travel, ~3 N/mm at the wheel (~12 N/mm at the shock), ζ 0.6; 48 mm trailing arm | No front rocker in CAD (sim v7 still has ±3°) |
| Drive motors | **4 × Damiao DM-2325** ($86, integrated FOC + encoder, CAN, 172 g), mounted inside the wheel | Data is reseller-only: bench two (+ a DM-J3507) before ordering a set. Fallbacks: DM-J3507, AK40-10 (~$937 set), M2006 + C610 on 6S |
| Battery | 7S1P Molicel P50B 21700 (5 Ah, 29.4 V full), BMS ≥ 40 A | Flat under the deck; swapping needs the robot on its side (rear drawer proposed) |
| Compute | Jetson Orin (Nano Super dev kit or NX 16GB) for perception, planning, SLAM; ROS 2 Jazzy | **VERIFY:** the Orin's JetPack 6 is Ubuntu 22.04, the stack is Jazzy (24.04): Docker on the Orin, or a JetPack 7 for Orin |
| MCU | **Teensy 4.1** (Fernando: assume for now), the safety authority; 29 of 42 pins used, CAN1 = motors, CAN3 (FD) = payload connector | Firmware written and running in the sim (§3) |
| Cameras | Stereo pair of OV9782 (1280×800, global shutter, 120 fps, MCU-triggered), 190 mm baseline, 30° up, on a stereo bar on two arms from the payload plate | Calibration stability is the biggest unknown (below) |
| Lidar / IMU | **RPLIDAR A2M12** at 150 mm scan plane, **BNO085** (Fernando, Oct 20) | Level mount, bench-calibrate plane tilt, filter floor returns, de-skew scans |
| Catch module | Tilting module (−30…+40°, catch tilt up to 35°): fabric-net funnel (Ø330 rim, 55° walls), **padded pocket**, **chute pocket** (floor sloping into the nip of two driven feed wheels, J-bend to the flywheels), IR beams for catch and feed-exit | One ball per trigger, 0 misfires |
| Feed wheels | Two small TPU wheels on **direct-drive gimbal-class motors that hold the ball with current** (LK-TECH MF4005 integrated, or GM3506 + SimpleFOC) in the proposal; the cost-first set uses a **brushed Pololu 25D on a RoboClaw** | §23.1 vs §24: decide after the bench |
| Launcher | 2 × Ø70 printed PETG/TPU flywheels, **T-Motor F90 2806.5 1300KV** on an **8S AM32 4-in-1 ESC** (bidirectional DShot); fixed elevation, aimed by turning the base; rpm → exit speed table in `launcher_cal.json` | Indoors 7 m returns need ~35–40° and ~4,500 rpm; spin-test flywheels to ≥ 5,000 rpm behind a guard |
| Tilt actuator | Feetech STS3250 (serial bus, encoder); a $30 Hiwonder HTD-45H might do | |
| Payload plate | On six Ø12 standoffs, `plate_gap` 0.096; 25 mm M3 grid, two Ø4 dowels, Ø28 power/CAN/USB hole | The lidar sees past everything but the standoffs (30° blocked) |
| Ball | 74 mm foam pickleball, ~22 g (proposed); drag ≈ 25% of weight at 6 m/s, so every predictor needs drag | Firmware ball spec ±0.5 mm unless the pocket is made size-tolerant (chute takes 72–76 mm) |
| Safety | Stops are **always ramped** (a stepped zero-speed stop at 1.4 m/s rocks the kiwi 23° and tips it at μ 1.2); e-stop cuts power; 0.5 s link watchdog → ramped stop, then disarm; firing needs AUTO, both flywheels within 3%, no bumper | Adult: 2 m/s cap, acceleration ~2.5 m/s² direction-aware (up to ~4 toward a wheel); kid: 0.8 m/s, 60% accel, 2,600 rpm (est) |
| Rules for parts | Printed PETG/PLA/TPU; **no heat-set inserts**; M3 thread-forming into 2.8 mm holes (≥ 6 mm engagement), 3.4 mm through; mostly M3; pan and flat heads only (Fernando) | CAD: 57 printed parts, 331 screws, 0 interferences at every tilt and suspension state |

**Cost-first actuator set (§24): ~$682 on 7S** (DM-2325 drive, Pololu 25D feed on DRV8874s, STS3250 tilt, F90 flywheels on an 8S AM32 4-in-1); ~$738 with a RoboClaw feed; ~$937 with the AK40-10 drive. Brushed gearmotors are out for the drive (overheat in continuous play).

**Software split.** *MCU firmware* (`firmware/lib/omni_fw`, C++17, board-free): modes (DISARMED / AUTO / MANUAL / ESTOP), the Jetson can only ask for less, X4 base controller (jerk-limited, direction-aware), odometry from DM-2325 positions + BNO085, flywheel rpm loop, one-ball feed, tilt, DM-2325 CAN, link. The catch's after-arrival sequence (coast, brake with the ball, hold, tilt dump 30 ms after arrival) runs here from the Jetson's predicted arrival time. *Jetson code* (`ros/src/omni_mcu`, `omni_catch`, ROS-free cores): `mcu_link`, `catch_perception` (colour-blob detector on sparse, rendered or real frames; image-space EKF with drag; a track starts only on a ball moving > 1.5 m/s), `catch_behavior`, `mode_manager` (IDLE / TELEOP / CATCH). The link: COBS + CRC-16 over USB, CMD 100 Hz (heartbeat), STATE 200 Hz, IMU 400 Hz, NTP-style time sync. Constants are generated from the sim (`robot_export.py`), like the CAD's `dims.py`. Lidar mapping/Nav2 use `slam_toolbox`, `robot_localization` and Nav2; v9's scan matcher on that map stays the catch-time localizer.

## 3. Expected performance (simulation)

| Metric | Result | Source |
| --- | --- | --- |
| Catch, thrower 6–7 m, expected cameras, flat floor | **93%** (216 throws, ±1.7); 99–100% at 1.5–6 m | §22, §30 |
| Same through the robot's real code (C++ firmware at 1 kHz + Jetson code + link) | 93.1% in one process; **85% ± 5 (41/48) through the full ROS stack** at ≈ 1.0× real time | §30 |
| Home floor vs flat (v7) | 89% vs 92%; a 45°-turned rug 88% (v6: 68%) | §25 |
| Randomized worlds (192) | **86%** (per-world median 89%, 10–90% 72–97%); calibration 0.04° → 97%, 0.3° → 78% | §23.3 |
| Camera pipeline estimate | 86–92% expected, 72% pessimistic (slow stacked pipeline), 85% at 0.3° calibration drift | §21.4 |
| Return throws | Median 3 cm from the thrower at 6.5 m (sim upper bound); 25 cm through ROS | §22, §30 |
| Indoors, 8 ft ceiling | Ceiling-aware return (~35–40°, ~4,500 rpm) reaches 7 m, 0 ceiling hits, 7 cm median | §22.5 |
| Floor crossings (v7) | 8 / 12 mm square rugs, 6–18 mm thresholds at all 24 body directions, no airborne phase, 0 balls lost in ~11,000 crossings | §25 |
| Cost of v7 over v6 | +1 kg, CG +0.8 cm, ~2.7 points on a flat floor; speed margin on a low pack +22% (AK40-10), +57% (DM-2325) | §25 |
| Localization with lidar | ~1.2 cm median, < 3 cm at 95%, 0.5° at up to 2 m/s (v9 matcher); slam_toolbox + EKF 9 cm median | §27, §28 |
| Map / Nav2 | 2.1 cm median wall error; 24 of 24 goals | §28 |
| Launcher (v2 bench) | rpm 1000/2000/3000/4000/5000 → 2.4/5.1/7.7/10.2/12.6 m/s; transfer ~0.70 of rim speed | `launcher_bench.py` |
| Power | Worst burst 12.6 A; 28 W average (Jetson ~22 W) → ~3.9 h | v2 |

Key lessons behind these numbers:

- **Perception, not drive or catcher, limits the catch rate:** latency and frame rate, then stereo calibration stability (0.5° drops catches to 58%), global shutter and hardware sync. Spin of real throws (10 rad/s doubles landing error) and camera latency (50 ms assumed) are unmeasured.
- **The honest stereo track start** (covariance from the stereo geometry, not a fixed ±5 cm) took 6–7 m throws from 75% to 96%; roller feed + padded pocket + 35° catch tilt took the module from 80% to 99% (static rig). Padding alone or tilt alone each reach ~89%.
- **A thrower farther than ~4 m was a problem for v5, not for v6:** with the v6 module the throw line can be 6–7 m without high lobs.
- Traction, not torque, limits acceleration. The tip lever of a 3-wheel kiwi is half its wheel-circle radius; the X4 has a larger support polygon.

## 4. Important findings from designs that were not adopted

| Design | Finding to keep | Where |
| --- | --- | --- |
| **v2 baseline** (3-wheel kiwi, 96 mm goBILDA, AK40-10, J-bend gravity feed) | Stop-and-catch inside a 0.65 m radius is easy with perfect perception; real error budget ±11 cm. Gravity J-feed can't serve a 52° launch from over-under wheels. Wheels 125 → 96 mm saved weight but the home-floor base returned to 127 mm. The kiwi tips at ~2.2–2.5 m/s² toward the wheel-triangle edge | §2, §3 |
| v3 compact | The X4 base, axle-plane deck, tilting module and base-mounted cameras. Rim 494 → 387 mm; the plunger feed (replaced by v6 roller / chute feed) | §20 |
| v5 unlimited | Real firmware limits cost little; a **bare funnel stops after 2 balls**: multi-ball needs a single-file magazine tube; camera spec by what the sweep says matters | §21 |
| v6 | A tennis-style indexer wheel feeds but catches only 20%; stirrers/vibrators unreliable; the **tube insert** is the multi-ball path (open: a catch path that bypasses an indexer). Pinching the resting ball for size tolerance perches caught balls on the tires. Missed balls under a wheel or a threshold hop flipped the robot: bumper skirt to ~20 mm, wheel-top guards, ≤ 1 m/s over mapped thresholds | §22, §23.4 |
| v6 motor study | Keep four wheels (three lose 11 points); the AK40-10 uses a third of its torque; DJI M2006 catches the same for less | §23.5 |
| **v7 rejected options** | Cross-axis swing pods (lower the chassis instead of lifting the wheel), a rear rocker, mecanum (sticks, pitches 44° over 18 mm), 32 mm-roller wheels with the edge map (lose 13 points on a 45° rug), 152 mm wheels | §25 |
| v7 terrain assist | Turning away from the ball near an edge cost up to 30 points; use the edge map only for 16 mm+ edges, never during a catch | §25.4 |
| **v8 swerve** | Better terrain (16 / 20 mm rugs, 5× better odometry, ~$420 vs $444, 0.7 kg lighter) but 87.5% vs 90% at home (the triangle rocks at the catch), 47 cm footprint, 21 mm belly clearance, 3 more actuators. **Keep v7 for the catch robot**; revisit for plush rugs or an arm | §26 |
| v9 sensors | The PMW3901 flow sensor is the wrong part (blind under 80 mm); a PAA5100JE is blind above ~1.1 m/s and over rugs: optional or dropped. An arm base is limited by tipping (11–16 N at 35–50 cm), not positioning | §27 |
| v10 ROS bridge | Two scan fixes the real robot needs: drop floor returns (a 1.2° tilted A2M12 13 cm up sees the floor at 6.4 m) and de-skew scans. Small-file I/O in the Dropbox folder is ~110× slower: build in WSL ext4 | §28 |
| v7 CAD | The motor can't sit behind the 80 mm wheel (→ in-wheel); the printed wheel is 515 g not 285; the lidar would see only ~184° at the sim's position (fixed to 330° by the six-standoff plate); the sim's camera spot is swept by the flywheel motors | §29 |

## 5. Simulation strategy and options

**Strategy.** MuJoCo is the physics master (not Gazebo). Each version copies the previous one and adds one layer, so old results stay reproducible and `SimParams()` defaults mostly stay put. The path: physics and catch game (v2–v6) → home floors (v7–v8) → sensors and localization (v9) → ROS 2 in WSL against the sim (v10) → the robot's own code in the loop: MCU firmware as software-in-the-loop plus the Jetson's ROS code (v11) → hardware in the loop (next). Every change must pass the version's tests. **Key trust metric once hardware exists: the gap between simulated and real catch rate for the same settings.** Real data to feed back: motor step responses, end-to-end latency, ball drag/mass/restitution, funnel bounce, roller μ, launcher calibration (50–100 logged shots). Randomize each parameter within its measured range.

| Sim | What it is | Tests / speed |
| --- | --- | --- |
| v1 hello world | Simple kiwi + catch loop | 10 tests |
| **v2 detailed physics** | Baseline physics: motor/battery/thermal, omni wheels (sphere or explicit rollers), physical flywheels with ESC, trapdoor gate, foam ball with drag, lossy net. Ground-truth perception | 7 tests; ~4× real time |
| v3 compact (§20) | X4 base, tilting module, stereo perception model | 13 tests (`test_v3.py`) |
| v5 unlimited (§21) | v3b with `limits="firmware"` off by default, per-pixel vision model, hopper mode (no v4) | 25 tests |
| **v6 shallow throw** (§22–23) | 6–7 m throws, chute feed, padded pocket, 35° tilt, honest track start, ceiling-aware returns, hopper options; round 3: GM3506 feed motors, viscoelastic contacts (`contact_model.py`), randomized worlds (`randomize.py`), floors (`floor.py`), motor study | 43 tests; ~1.5× real time |
| **v7 home floor** (§25) | v6 + big-roller wheels, sprung pods, terrain-aware firmware, mecanum and swing-pod options, `house.py` (drive with keyboard / gamepad / flight stick). Crossings need `wheel_model="rollers"` | + 11 tests (`test_v7.py`) |
| v8 swerve (§26) | v7 + 3/4-module swerve (`layout="swerve3"`); `house.py` defaults to the swerve | + 6 tests |
| v9 sensors (§27) | v8 + `sensors.py` (IMU, encoders, flow, RPLIDAR) and `estimator.py` (EKF, map scan matcher, `Localizer`) | + 5 tests |
| v10 ros bridge (§28) | v9 + `bridge_core.py` (no ROS), A2M12 / BNO085 models, `map_quality.py` | 66 tests |
| **v11 sil** (§30–31) | v10 + the real firmware ticked at 1 ms (`sil_robot.SilRobot`), the Jetson's catch code (`sil_game.py` or ROS nodes), `robot_export.py`, `scene="house"` (furnished house, decoration only), `frames="rendered"` (MuJoCo renders through a camera sensor model, sim time only). **Default sim for `ros/omni.sh`** | + 19–22 tests |

Which sim to use: **v11** for anything about the robot's code, the ROS stack, the catch end to end; **v7** (inside v11) for the current robot's physics; **v6** for catch-module / feed / vision studies; **v2** only for its own regression. `simulation/v7`'s `house.BASES["v7"]` is the proposed base.

**MuJoCo lessons (read before changing the models)**

1. Frame sensors with `objtype="body"` report the inertial frame; use `xbody`.
2. Soft-contact stiffness scales with effective mass: a ball pinched between wheels gets 250 N. Model squeeze compliance as sprung slides, keep the contact stiff.
3. Friction-critical contacts are explicit `<pair>`s with `solreffriction 0.002`.
4. Flywheel shots above ~4,000 rpm are erratic at a 1 ms step; v2 uses 0.5 ms.
5. Thin collision panels let a 6.5 m/s ball tunnel through: 10 mm panels.
6. Sphere-wheel contact bodies need ≥ 0.06 kg.
7. Roller bearing drag scales with (r_roller/R)².
8. Keep Python out of the inner loop (plain floats for the drag predictor).
9. build123d: extrusion follows the face normal; touching faces don't fuse (overlap 0.5 mm).
10. A 2D texture shows undistorted only on a box's local ±z faces. Don't draw MuJoCo inside WSL (D3D12 ~0.8 s/frame): the 3D view is `ros/tools/mujoco_view.py` on Windows.

**Not modelled / honest limits:** the CAD's masses (wheel 515 g, robot 7.7 kg, CG 121 mm), no front rocker, Ø434 mm bumper, CAD camera positions, plate height and lidar mask are not in the sim. Rendered images lack global illumination, auto exposure, Bayer artefacts, lens distortion; rendered runs run ~0.03× real time and are not bit-reproducible. Perception noise values, contact parameters and floor μ are guesses until measured.

## 6. Where things live

```
omni-robot/
├── design.md                 ← this file;  design full history.md ← everything (§N)
├── archive/                  ← superseded docs
├── todo.md, Launcher.cmd + launcher.ps1   ← Fernando's button launcher (keep it working when steps change)
├── simulation/               ← README.md (venv, version index); v1 … v11 as in section 5
│   ├── v2 detailed physics/  ← catalog.py (part data), params.py (fitted config), models.py, robot_model.py, robot.py, catch_game.py, run.py, launcher_bench.py, studies/
│   ├── v3 compact/ v5 unlimited/ v6 shallow throw/ v7 home floor/ v8 swerve/ v9 sensors/ v10 ros bridge/   ← each has README.md, test_vN.py, studies/ (results/ holds JSON, plots, logs)
│   └── v11 sil/              ← sil_robot.py, catch_world.py, sil_game.py, sil_link.py, robot_export.py, house_scene.py, rendered_camera.py, studies/
├── firmware/                 ← Teensy 4.1 (README, PINS.md): lib/omni_fw (board-free core), src/main.cpp (drivers), sil/ (SIL library: zig c++ on Windows, g++ in WSL)
├── ros/                      ← omni.sh (every ROS step, in WSL), src/omni_bringup omni_perception omni_interfaces omni_mcu omni_catch (→ Jetson), omni_sim (PC only), maps/, foxglove/, tools/
├── database/                 ← shared actuator DB: motors/servos/controllers/motor_tests/sources CSV + actuator_db.py (981 motors, 106 servos, 60 controllers, 424 dyno tests; every value has a source); keep it free of omni-robot imports
└── cad/                      ← v7/ = current robot to the fastener level (build.py, render.py, view.py, step_export.py; out/ has STEP, STL, assembly.step, BOM.md, PRINT_LIST.md); the top level is the v2 CAD
```

**Environment**

- Python: `$py = C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe` (MuJoCo 3.16, build123d 0.13). `python` on PATH is PlatformIO's: never install into it.
- ROS runs in WSL (`wsl -d Ubuntu-24.04`, Jazzy, Python `~/.venvs/omni-ros`); always use `ros/omni.sh <step>`, which mirrors `ros/src` and the sim into `~/omni_ws` (ext4) and builds there. Never build on `/mnt/c`. Several runs at once need separate `ROS_DOMAIN_ID`s.
- Sister project with motor / battery / thermal models: `C:\Dropbox\Personal Projects\Robotics\wheeled-leg-robot\simulation\mujoco\v4_twin_279mm_baseline`.
- Flashing only in attended sessions. `protocol.h` and `omni_mcu/protocol.py` change together. Rerun `robot_export.py` after any params / catalog / model change (a test checks).

**Common commands** (from the folder shown)

| Folder | Command | What it does |
| --- | --- | --- |
| repo root | double-click `Launcher.cmd` | Buttons for the ROS steps, Foxglove, the joystick, the Windows sims, CAD view |
| `ros` (WSL) | `bash omni.sh map` / `save-map` / `localize` / `nav` / `sim` / `catch` (`dist:=6.5`) / `catch-score tosses:=24` / `test` / `stop` | The ROS 2 stack against the sim |
| `simulation/v11 sil` | `& $py sil_game.py --view --returns`; `--frames rendered --scene house` | Catch game on the real code in one process; on rendered images in the furnished house (`house_scene.py fetch` once) |
| `simulation/v11 sil/studies` | `sil_catch.py both` / `report`; `rendered_frames.py --png`; `rendered_catch.py run --frames rendered --seeds 3 --workers 12` | SIL vs the sim's own game on 216 throws; detector on renders; the 216-throw grid on rendered vs sparse frames (~35 min) |
| `firmware` | `pio run`; `& $py sil\build_sil.py --pins` | Teensy build (checks the pin map); the SIL library and PINS.md |
| `simulation/v7 home floor` | `& $py house.py` | Drive the v7 base through a house (keyboard, gamepad, flight stick) |
| `simulation/v7 home floor/studies` | `home_floor.py A` / `B` / `C` / `report` | Crossings, catch games, costs |
| `simulation/v9 sensors` | `house.py --base v7 --sensors --camera top`; `studies/localization.py` | Fused estimate live; localization study |
| `simulation/v6 shallow throw` | `watch_floor.py` ; `studies/actuator_choice.py` | Watch thresholds / rugs; cost-ranked actuator study |
| `simulation/v2 detailed physics` | `run.py` (`--headless -n 30`, `--drive`) ; `-m pytest -q` | Catch demo; regression checks |
| `database` | `actuator_db.py validate`; `-m pytest -q` | After any database edit |
| `cad/v7` | `build.py` (`--quick`), then `build.py --render-only; render.py`; `view.py` | STEP/STL, BOM, clash checks over tilt and suspension; interactive view |

After CAD changes run `build.py` (must report 0 clashes). Run `python -m pytest -q` in the sim folder you touched.

## 7. Next steps

1. **Re-simulate with the CAD's findings** (v7 sim): masses (wheel 515 g, +0.75 kg sprung), no rocker, bumper ring r 205–217, CAD camera positions, `plate_gap` 0.096; then the v10 stack with the A2M12 at 150 mm and its standoff mask; recalibrate `feed_clear_travel` for the moved feed-exit beam.
2. **Bench, in this order:** stereo calibration stability on the robot (the dominant unknown); two DM-2325s + a DM-J3507 (torque constant/peak, undervoltage cutoff on a low 7S pack, winding temperature at ~0.4 N·m RMS, backlash, comm-loss behaviour, CAN ids and feedback ranges; record in `meas_*` columns) → decide the drive; throw spin and camera latency (LED + photodiode); foam-ball drop tests; one printed 127 mm wheel + one pod (roller grip and rolling resistance on wood/tile/carpet, pod rate and damping, hand-push an 8 / 12 mm square rug edge); a feed-wheel pair + printed chute and J with the exit beam.
3. **Hardware in the loop:** write the stub drivers (BNO085 SH-2 over SPI, bidirectional DShot, CRSF); with a Teensy 4.1 and one DM-2325 check the CAN against `dm_motor.h`, the USB link latency and the 1 kHz loop CPU time; schedule the coast / brake / dump on an arrival time refined after the freeze (SIL bounced 5 balls to the sim's 1).
4. **Jetson:** profile slam_toolbox + Nav2 and the catch code on its CPU (C++ or GPU detector planned; the acquisition search is Python); resolve the JetPack / Jazzy question; confirm OV9782 driver support (1280×800 at 120 fps, frame-sync pin) for the carrier.
5. **Software gaps:** port the terrain assist (edge map) into the firmware or keep it on the Jetson as speed caps (and decide where the edge map comes from: app markup, IMU-bump learning, stereo depth); feed the lidar fixes into the catch-time estimator; model calibration error as a slowly varying bias in the tracking filter; feed rendered images into ROS in real time from a Windows render server (2–5 ms a frame; `catch_perception source:=image` is ready); port v9's matcher to the ROS side.
6. **Design gaps:** a rear battery drawer; a counterbalance spring on the tilt pivot; a bumper skirt and wheel-top guards (done in CAD); a multi-ball path (tube insert now); choose the shock spring (~12 N/mm at the shock); rework the deck for the 80 mm wheel (motor 21 mm inward).
7. Print and bench: one wheel (3 hubs, 10 rollers, 10 pins) and one pod, with a DM-2325, before printing the rest. SolidWorks: open `cad/v7/out/assembly.step`.
8. Fill the database gaps (`database/gaps.csv`), prices first. Spec sweep over perception latency, noise and frame rate is still open.

## 8. Open items (VERIFY)

- **Decisions:** the drive (DM-2325 vs fallbacks); Jetson tier and carrier; feed motor (MF4005 / GM3506 vs Pololu 25D); chute pocket vs round-1 roller pocket; tube vs indexer for multi-ball; the proposed limits (2.5 m/s² direction-aware, 74 mm / 22 g ball, honest track start, 35° tilt cap).
- **Parts:** DM-2325 torque labels (1.35 / 5 N·m assumed), supply range and comm-loss timeout; DM-J3507 and AK40-10 voltage rating on 7S; STS3250 continuous torque; F90 pole pairs (7 assumed); RoboClaw baud/counts; 8S ESC with bidirectional DShot under AM32; A2M12 scan-plane tilt (±1.5°) and range noise at 1/3/6/10 m, `rplidar_ros` direction and stamp; BNO085 drift (CEVA datasheet); shaft loads on the DM-2325 and the tilt servo.
- **Physics:** roller μ on the real floor (0.95 assumed) and TPU-roller grip (0.9 assumed); rolling resistance (Crr 0.02 / 0.06 on floor / carpet assumed); foam ball diameter, mass, Cd, restitution (0.55 / 0.38 floor and 0.10 / 0.04 pad assumed); squeeze force on a flywheel; feed-wheel preload and TPU-on-foam grip; net rolling resistance; ball spin; room ceiling height; real rug and threshold heights.
- **Camera:** end-to-end latency (50 ms assumed; 22–26 ms raw V4L2, 40–50 ms Argus estimated); stereo calibration (0.02° assumed); OV9782 shutter/ISP behaviour on the Orin.

## 9. Decision log (condensed)

| Date | Decision | Reason |
| --- | --- | --- |
| Oct 16 | Kiwi base, AK40-10, 7S Li-ion, Jetson + MCU, ROS 2, stereo OV9782, MuJoCo-first | Original design |
| Oct 17 | Printed-first mechanics (PLA / PETG / TPU); 74 mm foam pickleball (proposed); stops always ramped; adult mode 2 m/s, moving catches only with a 20 cm clear path | Fernando; a stepped zero-speed stop tips the kiwi |
| Oct 17 | v3 compact X4 base with tilting module and axle-plane deck (**proposed**); keep the X4 omni for catching, swerve only as the upgrade path | CG 142 → 95 mm; swerve only +2–4 points |
| Oct 17 | Camera spec (proposed): MCU-triggered stereo, stiff bar, raw V4L2 if Argus > ~50 ms, 1280×800 at 120 fps | Pipeline 86–92%; calibration drift and a slow stack are the risks |
| Oct 18–19 | v6: roller feed, padded pocket, 35° tilt, honest track start; then the **chute pocket**; ceiling-aware returns; multi-ball via a magazine tube (**proposed**) | 6–7 m throws 57% → 96%; 0 misfires; indexer catches only 20% |
| Oct 19 | Feed wheels on direct-drive gimbal motors holding with current (MF4005 integrated, or GM3506); viscoelastic contacts and rolling resistance are v6 defaults; measure stereo calibration first | Feeds 72–76 mm at every tilt; calibration 0.04 → 0.3° costs 19 points |
| Oct 19 | Keep 4 wheels; skirt to ~20 mm and wheel guards; ≤ 1 m/s over thresholds; no square rug edges | Missed balls and threshold hops flipped the robot |
| Oct 20 | Shared actuator database in `database/`; **7S pack with Damiao DM-2325 drive** (~$682 set), bench two first | Cheapest drive that passes and is integrated (Fernando prefers integrated devices) |
| Oct 20 | **v7 base adopted by Fernando: the larger wheels** (127 mm big-roller printed omnis on sprung pods); swing pods, rear rocker, mecanum, 32 mm rollers rejected | Crosses all rug edges and thresholds; home 89% vs 92% flat |
| Oct 20 | **A2M12 lidar and BNO085 IMU (Fernando)**; PMW3901 dropped; lidar fixes go into the catch-time estimator | Motion over a 1 s catch 13–20 mm with lidar vs 42–65 mm without |
| Oct 20 | Swerve explored (v8); **keep the v7 X4 for the catch robot** | Terrain better, catching 2.5 points worse, bigger footprint |
| Oct 20 | ROS 2 Jazzy in WSL2 Ubuntu 24.04, built in `~/omni_ws` from `ros/omni.sh`; monorepo with `ros/`; joystick via TCP relay; MuJoCo view drawn on Windows + Foxglove, not Gazebo; slam_toolbox for the map and Nav2, v9 matcher for catch time | I/O speed in Dropbox; no admin rights; D3D12 draw speed |
| Oct 21 | **v7 CAD rules (Fernando):** no heat-set inserts, M3 thread-forming into 2.8 mm, pan/flat heads only; payload plate on six standoffs (A2M12 sees past everything but them) | Fernando's stock and review |
| Oct 21 | **Start the robot's real code (Fernando):** Teensy 4.1 firmware (board-free core, pin map checked by `static_assert`) in the loop as SIL; Jetson ROS packages `omni_catch`, `omni_mcu`; COBS + CRC link, CMD 100 Hz heartbeat; after-arrival sequence on the MCU; camera pipeline takes sparse, rendered or real frames through one interface | Real code must be tested against the sim; the sim's Python firmware used the true pose |
| Oct 21 | v11 round 2: furnished house scene and rendered-camera frames (proposed); acquisition change in the detector | Realistic camera input; results table (§31.3) was never filled in the history file |
