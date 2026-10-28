# Play-Catch Robot — Design Baseline

Condensed Oct 21, 2026 · Fernando (with Claude). This file holds only what a new session needs: the current design, its expected performance, important findings from designs that were not adopted, the sim strategy and options, where things live, next steps and the decision log.

**The full history is in [design full history.md](design%20full%20history.md)** (every study, table and the long decision log, 2,806 lines). Section numbers written "§N" below point there. Superseded documents are in `archive/`. Don't edit either; don't start parallel design docs.

Conventions: **(est)** = engineering estimate awaiting bench data. **VERIFY** = confirm on the real part. **Proposed** = not yet agreed with Fernando. No hardware has been bought or built; everything below is simulation and CAD.

## 1. Goal

A kiwi-style omni robot tracks a tossed 74 mm foam ball with stereo cameras, drives under it, catches it in a funnel on a tilting module and throws it back with a flywheel launcher. It is also a general mobile platform (lidar SLAM, RC driving, swappable top module, an arm later). Fun with kids (kid mode), mostly 3D printed (PLA / PETG / TPU, ~256 mm Bambu bed). Most tuning happens in MuJoCo; real sessions are short, attended calibration and validation runs.

## 2. Current design (the baseline)

The current robot is the **v7 base + v6 round-3 catch module + §24 actuators + v11 firmware / ROS code**, drawn to the fastener level in `cad/v7`. Status: Proposed, except where it says Fernando decided. **Fernando baselined the drawn layout on Oct 25** (section 12.2-12.3): the head (camera bar, 5 in face), the Ø8 front rods, and the arm kit for the SO-101; the v11 sim's robot is the drawn one. Also Oct 25: the face redrawn for the 5 in 800×480 at the front, the console's Say box (text to speech with an expression), antics, voice commands, the Thrower bench mode's 25° floor (firmware too), and the case for offloading to the operator PC (section 13). **Oct 26: CAD v8 (`cad/v8`, proposed, section 14)**: a fork round each wheel (a second 61808 outboard, bumper Ø458), hex-drive pan heads, magenta fasteners, labelled boards, stronger cheeks, both robots as STEP. Oct 26: the face talks less, speaks with Kokoro's neural voice, and has a persona per mission (Patrol stern, Explore adventurous, Tidy joyful). **Oct 27: Fernando baselined Nav2's speed**: up to 2.0 m/s, the firmware's adult limit (Go To's default; Nav2 slows for each goal by itself). The console's **Driving** choice (forward by default, agnostic, spinning) decides which way the robot faces on the way. **Oct 27: electronics (section 15, proposed)**: the block diagram, one custom PCB (the Teensy's carrier), the power side off the shelf first. **Oct 27: CAD v8 round 2 (`cad/v8`, proposed, section 14.1)**: each wheel's fork pivots on a shaft held on both sides of the wheel, with a shock on each side; the face, speaker and mics on a mast at the rear, the same on both robots (0.54 m tall); one electronics bay; a round on/off switch instead of the e-stop; the bumper skirt at 34 mm; Ø462.

| Subsystem | Baseline | Notes |
| --- | --- | --- |
| Base | 4-wheel X (not the original 3-wheel kiwi), deck in the axle plane. Ø434 mm in CAD, 8.4 kg with the display, CG 127 mm | v3b/§20: CG 95 mm, rim 387 mm in the sim. The sim has the CAD's masses since Oct 25 (`robot_export.drawn_params`: 8.19 kg, CG 125 mm) |
| Wheels | **Printed 127 mm omni wheels, two rows of five 36 mm TPU-tread rollers** (Fernando, Oct 20), 515 g each in CAD | Crosses square rug edges an omni wheel otherwise can't (§25) |
| Suspension | Each wheel + motor on a **sprung, damped pod**: 15 mm travel, ~3 N/mm at the wheel (~12 N/mm at the shock), ζ 0.6; 48 mm trailing arm | No front rocker in CAD (sim v7 still has ±3°) |
| Drive motors | **4 × Damiao DM-2325** ($86, integrated FOC + encoder, CAN, 172 g), mounted inside the wheel | Data is reseller-only: bench two (+ a DM-J3507) before ordering a set. Fallbacks: DM-J3507, AK40-10 (~$937 set), M2006 + C610 on 6S |
| Battery | 7S1P Molicel P50B 21700 (5 Ah, 29.4 V full), BMS ≥ 40 A | Flat under the deck; swapping needs the robot on its side (rear drawer proposed) |
| Compute | Jetson Orin (Nano Super dev kit or NX 16GB) for perception, planning, SLAM; ROS 2 Jazzy | **VERIFY:** the Orin's JetPack 6 is Ubuntu 22.04, the stack is Jazzy (24.04): Docker on the Orin, or a JetPack 7 for Orin |
| MCU | **Teensy 4.1** (Fernando: assume for now), the safety authority; 29 of 42 pins used, CAN1 = motors, CAN3 (FD) = payload connector | Firmware written and running in the sim (§3) |
| Cameras | Stereo pair of OV9782 (1280×800, global shutter, 120 fps, MCU-triggered), 190 mm baseline, 30° up, on **one camera bar** (20 × 20 aluminium tube) at the plate's front edge, indexed 30° up (catch) / 40° down (arm): optical centres at (192, ±95, 211) mm | Drawn in CAD Oct 24 (section 12.2, `cad/v7/head.py`). Calibration stability is the biggest unknown (below) |
| Lidar / IMU | **RPLIDAR A2M12** at 150 mm scan plane, **BNO085** (Fernando, Oct 20) | Level mount, bench-calibrate plane tilt, filter floor returns, de-skew scans |
| Time, frames, calibration, tags | MCU-triggered cameras + strobe, 1 Hz SYNC_OUT to a Jetson GPIO (clock map: offset + drift), every MCU time stamped at the event (protocol 4); REP-105 frames from one generated description; `omni_calib` workflow; AprilTag landmarks + localization health (`omni_slam`) | Proposed, section 10. Teensy pins 24–27, 29 added (PINS.md) |
| Catch module | Tilting module (−30…+40°, catch tilt up to 35°): fabric-net funnel (Ø330 rim, 55° walls), **padded pocket**, **chute pocket** (floor sloping into the nip of two driven feed wheels, J-bend to the flywheels), IR beams for catch and feed-exit | One ball per trigger, 0 misfires |
| Feed wheels | Two small TPU wheels on **direct-drive gimbal-class motors that hold the ball with current** (LK-TECH MF4005 integrated, or GM3506 + SimpleFOC) in the proposal; the cost-first set uses a **brushed Pololu 25D on a RoboClaw** | §23.1 vs §24: decide after the bench |
| Launcher | 2 × Ø70 printed PETG/TPU flywheels, **T-Motor F90 2806.5 1300KV** on an **8S AM32 4-in-1 ESC** (bidirectional DShot); fixed elevation, aimed by turning the base; rpm → exit speed table in `launcher_cal.json` | Indoors 7 m returns need ~35–40° and ~4,500 rpm; spin-test flywheels to ≥ 5,000 rpm behind a guard |
| Tilt actuator | Feetech STS3250 (serial bus, encoder); a $30 Hiwonder HTD-45H might do | |
| Payload plate | On six standoffs (four printed Ø12, the front pair Ø8 6061 rods), `plate_gap` 0.096; 25 mm M3 grid, two Ø4 dowels, Ø28 power/CAN/USB hole | The lidar sees past everything but the standoffs (22° blocked, widest wedge 8°) |
| Ball | 74 mm foam pickleball, ~22 g (proposed); drag ≈ 25% of weight at 6 m/s, so every predictor needs drag | Firmware ball spec ±0.5 mm unless the pocket is made size-tolerant (chute takes 72–76 mm) |
| Safety | Stops are **always ramped** (a stepped zero-speed stop at 1.4 m/s rocks the kiwi 23° and tips it at μ 1.2); a round on/off switch and the radio, no e-stop button (Fernando, Oct 27; CAD v8 round 2, section 14.1); 0.5 s link watchdog → ramped stop, then disarm; firing needs AUTO, both flywheels within 3%, no bumper, a launch ≥ 25° (tilt ≤ +5°: the face) | Adult: 2 m/s cap, acceleration ~2.5 m/s² direction-aware (up to ~4 toward a wheel); kid: 0.8 m/s, 60% accel, 2,600 rpm (est) |
| Rules for parts | Printed PETG/PLA/TPU; **no heat-set inserts**; M3 thread-forming into 2.8 mm holes (≥ 6 mm engagement), 3.4 mm through; mostly M3; pan and flat heads only (Fernando); fillets on load paths, lead-ins on bearing seats, first-layer chamfers where faces mate | CAD: 61 printed parts (167 prints), 362 screws (377 fasteners), 8.2 kg; 0 interferences at every tilt and suspension state and over the camera bar's index swing; every screw clamps something and can be driven; every print one solid (`cad/v7/README.md`: assembly order) |
| Face and voice | **5 in 800×480 HDMI display at the front**, between the cameras, leaning back 10° behind a 2 mm polycarbonate window (baseline, Oct 25, section 12.2); **USB 2-mic array (echo cancelling) and a 3 W speaker** in the audio box at the plate's rear (Fernando, Oct 22) | Every viewer in front sees all of it; the launch line clears it from 24° up, so nothing fires flatter than 25°. Software: `omni_face` (below), drawn for the 800×480 (Oct 25) |
| Arm kit (the other payload) | The SO-101 at the plate's front edge on a 50 mm riser and an adapter over the camera bar (base origin x 140 mm), the bar at its arm index (40° down), a 120 × 120 × 70 bin on the plate's rear, the same face unit and audio box on two uprights over it (baseline, Oct 25, section 12.3) | Floor grasps 0.26–0.42 m from the centre (fingers straight down 0.26–0.36); the tidy 102/120 in the sim; swap list `cad/v7/out/ARM_KIT.md` |

**Cost-first actuator set (§24): ~$682 on 7S** (DM-2325 drive, Pololu 25D feed on DRV8874s, STS3250 tilt, F90 flywheels on an 8S AM32 4-in-1); ~$738 with a RoboClaw feed; ~$937 with the AK40-10 drive. Brushed gearmotors are out for the drive (overheat in continuous play).

**Software split.** *MCU firmware* (`firmware/lib/omni_fw`, C++17, board-free): modes (DISARMED / AUTO / MANUAL / ESTOP), the Jetson can only ask for less, X4 base controller (jerk-limited, direction-aware), odometry from DM-2325 positions + BNO085, flywheel rpm loop, one-ball feed, tilt, DM-2325 CAN, link. The catch's after-arrival sequence (coast, brake with the ball, hold, tilt dump 30 ms after arrival) runs here from the Jetson's predicted arrival time. *Jetson code* (`ros/src/omni_mcu`, `omni_catch`, ROS-free cores): `mcu_link`, `catch_perception` (colour-blob detector on sparse, rendered or real frames; image-space EKF with drag; a track starts only on a ball moving > 1.5 m/s), `catch_behavior`, `mode_manager` (IDLE / TELEOP / CATCH / THROWER; it sends the MCU one fire counter that advances once per new request of the source in control, so switching sources never fires a ball). The link: COBS + CRC-16 over USB, CMD 100 Hz (heartbeat), STATE 200 Hz, IMU 400 Hz (each sample at its own time), INFO 10 Hz (protocol version 4, the firmware's build id = a hash of `lib/omni_fw` from `firmware/fw_build_id.py`, the RC receiver's channels, each DM-2325's winding and driver temperature), TIME events (sync pulse, camera triggers, strobe); the clock map from the sync pulses, the NTP-style echo as fallback (section 10). Constants are generated from the sim (`robot_export.py`), like the CAD's `dims.py`. Lidar mapping/Nav2 use `slam_toolbox`, `robot_localization` and Nav2; v9's scan matcher on that map stays the catch-time localizer.

**Operator console** (`ros/src/omni_console`, Oct 21–22, every tab working against the sim). This is how Fernando sees and commands the robot from the PC or an iPad. It is a web page that the robot serves itself at `http://<robot>:8080` (on the robot `http://omni.local:8080`); in the sim that is `http://localhost:8080`.
- **The page is the launcher.** A small supervisor (`supervisor.py`, standard library, no ROS) serves the page and starts and stops the robot's software. On the Jetson systemd starts it at boot and it starts the `robot` step at once (`systemd/README.md`; VERIFY on the robot); in the sim `omni.sh console` (the Launcher's "Console (start here)" button) does the same in WSL. Home has a card per mode; Robot > System starts, stops and restarts steps (the sim's catch world and scored runs too), shows the step's log, the robot computer and, on the robot, shutdown and reboot. A `keep_running` step is started again if it crashes (3 times in 5 min at most). No browser runs on the Jetson: for a screen on the robot, a phone or tablet shows the page.
- **The Jetson only sends data.** The browser does the drawing and the arithmetic. The map, the lidar scan and the MCU's 200 Hz samples go to the page as the publisher's own bytes (rosbridge `cbor-raw`; `web/js/cdr.js` decodes them; the page places the scan with the status's pose at the scan's stamp). The mission node reads `/tf` and the ball track as raw bytes too (`tflite.py`, `cdr.py`), takes the MCU's state at 10 Hz (`/mcu/state_10hz` from `mcu_link`), and drains its inputs 20 times a second instead of being woken per message. The camera goes as a small, slow JPEG preview made once (`omni_perception/image_preview.py`; on the robot the camera pipeline's VIC can make it) that `web_video_server` forwards without encoding (the Orin Nano has no hardware video encoder). Measured in the sim (WSL core): the console's processes 12–13% of a core → **5–6%**, the mission node 10% → 2.7%.
- **Connection.** The page reaches ROS through rosbridge (`ws://<robot>:9090`). It is plain JavaScript with Preact, htm and roslib vendored, so there is no build step and no Node. Because the robot serves the page over plain HTTP, its `ws://` connection to rosbridge is allowed in every browser. Foxglove's HTTPS web app can't open `ws://` to the robot, iPad Safari included.
- **Mission node.** `mission_node` switches between IDLE, MANUAL, GOTO, EXPLORE, PATROL, CATCH, THROWER, FOLLOW, PROGRAM and CALIBRATE on top of `mode_manager` and Nav2. It keeps the per-mode settings, per map the saved places, routes, no-go zones and floor edges, the programs, calibration results, throw log, health counters and session logs. It publishes a 10 Hz status and a 1 Hz path; the camera blobs flow only while the Vision tab is open. Every command goes through one service, `/console/command` (verb + JSON). The page draws each mode's settings form, with hints, from the schema the node publishes.
- **`omni.sh robot`** ("Robot (all modes)") runs everything at once, so the modes switch at runtime:
  - **SLAM:** the console runs slam_toolbox itself (its own launch files, as a child process): New map, Save as (only while mapping; slam_toolbox can't serialize a map it localizes on), Load (here, or then Set position).
  - **Nav2:** counts only when its lifecycle manager reports it active. A bring-up that failed because the map frame came late is reset and restarted (~4 s). SLAM that gives no map frame within 40 s (its lifecycle activation reply timed out, seen after restarts under load) is started again.
  - The catch game keeps its own world (`catch`); Home switches to it and back through the supervisor.
- **Catch** has a throw radar (each throw's speed, apex and aim, replayed in a side and top view from the path the robot tracked), players and a leaderboard (`throws.py`), and the **pass game**: with "Throw it to: the other player" the robot learns two throwing spots and throws each ball to the one who didn't throw it (`omni_catch/game.py` `return_to`; the sim's `catch_world` has two throwers, `pass_game:=true`). In the sim the first return goes back to the thrower (it hasn't seen the second throw yet), the next ones land 0.27–0.62 m from the other player 2.5 m away.
- **Follow / Tag** (`people.py`): legs in the lidar scan (clusters 4–30 cm, paired within 55 cm; furniture on the map dropped), followed at a set distance, or chased at kid speed with a tag at 0.5 m, a back-off and a 3 s count. The sim's house has a person walking ahead of the start (`person:=true`, `bridge_core.Walker`). Through the collision monitor when Nav2 runs.
- **Program** (kids, `program.py`): blocks (forward, back, slide, turn, spin, wiggle, wait, go to a place, repeat) run closed on the odometry, each measured from where the last one should have ended (a square closes within 2–4 cm), a time limit per block.
- **Map edit** (`zones.py`): no-go zones (Nav2's keepout filter: Go To refused a goal inside one) and floor edges with their height and speed (a 35 cm slow band: Nav2's speed filter, and the stick capped in every mode). The masks are the map's grid, published by the mission node.
- **Calibrate** (`calibration.py`): odometry on a straight line and on a spin (you enter the tape measure's answer), IMU level (measured here and turned half round: mount offset vs floor slope), the launcher (shots at five rpm in Thrower mode, landings measured: the rpm correction to the table), camera latency (entered). Results are kept with their time; nothing is applied by itself.
- **Health** (`health.py`): 30 min of battery, the robot computer (GPU too on the Jetson, VERIFY path), each motor's temperatures; lifetime km, hours, throws, catches, balls fired; maintenance reminders (intervals est).
- **Logbook** (`session_log.py`, omni-log/1): every session as JSON Lines (pose 2 Hz, status 1 Hz, throws, events), read by the page over HTTP and replayed on the map. The sim writes the same format (`sil_game.py --log`), and `studies/replay_log.py` replays a log's throws in the sim (below, section 5).
- **Faults (sim):** System tab, the sim only: the link, a motor, the battery, the e-stop, the bumper, the IMU, the flywheels' telemetry (`simulation/v11 sil/faults.py` through `/sim/fault`). The console says each fault once with its hint, an e-stop ends the mission, a silent MCU is called out and the robot's position marked stale (the EKF keeps predicting with its last velocity while no odometry comes: found this way).
- **Explore** is frontier exploration (`explore.py`) with Nav2: openings it can't reach are skipped. It mapped 34 → 100 m² of the house in 45 s, with a few skips early on.
- **Patrol** drives a route: once, in a loop or back and forth, with laps and a wait at each point.
- **Speed and driving on the way** (Go To, Patrol, Explore; Oct 27, the speed baselined by Fernando): speed up to 2.0 m/s, the firmware's adult limit and the real cap (Go To 2.0 by default, Patrol 0.8; the kid profile caps it at 0.8); without the console (the `goals` and `nav` steps) Nav2 runs at the Go To defaults. **Driving**, three buttons near the top of each of the three tabs: forward (default: the face turns toward where it's going), agnostic (Nav2's own heading: it drove sideways or backwards most of the time) or spinning (it spins all the way, at the Spin rate setting, 120°/s by default; Explore greys it out and drives forward). `omni_bringup/heading.py` (core) and `heading_node` sit between Nav2's velocity smoother and its collision monitor. They change only the turn rate (the footprint is a circle, so the heading never matters to Nav2), hand the turn back to Nav2 in the last 0.4–0.9 m so it ends at the goal's heading, and re-aim Nav2's velocity at 50 Hz with the yaw predicted for when the firmware holds it. The firmware turns each body-frame command with its own yaw at 1 kHz, so a 20 Hz command held while spinning bends the course: 41 cm off over 4 m at 120°/s in the core's test loop, 1 cm shaped.
- **Thrower** is a bench mode: base parked, flywheel rpm, tilt and Fire, all still behind the firmware's interlocks. Fire only at a launch of 25° or steeper (tilt ≤ +5°: the launch line hits the face below 24°), refused by the console and by the firmware (`FIRE_TILT_MAX`, from `robot_export.py`'s `FIRE_MIN_ELEVATION_DEG`); the game's flattest return is 30°.
- **Base, Vision, RC and Version** show, respectively:
  - Base: wheel states and the speed, turn, slip and tilt charts.
  - Vision: track, detector, and the camera (the small preview through `web_video_server` on :8081, or another image topic asked at 320 px; the sim's sparse frames as blobs).
  - RC: the receiver's 8 channels and MANUAL takeover.
  - Version: software commit, packages, the firmware's build id against this checkout's, and the protocol. A mismatch shows a red banner on every tab.
- **Layout.** Home and a tab per mode (Manual, Go To, Explore, Patrol, Catch, Follow, Program); the subsystems and tools under "Robot" (Base, Thrower, Face, Vision, SLAM, Map edit, Calibrate, Health, Logbook, RC, System, Version, ROS). Each tab has a "How to use" box. A fixed status bar shows:
  - connection and round trip, the mode and the current task;
  - the MCU mode and profile;
  - battery (estimated from `v_bus`), Wi-Fi, faults with hints, MCU SILENT when its state stops;
  - while the software isn't running: "Robot software stopped" or "Starting ..." (from the supervisor);
  - the latest message, with a log drawer;
  - STOP (also the space bar).
- **Manual driving.** It works from a gamepad, the keyboard or on-screen pads, all behind a dead man. The robot stops 0.3 s after the input stops. **Oct 27 (Fernando): the gamepad / Logitech flight stick works on every tab**: the drive loop runs in the App, and a fresh press of its dead man switches any mission (Go To, Explore, Patrol...) to MANUAL and drives (`web/js/drive.js`). Oct 27 fix: the page read only the browser's first controller, which on the sim PC was the 3Dconnexion driver's "KMJ Emulator" (a virtual joystick), so neither the Xbox pad nor the Extreme 3D drove; now every connected controller is read, whichever holds its dead man drives, virtual ones (SpaceMouse, vJoy) are off by default, and the Manual tab lists each with an on/off box (a flight stick's mapping under it). The keyboard and pads stay on the Manual tab. The Explore tab has a **Save map** box (the `slam_save` verb, also while exploring).
- **Debugging** stays in Foxglove.

**Face, voice and ears** (`ros/src/omni_face`, Oct 23; redrawn and given ears Oct 25; proposed). The Jetson stays headless (no desktop, no browser). One small program owns the 5 in 800×480 panel at the front, the speaker and the mic array: `display.py` (pygame / SDL) full screen at boot (`systemd/omni-face.service`, SDL straight on the panel through KMS/DRM; fallback a bare X server with only the face, VERIFY on JetPack). `face_node` (with the console's nodes in every step) turns `/console/status`, the console's events and `/face/say` into what to show and say (`moments.py`) and streams it to the display (TCP, JSON lines both ways); with no node it asks the supervisor and says the software stopped or is starting.
- **Shows** (no engineering data), laid out for the 800×480 at the front: the eyes take the middle (they, their colour and the score's big digits are what carries to the thrower at 6–7 m: text on a 5 in panel can't be read there), a top line (the mode in words, the score, the battery), a text band at the bottom for kids and adults at 1–2 m. Eyes with moods (idle, happy, sad, focused, surprised, sleepy after 2 min, worried, alert) that follow the ball in catch and the person in Follow, worked out for a screen facing forward (`facing_deg` 0); a mouth while it speaks; in catch "Throw it to me!", "Here it comes!"; a fault card (red: e-stop, motors; amber: bumper, battery, sensors) with the console's hint; what it heard; a short note for warnings.
- **Says** (by priority: a fault interrupts anything; game lines, the Say box and voice answers next; chatter dropped when stale): catches, misses ("too far" when the throw missed the spot by > 1.5 m), streaks (3, 5, 10, a new record once), "throw it to me!" three times at most while waiting (after 8 s, then 30 s and 60 s apart; then only the screen asks; Fernando, Oct 26: it talked too much), mission greetings, faults in plain words, low battery, lost / found. Synthesized sound effects and songs. Voice (Oct 26): **Kokoro-82M** (offline neural TTS, Apache 2.0, onnxruntime; voice af_heart raised 12% in pitch: cute, a small speaker) on the PC and the robot (VERIFY its speed on the Orin's CPU; PC 0.2× real time), Piper then espeak-ng as fallbacks, Windows' own voice if none is installed; phrases cached as WAVs. Everything plays through the mic array's output so its echo cancelling hears it.
- **Personas** (Oct 26, Fernando): Patrol is stern (red-orange eyes under angry brows, dark red background, radar sweeps, a lower and slower voice), Explore adventurous (green, wide eyes, an open grin, looking around), Tidy a joyful worker (sunny yellow smiling eyes with blush, sparkles, whistling, a cheer for each thing in the bin). Each has its own greetings, quips and antics; faults keep their own colours and are said in the plain voice.
- **How it's going** (Oct 27, Fernando: phrases after each goal, at checkpoints, when Follow loses you; progress when it's useful; funny or cool for every mode): Go To ("Yes, my master!", "Onward to the kitchen!", halfway, "almost there, I can see the goal!", "mission accomplished" or "I can't get there"), Patrol (checkpoint N of M all clear, laps, the end), Explore (every 25 m² mapped, the way back, the total), Follow ("got you", "wait for me!", "where are you going? Don't leave me!"; Tag's chase and tags), Program (its steps), Tidy (a count per thing in the bin, the total). Each line at most every 20–45 s per kind; the ends at game priority; "be quiet" hushes all (`moments.MISSION_SAY`).
- **The Say box** (console, Manual and Face tabs): a typed line is said at once with an expression (happy, excited, silly, love, wink, thinking, sad, worried, sleepy) that the face shows while it speaks; on the Manual tab Enter jumps into the box from driving and Esc goes back (the gamepad keeps driving). Quick phrases and the last eight lines are buttons.
- **Antics** (the Face tab: off / quiet / normal / lively, default normal = every 90–240 s, a spoken quip at most every 5 min; Tricks buttons play one now): it yawns, naps (Zzz, snores, "huh? I'm awake!"), sings on "la" (Twinkle Twinkle, Frère Jacques, Ode to Joy, its own jingle), hums, whistles, giggles, winks, sneezes, hiccups, babbles in beeps, looks around, sighs or says something silly; naps and yawns once it has been idle for a while. Driven fast (> 1.3 m/s) it says "wheee!"; after a fast spin (1.5 turns at > 2.5 rad/s) it gets dizzy. Only light ones while it drives or waits for a throw; none during a fault, a throw, the Thrower bench mode or a calibration.
- **Voice commands** (`commands.py`, `listen.py`; the Face tab: off / chat only / on, default on). Its name first ("Omni, follow me"), or the name, a chime, then the command within 6 s. A closed grammar (the commands plus the map's places) in an offline recognizer: Vosk with its small English model on the Jetson (VERIFY speed and accuracy with the mic array), Windows' System.Speech in the sim (the PC's microphone; L on the face window switches it off). Stop, follow me / come here, tag, let's play catch, go to the <place>, go home, explore, go to sleep, wake up, be quiet / you can talk; the face answers jokes, songs, dance, battery, name, how are you, time, hello, thanks, good robot. The console's commands go through `/console/command` as if clicked (its checks and the MCU's interlocks apply); "chat only" refuses the ones that drive; stop always works; while it is speaking only "stop" is taken. The Face tab types a command as if heard (`/face/hear`) and lists what it heard.
- **In the sim** the same program opens an 800×480 window with sound and the microphone on Windows by itself (like the MuJoCo view); the Launcher has **Robot face** and **Face demo** (a scripted session with driving and antics, no ROS). On the catch step: streaks, records and misses come out as in the game. M mutes. Checked through the `robot` step: the Say box and a trick reach the face, typed "omni follow me" starts Follow, "omni stop" stops, a command without the name is ignored; Windows' recognizer hears a synthesized "Hey omni, follow me" (the tests).

## 3. Expected performance (simulation)

| Metric | Result | Source |
| --- | --- | --- |
| Catch, thrower 6–7 m, expected cameras, flat floor | **93%** (216 throws, ±1.7); 99–100% at 1.5–6 m | §22, §30 |
| Same through the robot's real code (C++ firmware at 1 kHz + Jetson code + link) | 93.1% in one process; **85% ± 5 (41/48) through the full ROS stack** at ≈ 1.0× real time | §30 |
| The robot as drawn (the baseline since Oct 25: plate 96 mm over the deck, the head's cameras), same throws | **90.7%** in the sim's game, **87.5%** through the real code (216 throws, ±2); 92.6% / 89.8% with the head's mass added; the robot before the head 93.1% / 88.4% | `studies/sil_catch.py both` (`--robot cad`: + the head's mass), section 12.2 |
| The drawn robot with the CAD's masses (8.19 kg, CG 125 mm, 515 g wheels, no rocker, the bumper ring; Oct 25) | **93.1%** in the sim's game, **88.0%** through the real code (216 throws): the extra 1.8 kg and the higher CG cost no catches | `sil_catch.py both --robot drawn` |
| The "drift" of the real code's catch rate (92.1% at the Oct 21 commit, 88.4% on Oct 24) | **None**: today's code against the Oct 21 code on the same throws: 90.7% vs 92.1% on the grid, and at the 0.75 m apex (where the gap was) 81.5% vs 78.2% on 486 fresh throws (today better). A code change reshuffles which marginal throws are caught: 162 throws swung 8 points the other way. Compare versions on ≥ 500 throws at the 0.75 m apex | Oct 25 bisection (worktree, component swaps) |
| Pass game: returns to the other player 2.5 m away (6 seeds) | Median **0.20 m** (was 0.58 m on the drawn robot, 0.45 m before it) since the thrower's spot is learned from the track's converged state instead of its first one | `omni_catch/game.py` `_learn_hand`, Oct 25 |
| Arm robot as drawn (arm kit, section 12.3): the tidy through the robot's own code | **102/120** in the bin (85%; the Oct 24 nose layout 98/120): arena 28 + 22, house 28 + 24 of 30 (toys + clutter); ~50 s of sim per thing | `studies/arm_pick.py --seeds 6` |
| Arm robot: ramped stops from 1–2 m/s, arm stowed / upright / reaching, 0–0.25 kg, both mass models | 48 stops, **none tip**; worst 8.2° tilt (upright, 0.25 kg, 1 m/s toward a wheel) of 25.5° to the edge; a wheel off the floor up to 0.84 s | `studies/arm_study.py K` |
| Home floor vs flat (v7) | 89% vs 92%; a 45°-turned rug 88% (v6: 68%) | §25 |
| Randomized worlds (192) | **86%** (per-world median 89%, 10–90% 72–97%); calibration 0.04° → 97%, 0.3° → 78% | §23.3 |
| Camera pipeline estimate | 86–92% expected, 72% pessimistic (slow stacked pipeline), 85% at 0.3° calibration drift | §21.4 |
| Return throws | Median 3 cm from the thrower at 6.5 m (sim upper bound); 25 cm through ROS | §22, §30 |
| Indoors, 8 ft ceiling | Ceiling-aware return (~35–40°, ~4,500 rpm) reaches 7 m, 0 ceiling hits, 7 cm median | §22.5 |
| Floor crossings (v7) | 8 / 12 mm square rugs, 6–18 mm thresholds at all 24 body directions, no airborne phase, 0 balls lost in ~11,000 crossings | §25 |
| Cost of v7 over v6 | +1 kg, CG +0.8 cm, ~2.7 points on a flat floor; speed margin on a low pack +22% (AK40-10), +57% (DM-2325) | §25 |
| Localization with lidar | ~1.2 cm median, < 3 cm at 95%, 0.5° at up to 2 m/s (v9 matcher); slam_toolbox + EKF 9 cm median | §27, §28 |
| Map / Nav2 | 2.1 cm median wall error; 24 of 24 goals (v10). On the v11 stack 4 of 8 failed until Oct 25 ("Unable to transform goal pose": a late map → odom, MPPI waited 0.1 s); now **8 of 8**. The "~1 s stall every 32 s" was WSL's clock stepping, not slam_toolbox (Oct 26, section 10), also after MPPI's goal-approach fix (a goal 1.2 m away behind a wall, 8.6 m by the path: the robot turned in place until Nav2 gave up; now 32 s) | §28, `omni.sh goals` |
| Nav2 speed and facing (Oct 27): the 8 house goals (`omni.sh goals speed:=1.5 heading:=forward`, the v7 base, with the firmware in the loop, run alongside another stack) | Before: **114 s**, peaks 0.50–0.64 m/s under the 0.8 cap (MPPI's 2.8 s horizon ran off the 3 m local costmap, which it scores as collisions), heading 51–173° off the direction of travel (median per goal). Now, at 1.5 m/s: forward **83 s**, peaks 0.45–1.33 m/s (medians 0.33–0.58: the legs are 2–7 m through doorways), heading 1–5° off travel; agnostic (the old turning; run as "free") 87 s, peaks 0.47–1.10 m/s, heading 52–165° off travel, so facing forward costs nothing; spinning (run as "ballerina") 99 s, 2–5 turns per leg, paths 3–7% longer than forward. At 1.0 m/s, forward: 83 s, peaks 0.43–1.03 m/s. **The baseline** (`omni.sh goals` with no arguments: 2.0 m/s, forward, MPPI's samples and path target sized for 2 m/s): **80 s**, peaks 0.44–1.22 m/s (MPPI asked for ≤ 1.24), heading 2–4° off travel, ending 2–13 cm from the goal. The cap no longer binds in this house: legs of 2–7 m through doorways leave no room to reach 2 m/s (1.0, 1.5 and 2.0 m/s all take 80–83 s). On a long open leg it does: `omni.sh goals route:=sprint` (the hallway end to end, 8.2 m east, 10.4 m west, back) peaks at 1.89 and 1.95 m/s true (MPPI and the robot commanded 1.96 and 2.00), 9.6 s per leg, 0 recoveries, ending 8 and 15 cm from the goal (`studies/results/ros_nav_sprint.json`). The baseline run is `studies/results/ros_nav_goals.json` All 8 of 8, ending 3–17 cm / ≤ 9° from the goal (as before) | `nav_goals.py` (speed, heading off travel, MPPI's and the final commanded speed) |
| Catch in the house (the robot step, Oct 25): the console's Catch, a thrower 6.5 m down the hallway | **16 of 16** caught and thrown back in 120 s; back to Manual, the thrower leaves | `omni_sim` bridge `catch:=true`, `house_catch.py` |
| Soak (Oct 25): the drawn robot patrols all 8 rooms' spots in a loop through the console, the full robot step running | **2 h, 43 laps, 336 points: no crash, freeze, restart or stuck robot**; localization vs truth 7.4 cm median, 13 cm p95, 19 cm max; resident memory +8 MB at most per process (none climbing). It found three things, fixed since (section 7, 13): 4 goals lost to slam_toolbox's map → odom stalls, the console republishing Nav2's filter masks ~800 times, the mission node's "/tf flood" | `ros/tools/soak.py --hours 2` (`omni.sh soak`, the Launcher's "Soak test") |
| Launcher (v2 bench) | rpm 1000/2000/3000/4000/5000 → 2.4/5.1/7.7/10.2/12.6 m/s; transfer ~0.70 of rim speed | `launcher_bench.py` |
| Power | Worst burst 12.6 A; 28 W average (Jetson ~22 W) → ~3.9 h | v2 |
| Operator console (sim, PC) | Status 10 Hz, ~520 kbit/s over the link (raw map and full scan, status), page 144 fps. **The console's processes 5–6% of a WSL core with a page open (mission node 2.7%, rosbridge 2–3%), was 12–13%** (mission node 10%). Closing the page mid-drive: stopped within 0.8 s. Go To reaches a goal 2.5 m away; one inside a no-go zone is refused. A program drives out and back to within 4 cm. Follow tracks the sim's walking person; an edge caps the stick to its 0.4 m/s. A link drop: the MCU stops the robot by itself (2.0 → 0.002 rad/s), the console says so. Thrower fires once; Patrol finishes; the catch world is switched to and back from the page, two throws reach the leaderboard. Explore maps 34 → 100 m² in 45 s | `ros/tools/console_smoke.py --tabs --faults --catch --cpu` (75 checks), `--slam` |
| Ball detector, C++ pixel loops vs Python (PC) | The same blobs on every frame; rendered house: full search 1.8 → 0.28 ms, acquisition among 40 candidates 3.8 → 0.62 ms, tracking window 0.15 → 0.02 ms per camera frame (6–7×); a frame full of clutter 15 → 2.6 ms | `studies/detector_speed.py` |
| A session log replayed in the sim | A sim session's 6 throws replayed from the robot's own tracked states: 6/6 the same outcome. The replayed release sits ~0.35 m behind the true hand and ~5% fast (spin isn't in the replay) | `studies/replay_log.py --demo` |

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
| v7 CAD | The motor can't sit behind the 80 mm wheel (→ in-wheel); the printed wheel is 515 g not 285; the lidar would see only ~184° at the sim's position (fixed to 330° by the six-standoff plate); the sim's camera spot is swept by the flywheel motors. Oct 22 review: static clash and thread checks missed six assembly blockers (motor connector, lidar and e-stop risers, a buried keel screw, unjoined side lobes, yoke bosses floating in a window) and thin skins (0.15 mm over the wheel bearing's seat); the build now checks access, insertion paths, single solids and wall thickness. Launch elevations under ~8° (tilt > +22°) hit the stereo bar | §29, `cad/v7/README.md` |

## 5. Simulation strategy and options

**Strategy.** MuJoCo is the physics master (not Gazebo). Each version copies the previous one and adds one layer, so old results stay reproducible and `SimParams()` defaults mostly stay put. The path: physics and catch game (v2–v6) → home floors (v7–v8) → sensors and localization (v9) → ROS 2 in WSL against the sim (v10) → the robot's own code in the loop: MCU firmware as software-in-the-loop plus the Jetson's ROS code (v11) → hardware in the loop (next). Every change must pass the version's tests. **Key trust metric once hardware exists: the gap between simulated and real catch rate for the same settings.** The robot logs every session in the sim's own format (omni-log/1), and `studies/replay_log.py <log>` throws each logged throw again at the robot's code in the sim and prints the gap. Real data to feed back: motor step responses, end-to-end latency, ball drag/mass/restitution, funnel bounce, roller μ, launcher calibration (50–100 logged shots). Randomize each parameter within its measured range.

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
| **v11 sil** (§30–31) | v10 + the real firmware ticked at 1 ms (`sil_robot.SilRobot`), the Jetson's catch code (`sil_game.py` or ROS nodes), `robot_export.py`, `scene="house"` (furnished house, decoration only), `frames="rendered"` (MuJoCo renders through a camera sensor model, sim time only). Oct 22: fault injection (`faults.py`, `/sim/fault`), session logs (`sil_game.py --log`) and their replay (`studies/replay_log.py`), the pass game (`WorldConfig.throwers=2`), a walking person in the house (`params.person`, `bridge_core.Walker`). Oct 22 (section 10): `timing="real"` (drifting MCU clock, CAN timing), the GPIO sync model, `tag_sim.py`, `studies/tag_slam.py`. Oct 23 (section 11): `payload="arm"` (an SO-101 in place of the catch module, `arm_so101.py`), `extra_masses`, `arm_view.py`, `studies/arm_study.py`. Oct 24 (11.1): the picking arm robot (`robot_export.ARM`, `arm_bus.py`, `arm_objects.py`, `sil_pick.py`, `studies/arm_mount.py`, `arm_pick.py`, `CoreConfig(payload="arm")`). **Default sim for `ros/omni.sh`** | 113 tests (`test_v11.py`), ~7 min |

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

**Not modelled / honest limits:** the CAD's masses (wheel 515 g, robot 8.2 kg, CG 125 mm), no front rocker, Ø434 mm bumper, the 150 mm scan plane and the lidar mask are not in `SimParams()`'s defaults: they are the drawn robot, `robot_export.drawn_params()` (the masses as one point mass that matches the BOM's total and CG; the head's and face's own inertia isn't modelled), `CoreConfig(base="drawn")` in the ROS bridges (the interactive steps' default: `robot:=drawn`), `sil_catch.py --robot drawn`. The CAD's plate height and head cameras are the v11 robot's since Oct 25 (`robot_export.V11`); the head's own mass is not (`sil_catch.py --robot cad` adds it). Rendered images lack global illumination, auto exposure, Bayer artefacts, lens distortion; rendered runs run ~0.03× real time and are not bit-reproducible. Perception noise values, contact parameters and floor μ are guesses until measured.

## 6. Where things live

```
omni-robot/
├── design.md                 ← this file;  design full history.md ← everything (§N)
├── archive/                  ← superseded docs
├── todo.md, Launcher.cmd + launcher.ps1   ← Fernando's launcher = the sim's controller; launcher_full.ps1 = every step and study (its More menu); keep both working when steps change
├── simulation/               ← README.md (venv, version index); v1 … v11 as in section 5
│   ├── v2 detailed physics/  ← catalog.py (part data), params.py (fitted config), models.py, robot_model.py, robot.py, catch_game.py, run.py, launcher_bench.py, studies/
│   ├── v3 compact/ v5 unlimited/ v6 shallow throw/ v7 home floor/ v8 swerve/ v9 sensors/ v10 ros bridge/   ← each has README.md, test_vN.py, studies/ (results/ holds JSON, plots, logs)
│   └── v11 sil/              ← sil_robot.py, catch_world.py, sil_game.py, sil_link.py, robot_export.py, house_scene.py, rendered_camera.py, faults.py, studies/ (replay_log.py, detector_speed.py ...)
├── firmware/                 ← Teensy 4.1 (README, PINS.md): lib/omni_fw (board-free core), src/main.cpp (drivers), sil/ (SIL library: zig c++ on Windows, g++ in WSL)
├── ros/                      ← omni.sh (every ROS step, in WSL), src/omni_bringup omni_perception omni_interfaces omni_mcu omni_catch omni_console omni_face omni_calib omni_slam (→ Jetson; omni_console/web = the operator page, supervisor.py + systemd/ = the page as the launcher; omni_calib/README.md = the calibration procedure), omni_sim (PC only), maps/ (+ <map>.places.json, <map>.tags.json), foxglove/, tools/ (console_smoke.py, cpu_sample.py)
├── database/                 ← shared actuator DB: motors/servos/controllers/motor_tests/sources CSV + actuator_db.py (981 motors, 106 servos, 60 controllers, 424 dyno tests; every value has a source); keep it free of omni-robot imports
└── cad/                      ← one folder per version, each with its own out/ (README.md): v8/ = latest (section 14; out/assembly.step, out/assembly_arm.step), v7/ = the Oct 25 baseline the v11 sim reads (to the fastener level: build.py, render.py, view.py, step_export.py; out/ has STEP, STL, BOM.md, PRINT_LIST.md), v2/ = the v2 CAD
```

**Environment**

- Python: `$py = C:\Users\ferna\.venvs\omni-robot-sim\Scripts\python.exe` (MuJoCo 3.16, build123d 0.13). `python` on PATH is PlatformIO's: never install into it.
- ROS runs in WSL (`wsl -d Ubuntu-24.04`, Jazzy, Python `~/.venvs/omni-ros`); always use `ros/omni.sh <step>`, which mirrors `ros/src` and the sim into `~/omni_ws` (ext4) and builds there. Never build on `/mnt/c`. Several runs at once need separate `ROS_DOMAIN_ID`s.
- Sister project with motor / battery / thermal models: `C:\Dropbox\Personal Projects\Robotics\wheeled-leg-robot\simulation\mujoco\v4_twin_279mm_baseline`.
- Flashing only in attended sessions. `protocol.h` and `omni_mcu/protocol.py` change together. Rerun `robot_export.py` after any params / catalog / model change (a test checks).

**Common commands** (from the folder shown)

| Folder | Command | What it does |
| --- | --- | --- |
| repo root | double-click `Launcher.cmd`, then **Start** | The sim's controller (`launcher.ps1`, Oct 26). Start runs `omni.sh console --autostart robot|arm` at real time with the firmware in the loop and opens the console in Manual (`?drive=1`: a gamepad drives from its Manual tab), the 3D view and the face. Robot (catch / arm), map and kid mode restart the sim (the supervisor's `/api/start`); person, clutter (none / toys / toys + obstacles) and thrower change live (`simulation/v11 sil/sim_world.py`: the bridge's control port, TCP 9873, `live_world` in the robot and arm steps); the 3D view's camera (follow, chase, free, top, the robot's camera: `mujoco_view.py`, UDP 9872) and the face's size / mute / hide (`omni_face/display.py`, UDP 9883) are local. Choices persist in `logs\controller.json`. More: CAD, firmware, setup, tests, and `launcher_full.ps1` (every step, the studies, the scored runs) |
| `ros` (WSL) | `bash omni.sh console --autostart robot [person:=true ...]` / `stop-console`; `bash omni.sh robot` (all console modes, Catch too; `robot:=drawn|v7`, `catch:=true`) / `map` / `save-map` / `localize` / `nav` / `sim` / `catch` (`dist:=6.5`, `pass_game:=true`) / `catch-score tosses:=24` / `soak --hours 2` (with `robot` running; the Launcher's "Soak test") / `test` / `stop` | The supervisor (page + launcher); the ROS 2 stack against the sim. The interactive steps start the operator console (`console:=false` turns it off; `web:=false` when the supervisor serves the page); `mcu_port:=5771` with another `ROS_DOMAIN_ID` runs a second stack alongside |
| repo root (Windows) | browser at `http://localhost:8080`; `& $py ros\tools\console_smoke.py --tabs --faults --catch --cpu --shots <dir>` (`--slam`); `& $py -m pytest -q ros/src/omni_console/test` | The operator console; its end-to-end check in Edge (Playwright) against a running `robot` step started by the supervisor with `person:=true` (~10 min; `--slam`: also New map + Explore); its core's tests (the supervisor's and rclpy's checks run in WSL) |
| `simulation/v11 sil` | `& $py sil_game.py --view --returns`; `--frames rendered --scene house` | Catch game on the real code in one process; on rendered images in the furnished house (`house_scene.py fetch` once) |
| `simulation/v11 sil/studies` | `sil_catch.py both` (`--robot cad`: the drawn robot) / `report`; `rendered_frames.py --png`; `rendered_catch.py run --frames rendered --seeds 3 --workers 12`; `replay_log.py <log.jsonl>` (`--demo`); `detector_speed.py --rendered` | SIL vs the sim's own game on 216 throws; detector on renders; the 216-throw grid on rendered vs sparse frames (~35 min); a session log's throws replayed in the sim (the sim-vs-real gap); the C++ detector against the Python one |
| `simulation/v11 sil` | `& $py arm_view.py --payload 0.25` (`--mount front`, `--png`); `& $py arm_so101.py fetch` once; `& $py studies\arm_study.py` (`R S D T F B`, `report`) | The robot with an SO-101 instead of the catch module (keys 1-6 poses, space drives 2 m/s and stops); the arm's meshes; reach, static tipping, stops, swings, floor edges, the firmware check, fixes (~12 min, section 11) |
| `simulation/v11 sil/studies` | `& $py layout_trade.py` (`L`, `png`, `report`) | Section 12's layout trade: lidar, camera, display and arm checks on every layout, C2 / A6 on the CAD's head meshes (~1 min); `L` the house tour with each layout's lidar shadows (~5 min); `png` the renders |
| `simulation/v11 sil` | `& $py sil_pick.py --view --world house --n 5`; `& $py studies\arm_pick.py` (`--worlds house`); `& $py studies\arm_mount.py`; `& $py -m pytest -q ros/src/omni_arm/test` (repo root) | The arm robot tidying up on its own code (section 11.1); the tidy scored (~25 min); where the arm goes; omni_arm's cores. ROS: `bash omni.sh arm` (WSL; the Launcher's "Arm robot: tidy up"), then the console's Tidy tab |
| `firmware` | `pio run`; `& $py sil\build_sil.py --pins` | Teensy build (checks the pin map); the SIL library and PINS.md |
| `simulation/v7 home floor` | `& $py house.py` | Drive the v7 base through a house (keyboard, gamepad, flight stick) |
| `simulation/v7 home floor/studies` | `home_floor.py A` / `B` / `C` / `report` | Crossings, catch games, costs |
| `simulation/v9 sensors` | `house.py --base v7 --sensors --camera top`; `studies/localization.py` | Fused estimate live; localization study |
| `simulation/v6 shallow throw` | `watch_floor.py` ; `studies/actuator_choice.py` | Watch thresholds / rugs; cost-ranked actuator study |
| `simulation/v2 detailed physics` | `run.py` (`--headless -n 30`, `--drive`) ; `-m pytest -q` | Catch demo; regression checks |
| `database` | `actuator_db.py validate`; `-m pytest -q` | After any database edit |
| `cad/v8` (latest; `cad/v7` the same) | `build.py` (`--quick`, `--head-only`), then `build.py --render-only; render.py`; `view.py` (`--arm`); `step_export.py --kit arm`; `walls.py <part>` | STEP/STL, BOM, clash checks over tilt and suspension, thread, screw access and clamping, insertion paths, single solids, wall thickness (full build); interactive view |

After CAD changes run `build.py` (must report 0 clashes, 0 short of thread, 0 access problems, every print one solid). Run `python -m pytest -q` in the sim folder you touched.

## 7. Next steps

1. **Re-simulate with the CAD's findings:** done Oct 25 for the masses (8.19 kg, CG 125 mm, 515 g wheels), no rocker, the bumper ring, the A2M12 at 150 mm and its standoff mask (`drawn_params`, `sensors.BLOCKED_CAD`; the catch 93.1% / 88.0%, the ROS steps run it), and the launch elevation (≥ 25°, the face). Left: recalibrate `feed_clear_travel` for the moved feed-exit beam and the catch beam (22 mm up the chute from the resting ball, 10 above its centre); the v9 localization tour with the mask (`studies/localization.py`).
2. **Bench, in this order:** stereo calibration stability on the robot (the dominant unknown); two DM-2325s + a DM-J3507 (torque constant/peak, undervoltage cutoff on a low 7S pack, winding temperature at ~0.4 N·m RMS, backlash, comm-loss behaviour, CAN ids and feedback ranges; record in `meas_*` columns) → decide the drive; throw spin and camera latency (LED + photodiode); foam-ball drop tests; one printed 127 mm wheel + one pod (roller grip and rolling resistance on wood/tile/carpet, pod rate and damping, hand-push an 8 / 12 mm square rug edge); a feed-wheel pair + printed chute and J with the exit beam.
3. **Hardware in the loop:** write the stub drivers (BNO085 SH-2 over SPI, bidirectional DShot, CRSF); with a Teensy 4.1 and one DM-2325 check the CAN against `dm_motor.h`, the USB link latency and the 1 kHz loop CPU time; schedule the coast / brake / dump on an arrival time refined after the freeze (SIL bounced 5 balls to the sim's 1).
4. **Jetson:** profile slam_toolbox + Nav2 and the catch code on its CPU (the detector's pixel loops are C++ now, `omni_catch/fast_detector.py`, 6–7× faster on the PC; the tracker is Python; a GPU detector only if that's not enough); resolve the JetPack / Jazzy question; confirm OV9782 driver support (1280×800 at 120 fps, frame-sync pin) for the carrier, and have its pipeline make the console's camera preview with the VIC.
5. **Software gaps:** port the terrain assist (edge map) into the firmware or keep it on the Jetson as speed caps (and decide where the edge map comes from: app markup, IMU-bump learning, stereo depth); feed the lidar fixes into the catch-time estimator; model calibration error as a slowly varying bias in the tracking filter; feed rendered images into ROS in real time from a Windows render server (2–5 ms a frame; `catch_perception source:=image` is ready); port v9's matcher to the ROS side.
6. **Design gaps:** a rear battery drawer; a counterbalance spring on the tilt pivot; a bumper skirt and wheel-top guards (done in CAD); a multi-ball path (tube insert now); choose the shock spring (~12 N/mm at the shock); rework the deck for the 80 mm wheel (motor 21 mm inward).
7. Print and bench: one wheel (3 hubs, 10 rollers, 10 pins) and one pod, with a DM-2325, before printing the rest (check the connector and its plug in the cup's 12 mm channel). Buy 5 × 6808-2RS (9 with v8's forks; the 623s and the 608 are kit sizes). VERIFY the display, mic board, speaker, on/off switch and payload connector sizes before printing their parts. Round 2 (section 14.1) adds 8 × 608 (kit), 4 × 8 mm shafts and 4 more shocks. SolidWorks: open `cad/v8/out/assembly.step` (catch) or `assembly_arm.step` (arm); v7's is the baseline.
8. **Face and voice:** done in the sim (`omni_face`, section 2; Oct 25): the face at 800×480 facing forward, the Say box, antics, voice commands. Left, on the Jetson: SDL on KMS/DRM vs a bare X server, the panel's 800×480 mode over HDMI, backlight control, Piper's voice and speed on the Orin; Vosk on the Orin with the mic array (its ALSA name, echo cancelling while it talks, kids' voices, a noisy room, false wakes on "robot": drop that wake word if so); the direction of the voice from the mic array (DOA) so the eyes look at the speaker and "come here" turns to them.
9. **Time, calibration, tags (section 10):** bench the timing constants with a scope (SYNC_OUT vs the Jetson's GPIO stamp, CAN reply turnaround, FSIN → exposure, strobe); an HTE-capable GPIO on the carrier; sllidar's index order; the camera node on `FrameStamper`; calibration sessions as rosbags + scripts over `omni_calib`; slam_toolbox and `/initialpose`; print and hang the tags (36h11, 16 cm, measure the print). (Done Oct 25: the Nav2 goals step's 4 failures of 8, see section 3.)
10. Fill the database gaps (`database/gaps.csv`), prices first. Spec sweep over perception latency, noise and frame rate is still open.
11. **Arm robot (sections 11.1, 12.3):**
   - Done in the sim and ROS (Oct 24): the "arm out" base caps, the wrist camera, the tidy. Oct 25: the arm kit drawn and baselined (riser, adapter, bin, uprights, shelf), the arm robot moved to it in the sim and in `robot_arm.json` (poses, keep-outs, the reach's sweet spot).
   - Left: VERIFY the SO-101 base's hole pattern on TheRobotStudio's STEP before printing the adapter.
   - What the e-stop does to the arm (the SO-101 has no brakes).
   - Bench one STS3215 at the shoulder's load.
   - Calibrate the real servos with LeRobot's tool.
   - Long approaches through Nav2.
   - A learned floor-thing detector.
   - The LeRobot side for learned grasps, through the trajectory action or a LeKiwi-compatible host.
12. **Layout for both payloads (section 12, baseline since Oct 25):**
   - Done in CAD: the head (camera bar, holders, cheeks, index), the 5 in face, the Ø8 front rods (12.2); the arm kit and the swap (12.3); the base's own boards off the catch kit's yokes (the Jetson's 19 V buck, the RC receiver).
   - Done Oct 25: the Thrower bench mode fires only at ≥ 25° (console and firmware); the face redrawn for 800×480.
   - Left: VERIFY the 5 in panel's outline and connectors and the OV9782 lens's optical centre before printing; at the arm index the lenses' barrels come to 5.5 mm over the scan plane (bench the A2M12's plane tilt).
13. **Operator console, what's left:**
   - **One sim world for everything:** done Oct 25. In `robot` (catch:=true) the house has the cameras' frames and a thrower who steps in front of the robot when Catch starts (`house_catch.py`; refused with the reason when the room is under 3.1 m); the robot's catch code runs in the step and takes the field from `/catch/setup`. The `catch` step stays (the open field, the scored grid).
   - **Soak** (Oct 25, `soak.py`): run it after stack changes and before a long real session; longer than 2 h still to do (no longer for the slam "stall": that was WSL's clock, section 10).
   - **Filter masks** (Oct 25): published when the map outgrows them or the zones change (`zones.covers`), not on every slam_toolbox rebuild.
   - **Explore:** tune it on the real house (skipped openings, goal clearance 0.35 m against Nav2's 0.40 m inflation).
   - **Optionally:** a firmware software e-stop request; a PIN on the System tab for kid-proofing (`OMNI_CONSOLE_PIN`, there, untested on an iPad).
   - **On the Jetson:** install the supervisor's systemd unit, `omni.local` and the sudoers line for shutdown (`ros/src/omni_console/systemd/README.md`); read the mission node's own load (`LoadMeter`, its "busy:" log lines and the Version data) and `ros/tools/cpu_sample.py`; check the GPU load path; check that the Teensy's INFO arrives (the CRSF driver is still a stub, so RC shows "no link"). The "/tf flood" (Oct 25): never a flood. The capped drains held 1-3 distinct messages taken over and over while a normal /tf subscriber saw nothing unusual: rmw hands a raw take's message back again. `Feed.drain` now stops at the first exact repeat (0 floods since, 42 warnings in the 2 h soak before).
   - **Calibration results** are numbers to apply by hand: feed them into `robot_export.py` / the launcher table once a few repeats agree.
   - **The session log's replay:** spin isn't logged (unmeasured); the replayed release sits ~0.35 m behind the true hand at ~5% more speed. Check against the first real sessions.

## 8. Open items (VERIFY)

- **Electronics (section 15):** the DM-2325's undervoltage cutoff against the 7S pack (7S vs 8S); the Orin dev kit's DC input range (19 V buck headroom); DP→HDMI for the display; a USB hub for the arm kit; `BUS_EN` into the e-stop chain; the STS3250 bus level; board space for a ~90 × 55 carrier.
- **Decisions:** the drive (DM-2325 vs fallbacks); Jetson tier and carrier; feed motor (MF4005 / GM3506 vs Pololu 25D); chute pocket vs round-1 roller pocket; tube vs indexer for multi-ball; the proposed limits (2.5 m/s² direction-aware, 74 mm / 22 g ball, honest track start, 35° tilt cap).
- **Parts:** DM-2325 torque labels (1.35 / 5 N·m assumed), supply range and comm-loss timeout; DM-J3507 and AK40-10 voltage rating on 7S; STS3250 continuous torque; F90 pole pairs (7 assumed); RoboClaw baud/counts; 8S ESC with bidirectional DShot under AM32; A2M12 scan-plane tilt (±1.5°) and range noise at 1/3/6/10 m, `rplidar_ros` direction and stamp; BNO085 drift (CEVA datasheet); shaft loads on the DM-2325 and the tilt servo.
- **Physics:** roller μ on the real floor (0.95 assumed) and TPU-roller grip (0.9 assumed); rolling resistance (Crr 0.02 / 0.06 on floor / carpet assumed); foam ball diameter, mass, Cd, restitution (0.55 / 0.38 floor and 0.10 / 0.04 pad assumed); squeeze force on a flywheel; feed-wheel preload and TPU-on-foam grip; net rolling resistance; ball spin; room ceiling height; real rug and threshold heights.
- **Camera:** end-to-end latency (50 ms assumed; 22–26 ms raw V4L2, 40–50 ms Argus estimated); stereo calibration (0.02° assumed); OV9782 shutter/ISP behaviour on the Orin; FSIN level (1.8 V DOVDD?) and minimum pulse, trigger → exposure delay, STROBE output on the module's connector.
- **Timing (section 10):** which Orin 40-pin GPIO has HTE (pin 7 / PAC.06 is an AON GPIO: check) and JetPack's gpiod clock options; FlexCAN's timestamp capture point and clock; the DM-2325's reply turnaround; the BNO085's SH-2 timestamps on the bench (CEVA 1000-3625); the Teensy crystal's ppm; the USB path's asymmetry (the echo-minus-pulse number); sllidar_ros2's array order and stamp.

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
| Oct 21 | **Operator console (Fernando):** a web console served by the robot over rosbridge (`omni_console`), with a tab per mode and subsystem and a fixed status bar. Foxglove is kept for debugging. Driving by gamepad, keyboard or on-screen pads. Phased: Manual, Go To, Catch and ROS now, the rest in phase 2 | Works on the PC and the iPad with nothing installed, and always matches the robot's software. Foxglove is a viewer, not an operator console: no fixed status bar, goals hidden in a menu, sign-in, and its HTTPS app can't open `ws://` to the robot. A Python GUI wouldn't be faster: the link is the limit, so high-rate data is summarized on the robot and the camera goes as video, not through rosbridge |
| Oct 22 | **Operator console, all tabs:** Explore (frontiers), Patrol (routes per map), SLAM run by the console in the new `robot` step (new / save / load / set position), Thrower bench mode (THROWER in `ModeMux`), Base charts, Vision (MJPEG via web_video_server; sim blobs), RC and Version. The link gets an INFO message (protocol 2: build id, RC channels), and `mode_manager` one fire counter | Fernando: do the other tabs. A source switch must never fire a ball. The console must say when the MCU runs other firmware than the Jetson expects. slam_toolbox can't save a map it localizes on, and its map saver gives up on a slow `/map`, so the console writes the image itself |
| Oct 22 | **Operator console round 3 (Fernando): the Jetson only sends data, the page is the launcher, new tabs.** Raw map / scan / MCU samples decoded in the browser (cbor-raw), raw `/tf` and drained inputs in the mission node, a 10 Hz MCU state; a supervisor serves the page and starts / stops the steps (systemd at boot on the robot); Home and System; Follow / Tag, Program, Map edit (Nav2 keepout and speed filters), Calibrate, Health (protocol 3: motor temperatures in INFO), Logbook (omni-log/1, also written by the sim and replayed in it), the throw radar, leaderboard and pass game; the detector's pixel loops in C++; fault injection in the sim, checked end to end | Console CPU 12–13% → 5–6% of a core; no browser on the Jetson (a face on its display stays a light native renderer); the robot must boot and come up unattended (SLAM and Nav2 bring-ups retry themselves); the sim-vs-real gap needs the robot's sessions in the sim's format |
| Oct 22 | **Time sync, frames, calibration and AprilTag landmarks (Fernando's spec; implemented, proposed: section 10).** Protocol 4: every MCU time taken where it happened (CMD arrival, CAN reply interrupt, BNO085 SH-2 sample times); a 1 Hz SYNC_OUT pulse stamped on a Jetson GPIO maps the clocks by a fitted line (offset + drift); the MCU triggers the stereo pair and reports each trigger and the left strobe. One generated frame description (REP-105 / 103) and a checker. `omni_calib` (solvers, checks, bounded online refinement) and `omni_slam` (tag landmarks in a factor graph next to slam_toolbox, localization health → speed scale and relocalization) | The USB echo can't see its own path's asymmetry (≈0.25–0.5 ms in the sim) and arrival stamps carry the 1 ms tick; tags map, bridge lidar gaps and relocalize but don't beat a good lidar localizer; a planar tag model needs the IMU's roll / pitch (the sprung base); one tag's mirror pose can be metres off |
| Oct 22 | **CAD review (Fernando): make every part assemblable and structurally sound; add a 7 in display, USB mics and a speaker.** Fixed the six assembly blockers and the thin sections the new wall scan found (wheel-bearing boss skin, yoke shoulder, collar servo boss, flywheel-mount pockets, battery tray, camera arms); fillets, lead-ins and chamfers; solid yokes; pod mounts tie the side lobes to the centre; display and audio at the plate's rear edge facing the rear | The build checked clashes and thread only, so parts that couldn't be put together or would print thin passed; the front of the robot has no room for a 7 in panel without blocking the cameras, launch line, catch or lidar |
| Oct 23 | **Face and voice (Fernando: status, eyes, the catch game's prompts and score, fault reasons; sounds and speech; "execute", testable in the sim):** headless Jetson, one pygame program on the panel and the speaker, fed by `face_node` from the console's status; Piper / espeak-ng on the robot, Windows' voice in the sim | No desktop or browser on the Jetson (memory, GPU, boot time); one process for screen and sound needs no sound server; the same program runs on Windows against the sim |
| Oct 23 | **Arm module in the sim (Fernando: the arm, likely an SO-101, replaces the catch module; sim it on the payload plate, flag-based, no CAD yet):** `payload="arm"` with the official SO-101 model; reach and tipping study (section 11, proposed) | It doesn't tip under today's firmware limits, the arm's servos limit the payload long before the base does; the arm out needs a gentler base profile and unloads wheels; floor reach is only 2-23 cm past the bumper, no dining tables |
| Oct 24 | **Arm picking (Fernando: drive the arm from ROS, toward picking small things off the floor; the Jetson drives the servos, a wrist camera, a bin on the robot, the arm lower and forward, the cameras moved down):** own stack `omni_arm`, not MoveIt; the arm robot layout (nose bracket, stereo pods 35° down, lidar at the rear, bin with the display on its back); the tidy task, PICK mode with the arm-out caps, the console's Tidy tab (section 11.1, proposed) | A 5-joint arm in a small fixed space needs exact top-down IK and a few checked waypoints; the hard parts are perception, calibration and grasp checks. The plate mount reaches 2-17 cm past the bumper, the nose 3-25 cm |
| Oct 25 | **Baseline the drawn design; draw the arm kit too (Fernando).** v11's robot is the drawn one (plate 96 mm, head cameras). The arm kit (section 12.3): the SO-101 on a 50 mm riser at x 140 mm with an adapter over the camera bar, the bin on the plate's rear, the face and the audio box over it on uprights; the Jetson's 19 V buck and the RC receiver moved off the catch kit's yokes to the base. omni_arm takes the layout's poses, keep-outs and sweet spot from `robot_arm.json`; the catch code learns the thrower's spot from the converged track | The study's A5 (the arm on the plate at x 100) put the SO-101's base and shoulder bracket where the camera bar is: its capsules missed them; the arm's own meshes against the CAD (`mesh_sdf.py`) found it. On the riser it reaches 0.26–0.42 m and drops 10° off vertical; a face over the bin blocks straight swings back, so a pre-drop pose folds the arm first. The tidy 102/120 (nose layout 98), no tip in 48 stops. The pass game's returns had got worse on the drawn robot (median 0.58 m): the first track state's noisy velocity, integrated back 2 m to the hand |
| Oct 25 | **Re-simulated with the CAD's masses; the catch "drift" was noise; Nav2's goals fixed; Catch in the house's one world; a soak test (Fernando: next steps 1, 2, 3, 5, 6 and a long robustness run)** | The drawn robot (8.19 kg, CG 125 mm) catches as well (93.1% / 88.0%). Bisecting Oct 21 against today on the same throws found two "causes" that reversed on 486 fresh throws: code changes reshuffle marginal throws, so versions need ≥ 500 throws to compare. MPPI's 0.1 s TF wait aborted goals when slam_toolbox's map → odom stalled (0.5–0.9 s on the loaded stack). One world: the console's Catch no longer needs a step switch; the thrower steps up where the room allows. A 2 h soak patrol: no crash or freeze; it and the smoke check found the console's filter masks republished on every map rebuild, MPPI dropping the path for a goal just behind a wall (its critics' 1.4 m straight-line switch), and the "/tf flood" (rmw handing a raw take back again, not a flood): fixed. slam_toolbox's ~1 s map → odom stall every 32 s stays open (tolerated) |
| Oct 26 | **The slam_toolbox "stall" is closed: WSL's clock, not slam_toolbox (section 10).** One clock discipline in WSL: systemd-timesyncd off (`install_wsl.sh`); Nav2's 1.0 s waits kept as margin | Windows 0.64 s off NTP: WSL's host sync and timesyncd fought, the clock ran 3% slow and stepped ~1 s every 32 s. Stock slam_toolbox, our localize step and the robot step on patrol showed 0 stalls on a monotonic probe |
| Oct 27 | **The face says how each mission is going (Fernando)** | Lines tied to real progress (goal reached, checkpoint, person lost) are useful, unlike the idle prompts it had too many of: each kind at most every 20–45 s, endings at game priority, idle quips still at most every 5 min |
| Oct 26 | **The face talks less, speaks with a neural voice and has a persona per mission (Fernando)** | "Ready when you are!" every 25 s was annoying: three asks per wait, quips at most every 5 min, antics half as often. Kokoro (free, offline) over Piper: clearly more natural, and a 12% pitch raise makes it small and cute without slowing it; 54 voices to pick from (`OMNI_KOKORO_VOICE`). Personas carry at 6–7 m through colour and eye shape, not text |
| Oct 26 | **The Launcher becomes the sim's controller (Fernando):** Start always runs the console (browser, Manual, gamepad), the 3D view and the face at real time; robot / map restart the sim, person / clutter / thrower change live (`sim_world.py`), the camera and the face are set from the Launcher; studies and scored runs move to `launcher_full.ps1` (More), CAD to the More menu | Every mode is a console tab, so the Launcher only shapes the world. Live changes keep the robot's place and map (a restart costs ~1 min); clutter = the arm's toys plus larger obstacles in the rooms (boxes, bag, shoes, basket, stool, play ball) |
| Oct 27 | **Nav2 faster, and a Facing setting (Fernando asked: go faster than 0.8 m/s; face forward while driving, or a ballerina spin): proposed.** MPPI up to 1.5 m/s with a 9 m local costmap (its horizon at top speed), `vx_std` 0.4 and the path-follow target 12 points ahead; Go To 1.0 m/s by default. A heading node after the velocity smoother: forward (default), free, ballerina | The 3 m costmap, not the 0.8 cap, held the robot under ~0.6 m/s: MPPI scores trajectory points off the costmap as collisions. Then MPPI itself commanded ≤ 0.84 m/s until its samples and target reached further. The shaper sits after the smoother because the smoother's body-frame acceleration limits fight a velocity that rotates in the body frame while spinning. Open: tune on the real house; spinning with the real A2M12 (the sim's de-skewed scans localized fine) |
| Oct 27 | **Fernando baselined the speed change and asked for the facing choice in the GUI.** The setting is now "Driving": forward, agnostic, spinning (were forward, free, ballerina), three buttons near the top of the Go To, Patrol and Explore tabs. Without the console, Nav2 runs at the Go To defaults: the velocity smoother at 1.0 m/s (was 0.8), the heading node forward (was free) | The scored `goals` step now measures what an operator gets from Go To by default |
| Oct 27 | **Nav2 up to the real cap, 2.0 m/s (Fernando: "is 1.5 the fastest? find the real cap and set it", sprints between goals, slow near them); Fernando baselined it.** The baseline goals result is `studies/results/ros_nav_goals.json` (80 s; Oct 26's, 115 s, kept as `ros_nav_goals_oct26.json`; `studies/ros_report.py goals` shows both). MPPI and the velocity smoother at 2.0 (vx_min -2.0), Go To 2.0 by default (Patrol 0.8, up to 2.0); the local costmap 12 m (2.8 s × 2 m/s each way), prune 6 m, the lidar's obstacles to 6 m; `vx_std` / `vy_std` 0.6, the path-follow target 30 points ahead | The chain: drive motors 3.5 m/s diagonally / 5.0 straight (0.95 of no-load, less under load), the firmware's adult limit 2.0 (Oct 17; stops and tipping simulated to it), kid 0.8, the lidar 12 m. Going past 2.0 is a firmware change that needs the stop and tip studies above 2 m/s. MPPI slows for the goal itself (GoalCritic within 0.5 m, the heading hand-back in the last 0.4–0.9 m); the goals still end 2–13 cm off. The hallway sprint (`route:=sprint`) reaches 1.95 m/s, so the cap is reached when a leg is long enough |
| Oct 27 | **Electronics block diagram; do we need a PCB (Fernando's todo): proposed, section 15.** Power tree (always-on logic for the Jetson and the MCU, an e-stop-switched motor bus with soft start) and the signal map. One custom PCB, the Teensy's carrier (2 CAN transceivers, level shifters, drivers, protected inputs, keyed connectors); the power side off the shelf until after the bench | The pin map is fixed and checked, so a carrier is low risk; perfboard would be ~150 joints in a shaking robot. Sim currents 8–19 A in hard bursts to 2 m/s (fuse 40 A). Found: the DM-2325's listed 24 V minimum sits above the 7S pack's last ~10–15% and the firmware's 22.4 V low-battery fault; the arm kit runs out of the dev kit's USB ports; no MCU output into the e-stop chain (`BUS_EN` proposed); the CAD's MCU board space (40 × 30) is too small for a carrier (~90 × 55) |
| Oct 27 | **CAD v8 round 2 (Fernando's review of the arm robot): proposed, section 14.1.** The fork pivots on an 8 mm shaft held by an ear on each side of the wheel (a 608 in each plate), a shock on each side, thicker plates and bar; the face, speaker and mics on a two-post mast at the rear for both robots; one electronics bay on the deck's centreline (MCU carrier 55 × 90 under the Jetson) and one payload connector panel; a round on/off switch, no e-stop; rigid bumper from 48 mm, skirt from 34 | Both-sides support was Fernando's must; the only spot for the face that neither the arm (front) nor the funnel (plate's rear) sweeps is above and behind the funnel. The diameter could not shrink with a both-sides pivot (Ø462, v8 458): the ear and boss at the slot's corner set it, and moving the wheels in makes the pods sharing a lobe collide; a one-sided pivot at L 140 would give ~Ø430 (Fernando's call) |
| Oct 25 | **The face at 800×480 at the front, the Say box, antics, voice commands; the Thrower's 25° floor (Fernando); offloading weighed (section 13, proposed)** | The panel moved to the front (12.2): the eyes carry to the thrower, text only to 1–2 m. Typed lines with an expression are said at the game's priority. Antics only when nothing else is going on and never during a fault or a throw. Voice commands use a closed grammar behind the robot's name in an offline recognizer, through the console's own verbs (its checks, the MCU's interlocks), "chat only" for no voice driving. The launch line hits the face below 24°: the console and the firmware refuse flatter shots. Offloading pays for the tidy's perception and for chat, never for the catch or anything that must work without Wi-Fi |
| Oct 26 | **CAD v8 (Fernando's review, proposed: section 14):** a fork round each wheel (second 61808 outboard on hub A, outer plate, 6061 bar under the deck; deck radius 205 → 217), hex-drive shallow pan heads (M3 6.5 × 2.5), magenta fasteners, labels on the boards, cheeks with a jaw under the plate and a 14 mm web, STEP of both robots; the v2 CAD into `cad/v2` | A wheel hung off one printed arm loads one plate and four printed threads; the fork closes a loop round the wheel. The outer plate needs the guard's panel 12 mm further out. The module's sweep leaves no room over the cheek's pad, so the cheek grows down (a jaw clamping the plate's edge) and out |
| Oct 24 | **The head drawn in CAD (Fernando: a detailed CAD of the sim's layout; cameras, lidar shadows and the display in front first; everything else can move):** `cad/v7/head.py` (20 × 20 tube bar, axis x 155 z 195, holders with journals in two cheeks, one index screw), the 5 in face at the front (`display.py`), the front standoffs as Ø8 rods; the audio box stays at the rear; section 12.2 | Lidar 31 → 22° blocked; the cameras see 95% / 92% of the last 1.5 s of 6.5 m throws, nothing of the robot; every viewer sees the face; catch unchanged on the drawn robot (92.6% vs 93.1%, SIL 89.8% vs 88.4%). The bar sits as low as the arm needs (its top under 210 mm, else the SO-101 behind it reaches no floor), so its axis can't pass through the lenses |
| Oct 24 | **One layout for both payloads (Fernando: rearrange for both missions; swaps are rare; one stiff camera bar; a display at the front; more arm reach if possible): proposed, section 12.** The lidar stays at the front of its layer under the plate and nothing goes into that layer. One stiff camera bar on the plate's front edge serves both robots; it turns about its own axis between indexed stops. A 5 in display between the cameras on the catch robot. On the arm robot, the arm sits at the plate's front edge behind the bar, the bin at the rear and the display on a post over the bin | `studies/layout_trade.py`. The arm robot's rear lidar has blind lanes straight ahead (a nose arm in the layer). A display in the head blocks every floor grasp. A 7 in panel hits the 30° launch line. A high camera mast puts the stowed arm in the cameras' view and doubles the range to the near floor. The plate-edge arm loses 5 cm of reach (0.47 → 0.42 m) and keeps a bin drop |

## 10. Time, frames, calibration and landmarks (Oct 22, proposed)

Fernando's spec (Oct 22): timestamps at acquisition, hardware capture where possible, one ROS time domain, sub-ms sync measured not assumed; REP-105 frames; a calibration workflow with validation and bounded online refinement; AprilTags as unknown landmarks in a factor graph next to the lidar; localization health that slows the robot and relocalizes. Code comments cite it as "design.md 10". All of it runs in the sim; every hardware number is (est) or VERIFY.

**Time: where each stamp comes from (protocol 4).**

| Sensor (part) | Stamped where | How | Error (sim / est) |
| --- | --- | --- | --- |
| MCU ↔ Jetson clocks | both ends of a wire | Teensy pin 25 **SYNC_OUT** rises on every whole second of MCU time (a PIT one-shot; the interrupt reads when it really fired); a Jetson 40-pin GPIO stamps the edge (HTE on an AON pin, VERIFY which; else the GPIO interrupt). `omni_mcu/clock_sync.py` fits `t_jetson = a·t_teensy + b` over 120 s, drops late stamps; the CMD/STATE echo is the fallback and pairs pulses with edges | HTE ≤ 5 µs; interrupt 9 µs (its fixed ~8 µs is a bench constant: `edge_latency_us`); **echo alone 0.25–0.5 ms biased** by the USB path's asymmetry, which it cannot see. Drift (20 ppm crystal) recovered to ±1 ppm. In the ROS stack: `/mcu/clock` pulse mode, 3.6 µs RMS |
| Drive encoders (DM-2325 ×4, CAN1) | the reply's CAN RX | FlexCAN's 16-bit bit-time stamp (captured in hardware) + micros64 → reply end; sample = end − `DM_SAMPLE_TO_RX_US` (140 µs est, scope it). Odometry speeds use the samples' own intervals | At 1 kHz the 4 commands + 4 replies keep the bus **~85% busy**; the 4th reply lands ~0.8 ms after the tick (the old per-tick stamp was off by that). A reply waiting for the bus is off by ≤ one frame (~90 µs) |
| IMU (BNO085, SPI, all pins wired: INT, RST, WAKE/PS0, PS1, BOOTN) | its INT edge | the INT interrupt stamps H_INTN; each SH-2 report's time = INT − base delta + rebase + its delay (100 µs steps; `sh2.cpp`, as CEVA's sh2.c); a FIFO batch keeps every sample's own time (IMU msg = sample time) | 100 µs resolution; the hub's own latency is inside its delay (VERIFY) |
| Cameras (OV9782 pair) | the MCU's trigger | pin 24 **CAM_TRIG** pulses both FSIN at 120 Hz (external-trigger mode: the pair exposes together); each trigger and the left camera's **STROBE** (pin 26, exposure start + length) are TIME events. `omni_perception/camera_time.py` pairs V4L2 frames (sequence + SOF stamp) with triggers and stamps **mid-exposure** = trigger + delay + exposure/2; it re-locks after a lost trigger or frame | trigger-exact; the SOF's jitter (20 µs modelled) stays out. Stereo skew = each sensor's trigger response (bench, strobes). Semantics: ROS stamp = middle of the exposure |
| LiDAR (A2M12) | the Jetson (no clock in the lidar) | the driver stamps a turn; per point t = stamp + i·dt, assuming a steady head (`time_order` −1 when the driver flips the array: VERIFY sllidar's setting); `ScanClock` fits turn starts on the line under the stamps (serial jitter out) minus a bench latency; scans de-skewed with odom at each point's time (`scan_time.py`) | ~1.5 ms delivery jitter → < 0.6 ms; wrong time order = 15+ cm smear at 1.5 m/s + 3 rad/s (as no de-skew). Option if needed: the lidar's UART through a Teensy port, stamped per packet |

Everything reaches ROS on the Jetson's clock (system time on the robot, sim time in the sim): the MCU's times through the clock map, never by arrival. `/mcu/time_ref` carries the camera triggers and strobes (`sensor_msgs/TimeReference`).

**Frames (REP-105 / REP-103).** `robot_export.py` writes every fixed frame into `robot_v11.json` (`frames`): base_link → laser, imu_link, cam_left/right (x along the view) → cam_*_optical (z forward, x right, y down), module_pivot (the tilt axis; `robot_frames` adds module at the measured tilt), arm_mount (plate top centre). `robot_frames` publishes them on the robot; the bridges publish the same list in the sim. Calibration overrides a frame only from a passed run and only within 2 cm / 3° of the drawing. `frame_check` audits a live /tf: one parent per frame, map → odom → base_link, sensors on base_link, optical frames REP-103, jumps in odom → base_link (must be none) and the size of map → odom corrections. On the sim's goals step: no problems, 0 odom jumps, map → odom corrections up to 6 cm. Rules: **one** map → odom publisher (slam_toolbox where it runs; tag_slam only without it), odom → base_link from the EKF (or mcu_link), dock and tags hang off map.

**Calibration** (`ros/src/omni_calib`, README.md is the procedure, in order: clocks, intrinsics, stereo, camera timing, camera-IMU, lidar-camera, lidar timing, sensors on base_link, dock tag). Solvers in numpy / scipy (OpenCV only to detect), checked on synthetic data in the robot's geometry: focal length to 0.3%, stereo rotation < 0.02° and epipolar error < 0.3 px (the drawing's 0.15°-off rotation shows as 0.65 px), camera-IMU time offset ±0.2 ms and rotation < 0.3° from a 8 s wiggle (a constant spin is refused as unobservable), lidar-camera < 6 mm / 0.4° from 14 board poses with projected points on the board, lidar on base 4 mm / 0.3° (planar hand-eye), dock tag to contacts < 4 mm / 0.6° from 8 dockings. Every run is kept with its metrics; limits decide what may be used; session-to-session drift of the stereo rotation (> 0.1°) is flagged. **Online refinement** only bounded (`online.BoundedParam`: prior, bound, holds while unobservable, half its updates refused = inconsistent → recalibrate) and in the tag graph the camera's mount rotation (prior 0.5°, bound 1°, on only when the lidar fixes the map).

**AprilTag landmarks** (`ros/src/omni_slam`). Detection (OpenCV 36h11 on the robot; `tag_sim.py` in the sim: 16 cm tags at picture height every 2.4 m along the house's walls, a 10 cm dock tag, misses, misdecodes, corner noise 0.25 + 4/side px, 2D wall occlusion). Each view's pose from its corners; refused when small, oblique, far, badly fitted or low margin; **ambiguous** (its mirror pose fits within the noise) views don't start a landmark or relocalize. The graph (`graph.py`, sparse Levenberg-Marquardt; GTSAM's Pose2/Pose3/projection factors map one to one, `ros-jazzy-gtsam` if iSAM2 is needed): SE(2) keyframes with the IMU's roll / pitch as known attitude, SE(3) tags, odometry between-factors, the lidar's poses as absolute factors (or the first keyframe anchors the gauge), tag-corner reprojection factors (Huber), saved tag maps as priors; views still > 4 px after the fit dropped. `tag_slam` node next to slam_toolbox (it keeps map → odom): `/tag_slam/pose`, the tag map (`<map>.tags.json`), map → tag_<id> / dock / dock_contacts, `/initialpose` relocalization, `/localization/health`.

On v9's house tour (45 s, 46 m, 0.3° camera calibration error, the BNO085's own tilt bias; `studies/tag_slam.py`, `results/tag_slam.json`), position error median (95%):

| Estimator | Whole tour | 15 s without lidar fixes |
| --- | --- | --- |
| odometry (wheels + IMU EKF) | 24 cm (57) | 32 cm |
| tags + odometry (no lidar, unknown tags) | 11 cm (30) | 15 cm |
| lidar (v9 matcher on the known map; dead reckoning in the gap) | 1.4 cm | 11.4 cm |
| lidar + tags, camera refined | 3.0 cm | **9.0 cm** |

Tag map: 7 cm median from odometry alone, 3.2 cm with the lidar, **1.7 cm** with the camera refined (it recovers the camera error plus the IMU's tilt bias to 0.06°). Relocalization from the map's tags: single close views ≤ 12 cm; two independent views that agree (different tags, or one tag from 0.3 m apart) **2.8 cm median, max 9 cm / 1.7°**.

Findings: (1) **the tags don't beat a good lidar localizer in a known map**; they map themselves, bridge lidar gaps, relocalize (kidnap, start-up), check the lidar's answer and dock: keep the lidar primary, as the spec says; (2) the planar model must see through the IMU's roll and pitch: the sprung base's 1–2° is 10–20 px and dropped 70 views a tour, 1 with it; (3) a small near-frontal tag's mirror pose can fit better than the true one (0.01 vs 0.19 px) and be 1–2 m off: refine both branches and judge against the noise, never relocalize from one view, and two frames of one view are not two votes; (4) a lidar fix taken "latest" instead of at its time reads as a camera yaw (0.7°) in the refinement: every input at its own time. **Health** (`health.py`): σ ≤ 5 cm full speed, ≤ 12 cm 0.6, ≤ 25 cm 0.3, beyond stop; no absolute fix for 5 / 15 s one level down / POOR; a poor lidar match for 5 scans one level down; tags that agree with each other but not the estimate → relocalize from them; POOR for 3 s → LOST (stop, relocalize); the catch waits for the pulse clock. The console scales Follow / Program / Calibrate by it and shows it; Nav2's `/speed_limit` from it is optional (the speed filter also publishes there).

**What's left:** bench the timing constants (scope SYNC_OUT vs the Jetson's stamp, CAN reply turnaround, FSIN → exposure, the strobe), find an HTE-capable 40-pin GPIO on the Orin carrier, check sllidar's index order and stamp; write the camera node (V4L2 sequence + SOF → `FrameStamper`); record calibration sessions as rosbags and the scripts that run the solvers on them; check slam_toolbox takes `/initialpose` in localization; the catch tracker should take the camera stamps and model calibration error as a slow bias. **Nav2 goals step** (fixed Oct 25): 4 of 8 goals failed with "Unable to transform goal pose into costmap frame". MPPI transforms the plan's last pose at the robot pose's stamp and waited 0.1 s for map → odom; slam_toolbox's transform (post-dated 0.2 s) stalls up to ~0.9 s on the loaded sim stack. Now MPPI and both costmaps wait up to 1.0 s (`nav2.yaml`) and slam_toolbox restamps (`restamp_tf`): 8 of 8. The "stall" (probed Oct 25: ~1 s every 32.2 s of wall time, whatever slam_toolbox's settings, growing 0.8 → 1.1 s) **was WSL's clock, not slam_toolbox** (Oct 26). Two agents steer the Linux clock. WSL pulls it toward Windows' time through the kernel's `tick` (seen at 9444–9798 of 10000: up to 6% slow). systemd-timesyncd polls ntp.ubuntu.com every 32 s and jumps it forward when it is > 0.4 s behind. Windows ran 0.64 s behind NTP, so they never agree: steady time ran 3.2% slow against Windows and realtime stepped +0.9…+1.3 s every 32.25 s, with nothing running. A gap timed on the wall clock shows each step as a ~1 s stall. Checked with a probe that times on the monotonic clock (`/tf` map → odom arrival and stamp against `/clock`, slam_toolbox's `/pose`):
- stock slam_toolbox on a synthetic room, wall or sim time: 0 stalls across the steps;
- our localize step, standing, turning and driving: 0;
- the robot step on the soak patrol, 7 min each with `restamp_tf` true and false (13 steps each, ~650 scans localized): 0 arrival gaps and no stamp lag (one 0.34 s at start-up).

Our robot code times durations on `time.monotonic()`; `time.time()` only stamps logs and events. The fix is one clock discipline in WSL: `sudo systemctl disable --now systemd-timesyncd` (now in `ros/setup/install_wsl.sh`; with it stopped: 0 steps in 120 s, tick back to 10000, realtime within 25 ms of Windows). The Nav2 waits stay at 1.0 s as margin. On the Jetson: one NTP client (chrony), stepping only at boot. MPPI's `GoalCritic` and `PathFollowCritic` switch at 0.5 m now (1.4: straight-line distance, so a goal just behind a wall dropped the path).

## 11. Arm module: an SO-101 in place of the catch module (Oct 23, proposed, sim only)

Fernando (Oct 23): the robot will carry an arm, likely an SO-101, where the catch module (funnel, pocket, launcher) is now. This is the sim of that, flag-based, with no CAD yet; the history file's §15 has the payload interface.

**In the sim** (`simulation/v11 sil`):
- `SimParams(payload="arm")` drops the tilt body and the whole catch module (funnel, pocket, feed, flywheels, tilt servo: 1.23 kg in the sim) and bolts `arm_so101.py`'s arm onto the payload plate at `arm_mount` (x, y, yaw).
- The arm is TheRobotStudio's official SO-101 MuJoCo model, its links, ranges and inertials verbatim (0.63 kg, Apache-2.0, pinned commit). Added (est): adapter, bus board and cables 0.10 kg, wrist camera 0.03 kg, `arm_payload_kg` at the tool.
- Six STS3215 12 V from the database (2.94 N·m stall, 4.7 rad/s); their torque falls with speed. The arm's capsules collide with the floor only.
- `extra_masses` adds point masses (ballast, the CAD's mass gap). `Robot` and `SilRobot` (the firmware in the loop) run it unchanged, and the defaults reproduce earlier results.
- `arm_view.py` shows it (Launcher: "Arm (SO-101) on the base"); the meshes come from `python arm_so101.py fetch`, not in git.
- `studies/arm_study.py` writes `results/arm_study.json`, `arm_reach.png`; `renders/v11_arm_poses.png` shows the poses.

**Setup.**
- Plate: the CAD's (`plate_gap` 0.096, top 168.5 mm).
- Mounts: the plate's centre, or 60 mm ahead.
- Two mass models:
  - "sim": 5.93 kg with the arm.
  - "cad": a 1.93 kg point mass added so that the robot without its module matches the CAD's 7.10 kg (cad/v7 BOM), 7.86 kg with the arm.
- Firmware limits: 2 m/s, the direction table up to 4.26 m/s² (stops ramp at the table).
- Explicit rollers, sprung pods.
- The X4's support square has its edges 106 mm out ahead, behind and to each side, and its corners at the wheels, 150 mm out.

**Reach** (the tool point, m from the robot's centre; the bumper is at 0.217):

| Mount | Arm | Floor | Floor, top-down | Coffee table 0.45 m | 0.75 m / 0.90 m | Highest |
| --- | --- | --- | --- | --- | --- | --- |
| centre | ahead | 0.23-0.39 | 0.23-0.37 | to 0.44 | out of reach | 0.70 |
| centre | side | 0.23-0.35 | 0.23-0.33 | to 0.40 | out of reach | 0.70 |
| 60 mm ahead | ahead | 0.23-0.45 | 0.23-0.43 | to 0.50 | out of reach | 0.70 |

The stereo bar across the plate's front edge blocks the front mount (it must split into two pods) and steep reaches just past the bumper. Reaching for the floor ahead hides 11-21° of the lidar's scan.

**Findings**

1. **It doesn't tip.** No run tipped:
   - 486 stops from 1 and 2 m/s with the firmware's ramp, in 4 directions, the catch-module baseline included. The arm cases cover 5 poses, 0-0.5 kg at the tool, both mounts and both mass models.
   - 24 parked full-speed swings.
   - 32 crossings at 1 m/s of a 12 mm square rug edge and an 18 mm threshold.

   Peak tilts against the tilt that would put the CG over an edge:
   - stops: ≤ 12° (16-45° depending on pose and payload);
   - over the threshold, arm level with 0.25 kg: 13.5° (26°).

   The C++ firmware (SilRobot) agrees with the Python controller: 8.9 vs 9.2°, 12.2 vs 11.4°.
2. **The margin shrinks with the arm out, most with the mount ahead.** The closest case is the front mount, arm level at full reach with 0.5 kg: 11.4° of 16.4°. The servos can't hold that pose anyway; it sags 41 mm. Heading toward a wheel (45°) is the worst direction on the sprung pods, which pitch about a diagonal, even though statically the wheel side has the most support.
3. **Wheels unload, as the catch robot's already do.**
   - Catch robot: a loaded wheel is off the floor for 0.17-0.24 s per stop.
   - Arm stowed: 0.04-0.19 s.
   - Arm out with 0.25 kg: 0.4-1.4 s, and its lightest wheel carries only 3-10 N at rest. That costs traction and odometry, but it is not a tip.
4. **The SO-101, not the base, limits the payload.**
   - At full reach (0.36-0.52 m from the centre) the servos hold 0.55-0.8 kg at stall, but only 0.03-0.09 kg at their rated 0.98 N·m (continuous).
   - Holding the arm level and empty uses half of stall.
   - At the same reach the base tips only at 1.1-2.0 kg (sim masses) or 1.7-3.0 kg (CAD masses).
   - A horizontal push at the tool at shoulder or table height tips the sim robot at 10-26 N, about what the arm can push close in. Floor-level pushes take over 100 N.
5. **Fixes for heavier work** (the worst stops the servos can hold, ≤ 0.25 kg):
   - Half the acceleration table: peak tilt 8.6 → 2.5° and 8.1 → 5.3°. The firmware already scales the table (kid mode's `accel_scale`).
   - 2 kg of ballast under the battery: about half the lifted-wheel time, less effect on tilt.
   - Proposed: an "arm out" profile the Jetson asks for (accel × 0.5, 1 m/s) while the tool is outside the support polygon or above 0.4 m; drive with the arm stowed or carrying.

**Not modelled:** the arm's self-collisions (joint limits only), cable drag, servo backlash and heating, the LeRobot side, the CAD's masses for the base (the "cad" point mass stands in).

### 11.1 Picking things off the floor (Oct 24, proposed, sim and ROS)

Fernando (Oct 24): drive the arm from ROS. The end goal is to pick up small things from the floor by itself, like a cleaning robot. He asked whether MoveIt is right first, then chose:
- the Jetson drives the servos (a USB bus board, not the Teensy);
- the wrist camera now;
- a bin on the robot;
- the arm may sit lower and further forward, off the plate or on a wider one;
- the stereo pair may move down to see the floor.

**Not MoveIt.** The SO-101 has 5 joints and works in a small, fixed space just in front of the bumper.
- A top-down grasp has an exact solution, and the robot's outline doesn't move, so a few checked waypoints are all the planning it needs. MoveIt's IK expects 6 joints.
- The hard parts are seeing the floor, the servos' calibration error, and knowing that the grasp held. MoveIt does none of these.
- `arm_node` still serves `control_msgs/FollowJointTrajectory`, so MoveIt (for tables and clutter later) or a LeRobot policy (learned grasps for odd things) can drive the same arm.

**The arm robot of Oct 24** (`studies/arm_mount.py`, `results/arm_mount.json`). Superseded on Oct 25 by the arm kit as drawn (section 12.3), which is `robot_export.ARM` now; this layout stays as an option (`arm_mount_z`, `camera_mount="nose_pods"`) and its findings below stand.

| Part | Where | Why |
| --- | --- | --- |
| Arm | On a nose bracket from the deck's front lobe, adapter at x 0.12, z 0.105 m (the plate gets a notch) | Floor grasps fingers-down reach 0.25–0.47 m from the centre (3–25 cm past the bumper), against 0.25–0.38 on the plate. It drops into the bin behind it and folds inside the bumper's edge. Mounts 30–50 mm further forward reach 3 cm more but stow outside the bumper |
| Stereo pair | Two pods either side of the bracket, x 0.17, z 0.125, ±95 mm, looking 35° down | The floor from 0.22 m to the horizon |
| Lidar | The deck's rear edge, the same scan plane | The low arm is where it was. The arm base hides ~15° of the forward scan |
| Bin and display | Bin 150 × 140 × 70 mm on the plate behind the arm. The 7 in display in a 25 mm housing on the bin's back, facing the rear | The CAD's 65 mm display housing sat where the bin goes. The display backs the drops |
| Wrist camera | A 32 × 32 mm UVC module 40 mm beside the fixed jaw, looking down past the fingers (est) | The SO-101's own wrist mount looks along the fingers, so a top-down hover sees them, not the object |
| Poses | stow (upper arm up, forearm folded down over the nose), high, ready (fingers down 16 cm up), drop (tool 5 cm behind the bin's middle, fingers 50° back) | The arm reaches behind itself only that far |

**The tidy** (`omni_arm/pick.py`). It looks, approaches, takes a close look, hovers, takes a wrist look, descends, closes, lifts and drops, in that order.
- **Looking.** Depth comes from the left pod. Floor things are separated from the floor plane and from rug levels, then grouped on a top-down grid. Each one needs seen floor around it, which rules out rug edges and things cut off at the image edge.
- **Approaching.** The base drives straight to each thing (no Nav2 yet) until it sits 0.33 m ahead, then takes a closer stereo look.
- **The wrist look.** The arm hovers 8 cm over the thing. The wrist camera masks out the gripper's own pixels and segments by colour against the floor around the thing. This cancels the servos' calibration error, which the sim shows at 1–3 cm.
- **The grasp.** The fingers close across the short side, tips 5 mm off the floor (2 mm for things under 20 mm). The gap the fingers stop at says whether it held. A miss or a drop means stow, then a fresh close look, then retry (two retries).
- **Searching.** When nothing is in view it turns 60° at a time, goes back to its start for a look round, and drives toward far sightings.
- **The base while the arm is out.** The base is capped at 1 m/s and 2.1 m/s², whoever drives: `ModeMux.arm_out`, from `/arm/out`.

**In the sim:**
- `arm_pick=True` gives each finger a pad that grasps (contact bit 8) and a fingertip contact with the floor.
- `arm_objects.py` puts the things on the floor: blocks, cubes, crayons, a marker, caps, a ball.
- `arm_bus.py` emulates the six servos' bus, with a 1° rms calibration error per joint.
- `sil_pick.py` runs the whole stack in one process. Its stereo error model is 0.25 px disparity on the full pair plus 3% unmatched pixels, and the left camera is 0.3° off its calibration.
- ROS: `omni.sh arm` (the bridge serves the bus on TCP 5763 and renders the cameras on request through EGL in WSL), `arm_node`, PICK in `mode_manager`, and the console's Tidy tab. Besides the tidy, the tab moves the arm by hand: the poses stow, ready, high and over the bin (checked paths), the gripper, torque, and one joint at a time (`jog`: up to 30° a step, kept in range, refused if it would hit the robot, the bumper or the floor; `motion.jog`).

**Results** (`studies/arm_pick.py`, 5 things per run, 6 seeds per set and world, `results/arm_pick.json`). The robot's own code ran with the C++ firmware, the servos 1° off their calibration and the camera 0.3° off:

| World | Toys (blocks, cubes, a crayon, a ball) | Clutter (markers, caps, a brick, cubes, crayons) |
| --- | --- | --- |
| Arena (bare wooden floor) | 28/30 | 26/30 |
| House (hallway and living room: a runner, rugs, carpet through a doorway) | 23/30 | 21/30 |

- **Overall:** 98/120 in the bin (82%), about 46 s of sim per thing picked.
- **Grasping:** once a thing is reached the grasp mostly works: 4 missed grasps and 12 dropped on the lift (caps, a ball) over 120 things, most recovered on the retry.
- **What it lost:** things never found (the house: sightings beyond 1.6 m, partial views), a few things dropped beside the bin, and 3 too big (a 13 cm marker seen whole).
- **The cost of phantoms:** 90 close looks found nothing (carpet seen through a doorway, rug patches at range). Each costs ~5 s and none got grasped.
- **Through ROS** (`omni.sh arm`, WSL, ~0.75× real time): `arm_node` on the bridge's servo bus picked a thing into the bin end to end in ~50 s, with the console's mode switch, PICK and the arm-out caps.

**Not modelled / limits:**
- Contacts: the wheels pass through loose things below the bumper (the tidy stops short of them), and the arm's links touch things only with the finger pads.
- Stereo: rendered depth plus noise, with no matching failures on bare floors or reflections. Heights read ~1 cm low on the house's renders.
- Segmentation: by colour only, so a thing the floor's colour falls back to the stereo estimate.
- Arm: no servo backlash or deadband.
- Approach: straight driving with no obstacle check beyond the bumper.

**Next:**
- CAD of the nose bracket, the pods, the bin and the thin display, plus the plate's notch.
- What the e-stop does to the arm (it drops).
- Long approaches through Nav2.
- A learned floor-thing detector and segmenter on the Jetson.
- Calibrate the real servos with LeRobot's tool (`ros/src/omni_arm/README.md`).
- Bench the wrist camera's pose.
- Learned grasps (a LeRobot policy) for things the top-down grasp can't take.

## 12. One layout for both payloads (Oct 24; drawn and baselined Oct 25: 12.2, 12.3)

Fernando (Oct 24): the catch robot and the arm robot each have a layout that suits neither well. He asked for a layout that serves both, from the same base, with:
- the lidar's view weighed: is a blocked back costly, or worth a high, clear spot?
- the stereo pair on one stiff bar, tilted per mode, unless separate pods are fine once calibrated;
- a display at the front, so people see what the robot is doing and it has a clear forward direction (5 in is fine if 7 in won't fit);
- as much arm reach as can be had.

Swaps are rare and may take hours; plug-in plates would be a bonus.

**How it was checked.** `simulation/v11 sil/studies/layout_trade.py` builds each layout as the v11 model with the CAD's 96 mm plate gap. The sim's own lidar, cameras, standoffs and display are swapped for the layout's (MjSpec). Every check then casts rays or measures distances against the sim's own geometry: the catch module at every tilt, and the SO-101's meshes at its poses.
- **Lidar:** rays in the A2M12's 150 mm scan plane. It reports the degrees blocked, the widest blocked wedge, and the travel directions with a *hidden lane*: a strip of the robot's path, 0.4-1.5 m ahead, that stays in a shadow the whole way. A thin shadow hides a thin thing only while the robot drives along that shadow.
- **Cameras:** the ball's last 1.5 s of a 6.5 m throw, seen by both cameras with the module at the 35° catch tilt; the floor ahead, seen by both cameras with the arm stowed.
- **Display:** the share of the panel each viewer sees: an adult at 1.5 m, a kid at 1.2 m (straight on and 40° off), the thrower at 6.5 m.
- **Clearances:** the module against the head at −30…+40° of tilt, and the launch line (the ball plus 5 mm).
- **Arm:** `arm_mount.py`'s sweep with the layout's obstacles (top-down floor grasps), a search for bin positions it can drop into, and a stow search. A stow must stay inside the bumper and above the plate, and is chosen to leave the cameras the most floor.
- **`L`:** v9's house tour (v7 base, BNO085 + A2M12, wheels + IMU + lidar EKF, 3 seeds) with each layout's shadow blanked from the scans.

Renders: `renders/layout_trade.png` and `renders/layout_lidar.png`. Results: `results/layout_trade.json` and `layout_lidar.json`.

**The lidar: what a blocked sector costs.** The house tour with sectors blanked:

| Scan | Blocked | Position median / p95 | Yaw p95 |
| --- | --- | --- | --- |
| Full 360° | 0° | 12.9 / 26.6 mm | 0.34° |
| Front lidar, the CAD's six standoffs (catch robot; proposed arm robot, stowed) | 31° | 12.9 / 27.2 mm | 0.34° |
| Proposed arm robot during a far grasp (the arm crosses the scan ahead) | 45° | 12.8 / 26.6 mm | 0.35° |
| Rear lidar with the nose arm (arm robot today) | 50° | 13.1 / 25.4 mm | 0.35° |
| Rear 90° blocked | 90° | 14.0 / 32.0 mm | 0.33° |
| Rear half blocked | 180° | 21.3 / 49.9 mm | 0.42° |

- **Localization doesn't care until about a quarter of the view is gone; half gone doubles the p95.** A clear 360° buys nothing measurable over today's 330°.
- **What a shadow costs is obstacle sight, and it depends on where the shadow points.**
  - The front lidar's shadows are thin wedges at the rear diagonals: 128-152°, 208-232° and around 180°.
  - The arm robot as drawn on Oct 24 (lidar at the rear, the arm's base and the side standoffs ahead of it) has **hidden lanes straight ahead**: 0-20°, 32-58° and 302-358°. That is the way it drives.
  - The widest wedges (12.5°) come from the front pair of 12 mm standoffs, only 55 mm from the lidar. 8 mm steel rods would make them 8°.
- **Keep the lidar at the front of its layer and keep that layer free**, in both modes. A high mast is not worth it: no spot over the funnel or the arm is free, and a high scan misses chair rungs and low furniture.

**The cameras: one bar.** The catch rate depends most on the stereo pair's *relative* calibration (0.04° → 97%, 0.3° → 78%, section 3), and the arm's depth depends on it too.
- Two pods on separate printed posts from the deck's front lobe move against each other whenever the sprung deck flexes, the PETG creeps or warms unevenly (est: 0.05-0.1° for a 10 K gradient across a post).
- A bar carrying both cameras keeps them together. Then only the bar's pose on the robot changes, which `omni_calib`'s "sensors on base_link" step and the floor plane recover.
- Make it a 20 × 20 mm aluminium square tube, not today's 25 × 6 flat bar. The flat bar is weak fore and aft, and a fore-aft bend shows as camera yaw, the direction that biases disparity. Hold it at one fixed point and one sliding one, so plate flex can't bend it.
- The bar turns about its own axis in two pillow blocks between **indexed stops: 30° up for catching, 40° down for the arm**. Swapping payloads moves one pin. Both cameras stay where the CAD has them (x 0.184, ±95 mm, z 0.201).
- With the bar 40° down on the arm robot:
  - The floor is seen from 0.25 m ahead.
  - Both cameras see 87% of the near zone (0.25-0.6 m, |y| < 0.2) and 94% of the far zone (0.6-1.6 m, |y| < 0.6). Today's pods see 77% and 93%.
  - The mean range to the near zone is 0.34 m.
- A head on a 43 cm mast (A2) sees the floor only from 0.44 m, the stowed arm fills a third of each image, and its near-zone range doubles (0.68 m): ~4× the depth error.

**The display: 5 in at the front, between the cameras.** The catch game never launches flatter than 30° (`min_return_elevation_deg`), so below the launch line there is room at the plate's front edge.
- **The 5 in fits.** A 5 in 800×480 panel (housing 129 × 85 mm) hangs from the bar between the cameras:
  - leaning back 10°, its face behind the lenses (x 0.194);
  - its bottom 2.5 mm over the lidar's top and ahead of it, so the scan passes under it.
  - Every viewer sees all of it, the thrower 2° off its axis. Today's rear-facing 7 in: no viewer in front sees it.
  - Clearances: the module (whole tilt range) stays ≥ 16 mm away; the closest part is the launcher's motor bracket at −6° in the sim's model (VERIFY in CAD). The launch line clears it from 22° up, with 17 mm to spare at 30° after a 5 mm margin. The Thrower bench mode's lowest elevation rises from ~10° to ~25°.
  - It wants a 2 mm polycarbonate window: a ball thrown low hits it.
- **The 7 in does not fit:** it hits the motor bracket at −10° tilt and the 30° launch line (−7 mm), and it needs a 230 mm baseline.

**The arm: at the plate's front edge, right behind the bar.**

| Layout | Floor, top-down (m from the centre; bumper 0.217) | Pan 60° / 90° | Bin drop | Lidar | Display |
| --- | --- | --- | --- | --- | --- |
| A0 today: nose bracket in the lidar layer, pods, rear lidar | 0.25-0.47 | 0.41 / 0.35 | 45° off vertical | 50° blocked, lanes hidden ahead | 7 in facing the rear |
| A1: on the plate behind the full head (x 0.05) | 0.25-0.36 | 0.32 / 0.28 | none | 31° | in the head |
| A1 on a 60 mm riser | 0.25-0.31 | 0.28 / - | none | (the sim draws the riser down to the deck) | in the head |
| A3: plate edge (x 0.10), display still in the head | **none** | - | - | - | in the head |
| **A5: plate edge (x 0.10) behind the bar, bin at the rear, display on a post** | **0.26-0.42** | 0.37 / 0.31 | **18° off vertical** | 31° stowed, 45° grasping | all of it seen |
| A2: plate edge (x 0.12), cameras on a 43 cm mast | 0.25-0.44 | 0.38 / 0.32 | none within 12 cm of the rim | 31° | on the mast |

- **A display in the head's middle blocks every floor grasp**, because the arm comes down exactly there. On the arm robot the display moves to a post at the plate's rear, its centre 0.40 m up, over the bin. There every viewer sees all of it, over the stowed arm.
  - On the bin's front wall instead it would be hidden behind the arm's base: kid 59%, thrower 26%.
- **Reach:** the plate-edge arm gives up 5 cm against the nose (0.42 vs 0.47 m). The tidy works at 0.33 m.
  - Without the bar in front it would reach 0.44 m (A2's mount).
  - Mounting it lower bought ~1 cm in the Oct 24 study; mounting it higher costs (riser: 0.31 m).
- **Bin:** 12 × 12 × 7 cm at the plate's rear (x −0.07, overhanging the plate inside the bumper).
  - The arm folds back over itself to drop with the fingers 18° off vertical. Bins nearer the arm need it folded tighter than it can go.
  - The pose sits at the elbow's and wrist's limits (3 grid poses): bench it. A bin reaching forward to x −0.01 drops at 45-60°.
- **Stow:** folded sideways (pan −90°), top 0.315 m, clear of the cameras' view, the display and the lidar layer.
- **Mass:** CG 18 mm higher than A0 (z 0.104 vs 0.086, sim masses), because the arm sits on the plate. The Oct 23 study found no tip with the arm on the plate.

**The proposal.**
- **Base, both modes:** the lidar at the front of its layer, the layer kept free; one camera bar on the plate's front edge.
- **Catch kit:** the module, as now, plus the 5 in panel in the head's middle.
- **Arm kit:** the arm on a plate-edge adapter, the rear bin and the display post.
- **A swap:** unbolt one kit, bolt the other to the plate's M3 grid and dowels, move the bar's index pin, move the panel (one HDMI and one USB cable), then check the cameras on the floor plane. It is close to plug-in plates.

**Not modelled / limits:**
- Geometry only, from the sim's module, not the CAD's: the yokes, the ESC and the bracket details differ.
- The ball's flight is drag-free.
- No vibration or thermal model of the bar.
- The display's viewers are points.
- The arm's grasps are the sweep's, not run through `arm_pick.py`.
- The lidar tours use v9's matcher, not slam_toolbox.

### 12.1 CAD brief (for drawing the proposal in `cad/v7`)

Everything below is in the ROBOT frame of `cad/v7/dims.py`, in mm: origin on the floor under the centre, x forward, z up. Fixed context: plate top 168.5 and underside 162.5, plate radius 120; A2M12 at (140, 0), scan plane 150, top 160.5, head radius 35 (x 105-175); bumper radius 217; e-stop at (−168, 0). The study's numbers are constants in `layout_trade.py` (`HEAD_BAR`, `HEAD_CAM`, `HEAD_5`, `DISPLAY_BOT`, `BIN_SIZE`, `POST_5`, `LAYOUTS["A5"]`). After drawing, set those to the CAD's values and rerun it (`python layout_trade.py`, then `png`), so the checks run on the drawn geometry.

**Kept from today's CAD** (base, both modes): the deck, pods, wheels, battery, Jetson and lidar; the payload plate with its 25 mm M3 grid, two Ø4 dowels and Ø28 pass-through; the catch module, yokes and their boards.

**Removed from today's CAD:**
- `mounts.py`: `cam_arm` ×2, `cam_mount` ×2 and the 25 × 6 `stereo_bar`.
- `display.py`: the rear 7 in housing and bezel.

**New, base (both modes): the head.**
- **Camera bar:** 6061 square tube 20 × 20 × 1.5, ~250 long, along y. It turns about its own axis.
  - Put the axis through both cameras' optical centres, at about (160, ±95, 201). The index then changes only their pitch. Turned about an axis below and behind the lenses, the cameras would drop into the scan plane at −40°.
  - Keep the tube at x 150-170, so it passes behind the display.
- **Pillow blocks:** two printed blocks on the plate's front edge, one at each end inboard of the cameras, at |y| ≈ 110.
  - The bar overhangs the plate (radius 120), so the blocks need a short tongue or bracket forward from the plate's edge.
  - Everything stays above z 162.5, because the lidar is underneath.
  - One block is fixed and the other lets the tube slide along its axis, so plate flex can't bend the bar.
  - Kit bearings (608) or a printed bushing with a clamp screw.
- **Index:** a printed disc clamped on the tube, with an M3 pin (or a dowel) into holes for +30° (catch) and −40° (arm), plus a clamp screw so it holds without play.
- **Cameras:** OV9782 boards on clamps at the tube's ends, lens on the axis (est board 38 × 38, M2 at ±17: VERIFY), 190 mm apart. Each clamp is keyed to the tube's flat face. The pair is calibrated once on the bar.

**New, catch kit: the 5 in display at the front.**
- **Panel:** a 5 in HDMI 800×480, about 121 × 77 module, in a housing 129 × 85 × 16 (est; pick the part and VERIFY its outline, HDMI and USB connectors and driver board).
- **Where:** a bracket from the pillow blocks (not the turning tube) holds it between the cameras (|y| ≤ 64.5), leaning back 6°:
  - its front face at x 194, behind the lenses;
  - its bottom at z 163, ahead of the lidar's head (x > 175);
  - its top about z 248.
  - At the study's 10° lean its back would cross the tube's front face (x 170) near z 211; 6° leaves ~3 mm.
- **Window:** a 2 mm polycarbonate front window.
- **Clearances to keep** (sim geometry; check against the CAD's module):
  - ≥ 16 mm to the launcher's motor bracket at −6° tilt;
  - the launch line (ball plus 5 mm) clear for every elevation ≥ 22°;
  - the funnel's rim at +40° (x 171, z 304).
- **Firmware:** the Thrower bench mode's lowest launch elevation becomes 25°. The game's flattest is already 30°.

**New, arm kit:**
- **Arm adapter:** a printed plate on the plate's M3 grid and dowels.
  - It puts the SO-101's base origin at (100, 0, 168.5 + 6 mm adapter); the shoulder-pan axis is then at x 139.
  - It overhangs the plate's front edge above the lidar.
  - Take the base's hole pattern from TheRobotStudio's SO-ARM100 STEP files (`arm_so101.py` pins the commit; VERIFY).
  - The servo bus board goes beside the base.
- **Bin:** 120 × 120 × 70 at (−70, 0) on the plate.
  - Its corners reach radius 143, past the plate, so it needs a lip or brackets.
  - It sits inside the bumper (radius 217).
  - It must not cover the plate's rear standoffs' screws, or the e-stop's reach.
- **Display post:** the 5 in panel's centre at (−132, 0, 400), facing forward, leaning back 10°.
  - The study's post stood at x −152…−172 on the centreline, which is **where the e-stop is**.
  - Move the post off the centre (two posts at |y| ≈ 40, or one at y ≈ 45), or stand it on the bin's back wall. Then recheck the drop pose against it.
- **Arm poses** to check for clashes (degrees, joint order pan / lift / elbow / wrist flex / wrist roll / gripper):

  | Pose | Pan | Lift | Elbow | Wrist flex | Wrist roll | Gripper |
  | --- | --- | --- | --- | --- | --- | --- |
  | Stow | −90 | 100 | 87.6 | −31.7 | 0 | 0 |
  | Far grasp (0.417 m) | 0 | 100 | −70.1 | 49.1 | 0 | 0 |
  | Near grasp (0.259 m) | 0 | 65.5 | 23.4 | 9.8 | 0 | 0 |
  | Drop | 0 | −60 | −96.8 | −95 | 0 | 0 |

  - The drop pose sits at the elbow's and wrist's limits.
  - `build.py` checks the catch module over its tilt range but has no arm: add the SO-101 (STEP from the same repo) at these poses.

**Open decisions to settle in CAD:**
1. **Mics and speaker.** Today they sit in `display.py`'s audio base at the plate's rear (x −121…−70), which is where the arm's bin goes.
   - Proposal: one "face" unit (panel, mic array and speaker) that moves between its two seats.
   - If it doesn't fit in front of the bar, keep the audio base on the catch robot and put a second spot for it beside the bin on the arm robot.
2. **Front standoffs.** Replace the front pair (Ø12 printed, 55 mm from the lidar, the 12.5° wedges) with 8 mm steel rods or thicker-walled printed ones nearer Ø8 (about 8°). They must still carry the arm's moment at the plate's front edge.
3. **Cables.** Two FFC or MIPI runs from the bar to the Jetson along the pillow blocks; the panel's HDMI and USB, which move with the panel between seats.

**The swap** (catch → arm):
1. Remove the catch module and its yokes from the plate.
2. Move the panel (and the face unit) to the arm seat.
3. Turn the bar from +30° to −40° and set the pin.
4. Bolt the arm adapter, the bin and the display post to the grid.
5. Check the cameras against the floor plane (`omni_calib`'s sensors-on-base step).

### 12.2 As drawn (cad/v7, Oct 24)

Fernando (Oct 24): a detailed CAD of the sim's layout; the cameras, the lidar's shadows and the display at the front come first, everything else can move. Drawn for the catch robot, with the bar's arm index checked; the arm kit (adapter, bin, post) is not drawn. Files: `cad/v7/head.py`, `display.py`, `base.py` (rods); README "Head" and "Face and voice".

**What was drawn** (ROBOT frame, mm):
- **Camera bar:** 6061 square tube 20 × 20 × 1.5, 228 long, axis at x 155, z 195, behind the display. It can't pass through the lenses as the brief wanted: the display stands in front of it.
- **Camera holders** (printed, one per end): a sleeve on the tube (one M3 × 30 through the sleeve and both tube walls), the OV9782 board on its front face. Optical centres at **(192, ±95, 211), 30° up** at the catch index: 7 mm behind the study's camera, so the lenses stay inside the bumper's 217 mm radius. At the arm index (−40°): (183, ±95, 166).
- **Cheeks** (printed, |y| 64–72): a pad on the plate's front edge (3 × M3), a web under the module's swept volume, a ring round each holder's 34 mm journal, a horn beside the face. **Index:** one M3 × 10 radially through the left ring into one of two tapped holes in the left journal (6.5 mm of thread at both); the right journal slides in its bore, so plate flex can't bend the bar.
- **Face:** 5 in 800 × 480 panel between the cameras, front-bottom edge at (198, 160), leaning back 10°: housing (lip, rails for the screws), 2 mm polycarbonate window, back cover, 2 × M3 × 16 from each cheek. The audio box (mics, speaker) stays at the plate's rear; the rear 7 in and its shelf are gone.
- **Front standoffs:** Ø8 6061 rods (M3 tapped at both ends, 1 mm recesses in deck and plate) in place of the printed Ø12 pair. The other four stay printed and carry the sway.

**Checked:**

| Check | Result | Where |
| --- | --- | --- |
| Clashes: static, module at −30…+40°, wheels at bump and droop, the bar turned from +30° to −40° (module and face off) | 0 | `build.py` (`checks.json` "sweep", "index") |
| Screws, prints | 0 short of thread, 0 access problems, every print one solid; no new thin walls | `build.py`, `walls.py` |
| Module to the head (exact) | ≥ 6 mm (a flywheel mount near a cheek) | `checks.json` "head" |
| Launch line (ball + 5 mm) | clears the face from **24°** up; 13.5 mm to spare at 30° (study: 22°, 17 mm). Thrower bench mode ≥ 25° | same |
| Lidar (scan plane 150) | **22° blocked** (was 31°), widest wedge **8.2°** (was 12.5°); nothing of the head below z 157 | `build.py`; `layout_trade.py` C2 |
| Cameras at the catch tilt (the CAD's meshes) | both see 95% / 92% of the last 1.5 s of 6.5 m throws (apex 1.6 / 2.4 m); the robot fills 0% of either image | `layout_trade.py` C2 |
| Display | adult, kids and thrower see all of it (thrower 2° off its axis) | same |
| Localization with that shadow (v9 house tour, 3 seeds) | 12.8 mm median, 27.0 mm p95: the same as a clear 360° (12.9 / 26.6) | `layout_trade.py L` |
| Arm index with the SO-101 (A6: as A5, the drawn head) | top-down floor grasps 0.26–0.42 m, bin drop 18° off vertical: as A5. Both cameras see the floor from 0.23 m ahead (A5: 0.25), 85% of the near zone, 94% of the far one. With the bar 10 mm higher (top at 219) the arm reached no floor at all, so the bar sits at z 195 | `layout_trade.py` A6 |
| Catch on the drawn robot | 92.6% (sim game), 89.8% (real code) vs 93.1% / 88.4% for today's sim robot, the same 216 throws | `sil_catch.py both --robot cad` |

The study's C1 / A5 used boxes; C2 / A6 load the CAD's own meshes (`cad/v7/out/head_stl`, from `build.py` or `build.py --head-only`) and take the module and launch clearances from the CAD's exact check.

**Open:** VERIFY the 5 in panel (outline, depth, HDMI / USB connectors), the OV9782 board holes and its lens's optical centre (16 mm ahead of the board's back assumed), 8 mm aluminium rod or Ø8 M3 round standoffs of 98 mm. Route the head's cables down the side standoffs on the side away from the lidar (inside the rods' wedges). The arm kit. (Done Oct 25: the face's software for 800 × 480, the Thrower's 25° floor.)

### 12.3 The arm kit, as drawn (cad/v7 arm_kit.py, Oct 25, baseline)

Fernando (Oct 25): baseline the drawn design, and do the arm kit too. Files: `cad/v7/arm_kit.py`, `display.py` (the face's two seats), `assembly.build(kit="arm")`; `simulation/v11 sil`: `robot_export.ARM`, `arm_so101.py` (`arm_riser`, `arm_display="face"`, `start_pose`), `mesh_sdf.py`, `studies/layout_trade.py` A6 (`arm-check`, `arm-stl`), `studies/arm_study.py K`; `omni_arm` (`poses`, keep-outs and `reach` from `robot_arm.json`).

**What the brief got wrong.** The study's A5 put the SO-101 on the plate at x 0.10, right behind the camera bar. The SO-101's base reaches 65 mm ahead of its frame and its shoulder bracket 86 mm, from 0 to 100 mm up: both stand where the bar is. The reach study checked thin capsules along the links and never saw them. Every arm check now uses the arm's own meshes against the CAD's (`mesh_sdf.py`: a 1.5 mm distance field of the drawn parts; ≥ 3 mm wanted). Mounts on the plate clear only at x ≤ 0.05 (floor to 0.36 m, no bin drop); a riser lets the arm stand further forward.

**The kit** (ROBOT frame, mm):

| Part | What | Why |
| --- | --- | --- |
| Riser + adapter | A hollow printed block on the plate's front edge (4 × M3, keyed by the front dowel), 43.5 tall; a 10 mm adapter on it running forward over the camera bar. The SO-101's base origin at (140, 0, 224.5) | The base clears everything but its adapter by 9.8 mm and the bar's top by 3+ mm at any index; its front edge stays inside the bumper (x 150 would not) |
| Bin | 120 × 120 × 70 on the plate's rear (x −130…−10), the audio box's 4 holes | A5's spot: the drop 10° off vertical |
| Uprights, face, shelf, audio box | Two uprights as the cheeks (y ±64-72) on the bin's walls carry the same face unit, its front-face middle at (−132, 0, 400), facing forward; the audio box on a shelf above it (top 0.50 m) | Every viewer sees all of the face over the stowed arm. The e-stop stays reachable from behind (the face's back edge is 25 cm over its front 2 cm) |
| 12 V buck | On the riser's left wall; the bus board rides on the arm's base | The catch kit's 12 V buck leaves with its yokes |
| Moved to the base | The Jetson's 19 V buck on posts on the front deck splice (under the plate, out of the scan plane); the RC receiver on the right cheek | They were on the catch kit's yokes |

**Poses** (`robot_export.ARM_POSES`, rad; omni_arm takes them): a stow folded sideways over the plate's right half (0.41 m up, 14 mm from the robot), an upright transit pose (high), a pre-drop folded back low, the drop 10° off vertical over the bin's middle. Each straight joint path stow–high, high–ready, high–pre-drop, pre-drop–drop clears the drawn robot by ≥ 9.8 mm (meshes) and omni_arm's keep-outs; a straight swing back to the drop would hit the face. The reach's sweet spot (where the base brings a thing) is 0.31 m: the middle of the fingers-down reach (0.26–0.36 m; it was 0.33 for the nose).

**Checked:**

| Check | Result | Where |
| --- | --- | --- |
| Arm robot clashes (static), the kit's 49 screws, prints | 0 clashes, 0 short of thread, 0 access problems, every print one solid, 7.8 kg | `build.py` (`checks.json` "arm_robot"), `out/ARM_KIT.md` |
| The SO-101's meshes at its fixed poses and grasps | ≥ 14 mm from the drawn robot; its base 9.8 mm from all but its adapter | `checks.json` "arm" |
| Reach (layout_trade A6, the CAD's meshes) | top-down floor 0.26–0.42 m (pan 30 / 60 / 90: 0.40 / 0.36 / 0.30); a drop 10° off vertical; the floor seen by both cameras from 0.23 m, 85% of the near zone, 94% of the far | `layout_trade.py` |
| Lidar | 22° blocked stowed, 36° while reaching far ahead | same |
| The tidy (6 seeds × toys / clutter × arena / house, the robot's own code, 1° servo and 0.3° camera calibration error) | **102/120** in the bin (nose layout 98/120); 14 missed grasps, 10 dropped on the lift, 71 close looks that found nothing | `arm_pick.py --seeds 6` |
| Ramped stops, 1-2 m/s, stowed / upright / reaching, 0-0.25 kg | none tip; worst 8.2° of 25.5° (upright, 0.25 kg) | `arm_study.py K` |

**The swap** (catch → arm; `cad/v7/README.md` has the steps): the face and the audio box off; the module and its yokes (with their ESC, RoboClaw, 12 V buck, servo) off; the bar to −40° (one screw); the riser, adapter and arm; the bin, uprights, face, shelf, audio box; check the cameras against the floor plane.

**Open:** VERIFY the SO-101 base's hole pattern (the adapter's 4 holes are est). What the e-stop does to the arm (no brakes).

## 13. Offloading compute to the operator PC (Oct 25, analysis, proposed)

Fernando (Oct 25): would it pay to send the larger operator computer some of the heavy work, for example a high-res image in which it finds the clutter to pick up?

**Where it can't help: anything on the catch's clock or that must work without Wi-Fi.**
- **The catch.** The stereo pair is 2 × 1280×800 at 120 fps, ~250 MB/s raw (~2 Gbit/s), about four times what home Wi-Fi carries at best. JPEG would cost encode time on an Orin Nano that has no video encoder (section 2). Latency is the catch's most sensitive input (section 3: perception, not the drive, limits it; 50 ms assumed end to end). One Wi-Fi stall of 50–200 ms during a 1 s flight loses that ball. Tracking, the intercept and the after-arrival sequence stay on the robot.
- **Never offloaded:** the MCU and the base's control, localization at catch time, Nav2's local planner and collision monitor, the face's drawing, "stop" by voice. The robot must work, and stop, with the PC off or the Wi-Fi down.

**Where it pays: the tidy, and talking.**
- **The tidy's looks are already stop-and-go.** The base stops, takes a close look (~5 s each), then grasps. A 1280×800 stereo pair as JPEG is ~0.5 MB (est), 40 ms at 100 Mbit/s and 0.2 s at a poor 20 Mbit/s. A desktop GPU (VERIFY which Fernando's PC has) then runs models the Orin can't run at all or runs slowly, in ~0.3–1.5 s (est) per look:
  - learned stereo (FoundationStereo, RAFT-Stereo class) in place of block matching;
  - open-vocabulary detection and segmentation (OWLv2 / Grounding DINO / YOLO-World plus SAM 2) in place of the colour segmenter;
  - a vision-language model to decide what is clutter and what is not ("a sock: pick it up", "a cable: leave it").
- **What that targets** (section 12.3's tidy, 102/120 in the bin): the 71 close looks that found nothing (carpet through a doorway, rug patches, ~5 s each), things never found (sightings beyond 1.6 m, partial views), things the colour segmenter merges with the floor, the 13 cm marker judged too big. Not the grasps themselves (14 missed, 10 dropped on the lift), unless a grasp-pose model runs on the wrist camera's view too. The gain is unmeasured until the sim scores both pipelines.
- **The Orin can do part of it itself:** a YOLO-class detector with TensorRT, a small SAM (MobileSAM / EfficientViT-SAM) (est: tens of fps / ~100 ms on an Orin Nano Super, VERIFY). The big stereo nets, SAM 2's large models and VLMs are the PC's.
- **Talking:** Whisper on the PC for free speech, and an LLM for conversation beyond the fixed grammar (a kid asking it anything), at chat's 1–2 s latency. The grammar recognizer stays on the robot for stop and the core commands, so voice still works with the PC off.
- **Always on the PC anyway:** training (LeRobot policies, fine-tuning the detector on logged images), map post-processing, session replay in the sim (`studies/replay_log.py`).

**How, if adopted.**
- One interface for the tidy's perception, `detect(images, pose) -> things`, with two backends: on the robot (today's) and an assist server on the PC.
- The robot asks with a deadline (e.g. 1 s) and uses its own answer when none comes, so the PC is an accelerator, never a dependency.
- Transport: plain HTTP (one JPEG pair + camera info + pose up, JSON with boxes, masks as polygons and depths down), or zenoh. Not DDS over Wi-Fi for images (fragmentation, multicast discovery) and not rosbridge (the console's rule: no images through it).
- The sim exercises it as is: its rendered frames go to the same server, so `studies/arm_pick.py` can score both backends on the same seeds before anything is bought.

**Measure first:** the house's Wi-Fi where the robot works (iperf3 throughput and a ping histogram while it drives); the PC's GPU and its time per look; the Orin's own detector speed. Status: analysis only, nothing built; the decision is Fernando's.

## 14. CAD v8: wheel forks, hex-drive heads, labels, stronger cheeks (Oct 26, proposed)

Fernando's review of `cad/v7` (Oct 26), for both robots (catch and arm): more detailed fasteners (hex-drive shallow pan heads, M3 head 6.5 × 2.5), every fastener magenta, the wheels supported on both sides (a single printed support will break), text labels on the green boxes and other non-obvious parts, the cheeks too thin where the pad turns into the web, both robots viewable as STEP in SolidWorks, and `cad/out` cleaned up. All of it is in `cad/v8` (README "v8"); `cad/v7` stays as the Oct 25 baseline the v11 sim reads.

**What changed** (ROBOT / POD frames, mm):
- **Fork round each wheel.** v7 hung the wheel, cup and motor off one printed arm. v8: a second 61808 on a boss on hub A (outboard of its end plate; the #1999's screws are driven through the boss's Ø31 bore), held by a printed **outer plate** (Ø62 housing with a 3 mm lip, a web over the wheel's outer face 1.9 mm off the rollers, a lug), joined to a **leg** on the arm by a **6061 bar 6 × 20 × 104** on edge under the deck (2 × M3 × 14 at each end, 8 mm of thread). The bar is on the pivot's side, where bump moves it down 6 mm and inward (on the far side a point would rise ~36 mm into the deck). The outer plate stood where the wheel guard's panel was, so **the deck, guards and bumpers grew 12 mm in radius: DECK_R 217, bumper Ø458 (v7 Ø434)**; the corner bumpers' side screws moved over the deck (z 72, 88).
- **Fasteners:** hex-drive shallow pan heads (M3 6.5 × 2.5, 2 mm hex, rounded top edge) in 7.2 × 2.9 counterbores; flat heads keep the ISO 7046 head with a 2 mm hex; chamfered nuts and shank tips, the nyloc's collar. The taller heads moved the STS3250 out 0.7 mm (room for its horn's screws), opened the chute's keel-screw bore, and shortened the arm adapter's screws to M3 × 18.
- **Magenta** fasteners (screws, nuts, washers, shims, set screws) in the STEP, renders and viewer.
- **Labels:** raised white text on the Jetson, power board, BMS, MCU + IMU, 19 V / 12 V bucks, ESC, RoboClaw, ELRS receiver, mic array, speaker, battery, lidar (`hw.label`, `Hardware.part(text=)`); cosmetic (no mass, BOM line or check).
- **Cheeks:** the module's sweep keeps the web low over the pad, so the cheek grows down and out: an 8 mm **jaw** under the plate's edge (z 154.5, 4.5 mm over the scan plane, ≥ 64 mm from the lidar's axis) that pad and jaw clamp like a C (the pad's 3 × M3 × 20 pass through the plate, now clearance holes, into the jaw), the web filled down past the edge, and 14 mm thick (|y| 64–78, was 8) below the bar's ring. The module still comes no closer than 6.2 mm; the launch line still clears from 24°. The RC receiver moved to cheek R's new outer face.
- **STEP of both robots:** `out/assembly.step` (catch) and `out/assembly_arm.step` (arm kit, bar at −40°, the SO-101 stowed: TheRobotStudio's own STEP of every part and the STS3215, vendored unmodified in `cad/v8/vendor/so101` (Apache-2.0, the sim's pinned commit; editable), placed at the sim's stow pose by `layout_trade.py arm-poses`). `view.py --arm`; the Launcher's CAD (v8) menu opens both.
- **`cad/` cleanup:** the v2 CAD moved from `cad/` into `cad/v2/` (its stale `cad/out` deleted; `cad/v2/build.py` regenerates it there). Each version keeps its own `out/`; `cad/README.md` indexes them.

**Checked** (`cad/v8/out/checks.json`, full build):

| Check | Result |
| --- | --- |
| Clashes: static, module at −30…+40°, wheels at bump (+15) and droop (−3), the bar's index swing +30 → −40° | 0 (catch robot); arm robot static 0 |
| Screws | 368 + the arm kit's 49: 0 short of thread; 378 + 49: 0 access / clamping problems; every print one valid solid |
| Walls under 1.2 mm | only v7's accepted spots (motor cup webs, wheel spokes' knife edges) |
| Module to the head; launch line | 6.2 mm (unchanged); clear from 24° up, 13.5 mm at 30° (unchanged) |
| Lidar | 22° blocked (unchanged) |
| SO-101's meshes against the arm robot (v8's meshes) | ≥ 14.2 mm at its poses and grasps, its base 9.8 mm (unchanged) |
| Mass | catch robot **8.82 kg**, CG (17, −1, 121) mm (v7 8.19 kg); arm robot 8.38 kg |
| STEP | `assembly.step` 710 instances / 140 parts (39 MB); `assembly_arm.step` 624 + the SO-101's 17 parts (46 MB) |

**Open / VERIFY:** three supports on one shaft (two 61808s and the motor's own bearings) are statically indeterminate: bench that the #1999 hub doesn't load the DM-2325's shaft (else a slip fit in the outer housing); buy 9 × 6808-2RS (was 5); the bigger bumper ring (Ø458) isn't in the sim's drawn robot (`robot_export.drawn_params` is v7's Ø434, 8.19 kg); v8's mass and CG are in its BOM. The decision to adopt v8 over v7 is Fernando's.

### 14.1 CAD v8 round 2: both-sides suspension, one place for the face, one electronics bay (Oct 27, proposed)

Fernando's review of the arm robot (Oct 27), applied to both robots in `cad/v8` (README "round 2"):
- the screen and speakers in the same place on both robots ("I am not going to be moving those"); cameras may reorient;
- bearings, and maybe shocks, on both sides of each wheel ("a cantilever wheel assembly is going to break"), thick parts;
- the electronics collocated in one area (one box or open, under the payload; kit electronics may stay on the kits with an easy connect);
- the bumper higher off the floor (it would catch a rug);
- no e-stop: a physical on/off switch (round) and the radio;
- a smaller diameter, if possible.

**What changed** (ROBOT / POD frames, mm):
- **Pods** (`pod.py`). v8's fork (inner plate, outer plate, the bar under the deck) pivoted on the inner side only. Now it swings on one **8 mm steel shaft** beside the wheel (POD y 68.2, z 30: 74.5 from the axle), in a **608 in each plate**, and the shaft is held by **an ear on each side of the wheel**, both part of the wheel guard (the inner one beside hub C pins it with an M3 through a cross hole; the outer one beside the rollers is cut back to r 65.5 from the axle). **Two shocks**, one each side: inner plate → the pod mount's tower (motion ratio 0.56), outer plate → a boss on the guard's front panel (0.47); one ~5.6 N/mm spring (est) gives the sim's 3 N/mm at the wheel; 14.8 mm of bump and 3 mm of droop in the stroke. The wheel still turns on a 61808 at each end. Inner plate 12 mm (was 8), outer plate 9 mm with a 10.5 mm housing (6.5), bar 10 × 20 (6 × 20). The pivot sits 30 mm above the axle, so the wheel moves 6 mm along its rolling direction over the full bump. The 623 pivots are gone (608s, kit size).
- **Face, speaker and mics: the face mast** (`mast.py`, `display.py`), the same on both robots. The front can't take them on the arm robot (the SO-101 comes down over the camera bar there, section 12), and the plate's rear can't on the catch robot (the funnel tilts back over it to x −193). So two 20 × 20 × 1.5 tubes (the camera bar's stock, 357 mm) stand in printed feet on the deck's rear edge at (−202.4, ±48), behind the funnel's sweep; a printed crown on their tops holds the 5 in face between two ears (front-bottom edge at (−187, 430), leaning back 6°) and the **audio cap** (mic array and speaker) on the ears. The face is over the funnel rim's strut sockets (420 mm at tilt 0), so the thrower sees it with the module level. **The robot is 0.54 m tall** (the funnel alone reached 0.49 m). The cheeks lost their horns, the arm kit its uprights and shelf; the cameras stay on the indexed bar.
- **Electronics bay** (`base.py`) along the deck's centreline under the plate: the Jetson on 22 mm posts with the **MCU carrier** (55 × 90: Teensy 4.1 + BNO085, the size section 15 asked for) under it; the 19 V and 5 V bucks on the rear splice; the power board (anti-spark switch, 40 A fuse, current sensor) on the rear lobe; the BMS under it by the battery's leads; the ELRS receiver on the mast's foot. Each kit keeps its own boards (catch: ESC, RoboClaw, 12 V buck, servo on the yokes; arm: 12 V buck, bus board) and plugs into **one connector panel** on the plate (`payload_ports`: XT30, Micro-Fit 3.0 2 × 6, USB-C).
- **On/off switch** (no e-stop): a round 22 mm latching push button with an LED ring on a riser at the rear edge. It switches the power board's anti-spark switch (its enable loop), not motor current.
- **Bumper**: rigid parts from 48 mm up (was 42), the TPU skirt from 34 mm (was 20). A 74 mm ball's centre is 37 mm up, so the skirt still pushes a missed ball instead of running over it.
- **Diameter: Ø462** (v8 Ø458, v7 Ø434). The both-sides pivot puts the outer plate's pivot boss beside the slot's corner (r 215), so the guard's panel moved out to r 218–223; the bumper went from 12 to 8 mm deep to give some of that back.

**Why not smaller.** The diameter is set by whatever stands outboard of the wheel at the slot's corner. A shaft held on both sides needs an ear and a boss there (y ≈ 70–82 at x ≈ −50 in the POD frame): r ≈ 215 at L 150. Moving the wheels in (`wheel_base_radius` 150 → 140) doesn't work with it: the two pods sharing a side lobe would put their pivot bosses through each other, and the inner plates would meet the battery tray. The compact alternative is v8's one-sided pivot made stiffer (the fork plus a roller in a curved track in the guard to catch the outer side): about Ø430 at L 140. That changes the sim's base and puts back a pivot on one side only, so it's Fernando's call.

**Checked** (`cad/v8/out/checks.json`, full build):

| Check | Result |
| --- | --- |
| Clashes: static, module at −30…+40°, wheels at bump (+15) and droop (−3), the bar's index swing | 0 (catch robot); arm robot static 0 |
| Screws | 388 + the arm kit's 21: 0 short of thread; 398 + 21: 0 access / clamping problems; every print one valid solid |
| Walls under 1.2 mm | only v7's accepted spots (motor cup webs, wheel spokes' knife edges) |
| Module to the head and the mast | 3.0 mm (the funnel rim's strut socket to a mast post at −24°); the face 7.5 mm over the rim's sockets |
| Launch line | clears the head from **8°** up (43 mm at 30°): the face no longer limits it (was 24°; the firmware's 25° floor can drop, not changed) |
| Lidar | 23.8° blocked (v8 22°): the posts add two thin wedges next to the rear standoffs' |
| SO-101's meshes against the arm robot | ≥ 18.8 mm at its poses and grasps (v8 14.2: the face over the bin is gone), its base 11.2 mm |
| Suspension | inner shock motion ratio 0.56, outer 0.47; ~5.6 N/mm each; bump 14.8 mm, droop 3.0 mm (the inner one's stroke) |
| Mass | catch robot **9.80 kg**, CG (−1, −1, 132) mm (v8 8.82 kg, CG 121); arm robot 9.18 kg, CG 120. The pods +0.7 kg (shafts, 608s, four more shocks, thicker plates and bar), the mast 0.6 kg |
| STEP | `assembly.step` 749 instances / 159 parts; `assembly_arm.step` 651 + the SO-101's parts |

**Open / VERIFY:** the shocks' spring (~5.6 N/mm, est) and stroke; the 8 mm shaft's fit in the ears (slip) and the 608s' inner races on it; the payload connectors' pinout against PINS.md (Micro-Fit 2 × 6: CAN3, 2 × DShot, Serial3, Serial4, beams, IR, 5 V) and their panel cut-outs; the switch's body (24 mm deck hole). The sim's drawn robot is still v7's (`robot_export.drawn_params`): round 2's mass (9.8 kg catch, 9.2 kg arm) and CG (131 mm up, was 121) aren't in it, nor the 0.54 m mast; the firmware's e-stop input (pin 35) becomes the switch's sense or goes unused, and `BUS_EN` (section 15) is now the only motor-bus cut besides the switch. The decision to adopt round 2 is Fernando's.

## 15. Electronics: block diagram, power and boards (Oct 27, proposed)

Fernando's todo: "block diagram of all components, do we need PCB?" Drawn from the BOM (`cad/v8/out/BOM.md`), the pin map (`firmware/PINS.md`), the actuator database, the history's §10 (power), §14 (safety) and §15 (payload connector), and new sim runs for the currents.

**Power** (⊘ = cut by the e-stop chain; the MCU and the Jetson stay up so they can report and log it):

```
7S1P P50B, 21.0-29.4 V (25.2 nominal)
 └ BMS 7S 40 A ─ XT60 ─ main fuse 40 A ─ hall current sensor (Teensy A15) ─ pack divider (A17)
    ├ LOGIC, always on
    │  ├ 19 V buck 4 A ─ Jetson Orin Nano Super dev kit (~22 W in play)
    │  └ 5 V buck ~2 A ─ MCU carrier: Teensy 4.1, BNO085 (3.3 V), ELRS receiver, IR beams, bumpers, buzzer
    └ ⊘ MOTOR BUS: soft-start switch, enabled by the e-stop chain (below)
       ├ 4 × DM-2325 (CAN1)                 sim: 8-19 A bursts to 2 m/s
       ├ ESC 8S 4-in-1 (AM32) ─ 2 × F90     spin-up current set by AM32's limit
       ├ 12 V buck 5 A ─ RoboClaw (2 × Pololu 25D feed) + STS3250 tilt     catch kit, on its yokes
       └ payload connector: 24 V + CAN3 + USB ─ arm kit: 12 V buck ─ SO-101 bus board (USB to the Jetson)
```

**Signals:**

| Link | Between | Notes |
| --- | --- | --- |
| USB (COBS link, protocol 4) + SYNC_OUT to a 40-pin GPIO | Jetson ↔ Teensy | section 10 |
| 2 × 22-pin CSI | Jetson ↔ 2 × OV9782 | the dev kit has two CSI connectors; FSIN and STROBE are separate wires from the Teensy (pins 24, 26), FSIN at 1.8 V (VERIFY) |
| USB (Slamtec's UART adapter) | Jetson ↔ A2M12 | or the 40-pin UART, to free a USB port |
| DisplayPort + USB touch | Jetson ↔ 5 in display | the dev kit has DisplayPort, not HDMI: a DP→HDMI adapter (VERIFY) |
| USB | Jetson ↔ mic array (its amp drives the speaker) | |
| USB × 2 | Jetson ↔ arm kit: bus board (6 × STS3215), wrist camera | |
| CAN1, 1 Mbit/s | Teensy ↔ 4 × DM-2325 | transceiver on the carrier |
| CAN3 (FD) | Teensy ↔ payload connector | module id |
| 2 × bidirectional DShot | Teensy → ESC | 3.3 V is fine for AM32 |
| Serial3 packet serial | Teensy ↔ RoboClaw | |
| Serial4 half duplex | Teensy ↔ STS3250 | the bus's logic level and a buffer: VERIFY |
| Serial2 CRSF | Teensy ↔ ELRS receiver | 5 V power |
| SPI0 + INT, RST, WAKE, BOOTN, PS1 | Teensy ↔ BNO085 | on the carrier, under the deck's rear lobe |
| GPIO / ADC | Teensy ↔ 2 beam receivers, IR emitters (38 kHz), front / rear bumpers, e-stop sense, buzzer, LED, pack V / I, 12 V sense | 34 of 42 pins used, 8 spare |

**Do we need a PCB? Yes, one: the MCU carrier. The power side can wait.**
- **Why the carrier.** The Teensy 4.1 has CAN controllers but no transceivers, and two are needed (CAN1, CAN3 FD). The camera trigger needs a level shifter to 1.8 V. The IR emitters, the buzzer and the motor-bus enable need MOSFETs. The bumper, beam and e-stop lines run 20–30 cm to the deck's edge and the module, so they want pull-ups, RC filters and ESD protection. The pack sense needs a divider and filter, and the board needs 5 V and 3.3 V. On perfboard that is ~150 hand-soldered joints and ~14 cables in a robot that shakes over rug edges.
- **Why it's low risk now.** The pin map is fixed and checked (`static_assert`s, PINS.md).
- **The board:** 2 layers, ~90 × 55 mm (est). The Teensy is socketed on headers and the Adafruit BNO085 (#4754) plugs on. Each cable has its own keyed JST-GH connector. KiCad, made at a JLCPCB-class shop: ~$20 for five boards plus ~$30 of parts (est).
- **For the bench before the board:** a Teensy and two SN65HVD230 CAN breakouts on a breadboard are enough for next step 3 (one DM-2325).
- **Power side: off the shelf for the first build.**
  - XT60, a 40 A fuse holder, a hall current-sensor breakout, XT30 pigtails or a small 8S distribution board, the two bucks.
  - A **soft-start MOSFET switch with an enable loop** ("anti-spark switch", ≥ 8S, ≥ 40 A). The loop runs through the mushroom's NC contact and the Teensy's enable.
  - Soft start matters because the four DM-2325 drivers and the 4-in-1 ESC carry input capacitors. A plain relay closing on them at 29 V sparks and pits its contacts.
  - Fold the power side into the BOM's "power board" PCB after the bench, once the parts and connectors are settled.
- **Sim currents** (`drawn_params`, 8.19 kg, open-loop ramps to 2 m/s):
  - 8.4 A straight at 2.5 m/s² on a full pack;
  - 19.2 A diagonal at 4 m/s² at 10% charge;
  - idle electronics 0.76 A at 29 V (`electronics_power` 22 W).
  - The 40 A fuse and BMS have margin. The flywheels' spin-up adds to a drive burst, so set AM32's current limit to keep the sum under ~35 A (est).

**Found on the way** (this section's VERIFY list):
- **The DM-2325's minimum supply.**
  - The database lists 24 V as the minimum (reseller data). A 7S pack is under 24 V below ~3.43 V per cell.
  - The sim at 10% charge: 23.97 V at rest, 23.1 V in a 4 m/s² burst. The firmware's low-battery fault is at 22.4 V (`BATT_LOW`).
  - If 24 V is a real undervoltage cutoff, the drive quits before the firmware warns, with roughly the last 10–15% of the pack left (est).
  - Bench it (next step 2 already lists it). If it is real: an 8S pack (the ESC is already 8S, the DM-2325 is listed to 48 V, the bucks take it), or raise `BATT_LOW`.
- **The 19 V buck's headroom:** about 3 V at 22.4 V and about 2 V on a flat pack, at a buck's 1–2 V dropout. VERIFY the dev kit's DC input range; use a lower output if it accepts one.
- **USB ports:** the catch robot already uses the dev kit's four USB-A ports (Teensy, lidar, mic array, touch). The arm kit adds two (bus board, wrist camera), so it needs a hub, or the lidar moved to the 40-pin UART.
- **A motor-bus enable from the MCU.**
  - The pin map has the e-stop chain's sense (pin 35) but no output into the chain.
  - The history's §14 wants the RC e-stop "wired to the same power cutoff". The ELRS receiver speaks only CRSF, so the Teensy has to do that.
  - Proposed: a `BUS_EN` on a spare pin (0, 1, 16, 18–21, 34 are free), in series with the mushroom. The firmware opens it after the ramped stop of ESTOP, an RC failsafe or the watchdog. The button still cuts the bus with no code involved.
  - The arm kit's 12 V is on the switched bus, so the arm drops on an e-stop (open item, section 12.3).
- **Board space in CAD:** v8 draws the MCU board as 40 × 30 mm under the deck (an STM32 guess; its BOM text now says Teensy) and the power board as 60 × 38 mm on the deck. A Teensy carrier is ~90 × 55 mm, and the off-the-shelf power parts need a tray. Reserve the space before printing the deck.

**Round 2 (Oct 27, section 14.1):** Fernando dropped the e-stop button: a round on/off switch enables the power board's anti-spark switch, and the radio stops the robot through the MCU. In the tree above, "⊘" is now `BUS_EN` (the firmware's ramped stop, an RC failsafe, the watchdog) plus the on/off switch for everything; pin 35 (the e-stop sense) can read the switch. CAD v8 round 2 reserves the carrier's 55 × 90 under the Jetson and puts the power side in one bay on the rear lobe.

Status: analysis only, nothing changed in the firmware, sim or CAD geometry (only the MCU board's BOM text). The decisions are Fernando's: the carrier PCB, the off-the-shelf power side, `BUS_EN`, and 7S vs 8S after the bench.
