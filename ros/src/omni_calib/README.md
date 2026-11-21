# omni_calib: the calibration workflow (design.md 10)

Every calibration the robot depends on, in the order to do them, what to record, how it is solved and checked, and
what a run must reach before it is used. The solvers are ROS-free (`omni_calib/*.py`) and tested on synthetic data
in the robot's own geometry (`test/test_calib.py`). Results go to the store (`results.py`, default
`~/.omni/calibration`, one JSON per item, every run kept); `calib_report` prints them. Nothing is applied by itself:
`robot_frames` takes a frame only from a passed run and only within its bounds of the drawing (2 cm, 3°:
`omni_bringup/frames.py`), and the intrinsics go to the camera node.

Limits below are (est) until the bench shows what the parts can do (`results.LIMITS`).

| # | Item | Target / motion | Solver | Checks (limit) |
| --- | --- | --- | --- | --- |
| 0 | **Clocks** (Teensy ↔ robot computer) | nothing: runs all the time | `omni_mcu/clock_sync.py`: line through the 1 Hz SYNC_OUT pulses | `/mcu/clock`: mode `pulse`, residual RMS < 50 µs, drift steady; the echo-minus-pulse offset is the USB path's asymmetry (record it) |
| 1 | **Camera intrinsics** (each OV9782) | AprilGrid 6×6 (Kalibr's, 88 mm tags at A0; caliper the print) or a 9×6 checkerboard; 30–60 views, tilted ±40°, corners into every part of the image | `cameras.calibrate_camera`: Zhang's closed form, then all intrinsics + distortion (k1 k2 p1 p2, k3 held) + poses, robust | reprojection RMS < 0.5 px, coverage ≥ 60% of an 8×5 grid; per-view outliers dropped and redone |
| 2 | **Stereo extrinsics** | the same target, both cameras at one trigger (the hardware trigger guarantees the same instant) | `cameras.calibrate_stereo`, intrinsics held | RMS < 0.5 px, **epipolar error < 0.3 px**, rotation σ < 0.02°; baseline vs the drawing's 190 mm; **session-to-session drift** (`Store.drift`): > 0.1° = the bar moved (the catch rate's biggest risk, design.md 3) |
| 3 | **Camera timing** | the strobe on the scope (bench), then nothing | `omni_perception/camera_time.py` locks frames to triggers | SOF jitter, stereo skew (strobes), dropped frames: `FrameStamper.report()` |
| 4 | **Camera ↔ IMU** (time offset, rotation) | the target in view, the robot wiggled (±2 rad/s yaw at ~1 Hz) and rocked by hand for 20 s | `temporal.time_offset` on \|ω\| (camera from target poses, `rates_from_rotations`; gyro), then `temporal.rotation_from_rates` | offset σ < 0.5 ms and *observable* (a constant spin is refused), rotation RMS; the conditioning says whether roll / pitch were excited |
| 5 | **LiDAR ↔ camera** | a 0.8×0.6 m board standing across the scan plane, 1–2.5 m away, turned ±35° and leaned ±30°, 12+ poses | `extrinsics.lidar_to_camera`: point-to-plane, a weak prior on the drawing | plane RMS < 15 mm; the weakest direction's σ; **projected lidar points land on the board** in the image (`lidar_in_image` ≥ 90%) |
| 6 | **LiDAR timing** | a spin with changing rate (as 4) | `temporal.time_offset` of scan-matched yaw rates against the gyro → `scan_prep latency`; `ScanClock` on the driver's stamps | offset σ < 1 ms; ScanClock's jitter |
| 7 | **Sensors on base_link** | drive 10 short legs with turns in a mapped room | `extrinsics.hand_eye_se2` (lidar moves from scan matching vs wheel odometry) | residual < 1 cm, yaw mismatch < 1°; the IMU's level is the console's Calibrate › IMU level |
| 8 | **Dock tag ↔ contacts** | dock 8 times (each: the contacts close, the camera sees the tag) | `extrinsics.dock_tag` | spread < 5 mm, < 1°; the result is `tag_slam`'s `dock_tag_to_contacts` |
| 9 | **SO-101 ↔ D435** (arm robot) | a printed 40 mm cube with tag36h11 30 mm on three faces (`arm_calib.held_cube`), keyed to the gripper if possible; 60+ still arm poses in the D435 colour image, each approached from the same side | `arm_calib.calibrate_arm`: joint zeros (lift, elbow, wrist; + roll with a keyed block), each link's pose, sag past the encoders (with a gravity model), the camera on the arm and the block in the gripper, all at once on corner reprojection, held near the drawing by priors | rms < 1 px; **`tool_sigma_mm` < 1** (the fit's own uncertainty of the tool at the tidy's floor grasps, in the camera's frame); `apply_to_config` writes a robot_arm json `omni_arm` uses as it is (`config:=<path>`) |
| 10 | **The arm's load** (design.md 4.5): what each servo's duty / current means in torque, the links' masses, the joints' true zeros and sag | the console's Calibrate › Arm: load and gravity; arm_brain runs it (`cal_load`): ~44 poses each approached from both sides, constant-speed sweeps of every joint, the moves between them; two weights (~150 g, ~300 g on a string over the fixed finger: kitchen scale, calipers) held at ~10 poses each; 12 fingertip heights read with a ruler (4 with the heavier weight); one luggage-scale pull as a check. ~10 min, the arm on the robot (gravity from the IMU) | `arm_load.fit`: linear least squares with bounds, priors on the official masses (Huber): per joint gain, back-EMF, rotor inertia, offset, friction moving and still; the links' masses and centres; pan and roll held near the loaded joints' servo (gravity can't scale them); the current model if the servos report one and it does better; then the zeros, sag and the floor's height from the ruler; thresholds from the held-out and moving scatter | `results.LIMITS["arm_load"]` (est): still torque on held-out holds < 0.15 N m rms, a held-out weight's force < 1 N, leave-one-out heights < 2.5 mm. A passed run is written to `load_model_path` (`<state>/arm_load_model.json`) and the driver rereads it |

**Touch blocks (a proposal, `arm_datum.py`, Nov 20):** removable blocks beside the arm's adapter the fixed fingertip touches slowly (3 mm/s, 30 % torque); where the joints fall behind the goal is the touch. On the detailed sim arm (`simulation/v11 sil/studies/arm_datums.py`) they pin pan's zero (which no height sees) to 0.1-0.5°; wrist flex, roll and sag are poorly seen that close to the arm's base, so only pan's zero is taken from them (`fit(..., free=(0,))`): reach 4.3-6.2 → 1.8-4.5 mm on 3 seeds.

**Item 10, what it can and can't pin** (the detailed sim arm, `simulation/v11 sil/studies/arm_load_cal.py`): gravity
fixes each loaded joint's gain only with a known weight (otherwise masses and gains scale together); pan (its axis
vertical) and roll (hardly loaded) get no scale at all and borrow the loaded joints'. Masses trade between neighbouring
links (only their torques are seen); the ruler's dozen heights pin the heights to ~1 mm but fold sag into the zeros. A
still reading lies anywhere in the gearbox's friction band, so still holds are taken from both sides. Not covered:
heating (the case temperature stands in for the winding's), the servos' slow drift, a door's or a drawer's own load.

**Item 9, what the views can and can't pin** (synthetic and sim: `test/test_arm_calib.py`,
`simulation/v11 sil/studies/arm_calib.py`):
- **Seen only together:** pan's zero and the camera turned about the pan axis; roll's zero and the block turned about
  the roll axis. The calibration places the tool relative to the camera (what a pick uses), not the arm's heading on
  base_link (the lidar's job). So the solver leaves the pan zero at the drawing, and the roll zero too unless the block
  is keyed.
- **Effective zeros:** the lift, elbow and wrist zeros trade exactly against a link shifted across the arm, so their
  numbers are effective zeros in the drawing's links; judge the fit by `tool_sigma_mm` and a floor touch test.
- **A gripped block** leaves the fingers' place past the wrist at the drawing (tool sigma ~2 mm). A block keyed to the
  gripper's faces pins it (~0.3 mm).
- **Not covered:** the base's creep between the look and the grasp, depth error, servo droop and deadband (settle and
  correct on the measured angle first), encoder nonlinearity.

**Validation in use** (not a calibration step, but what keeps the results honest): the tag graph's corner residuals
(`/localization/health`, `tag_rms_px`), the online camera-rotation refinement (`omni_slam.graph`, below) and its
distance from the calibration, the stereo epipolar error on live ball tracks, the lidar's floor-return rate
(`scan_prep`), and motion tests (Calibrate › odometry line and spin).

## Online refinement, bounded

Only parameters that move in use, only when the data can see them, never past a bound (`online.BoundedParam`,
`omni_slam.graph` for the camera's mount):

* the camera's mount rotation in the tag graph: on only when the lidar fixes the map (otherwise a camera rotation
  and a map rotation look alike), prior 0.5°, bound 1°; at the bound it says so (recalibrate the bar);
* the camera / lidar time offsets: a random walk of ~10 µs/√s, refused samples counted; half the recent ones refused
  = inconsistent (a knock, a loose connector): the health slows the robot and asks for the bench.

Mechanical flex that static calibration can't fix (the printed bar under the module's tilt, the sprung base rocking):
the IMU's roll and pitch go into every camera observation (the graph uses them; the catch tracker models the
calibration error as a slowly varying bias: design.md next steps); the bar's stiffness is the design's answer.

## Detection on the robot

`detect.py`: checkerboards with OpenCV's `findChessboardCornersSB`; AprilTags with the C apriltag library
(`pupil_apriltags`, since Nov 12; the `tag_detector` node uses it at decimate 2), else OpenCV's aruco module with
apriltag corner refinement. Corners are apriltag's order, counter-clockwise from the bottom left of the **official**
tag images. OpenCV's AprilTag dictionaries are those tags turned 180°, so its raw corners are reordered
(`OPENCV_TO_APRILTAG`; before Nov 12 the reorder was right only for OpenCV's own marker images, and on a printed
official tag the pose was rolled 180° about the normal). Print tags from the official images (AprilRobotics
`apriltag-imgs`), not from OpenCV's `generateImageMarker`; `test/test_detect.py` checks both backends.
Record sessions as rosbags of `/cam/*/image_raw`, `/imu`, `/scan`, `/odom`, `/mcu/time_ref`; a calibration script
reads the bag, detects, and calls the solver; the store keeps the result with its metrics.
