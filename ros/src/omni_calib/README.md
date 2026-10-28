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
| 0 | **Clocks** (Teensy ↔ Jetson) | nothing: runs all the time | `omni_mcu/clock_sync.py`: line through the 1 Hz SYNC_OUT pulses | `/mcu/clock`: mode `pulse`, residual RMS < 50 µs, drift steady; the echo-minus-pulse offset is the USB path's asymmetry (record it) |
| 1 | **Camera intrinsics** (each OV9782) | AprilGrid 6×6 (Kalibr's, 88 mm tags at A0; caliper the print) or a 9×6 checkerboard; 30–60 views, tilted ±40°, corners into every part of the image | `cameras.calibrate_camera`: Zhang's closed form, then all intrinsics + distortion (k1 k2 p1 p2, k3 held) + poses, robust | reprojection RMS < 0.5 px, coverage ≥ 60% of an 8×5 grid; per-view outliers dropped and redone |
| 2 | **Stereo extrinsics** | the same target, both cameras at one trigger (the hardware trigger guarantees the same instant) | `cameras.calibrate_stereo`, intrinsics held | RMS < 0.5 px, **epipolar error < 0.3 px**, rotation σ < 0.02°; baseline vs the drawing's 190 mm; **session-to-session drift** (`Store.drift`): > 0.1° = the bar moved (the catch rate's biggest risk, design.md 3) |
| 3 | **Camera timing** | the strobe on the scope (bench), then nothing | `omni_perception/camera_time.py` locks frames to triggers | SOF jitter, stereo skew (strobes), dropped frames: `FrameStamper.report()` |
| 4 | **Camera ↔ IMU** (time offset, rotation) | the target in view, the robot wiggled (±2 rad/s yaw at ~1 Hz) and rocked by hand for 20 s | `temporal.time_offset` on \|ω\| (camera from target poses, `rates_from_rotations`; gyro), then `temporal.rotation_from_rates` | offset σ < 0.5 ms and *observable* (a constant spin is refused), rotation RMS; the conditioning says whether roll / pitch were excited |
| 5 | **LiDAR ↔ camera** | a 0.8×0.6 m board standing across the scan plane, 1–2.5 m away, turned ±35° and leaned ±30°, 12+ poses | `extrinsics.lidar_to_camera`: point-to-plane, a weak prior on the drawing | plane RMS < 15 mm; the weakest direction's σ; **projected lidar points land on the board** in the image (`lidar_in_image` ≥ 90%) |
| 6 | **LiDAR timing** | a spin with changing rate (as 4) | `temporal.time_offset` of scan-matched yaw rates against the gyro → `scan_prep latency`; `ScanClock` on the driver's stamps | offset σ < 1 ms; ScanClock's jitter |
| 7 | **Sensors on base_link** | drive 10 short legs with turns in a mapped room | `extrinsics.hand_eye_se2` (lidar moves from scan matching vs wheel odometry) | residual < 1 cm, yaw mismatch < 1°; the IMU's level is the console's Calibrate › IMU level |
| 8 | **Dock tag ↔ contacts** | dock 8 times (each: the contacts close, the camera sees the tag) | `extrinsics.dock_tag` | spread < 5 mm, < 1°; the result is `tag_slam`'s `dock_tag_to_contacts` |

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

`detect.py` (OpenCV: `findChessboardCornersSB`, the aruco module's AprilTag 36h11 with apriltag corner refinement).
Record sessions as rosbags of `/cam/*/image_raw`, `/imu`, `/scan`, `/odom`, `/mcu/time_ref`; a calibration script
reads the bag, detects, and calls the solver; the store keeps the result with its metrics.
