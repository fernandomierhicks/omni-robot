# RealSense D435 characterization report

Generated 2026-10-30 08:27 by `rs_report.py` from `results/`. Measured on the bench, not the robot; datasheet values are marked so. The design's requirements: design.md 4.1, 4.9 and section 8 (phase 1C).

## Device

- RealSense D435, serial 827312072573, firmware 5.17.3.10 (recommended None), USB 3.2, librealsense 2.59.1
- IR baseline 49.85 mm (calibration; datasheet 50); infrared1_848x480 fx 424 px FOV 90.02x59.03
- Timestamps: depth global_time, ir global_time, color global_time

## Against the design's camera requirements

| Check | Result |
| --- | --- |
| ir_pair: global shutter | PASS (OV9282, datasheet) |
| ir_pair: colour | FAIL: mono (the robot's colour-blob detector would need a shape / motion detector) |
| ir_pair: >= 60 fps at >= 640x400 | PASS: 640x480@60, 640x480@90, 848x480@60, 848x480@90 |
| rgb: global shutter | FAIL: rolling shutter (OV2740, datasheet); the sweep put rolling 20 ms at 72% |
| rgb: >= 60 fps at >= 640x400 | PASS: 640x480@60, 640x480@60, 640x480@60, 640x480@60 |
| hw sync (inter_cam_sync_mode) | offered, range 0.0..4.0: VERIFY a slave mode follows an external 1.8 V pulse |
| usb3 | PASS |
| latency <= 60 ms | measure: rs_timing.py (to host), rs_flash_latency.py (photons to host) |
| capture -> host (rs_timing, best IR mode) | PASS: stereo:848x480@90 median 9.6, p95 9.7 ms of 60 (detection comes on top) |

The catch's colour-blob detector needs colour + global shutter in one camera; the D435 has global-shutter mono (IR pair) or rolling-shutter colour, not both. Uses that fit as it is: the arm's floor depth, low obstacles under the lidar plane, a bench reference for the OV9282 sensor family the design's OV9782 belongs to.

## Timing per mode (rs_timing.py)

| Mode | Stream | fps | jitter ms | lost | exposure us | capture -> Python ms (median / p95 / max) | CPU cores |
| --- | --- | --- | --- | --- | --- | --- | --- |
| depth:848x480@90+color:640x480@60 | color | 59.5 | 0.00 | 0 | 10 | 17.6 / 17.8 / 18.0 | 0.28 |
| depth:848x480@90+color:640x480@60 | depth | 89.9 | 0.00 | 0 | 1000 | 15.4 / 22.8 / 23.6 | 0.28 |
| stereo:848x480@90 | infrared1 | 89.9 | 0.00 | 0 | 1000 | 9.6 / 9.7 / 9.8 | 0.21 |
| stereo:848x480@90 | infrared2 | 89.9 | 0.00 | 0 | 1000 | 9.6 / 9.7 / 9.8 | 0.21 |

## Values to carry over (by hand, with their source)

- `database/cameras.csv` (a new `intel-realsense-d435` row): `meas_latency_ms` = 9.6 (capture -> host, median; p95 9.7)
- sim `vision.py` VisionConfig: `latency_mean` = 0.0096 s + the detector's time; `latency_jitter` = 0.0000 s (camera part only)
- `meas_fps`: stereo:848x480@90 89.9, stereo:848x480@90 89.9
- dropped frames (sim `drop_prob` floor): worst depth:848x480@90+color:640x480@60 0 of 595

Rule (design.md 8): a value goes into the database and the sim with its source, then the scoring study is rerun.
