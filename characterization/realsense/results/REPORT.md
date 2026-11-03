# RealSense D435 characterization report

Generated 2026-10-29 19:15 by `rs_report.py` from `results/`. Measured on the bench, not the robot; datasheet values are marked so. The design's requirements: design.md 4.1, 4.9 and section 8 (phase 1C).

## Device

- RealSense D435, serial 827312072573, firmware 5.17.3.10 (recommended None), USB 2.1, librealsense 2.59.1
- IR baseline 49.85 mm (calibration; datasheet 50); infrared1_848x480 fx 424 px FOV 90.02x59.03
- Timestamps: depth Frame didn't arrive within 8000, ir Frame didn't arrive within 8000, color Frame didn't arrive within 8000

## Against the design's camera requirements

| Check | Result |
| --- | --- |
| ir_pair: global shutter | PASS (OV9282, datasheet) |
| ir_pair: colour | FAIL: mono (the robot's colour-blob detector would need a shape / motion detector) |
| ir_pair: >= 60 fps at >= 640x400 | FAIL |
| rgb: global shutter | FAIL: rolling shutter (OV2740, datasheet); the sweep put rolling 20 ms at 72% |
| rgb: >= 60 fps at >= 640x400 | FAIL |
| hw sync (inter_cam_sync_mode) | offered, range 0.0..4.0: VERIFY a slave mode follows an external 1.8 V pulse |
| usb3 | FAIL: The D435 needs USB 3 for its fast modes. If it shows up as USB 2.x (or as PID 0AD6 in Device Manager with Code 10): use the cable it came with, a USB 3 port straight on the PC (not a hub, not a front-panel USB 2 port), and try flipping the USB-C plug at the camera end (a known D435 quirk). |
| latency <= 60 ms | measure: rs_timing.py (to host), rs_flash_latency.py (photons to host) |

The catch's colour-blob detector needs colour + global shutter in one camera; the D435 has global-shutter mono (IR pair) or rolling-shutter colour, not both. Uses that fit as it is: the arm's floor depth, low obstacles under the lidar plane, a bench reference for the OV9282 sensor family the design's OV9782 belongs to.

## Values to carry over (by hand, with their source)

- nothing measured yet

Rule (design.md 8): a value goes into the database and the sim with its source, then the scoring study is rerun.
