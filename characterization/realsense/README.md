# RealSense D435 characterization

Bench software that measures an Intel RealSense D435 against what `design.md` asks of the robot's cameras. Nothing here is robot code: it produces numbers (`results/`), and those go into `database/cameras.csv` and the sim by hand, with their source (design.md 8, rule 2).

**Double-click `RealSense.cmd`** for a window with one button per step. The commands below do the same from a terminal.

## Why the D435, and for what

The design's catch camera is a stereo pair of colour, global-shutter OV9782s (1280×800, 120 fps, MCU-triggered) on a 190 mm aluminium bar (design.md 2). Perception limits the catch (design.md 4.1): latency, frame rate, and above all the pair's calibration stability (0.04° → 97%, 0.3° → 78%, 0.5° → 58%). The D435 is relevant three ways:

| Use | What the D435 brings | Steps |
| --- | --- | --- |
| **Catch camera, as a reference** | Its depth imagers are OV9282s, the mono twin of the OV9782: global shutter, 1280×800, 3 µm pixels, on a factory-calibrated 50 mm pair. What latency, fps, drops and blob-stereo noise does this sensor family give over USB, and what would the same pixel noise do on the design's 190 mm bar? | `rs_timing`, `rs_flash_latency`, `rs_ball`, `rs_throw` |
| **Catch camera, as is** | Against `compute_choice.py`'s CAMERA_REQ it fails up front: the IR pair is mono (the colour-blob detector needs colour) and the RGB camera is rolling shutter (OV2740). `rs_info` makes that explicit; the measured steps say how far off the rest is (dense depth on a 2–3 px ball at 6–7 m, latency, sync) | `rs_info`, `rs_ball`, `rs_throw` |
| **Calibration drift** | A first real number for the design's biggest unknown: how much a rigid, factory-calibrated stereo pair moves while warming up, from day to day and after knocks, in the design's own unit (degrees of relative yaw) | `rs_drift` |
| **Arm floor vision, low obstacles** | Dense depth at 0.3–1.5 m: the tidy's floor objects at 0.26–0.42 m from the centre, things under the lidar's 150 mm plane, the camera's pose from the floor plane (design.md 4.1 recovers the bar's pose that way) | `rs_depth_plane --floor` |

## Setup (done once)

- Python: its own venv, `C:\Users\ferna\.venvs\realsense` (made from the same 3.11 as `omni-robot-sim`; kept apart because opencv-python pins numpy). Rebuild it with:
  `& C:\Users\ferna\.platformio\python3\python.exe -m venv C:\Users\ferna\.venvs\realsense` then `...\realsense\Scripts\python.exe -m pip install -r requirements.txt`.
- **USB 3.** The D435 needs a USB 3 port straight on the PC (rear panel), with the cable it came with. If it appears as `VID_8086&PID_0AD6` with Code 10 in Device Manager, or `rs_info` reports USB 2.x: another port or cable, and flip the USB-C plug at the camera end (a known D435 quirk).
- **Metadata (hardware timestamps).** Run `enable_metadata.ps1` once as administrator with the camera working, then replug it (or the launcher's "Enable metadata (admin)"). Without it frames are stamped on arrival at the host, and the latency can't be split into its parts. `rs_info` says which you have (`global_time` = good, `system_time` = metadata off).
- **Firmware.** librealsense 2.59 wants 5.17.3.10 or later on a D435. This unit came with 5.9.2 (2018), which shows up as "RealSense USB2", PID 0AD6, with a handful of slow modes; `RealSense.FW.Update.exe` mistook it for a GMSL camera and couldn't update it, so `rs_fw_update.py <image.bin> --flash` does the steps through pyrealsense2 (attended; the image from the [D400 firmware page](https://dev.realsenseai.com/docs/firmware-releases-d400)). Done Oct 29, 2026: 5.9.2 → 5.17.3.10, with the old flash backed up to `C:\Users\ferna\tools\realsense-fw-5.17.3.10\backup_827312072573_fw5.9.2.bin` (the RealSense tools 2.59.1 are in `C:\Users\ferna\tools\realsense-2.59.1`).
- **Status Oct 29, 2026: on the PC the camera links only at USB 2.1 and streams almost nothing.** Three cables, three rear ports and both plug orientations: control works (firmware, options, calibration), frames don't (0 in 3 s at 480×270 @ 30; a few at 6 fps), and Windows' own Media Foundation gets none either.
- **Status Oct 30, 2026: the same camera (serial 827312072573, firmware 5.17.3.10) on the Pi 4's blue USB 3 port links at USB 3.2 (5000 Mb/s) and streams every mode.** So the camera and its cable are fine and the PC's ports are the suspect. The steps run on the Pi through `rs_pi.sh` (next section).
- Results go to `results/` (JSON, PNG, `REPORT.md`; small, kept in git). Throw recordings (~110 MB/s) go to `%USERPROFILE%\realsense_recordings`, outside Dropbox.

## The steps, in order

Use `$py = "C:\Users\ferna\.venvs\realsense\Scripts\python.exe"`. Run everything from this folder.

| # | Command | Setup | What it measures | Against |
| --- | --- | --- | --- | --- |
| 1 | `& $py rs_info.py` | Plugged in | Device, firmware, USB link, every mode, intrinsics and FOV, the IR baseline from the calibration, sync / emitter / exposure options, timestamp domain and metadata; the static requirement checks | CAMERA_REQ: global shutter, colour, ≥ 60 fps at ≥ 640×400, hardware sync |
| 2 | `& $py rs_timing.py` (`--mode ir:848x480@90 --exposure-us 1000`) | Anything in view | Per mode: achieved fps, frames lost (frame counter), interval jitter, exposure, **mid-exposure → Python latency** (and its split: to host arrival, then to the callback), CPU of this process | ≤ 60 ms capture → detection (design.md 4.9; sims assume 45–50 ms) |
| 3 | `& $py rs_flash_latency.py` (`--mode color:640x480@60`) | The camera 20–50 cm from this monitor, the full-screen window filling its view | Photons → host from outside: per flip, the display's lag (camera stamp − command) and the camera's (arrival − stamp). A negative display lag means the timestamps are wrong | The same 60 ms; checks step 2's clock chain. With hardware, the Teensy's LED replaces the screen (design.md 8, 1C) |
| 4 | `& $py rs_depth_plane.py` | Square to a flat matte wall at 0.5, 1, 2, 3, 4, 6 m; SPACE, then the tape distance (front glass to wall) | Fill rate, spatial noise (plane-fit RMS, Intel's metric), temporal noise, bias against the tape, the **subpixel disparity noise** this implies, and from it the noise at 6–7 m on the D435 and on the design's bar | Datasheet ≤ 2% at 2 m; the sim's stereo noise |
| 4b | `& $py rs_depth_plane.py --floor` | Looking down at the floor ~40° from ~0.3–0.5 m, a few small things on it | The camera's height, pitch and roll from the floor and their noise per frame; the objects taller than `--min-height-mm` | Pose from the floor (design.md 4.1); tidy objects, low obstacles |
| 5 | `& $py rs_ball.py` (`--ball-mm 74`, `--mode 1280x720`) | A ball on a stand at 1–7 m; click it, SPACE, the tape distance to its front surface | Dense depth on the ball (fill, centre depth, noise, bias) with the emitter on and off; **blob stereo** (the robot's method: the ball's disc matched across the IR pair): disparity noise in px, depth noise and bias, and that pixel noise on the design's bar | The sim's centre noise (`vision.py` `sigma_c`, `edge_sigma`) |
| 6 | `& $py rs_throw.py record --preview --analyze` | Camera still at robot height, tilted up ~30°; throws toward it from 2–7 m; the first second empty | Per throw: first range seen, frames with the ball, residual against a constant-acceleration fit, blur, and \|a\| (≈ g if the clocks and depth scale are right) | The honest track start (> 1.5 m/s), the tracker's measurement noise |
| 7 | `& $py rs_drift.py --minutes 60` (`--label "day 2"`) | Clamped rigidly, facing a wall or the floor 1.5–3 m away, **started cold** (unplugged 30 min). Repeat on other days and after knocks, same setup | Plane distance and normal every 15 s, with ASIC and projector temperatures; depth drift turned into the pair's **equivalent relative yaw** (B·Δz / z²) | Phase 1C's pass band ≲ 0.1°; the sim's 0.04° |
| 8 | `& $py rs_report.py` | | `results/REPORT.md` + plots: every check against the requirements, and the values to copy into `cameras.csv` (`meas_latency_ms`, `meas_fps`, `meas_calib_drift_deg`) and the sim (`VisionConfig.latency_mean / latency_jitter / calib_rot_deg / drop_prob`, `randomize.py` `cam_latency`, `cam_calib_deg`) | |

Tests (no camera; synthetic scenes with known answers): `& $py -m pytest -q`.

## On the Pi (rpi.md): does it see the camera, and what does it deliver

The Pi 4 `omni-cam` (also the camera-head rehearsal, `characterization/pi_camera`) holds the D435 on a blue USB 3 port. From Git Bash, in the repo:

| Command | What |
| --- | --- |
| `bash characterization/realsense/rs_pi.sh setup` | Once (done Oct 30): venv `~/.venvs/rs` with `pyrealsense2` 2.59.1, the PyPI aarch64 wheel (V4L backend; no librealsense build). Nothing else to install: the user is in `video` / `plugdev`. |
| `rs_pi.sh usb` | Is it on the bus and at what speed (`lsusb -t`: 5000M = USB 3), `dmesg`, power (`vcgencmd get_throttled`). No librealsense needed. |
| `rs_pi.sh check [--seconds S] [--mode M]...` | **The one-screen check** (`rs_pi_check.py`): the link speed from sysfs, the camera through librealsense, the Pi's temperature / throttling, then each catch-relevant mode with every frame's pixels read into numpy: fps, frames lost, jitter, mid-exposure → pixels latency, CPU; and a verdict. `--seconds 60` for a sustained run. |
| `rs_pi.sh info` / `timing [ARGS]` / `run SCRIPT ARGS` | `rs_info.py`, `rs_timing.py` (`--touch` reads the pixels; `--exposure-us 1000`), any other step. Each deploys this folder to `~/omni_ws/realsense` first. |
| `rs_pi.sh fetch` | Copies the Pi's `results/` to `results/omni-cam/` (the other commands do it after a run). `RS_RESULTS=results/omni-cam python rs_report.py` reports on them. |

Differences from Windows: hardware timestamps and per-frame metadata work out of the box (`global_time`, 22 fields; no registry change); IR / depth intrinsics are missing for a few profiles (Y8 1280×800), which `rs_info` skips; ASIC and projector temperatures are listed but not readable (V4L backend). Check `rs_pi.sh usb` first if the camera isn't found: a USB 2 link shows as 480M there.

**Results, Oct 30, 2026** (Pi 4 4 GB, performance governor, USB 3.2, Pi at 30–32 °C and never throttled; `results/omni-cam/`). Latency is mid-exposure → pixels in numpy in this Python process, on librealsense's `global_time` mapping of the camera clock to the Pi's clock (not yet cross-checked from outside with `rs_flash_latency.py`); detection and transport come on top. CPU is this process, in cores of the Pi's 4.

| Mode | fps | lost | latency median / p95 (ms) | CPU |
| --- | --- | --- | --- | --- |
| IR pair 848×480 @ 90, auto exposure (9.9 ms) | 89.9 | 0 | 14.8 / 14.9 (IR1), 15.2 / 15.2 (IR2) | 0.20 |
| IR pair 848×480 @ 90, **1 ms exposure** | 89.9 | 0 | 10.3 / 10.3 (IR1), 10.6 / 10.7 (IR2) | 0.21 |
| IR pair 640×480 @ 90 | 89.9 | 0 | 15.0 / 15.0, 15.3 / 15.3 | 0.18 |
| Depth 848×480 @ 90 | 89.9 | 0 | 14.7 / 14.8 | 0.15 |
| IR 848×100 @ 300 | 298.9 | 0 | 4.2 / 4.2 | 0.25 |
| IR 1280×800 @ 30 | 30.0 | 0 | 31.5 / 31.9 (to host) | 0.08 |
| Colour 640×480 @ 60 | 59.5 | 0 | 18.2 / 18.3 | 0.14 |
| Colour 1280×720 @ 30 | 30.0 | 0 | 37.1 / 37.1 | 0.15 |
| **60 s:** IR pair + depth 848×480 @ 90, 1 ms | 89.9 each | 0 of 5393 | 10.8–11.5 / ≤ 11.7 | 0.39 |
| **60 s:** the same + colour 640×480 @ 60 | 89.9 / 59.5 | 0 | colour 20.2 / 21.2; IR and depth 16–17 / 23.5–24.5 | 0.59 |

Reading it: the Pi's USB 3 port carries the IR pair and depth at 90 fps for a minute with no lost frame and little CPU, so the link isn't the limit. Latency is half the exposure plus ~10 ms of readout, transfer and host (auto exposure 9.9 ms → 14.8; 1 ms → 10.3): at the catch's 1 ms the IR pair is at ~10 ms, well inside 60 ms. Adding the colour stream to the same pipeline makes its syncer hold the 90 fps frames for the 60 fps colour ones (IR and depth median +5 ms, p95 +13 ms), so keep the IR pair out of a pipeline with colour. The options stay set in the camera while it is powered (the 1 ms exposure above persists into the next run: pass `--exposure-us` or restart the camera to get auto back).

## Notes on the numbers

- **Latency** here stops at the host. The robot adds detection (C++ blob detector), the tracker and transport, so a D435 passing step 2 at 30 ms still needs its detection measured. The flash test's "display" part is this monitor's lag; it isn't on the robot.
- **Depth noise scales as z²σd / (f·B).** The disparity noise σd (px) is the number that transfers: the design's bar has 3.8× the baseline and a longer lens (f ≈ 571 px at 1280×800, 70° vertical, against the D435's ≈ 447 px at 848×480 and ≈ 640 px at 1280×800), so the same σd gives ~5× less depth noise. A plane gives σd at its best; a 2–3 px ball at 6–7 m is worse, which is what step 5 measures.
- **Drift as an angle.** A disparity change of Δd px is a relative yaw of Δd / f rad between the two imagers, which shows as Δz = z²·Δd / (f·B) on a fixed target. Hold the target still: the camera moving on its mount looks like drift. The plane's normal catches mount tilts.
- **The D435's own limits:** the IR pair's ~50 mm baseline (5–6 cm of dense-depth noise at 6.5 m even on a plane, est), mono IR or rolling-shutter colour, a projector pattern on the IR images while the emitter is on, and a sync connector (`inter_cam_sync_mode`, 1.8 V) whose slave modes follow an external pulse on some firmware only (VERIFY on this unit).
- Datasheet values in `rs_common.D435_DATASHEET` (sensors, FOVs, min-Z, the 3.2 mm depth origin behind the glass) are marked VERIFY: the calibration the camera reports wins where both exist.

## Files

| File | What |
| --- | --- |
| `rs_common.py` | Camera helpers (device, modes, options, clocks, results) and the shared maths (plane fits, statistics, stereo error model, flash edges, ballistic fit); `REQ` = the design's requirements |
| `rs_info.py`, `rs_timing.py`, `rs_flash_latency.py`, `rs_depth_plane.py`, `rs_ball.py`, `rs_throw.py`, `rs_drift.py`, `rs_report.py` | The steps above |
| `test_realsense.py` | The maths on synthetic depth images, textured stereo pairs, flash trains and trajectories |
| `rs_pi.sh`, `rs_pi_check.py` | The Pi loop (deploy, run, fetch) and its one-screen check: link, camera, host health, per-mode rate / latency / CPU (the section above) |
| `enable_metadata.ps1` | Windows registry change for per-frame metadata (admin) |
| `rs_fw_update.py` | Firmware update through pyrealsense2, for cameras the official tool can't update (attended) |
| `RealSense.cmd`, `realsense_launcher.ps1` | The button window |
| `results/` | One JSON per run (`<test>_<serial>_<time>.json`), plots, `REPORT.md` |
