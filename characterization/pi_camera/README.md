# Pi camera head prototype (Pi 4 + Camera Module 3)

A rehearsal of design.md 4.9's proposed camera head (a Pi 5 with two Pi Global Shutter cameras, capture and the blob detector on the Pi, detections to the main computer) and of phase 1C step 3, on hardware already at hand: a **Raspberry Pi 4 Model B (4 GB)** and an **Arducam 12MP Camera Module 3 (IMX708)**. It is also the first board that runs the robot's code off the PC, and the PC → board deploy loop the robot will use.

What this setup can't tell us: global shutter, the XTR hardware trigger, stereo or calibration drift (one camera). The IMX708 is rolling shutter; its 120 fps mode is a 2×2 binned 1536×864 crop. A Pi 5 is ~2–3× faster per core (est).

**Nov 2 interface audit:** [compute purchase trade and interface budget](../../components/computer/compute_purchase_trade_2026-11-02.md) records the proposed host USB/hub allocation, MCU pins and unfinished timing path. `camera_head.launch.py` currently launches one capture instance, and `libcamera_capture.cpp` uses SensorTimestamp without Teensy trigger pairing; this bench does not validate the future stereo/clock architecture. Mini-PC USB echo is a fallback, not proof of the older GPIO pulse accuracy target. No new runs were made for that audit.

## The board

| | |
| --- | --- |
| Host | `omni-cam` (`omni-cam.local`, mDNS; Wi-Fi `Minnexico_5ghz`, DHCP) |
| OS | Ubuntu Server 24.04.5 LTS (arm64+raspi), kernel 6.8 raspi, 4 GB swap file |
| Login | `ssh omni-cam` from the PC (Windows key `~/.ssh/id_ed25519`, entry in `~/.ssh/config`); user `fernando`, sudo without a password; SSH password login off |
| ROS | ROS 2 Jazzy ros-base, colcon, rosdep (`setup/install_ros.sh`); `~/omni_ws` is the workspace |
| Camera | Raspberry Pi's libcamera fork (0.7.2) + rpicam-apps built from source into `/usr/local` (`setup/build_libcamera.sh`); Picamera2 in `~/.venvs/cam` |

**Why Ubuntu, not Raspberry Pi OS:** ROS 2 Jazzy is native on Ubuntu 24.04 only (Pi OS would need Docker), and the main computer (GMKtec M8) runs it too, so builds and deploys are the same on both. The price: Ubuntu's libcamera 0.2 lacks Raspberry Pi's IPA ("Failed to load a suitable IPA library"), so the fork is built from source (~30 min on a Pi 4), and Picamera2 needs two shims: a `.pth` pointing at `/usr/local/lib/python3/dist-packages` and a headless `pykms` stand-in in the venv.

## Setting up a fresh card (what was done, Oct 29–30)

1. Raspberry Pi Imager's CLI writes `ubuntu-24.04.5-preinstalled-server-arm64+raspi.img.xz` (admin prompt), then cloud-init's `user-data` (hostname, user, key, NOPASSWD sudo, avahi) and `network-config` (Wi-Fi, Ethernet) go onto the `system-boot` partition. No monitor or keyboard.
2. `config.txt`: `dtoverlay=imx708` appended (auto-detect found nothing until the ribbon was reseated; kept explicit).
3. On the Pi: `bash install_ros.sh` (swap, ROS 2), `bash build_libcamera.sh` (libcamera, rpicam-apps, Python path), then `python3 -m venv --system-site-packages ~/.venvs/cam && ~/.venvs/cam/bin/pip install picamera2` (needs `libcap-dev`) and the `pykms.py` / `kms.py` stand-in in that venv's site-packages.

Lessons: the CPU governor must be `performance` (`install_ros.sh` installs `cpu-performance.service`; `ondemand` doubled capture latency); the Pi 4 hung (no swap, four ~1 GB rclcpp compiles) until the swap file and `pi.sh build`'s `--executor sequential`, `MAKEFLAGS=-j2`; Windows' OpenSSH refuses a config file other local accounts can read (`icacls ... /inheritance:r`).

## The loop (from Git Bash on the PC)

| Command | What |
| --- | --- |
| `bash characterization/pi_camera/pi.sh deploy` | Copies `ros/src` (no `omni_sim`), `firmware/` and this folder to `~/omni_ws` (`src/`, `firmware/`, `pi_camera/`), like `omni.sh`'s mirror; `results/` on the Pi is kept |
| `pi.sh build` | Deploy + `colcon build --packages-up-to omni_interfaces omni_mcu omni_catch omni_perception omni_cpp omni_camera omni_bringup` (~7 min clean; ~1 min incremental) |
| `pi.sh test` | Deploy + `omni_cpp`'s core tests: the C++ and Python cores on the same inputs, on ARM |
| `pi.sh run CMD` | CMD on the Pi in `~/omni_ws` with ROS and the workspace sourced |
| `pi.sh head [ARGS]` | The camera head in the foreground (Ctrl-C stops it): `bench_config.py`, then `omni_camera`'s `camera_head.launch.py` with the bench config and `sleep:=false`; ARGS go to the launch (`perception:=false`, `exposure_us:=2000`, ...) |
| `pi.sh probe [S]` | `head_probe.py` for S seconds (10) against a running head, from a second terminal |
| `pi.sh ping [N]` | ROS 2 round trip between WSL and the Pi: an echo on the Pi, N pings from WSL (200) |
| `pi.sh robot STEP [ARGS]` | The robot's half of a sim step on the Pi (`side:=robot`, `mcu_host:=` the PC, ROS domain 43, `console:=false`): `catch`, `catch-score`, `robot`, `goals` ... |
| `pi.sh sim STEP [ARGS]` | The PC's half in WSL (`side:=sim`: the world and the firmware in the loop, MCU port open to the LAN) |
| `pi.sh split STEP LABEL [SECONDS] [ARGS]` | One scored run: the Pi's half, the PC's half, `ros/tools/pi_keepup.py` on the Pi for SECONDS, both stopped, `results/pi_robot_<label>.json` (+ logs). `OMNI_MCU_HOST=127.0.0.1` with an ssh tunnel for bring-up without the firewall rule |
| `pi.sh keepup OUT [S]` | `pi_keepup.py` alone beside whatever runs on the Pi |
| `pi.sh mcucheck` | Can the Pi open the PC's MCU port (TCP)? Says which firewall rule is missing |
| `pi.sh ghost [ARGS]` | The camera head publishing its track as `/ghost/track` on the sim's domain, for the ghost-ball test (`../throw_track`, omni_sim `ghost_ball`) |
| `pi.sh ssh` | A shell on the Pi |

The network, SSH and the Pi's folders: [rpi.md](../../rpi.md) at the repo root.

Results come back with `scp "omni-cam:omni_ws/pi_camera/results/*.json" characterization/pi_camera/results/`.

## Results so far (Oct 30, 2026)

| Check | Result |
| --- | --- |
| `omni_cpp` core tests on aarch64 | 19 / 19 pass (g++ builds `core_check` and `fast_detector`'s `blob.so` on the Pi) |
| colcon build of the camera head's packages | 5 / 5; all six omni_cpp nodes and components registered (`catch_perception`, `catch_behavior`, `mcu_link`, `mode_manager`, `scan_prep`, `heading_node`) |
| `cam_probe.py`, 120 fps, ISP output 768×432, detector on, `performance` governor | **120.1 fps, 0 dropped of 1202; sensor → detection median 16.8 ms, p95 19.6, max 21.8**; detector (acquisition, whole frame) 0.54 ms median; 0.58 cores |
| Same, capture only (`--no-detect`) | 120.1 fps, 0 dropped; sensor → Python median 16.0 ms, p95 18.9; 0.49 cores |
| Same at 1536×864 (detector on / off) | 117–118 fps, 18–27 dropped (~2%, with or without the detector: the 4 MB frame's copy into numpy, est); sensor → detection median 21.7 ms; detector 1.56 ms; 1.1 cores |
| The kernel's default `ondemand` governor, 768×432 | detector on: 120 fps, 1–2 dropped, median 22–24 ms; **capture only: 115 fps, ~35 dropped, median ~45 ms**. The lighter load lets the clock drop between frames, so each one waits for the CPU to ramp back up. Fixed with `cpu-performance.service` (`setup/install_ros.sh`); each result records its `governor` |
| `rpicam-vid` to `/dev/null` | 63 fps at 1536×864, 90 at 768×432: its output path, not the camera |
| ROS 2 round trip WSL ↔ Pi over Wi-Fi (`pi.sh ping`, `dds_ping.py`, domain 42, unicast discovery) | 200–300 of 200–300 answered; median 4.2–4.6 ms, p95 7–10, max 22–26: ~2 ms one way |
| **The camera head as ROS nodes** (`pi.sh head`, `head_probe.py`, 15 s): `omni_camera`'s `libcamera_capture` (C++) + `omni_cpp`'s `catch_perception` in one container, intra-process, the bench's one-camera config | **120.1 fps, 0 dropped; all 1561 frames through perception; sensor → published median 15.2 ms, p95 ≤ 17.8; perception 0.57 ms a frame; 0.45 cores for both** (Picamera2 + the detector in Python: 0.58) |

In the camera head, sensor → detection is the lag to published, the intra-process handoff (not measured; a queue wake-up) and perception's 0.6 ms: ~16 ms. About half of the 15 ms is the frame's own readout (rolling shutter, ~8 ms at 120 fps), the rest the ISP and the copy out of its buffer.

Sensor → detection starts at SensorTimestamp (start of readout), so it leaves out the exposure (7.9 ms here, capped by the 8.3 ms frame) and the transport to the main computer. The design needs ≤ 60 ms capture → detection (sims assume 45–50 ms end to end). No ball was in view: `frames_with_ball` 0; the detector's cost on a frame with a ball (windowed tracking) is lower than acquisition.

## The robot's software on the Pi, the sim on the PC (Oct 30, provisional)

`pi.sh split STEP LABEL SECONDS [ARGS]` runs the robot's half on the Pi and the world on the PC (rpi.md has the setup: ROS domain 43, unicast discovery, the TCP firewall rule, the ssh-tunnel stopgap used here), with `ros/tools/pi_keepup.py` measuring the Pi. `ros/tools/pi_report.py` makes the table. What ran:

| Tier | Pi runs | Result |
| --- | --- | --- |
| T1 catch | mcu_link, mode_manager, C++ catch_perception / catch_behavior (`catch-score`, dist 4 m) | sim `/clock` 1 ms: 1.4 cores, 11 of 14 caught; 5 ms: 0.9 cores, 13 of 13; 10 ms: 0.76 cores, 13 of 13 (the clock's per-message cost is sim overhead) |
| T3 navigation | + EKF, scan_prep, slam_toolbox localization, Nav2 (lean, composed) (`goals`, 8 goals) | CPU 87% median / 100% busiest core; Nav2 controller 17–29 Hz of 20; scans 8 of 10 Hz; **1 of 8 goals**, 0.44 m/s peak |

Lessons: a monitor that subscribes `/clock` in Python was the biggest process on the Pi (it now runs on the system clock and reads only 10 Hz topics); `pkill -f 'pattern'` over ssh matches the remote shell itself, so the cleanup never ran and robot halves from earlier runs were left running (use `[x]` in the pattern, as `pi.sh` now does; check `ps` between runs); started together with the bridge, the robot's nodes engaged only ~20 s into the sim, so `pi.sh split` starts the Pi's half first.

## Next

1. ~~The `--no-detect` anomaly~~: the `ondemand` governor (above). The camera head's computer needs `performance` (or a fixed clock) too; the same may hold for the main computer's catch nodes (VERIFY on the M8).
2. Photons → detection with an LED on a Pi GPIO pin (the Pi stamps the edge and the detection on one clock); then a real yellow ball in view (`frames_with_ball`, the detector's thresholds under this house's light).
3. ~~`catch_perception` on the Pi~~ (done: `omni_camera`, above). ~~ROS between WSL and the Pi~~ (done Oct 30: mirrored networking, a firewall rule, unicast discovery; [rpi.md](../../rpi.md)). Left: the head's `/catch/track` seen on the PC with a ball in view. With one camera there are no stereo tracks; a mono track from the ball's size is possible (the tracker fits it) but untested.
4. A systemd service so the probe / camera node starts on boot.

## Files

| File | What |
| --- | --- |
| `pi.sh` | Deploy, build, test, run on the Pi |
| `cam_probe.py` | Capture rate, drops, sensor → Python and sensor → detection latency, the detector's cost (Picamera2, no ROS) |
| `bench_config.py` | The bench's `robot_config` for `catch_perception`: robot_v11.json with one camera, this module's size and FOV (est) |
| `head_probe.py` | A running camera head's capture stats, perception stats, tracks and container CPU |
| `dds_ping.py` | ROS 2 round trip between WSL and the Pi (`pi.sh ping`) |
| `../../ros/tools/pi_keepup.py`, `clock_offset.py` | Does the robot computer keep up (CPU, memory, throttling, topic rates and ages); PC ↔ Pi clock offset |
| `../throw_track` | Real throws on this camera: recorder, analysis, the sim's camera model from them |
| `../../ros/src/omni_camera` | The camera head's capture node and launch (robot code, not bench code) |
| `setup/install_ros.sh` | Swap, the `performance` governor at boot, ROS 2 Jazzy ros-base, colcon, rosdep |
| `setup/build_libcamera.sh` | Raspberry Pi's libcamera + rpicam-apps from source, Python path |
| `results/` | One JSON per probe run |
