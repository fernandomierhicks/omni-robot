# The Raspberry Pi camera head ("omni-cam"): network, access, software

Everything a session needs to reach the Pi and work on it. The board is a **Raspberry Pi 4 Model B (4 GB, Rev 1.5) with an Arducam 12MP Camera Module 3 (IMX708)**. It rehearses design.md 4.9's proposed camera head, a Pi 5 with two Pi Global Shutter cameras that doesn't exist yet. A Pi 5 would be set up the same way (same OS, user, key, `pi.sh`); only the hostname would change. It runs on Wi-Fi by default; for bulk copies and honest network numbers it goes on a wired switch with the PC (section "Wired Ethernet"). Results and setup history: [characterization/pi_camera/README.md](characterization/pi_camera/README.md).

## Network

```
                    AT&T router 192.168.1.254 (DHCP, DNS suffix attlocal.net)
                    Wi-Fi "Minnexico_5ghz", 192.168.1.0/24
                         |                                   |
   FernandoDesktop (Windows 11) 192.168.1.131      omni-cam (Pi 4) 192.168.1.90, wlan0
   + WSL2 Ubuntu-24.04, mirrored networking:       Ethernet unplugged (eth0 DHCP if plugged)
     the same 192.168.1.131
```

| | PC | Pi |
| --- | --- | --- |
| Name | `FernandoDesktop` (`FernandoDesktop.local` resolves on the Pi) | `omni-cam` (`omni-cam.local`, mDNS via avahi; resolves from Windows and WSL) |
| Address | 192.168.1.131, DHCP | 192.168.1.90, DHCP (no reservation: use the name, not the number) |
| MAC | | wlan0 `d8:3a:dd:e1:19:ea`, eth0 `d8:3a:dd:e1:19:e7` (for a router DHCP reservation, if wanted) |

**WSL networking is mirrored** (since Oct 30, `C:\Users\ferna\.wslconfig`: `[wsl2] networkingMode=mirrored`). WSL shares Windows' addresses, so the Pi reaches WSL at 192.168.1.131, and Windows ↔ WSL over `localhost` still works (Launcher, console, 3D view, face: checked for servers on 127.0.0.1 and 0.0.0.0). Undo: delete `.wslconfig`, then run `wsl --shutdown`.

**WSL's firewall** (Hyper-V firewall: inbound blocked by default) has one rule, `WSL-ROS2-DDS-LAN`: UDP 7400–32999 (ROS domains 0–101) from 192.168.1.0/24 allowed in. Check it with `Get-NetFirewallHyperVRule -Name WSL-ROS2-DDS-LAN`; remove it with `Remove-NetFirewallHyperVRule -Name WSL-ROS2-DDS-LAN` (admin). TCP from the LAN into WSL stays blocked, so the console on :8080 is not reachable from other devices.

## Wired Ethernet (Oct 31: PC and Pi on one switch)

Why: the throw recordings are 0.4-2.2 GB per location (`characterization/throw_track`, four streams ~440 MB a throw), and this room's Wi-Fi couldn't move them. On Oct 31 `scp` of location 3 (2.1 GB) stalled at 1.7 GB for 25 minutes and a 145 MB file timed out at 240 s (under 1 MB/s). Over the cable the same 145 MB `frames.npy` took 13 s (~11 MB/s) and a 2 GB location minutes.

**Setup used (Oct 31):** the PC and the Pi each plugged by an RJ45 cable into the **same Ethernet switch**. The Pi's `eth0` takes its address by DHCP (netplan from cloud-init), so a switch or router on the PC's network is all it needs: **192.168.0.101/24** on Oct 31, **192.168.0.100** on Nov 11 (the lease moves: `ssh omni-cam 'ip -br a show eth0'`; on Nov 11 .101 was the PC's own Ethernet adapter, and the Windows metrics were set Wi-Fi 10, Ethernet 500 so internet and DNS go over Wi-Fi while 192.168.0.x stays on the cable), gateway 192.168.0.1 (a different network from the Wi-Fi's 192.168.1.0/24). `eth0` came up on boot with the cable attached; **hot-plugging a cable into a running Pi was not tested** (it was shut down, moved, and powered on with the cable in).

- **Both interfaces stay up.** `wlan0` (192.168.1.90) and `eth0` are both addressed; the default route prefers `eth0` (metric 100 against 600), so the Pi's own traffic goes over the wire, and the Wi-Fi address still answers SSH, handy for changing settings if the cable side misbehaves.
- **Link speed measured: 100 Mb/s**, not the gigabit a Pi 4 can do (`cat /sys/class/net/eth0/speed` = 100). Probably the cable or the switch port; not investigated. 100 Mb/s is ~11 MB/s, still far faster than the Wi-Fi was.
- **SSH to the wired address:** plain `ssh fernando@192.168.0.101` fails (`Host key verification failed`: the number isn't in `known_hosts`, and scripts use `BatchMode`). Reuse the Pi's known key with an alias: `ssh -o HostKeyAlias=omni-cam.local fernando@192.168.0.101` (same for `scp`). To make it a name, add to `C:\Users\ferna\.ssh\config`:

  ```
  Host omni-cam-eth
      HostName 192.168.0.101
      HostKeyAlias omni-cam.local
      User fernando
      IdentityFile ~/.ssh/id_ed25519
      ServerAliveInterval 30
  ```

  then `ssh omni-cam-eth`, or `OMNI_PI=omni-cam-eth bash characterization/throw_track/tt.sh rsfetch SESSION` (`tt.sh` takes the host from `OMNI_PI`). This entry is **not created yet**. The address is a DHCP lease (no reservation): check it with `ssh omni-cam 'ip -br a show eth0'` if it stops answering.
- **Not checked:** which of the two addresses `omni-cam.local` resolves to when both are up (mDNS may return either): use the wired address or the alias when speed matters. ROS between the PC and the Pi (section "ROS 2 between the PC (WSL) and the Pi") was **not** tried over the wire; the firewall rules and `ROS_STATIC_PEERS` below are written for the 192.168.1.x network, and WSL's mirrored networking would have to see the wired adapter's 192.168.0.x address.
- **Direct cable PC to Pi, no switch: not tried.** No DHCP server on that link, so `eth0` would have no address (link-local or a fixed address on both ends would have to be set up; `netplan` on the Pi, the adapter's IPv4 on the PC).
- **Copy a session and check it:** `scp -r -o BatchMode=yes -o HostKeyAlias=omni-cam.local fernando@192.168.0.101:omni_ws/throw_track/data/SESSION /c/Users/ferna/throw_data/`, then compare `find . -type f -printf '%s %p\n'` on both sides (names and sizes). Add `-o ServerAliveInterval=10 -o ServerAliveCountMax=3` and `timeout N` per file so a stalled link fails instead of hanging; a hung `scp -r` left two `/usr/bin/scp` processes running on the PC and had to be killed.
- **Shut down before moving it:** `ssh omni-cam 'sync; sudo shutdown -h now'` (check first that nothing is writing: `ps -eo pid,args | grep -E "[r]ecord_rs|[s]cp|[r]sync"`).

## SSH

- From Windows: `ssh omni-cam` (in Git Bash, or call `C:\Windows\System32\OpenSSH\ssh.exe` by full path; the PowerShell tool's PATH lacks it). Key login only: SSH password login is off.
- `C:\Users\ferna\.ssh\config`: `Host omni-cam`, `HostName omni-cam.local`, `User fernando`, `IdentityFile ~/.ssh/id_ed25519`, `ServerAliveInterval 30`. The key `id_ed25519` (ed25519, no passphrase, comment `fernando@pc-omni`) was made for this. Windows OpenSSH refuses a config or key that other local accounts can read: keep `icacls ... /inheritance:r /grant:r ferna:F SYSTEM:F Administrators:F` on them.
- User `fernando`: passwordless sudo (`NOPASSWD:ALL`, from cloud-init). Password (local console with a keyboard and monitor, and sudo if NOPASSWD is ever removed): `5VpNnhLC30cG`. SSH doesn't accept it (key login only). `sudo passwd fernando` over SSH sets a new one; update it here.
- Scripts use `ssh -o BatchMode=yes` (fail instead of prompting). VS Code: the Remote - SSH extension, host `omni-cam`.
- WSL has no key for the Pi: SSH from Windows. ROS between them needs none.
- If `ssh` times out while `ping omni-cam.local` answers, the Pi is hung (it was once, out of memory); only a power cycle by Fernando fixes it. A hard power-off is fine.

## ROS 2 between the PC (WSL) and the Pi

- Both run Jazzy with Fast DDS (the default RMW). **Discovery must be unicast:** multicast doesn't cross between WSL and the LAN (WSL's discovery multicast never reaches the Pi, and the Pi's never reaches WSL). In WSL, `export ROS_STATIC_PEERS=omni-cam.local`; the Pi needs nothing (it answers the peer that contacted it).
- Use a ROS domain the sim doesn't use, e.g. `ROS_DOMAIN_ID=42`; the sim's steps run on 0 (and others for parallel runs). Don't set `ROS_STATIC_PEERS` globally in WSL's `.bashrc`: only in the shell or script that talks to the Pi.
- Check it: `bash characterization/pi_camera/pi.sh ping` (`dds_ping.py`: an echo on the Pi, 200 pings from WSL). Oct 30, over Wi-Fi: 300 of 300 answered, round trip median 4.4–4.6 ms, p95 7–10 ms, so ~2 ms one way.

## The robot's software on the Pi, the sim on the PC (Oct 30; `pi.sh robot | sim | split | keepup | mcucheck | ghost`)

`stack.py` takes `side:=sim` (only the world: the MuJoCo bridge with the firmware in the loop, its MCU port open to the LAN, and a step's scorer) or `side:=robot` (everything else, `mcu_link` on `mcu_host:=<the PC>`); `both` is the old one-computer default. The two halves share a ROS domain of their own (**43**, `OMNI_DOMAIN`) with unicast discovery both ways (`ROS_STATIC_PEERS`: the PC names `omni-cam.local`, the Pi the PC's LAN address). Two things the PC needs:

- **TCP 5761 into WSL** (the MCU link; the existing `WSL-ROS2-DDS-LAN` rule is UDP only). `pi.sh mcucheck` tests it. Once, in an **admin** PowerShell: `New-NetFirewallHyperVRule -Name WSL-MCU-TCP -DisplayName 'WSL MCU port from the LAN' -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol TCP -LocalPorts 5761 -RemoteAddresses 192.168.1.0/24 -Action Allow` (remove with `Remove-NetFirewallHyperVRule -Name WSL-MCU-TCP`). Until it exists, bring-up can carry the port through an SSH reverse tunnel (`ssh -N -R 5761:127.0.0.1:5761 omni-cam`, then `OMNI_MCU_HOST=127.0.0.1`): fine for checking that things run, not for CPU numbers (sshd decrypts every byte on the Pi).
- **A clock that is right**, for `clock:=wall` (the nodes on the system clock, no `/clock`, like the robot): Windows' clock was **0.48 s off** NTP on Oct 30 (`w32tm /query /status`: source "Local CMOS Clock", never synced) and WSL follows it, while the Pi is within 2 ms. Fix once on the PC (admin): `w32tm /resync` (start the Windows Time service first if needed). `ros/tools/clock_offset.py` measures PC ↔ Pi (server in WSL, client on the Pi, since inbound UDP to WSL must land on a port in 7400–32999). The default `clock:=sim` does not care.

**Results (Oct 30, provisional: Wi-Fi, the MCU port through an ssh tunnel, the Python monitor on the Pi; `ros/tools/pi_report.py` tabulates `results/pi_robot_*.json`):** the catch tier (mcu_link, mode_manager, C++ catch_perception and catch_behavior, `catch-score` from the PC) takes **1.4 cores of the Pi 4's four with the sim's 1 kHz `/clock` and loses catches (11 of 14), 0.9 cores at a 5 ms clock and 0.76 at 10 ms, 13 of 13 caught**: mostly the cost of the sim's `/clock` (each node wakes per message), which the real robot does not have; `clock:=wall` removes it once the PC's clock is right. The navigation tier (EKF, slam_toolbox localization, lean Nav2 composed; `goals`) **does not keep up**: 87% median CPU, busiest core 100%, Nav2's controller at 17–29 Hz instead of 20, scans at 8 Hz, **1 of 8 goals** at 0.44 m/s peak. Not yet: Ethernet, `clock:=wall`, the tiers in between, the Pi's own camera running at the same time.

Ethernet matters for the numbers: the Pi's `eth0` had no cable for these runs (Wi-Fi jitter p95 7–10 ms, max ~26 ms); the wired setup of Oct 31 is in "Wired Ethernet" above, not yet used for them. Results: `characterization/pi_camera/results/pi_robot_*.json` (`ros/tools/pi_keepup.py`: CPU, memory, throttling, topic rates and ages), the PC's half of each run in `results/logs/`.

## Software on the Pi

| What | Where / how |
| --- | --- |
| OS | Ubuntu Server 24.04.5 LTS arm64+raspi, kernel 6.8 raspi; set up headless by cloud-init (`/boot/firmware/user-data`, `network-config`; netplan `/etc/netplan/50-cloud-init.yaml` has the Wi-Fi) |
| Storage | 32 GB SD card (29 GB root; ~9 GB used before the throw recordings, ~23 GB on Oct 31 with ~11 GB of them in `~/omni_ws/throw_track/data`, ~5 GB free), 4 GB `/swapfile` (the build hung without it) |
| Boot config | `/boot/firmware/config.txt` ends with `dtoverlay=imx708` (camera forced; backup `config.txt.orig`) |
| CPU governor | `performance` at boot (`/etc/systemd/system/cpu-performance.service`): `ondemand` doubled capture latency |
| ROS | ROS 2 Jazzy ros-base, colcon, rosdep (`/opt/ros/jazzy`, sourced in `~/.bashrc`) |
| Camera stack | Raspberry Pi's libcamera fork 0.7.2 + rpicam-apps, built from source into `/usr/local` (sources in `~/src/libcamera`, `~/src/rpicam-apps`); Ubuntu's libcamera 0.2 can't drive Pi cameras. Python finds it through `/usr/local/lib/python3.12/dist-packages/libcamera-local.pth` |
| Picamera2 | venv `~/.venvs/cam` (system site packages), with a stand-in `pykms` / `kms` module (headless). Also there: OpenCV 4.6 (system `python3-opencv`), simplejpeg, and `pupil-apriltags` 1.0.4.post11 (pip, Nov 4: the AprilTag detector of the Cameras tab and `characterization/apriltag`). Destroying a tag25h9 detector of that build corrupts the heap, so `apriltag_view.py` never frees one |
| RealSense D435 | On a blue USB 3 port (`lsusb -t`: 5000M, `uvcvideo`; librealsense serial 827312072573, firmware 5.17.3.10). `pyrealsense2` 2.59.1 (PyPI aarch64 wheel, V4L backend) in venv `~/.venvs/rs`; no librealsense build, no udev rules needed. `characterization/realsense/rs_pi.sh` runs its characterization here |
| Attached Nov 11 | The **Arducam is unplugged** (`rpicam-hello --list-cameras`: none). Through a VIA USB 2 hub: a **USB webcam** (A4Tech/Sonix "W4DS", `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._W4DS_SN0001-video-index0`, uvcvideo; MJPEG 640×480 to 3264×2448, 1080p/720p/480p at up to 60 fps listed; delivered 18–19 fps at 720p in a dark room, auto exposure) and the **RPLIDAR A2M8** on its CP2102 adapter (`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` = `/dev/ttyUSB0`, 115200 baud; firmware 1.26, hardware 5, health good; Standard 2 k, Express 4 k, Boost 8 k, Stability 4 k samples/s, all read with no framing errors; ~12.4 Hz at the default PWM 660, 6.0 Hz at 300, 20.0 Hz at 1023). `fernando` is in `dialout` and `video`. pyserial 3.5 is in `~/.venvs/cam`. **No ROS lidar driver** (sllidar_ros2) is installed on the Pi yet; note the robot's baseline lidar is the A2M12 (256000 baud) |
| Attached Nov 12 | An **Arduino UNO R4 WiFi** standing in for the Teensy, on its RA4M1's native USB `/dev/serial/by-id/usb-Arduino_UNO_R4_WiFi_350D260F593130388D3A33334B57303E-if00` (`ttyACM0`; the ESP32-S3 bridge's `usb-Arduino_UNO_WiFi_R4_CMSIS-DAP_D885ACA796AC-if01` while a sketch uses the bridge), running the robot's firmware core with one AK40-10 on CAN: [firmware/bench/omni_fw_uno_r4](firmware/bench/omni_fw_uno_r4/README.md). Flashed from the Pi with Arduino's `bossac` 1.9.1-arduino5 in `~/tools/bossac` (through the bridge: the firmware hands the USB over on a 1200-baud touch); bench files in `~/fw_bench`. `console_bench.sh start` runs mcu_link, mode_manager and the operator console here (ROS domain 44): `http://omni-cam.local:8080` (Test › Motors). `gdb` installed (apt, Nov 12) for native crashes |
| Services | `ssh`, `avahi-daemon` (mDNS), `cpu-performance`, `systemd-timesyncd` (NTP, since Nov 12), `unattended-upgrades` (Ubuntu's default, still on: design.md 7). Ours, both up after every reboot (4 on Nov 12): **`omni-cam-snap`**: `camera_snap` for the console's See › Cameras and Test › Lidar pages on :8086, restarted ~4 s after a crash, so the console needs nothing started on the Pi. Idle until a page asks (~46 MB). Unit `/etc/systemd/system/omni-cam-snap.service`, written by `ros/tools/cam_snap.sh start` (`stop` until the next boot, `remove` takes it off). **`omni-console`**: omni_console's supervisor, the robot's front door as on the real robot, `http://omni-cam.local:8080` (https :8443); Robot › System › Start runs the step `robot` (`omni_bringup hardware.launch.py`: mcu_link on the MCU found by id, mode_manager, rosbridge, rosapi, the mission node; ROS domain 0, localhost discovery). Nothing autostarted. `ros/tools/robot_console.sh install | status | log | restart | stop | remove`; what each failure does and how to deploy: omni_console/systemd/README.md. **`/etc/systemd/logind.conf.d/omni-keep-ipc.conf`** (`RemoveIPC=no`, Nov 12; `robot_console.sh install` writes it): without it the step's DDS shared memory is deleted whenever the last SSH session ends (Gotchas). The bench (`~/fw_bench/console_bench.sh`) and the Robot step can't run together (same ports and MCU): stop one first |
| Debug tools | `~/.venvs/tools` (Nov 12): `py-spy` (`sudo ~/.venvs/tools/bin/py-spy dump --pid PID`: a running Python node's stacks, read-only) |

Folders on the Pi:

```
~/omni_ws/              the ROS workspace: a copy, never the master (pi.sh deploy overwrites it)
├── src/                ros/src without omni_sim
├── firmware/           firmware/ (omni_cpp compiles the MCU protocol from it: OMNI_FIRMWARE_DIR)
├── pi_camera/          characterization/pi_camera; results/ is kept across deploys (copy it back with scp)
├── build/ install/ log/
~/src/libcamera, ~/src/rpicam-apps      the camera stack's sources (rebuild: setup/build_libcamera.sh)
~/.venvs/cam                            Picamera2 (cam_probe.py runs with ~/.venvs/cam/bin/python)
~/install_ros.sh, ~/build_libcamera.sh, *.log, *.done    the setup scripts as run, and their logs
```

## Software in the repo

| Path | What |
| --- | --- |
| `characterization/pi_camera/pi.sh` | The PC → Pi loop, from Git Bash: `deploy`, `build`, `test`, `run CMD`, `head`, `probe`, `ping`, `ssh` |
| `characterization/pi_camera/setup/` | `install_ros.sh` (swap, governor, ROS), `build_libcamera.sh` (the camera stack): to rebuild a card |
| `characterization/pi_camera/cam_probe.py` | Picamera2 capture + the detector, no ROS: fps, drops, latency |
| `characterization/pi_camera/bench_config.py`, `head_probe.py`, `dds_ping.py` | The bench's one-camera robot_config; a running head's numbers; the PC ↔ Pi round trip |
| `characterization/pi_camera/results/` | One JSON per run |
| `characterization/ball_view/bv.sh`, `ball_view.py` | The ball detector live on either camera, in the PC's browser (`http://omni-cam.local:8085/`): `bv.sh picam`, `realsense`, `synthetic`, `stop`. Runs in `~/.venvs/cam` (Picamera2 and `pyrealsense2` both there); one camera per process |
| `characterization/realsense/rs_pi.sh`, `rs_pi_check.py` | The D435 on the Pi: `rs_pi.sh usb` (is it on the bus, at what speed), `check [--seconds S]` (link, camera, Pi health, per-mode fps / lost / latency / CPU), `info`, `timing`, `run SCRIPT`, `fetch` (results to `results/omni-cam/`). Copies to `~/omni_ws/realsense` |
| `characterization/throw_track/tt.sh`, `record_throws.py` | Thrown balls on the Arducam: `tt.sh record` (ring buffer, y / n after each throw), `fetch`, `analyze`, `tag`, `test`; the Pi side needs only numpy and the robot's detector (README.md there) |
| `characterization/pi_camera/pi.sh robot \| sim \| split \| keepup \| mcucheck \| ghost` | The robot's nodes on the Pi against the PC's sim; `ros/tools/pi_keepup.py`, `pi_report.py`, `clock_offset.py` |
| `ros/tools/robot_console.sh`, `pi_console.py`, `pi_robust.py` | The console on the Pi (omni_console/systemd/README.md): install and manage the service; drive its page from the PC in Edge step by step (`pi_console.py serve` in the background, then `open '#lidar'`, `click`, `text`, `shot`, `errors`, `hold w 2`, `tab new --mobile`); start/stop cycles and node kills with checks for leftovers (`pi_robust.py cycles -n 20`, `kill mcu_link`) |
| `ros/tools/cam_snap.sh` | The console's Cameras tab's picture server and the Test › Lidar page's lidar server (`omni_console/camera_snap.py` + `apriltag_view.py` + `omni_slam/tags.py` + `rplidar.py`, :8086, `~/.venvs/cam`, copied to `~/omni_ws/cam_snap`): `start` (deploy and (re)start the boot service `omni-cam-snap`), `stop`, `status`, `log`, `remove`. When it doesn't answer, both pages list what to check (Pi off, address, server stopped). Snapshots, live video (`/stream`) and AprilTags; every camera (Arducam, D435, USB webcam) in one process, and the lidar (`/lidar`: info, health, modes, live turns; motor only while the page scans). Nov 11, wired: webcam 720p live 18.5 fps (dark room), 640 px JPEGs ~10 kB; lidar Boost turns to the page at ~12.5 Hz, ~120 kB/s. Each camera opens at its first request and closes 20 s after the last or after its last stream. Nov 4, wired, one stream at 640 px: Arducam 30 / 56 / 99 fps (full field 30, full field 56, 2/3 crop 100), D435 30 fps, ~0.3–0.6 MB/s each; three streams with tags saturate the Pi 4. Nov 3, wired: first D435 picture 2.5 s after opening (it waits ~20 frames for auto exposure), then 10–110 ms; 640 px JPEGs 13–48 kB. `omni-cam.local` took ~11 s to resolve from Windows that day, so typing the address (192.168.0.101 on the switch) in the tab's Server field is quicker |
| `characterization/apriltag/tl.sh`, `tag_lock.py` | The AprilTag lock bench on either camera (`tl.sh run`; it stops `cam_snap` first): detection rate, corner / pose scatter, accuracy, detector cost, misreads. Results and setup in its README |
| `ros/src/omni_console/omni_console/rplidar.py` | The RPLIDAR's serial protocol without ROS or the SDK (info, health, GET_LIDAR_CONF scan modes, motor PWM; standard, express and ultra-capsule answers): `~/.venvs/cam/bin/python ~/omni_ws/cam_snap/rplidar.py [/dev/ttyUSB0] [--mode Boost]` prints the device and a few seconds of turns. Stop `cam_snap` (or leave the Lidar page) first: one reader per port. **Motor:** the A2M8's adapter runs the motor whatever the PWM while DTR is raised (pyserial raises it on open), so the driver keeps DTR cleared and stops by PWM 0 (Nov 13; before, Stop left it turning). `ros/tools/lidar_motor_test.py` checks which signal stops it by counting turns in short scans |
| `ros/src/omni_camera` | Robot code: `libcamera_capture` (C++) and `camera_head.launch.py` (+ omni_cpp's `catch_perception`, intra-process). Built only where Raspberry Pi's libcamera exists |

The workflow: edit on the PC, then `bash characterization/pi_camera/pi.sh build` (deploy + `colcon build`, sequential, `-j2`: parallel rclcpp compiles hung the 4 GB Pi), then `pi.sh head` (the camera head in the foreground, Ctrl-C stops it) and `pi.sh probe` from a second terminal.

## Gotchas

- `pkill -f PATTERN` over SSH matches the remote shell's own command line, which contains PATTERN, and kills it. Use a pattern that can't match it (e.g. `pkill -f "python3 pi_camera/dds_ping"` from a script, or kill by pid).
- A camera head started detached (`setsid nohup ros2 launch ...`) can leave its `camera_head` component_container running after the launch is killed: `kill -INT <pid>`.
- Only one process can hold the camera: stop the head before `cam_probe.py`, and the reverse. The same goes for `ball_view`, `cam_snap` and the tag bench.
- **The Pi's clock: NTP turned on Nov 12** (`sudo timedatectl set-ntp true`; it had read Oct 12 at every boot since early November: no RTC, NTP inactive). The wrong clock broke more than file names. TLS failed (certificates "not yet valid"), and **incremental colcon builds linked stale code**: a boot that restarted the clock earlier than the last build left the old `omni_cpp_cores` archive looking newer than fresh objects, and `mcu_link` segfaulted on its first STATE (two halves with different struct layouts). After any clock jump, rebuild clean (`rm -rf build/<pkg> install/<pkg>`). Check `timedatectl` (System clock synchronized: yes) before timing work; it needs the Wi-Fi's internet at boot.
- **The lidar not answering (Nov 12):** the CP2102 adapter enumerated, but at every baud only `0x00` bytes came back (~480/s, whatever the request): the lidar's TX line held low, so its side was unpowered or its cable to the adapter loose. Reseating it fixed it. The Lidar page now says so ("only 0x00 bytes"), reconnects by itself when the adapter re-enumerates (it came back as `ttyUSB1`), and lists what happened under Events. The motor's start at PWM 660 on the VIA hub did not reset the USB that day.
- **logind's `RemoveIPC` broke ROS under systemd (Nov 12):** the robot step runs as `fernando` under a system service, not a login session, so with Ubuntu's default `RemoveIPC=yes` every time the last SSH session ended logind deleted `fernando`'s `/dev/shm`, including Fast DDS's shared-memory segments in use (`(deleted)` in the node's `/proc/PID/maps`). Topics already flowing kept going; anything new (a page reconnecting to rosbridge, a `ros2` CLI) never reached the nodes, and the console's `/console/command` timed out ("The robot didn't answer"). It worked while any session stayed open (VS Code's), which hid it. Fixed with `RemoveIPC=no` (Services); check with `systemd-analyze cat-config systemd/logind.conf | grep RemoveIPC`.
- **Times early in a boot:** no RTC, so the Pi boots with the time saved at shutdown and steps forward ~30–40 s at the first NTP sync. Journal gaps at boot (a service "taking 40 s") are that jump: time boots from the PC.
- **The Pi's colcon install copies files** (no `--symlink-install`): after a deploy, rebuild even for a web-only change to `omni_console`.
- To rebuild the SD card from scratch: README.md "Setting up a fresh card" (Raspberry Pi Imager's CLI, cloud-init files, then the two setup scripts).
