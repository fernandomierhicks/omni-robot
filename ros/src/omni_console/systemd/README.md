# The console on the Jetson: page and launcher at boot

The supervisor (`omni_console/supervisor.py`) is the robot's front door. systemd starts it at boot; it serves the
operator page on port 8080 and starts the robot's software (`config/robot_steps.json`, step `robot`) at once. From the
PC or the iPad, open `http://omni.local:8080` and pick a mode: there is no launcher to run. Its System tab restarts the
software, shows its log, and shuts the Jetson down.

Not tried on a Jetson yet (no hardware): the steps below are the plan. VERIFY each one on the robot.

1. A user `omni` with the workspace in `/home/omni/omni_ws`, built with `colcon build --packages-skip omni_sim`.
2. The unit:

   ```bash
   sudo cp systemd/omni-console.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable --now omni-console
   journalctl -u omni-console -f        # the supervisor's own log; each step's is in ~/.omni/logs/steps/
   ```

3. The name `omni.local`: set the hostname to `omni` (`sudo hostnamectl set-hostname omni`); Ubuntu's avahi-daemon
   announces it, and iPad Safari and Windows both resolve `.local` names.
4. **Keep the user's shared memory** (`ros/tools/robot_console.sh install` does it): the robot's nodes run as a regular
   user under a system service, not a login session, and logind's default `RemoveIPC=yes` deletes that user's
   `/dev/shm` when their last SSH session ends. That cuts Fast DDS's shared memory from under the running nodes:
   topics already flowing keep going, but anything new (a page reconnecting, a CLI) never reaches them, and the
   console's `/console/command` stops answering ("The robot didn't answer"). Found on the Pi, Nov 12.

   ```bash
   printf '[Login]\nRemoveIPC=no\n' | sudo tee /etc/systemd/logind.conf.d/omni-keep-ipc.conf
   sudo systemctl restart systemd-logind
   ```

5. Shutdown and reboot from the page: the supervisor runs `sudo -n systemctl poweroff | reboot`. Allow exactly those:

   ```bash
   echo 'omni ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff, /usr/bin/systemctl reboot' | sudo tee /etc/sudoers.d/omni-power
   sudo chmod 440 /etc/sudoers.d/omni-power
   ```

6. Optional PIN for the System tab and power (kid-proofing, not security): `Environment=OMNI_CONSOLE_PIN=1234` in the
   unit.
7. JetPack 6 is Ubuntu 22.04 and the stack is Jazzy (24.04): if Jazzy runs in Docker, the unit starts the container and
   the supervisor runs inside it (with `--network host`, so 8080 and rosbridge's 9090 are the Jetson's ports).
8. The secure page, `https://omni.local:8443` (`OMNI_CONSOLE_HTTPS` in the unit; `openssl` makes a self-signed
   certificate in `~/.omni/tls` on first start). Browsers let a page use the microphone only when it is secure, so the
   Call page's push-to-talk works there; rosbridge and the camera streams go through the same port. Each phone or
   tablet warns about the certificate once: accept it. The robot's own microphone (Listen on the Call page) is
   `arecord` on the default ALSA capture device (`OMNI_MIC_CMD` to change it; VERIFY the device on the robot).

## On the Pi 4 bench (since Nov 12)

The same front door runs on the Pi ("omni-cam", rpi.md), so the PC needs no WSL to look at or drive the hardware: open
`http://omni-cam.local:8080` (or the Pi's address). `bash ros/tools/robot_console.sh install` writes this unit with
the Pi's user and home, sets logind's `RemoveIPC=no` (step 4 above), enables it for boot and starts it (unless another
program holds port 8080). It differs from the Jetson's in
two ways: **no autostart** (the bench MCU isn't always plugged in; Robot › System › Start starts the robot;
`OMNI_AUTOSTART=robot` gives the real robot's setting) and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (the page reaches
the robot through rosbridge, so its DDS stays on the Pi). The step `robot` is `omni_bringup hardware.launch.py`: the
robot's software on its real hardware as far as it exists (mcu_link on the Teensy, or the UNO R4 standing in for it,
mode_manager, rosbridge, rosapi, the mission node); the lidar driver, EKF, SLAM and Nav2 join it as their drivers do.
The Cameras and Lidar pages use `camera_snap` (its own boot service, `omni-cam-snap`). At its start it self-tests the
lidar (spins it ~4 s, counts turns, stops it) and writes `~/.omni/lidar_selftest.json` (`--state-dir`, default
`$OMNI_STATE_DIR`), which the console's preflight reads, so the page says whether the lidar or its driver is missing;
the robot step's `arm` option (the Arm page's "Switch the arm on") adds the SO-101 once its calibration is in the state
dir (`hardware.launch.py`). Measured on the Pi 4: the
supervisor 0.0 % of a core idle and 0.2 % with the System page open, 20 MB; `camera_snap` ~0.2 % idle, 46 MB.
`robot_console.sh status | log | restart | stop | remove`.

**Getting changes onto it:** `bash characterization/pi_camera/pi.sh deploy` (it also writes `~/omni_ws/version.json`
for the Version tab), then `pi.sh run "colcon build --packages-select omni_console omni_bringup"`. The Pi's install
**copies** the web files (no `--symlink-install`), so a page-only edit needs the build too; reload the page after. A
supervisor change needs `robot_console.sh restart`; `camera_snap` changes, `bash ros/tools/cam_snap.sh start`. Stop the
robot step first (System › Stop) so nothing runs out of the install while it is replaced.

**What each failure does** (Nov 12, the Pi with the UNO R4 stand-in; `pi_robust.py` reproduces the kills):

| Event | What happens |
| --- | --- |
| Start / Stop from the page | MCU answering ~2 s after Start, rosbridge ~2.5 s; Stop in 1–2 s, exit 0, no processes left, MCU port and 9090 free (20 cycles) |
| A node dies (mcu_link, mode_manager, rosbridge, rosapi, mission_node) | `hardware.launch.py` ends the launch; the supervisor starts the step again within ~4 s, at most 3 times in 5 min, then leaves it stopped and the System page says so |
| The MCU's USB pulled and replugged | `mcu_link` says "link lost", retries every 5 s and is back ~0.5 s after the board re-enumerates; the page logs "The MCU went quiet" and "The MCU is back". No step restart |
| The lidar adapter pulled mid-scan | Lidar page: "lost the lidar while scanning", reopened by itself ~3 s after it re-enumerates; the scan isn't resumed (press Start) |
| The page closes while scanning / driving | The lidar motor stops and the port is let go after 20 s; motion stops within 0.3 s (the console's stick timeout, mode_manager's command timeout, the MCU's own 0.5 s), but the robot stays armed in its mode (design.md 7) |
| `camera_snap` killed | systemd starts it again; answering in ~4 s |
| The supervisor killed (`kill -9`) | systemd stops the step's leftover processes (SIGINT, ~4 s) and restarts the supervisor: the page answers in ~7 s; on the bench the step stays stopped (no autostart) |
| `systemctl restart omni-console` | The supervisor stops the step, restarts; the page answers in ~4 s |
| 30 min with the step running and the Lidar page scanning | The robot's processes 68 % of one core (mission_node 22, camera_snap 19, mcu_link 14), 36 °C, no throttling, memory flat (mission_node +1.5 MB); `characterization/pi_camera/results/pi_console_soak_2026-11-13.json` |
| Reboot (System › Reboot, or `sudo systemctl reboot`) | Both services up, nothing ROS until Start; the page answers 64–101 s after the request (one shutdown was held 30 s by `unattended-upgrades`) |

The Pi has no real-time clock: until NTP syncs at boot, its clock runs behind (it restores the time saved at
shutdown), so journal times early in a boot are off by the downtime and a ~30–40 s jump appears at the sync. Time boot
and service starts from the PC, not from the journal.

**Driving the page from the PC** (an AI session or a terminal; Playwright with the installed Edge, the sim venv):
`ros/tools/pi_console.py serve` in the background holds one browser; then `open '#lidar'`, `click Start`, `text`,
`shot`, `errors` (404s, failed requests, console errors since the last call), `hold w 2 --press Space@1` (the
keyboard's dead man, STOP mid-run), `eval` (the page's `window.omniConsole.store`), `tab new --mobile` (a phone-sized
second viewer). Keyboard focus matters: after `fill` into a number box, W and S type into it, so click the page first.
`ros/tools/pi_robust.py cycles -n 20` (start / stop from the page, checking for leftovers), `kill NAME` (one piece,
then the timeline), `state`.

No browser runs on the Jetson: drawing the page costs CPU and memory the robot needs, so the page always runs on the
viewing device. For a screen on the robot, mount an old phone or tablet showing the page.
