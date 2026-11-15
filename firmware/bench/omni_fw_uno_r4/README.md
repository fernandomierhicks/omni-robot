# The robot's firmware on an UNO R4 WiFi, one real AK40-10, ROS on the Pi

A bench stand-in for the Teensy 4.1: the robot's firmware core (`lib/omni_fw`, arm variant, **unchanged**: same build
id as the Teensy's) with a small UNO R4 board layer (`src/main.cpp`), on the Pi's USB. The robot computer runs the
robot's own `mcu_link` and `mode_manager` against it, so a twist on `/cmd_vel/manual` turns the real motor exactly as the
robot's software would: TELEOP → `/mcu/cmd` → COBS/CRC protocol 8 → the core's base controller → servo-mode CAN. The
operator console runs on the Pi too (`console_bench.sh`): `http://omni-cam.local:8080`, **Test › Motors** drives each
wheel motor by itself (the firmware's `DRIVE_WHEELS`).

**Since Nov 13 the default build reports the bench as it is:** the motor(s) on the bus real, whatever their CAN id, the
other wheels absent (`DM_NO_REPLY` in STATE, protocol 8), the IMU silent. The console then refuses driving (by hand too)
and everything that drives itself, and its motor test turns the wheels that answer. (Until the afternoon of Nov 13 only
id 1 was commanded: a motor set to id 2 showed as answering but never turned. Checked after the fix: wheel 2 tracked
±5.5 rad/s within 0.2 in Test › Motors.) `pio run -e virtual` makes wheels 2-4 virtual again, for the whole-chain TELEOP run of Nov 12
(the console believes four motors are there: never on a robot).

Wiring as [../ak40_uno_r4](../ak40_uno_r4/README.md) (D10/D13 to the transceiver, **common ground with the motor**).

## What differs from the Teensy layer

| | Teensy (`firmware/src/main.cpp`) | This bench |
| --- | --- | --- |
| Link | native USB | **the RA4M1's native USB** (`platformio.ini` drops the board's `-DNO_USB`, so the core moves the USB mux off the ESP32-S3; also `-DBACKTRACE_SUPPORT`, whose printer needs the UART; the 1200-baud touch hands the USB back to the bridge for flashing, below). The port is `/dev/serial/by-id/usb-Arduino_UNO_R4_WiFi_<serial>-if00` |
| CAN | FlexCAN, hardware timestamps | `Arduino_CAN` on D10/D13 with the start-up kick (CANRX stuck dominant until D10 is driven high once); stamped when read |
| Motors | four on the bus | **whichever ids are heard are real** (a motor's status broadcast puts its id on the bus; any of 1-4); the others absent (STATE `DM_NO_REPLY`; the core fault-stops driving on a silent wheel, but its motor test turns the wheels that answer). Frames for ids never heard never go on the bus (an unacknowledged frame holds the single extended TX mailbox). Env `virtual` (and `profile`): id 1 real, ids 2-4 **virtual** (status broadcast at 500 Hz, 50 ms speed response) |
| Battery | ADC divider | fixed 24.0 V (`BENCH_V_BUS`), not measured |
| Not here | | BNO085 (IMU_SILENT is reported), RC, e-stop input, bumpers, flywheels, feed, tilt, time references, hardware watchdog, LED (D13 is CANRX) |

**Not the ESP32-S3 bridge's UART.** The R4 WiFi's default `Serial` is the RA4M1's UART behind the ESP32-S3 USB bridge.
Listening only, it was clean; with CMDs at 100 Hz both ways the MCU counted 40-45 corrupted frames, STATE stopped within
2-4 s and the board stayed dead (bridge included: no 1200-baud touch) until a power cycle: at 2 Mbaud and again at
921600, so not the baud. Likely the R4 core's UART: receive errors are ignored and its blocking `write()` waits on a
transmit-complete callback (not isolated further). Native USB: 10-15 s of CMDs, 0 receive errors, no hang.

**Flashing with native USB.** The core's own 1200-baud bootloader jump left the board silent and unflashable (the
bootloader in front of the app at 0x4000 does not answer bossac on the native port), so it is off
(`-DNO_1200_BPS_RESET=1`). Instead `src/main.cpp` watches the line coding: on a touch it sends zero current to the real
motor, detaches its USB and drives the mux (P408) low; the bridge (2341:1002) appears and `flash_pi.sh` touches and
flashes it as usual. Checked Nov 12: reflashed from the native port with no button. Recovery if the board ever hangs on
the bridge firmware: unplug / replug; on native firmware that won't hand over: unplug, replug, double-press RESET.

## Run (attended)

```bash
bash firmware/bench/omni_fw_uno_r4/flash_pi.sh          # Git Bash on the PC: build, copy, flash through the Pi
ssh omni-cam '~/.venvs/cam/bin/python ~/fw_bench/link_probe.py [--cmds]'   # rates, mode, faults, build id, loop gaps
ssh omni-cam 'bash ~/fw_bench/ros_bench.sh'             # mcu_link + mode_manager + teleop_bench.py: the motor turns
ssh omni-cam 'bash ~/fw_bench/console_bench.sh start'   # + rosbridge, rosapi, the console: http://omni-cam.local:8080
ssh omni-cam 'bash ~/fw_bench/console_bench.sh stop'    # (status: what runs; logs in ~/fw_bench/console_run)
```

Flashing uses Arduino's `bossac` 1.9.1-arduino5 for linux-aarch64 in `~/tools/bossac` on the Pi (downloaded on the PC from
`downloads.arduino.cc/tools/`) through the bridge (above). Stop `console_bench.sh` first (it holds the port). The Pi needs
`omni_cpp` and `omni_console` built from this checkout (`OMNI_PI_PKGS="omni_cpp omni_console" bash
characterization/pi_camera/pi.sh build`; rebuild clean after a clock jump: rpi.md). Both scripts use ROS domain 44,
localhost discovery, and stop only the processes they started (process groups).

`teleop_bench.py` asks `/set_mode TELEOP` (kid profile) and plays yaw +0.5, yaw -1.0, vx +0.15 with rests between,
recording `/mcu/state` to `~/fw_bench/teleop_*.csv`. Wheel 1 is the real motor; the odometry is the pseudo-inverse over
all four wheels, so for a pure yaw a stopped real wheel reads 75% of the command and a tracking one 100%.

## Results (Nov 12, unloaded motor, bench supply)

| Phase | Command | Odometry (second half) | Ratio | Max slip |
| --- | --- | --- | --- | --- |
| yaw +0.5 | wz 0.5 rad/s (wheel 1: 1.04 rad/s) | wz 0.498 | 1.00 | 0.002 m/s |
| yaw -1.0 | wz -1.0 (wheel 1: -2.09 rad/s) | wz -1.000 | 1.00 | 0.002 |
| vx +0.15 | vx 0.15 m/s (wheel 1: -1.52 rad/s) | vx 0.150 | 1.00 | 0.002 |

DISARMED → AUTO on `/set_mode TELEOP`, back on IDLE; rests read zero; only IMU_SILENT. STATE ~180-197/s, INFO 10/s.

**Top speed (Nov 13, Test › Motors at full stick, unloaded, 24 V bench supply):** the command ramps at 20 rad/s² to
the motor test's cap, 44.88 rad/s (`MAX_WHEEL_SPEED`); the motor tracks it within 0.3 rad/s up to ~38 rad/s, then
holds **38.5 rad/s (368 rpm)**; letting go ramps both to zero together. The datasheet says 435 rpm no-load, 370 rpm
rated (design.md 2.1).

**Late ticks (Nov 13): the R4 is too slow for a 1 kHz tick; the link read was the worst part, now fixed.** The
`profile` build (`flash_pi.sh profile`, then `link_probe.py --cmds --profile --seconds 10`) reports each loop
section's longest time per 0.5 s as virtual wheels 2-4's temperatures (20 us steps). With DISARM CMDs at 100 Hz:

| Read path | Receive per CMD | Tick (inputs + core + outputs) | Transmit | Late ticks (> 1.5 ms) in 10 s | Longest gap |
| --- | --- | --- | --- | --- | --- |
| `Serial.readBytes()`, `Serial ?` (before) | 1280 us | 860-940 us | 540-660 us | 829 | 2680 us |
| TinyUSB bulk `tud_cdc_read()`, `tud_cdc_connected()` (now) | 460 us | 840-920 us | 500-660 us | 172 | 1859 us |

Listening only: 0-1 late ticks either way, median gap 1.07-1.10 ms. `readBytes()` goes one `read()` per byte, each
checking the connection, and `Serial ?` runs `tud_task()` (the whole USB service, which the core already runs in the
USB interrupt) every call. What remains is the shared core on a 48 MHz M4F: ~0.8 ms per tick and ~0.46 ms to take a
CMD (the COBS/CRC decode and handling), so tick + transmit alone run past 1 ms. The build links software double
routines (26 call sites; one is the camera-trigger schedule), not located further: the Teensy's 600 MHz M7 has a double
FPU, so this is a bench-only cost and the core is left as is. Control held (above): the core integrates the real dt.
