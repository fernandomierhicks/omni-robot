# firmware/: the MCU (Teensy 4.1)

The robot's real-time microcontroller (design.md 2 and 2.1; history §§14, 30). It is the safety authority under the robot computer (still open; older text says "the Jetson"): it drives the four wheels (the arm robot's CubeMars AK40-10s, the baseline; the parked catch robot's DM-2325s), the catch robot's flywheels, feed and tilt, and it enforces every limit. The computer only asks.

**The same code runs in two places.** `lib/omni_fw/` (the core) has no Arduino in it:

- on the Teensy, `src/main.cpp` wraps it with the drivers;
- in the MuJoCo sim, `sil/` builds it into a library that `simulation/v11 sil/sil_robot.py` ticks every 1 ms of sim time (software in the loop, SIL).

So the sim, the ROS stack and the tests exercise the firmware that will be flashed.

## Layout

| Path | What |
| --- | --- |
| `lib/omni_fw/firmware.h/.cpp` | The top level: the 1 ms tick, modes (DISARMED, AUTO, MANUAL, ESTOP), the motor test (`DRIVE_WHEELS`: each wheel its own speed, capped and ramped, protocol 7; protocol 9: at the CMD's `a_brake` ramp up to `WHEEL_TEST_ACCEL_MAX` 2000 rad/s², else 20, and STATE `wheel_cur` carries each motor's current; a silent motor fault-stops only when the base needs it: driving, or that wheel being turned), safety rules (a CMD must pass `cmd_ok`: this protocol version, known enums, finite values in range; leaving MANUAL ramps to rest before disabling; kid off only at rest; bumpers only with `HAS_BUMPERS`), the CMD / STATE link |
| `lib/omni_fw/base.h/.cpp` | X4 kinematics, the jerk-limited, direction-aware velocity controller, the braking-curve goto with an end speed (the moving catch), and wheel + BNO085 odometry. A port of the sim's `omni_control.py` |
| `lib/omni_fw/module.h/.cpp` | Flywheel rpm loops (PI + back-EMF feed-forward on DShot eRPM) and the chute feed's one-ball-per-trigger sequence |
| `lib/omni_fw/sh2.h/.cpp` | An SH-2 report parser (a batch's samples, each with its own time), tested in SIL on built packets. Since Nov 17 the Teensy reads the BNO085 with CEVA's SH-2 library instead (`lib/Adafruit_BNO08x`, `src/main.cpp`) |
| `lib/Adafruit_BNO08x/` | CEVA's SH-2 + Adafruit's BNO08x wrapper, vendored and patched (bounded handshakes, report FIFO, INT-edge stamps): its README |
| `lib/omni_fw/dm_motor.h/.cpp` | Damiao DM-2325 CAN protocol, velocity mode: enable, disable, clear, speed frames, feedback decode (the catch variant) |
| `lib/omni_fw/ak_motor.h/.cpp` | CubeMars AK-series CAN protocol, **servo mode** (the arm variant's AK40-10s): extended frames, velocity in ERPM, zero current to coast, the 500 Hz status broadcast. Why not MIT mode, and the wheeled-leg robot's lessons, are in the header. One frame a tick, the motors in turn; odometry from the motors' reported speed |
| `lib/omni_fw/protocol.h/.cpp` | Computer link: COBS + CRC-16 frames; CMD (100 Hz, also the heartbeat), STATE (200 Hz), IMU (every BNO085 sample: 100 Hz since Nov 17, on the Teensy and in the sim), INFO (10 Hz: `PROTOCOL_VERSION` 9 (9: STATE `wheel_cur`, the motor test's asked ramp; 8: STATE `dm_err` 0xFF `DM_NO_REPLY`, a motor not answering), the build id, the RC channels, each drive motor's winding and driver temperature from its CAN feedback), TIME (each sync pulse, camera trigger and strobe, at its MCU time: design.md 10). Python mirror: `ros/src/omni_mcu/omni_mcu/protocol.py` |
| `fw_build_id.py` | The build id (a hash of `lib/omni_fw`): compiled in as `FW_BUILD` by PlatformIO (`extra_scripts`) and the SIL build; the console compares it with its own copy's |
| `lib/omni_fw/hw_io.h` | The hardware boundary: `FwHwInputs` / `FwHwOutputs`, the only things that cross between the core and a board |
| `lib/omni_fw/board_teensy41.h` | The pin map, **checked at compile time** (see below) |
| `lib/omni_fw/fw_params.h`, `fw_params_arm.h` | **Generated** from the sim's `SimParams` by `simulation/v11 sil/robot_export.py`: the catch robot's and the arm robot's kinematics, limits per profile, flywheel loop, feed, tilt range, the motors' CAN ids and ranges, `HAS_BUMPERS`, `DRIVE_DAMIAO`. Don't edit by hand. `fw_params_select.h` picks one (`-DFW_PARAMS_HEADER`, set per PlatformIO env) |
| `src/main.cpp` | The Teensy layer (drivers, 1 kHz loop) |
| `sil/` | `sil_api.cpp` (C entry points), `build_sil.py` (zig c++ on Windows, g++ in WSL), `omni_fw_sil.py` (ctypes) |
| `PINS.md` | The pin table, written from `board_teensy41.h` (`python sil/build_sil.py --pins`) |

## The pin map is checked

`board_teensy41.h` lists every signal, its pin and what that pin must do (a fixed peripheral pin such as CAN1 TX = 22, PWM, analog, or plain GPIO). Its `static_assert`s fail the build when:

- two signals share a pin;
- a signal is on a pin that can't do the job (for example, Serial3 RX not on pin 15, or the battery divider not on an analog pin);
- a signal the robot needs has no pin;
- fewer than 4 header pins are left spare.

Today 34 of the 42 header pins are used and 8 are spare. Both the Teensy build and the SIL build compile it. [PINS.md](PINS.md) has the table.

## Build, flash, test

```powershell
cd firmware
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e catch   # Teensy 4.1, the catch variant: builds today
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run            # env arm (the default): the AK40-10s
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e arm_bench -t upload --upload-port COM9   # a bare bench
#   Teensy (Nov 13): e-stop input ignored, fixed 24 V reported (neither wired); never on a robot. Attended only
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -t upload  # attended sessions only (design.md 5)
& $py sil\build_sil.py --pins                                  # the SIL library + PINS.md
```

The build fetches two libraries (`lib_deps`: Adafruit BusIO, Unified Sensor, for the BNO085). PlatformIO's lock file fails inside Dropbox (`[Errno 22] ... .lock`), so set `PLATFORMIO_WORKSPACE_DIR` to a folder outside it (e.g. `$env:USERPROFILE\pio_ws\omni_fw`); the build output then lands there.

**The Teensy on the Pi** (attended): `bash firmware/flash_pi.sh [arm_bench|arm|catch]` from Git Bash builds, copies the hex, stops the console's robot step if it runs (mcu_link holds the port), flashes with `teensy_loader_cli` and starts the step again with the same arguments.

Or click **Build firmware** in `Launcher.cmd`. The firmware tests are in `simulation/v11 sil/test_v11.py`; they need no hardware.

## What is done and what is not

| Part | State |
| --- | --- |
| Core: modes, watchdog, ramped stops, bumper, e-stop, kid / adult profiles, base control, odometry, flywheel loop, feed, tilt, link, DM-2325 CAN | Written and tested in SIL against the sim (design.md 30) |
| Teensy: CAN1 (FlexCAN_T4), USB link, beams, e-stop, bumpers (normally closed: a broken wire reads as pressed; none fitted on the arm robot yet), battery ADC, buzzer, LED, WDOG1 watchdog (0.5 s) | CAN1 + USB link on hardware (Nov 13, env `arm_bench`: one AK40-10 driven and read through the robot's stack, 200 STATE/s); protocol 9 flashed on the bench Teensy Nov 13; two AK40-10s timed on CAN the same day (design.md 4.7). The rest written, not yet on hardware |
| CubeMars AK40-10 CAN, servo mode (the arm robot's drives) | Written Nov 7 from the protocol as the wheeled-leg robot's AK45-10 scripts use it (the CubeMars manual was out of reach); builds for the Teensy; the v11 SIL's `AkMotor` speaks it. **Codec run on a real AK40-10 Nov 12** (UNO R4 bench, [bench/ak40_uno_r4](bench/ak40_uno_r4/README.md)): 500 Hz status decodes, velocity in both directions, ERPM scaling (14 pole pairs × 10) confirmed unloaded. The whole chain too: this core on the R4 ([bench/omni_fw_uno_r4](bench/omni_fw_uno_r4/README.md)) driven by `mcu_link` + `mode_manager` on the Pi, TELEOP odometry 1.00× the command. **VERIFY on the Teensy**: the position wrap, ERPM limit, error clearing, comm-loss timeout (console Test > Bring-up) |
| Teensy: RoboClaw packet serial (feed speeds), STS3250 bus (tilt goal and position) | Written from the protocols, builds; **VERIFY on the bench** |
| Teensy: BNO085 over SPI (Nov 17: the wheeled-leg robot's proven driver ported, `Bno085` in `src/main.cpp`, on CEVA's SH-2 through the vendored, patched [lib/Adafruit_BNO08x](lib/Adafruit_BNO08x/README.md)). The bench module has PS0/WAKE and PS1 strapped high, so reset-as-wake handshakes; game RV + calibrated gyro + accelerometer at 100 Hz each; each sample stamped at the INT edge less its SH-2 delay; stalls and sensor resets recovered without a reboot (heading kept across a reset) | **On hardware Nov 17**: that robot's `test_imu` passed on this Teensy and module (at its own 400 Hz, 150 ms stall and forced reset recovered, 0 errors); the robot firmware (`arm_bench`) flashed through the Pi (`flash_pi.sh`) gives `/imu` at 100.3 Hz from `mcu_link`, no IMU_SILENT; console Test › IMU draws it. `lib/omni_fw/sh2.*` (our own SH-2 parser) is now used by the SIL tests only |
| Teensy: time references (design.md 10): SYNC_OUT (pin 25) and CAM_TRIG (pin 24) on one-shot PIT timers at the core's scheduled times, CAM_STROBE (pin 26) on both edges, CAN replies stamped by FlexCAN's timer, USB bytes stamped on read | Written, builds; **VERIFY** (FlexCAN's capture point, the PIT edges on a scope) |
| Teensy: bidirectional DShot, CRSF (ELRS) | **Stubs** (marked TODO in `src/main.cpp`). Without DShot the flywheels don't run; without CRSF, MANUAL mode can't be reached |
| Terrain assist (design.md 25.4: the edge map) | Not ported to the firmware (the house runs without it under SIL) |

Values marked VERIFY in `fw_params.h` and `main.cpp` (the DM-2325's CAN ranges and ids, its comm-loss timeout, mounting directions, the F90's pole pairs, the RoboClaw's counts per revolution, the servo zero, the battery divider) must be checked on the real parts.
