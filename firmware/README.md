# firmware/: the MCU (Teensy 4.1)

The robot's real-time microcontroller (design.md sections 4, 14, 30). It is the safety authority under the Jetson: it drives the four DM-2325 wheels, the flywheels, the feed and the tilt, and it enforces every limit. The Jetson only asks.

**The same code runs in two places.** `lib/omni_fw/` (the core) has no Arduino in it:

- on the Teensy, `src/main.cpp` wraps it with the drivers;
- in the MuJoCo sim, `sil/` builds it into a library that `simulation/v11 sil/sil_robot.py` ticks every 1 ms of sim time (software in the loop, SIL).

So the sim, the ROS stack and the tests exercise the firmware that will be flashed.

## Layout

| Path | What |
| --- | --- |
| `lib/omni_fw/firmware.h/.cpp` | The top level: the 1 ms tick, modes (DISARMED, AUTO, MANUAL, ESTOP), safety rules, the CMD / STATE link |
| `lib/omni_fw/base.h/.cpp` | X4 kinematics, the jerk-limited, direction-aware velocity controller, the braking-curve goto with an end speed (the moving catch), and wheel + BNO085 odometry. A port of the sim's `omni_control.py` |
| `lib/omni_fw/module.h/.cpp` | Flywheel rpm loops (PI + back-EMF feed-forward on DShot eRPM) and the chute feed's one-ball-per-trigger sequence |
| `lib/omni_fw/dm_motor.h/.cpp` | Damiao DM-2325 CAN protocol, velocity mode: enable, disable, clear, speed frames, feedback decode |
| `lib/omni_fw/protocol.h/.cpp` | Jetson link: COBS + CRC-16 frames; CMD (100 Hz, also the heartbeat), STATE (200 Hz), IMU (400 Hz). Python mirror: `ros/src/omni_mcu/omni_mcu/protocol.py` |
| `lib/omni_fw/hw_io.h` | The hardware boundary: `FwHwInputs` / `FwHwOutputs`, the only things that cross between the core and a board |
| `lib/omni_fw/board_teensy41.h` | The pin map, **checked at compile time** (see below) |
| `lib/omni_fw/fw_params.h` | **Generated** from the sim's `SimParams` by `simulation/v11 sil/robot_export.py`: kinematics, limits per profile, flywheel loop, feed, tilt range, the motors' CAN ids and ranges. Don't edit by hand |
| `src/main.cpp` | The Teensy layer (drivers, 1 kHz loop) |
| `sil/` | `sil_api.cpp` (C entry points), `build_sil.py` (zig c++ on Windows, g++ in WSL), `omni_fw_sil.py` (ctypes) |
| `PINS.md` | The pin table, written from `board_teensy41.h` (`python sil/build_sil.py --pins`) |

## The pin map is checked

`board_teensy41.h` lists every signal, its pin and what that pin must do (a fixed peripheral pin such as CAN1 TX = 22, PWM, analog, or plain GPIO). Its `static_assert`s fail the build when:

- two signals share a pin;
- a signal is on a pin that can't do the job (for example, Serial3 RX not on pin 15, or the battery divider not on an analog pin);
- a signal the robot needs has no pin;
- fewer than 4 header pins are left spare.

Today 29 of the 42 header pins are used and 13 are spare. Both the Teensy build and the SIL build compile it. [PINS.md](PINS.md) has the table.

## Build, flash, test

```powershell
cd firmware
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run      # Teensy 4.1: ~44 KB flash, ~24 KB RAM1 variables
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -t upload   # attended sessions only (design.md 5)
& $py sil\build_sil.py --pins                                  # the SIL library + PINS.md
```

Or click **Build firmware** in `Launcher.cmd`. The firmware tests are in `simulation/v11 sil/test_v11.py`; they need no hardware.

## What is done and what is not

| Part | State |
| --- | --- |
| Core: modes, watchdog, ramped stops, bumper, e-stop, kid / adult profiles, base control, odometry, flywheel loop, feed, tilt, link, DM-2325 CAN | Written and tested in SIL against the sim (design.md 30) |
| Teensy: CAN1 (FlexCAN_T4), USB link, beams, e-stop, bumpers, battery ADC, buzzer, LED | Written, builds; not yet on hardware |
| Teensy: RoboClaw packet serial (feed speeds), STS3250 bus (tilt goal and position) | Written from the protocols, builds; **VERIFY on the bench** |
| Teensy: BNO085 (SH-2 over SPI), bidirectional DShot, CRSF (ELRS) | **Stubs** (marked TODO in `src/main.cpp`). Without the IMU the core uses the wheels' yaw rate; without DShot the flywheels don't run; without CRSF, MANUAL mode can't be reached |
| Terrain assist (design.md 25.4: the edge map) | Not ported to the firmware (the house runs without it under SIL) |

Values marked VERIFY in `fw_params.h` and `main.cpp` (the DM-2325's CAN ranges and ids, its comm-loss timeout, mounting directions, the F90's pole pairs, the RoboClaw's counts per revolution, the servo zero, the battery divider) must be checked on the real parts.
