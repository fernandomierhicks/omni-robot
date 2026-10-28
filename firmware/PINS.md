# Teensy 4.1 pin map

Generated from `lib/omni_fw/board_teensy41.h` by `sil/build_sil.py --pins` (the header's static_asserts fail the build on a clash, a pin that can't do the job, or a missing signal). design.md section 30.

| Pin | Signal |
| --- | --- |
| 2 | DShot ch1: flywheel L (FlexPWM4) |
| 3 | DShot ch2: flywheel R (FlexPWM4) |
| 4 | buzzer (start-of-motion chirp, fire warning) |
| 5 | BNO085 PS0/WAKE |
| 6 | BNO085 RST |
| 7 | Serial2 RX: ELRS receiver (CRSF) |
| 8 | Serial2 TX: ELRS receiver (CRSF telemetry) |
| 9 | BNO085 INT (data ready, interrupt) |
| 10 | BNO085 CS |
| 11 | SPI0 MOSI: BNO085 |
| 12 | SPI0 MISO: BNO085 |
| 13 | SPI0 SCK: BNO085 (also the on-board LED: it flickers) |
| 14 | Serial3 TX: RoboClaw (feed motors) |
| 15 | Serial3 RX: RoboClaw (feed motors) |
| 17 | Serial4 TX (half duplex): STS3250 tilt servo bus |
| 22 | CAN1 TX: DM-2325 x4 |
| 23 | CAN1 RX: DM-2325 x4 |
| 24 | CAM_TRIG: both OV9782 FSIN (external trigger, 120 Hz; level shift to DOVDD, VERIFY) |
| 25 | SYNC_OUT: 1 Hz pulse to the Jetson's 40-pin GPIO (AON pin for HTE stamps, VERIFY) |
| 26 | CAM_STROBE: left OV9782 STROBE in (exposure active), interrupt on both edges |
| 27 | BNO085 BOOTN (low at reset: its bootloader, for firmware updates) |
| 28 | status LED |
| 29 | BNO085 PS1 (high at reset with PS0: SPI mode) |
| 30 | CAN3 RX: payload connector, reserved |
| 31 | CAN3 TX: payload connector (module id), reserved |
| 32 | feed-exit beam receiver |
| 33 | catch beam receiver |
| 35 | e-stop chain sense (relay open = 1) |
| 36 | IR emitters, 38 kHz (both beams) |
| 37 | bumper switches, front (wired-OR) |
| 38 | bumper switches, rear (wired-OR) |
| 39 | A15: battery current (hall sensor) |
| 40 | A16: 12 V rail |
| 41 | A17: battery voltage (divider) |

34 of 42 header pins used, 8 spare: 0 1 16 18 19 20 21 34
