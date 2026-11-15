# CubeMars AK40-10 V3.0 KV170: the arm robot's drive motor

What we know about this motor from its manual, its settings and our benches, kept for future work. Design decisions
live in [design.md](../../design.md) 2.1 and 4.7; the sim's model in `simulation/v11 sil` (`catalog.DRIVE_MOTORS
["AK40-10"]`, `sil_robot.AkMotor`); the robot's driver in `firmware/lib/omni_fw/ak_motor.h`. Hardware facts below are
labelled **measured** (bench, with the date and report), **manual** (CubeMars) or **inference**.

## Files here

| File | What |
| --- | --- |
| `AK Series V3.0 Module Product Manual.pdf` / `.txt` | CubeMars' V3.0.0 manual (downloaded Nov 13 from cubemars.com); the `.txt` is its extracted text, for grep. Protocol: section 4; tool settings: 3.1 |
| `CubeMarsTool_V3.2.1.exe` | The CubeMars configuration tool (Windows; a VESC Tool derivative). Needs the motor on USB/UART through CubeMars' adapter |
| `Settings.png`, `settings motor 2.png` | Both motors' Application Configuration as set (below) |
| `Pinout.png` | The driver's connectors (manual 1.4.2) |
| `AK40-10 V3.0 KV170.STEP`, `AK40-10 with connector.STEP`, `XT30(2+2).STEP` | CAD: the motor, with its XT30(2+2) power + CAN plug |
| `ak48-2405-...installation-instructions.pdf` | A different CubeMars driver's sheet (kept for reference, not this motor) |

## Wiring

- **XT30(2+2):** red +, black −, blue CAN_L, white CAN_H. **Serial (3-pin):** black GND, yellow RX, green TX (the
  UART, 921600 baud as set; the tool's port and the only place the bus voltage is reported).
- CAN needs a **common ground** with the controller: on the benches the serial connector's GND to the transceiver's
  GND (without it the motor never ACKed, Nov 12). Termination: the robot's CAN is a daisy chain with both 120 Ω on
  the Teensy carrier (design.md 4.4), the motors' own termination off (VERIFY).

## Settings (CubeMars tool, Application Configuration; both motors, Nov 11–13)

| Setting | Value | Notes |
| --- | --- | --- |
| Mode | **Servo** (not MIT) | Why servo: design.md 2.1 / `ak_motor.h` |
| CAN Mode | Periodic Feedback | The other choice, **query-reply**: one status frame per command received |
| CAN Bitrate | 1 Mbps | |
| CAN Fdb Rate | **500 Hz** | The maximum (manual: 1–500 Hz) |
| CAN ID | 1, 2 | The firmware's wheel order (`DM_IDS` = 1–4) |
| Uart Bitrate | 921600 | |
| Timeout | 100 ms | No command for this long → the motor's timeout behaviour (brake current 0 = coast; VERIFY on comm loss) |

The tool also holds, under Advanced Settings (names from the tool's own strings, not yet read off a motor): the speed
loop's gains `s_pid_kp` / `s_pid_ki`, the switching frequency `foc_f_sw`, `send_can_status_rate_hz`, `timeout_msec`.
**Read and save every motor's advanced parameters (Export) before changing any** (manual 3.1.1.2 warns a write without
a read corrupts the others).

## CAN protocol (servo mode; manual 4.1, 4.3.1)

- Extended frames only, id = mode << 8 | motor id. Modes: 0 duty, 1 current (A × 1000), 2 brake current, 3 velocity
  (int32 ERPM), 4 position, 5 set origin (1 = permanent: writes flash), 6 position + velocity + **acceleration**
  (int16 × 10 ERPM/s², to 327,670 ERPM/s ≈ 234 rad/s² at the output).
- **Velocity mode uses a fixed "default maximum acceleration"** (manual 4.1): measured ~148 rad/s² at the output.
- **Status frame** (id 0x29 << 8 | id, 8 bytes): position int16 × 0.1° (wraps at ±3276.8°, ~9 output turns), speed
  int16 × 10 ERPM (**0.0068 rad/s** at the output), current int16 × 0.01 A, **temperature int8 °C of the driver
  board** (not the winding), error (0 ok, 1 motor over-temp, 2 over-current, 3 over-voltage, 4 under-voltage, 5
  encoder, 6 MOSFET over-temp, 7 stall).
- Scaling (measured Nov 12): ERPM = output rpm × 10 (gear) × 14 (pole pairs).
- **No bus voltage on CAN** (manual 4.3; MIT replies carry none either). The UART's `COMM_GET_VALUES` (0x45: frame
  `AA 01 45 18 61 BB`) returns input voltage (int16 / 10 V), input and phase current, MOSFET and **motor (winding)**
  temperatures, duty, Id/Iq, Vd/Vq (manual 4.3.2.1). Using it on the robot would take a UART per motor to the Teensy
  (3.3 V levels, the driver's VCC switch). The robot reads its pack on the Teensy's A17 divider instead.

## Measured (hardware, wheels up, 24 V bench supply)

| | Value | Source |
| --- | --- | --- |
| Top speed at 24 V, unloaded | 38.5 rad/s (368 rpm), at 0.2 A; voltage-limited, scales with the bus (~35.6 at 22.2 V, inference) | Nov 13, [wheel bench](../../simulation/v11%20sil/studies/results/wheel_bench_2026-11-13.md) |
| Acceleration in velocity mode | **147–150 rad/s²** whatever is asked (≈ 200k ERPM/s), two motors | Nov 13, wheel + [CAN bench](../../simulation/v11%20sil/studies/results/can_bench_2026-11-13.md) |
| Step 0 → 20 rad/s / reversal ±20 | 90 % in 0.13 s / 0.25 s; overshoot 0.25–0.6 rad/s | CAN bench |
| Friction current at steady speed | 0.077–0.089 + 0.0022–0.0035 \|ω\| A (two motors; one with a wheel) | CAN bench |
| Speed ripple at a hold / current noise at rest | sd 0.08–0.09 rad/s / sd 0.012–0.015 A, offset −0.05 to −0.07 A | CAN bench |
| J/k (A per rad/s² of acceleration) | bare 2.06 mA s²/rad; with the `cad/v13` wheel 3.86–4.1 | CAN + wheel bench |
| Torque per amp (reported current) | **≈ 0.48 N m/A (inference)**: the bare motor's J/k against the datasheet rotor (97.35 g cm², 0.97e-3 kg m² reflected). The sim's drive model uses 0.73 internally (chart fit). **VERIFY** with a known mass on the hub | CAN bench |
| Driver-board temperature, idle | 48 / 54 °C after ~20 min on the bench | CAN bench |
| Status broadcast | 500 Hz decodes; scaling and signs confirmed | Nov 12, [UNO R4 bench](../../firmware/bench/ak40_uno_r4/README.md) |

## Timing: where the delays are, and who owns them

Through the robot's baseline firmware (protocol 9), the motor's speed trails the firmware's asked speed by **8.6–9.0
ms on a ramp** (CAN bench, two motors). Mean budget, and whether it can change:

| Delay (mean, range) | Owner | Can it change? |
| --- | --- | --- |
| Robot computer → MCU: CMD every 10 ms (mean 5) | **Ours** (`mcu_link`, protocol) | Yes: a faster CMD; it's also the heartbeat |
| Round-robin wait for the motor's turn: 1.5 ms (0–3) | **Ours** (firmware: one frame a 1 ms tick, 4 wheels) | Yes: two frames a tick → 0.5 ms (bus ~56 %); every motor every tick → 0 (bus ~84 %: too full) |
| Frame on the wire: 0.15 ms | Physics (1 Mbit/s classic CAN) | No (the motor has no CAN FD) |
| **Inside the motor: ~5.5 ms** (fit; 5.5–7.5 on steps) | **The motor**, split unknown: fixed processing, its speed loop (gains `s_pid_kp/ki`: a setting), or reporting late | **Not established.** Its loop part is tunable; a fixed processing part isn't. See "Open" |
| Status age: ~1.2 ms (0–2 + frame) | Motor setting (500 Hz periodic, the maximum) | Partly: query-reply mode makes the status follow each command instead of a free-running 500 Hz |
| MCU picks the status up at its next tick: ~0.5 ms (0–1) | **Ours** (firmware) | Yes (read on the CAN interrupt), but control acts on the tick anyway |
| MCU → robot computer: STATE every 5 ms (mean 2.5) | **Ours** (`STATE_PERIOD_US`) | Yes: USB has room for far more |
| Acceleration cap 148 rad/s² | The motor's velocity mode | Not in velocity mode as far as the tool shows (no ramp parameter); position-velocity mode takes up to ~234 rad/s², current mode none. 3× what the robot asks: not a limit today |

So **not all of it is intrinsic**. Of the ~9 ms on the wheel side, ~2 ms is our firmware's scheduling and ~1–1.5 ms
the 500 Hz status, both changeable; the host links (CMD 100 Hz, STATE 200 Hz) add more, also ours. The ~5.5 ms inside
the motor is the largest piece, and whether it is fixed is open.

**In the sim (Nov 13):** `SilRobot(ak_latency=0.0055)` delays each frame's effect 5.5 ms (`ak_latency=0` reproduces
earlier runs); the 500 Hz status, the firmware's round robin (the SIL runs the firmware itself) and the 148 rad/s²
ramp (`DriveMotor.accel_limit`) were already there.

## Open (in order of value)

1. **Split the motor's ~5.5 ms** (needs motion, attended; the baseline firmware gets a current-mode option in the
   motor test rather than a side sketch): a small **current** step. If the reported current responds within ~1–2 ms
   of its frame, the 5.5 ms lives in the velocity loop (tunable `s_pid_kp/ki`, or avoidable); if it is ~5 ms late
   too, it's fixed processing or reporting. Then raise `s_pid_kp` in the tool and repeat the 50 rad/s² triangles.
2. **Query-reply mode** on one motor: the reply's arrival after its command bounds the turnaround and gives status at
   the command rate, aligned (no firmware change needed to try it; the firmware's feedback timeout and odometry rate
   then follow the 250 Hz command rate).
3. **Torque per amp** with a known mass on the hub (0.5 kg at 70 mm = 0.34 N m reads 0.72 A at 0.48 N m/A, 0.47 A at
   0.73); then the sim's drive model's current limit and heating.
4. Under load and on the pack: the torque-speed curve, top speed at 25.2 / 22.2 / 21 V, heating in the printed hub.
5. Read the advanced parameters off each motor (Export) and file them here: speed gains, current limits, ERPM limit,
   switching frequency.
6. Comm loss with the 100 ms timeout (coast or brake), the position wrap, error clearing.
