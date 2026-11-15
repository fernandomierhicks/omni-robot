# AK40-10 CAN bench on an Arduino UNO R4 WiFi

A temporary stand-in for the Teensy 4.1: one CubeMars AK40-10 V3.0 on CAN, driven and read with the robot's own
servo-mode codec ([`lib/omni_fw/ak_motor.cpp`](../../lib/omni_fw/ak_motor.cpp), compiled in unchanged by
`src/ak_codec.cpp`). Not robot firmware: no safety core, no USB protocol; a serial command line for bring-up.

## Wiring

| UNO R4 WiFi | Transceiver board | Motor |
| --- | --- | --- |
| D10 (CANTX, P103) | TX / CTX / TXD (its input) | |
| D13 (CANRX, P102) | RX / CRX / RXD (its output) | |
| 5 V / GND | VCC / GND | |
| GND | GND | CAN connector GND: **required** (without it the motor never ACKed) |
| | CANH / CANL | CANH / CANL |

TX → TX and RX → RX: transceiver pins are named for their own function, not crossed like a UART.

Motor settings (CubeMars tool, Nov 12): servo mode, CAN ID 1, 1 Mbit/s, periodic feedback at 500 Hz, timeout 100 ms.

## Use

```powershell
cd firmware\bench\ak40_uno_r4
& $env:USERPROFILE\.platformio\penv\Scripts\pio.exe run -t upload --upload-port COM8      # attended only
& $env:USERPROFILE\.platformio\penv\Scripts\python.exe ak_bench.py --port COM8 "listen 1" "v 2000 2000" "listen 3"
```

`ak_bench.py` uses PlatformIO's interpreter only because it already has pyserial (nothing is installed into it).
`pio device monitor` works too; `?` lists the commands (velocity `v`/`w`, current `c`, brake `b`, position+speed `pv`,
temporary origin `o`, stop `x`, raw frames `t`/`ts`, diagnostics `d`/`dt`/`dp`). Bench caps: 15000 ERPM, 4 A,
60 s; every motion command streams at 100 Hz for its duration, then sends zero current (coast).

## Findings (Nov 12, unloaded motor, bench supply)

- **R4 start-up quirk:** straight after `CAN.begin`, CANRX sat low (dominant) for good, so the controller never saw an
  idle bus and never sent or received. Driving D10 high as GPIO for 5 ms and handing it back to the CAN peripheral
  releases it; `setup()` does this. The Teensy's FlexCAN is a different controller: not expected there.
- **Feedback:** the servo-mode status broadcast (`0x2901`) arrives at 500 Hz and decodes with `ak_decode_status`
  (position, ERPM, current, temperature, error).
- **Scaling confirmed:** `v 2000` for 2.0 s moved the output 136.0° → 307.3° (171.3°; expected 1.496 rad/s × 2 s =
  171.4°), so ERPM = output rpm × 10 × 14 pole pairs and the status position is output-shaft degrees. Reverse
  (`w -1.5`) signs agree. Unloaded current 0.0–0.2 A, 34 °C, no errors.
- Still VERIFY: the position wrap, the ERPM limit, error clearing, speed-loop gains under load, the 100 ms timeout's
  behaviour on comm loss.
