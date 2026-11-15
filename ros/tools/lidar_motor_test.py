"""Which signal stops an RPLIDAR's motor: DTR, the SET_PWM command, or both? (Its adapter decides; A1 boards switch
the motor with DTR, A2 boards also take a PWM.) A scan only yields turns while the motor turns, so each step sets the
motor lines, waits for the motor to settle, and counts turns in a short scan; no one has to watch.

    ~/.venvs/cam/bin/python lidar_motor_test.py [/dev/ttyUSB0]
    ~/.venvs/cam/bin/python lidar_motor_test.py --after-server [/dev/ttyUSB0]   # is it still turning? (one scan)

Run with the port free (leave the console's Lidar page: its server lets go of the port 20 s after the last request;
the robot's ROS driver must not be running). Ends with the motor off (DTR cleared, PWM 0), then closes the port.

Nov 13, the A2M8 (firmware 1.26) on its USB adapter: DTR cleared + PWM 0 stopped it; DTR raised kept it at ~9.5
turns/s whatever the PWM (rplidar.py's motor control follows from this).
"""
import struct
import sys
import time

import serial

STOP, SCAN, SET_PWM = 0x25, 0x20, 0xF0


def request(cmd: int, payload: bytes = b"") -> bytes:
    if not payload:
        return bytes([0xA5, cmd])
    b = bytes([0xA5, cmd, len(payload)]) + payload
    cs = 0
    for x in b:
        cs ^= x
    return b + bytes([cs])


def turns(s: serial.Serial, seconds: float = 2.0) -> tuple[int, float]:
    """(samples, turns a second) in a standard scan of `seconds`."""
    s.reset_input_buffer()
    s.write(request(SCAN))
    buf, t0 = b"", time.monotonic()
    while time.monotonic() - t0 < seconds:
        buf += s.read(4096)
    s.write(request(STOP))
    time.sleep(0.05)
    s.reset_input_buffer()
    if len(buf) < 7 or buf[:2] != b"\xa5\x5a":
        return 0, 0.0
    n = starts = 0
    i = 7
    while i + 5 <= len(buf):
        b0, b1 = buf[i], buf[i + 1]
        if ((b0 & 1) != ((b0 >> 1) & 1)) and (b1 & 1):        # S and not-S differ, the check bit set
            n += 1
            starts += b0 & 1
            i += 5
        else:
            i += 1                                            # resync
    return n, starts / seconds


def motor(s: serial.Serial, dtr: bool, pwm: int):
    s.dtr = dtr
    s.write(request(SET_PWM, struct.pack("<H", pwm)))


def show(name: str, n: int, hz: float):
    print(f"{name:48s} -> {n:5d} samples, {hz:4.1f} turns/s  => {'TURNING' if hz > 2 else 'stopped'}")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    port = args[0] if args else "/dev/ttyUSB0"
    s = serial.Serial(None, 115200, timeout=0.2)
    s.port, s.dtr = port, False                                # opened with DTR cleared: raised runs an A2's motor
    s.open()
    with s:
        s.write(request(STOP))
        time.sleep(0.1)
        s.reset_input_buffer()
        if "--after-server" in sys.argv:                        # the motor as the last user left it (PWM untouched)
            show("as left (DTR cleared at open, PWM untouched)", *turns(s))
            return 0
        steps = [
            ("on: DTR cleared, PWM 660 (the SDK's start)", False, 660),
            ("PWM 0 only (DTR still cleared)", False, 0),
            ("on again", False, 660),
            ("DTR raised only (PWM still 660)", True, 660),
            ("on again", False, 660),
            ("both: DTR raised, PWM 0 (the old console stop)", True, 0),
        ]
        for name, dtr, pwm in steps:
            motor(s, dtr, pwm)
            time.sleep(3.0)                                    # spin up / run down
            show(name, *turns(s))
        motor(s, False, 0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
