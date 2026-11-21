"""Read-only throughput test: pipelined telemetry requests at fixed rates, count valid replies (the FTDI latency
timer adds delay but not a rate limit). python ak_uart_rate.py COM7"""
import struct
import sys
import threading
import time

import serial

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
from ak_uart_probe import crc16, frame  # noqa: E402


def count_frames(buf: bytes):
    i, good, bad = 0, 0, 0
    while i < len(buf):
        if buf[i] != 0xAA or i + 2 > len(buf):
            i += 1
            continue
        ln = buf[i + 1] + 5
        if i + ln > len(buf):
            break
        f = buf[i:i + ln]
        p = f[2:-3]
        if f[-1] == 0xBB and struct.unpack(">H", f[-3:-1])[0] == crc16(p):
            good += 1
            i += ln
        else:
            bad += 1
            i += 1
    return good, bad


def run(ser, q, hz, secs=2.0):
    ser.reset_input_buffer()
    rx = bytearray()
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            rx.extend(ser.read(4096))
    t = threading.Thread(target=reader)
    t.start()
    period = 1.0 / hz
    n = int(secs * hz)
    t0 = time.perf_counter()
    for k in range(n):
        while time.perf_counter() < t0 + k * period:
            pass
        ser.write(q)
    sent_dt = time.perf_counter() - t0
    time.sleep(0.1)
    stop.set()
    t.join()
    good, bad = count_frames(bytes(rx))
    return n, good, bad, sent_dt


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    ser = serial.Serial(port, 921600, timeout=0.01)
    masked = frame(struct.pack(">BI", 0x13, 0x81EB))
    full = frame(b"\x45")
    for name, q in (("masked 31 B", masked), ("full 90 B", full)):
        for hz in (200, 500, 1000, 2000):
            n, good, bad, dt = run(ser, q, hz)
            print(f"{name:12s} asked {hz:5d} Hz: sent {n} in {dt:.2f} s, replies {good} ({good / dt:.0f}/s, "
                  f"{100 * good / n:.0f}%), bad {bad}")
    ser.close()


if __name__ == "__main__":
    main()
