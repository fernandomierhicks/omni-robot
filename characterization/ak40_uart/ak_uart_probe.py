"""Read-only AK40-10 UART probe: COMM_GET_VALUES and GET_VALUES_SETUP polls, no motion commands.

python ak_uart_probe.py COM7 [n_polls]
"""
import struct
import sys
import time

import serial


def crc16(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if c & 0x8000 else (c << 1)
            c &= 0xFFFF
    return c


def frame(payload: bytes) -> bytes:
    return bytes([0xAA, len(payload)]) + payload + struct.pack(">H", crc16(payload)) + b"\xBB"


assert frame(b"\x45") == bytes.fromhex("AA014518 61BB".replace(" ", "")), frame(b"\x45").hex()
assert frame(bytes.fromhex("1300000001")) == bytes.fromhex("AA05130000 0001FAA9BB".replace(" ", ""))


def read_frame(ser: serial.Serial, timeout=0.05):
    t_end = time.perf_counter() + timeout
    buf = bytearray()
    while time.perf_counter() < t_end:
        b = ser.read(1)
        if not b:
            continue
        if not buf and b[0] != 0xAA:
            continue
        buf += b
        if len(buf) >= 2 and len(buf) == buf[1] + 5:
            payload = bytes(buf[2:-3])
            ok = struct.unpack(">H", buf[-3:-1])[0] == crc16(payload) and buf[-1] == 0xBB
            return payload, ok, len(buf)
    return None, False, len(buf)


def parse_values(p: bytes) -> dict:
    i = 1
    def g(fmt):
        nonlocal i
        v = struct.unpack_from(fmt, p, i)[0]
        i += struct.calcsize(fmt)
        return v
    d = {}
    d["mos_temp_C"] = g(">h") / 10
    d["motor_temp_C"] = g(">h") / 10
    d["out_current_A"] = g(">i") / 100
    d["in_current_A"] = g(">i") / 100
    d["id_A"] = g(">i") / 100
    d["iq_A"] = g(">i") / 100
    d["duty"] = g(">h") / 1000
    d["erpm"] = g(">i")
    d["v_in"] = g(">h") / 10
    i += 24
    d["fault"] = g(">B")
    d["pos_deg"] = g(">i") / 1000
    d["id"] = g(">B")
    i += 6
    d["vd"] = g(">i") / 1000
    d["vq"] = g(">i") / 1000
    return d


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 200
    ser = serial.Serial(port, 921600, timeout=0.002)
    ser.reset_input_buffer()

    # 1) full COMM_GET_VALUES, a few decoded
    q = frame(b"\x45")
    payload, ok, ln = None, False, 0
    for _ in range(3):
        ser.write(q)
        payload, ok, ln = read_frame(ser)
        if payload:
            break
    if not payload:
        print("no reply to COMM_GET_VALUES (wrong port/baud, tool holding it, or motor unpowered)")
        return
    print(f"GET_VALUES reply {ln} bytes, crc ok={ok}, raw={payload.hex()}")
    for k, v in parse_values(payload).items():
        print(f"  {k:14s} {v}")

    # 2) round-trip timing, full and masked
    masked = struct.pack(">BI", 0x13, (1 << 0) | (1 << 1) | (1 << 3) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 15))
    for name, q in (("GET_VALUES (78 B reply)", frame(b"\x45")), ("SETUP mask: temps, Iin, Iq, duty, speed, Vin, fault", frame(masked))):
        rtts, bad, miss, lens = [], 0, 0, set()
        temps = []
        for _ in range(n):
            ser.reset_input_buffer()
            t0 = time.perf_counter()
            ser.write(q)
            p, ok, ln = read_frame(ser)
            if p is None:
                miss += 1
                continue
            rtts.append((time.perf_counter() - t0) * 1e3)
            lens.add(ln)
            bad += not ok
            if p[0] == 0x45:
                temps.append(parse_values(p)["motor_temp_C"])
        rtts.sort()
        if rtts:
            print(f"{name}: n={len(rtts)} miss={miss} crc_bad={bad} reply_len={sorted(lens)} "
                  f"rtt ms median {rtts[len(rtts)//2]:.2f} p95 {rtts[int(len(rtts)*0.95)]:.2f} min {rtts[0]:.2f}")
        else:
            print(f"{name}: no replies ({miss} missed)")
        if temps:
            print(f"  motor_temp spread {min(temps)} .. {max(temps)}")
    # one decoded masked reply
    ser.reset_input_buffer()
    ser.write(frame(masked))
    p, ok, ln = read_frame(ser)
    print("masked reply raw:", p.hex() if p else None, "crc ok", ok)
    ser.close()


if __name__ == "__main__":
    main()
