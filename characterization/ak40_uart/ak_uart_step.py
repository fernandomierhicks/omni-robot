"""Gentle UART velocity step, telemetry requests tagged by a rotating field mask (echoed by the motor) so dropped
replies are located. Every 1 ms: COMM_SET_RPM (or current 0 to coast) + a masked GET_VALUES_SETUP.
0 -> 10 rad/s output at 0.2 s, -> 0 at 0.8 s, coast at 1.3 s.  python ak_uart_step.py COM7 out.csv [gap_us]"""
import struct
import sys
import threading
import time

import serial

sys.path.insert(0, __file__.rsplit("\\", 1)[0])
from ak_uart_probe import crc16, frame  # noqa: E402

GEAR, PP = 10, 14
ERPM_PER_RAD_S = 9.5492966 * GEAR * PP
BASE = 0x81EB                                  # mos T, motor T, I in, Iq, duty, speed, V in, fault
TAG_BITS = (1 << 2, 1 << 16, 1 << 17)          # output current, position, id: toggled as a 3-bit tag
SIZES = {0: ("h", 10), 1: ("h", 10), 2: ("i", 100), 3: ("i", 100), 4: ("i", 100), 5: ("i", 100), 6: ("h", 1000),
         7: ("i", 1), 8: ("h", 10), 15: ("B", 1), 16: ("i", 1000), 17: ("B", 1)}
NAMES = {0: "mos", 1: "motor_T", 2: "i_out", 3: "i_in", 4: "id", 5: "iq", 6: "duty", 7: "erpm", 8: "v_in",
         15: "fault", 16: "pos", 17: "mid"}


def mask_for(tag):
    m = BASE
    for b, bit in enumerate(TAG_BITS):
        if tag >> b & 1:
            m |= bit
    return m


def tag_of(mask):
    return sum(1 << b for b, bit in enumerate(TAG_BITS) if mask & bit)


def parse(p):
    mask, = struct.unpack_from(">I", p, 1)
    i, d = 5, {}
    for bit in range(32):
        if mask >> bit & 1:
            fmt, sc = SIZES[bit]
            v, = struct.unpack_from(">" + fmt, p, i)
            i += struct.calcsize(fmt)
            d[NAMES[bit]] = v / sc
    d["w"] = d["erpm"] / ERPM_PER_RAD_S
    return tag_of(mask), d


def frames(buf):
    i = 0
    while i + 2 <= len(buf):
        if buf[i] != 0xAA:
            i += 1
            continue
        ln = buf[i + 1] + 5
        if i + ln > len(buf):
            return
        f = buf[i:i + ln]
        if f[-1] == 0xBB and struct.unpack(">H", f[-3:-1])[0] == crc16(f[2:-3]):
            yield bytes(f[2:-3])
            i += ln
        else:
            i += 1


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
    out = sys.argv[2] if len(sys.argv) > 2 else "step.csv"
    gap_us = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0     # delay between command and request
    ser = serial.Serial(port, 921600, timeout=0.01)
    ser.reset_input_buffer()
    rx = bytearray()
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            rx.extend(ser.read(4096))
    th = threading.Thread(target=reader)
    th.start()
    rpm = lambda w: frame(struct.pack(">Bi", 0x49, int(round(w * ERPM_PER_RAD_S))))
    coast = frame(struct.pack(">Bi", 0x47, 0))
    sched = []
    t0 = time.perf_counter()
    try:
        for k in range(1500):
            t = k * 1e-3
            while time.perf_counter() < t0 + t:
                pass
            w = (10.0 if 0.2 <= t < 0.8 else 0.0) if t < 1.3 else None
            cmd = rpm(w) if w is not None else coast
            req = frame(struct.pack(">BI", 0x13, mask_for(k % 8)))
            if gap_us > 0:
                ser.write(cmd)
                while time.perf_counter() < t0 + t + gap_us * 1e-6:
                    pass
                ser.write(req)
            else:
                ser.write(cmd + req)
            sched.append((t, w))
    finally:
        ser.write(coast)
        time.sleep(0.1)
        stop.set()
        th.join()
        ser.close()
    replies = [parse(p) for p in frames(rx) if p[0] == 0x13]
    # align by tag: each reply takes the next request index with its tag
    aligned, k, drops = {}, 0, []
    for tag, d in replies:
        while k < len(sched) and k % 8 != tag:
            drops.append(k)
            k += 1
        if k >= len(sched):
            break
        aligned[k] = d
        k += 1
    print(f"sent {len(sched)}, replies {len(replies)}, aligned {len(aligned)}, dropped {len(sched) - len(aligned)}")
    phases = {"rest 0-200": (0, 200), "step+hold 200-800": (200, 800), "stop 800-1300": (800, 1300),
              "coast 1300-1500": (1300, 1500)}
    for name, (a, b) in phases.items():
        nd = sum(1 for j in drops if a <= j < b)
        print(f"  drops in {name}: {nd}")
    with open(out, "w") as f:
        f.write("t_ms,cmd_rad_s,mos_C,motor_C,i_in_A,iq_A,duty,w_rad_s,v_in,fault\n")
        for j, (t, w) in enumerate(sched):
            d = aligned.get(j)
            if d:
                f.write(f"{j},{'' if w is None else w},{d['mos']},{d['motor_T']},{d['i_in']},{d['iq']},{d['duty']},"
                        f"{d['w']:.4f},{d['v_in']},{d['fault']}\n")

    def first(cond, start_ms):
        for j in sorted(aligned):
            if j >= start_ms and cond(aligned[j]):
                return j - start_ms
    print("step up @200: |Iq|>0.15 A +%s ms, w>0.3 +%s ms, w>9 (90%%) +%s ms" % (
        first(lambda d: abs(d["iq"]) > 0.15, 200), first(lambda d: d["w"] > 0.3, 200), first(lambda d: d["w"] > 9, 200)))
    print("step down @800: Iq<-0.15 A +%s ms, w<9.5 +%s ms, w<1 +%s ms" % (
        first(lambda d: d["iq"] < -0.15, 800), first(lambda d: d["w"] < 9.5, 800), first(lambda d: d["w"] < 1, 800)))
    for j in range(196, 216):
        d = aligned.get(j)
        if d:
            print(f"   {j} ms: iq {d['iq']:+.2f} A  w {d['w']:+.3f} rad/s  duty {d['duty']:+.3f}")


if __name__ == "__main__":
    main()
