"""Drive the AK40-10 bench sketch (src/main.cpp) over USB serial and print or log what it says.

    python ak_bench.py --port COM8 "listen 2" "v 2000 2000" "listen 3" "x"

Each step is a sketch command (sent as one line) or "listen <s>" (just read for s seconds). After every step the
script reads for --gap seconds. On exit, Ctrl-C or error it sends "x" (zero current, coast). --log writes every
line with a host timestamp. Needs pyserial (PlatformIO's interpreter has it).
"""
import argparse
import sys
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--gap", type=float, default=0.3, help="seconds to read after each command")
    ap.add_argument("--log", help="write every received line here, with host time")
    ap.add_argument("--quiet-status", action="store_true", help="do not echo S lines (still logged)")
    ap.add_argument("steps", nargs="*")
    a = ap.parse_args()

    log = open(a.log, "w", encoding="utf-8") if a.log else None
    t0 = time.time()
    with serial.Serial(a.port, 115200, timeout=0.05) as s:
        def pump(dt):
            end = time.time() + dt
            while time.time() < end:
                line = s.readline().decode(errors="replace").rstrip()
                if not line:
                    continue
                if log:
                    log.write(f"{time.time() - t0:9.3f} {line}\n")
                if not (a.quiet_status and line.startswith("S ")):
                    print(line, flush=True)

        def send(cmd):
            print(f">> {cmd}", flush=True)
            if log:
                log.write(f"{time.time() - t0:9.3f} >> {cmd}\n")
            s.write((cmd + "\n").encode())

        try:
            pump(0.5)
            for st in a.steps:
                if st.startswith("listen"):
                    pump(float(st.split()[1]))
                else:
                    send(st)
                    pump(a.gap)
        except KeyboardInterrupt:
            pass
        finally:
            send("x")
            pump(0.3)
    if log:
        log.close()


if __name__ == "__main__":
    sys.exit(main())
