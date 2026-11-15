"""Listen to the bench MCU's link (no CMDs sent: it stays DISARMED) and print what the robot computer would see:
STATE / INFO rates, mode, faults, each wheel's error code and temperature, the loop's health.

    ~/.venvs/cam/bin/python link_probe.py [/dev/ttyACM0] [--seconds 2] [--cmds] [--profile]

Runs on the robot computer with its omni_mcu sources (~/omni_ws/src/omni_mcu) for the decoder; needs pyserial.
--profile reads a BENCH_PROFILE build's loop-section times from the virtual wheels' temperatures (src/main.cpp).
"""
import argparse
import glob
import os
import sys
import time

sys.path.insert(0, os.path.expanduser("~/omni_ws/src/omni_mcu"))
import serial  # noqa: E402
from omni_mcu import protocol as P  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    arduino = sorted(glob.glob("/dev/serial/by-id/usb-Arduino*"))
    ap.add_argument("port", nargs="?", default=arduino[0] if arduino else "/dev/ttyACM0")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--cmds", action="store_true",
                    help="also send DISARM CMDs at 100 Hz, as mcu_link's heartbeat does (the motors stay off)")
    ap.add_argument("--profile", action="store_true",
                    help="a BENCH_PROFILE build: print each loop section's longest time (virtual wheels' temperatures)")
    a = ap.parse_args()
    prof = []                                     # every INFO's (receive, tick, transmit) longest time, us
    link = P.HostLink()
    t0 = time.time()
    t_cmd = 0.0
    last_states, t_last_state = 0, t0
    gaps = []                                     # every STATE's longest tick gap (us)
    late0 = None
    with serial.Serial(a.port, a.baud, timeout=0.005) as s:
        s.reset_input_buffer()
        while time.time() - t0 < a.seconds:
            for m in link.feed(s.read(4096), time.time_ns()):
                if isinstance(m, P.State):
                    gaps.append(m.tick_max_us)
                    late0 = m.tick_late if late0 is None else late0
                elif isinstance(m, P.Info) and a.profile:
                    prof.append(tuple(20 * t for t in m.t_rotor[1:4]))
            now = time.time()
            if a.cmds and now - t_cmd >= 0.01:
                t_cmd = now
                s.write(link.encode(P.Command(req_mode=P.REQ_DISARM), time.time_ns()))
            if link.stats.states != last_states:
                last_states, t_last_state = link.stats.states, now
            elif now - t_last_state > 1.0:
                print(f"STATE stopped at {t_last_state - t0:.1f} s (after {last_states})")
                break
    dt = time.time() - t0
    st, inf, n = link.state, link.info, link.stats
    print(f"{dt:.1f} s: frames {n.frames}, bad {n.errors}, STATE {n.states / dt:.0f}/s, INFO {n.infos / dt:.1f}/s")
    if inf:
        print(f"INFO protocol {inf.protocol} (host {P.PROTOCOL_VERSION}), build {inf.build}, rotor C {inf.t_rotor}")
    if st:
        print(f"STATE mode {st.mode_name}, faults {st.fault_names()}, dm_err {st.dm_err}, v_bus {st.v_bus:.1f}, "
              f"tick_late {st.tick_late}, tick_max {st.tick_max_us} us, rx_errors {st.rx_errors}")
        print(f"      odom x {st.x:.3f} y {st.y:.3f} yaw {st.yaw:.3f}, v ({st.vx:.3f}, {st.vy:.3f}, {st.wz:.3f})")
        g = sorted(gaps)
        print(f"loop  {st.tick_late - late0} ticks late (> 1.5 ms) in this run; longest gap per STATE: median "
              f"{g[len(g) // 2]} us, p99 {g[int(len(g) * 0.99)]} us, max {g[-1]} us; "
              f"{sum(x > 1500 for x in g)} of {len(g)} STATEs had one over 1.5 ms")
    if prof:                                      # each value: the longest in a 0.5 s window, 20 us steps, 2540 = cap
        for k, name in enumerate(("receive", "tick", "transmit")):
            v = sorted(p[k] for p in prof)
            print(f"prof  {name:8s} longest per 0.5 s: median {v[len(v) // 2]} us, max {v[-1]} us")
    return 0 if st else 1


if __name__ == "__main__":
    sys.exit(main())
