"""The Jetson's stamps of the MCU's SYNC_OUT edges (design.md 10), through the kernel's GPIO character device
(libgpiod's Python binding, `python3-libgpiod`; API v2, falling back to v1).

The kernel stamps each edge in its interrupt handler, or, with the hardware timestamp engine (HTE: the Orin's AON
GPIO lines; VERIFY which 40-pin header pins), in hardware. The stamps come in the clock the request asks for:
  realtime   CLOCK_REALTIME, the ROS clock on the robot (use_sim_time false): used as is
  monotonic  CLOCK_MONOTONIC: moved onto CLOCK_REALTIME with the two clocks' offset, measured here
  hte        the HTE provider's stamps, on the monotonic base (VERIFY on JetPack): as monotonic
A plain interrupt stamp is ~5-30 us late with a long tail; clock_sync's fit drops the tail. The fixed part is measured
once on the bench (scope on the pin vs the stamp) and given to the link node as edge_latency_us.

    EdgeReader("/dev/gpiochip0", 105, on_edge=lambda t_ns: ..., clock="monotonic").start()
"""

from __future__ import annotations

import threading
import time


def realtime_minus_monotonic_ns() -> int:
    """CLOCK_REALTIME - CLOCK_MONOTONIC, from the tightest of a few bracketed reads."""
    best = None
    for _ in range(5):
        a = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        r = time.clock_gettime_ns(time.CLOCK_REALTIME)
        b = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        if best is None or b - a < best[0]:
            best = (b - a, r - (a + b) // 2)
    return best[1]


class EdgeReader:
    def __init__(self, chip: str, line: int, on_edge, clock: str = "realtime", log=print):
        if clock not in ("realtime", "monotonic", "hte"):
            raise ValueError(clock)
        self.chip, self.line, self.on_edge, self.clock, self.log = chip, int(line), on_edge, clock, log
        self.running = False
        self.n = 0
        self._off = 0
        self._t_off = 0.0

    def _to_ros_ns(self, t_ns: int) -> int:
        if self.clock == "realtime":
            return t_ns
        now = time.monotonic()
        if now - self._t_off > 1.0:                     # the offset moves as NTP / chrony slews the wall clock
            self._off, self._t_off = realtime_minus_monotonic_ns(), now
        return t_ns + self._off

    def start(self):
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()
        return self

    def stop(self):
        self.running = False

    def _run(self):
        try:
            import gpiod
        except ImportError:
            self.log("sync edges: no gpiod module (sudo apt install python3-libgpiod): no GPIO time sync")
            return
        try:
            if hasattr(gpiod, "request_lines"):         # libgpiod 2
                from gpiod.line import Clock, Edge
                clk = {"realtime": Clock.REALTIME, "monotonic": Clock.MONOTONIC, "hte": Clock.HTE}[self.clock]
                settings = gpiod.LineSettings(edge_detection=Edge.RISING, event_clock=clk)
                with gpiod.request_lines(self.chip, consumer="omni_sync", config={self.line: settings}) as req:
                    while self.running:
                        if req.wait_edge_events(0.5):
                            for ev in req.read_edge_events():
                                self.n += 1
                                self.on_edge(self._to_ros_ns(ev.timestamp_ns))
            else:                                       # libgpiod 1 (Ubuntu 22.04's): monotonic stamps
                self.clock = "monotonic"
                chip = gpiod.Chip(self.chip)
                ln = chip.get_line(self.line)
                ln.request(consumer="omni_sync", type=gpiod.LINE_REQ_EV_RISING_EDGE)
                while self.running:
                    if ln.event_wait(nsec=500_000_000):
                        ev = ln.event_read()
                        self.n += 1
                        self.on_edge(self._to_ros_ns(ev.sec * 1_000_000_000 + ev.nsec))
        except (OSError, ValueError, AttributeError) as e:
            self.log(f"sync edges on {self.chip}:{self.line}: {e}")
