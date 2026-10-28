"""v11 (design.md 30): the Jetson <-> MCU link inside one process, for the deterministic harness and the tests.

The Jetson side is the robot's own code (ros/src/omni_mcu/omni_mcu/protocol.py: HostLink, TimeSync), imported from the
repo. Bytes go to the SIL firmware (fw.rx) and come back (fw.tx) after a USB latency (est: 0.5 ms +/- 0.2 each way,
full-speed USB frames are 1 ms, the Teensy's native USB is high speed). The Jetson's clock is sim time; the MCU's is
sim time + its boot offset, which the Jetson recovers from the CMD / STATE echo like the real link.

design.md 10 adds, all off by default (the defaults reproduce earlier results):
  latency_up / latency_down   a USB path that isn't symmetric (host -> device OUT transfers go out in the next
                              microframe, device -> host IN data waits for the host's poll: est 0.25 / 0.75 ms). The
                              echo can't see it: its offset is biased by half the difference
  gpio                        the Jetson stamps the MCU's SYNC_OUT edges on a GPIO: "hte" (the Orin's hardware
                              timestamp engine on an AON pin: ~1 us), "irq" (a plain GPIO interrupt: 8 us + a late
                              tail, 2% stamps 0.1-0.3 ms late, est), None (not wired)
With a SilRobot built with timing="real", the MCU also stamps each CMD's bytes when they arrive.
"""

from __future__ import annotations

import os
import sys
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROS_SRC = Path(os.environ.get("OMNI_ROS_SRC", HERE.parents[1] / "ros" / "src"))
for pkg in ("omni_mcu", "omni_catch", "omni_console", "omni_arm"):    # omni_console: its ROS-free session_log
    if str(ROS_SRC / pkg) not in sys.path:
        sys.path.insert(0, str(ROS_SRC / pkg))

from omni_mcu.protocol import Command, HostLink  # noqa: E402


class GpioStamp:
    """The Jetson's stamp of a GPIO edge: true time + its latency (s)."""

    def __init__(self, kind: str, rng):
        if kind not in ("hte", "irq"):
            raise ValueError(kind)
        self.kind, self.rng = kind, rng

    def latency(self) -> float:
        r = self.rng
        if self.kind == "hte":
            return max(0.0, r.normal(1e-6, 0.3e-6))
        lat = 8e-6 + r.exponential(5e-6)
        if r.random() < 0.02:
            lat += r.uniform(100e-6, 300e-6)
        return lat


class SimLink:
    def __init__(self, robot, latency: float = 0.0005, jitter: float = 0.0002, seed: int = 0,
                 latency_up: float | None = None, latency_down: float | None = None, gpio: str | None = None):
        self.r = robot
        self.host = HostLink()
        self.lat, self.jit = latency, jitter
        self.lat_up = latency if latency_up is None else latency_up
        self.lat_down = latency if latency_down is None else latency_down
        self.rng = np.random.default_rng(70_000 + seed)
        self.gpio = GpioStamp(gpio, np.random.default_rng(71_000 + seed)) if gpio else None
        self.to_mcu: deque = deque()          # (t due, bytes)
        self.to_host: deque = deque()
        self.edges: deque = deque()           # (host stamp, ) of SYNC_OUT edges not yet seen by the Jetson
        self._n_edges = 0
        self.t_last_out = 0.0
        self.faults = None                    # faults.FaultInjector, if one is attached

    def _delay(self, lat: float) -> float:
        return max(0.0001, lat + self.rng.normal(0, self.jit))

    def send(self, cmd: Command, t: float):
        """The Jetson sends a CMD at sim time t."""
        data = self.host.encode(cmd, int(round(t * 1e9)))
        due = max(t + self._delay(self.lat_up), self.to_mcu[-1][0] if self.to_mcu else 0.0)     # in order
        self.to_mcu.append((due, data))

    def poll(self, t: float) -> list:
        """Move due bytes both ways; return the messages the Jetson received (State / Imu / ...). With a fault
        injector (faults.py) whose link is down, the bytes are lost both ways."""
        fw = self.r.fw
        down = self.faults is not None and self.faults.link_down()
        stamp = getattr(self.r, "real", False)
        while self.to_mcu and self.to_mcu[0][0] <= t:
            due, data = self.to_mcu.popleft()
            if not down:
                fw.rx(data, self.r.mcu_time_us(due) if stamp else 0)
        out = fw.tx()
        if down:
            out = b""
        if out:
            due = max(t + self._delay(self.lat_down), self.to_host[-1][0] if self.to_host else 0.0)
            self.to_host.append((due, out))
        if self.gpio is not None:
            edges = list(getattr(self.r, "sync_edges", []))
            n_new = min(len(edges), getattr(self.r, "n_sync", 0) - self._n_edges)
            for te in edges[len(edges) - n_new:] if n_new > 0 else []:
                self.edges.append(te + self.gpio.latency())
            self._n_edges = getattr(self.r, "n_sync", 0)
            while self.edges and self.edges[0] <= t:
                self.host.add_sync_edge(int(round(self.edges.popleft() * 1e9)))
        msgs = []
        while self.to_host and self.to_host[0][0] <= t:
            due, data = self.to_host.popleft()
            msgs += self.host.feed(data, int(round(due * 1e9)))
        return msgs
