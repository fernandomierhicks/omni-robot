"""v11 (design.md 30): the Jetson <-> MCU link inside one process, for the deterministic harness and the tests.

The Jetson side is the robot's own code (ros/src/omni_mcu/omni_mcu/protocol.py: HostLink, TimeSync), imported from the
repo. Bytes go to the SIL firmware (fw.rx) and come back (fw.tx) after a USB latency (est: 0.5 ms +/- 0.2 each way,
full-speed USB frames are 1 ms, the Teensy's native USB is high speed). The Jetson's clock is sim time; the MCU's is
sim time + its boot offset, which the Jetson recovers from the CMD / STATE echo like the real link.
"""

from __future__ import annotations

import os
import sys
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROS_SRC = Path(os.environ.get("OMNI_ROS_SRC", HERE.parents[1] / "ros" / "src"))
for pkg in ("omni_mcu", "omni_catch"):
    if str(ROS_SRC / pkg) not in sys.path:
        sys.path.insert(0, str(ROS_SRC / pkg))

from omni_mcu.protocol import Command, HostLink  # noqa: E402


class SimLink:
    def __init__(self, robot, latency: float = 0.0005, jitter: float = 0.0002, seed: int = 0):
        self.r = robot
        self.host = HostLink()
        self.lat, self.jit = latency, jitter
        self.rng = np.random.default_rng(70_000 + seed)
        self.to_mcu: deque = deque()          # (t due, bytes)
        self.to_host: deque = deque()
        self.t_last_out = 0.0

    def _delay(self) -> float:
        return max(0.0001, self.lat + self.rng.normal(0, self.jit))

    def send(self, cmd: Command, t: float):
        """The Jetson sends a CMD at sim time t."""
        data = self.host.encode(cmd, int(round(t * 1e9)))
        due = max(t + self._delay(), self.to_mcu[-1][0] if self.to_mcu else 0.0)     # in order
        self.to_mcu.append((due, data))

    def poll(self, t: float) -> list:
        """Move due bytes both ways; return the messages the Jetson received (State / Imu)."""
        fw = self.r.fw
        while self.to_mcu and self.to_mcu[0][0] <= t:
            fw.rx(self.to_mcu.popleft()[1])
        out = fw.tx()
        if out:
            due = max(t + self._delay(), self.to_host[-1][0] if self.to_host else 0.0)
            self.to_host.append((due, out))
        msgs = []
        while self.to_host and self.to_host[0][0] <= t:
            due, data = self.to_host.popleft()
            msgs += self.host.feed(data, int(round(due * 1e9)))
        return msgs
