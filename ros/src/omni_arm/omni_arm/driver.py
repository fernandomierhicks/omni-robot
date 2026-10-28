"""The arm through its bus, in joint angles (design.md 11). No ROS: arm_node runs it on the robot, the sim's harness on
the sim's bus. One read and one write per cycle (50 Hz on the robot): a SYNC_READ of every servo's position, speed,
load, volts and temperature, then a SYNC_WRITE of the six goal positions.

The servos run their own position loops; the driver streams goals along a motion (motion.py) and never sends one
outside a joint's range. The gripper runs at a lower torque limit (GRIP_TORQUE: the STS3215 stalled on an object
heats; LeRobot limits its gripper the same way) and its load says whether it holds something.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from omni_arm.feetech import Bus, JointCal, nominal_calibration
from omni_arm.kinematics import ArmConfig

ARM_TORQUE = 1000              # torque limit, 0.1% of stall: the arm's joints at full torque
GRIP_TORQUE = 500              # the gripper at half (est; LeRobot's SO-101 follower uses ~50%)
ACCELERATION = 0               # STS acceleration register: 0 = the servo's own (no ramp); the motions ramp


@dataclass
class ArmState:
    t: float = 0.0
    q: np.ndarray = field(default_factory=lambda: np.zeros(6))
    dq: np.ndarray = field(default_factory=lambda: np.zeros(6))
    load: np.ndarray = field(default_factory=lambda: np.zeros(6))     # fraction of each servo's torque limit
    volts: float = 0.0
    temp_c: np.ndarray = field(default_factory=lambda: np.zeros(6))
    ok: bool = False                   # every servo answered
    missing: tuple = ()


class ArmDriver:
    def __init__(self, bus: Bus, cfg: ArmConfig, cal: dict | None = None):
        self.bus, self.cfg = bus, cfg
        self.joints = cfg.joints
        self.ids = list(cfg.servos["ids"])
        self.cal: dict = cal or nominal_calibration(self.joints, self.ids, cfg.ranges)
        self.lo, self.hi = cfg.ranges[:, 0], cfg.ranges[:, 1]
        self.state = ArmState()
        self.goal: np.ndarray | None = None
        self.enabled = False
        self.n_reads = self.n_writes = 0

    def _c(self, k: int) -> JointCal:
        return self.cal[self.joints[k]]

    def ping(self) -> list:
        """The ids that didn't answer."""
        return [i for i in self.ids if not self.bus.ping(i)]

    def enable(self, on: bool = True):
        """Torque on (each servo's limit set first) or off (the arm goes limp: hold it, it has no brakes)."""
        for k, sid in enumerate(self.ids):
            if on:
                self.bus.write(sid, "torque_limit", GRIP_TORQUE if self.joints[k] == "gripper" else ARM_TORQUE)
                self.bus.write(sid, "acceleration", ACCELERATION)
            self.bus.write(sid, "torque_enable", 1 if on else 0)
        self.enabled = on

    def read(self, t: float) -> ArmState:
        st = self.bus.sync_read_status(self.ids)
        s = self.state
        s.t = t
        miss = []
        for k, sid in enumerate(self.ids):
            r = st.get(sid)
            if r is None:
                miss.append(sid)
                continue
            c = self._c(k)
            s.q[k] = c.to_rad(r.position)
            s.dq[k] = c.sign * r.speed * 2 * np.pi / 4096
            s.load[k] = c.sign * r.load
            s.temp_c[k] = r.temp_c
            s.volts = r.volts
        s.ok = not miss
        s.missing = tuple(miss)
        self.n_reads += 1
        return s

    def send(self, q):
        """Goal angles (6, rad), clipped to the joints' ranges."""
        q = np.clip(np.asarray(q, float), self.lo, self.hi)
        self.goal = q
        self.bus.sync_write_goals({sid: self._c(k).to_ticks(q[k]) for k, sid in enumerate(self.ids)})
        self.n_writes += 1
