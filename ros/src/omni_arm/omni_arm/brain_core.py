"""The arm brain's ROS-free half (design.md 11): what arm_brain.py and the v11 sim's tidy harness (sil_pick.py) both do
between the tidy task (pick.py) and omni_cpp's C++ arm driver, so the sim drives the arm the way the robot does.

  ArmState     the arm as the task sees it, built from the driver's joint states each cycle
  MotionSync   what the task now wants, as the driver's commands: a new motion whole (quintic waypoints at times), or a
               held goal; nothing when it hasn't changed (the driver keeps playing what it has)
  HEALTH_GRACE how long a tidy rides through unhealthy servo feedback before it stops with the arm holding
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

MOVE, HOLD, RELAX = 0, 1, 2       # omni_interfaces/ArmMotion's commands
HEALTH_GRACE = 0.5                # s of unhealthy /arm/hw (5 Hz on the robot: two or three in a row) a tidy rides through


@dataclass
class ArmState:
    t: float = 0.0
    q: np.ndarray = field(default_factory=lambda: np.zeros(6))
    dq: np.ndarray = field(default_factory=lambda: np.zeros(6))
    load: np.ndarray = field(default_factory=lambda: np.zeros(6))     # fraction of each servo's torque limit
    volts: float = 0.0
    temp_c: np.ndarray = field(default_factory=lambda: np.zeros(6))
    ok: bool = False                   # every servo answered, none with an error (overload, heat, voltage, ...)
    missing: tuple = ()                # the ids that didn't answer: their joints keep the last values read
    errors: tuple = ()                 # the ids that answered with an error


def state_from_joints(t: float, q, dq, load) -> ArmState:
    """The task's ArmState from one joint-state message (position, velocity, effort = load); its health comes from
    the driver's hardware report (see HEALTH_GRACE), so ok here."""
    return ArmState(t=t, q=np.asarray(q, float), dq=np.asarray(dq, float), load=np.asarray(load, float), ok=True)


class MotionSync:
    """Tell the driver what the task wants: commands() after each task step returns [(command, times, waypoints,
    linear)] to send (empty: unchanged)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._sent, self._sent_t = None, ()

    def commands(self, task, t: float) -> list:
        m = task.motion
        if m is not None:
            if self._sent is not m or self._sent_t != tuple(m.t):
                self._sent, self._sent_t = m, tuple(m.t)
                return [(MOVE, list(m.t), [np.asarray(q, float) for q in m.q], False)]
        elif task.goal is not None:
            key = tuple(np.round(task.goal, 9))
            if self._sent != key:
                self._sent = key
                g = np.asarray(task.goal, float)
                return [(MOVE, [t, t], [g, g], True)]             # a held goal: a zero-length straight move
        return []
