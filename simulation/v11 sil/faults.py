"""Fault injection for the robot with its firmware in the loop (design.md 7: "fault injection in SIL: link drops, a
silent motor, low battery, e-stop, each checked end to end through the console").

  link      the Jetson <-> MCU link drops (both ways): the firmware's 0.5 s watchdog stops the robot (LINK_LOST)
  motor     one drive motor goes silent on CAN (cable): it times out (error D), the firmware sees MOTOR_SILENT
  battery   the pack reads low (21.5 V by default, under BATT_LOW 22.4 V): LOW_BATT
  estop     the e-stop is pressed: ESTOP, the motors unpowered
  bumper    the bumper is pressed: a ramped stop (BUMPER)
  imu       the BNO085 goes quiet: IMU_SILENT
  flywheel  the ESCs' telemetry stops while the flywheels run: FLY_TELEM, the thrower won't fire

FaultInjector works on a sil_robot.SilRobot (and the link: sil_link.SimLink in one process, omni_sim's McuPort in
ROS); the ROS bridges take {"kind", "on", "s", "motor", "volts"} on /sim/fault and say what's active on /sim/faults.
"""

from __future__ import annotations

import math

KINDS = {
    "link": "The Jetson <-> MCU link drops (both ways)",
    "motor": "A drive motor goes silent on CAN",
    "battery": "The pack reads low (21.5 V)",
    "estop": "The e-stop is pressed",
    "bumper": "The bumper is pressed",
    "imu": "The IMU goes quiet",
    "flywheel": "The flywheels' telemetry stops",
}
EXPECT = {"link": "LINK_LOST", "motor": "MOTOR_SILENT", "battery": "LOW_BATT", "estop": "ESTOP", "bumper": "BUMPER",
          "imu": "IMU_SILENT", "flywheel": "FLY_TELEM"}      # the firmware's fault each one should raise


class FaultInjector:
    def __init__(self, robot):
        self.r = robot
        self.until = {}               # kind -> sim time it ends (math.inf: until cleared)
        self.link_until = -1.0
        self.motor = 0

    def inject(self, kind: str, on: bool = True, seconds: float | None = None, motor: int = 0, volts: float = 21.5):
        if kind not in KINDS:
            raise ValueError(f"no fault '{kind}' (one of {', '.join(KINDS)})")
        r, t = self.r, self.r.d.time
        end = (t + seconds if seconds else math.inf) if on else -1.0
        if on:
            self.until[kind] = end
        else:
            self.until.pop(kind, None)
        if kind == "link":
            self.link_until = end
        elif kind == "motor":
            self.motor = int(motor) % len(r.motor_cut_until)
            r.motor_cut_until = [-1.0] * len(r.motor_cut_until)
            r.motor_cut_until[self.motor] = end
        elif kind == "battery":
            r.v_bus_override = volts if on else None
        elif kind == "estop":
            r.estop = on
        elif kind == "bumper":
            r.bumper = on
        elif kind == "imu":
            r.imu_cut_until = end
        elif kind == "flywheel":
            r.fly_telem_cut_until = end

    def step(self):
        """Ends the timed ones (the battery, e-stop and bumper are states, not cut-offs)."""
        t = self.r.d.time
        for kind, end in list(self.until.items()):
            if t >= end:
                self.inject(kind, on=False)

    def link_down(self) -> bool:
        return self.r.d.time < self.link_until

    def active(self) -> list:
        t = self.r.d.time
        return [dict(kind=k, until=None if e == math.inf else round(e - t, 2), **({"motor": self.motor + 1}
                     if k == "motor" else {})) for k, e in sorted(self.until.items())]
