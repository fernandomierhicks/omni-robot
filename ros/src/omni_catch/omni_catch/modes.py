"""Robot modes on the Jetson (design.md 14, 15, 30): who drives, and the profile the MCU enforces.

  IDLE      the MCU is asked to disarm (ramped stop first, then motors off)
  TELEOP    /cmd_vel (stick, Foxglove, Nav2) drives the base; module parked
  CATCH     the catch behavior drives the base and the module
The MCU stays the safety authority: it caps everything by its own profile (kid / adult), stops on a lost link, a
bumper or the e-stop, and the RC switch takes over regardless of this (design.md 15). This only chooses whose request
goes out. A Request carries times on the Jetson's clock; to_command() turns it into the MCU's Command with the link's
time sync (the link node does that on the robot). mode_node.py is the ROS node around ModeMux.
"""

from __future__ import annotations

from omni_mcu.protocol import (DRIVE_GOTO, DRIVE_IDLE, DRIVE_VEL_BODY, REQ_AUTO, REQ_DISARM, Command, Request,
                               TimeSync, to_command)

MODES = ("IDLE", "TELEOP", "CATCH")


class ModeMux:
    def __init__(self, mode: str = "IDLE", kid: bool = False):
        self.mode = mode
        self.kid = kid

    def set_mode(self, mode: str, kid: bool | None = None) -> tuple[bool, str]:
        mode = mode.upper()
        if mode not in MODES:
            return False, f"unknown mode {mode!r} (one of {', '.join(MODES)})"
        self.mode = mode
        if kid is not None:
            self.kid = kid
        return True, f"{mode}, {'kid' if self.kid else 'adult'} profile"

    def request(self, catch=None, twist=None) -> Request:
        """This cycle's request: catch = a game.CatchCommand, twist = (vx, vy, wz) body frame, fresh (TELEOP)."""
        if self.mode == "IDLE":
            return Request(req_mode=REQ_DISARM, kid=self.kid)
        if self.mode == "TELEOP":
            if twist is None:
                return Request(req_mode=REQ_AUTO, kid=self.kid, drive_mode=DRIVE_IDLE)
            return Request(req_mode=REQ_AUTO, kid=self.kid, drive_mode=DRIVE_VEL_BODY, hold_yaw=True,
                           a=twist[0], b=twist[1], c=twist[2])
        if catch is None:
            return Request(req_mode=REQ_AUTO, kid=self.kid, drive_mode=DRIVE_IDLE)
        c = catch
        return Request(req_mode=REQ_AUTO, kid=self.kid, drive_mode=c.drive_mode, a=c.target[0], b=c.target[1],
                       c=c.target[2], v_end=c.v_end, t_brake=c.t_brake, coast_s=c.coast_s, a_brake=c.a_brake,
                       tilt=c.tilt, t_tilt=c.t_tilt, tilt_after=c.tilt_after, fly_rpm=c.fly_rpm, fire_seq=c.fire_seq)

    def command(self, sync: TimeSync, catch=None, twist=None) -> Command:
        return to_command(self.request(catch, twist), sync)


__all__ = ["DRIVE_GOTO", "MODES", "ModeMux", "Request", "to_command"]
