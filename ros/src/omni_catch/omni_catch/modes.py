"""Robot modes on the Jetson (design.md 14, 15, 30): who drives, and the profile the MCU enforces.

  IDLE      the MCU is asked to disarm (ramped stop first, then motors off)
  TELEOP    /cmd_vel (stick, Foxglove, Nav2, the console) drives the base; module parked
  CATCH     the catch behavior drives the base and the module
  THROWER   bench test of the module from the operator console: base parked, the console's tilt, flywheel rpm and
            fire requests (the firmware's interlocks still decide whether a ball goes)
  PICK      the arm robot's tidy task (omni_arm, design.md 11) drives the base: its twist, as TELEOP's
Arm out (design.md 11): while the arm robot's tool is outside the bumper or high (omni_arm says so), every request
asks the MCU for at most ARM_OUT_CAPS (speed m/s, acceleration m/s^2), whoever drives: a gentler base with the
arm's weight out front (robot_arm.json "arm_out"; test_v11 checks the two agree).
The MCU stays the safety authority: it caps everything by its own profile (kid / adult), stops on a lost link, a
bumper or the e-stop, and the RC switch takes over regardless of this (design.md 15). This only chooses whose request
goes out. A Request carries times on the Jetson's clock; to_command() turns it into the MCU's Command with the link's
time sync (the link node does that on the robot). mode_node.py is the ROS node around ModeMux.

Firing: the MCU fires one ball whenever the fire_seq it receives changes. Each source (the catch behavior, the console)
counts its own requests, so ModeMux sends one counter of its own that moves by one per new request of the source in
charge; a source that takes over is adopted without firing (a stale counter must never fire a ball).
"""

from __future__ import annotations

from omni_mcu.protocol import (DRIVE_GOTO, DRIVE_IDLE, DRIVE_VEL_BODY, REQ_AUTO, REQ_DISARM, Command, Request,
                               TimeSync, to_command)

MODES = ("IDLE", "TELEOP", "CATCH", "THROWER", "PICK")
ARM_OUT_CAPS = (1.0, 2.131578947)      # m/s, m/s^2: robot_arm.json arm_out (half the table's 4.26 straight ahead)


class ModeMux:
    def __init__(self, mode: str = "IDLE", kid: bool = False):
        self.mode = mode
        self.kid = kid
        self.fire_out = 0              # the fire_seq the MCU sees
        self._fire_src = None          # (mode, that source's last fire_seq)
        self.arm_out = False           # the arm robot's tool is out: cap the base (ARM_OUT_CAPS)

    def set_mode(self, mode: str, kid: bool | None = None) -> tuple[bool, str]:
        mode = mode.upper()
        if mode not in MODES:
            return False, f"unknown mode {mode!r} (one of {', '.join(MODES)})"
        self.mode = mode
        if kid is not None:
            self.kid = kid
        return True, f"{mode}, {'kid' if self.kid else 'adult'} profile"

    def fire(self, seq: int) -> int:
        """The fire_seq to send for this source's seq (see the module notes)."""
        if self._fire_src is None or self._fire_src[0] != self.mode:
            self._fire_src = (self.mode, seq)           # a new source: adopt its counter, fire nothing
        elif seq != self._fire_src[1]:
            self._fire_src = (self.mode, seq)
            self.fire_out = (self.fire_out + 1) & 0xFFFF
        return self.fire_out

    def request(self, catch=None, twist=None, thrower=None, pick=None) -> Request:
        """This cycle's request: catch = a game.CatchCommand, twist = (vx, vy, wz) body frame, fresh (TELEOP),
        thrower = a Request with tilt, fly_rpm, fire_seq (THROWER), fresh, pick = the tidy task's twist (PICK), fresh.
        With the arm out every request carries ARM_OUT_CAPS."""
        r = self._request(catch, twist, thrower, pick)
        if self.arm_out:
            r.speed_cap = min(r.speed_cap, ARM_OUT_CAPS[0]) if r.speed_cap > 0 else ARM_OUT_CAPS[0]
            r.a_cap = min(r.a_cap, ARM_OUT_CAPS[1]) if r.a_cap > 0 else ARM_OUT_CAPS[1]
        return r

    def _request(self, catch, twist, thrower, pick) -> Request:
        f = self.fire_out
        if self.mode == "IDLE":
            return Request(req_mode=REQ_DISARM, kid=self.kid, fire_seq=f)
        if self.mode == "PICK":
            twist = pick
        if self.mode in ("TELEOP", "PICK"):
            if twist is None:
                return Request(req_mode=REQ_AUTO, kid=self.kid, drive_mode=DRIVE_IDLE, fire_seq=f)
            return Request(req_mode=REQ_AUTO, kid=self.kid, drive_mode=DRIVE_VEL_BODY, hold_yaw=True,
                           a=twist[0], b=twist[1], c=twist[2], fire_seq=f)
        if self.mode == "THROWER":
            if thrower is None:                          # the console went quiet: flywheels down, tilt home
                return Request(req_mode=REQ_AUTO, kid=self.kid, drive_mode=DRIVE_IDLE, fire_seq=f)
            t = thrower
            return Request(req_mode=REQ_AUTO, kid=self.kid, drive_mode=DRIVE_IDLE, tilt=t.tilt, fly_rpm=t.fly_rpm,
                           fire_seq=self.fire(t.fire_seq))
        if catch is None:
            return Request(req_mode=REQ_AUTO, kid=self.kid, drive_mode=DRIVE_IDLE, fire_seq=f)
        c = catch
        return Request(req_mode=REQ_AUTO, kid=self.kid, drive_mode=c.drive_mode, a=c.target[0], b=c.target[1],
                       c=c.target[2], v_end=c.v_end, t_brake=c.t_brake, coast_s=c.coast_s, a_brake=c.a_brake,
                       tilt=c.tilt, t_tilt=c.t_tilt, tilt_after=c.tilt_after, fly_rpm=c.fly_rpm,
                       fire_seq=self.fire(c.fire_seq))

    def command(self, sync: TimeSync, catch=None, twist=None, thrower=None, pick=None) -> Command:
        return to_command(self.request(catch, twist, thrower, pick), sync)


__all__ = ["ARM_OUT_CAPS", "DRIVE_GOTO", "MODES", "ModeMux", "Request", "to_command"]
