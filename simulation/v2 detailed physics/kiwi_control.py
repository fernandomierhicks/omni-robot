"""Kiwi-drive kinematics and an acceleration-limited base controller.

Twists are (vx, vy, wz). Body-frame velocities use the robot frame (x forward,
y left); world-frame velocities use the floor frame. Wheel speeds are hub angular
speeds in rad/s with the sign convention of kiwi_model.py.
"""

from __future__ import annotations

import math

import numpy as np


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quat(q) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def world_to_body(v_world, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * v_world[0] + s * v_world[1], -s * v_world[0] + c * v_world[1]])


class KiwiKinematics:
    """Maps body twists to wheel speeds and back (odometry)."""

    def __init__(self, wheel_angles_deg, base_radius: float, wheel_radius: float):
        th = np.radians(wheel_angles_deg)
        # Rim speed of wheel i = t_i . v + L * wz, with tangent t_i = (-sin, cos).
        self.J = np.column_stack([-np.sin(th), np.cos(th), np.full(len(th), base_radius)]) / wheel_radius
        self.J_inv = np.linalg.inv(self.J)

    def wheel_speeds(self, twist_body) -> np.ndarray:
        return self.J @ np.asarray(twist_body, dtype=float)

    def body_twist(self, wheel_speeds) -> np.ndarray:
        return self.J_inv @ np.asarray(wheel_speeds, dtype=float)


def _approach_speed(dist: float, v_max: float, a_brake: float, gain: float) -> float:
    """Fastest speed that can still stop at the target, softened near it."""
    return min(v_max, math.sqrt(2 * a_brake * dist), gain * dist)


class BaseController:
    """Turns pose targets or velocity requests into wheel-speed commands.

    The commanded world-frame velocity is rate-limited to max_accel (a vector
    limit, so diagonal moves get the same acceleration as straight ones), and the
    wheel commands are scaled down together if any wheel would exceed its speed
    limit. This mirrors the firmware limits described in the design doc.
    """

    def __init__(self, kin: KiwiKinematics, max_speed=1.4, max_accel=4.5,
                 max_yaw_rate=4.0, max_yaw_accel=15.0, max_wheel_speed=38.7):
        self.kin = kin
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.max_yaw_rate = max_yaw_rate
        self.max_yaw_accel = max_yaw_accel
        self.max_wheel_speed = max_wheel_speed
        self.reset()

    def reset(self):
        self.v_cmd = np.zeros(2)     # world frame
        self.w_cmd = 0.0

    def velocity_step(self, dt: float, v_world_des, w_des: float, yaw: float) -> np.ndarray:
        dv = np.asarray(v_world_des, dtype=float) - self.v_cmd
        n = np.linalg.norm(dv)
        if n > self.max_accel * dt:
            dv *= self.max_accel * dt / n
        self.v_cmd = self.v_cmd + dv
        dw = float(np.clip(w_des - self.w_cmd, -self.max_yaw_accel * dt, self.max_yaw_accel * dt))
        self.w_cmd += dw
        wheels = self.kin.wheel_speeds([*world_to_body(self.v_cmd, yaw), self.w_cmd])
        peak = np.max(np.abs(wheels))
        if peak > self.max_wheel_speed:
            k = self.max_wheel_speed / peak
            wheels *= k
            self.v_cmd *= k
            self.w_cmd *= k
        return wheels

    def goto_step(self, dt: float, pose, target, v_ff=(0.0, 0.0)) -> np.ndarray:
        """pose, target: (x, y, yaw) in the world frame. v_ff: optional feed-forward velocity."""
        e = np.array([target[0] - pose[0], target[1] - pose[1]])
        d = float(np.linalg.norm(e))
        speed = _approach_speed(d, self.max_speed, 0.8 * self.max_accel, 6.0)
        v_des = np.asarray(v_ff, dtype=float) + (e / d * speed if d > 1e-9 else 0.0)
        n = np.linalg.norm(v_des)
        if n > self.max_speed:
            v_des *= self.max_speed / n
        ey = wrap_angle(target[2] - pose[2])
        w_des = math.copysign(_approach_speed(abs(ey), self.max_yaw_rate, 0.8 * self.max_yaw_accel, 8.0), ey)
        return self.velocity_step(dt, v_des, w_des, pose[2])
