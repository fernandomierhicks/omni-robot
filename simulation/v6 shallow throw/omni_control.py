"""Omni-base kinematics (any number of wheels) and a jerk-limited base controller.

Twists are (vx, vy, wz). Body-frame velocities use the robot frame (x forward,
y left); world-frame velocities use the floor frame. Wheel speeds are hub
angular speeds in rad/s.
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


class OmniKinematics:
    """Maps body twists to wheel speeds and back (odometry, least squares for N > 3)."""

    def __init__(self, wheel_angles_deg, base_radius: float, wheel_radius: float):
        th = np.radians(wheel_angles_deg)
        # Rim speed of wheel i = t_i . v + L * wz, with tangent t_i = (-sin, cos).
        self.J = np.column_stack([-np.sin(th), np.cos(th), np.full(len(th), base_radius)]) / wheel_radius
        self.J_inv = np.linalg.pinv(self.J)

    def wheel_speeds(self, twist_body) -> np.ndarray:
        return self.J @ np.asarray(twist_body, dtype=float)

    def body_twist(self, wheel_speeds) -> np.ndarray:
        return self.J_inv @ np.asarray(wheel_speeds, dtype=float)


def _approach_speed(dist: float, v_max: float, a_brake: float, gain: float) -> float:
    return min(v_max, math.sqrt(2 * a_brake * dist), gain * dist)


class BaseController:
    """Pose targets or velocity requests -> wheel-speed commands.

    The world-frame velocity command is acceleration-limited (a vector limit)
    and, if max_jerk > 0, jerk-limited: the commanded acceleration ramps instead
    of stepping, which keeps the base from rocking on its wheels at the start
    and end of a move. a_cmd (world frame) is kept for torque feed-forward.
    """

    def __init__(self, kin: OmniKinematics, max_speed=1.4, max_accel=2.5, max_jerk=40.0,
                 max_yaw_rate=4.0, max_yaw_accel=15.0, max_wheel_speed=38.7, accel_table=None, brake_margin=0.8):
        """accel_table: optional ((direction_deg, ...), (limit, ...)) in the body frame, 0..180 deg,
        mirrored for the left side: a direction-aware acceleration limit replacing max_accel.
        max_speed / max_yaw_rate may be inf: then only max_wheel_speed (the motors) limits them.
        brake_margin: stops are planned at this fraction of the worst-direction acceleration limit."""
        self.kin = kin
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.accel_table = accel_table
        self.max_jerk = max_jerk
        self.max_yaw_rate = max_yaw_rate
        self.max_yaw_accel = max_yaw_accel
        self.max_wheel_speed = max_wheel_speed
        self.brake_margin = brake_margin
        self.reset()

    def top_speed(self, u_world, yaw: float) -> float:
        """Fastest translation in a world direction: the configured cap or the wheel-speed ceiling,
        whichever is lower (an X drive goes ~1.4x faster along its sides than along a diagonal)."""
        ub = world_to_body(u_world, yaw)
        n = float(np.linalg.norm(ub))
        if n < 1e-9:
            return self.max_speed
        k = float(np.max(np.abs(self.kin.J[:, :2] @ (ub / n))))
        return min(self.max_speed, self.max_wheel_speed / k)

    def worst_accel(self) -> float:
        return float(min(self.accel_table[1])) if self.accel_table is not None else self.max_accel

    def reset(self):
        self.v_cmd = np.zeros(2)
        self.a_cmd = np.zeros(2)
        self.w_cmd = 0.0

    def accel_limit(self, u_world, yaw: float) -> float:
        """Acceleration limit for a world-frame direction (direction-aware if a table is set)."""
        if self.accel_table is None:
            return self.max_accel
        ub = world_to_body(u_world, yaw)
        ang = abs(math.degrees(math.atan2(ub[1], ub[0])))          # 0..180, left/right mirrored
        dirs, vals = self.accel_table
        return float(np.interp(ang, dirs, vals))

    def velocity_step(self, dt: float, v_world_des, w_des: float, yaw: float, a_cap: float | None = None) -> np.ndarray:
        """a_cap: extra acceleration cap for this step (e.g. braking gently with a ball in the pocket)."""
        v_world_des = np.asarray(v_world_des, dtype=float)
        sp = float(np.linalg.norm(v_world_des))
        if sp > self.max_speed:            # (v3 only capped pose targets; velocity requests bypassed the cap)
            v_world_des = v_world_des * (self.max_speed / sp)
        dv = v_world_des - self.v_cmd
        n = float(np.linalg.norm(dv))
        a_lim = self.accel_limit(dv / n, yaw) if n > 1e-9 else self.max_accel
        if a_cap is not None:
            a_lim = min(a_lim, a_cap)
        if self.max_jerk > 0:
            # fastest acceleration that can still ramp back to zero without overshooting
            mag = min(a_lim, math.sqrt(2 * self.max_jerk * n))
            a_des = dv / n * mag if n > 1e-9 else np.zeros(2)
            da = a_des - self.a_cmd
            m = float(np.linalg.norm(da))
            if m > self.max_jerk * dt:
                da *= self.max_jerk * dt / m
            self.a_cmd = self.a_cmd + da
            step = self.a_cmd * dt
            if float(np.dot(step, dv)) > n * n:            # don't step past the target
                step = dv.copy()
            self.v_cmd = self.v_cmd + step
        else:
            if n > a_lim * dt:
                dv *= a_lim * dt / n
            self.a_cmd = dv / dt
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

    def goto_step(self, dt: float, pose, target, v_ff=(0.0, 0.0), v_end: float = 0.0) -> np.ndarray:
        """pose, target: (x, y, yaw) in the world frame. v_ff: optional feed-forward velocity.
        v_end: speed to pass through the target at (a moving catch) instead of stopping there."""
        e = np.array([target[0] - pose[0], target[1] - pose[1]])
        d = float(np.linalg.norm(e))
        # plan the stop with the worst-direction limit: the base often yaws while it brakes, which moves the
        # braking direction (in the body frame) into a weaker one and overshoots the target
        a_brake = self.brake_margin * self.worst_accel()
        # jerk-aware: reversing from the current acceleration to braking takes (|a| + a_brake) / jerk, and the
        # base keeps moving (and speeding up) meanwhile, so plan the stop over the distance left after that
        d_eff = d
        if self.max_jerk > 0:
            # speed relative to the target (which moves at v_ff when riding along to an intercept)
            t_rev = (float(np.linalg.norm(self.a_cmd)) + a_brake) / self.max_jerk
            d_eff = max(0.0, d - float(np.linalg.norm(self.v_cmd - np.asarray(v_ff, dtype=float))) * t_rev)
        # top speed toward the target: the cap, or the wheels' ceiling in that direction (a small margin
        # keeps some motor voltage for tracking)
        v_top = 0.97 * self.top_speed(e / d, pose[2]) if d > 1e-9 else self.max_speed
        if v_end > 0:      # braking curve that ends at v_end instead of rest
            speed = min(v_top, math.sqrt(v_end * v_end + 2 * a_brake * d_eff), v_end + 6.0 * d)
        else:
            speed = min(_approach_speed(d_eff, v_top, a_brake, 6.0), 6.0 * d)
        v_des = np.asarray(v_ff, dtype=float) + (e / d * speed if d > 1e-9 else 0.0)
        n = np.linalg.norm(v_des)
        if n > self.max_speed:
            v_des *= self.max_speed / n
        ey = wrap_angle(target[2] - pose[2])
        w_des = math.copysign(_approach_speed(abs(ey), self.max_yaw_rate, 0.8 * self.max_yaw_accel, 8.0), ey)
        return self.velocity_step(dt, v_des, w_des, pose[2])
