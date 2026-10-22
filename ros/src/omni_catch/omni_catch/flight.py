"""Ball flight with air drag: landing prediction and throw solvers. (v11, design.md 30: moved from the sim's
ball_flight.py into the robot's code, with release_point() added; the sim's copy stays for the earlier versions.)

The drag law matches MuJoCo's ellipsoid fluid model for a sphere (blunt drag
rho * A * (Cd/2) * |v| * v plus Stokes drag 6 pi mu r v), so predictions made
from the true ball state agree with the simulator. Spin (Magnus) is ignored.
Integration uses plain floats (RK4): this runs inside the control loop.
"""

from __future__ import annotations

import math

import numpy as np


class BallFlight:
    def __init__(self, mass: float, radius: float, drag_cd: float,
                 air_density: float = 1.204, air_viscosity: float = 1.8e-5, gravity: float = 9.81):
        area = math.pi * radius ** 2
        self.k_quad = air_density * area * drag_cd / 2 / mass
        self.k_lin = 6 * math.pi * air_viscosity * radius / mass
        self.gz = gravity

    def _acc(self, vx, vy, vz):
        k = self.k_lin + self.k_quad * math.sqrt(vx * vx + vy * vy + vz * vz)
        return -k * vx, -k * vy, -self.gz - k * vz

    def accel(self, v) -> np.ndarray:
        return np.array(self._acc(*v))

    def crossing(self, pos, vel, z_plane: float, dt: float = 0.004, t_max: float = 4.0):
        """Where and when the ball next crosses z_plane going down: (xyz, t) or None."""
        hit = self.crossing_state(pos, vel, z_plane, dt, t_max)
        return None if hit is None else (hit[0], hit[2])

    def crossing_state(self, pos, vel, z_plane: float, dt: float = 0.004, t_max: float = 4.0):
        """(xyz, velocity, t) where the ball next crosses z_plane going down, or None."""
        x, y, z = (float(c) for c in pos)
        vx, vy, vz = (float(c) for c in vel)
        acc = self._acc
        t = 0.0
        h = dt / 2
        while t < t_max:
            a1 = acc(vx, vy, vz)
            v2 = (vx + h * a1[0], vy + h * a1[1], vz + h * a1[2])
            a2 = acc(*v2)
            v3 = (vx + h * a2[0], vy + h * a2[1], vz + h * a2[2])
            a3 = acc(*v3)
            v4 = (vx + dt * a3[0], vy + dt * a3[1], vz + dt * a3[2])
            a4 = acc(*v4)
            nx = x + dt / 6 * (vx + 2 * v2[0] + 2 * v3[0] + v4[0])
            ny = y + dt / 6 * (vy + 2 * v2[1] + 2 * v3[1] + v4[1])
            nz = z + dt / 6 * (vz + 2 * v2[2] + 2 * v3[2] + v4[2])
            nvz = vz + dt / 6 * (a1[2] + 2 * a2[2] + 2 * a3[2] + a4[2])
            nvx = vx + dt / 6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0])
            nvy = vy + dt / 6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1])
            if nvz < 0 and z >= z_plane > nz:
                f = (z - z_plane) / (z - nz)
                return (np.array([x + f * (nx - x), y + f * (ny - y), z_plane]),
                        np.array([vx + f * (nvx - vx), vy + f * (nvy - vy), vz + f * (nvz - vz)]), t + f * dt)
            x, y, z, vx, vy, vz, t = nx, ny, nz, nvx, nvy, nvz, t + dt
        return None

    def apex(self, pos, vel, dt: float = 0.004) -> float:
        """Highest z the ball reaches (drag, no spin)."""
        z, (vx, vy, vz) = float(pos[2]), (float(c) for c in vel)
        top = z
        while vz > 0:
            a = self._acc(vx, vy, vz)
            v2 = (vx + dt / 2 * a[0], vy + dt / 2 * a[1], vz + dt / 2 * a[2])
            a2 = self._acc(*v2)
            z += dt * v2[2]
            vx, vy, vz = vx + dt * a2[0], vy + dt * a2[1], vz + dt * a2[2]
            top = max(top, z)
        return top

    def release_point(self, pos, vel, z_release: float, dt: float = 0.004, t_max: float = 3.0):
        """Where the ball was when it last rose through z_release (the thrower's hand, design.md 7: back-extrapolate
        the trajectory), integrating backward in time: (xyz, seconds before now) or None."""
        x, y, z = (float(c) for c in pos)
        vx, vy, vz = (float(c) for c in vel)
        t = 0.0
        while t < t_max:
            ax, ay, az = self._acc(vx, vy, vz)
            # one backward step (midpoint)
            mx, my, mz = vx - dt / 2 * ax, vy - dt / 2 * ay, vz - dt / 2 * az
            bx, by, bz = self._acc(mx, my, mz)
            nx, ny, nz = x - dt * mx, y - dt * my, z - dt * mz
            nvx, nvy, nvz = vx - dt * bx, vy - dt * by, vz - dt * bz
            if nvz > 0 and z >= z_release > nz:
                f = (z - z_release) / (z - nz)
                return np.array([x + f * (nx - x), y + f * (ny - y), z_release]), t + f * dt
            x, y, z, vx, vy, vz, t = nx, ny, nz, nvx, nvy, nvz, t + dt
        return None

    def _horizontal_range(self, pos, vel, z_plane):
        hit = self.crossing(pos, vel, z_plane)
        if hit is None:
            return 0.0
        return float(math.hypot(hit[0][0] - pos[0], hit[0][1] - pos[1]))

    def solve_toss(self, release, target, apex_height: float) -> np.ndarray:
        """Release velocity that peaks apex_height above release and comes down
        through target (x, y, z) - the way a person lobs underhand."""
        release = np.asarray(release, dtype=float)
        d = np.asarray(target[:2], dtype=float) - release[:2]
        dist = float(np.linalg.norm(d))
        u = d / dist
        vz = math.sqrt(2 * self.gz * apex_height)

        def rng(s):
            return self._horizontal_range(release, [u[0] * s, u[1] * s, vz], target[2])

        lo, hi = 0.0, 20.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if rng(mid) < dist else (lo, mid)
        s = 0.5 * (lo + hi)
        return np.array([u[0] * s, u[1] * s, vz])

    def solve_launch_speed(self, exit_pos, direction_xy, elevation: float, target) -> float:
        """Launch speed at a fixed elevation (rad) that brings the ball down through target."""
        exit_pos = np.asarray(exit_pos, dtype=float)
        d = np.asarray(target[:2], dtype=float) - exit_pos[:2]
        dist = float(np.linalg.norm(d))
        u = np.asarray(direction_xy, dtype=float)
        u = u / np.linalg.norm(u)
        ce, se = math.cos(elevation), math.sin(elevation)

        def rng(s):
            return self._horizontal_range(exit_pos, [u[0] * s * ce, u[1] * s * ce, s * se], target[2])

        lo, hi = 0.0, 25.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if rng(mid) < dist else (lo, mid)
        return 0.5 * (lo + hi)
