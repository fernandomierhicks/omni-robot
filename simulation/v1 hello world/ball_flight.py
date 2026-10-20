"""Ball flight with air drag: landing prediction and throw solvers.

The drag law matches MuJoCo's ellipsoid fluid model for a sphere (blunt drag
rho * A * (Cd/2) * |v| * v plus Stokes drag 6 pi mu r v), so predictions made
from the true ball state agree with the simulator. Spin (Magnus) is ignored.
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
        self.g = np.array([0.0, 0.0, -gravity])

    def accel(self, v: np.ndarray) -> np.ndarray:
        return self.g - (self.k_lin + self.k_quad * np.linalg.norm(v)) * v

    def _rk4(self, p, v, dt):
        a1 = self.accel(v)
        v2 = v + 0.5 * dt * a1
        a2 = self.accel(v2)
        v3 = v + 0.5 * dt * a2
        a3 = self.accel(v3)
        v4 = v + dt * a3
        a4 = self.accel(v4)
        p = p + dt / 6 * (v + 2 * v2 + 2 * v3 + v4)
        v = v + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
        return p, v

    def crossing(self, pos, vel, z_plane: float, dt: float = 0.004, t_max: float = 4.0):
        """Where and when the ball next crosses z_plane going down: (xyz, t) or None."""
        p, v = np.array(pos, dtype=float), np.array(vel, dtype=float)
        t = 0.0
        while t < t_max:
            p1, v1 = self._rk4(p, v, dt)
            if v1[2] < 0 and p[2] >= z_plane > p1[2]:
                f = (p[2] - z_plane) / (p[2] - p1[2])
                return p + f * (p1 - p), t + f * dt
            p, v, t = p1, v1, t + dt
        return None

    def _horizontal_range(self, pos, vel, z_plane):
        hit = self.crossing(pos, vel, z_plane)
        if hit is None:
            return 0.0
        return float(np.hypot(hit[0][0] - pos[0], hit[0][1] - pos[1]))

    def solve_toss(self, release, target, apex_height: float) -> np.ndarray:
        """Release velocity that peaks apex_height above release and comes down
        through target (x, y, z) - the way a person lobs underhand."""
        release = np.asarray(release, dtype=float)
        d = np.asarray(target[:2], dtype=float) - release[:2]
        dist = float(np.linalg.norm(d))
        u = d / dist
        vz = math.sqrt(2 * 9.81 * apex_height)

        def rng(s):
            return self._horizontal_range(release, [u[0] * s, u[1] * s, vz], target[2])

        lo, hi = 0.0, 20.0
        for _ in range(40):
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
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if rng(mid) < dist else (lo, mid)
        return 0.5 * (lo + hi)
