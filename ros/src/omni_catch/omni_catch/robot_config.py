"""The robot as the Jetson's code knows it (design.md 30): config/robot_v11.json, written from the sim by
simulation/v11 sil/robot_export.py. Geometry of the tilting module, the launcher calibration, the ball, the cameras,
and the base's planning limits (the firmware enforces its own copy of those)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

CONFIG = Path(__file__).resolve().parent / "config" / "robot_v11.json"


class RobotConfig:
    def __init__(self, path: str | Path | None = None):
        self.raw = json.loads(Path(path or CONFIG).read_text())
        b, mo, la = self.raw["base"], self.raw["module"], self.raw["launcher"]
        self.base_link_height = b["base_link_height"]
        self.robot_radius = b["robot_radius"]
        self.J = np.array(b["J"])
        self.max_wheel_speed = b["max_wheel_speed"]
        self.max_speed = b["max_speed"]
        self.brake_margin = b["brake_margin"]
        self.accel_table = b["accel_table"]
        self.max_accel = b["max_accel"]
        self.kid = b["kid"]
        self.tilt_limits = tuple(mo["tilt_limits"])
        self._tilt = np.radians(mo["tilt_deg"])
        self._rim = np.array(mo["rim"])
        self._exit = np.array(mo["exit"])
        self.funnel_rim_radius = mo["funnel_rim_radius"]
        self.launch_angle = math.radians(mo["launch_angle_deg"])
        self.cal_tilts = np.array(la["tilt_rad"])
        self.cal_w = [np.array(w) for w in la["w"]]
        self.cal_v = [np.array(v) for v in la["v"]]
        self.cal_de = [np.array(e) for e in la["d_elev"]]
        self.ball = self.raw["ball"]
        self.cameras = self.raw["cameras"]

    # -- module geometry (robot frame: x forward, z up, from the floor) ---------------------------------------------
    def rim_center(self, tilt: float) -> np.ndarray:
        """Funnel rim center (x, z) at a tilt."""
        return np.array([np.interp(tilt, self._tilt, self._rim[:, 0]), np.interp(tilt, self._tilt, self._rim[:, 1])])

    def exit_point(self, tilt: float) -> np.ndarray:
        """The launch exit (x, z) at a tilt."""
        return np.array([np.interp(tilt, self._tilt, self._exit[:, 0]), np.interp(tilt, self._tilt, self._exit[:, 1])])

    # -- launcher calibration (robot.LauncherCal) ----------------------------------------------------------------
    def _blend(self, tilt: float, fn):
        if len(self.cal_tilts) == 1:
            return fn(0)
        k = int(np.clip(np.searchsorted(self.cal_tilts, tilt) - 1, 0, len(self.cal_tilts) - 2))
        f = float(np.clip((tilt - self.cal_tilts[k]) / (self.cal_tilts[k + 1] - self.cal_tilts[k]), 0.0, 1.0))
        return (1 - f) * fn(k) + f * fn(k + 1)

    def speed_for(self, v_exit: float, tilt: float) -> float:
        """Flywheel rad/s for an exit speed."""
        return self._blend(tilt, lambda k: float(np.interp(v_exit, self.cal_v[k], self.cal_w[k])))

    def elevation_at(self, v_exit: float, tilt: float) -> float:
        return self.launch_angle - tilt + self._blend(tilt, lambda k: float(np.interp(v_exit, self.cal_v[k], self.cal_de[k])))

    def tilt_for(self, v_exit: float, elevation: float) -> float:
        tilt = self.launch_angle - elevation
        for _ in range(3):
            tilt = self.launch_angle - elevation + \
                self._blend(tilt, lambda k: float(np.interp(v_exit, self.cal_v[k], self.cal_de[k])))
        return tilt

    # -- base planning limits (the firmware's adult profile) ----------------------------------------------------
    def plan_accel(self, kid: bool = False) -> float:
        """The intercept planner's acceleration: the worst direction of the firmware's table."""
        a = min(self.accel_table[1]) if self.accel_table else self.max_accel
        return a * (self.kid["accel_scale"] if kid else 1.0)

    def top_speed(self, u_world, yaw: float, kid: bool = False) -> float:
        """Fastest translation in a world direction: the speed cap or the wheels' ceiling (omni_control.top_speed)."""
        c, s = math.cos(yaw), math.sin(yaw)
        ub = np.array([c * u_world[0] + s * u_world[1], -s * u_world[0] + c * u_world[1]])
        n = float(np.linalg.norm(ub))
        cap = self.kid["max_speed"] if kid else self.max_speed
        if n < 1e-9:
            return cap
        k = float(np.max(np.abs(self.J[:, :2] @ (ub / n))))
        return min(cap, self.max_wheel_speed / k)
