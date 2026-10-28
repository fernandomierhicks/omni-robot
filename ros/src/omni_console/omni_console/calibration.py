"""Guided calibrations (the console's Calibrate tab), ROS-free. The bench list of design.md 7, as procedures a person
runs on the robot with a tape measure; each result is saved with its time (calibration.json) and says what to change.

  odom_line    drive D m straight on the odometry; you measure how far it really went  -> wheel distance scale
  odom_spin    spin N turns on the IMU heading; you say how far past the start mark   -> gyro (heading) scale
  imu_level    roll and pitch at rest, here and turned 180 deg                        -> IMU mount offset, floor slope
  launcher     fire at a list of rpm (Thrower bench mode); you measure each landing    -> rpm correction to the table
  camera_latency  the LED + photodiode test (design.md 7), entered by hand             -> recorded

The driving ones run as the mission CALIBRATE; the robot moves slowly (0.25 m/s, 45 deg/s) and the collision monitor
stops it as in any mode. Nothing is applied by itself: the results are numbers to put into the firmware's parameters or
the launcher table (robot_export.py), after a few repeats agree.
"""

from __future__ import annotations

import math
import statistics

PROCS = ("odom_line", "odom_spin", "imu_level", "launcher", "camera_latency")
LINE_SPEED, SPIN_RATE = 0.25, math.radians(45)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class DriveCal:
    """odom_line / odom_spin / imu_level. tick(now, pose, mcu) -> twist; phase: drive | settle | measure | done."""

    def __init__(self, proc: str, distance: float = 2.0, turns: int = 3):
        if proc not in ("odom_line", "odom_spin", "imu_level"):
            raise ValueError(proc)
        self.proc, self.distance, self.turns = proc, distance, turns
        self.phase = "start"
        self.t_phase = 0.0
        self.p0 = None
        self.turned = 0.0
        self.last_yaw = None
        self.samples = [[], []]          # imu_level: (roll, pitch) samples at 0 and 180 deg
        self.k = 0                       # imu_level: which half
        self.result = None
        self.odom = None                 # what the odometry says it did (for the person's measurement)

    def text(self) -> str:
        p = self.phase
        if self.proc == "odom_line":
            return {"start": "Starting", "drive": f"Driving {self.distance:g} m straight",
                    "settle": "Stopping", "measure": "Measure how far it really went, then enter it",
                    "done": "Done"}.get(p, p)
        if self.proc == "odom_spin":
            return {"start": "Starting", "drive": f"Spinning {self.turns} turns", "settle": "Stopping",
                    "measure": "How far past the start mark did it stop? (degrees, + past, - short)",
                    "done": "Done"}.get(p, p)
        return {"start": "Starting", "drive": "Measuring at rest (keep still)", "turn": "Turning 180 deg",
                "settle": "Settling", "done": "Done"}.get(p, p)

    def _set(self, phase, now):
        self.phase, self.t_phase = phase, now

    def tick(self, now: float, pose, mcu):
        """pose: (x, y, yaw) odom or None; mcu: dict with roll, pitch (rad), vx, vy, wz, or None."""
        if pose is None or self.phase in ("measure", "done"):
            return (0.0, 0.0, 0.0)
        x, y, yaw = pose
        if self.phase == "start":
            self.p0, self.last_yaw = pose, yaw
            self._set("drive", now)
        still = mcu is not None and math.hypot(mcu.get("vx", 0), mcu.get("vy", 0)) < 0.01 and abs(mcu.get("wz", 0)) < 0.01
        if self.proc == "odom_line":
            d = (x - self.p0[0]) * math.cos(self.p0[2]) + (y - self.p0[1]) * math.sin(self.p0[2])
            if self.phase == "drive":
                left = self.distance - d
                if left < 0.005 or now - self.t_phase > 4 * self.distance / LINE_SPEED + 5:
                    self._set("settle", now)
                    return (0.0, 0.0, 0.0)
                return (min(LINE_SPEED, 1.5 * left + 0.03), 0.0, 2.0 * wrap(self.p0[2] - yaw))
            if self.phase == "settle" and (still and now - self.t_phase > 0.5 or now - self.t_phase > 3.0):
                self.odom = math.hypot(x - self.p0[0], y - self.p0[1])
                self._set("measure", now)
            return (0.0, 0.0, 0.0)
        if self.proc == "odom_spin":
            self.turned += wrap(yaw - self.last_yaw)
            self.last_yaw = yaw
            goal = 2 * math.pi * self.turns
            if self.phase == "drive":
                left = goal - self.turned
                if left < math.radians(1.0) or now - self.t_phase > 3 * goal / SPIN_RATE + 5:
                    self._set("settle", now)
                    return (0.0, 0.0, 0.0)
                return (0.0, 0.0, min(SPIN_RATE, 2.0 * left + 0.05))
            if self.phase == "settle" and (still and now - self.t_phase > 0.5 or now - self.t_phase > 3.0):
                self.odom = self.turned
                self._set("measure", now)
            return (0.0, 0.0, 0.0)
        # imu_level: 5 s still, turn 180 deg, settle, 5 s still
        if self.phase == "drive":
            if mcu is not None and now - self.t_phase > 1.0:
                self.samples[self.k].append((mcu["roll"], mcu["pitch"]))
            if now - self.t_phase > 6.0:
                if self.k == 0:
                    self.k = 1
                    self.turned, self.last_yaw = 0.0, yaw
                    self._set("turn", now)
                else:
                    self._finish_level(now)
            return (0.0, 0.0, 0.0)
        if self.phase == "turn":
            self.turned += wrap(yaw - self.last_yaw)
            self.last_yaw = yaw
            left = math.pi - self.turned
            if left < math.radians(1.0) or now - self.t_phase > 15.0:
                self._set("settle", now)
                return (0.0, 0.0, 0.0)
            return (0.0, 0.0, min(SPIN_RATE, 2.0 * left + 0.05))
        if self.phase == "settle" and now - self.t_phase > 1.5:
            self._set("drive", now)
        return (0.0, 0.0, 0.0)

    def _finish_level(self, now):
        a = [(statistics.fmean(r for r, _ in s), statistics.fmean(p for _, p in s)) if s else (math.nan, math.nan)
             for s in self.samples]
        mount = (math.degrees((a[0][0] + a[1][0]) / 2), math.degrees((a[0][1] + a[1][1]) / 2))
        slope = (math.degrees((a[0][0] - a[1][0]) / 2), math.degrees((a[0][1] - a[1][1]) / 2))
        self.result = dict(mount_roll_deg=round(mount[0], 3), mount_pitch_deg=round(mount[1], 3),
                           floor_roll_deg=round(slope[0], 3), floor_pitch_deg=round(slope[1], 3),
                           samples=[len(s) for s in self.samples],
                           summary=f"IMU mount offset roll {mount[0]:+.2f} deg, pitch {mount[1]:+.2f} deg (the floor "
                                   f"here tilts {slope[0]:+.2f} / {slope[1]:+.2f} deg): subtract the offset in the "
                                   f"firmware's attitude")
        self._set("done", now)

    def measured(self, value: float) -> dict:
        """The person's measurement: metres driven (odom_line) or degrees past the mark (odom_spin)."""
        if self.phase != "measure":
            raise ValueError("not waiting for a measurement")
        if self.proc == "odom_line":
            if not 0.1 < value < 10:
                raise ValueError("the distance in metres, please")
            k = value / self.odom
            self.result = dict(odometry_m=round(self.odom, 4), measured_m=value, scale=round(k, 4),
                               summary=f"The wheels say {self.odom:.3f} m, it went {value:.3f} m: multiply the "
                                       f"wheel radius by {k:.4f}")
        else:
            if not -180 < value < 180:
                raise ValueError("degrees past the mark, -180 to 180")
            true = 2 * math.pi * self.turns + math.radians(value)
            k = true / self.odom
            self.result = dict(odometry_deg=round(math.degrees(self.odom), 2), true_deg=round(math.degrees(true), 2),
                               scale=round(k, 5),
                               summary=f"The heading says {math.degrees(self.odom):.1f} deg, it turned "
                                       f"{math.degrees(true):.1f} deg: gyro scale {k:.5f}")
        self.phase = "done"
        return self.result


def launcher_fit(shots, rc, flight) -> dict:
    """shots: [{rpm, tilt_deg, distance_m}] from the bench (the landing's first bounce, from the robot's centre, along
    its heading). For each, the exit speed that lands there (the ball's drag, the module's geometry), and the rpm the
    robot's launcher table (rc: omni_catch RobotConfig) would use for that speed: their ratio is the correction."""
    rows, ratios = [], []
    z_floor = rc.ball["radius"]
    for s in shots:
        tilt = math.radians(float(s["tilt_deg"]))
        ex, ez = rc.exit_point(tilt)
        v = 6.0
        for _ in range(4):
            el = rc.elevation_at(v, tilt)
            v = flight.solve_launch_speed([ex, 0.0, ez], [1.0, 0.0], el, [float(s["distance_m"]), 0.0, z_floor])
        rpm_table = rc.speed_for(v, tilt) * 60 / (2 * math.pi)
        ratio = float(s["rpm"]) / rpm_table if rpm_table > 0 else math.nan
        rows.append(dict(rpm=float(s["rpm"]), tilt_deg=float(s["tilt_deg"]), distance_m=float(s["distance_m"]),
                         exit_speed=round(v, 3), table_rpm=round(rpm_table), ratio=round(ratio, 4)))
        if math.isfinite(ratio):
            ratios.append(ratio)
    if not ratios:
        raise ValueError("no usable shots")
    k = statistics.median(ratios)
    spread = (max(ratios) - min(ratios)) if len(ratios) > 1 else 0.0
    return dict(rows=rows, rpm_scale=round(k, 4), spread=round(spread, 4),
                summary=(f"The launcher needs {100 * (k - 1):+.1f}% rpm against its table (median of {len(ratios)} "
                         f"shots, spread {100 * spread:.1f}%): scale the table's rpm by {k:.3f}"))
