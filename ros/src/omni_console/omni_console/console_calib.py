"""Calibration (calibration.py) and the launcher fit, part of missions.Console. ROS-free."""

from __future__ import annotations

from omni_console.console_defs import (LAUNCHER_RPMS, Reply)  # noqa: F401


class CalibMixin:
    def calib_state(self) -> dict:
        c = self.calib_run
        return dict(results=self.calib_results, shots=self.calib_shots, rpms=list(LAUNCHER_RPMS),
                    run=None if c is None else dict(proc=c.proc, phase=c.phase, text=c.text(), odom=c.odom,
                                                    result=c.result))

    def _calib_save(self, proc: str, result: dict):
        self.calib_results.setdefault(proc, []).append(dict(result, t=round(self.wall_time(), 1)))
        self.calib_results[proc] = self.calib_results[proc][-20:]
        self.calib_changed = True
        self.event("info", f"Calibration ({proc}): {result.get('summary', 'saved')}")

    def wall_time(self) -> float:
        import time
        return time.time()

    def _calib(self, verb: str, a: dict) -> Reply:
        from omni_console.calibration import PROCS, DriveCal, launcher_fit
        if verb == "calib":
            return Reply(True, "", self.calib_state())
        if verb == "calib_start":
            proc = str(a["proc"])
            if proc not in ("odom_line", "odom_spin", "imu_level"):
                return Reply(False, f"'{proc}' isn't a driving calibration (one of odom_line, odom_spin, imu_level)")
            if self.odom_pose is None:
                return Reply(False, "No odometry from the robot yet")
            r = self._enter("CALIBRATE", f"Calibrating: {proc}", "info")
            self.calib_run = DriveCal(proc, self.values["calib.distance"], int(self.values["calib.turns"]))
            return Reply(True, r.message, self.calib_state(), r.effects)
        if verb == "calib_measure":
            c = self.calib_run
            if c is None:
                return Reply(False, "No calibration is waiting for a measurement")
            res = c.measured(float(a["value"]))
            self._calib_save(c.proc, res)
            self.task = f"Calibration done: {res['summary']}"
            return Reply(True, res["summary"], self.calib_state())
        if verb == "calib_shot":
            if self.mission != "THROWER":
                return Reply(False, "The launcher calibration runs in the Thrower bench mode: press 'Take control'")
            d = float(a["distance_m"])
            if not 0.3 < d < 15:
                return Reply(False, "The landing distance in metres, from the robot's centre")
            shot = dict(rpm=float(a.get("rpm", self.thrower["rpm"])), tilt_deg=float(a.get("tilt_deg", self.thrower["tilt"])),
                        distance_m=d)
            if shot["rpm"] <= 0:
                return Reply(False, "The flywheels were off: set the rpm and fire first")
            self.calib_shots.append(shot)
            return Reply(True, f"Shot {len(self.calib_shots)}: {shot['rpm']:.0f} rpm, {d:.2f} m", self.calib_state())
        if verb == "calib_fit":
            from omni_catch.flight import BallFlight
            from omni_catch.robot_config import RobotConfig
            rc = RobotConfig()
            b = rc.ball
            fl = BallFlight(b["mass"], b["radius"], b["drag_cd"], b["air_density"], b["air_viscosity"])
            res = launcher_fit(self.calib_shots, rc, fl)
            self._calib_save("launcher", res)
            self.calib_shots = []
            return Reply(True, res["summary"], self.calib_state())
        if verb == "calib_record":
            proc = str(a["proc"])
            if proc not in PROCS:
                return Reply(False, f"No calibration '{proc}'")
            v = float(a["value"])
            self._calib_save(proc, dict(value=v, note=str(a.get("note", ""))[:200],
                                        summary=f"{proc}: {v:g}" + (" ms" if proc == "camera_latency" else "")))
            return Reply(True, "Recorded", self.calib_state())
        if verb == "calib_clear":
            proc = str(a.get("proc", ""))
            if proc == "shots":
                self.calib_shots = []
            elif self.calib_results.pop(proc, None) is not None:
                self.calib_changed = True
            return Reply(True, "Cleared", self.calib_state())
        return Reply(False, f"Unknown command '{verb}'")
