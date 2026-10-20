"""Launcher bench: fire the ball at a series of flywheel speeds on a stationary robot.

Measures exit speed, elevation, azimuth, spin, flywheel speed drop and recovery,
and writes launcher_cal.json (flywheel speed -> exit speed / elevation), which the
robot uses to pick a flywheel speed for a throw - the sim version of the design
doc's wheel-speed-to-distance calibration table.

  python launcher_bench.py                # default rpm sweep, writes launcher_cal.json
  python launcher_bench.py 2000 3000 4000 # specific rpms, no file written
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass

import mujoco
import numpy as np

from params import SimParams
from robot import CAL_FILE, Robot
from robot_model import build_model


@dataclass
class Shot:
    rpm: float
    exit_speed: float
    elevation: float      # deg
    azimuth: float        # deg relative to the robot heading
    spin: float           # rad/s magnitude
    rpm_drop: float       # fraction
    recovery: float       # s back to within 2%
    time_to_exit: float   # s from gate command
    ok: bool


class Bench:
    def __init__(self, p: SimParams | None = None):
        self.p = (p or SimParams()).with_(home_xy=(0.0, 0.0))
        self.m = build_model(self.p)
        self.d = mujoco.MjData(self.m)
        adr = self.m.joint("kiwi").qposadr[0]
        self.d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.m, self.d)
        self.r = Robot(self.m, self.d, self.p)
        j = self.m.joint("ball")
        self.bq, self.bv = j.qposadr[0], j.dofadr[0]
        self.dt = self.m.opt.timestep

    def run(self, seconds):
        for _ in range(int(round(seconds / self.dt))):
            self.r.step()

    def load_ball(self):
        """Drop the ball into the catch pocket and let it settle on the gate."""
        r, g = self.r, self.r.g
        self.d.qpos[self.bq:self.bq + 7] = [0, 0, g.plate_top + g.hold[2] + 0.02, 1, 0, 0, 0]
        self.d.qvel[self.bv:self.bv + 6] = 0
        self.run(0.4)
        return r.ball_held()

    def shot(self, rpm: float) -> Shot:
        r, d, g = self.r, self.d, self.r.g
        if not self.load_ball():
            return Shot(rpm, 0, 0, 0, 0, 0, 0, 0, False)
        r.fly_target = rpm * 2 * math.pi / 60
        t0 = d.time
        while not r.flywheels_ready() and d.time - t0 < 2.0:
            r.step()
        self.run(0.1)
        w0 = r.flywheel_speeds().mean()
        r.gate_open = True
        t_open = d.time
        exit_v, w_min, t_exit, t_min, spin = None, w0, 0.0, t_open, 0.0
        recovered = None
        while d.time - t_open < 1.0:
            r.step()
            w = r.flywheel_speeds().mean()
            if w < w_min:
                w_min, t_min = w, d.time
            rel = d.sensor("ball_pos").data - d.xpos[r.module]
            if exit_v is None and rel[0] > g.N[0] + 0.12:
                exit_v, t_exit = d.sensor("ball_vel").data.copy(), d.time - t_open
                spin = float(np.linalg.norm(d.sensor("ball_angvel").data))
            if exit_v is not None and recovered is None and d.time > t_min + 0.002 and abs(w - w0) < 0.02 * w0:
                recovered = d.time - t_min
            if exit_v is not None and recovered is not None:
                break
        r.gate_open = False
        if exit_v is None:
            return Shot(rpm, 0, 0, 0, 0, 1 - w_min / w0, 0, 0, False)
        sp = float(np.linalg.norm(exit_v))
        el = math.degrees(math.atan2(exit_v[2], math.hypot(exit_v[0], exit_v[1])))
        az = math.degrees(math.atan2(exit_v[1], exit_v[0]))
        return Shot(rpm, sp, el, az, spin, 1 - w_min / w0, recovered or float("nan"), t_exit, True)


def calibrate(p: SimParams | None = None, rpms=None, repeats: int = 2, write: bool = True, verbose: bool = True):
    p = p or SimParams()
    rpms = rpms or [1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000]
    bench = Bench(p)
    rows = []
    for rpm in rpms:
        shots = [bench.shot(rpm) for _ in range(repeats)]
        good = [s for s in shots if s.ok]
        if verbose:
            for s in shots:
                if s.ok:
                    print(f"{rpm:5.0f} rpm: exit {s.exit_speed:.2f} m/s at {s.elevation:.1f} deg, az {s.azimuth:+.1f}, "
                          f"spin {s.spin:.0f} rad/s, drop {100 * s.rpm_drop:.0f}%, recover {1000 * s.recovery:.0f} ms")
                else:
                    print(f"{rpm:5.0f} rpm: NO EXIT")
        if good:
            rows.append((rpm * 2 * math.pi / 60, np.mean([s.exit_speed for s in good]), np.mean([s.elevation for s in good]),
                         good))
    if write and len(rows) >= 2:
        cal = dict(flywheel_diameter=p.flywheel_diameter, flywheel_motor=p.flywheel_motor,
                   w=[r[0] for r in rows], v=[r[1] for r in rows], elev_deg=[r[2] for r in rows])
        CAL_FILE.write_text(json.dumps(cal, indent=1))
        if verbose:
            print(f"wrote {CAL_FILE}")
    return rows


if __name__ == "__main__":
    if len(sys.argv) > 1:
        calibrate(rpms=[float(a) for a in sys.argv[1:]], write=False)
    else:
        calibrate()
