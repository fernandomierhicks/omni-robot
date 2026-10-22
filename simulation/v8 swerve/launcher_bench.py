"""Launcher bench: fire the ball at a grid of flywheel speeds and module tilts on a stationary robot.

The tilt sets the launch elevation (launch_angle_deg - tilt) but also tilts the
gravity feed (drop tube and J), so the bench checks that the ball still reaches
the nip at every launch tilt. Writes launcher_cal.json: per tilt, flywheel
speed -> exit speed and elevation.

  python launcher_bench.py                     # default grid, writes launcher_cal.json
  python launcher_bench.py --tilts -20 0 --rpm 2000 3000   # spot checks, nothing written
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass

import mujoco
import numpy as np

from params import SimParams
from robot import Robot, cal_file
from robot_model import build_model

TILTS = {"j_bend": [-20.0, -10.0, 0.0, 5.0], "kicker": [-30.0, -20.0, -10.0, 0.0], "roller": [-30.0, -20.0, -10.0, 0.0],
         "chute": [-30.0, -20.0, -10.0, 0.0]}
RPMS = [1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000]


@dataclass
class Shot:
    rpm: float
    tilt_deg: float
    exit_speed: float
    elevation: float      # deg
    azimuth: float        # deg relative to the robot heading
    spin: float
    rpm_drop: float
    time_to_exit: float   # s from gate command
    ok: bool


class Bench:
    def __init__(self, p: SimParams | None = None):
        self.p = (p or SimParams()).with_(home_xy=(0.0, 0.0))
        self.m = build_model(self.p)
        self.d = mujoco.MjData(self.m)
        adr = self.m.joint("base").qposadr[0]
        self.d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.m, self.d)
        self.r = Robot(self.m, self.d, self.p)
        j = self.m.joint("ball")
        self.bq, self.bv = j.qposadr[0], j.dofadr[0]
        self.dt = self.m.opt.timestep

    def run(self, seconds):
        for _ in range(int(round(seconds / self.dt))):
            self.r.step()

    def module_point(self, xyz) -> np.ndarray:
        b = self.r.module
        return self.d.xpos[b] + self.d.xmat[b].reshape(3, 3) @ np.asarray(xyz, dtype=float)

    def set_tilt(self, tilt_deg: float):
        self.r.tilt_target = math.radians(tilt_deg)
        t0 = self.d.time
        while not self.r.tilt_settled() and self.d.time - t0 < 2.0:
            self.r.step()
        self.run(0.2)

    def load_ball(self) -> bool:
        """Drop the ball into the pocket (at the current tilt) and let it settle on the gate."""
        g = self.r.g
        pos = self.module_point((g.hold[0], 0, g.hold[2] + 0.003))     # below the throat flaps
        self.d.qpos[self.bq:self.bq + 7] = [*pos, 1, 0, 0, 0]
        self.d.qvel[self.bv:self.bv + 6] = 0
        self.run(0.4)
        return self.r.ball_held()

    def shot(self, rpm: float, tilt_deg: float = 0.0) -> Shot:
        r, d, g = self.r, self.d, self.r.g
        self.set_tilt(tilt_deg)
        bad = Shot(rpm, tilt_deg, 0, 0, 0, 0, 0, 0, False)
        if not self.load_ball():
            return bad
        r.fly_target = rpm * 2 * math.pi / 60
        t0 = d.time
        while not r.flywheels_ready() and d.time - t0 < 2.0:
            r.step()
        self.run(0.1)
        w0 = r.flywheel_speeds().mean()
        nip = self.module_point((g.N[0], 0, g.N[1]))
        r.gate_open = True
        t_open = d.time
        exit_v, w_min, t_exit, spin = None, w0, 0.0, 0.0
        while d.time - t_open < 1.0:
            r.step()
            w_min = min(w_min, r.flywheel_speeds().mean())
            if np.linalg.norm(d.sensor("ball_pos").data - nip) > 0.15 and d.sensor("ball_vel").data[2] > -1.0:
                exit_v, t_exit = d.sensor("ball_vel").data.copy(), d.time - t_open
                spin = float(np.linalg.norm(d.sensor("ball_angvel").data))
                break
        self.run(0.3)
        r.gate_open = False
        r.fly_target = 0.0
        self.run(0.3)
        if exit_v is None or exit_v[0] < 0.5:
            return Shot(rpm, tilt_deg, 0, 0, 0, 0, 1 - w_min / w0, 0, False)
        sp = float(np.linalg.norm(exit_v))
        el = math.degrees(math.atan2(exit_v[2], math.hypot(exit_v[0], exit_v[1])))
        az = math.degrees(math.atan2(exit_v[1], exit_v[0]))
        return Shot(rpm, tilt_deg, sp, el, az, spin, 1 - w_min / w0, t_exit, True)


def calibrate(p: SimParams | None = None, tilts=None, rpms=None, repeats: int = 2, write: bool = True,
              verbose: bool = True):
    p = p or SimParams()
    tilts = TILTS[p.feed] if tilts is None else tilts
    rpms = rpms or RPMS
    bench = Bench(p)
    table, shots_all = [], []
    for tilt in tilts:
        rows = []
        for rpm in rpms:
            shots = [bench.shot(rpm, tilt) for _ in range(repeats)]
            shots_all += shots
            good = [s for s in shots if s.ok]
            if verbose:
                for s in shots:
                    print(f"tilt {tilt:+5.1f} deg, {rpm:5.0f} rpm: " + (
                        f"exit {s.exit_speed:.2f} m/s at {s.elevation:.1f} deg (geometric {p.launch_angle_deg - tilt:.0f}), "
                        f"az {s.azimuth:+.1f}, drop {100 * s.rpm_drop:.0f}%, {1000 * s.time_to_exit:.0f} ms"
                        if s.ok else "NO EXIT"))
            if good:
                rows.append((rpm * 2 * math.pi / 60, float(np.mean([s.exit_speed for s in good])),
                             float(np.mean([s.elevation for s in good]))))
        if len(rows) >= 2:
            table.append(dict(tilt_deg=tilt, w=[r[0] for r in rows], v=[r[1] for r in rows],
                              elev_deg=[r[2] for r in rows]))
    if write and table:
        cal = dict(flywheel_diameter=p.flywheel_diameter, flywheel_motor=p.flywheel_motor,
                   launch_angle_deg=p.launch_angle_deg, tilts=table)
        cal_file(p, write=True).write_text(json.dumps(cal, indent=1))
        if verbose:
            print(f"wrote {cal_file(p, write=True)}")
    return table, shots_all


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tilts", type=float, nargs="*")
    ap.add_argument("--rpm", type=float, nargs="*")
    ap.add_argument("--repeats", type=int, default=2)
    a = ap.parse_args()
    spot = a.tilts is not None or a.rpm is not None
    calibrate(tilts=a.tilts, rpms=a.rpm, repeats=a.repeats, write=not spot)
