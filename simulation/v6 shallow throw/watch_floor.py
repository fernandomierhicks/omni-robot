"""Watch the robot on real floors (design.md 23.4): thresholds, rug edges, friction patches, the home floor.

Two scenes:
  cross   the base drives across one obstacle again and again with a ball in the pocket (studies/floor_wheels.py B)
  game    the 6-7 m catch game on a floor of studies/floor_wheels.py C (thresholds and rugs drawn in the arena)

  python watch_floor.py                                   cross a 12 mm threshold at 1 m/s, roller model, body 0 deg
  python watch_floor.py cross --obstacle "rug edge 8 mm" --dir 45      the X4 stuck on a square rug edge (two wheels sideways)
  python watch_floor.py cross --obstacle "threshold 18 mm" --speed 2   the hop over a high threshold
  python watch_floor.py cross --model sphere ...          the fast sphere wheel (it wrongly climbs square edges sideways)
  python watch_floor.py game --floor "home floor (field, threshold, rug)"            catch game on the home floor
  python watch_floor.py game --floor "home floor, <= 1 m/s over the threshold"       ... with the firmware slow zone
  python watch_floor.py flip                              replay: threshold hop at 1.7 m/s lands on a missed ball (3rd toss)
  python watch_floor.py flip --no-skirt                   replay v6's first flip: a missed ball in a wheel slot (no skirt)
  python watch_floor.py --list                            obstacles and floors
  python watch_floor.py ... --speed-factor 0.25           slow motion;  --record floor.mp4 -s 20   video instead of a viewer
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "studies"))

import floor_wheels as F  # noqa: E402
import shallow_sweep as S  # noqa: E402
from catch_game import GameConfig  # noqa: E402
from common import tilt_deg  # noqa: E402
from launcher_bench import Bench  # noqa: E402
from omni_control import wrap_angle  # noqa: E402
from params import SimParams  # noqa: E402
from robot_model import build_model, describe  # noqa: E402


class Crossing:
    """Drive across one obstacle (at world x = 0.6, along y) at a body direction and speed, ball aboard; repeat."""

    def __init__(self, args):
        self.args = args
        self.p = SimParams(wheel_model=args.model, floor_features=(F.OBSTACLES[args.obstacle],))
        self.b = Bench(self.p)
        self.m, self.d, self.r = self.b.m, self.b.d, self.b.r
        self.yaw = -math.radians(args.dir)
        self.runs, self.last = 0, ""
        self.reset()

    def reset(self):
        m, d = self.m, self.d
        adr = m.joint("base").qposadr[0]
        d.qpos[adr:adr + 7] = [-0.15, 0.0, 0.0, math.cos(self.yaw / 2), 0, 0, math.sin(self.yaw / 2)]
        d.qvel[m.joint("base").dofadr[0]:m.joint("base").dofadr[0] + 6] = 0
        self.r.ctl.reset()
        mujoco.mj_forward(m, d)
        self.b.run(0.3)
        self.b.load_ball()
        self.t0, self.tilt, self.phase = d.time, 0.0, "drive"

    def step(self):
        r, d = self.r, self.d
        pose = r.pose()
        if self.phase == "drive":
            w = float(np.clip(4.0 * wrap_angle(self.yaw - pose[2]), -2.0, 2.0))
            r.drive_velocity((self.args.speed, 0.0), w, pose[2], a_cap=2.0)
            if pose[0] > 1.25 or d.time - self.t0 > 1.0 + 1.25 / self.args.speed * 2.5:
                ok = pose[0] > 1.25
                self.last = (f"run {self.runs + 1}: {'crossed' if ok else 'STUCK'}, peak tilt {self.tilt:.0f} deg, "
                             f"ball {'kept' if r.ball_held() else 'LOST'}")
                print(self.last)
                self.phase, self.t_stop = "stop", d.time
        else:
            r.drive_velocity((0.0, 0.0), 0.0, pose[2], a_cap=2.0)
            if d.time - self.t_stop > 1.0:
                self.runs += 1
                self.reset()
                return
        r.step()
        if pose[0] > 0.2:
            self.tilt = max(self.tilt, tilt_deg(d))

    def hud(self):
        a = self.args
        return [f"{a.obstacle}, {a.model} wheels", f"moving at body {a.dir:g} deg, {a.speed:g} m/s",
                f"tilt {tilt_deg(self.d):4.1f} deg  ball {'aboard' if self.r.ball_held() else 'out'}", self.last]

    def done(self):
        return self.args.runs and self.runs >= self.args.runs


class Game:
    """The 6-7 m catch game (firmware adult, expected cameras) on one of floor_wheels.FLOORS."""

    def __init__(self, args, extra=None):
        self.args = args
        pkw = {**S.R3, **S.FW, **F.FLOORS[args.floor], **(extra or {})}
        self.p = SimParams(home_xy=S.HOME, thrower_xy=(S.HOME[0] - args.dist - 0.3, 0.0), area_size=S.AREA,
                           wheel_model=args.model, **pkw)
        self.m = build_model(self.p)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        cfg = GameConfig(**{**dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(args.apex, args.apex),
                                   return_throws=False, wait_between=1.2, perception="vision", vision_cfg=S.EXPECTED,
                                   toss_spin=6.0), **S.FWG, **S.V6G})
        self.game = S.HumanToss(self.m, self.d, self.p, cfg=cfg, seed=args.seed, verbose=True, dist=args.dist)
        self.r = self.game.robot
        self.tilt = 0.0

    def step(self):
        self.game.step()
        self.tilt = max(self.tilt, tilt_deg(self.d))

    def hud(self):
        g = self.game
        k = sum(x.caught for x in g.results)
        cap = self.r.ctl.max_speed
        return [f"{self.args.floor}, {self.args.model} wheels", f"{g.state}: caught {k}/{len(g.results)}",
                f"base {np.linalg.norm(self.d.sensor('base_linvel').data[:2]):.2f} m/s"
                + (f" (slow zone: {cap:.1f})" if cap < 1.9 else ""),
                f"tilt {tilt_deg(self.d):4.1f} deg, peak {self.tilt:.0f}"]

    def done(self):
        return self.args.runs and len(self.game.results) >= self.args.runs and self.game.state == "wait"


def add_floor_markers(scn, sim):
    """Outline the friction patches and slow zones on the floor (they have no geometry of their own)."""
    zones = [(z[:4], (0.2, 0.5, 1.0, 0.35)) for z in sim.p.floor_patches] + \
            [(z[:4], (1.0, 0.6, 0.1, 0.30)) for z in sim.p.slow_zones]
    for (x0, y0, x1, y1), rgba in zones:
        if scn.ngeom >= scn.maxgeom:
            return
        mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_BOX,
                            np.array([abs(x1 - x0) / 2, abs(y1 - y0) / 2, 0.0005]),
                            np.array([(x0 + x1) / 2, (y0 + y1) / 2, 0.0006]), np.eye(3).flatten(), np.array(rgba))
        scn.ngeom += 1


def follow(cam, sim, dist=1.6):
    cam.lookat[:] = sim.d.xpos[sim.m.body("base").id] + np.array([0.0, 0.0, 0.15])
    cam.distance = dist


def viewer(sim, args):
    import mujoco.viewer
    with mujoco.viewer.launch_passive(sim.m, sim.d) as v:
        v.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.6, 140, -18
        frame = 1 / 60
        while v.is_running() and not sim.done():
            t0 = time.perf_counter()
            stop = sim.d.time + frame * args.speed_factor
            while sim.d.time < stop:
                sim.step()
            if args.cam == "follow":
                follow(v.cam, sim, v.cam.distance)
            with v.lock():
                v.user_scn.ngeom = 0
                add_floor_markers(v.user_scn, sim)
                try:
                    v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                 "\n".join(sim.hud()), ""))
                except Exception:
                    pass
            v.sync()
            time.sleep(max(0.0, frame - (time.perf_counter() - t0)))


def record(sim, args, fps=30, width=1280, height=720):
    import imageio
    from PIL import Image, ImageDraw, ImageFont
    ren = mujoco.Renderer(sim.m, height, width)
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
    try:
        font = ImageFont.truetype("consola.ttf", 18)
    except OSError:
        font = ImageFont.load_default()
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation = 140, -18
    writer = imageio.get_writer(args.record, fps=fps, quality=8, macro_block_size=8)
    nxt, n = 0.0, 0
    t_end = sim.d.time + args.seconds
    while sim.d.time < t_end and not sim.done():
        sim.step()
        if sim.d.time >= nxt:
            nxt = sim.d.time + 1 / fps / args.speed_factor
            follow(cam, sim, 1.8)
            ren.update_scene(sim.d, camera=cam, scene_option=opt)
            add_floor_markers(ren.scene, sim)
            img = Image.fromarray(ren.render())
            draw = ImageDraw.Draw(img, "RGBA")
            lines = sim.hud()
            draw.rectangle((10, 10, 520, 18 + 24 * len(lines)), fill=(0, 0, 0, 120))
            for k, line in enumerate(lines):
                draw.text((20, 16 + 24 * k), line, fill=(255, 255, 255, 255), font=font)
            writer.append_data(np.asarray(img))
            n += 1
    writer.close()
    print(f"wrote {args.record} ({n} frames)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scene", nargs="?", default="cross", choices=["cross", "game", "flip"])
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--obstacle", default="threshold 12 mm", help="cross: a floor_wheels.OBSTACLES name")
    ap.add_argument("--dir", type=float, default=0.0, help="cross: direction of travel in the body frame (deg)")
    ap.add_argument("--speed", type=float, default=1.0, help="cross: speed (m/s)")
    ap.add_argument("--floor", default="home floor (field, threshold, rug)", help="game: a floor_wheels.FLOORS name")
    ap.add_argument("--dist", type=float, default=6.5, help="game: thrower distance (m)")
    ap.add_argument("--apex", type=float, default=1.0, help="game: apex above the 1 m release (m)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--model", choices=["rollers", "sphere"], default="rollers", help="wheel model")
    ap.add_argument("--no-skirt", action="store_true", help="v6's decorative bumper (no skirt, no wheel guards)")
    ap.add_argument("-n", "--runs", type=int, default=0, help="stop after this many crossings / tosses")
    ap.add_argument("--cam", choices=["follow", "free"], default="follow")
    ap.add_argument("--speed-factor", type=float, default=1.0, help="0.25 = slow motion")
    ap.add_argument("--record", metavar="FILE.mp4")
    ap.add_argument("-s", "--seconds", type=float, default=20.0, help="video length (simulated s)")
    args = ap.parse_args()
    if args.list:
        print("obstacles:\n  " + "\n  ".join(F.OBSTACLES) + "\nfloors:\n  " + "\n  ".join(F.FLOORS))
        return
    extra = dict(bumper_collides=False) if args.no_skirt else {}
    if args.scene == "flip":
        # the throws of floor_wheels.py C's 7 m / 1 m apex cell (seed 8021); the 3rd toss is missed
        args.dist, args.apex, args.seed = 7.0, 1.0, 8021
        if args.no_skirt:                     # v6's first flip: the roller model on a flat floor, no skirt
            args.floor, args.model = "uniform 0.95", "rollers"
        else:                                 # the threshold hop onto the missed ball (sphere model, no slow zone)
            args.floor, args.model = "home floor (field, threshold, rug)", "sphere"
        args.runs = args.runs or 4
        sim = Game(args, extra)
    elif args.scene == "game":
        sim = Game(args, extra)
    else:
        sim = Crossing(args)
    print(describe(sim.m, sim.p))
    if args.record:
        record(sim, args)
    else:
        viewer(sim, args)


if __name__ == "__main__":
    main()
