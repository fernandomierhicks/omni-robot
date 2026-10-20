"""v3 compact play-catch simulation: 4-wheel X base, tilting catch/launch module, base-mounted cameras.

  python run.py                          catch demo in the interactive viewer
  python run.py --drive                  omni-drive demo (square, turn while moving, spin, circle)
  python run.py --headless -n 30         30 tosses without graphics, prints catch statistics
  python run.py --record catch.mp4 -s 30 render a video (side view + robot close-up + telemetry)
  python run.py --wheel-model rollers    use the explicit-roller omni wheels
  python run.py --layout kiwi3           v2's three-wheel layout on the compact base
  python run.py --no-tilt                funnel fixed upright (shows why the shallow funnel needs the tilt)
"""

from __future__ import annotations

import argparse
import math
import time

import mujoco
import numpy as np

from catch_game import CatchGame, GameConfig
from omni_control import wrap_angle
from params import SimParams
from robot import Robot
from robot_model import build_model, describe


class DriveDemo:
    """Scripted moves that show off the holonomic base (and load the motors and battery)."""

    def __init__(self, model, data, p: SimParams):
        self.m, self.d = model, data
        self.robot = Robot(model, data, p)
        self.steps = [
            ("goto", (0.8, 0.0, 0.0), "forward 0.8 m"),
            ("goto", (0.8, 0.8, 0.0), "strafe left 0.8 m"),
            ("goto", (0.0, 0.0, math.pi), "diagonal back while turning 180 deg"),
            ("spin", 2 * math.pi, "spin in place one turn"),
            ("circle", (0.5, 0.9), "circle r=0.5 m at 0.9 m/s, heading fixed"),
            ("goto", (0.0, 0.0, 0.0), "back to start"),
        ]
        self.i = -1
        self.results = []
        self._advance(0.0, self.robot.pose())

    def _advance(self, t, pose):
        self.i = (self.i + 1) % len(self.steps)
        kind, arg, label = self.steps[self.i]
        self.hold = tuple(pose)
        self.t0 = t + 0.6
        self.turned, self.last_yaw = 0.0, pose[2]
        self.center = np.array([pose[0], pose[1] + (arg[0] if kind == "circle" else 0.0)])
        print(f"[{self.t0:6.2f}s] {label}")

    def step(self):
        r, t = self.robot, self.d.time
        pose = r.pose()
        kind, arg, _ = self.steps[self.i]
        done = False
        if t < self.t0:
            r.drive_to(pose, self.hold)
        elif kind == "goto":
            r.drive_to(pose, arg)
            done = np.hypot(arg[0] - pose[0], arg[1] - pose[1]) < 0.005 and abs(wrap_angle(arg[2] - pose[2])) < 0.01
        elif kind == "spin":
            self.turned += wrap_angle(pose[2] - self.last_yaw)
            self.last_yaw = pose[2]
            rem = arg - self.turned
            w = math.copysign(min(4.0, math.sqrt(2 * 10 * abs(rem)), 8 * abs(rem)), rem)
            r.drive_velocity((0, 0), w, pose[2])
            done = abs(rem) < 0.01
        else:
            rad, v = arg
            w = v / rad
            ang = -math.pi / 2 + w * (t - self.t0)
            ref = self.center + rad * np.array([math.cos(ang), math.sin(ang)])
            r.drive_to(pose, (*ref, self.hold[2]), v_ff=v * np.array([-math.sin(ang), math.cos(ang)]))
            done = w * (t - self.t0) >= 2 * math.pi
        if done:
            self._advance(t, pose)
        r.pre_step()
        mujoco.mj_step(self.m, self.d)
        r.post_step()


def make_sim(args):
    p = SimParams(wheel_model=args.wheel_model, layout=args.layout, tilt=not args.no_tilt,
                  base_max_speed=2.0 if args.moving else 1.4)
    if args.drive:
        p = p.with_(home_xy=(0.0, 0.0))
    model = build_model(p)
    data = mujoco.MjData(model)
    if args.drive:
        adr = model.joint("base").qposadr[0]
        data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    print(describe(model, p))
    cfg = GameConfig(align_tilt=not args.no_tilt, catch_mode="moving" if args.moving else "stop",
                     perception=args.perception, catch_zone_radius=args.zone,
                     apex_range=tuple(args.apex) if args.apex else GameConfig().apex_range)
    behavior = DriveDemo(model, data, p) if args.drive else CatchGame(model, data, p, cfg=cfg, seed=args.seed)
    return p, model, data, behavior


# ---------------------------------------------------------------------------
# overlays: ball trail and predicted landing path drawn into the scene

def add_trail(scene: mujoco.MjvScene, trail, rgba=(1.0, 0.85, 0.15, 0.55), size=0.008):
    for k, pos in enumerate(trail):
        if scene.ngeom >= scene.maxgeom:
            return
        g = scene.geoms[scene.ngeom]
        a = rgba[3] * (k + 1) / len(trail)
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([size, 0, 0]), np.asarray(pos, float),
                            np.eye(3).flatten(), np.array([*rgba[:3], a], np.float32))
        scene.ngeom += 1


def hud_lines(behavior) -> list[str]:
    r = behavior.robot
    tm = r.telemetry()
    lines = [f"t {behavior.d.time:6.1f} s"]
    if isinstance(behavior, CatchGame):
        n = len(behavior.results)
        k = sum(x.caught for x in behavior.results)
        lines.append(f"catches {k}/{n}   state: {behavior.state}")
    lines += [f"battery {tm['v_bus']:5.2f} V  {tm['i_bus']:5.1f} A  {100 * tm['soc']:5.1f}%",
              f"flywheels {tm['fly_rpm']:5.0f} rpm   tilt {tm['tilt_deg']:+5.1f} deg",
              f"drive motor winding {tm['t_motor']:5.1f} C"]
    return lines


def run_viewer(model, data, behavior):
    import mujoco.viewer
    trail = []
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        viewer.cam.lookat[:] = (-0.4, 0.0, 0.7)
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 6.0, 90, -14
        frame = 1 / 60
        while viewer.is_running():
            start = time.perf_counter()
            stop = data.time + frame
            while data.time < stop:
                behavior.step()
            trail.append(data.sensor("ball_pos").data.copy())
            del trail[:-24]
            with viewer.lock():
                viewer.user_scn.ngeom = 0
                add_trail(viewer.user_scn, trail)
                if hasattr(viewer, "set_texts"):
                    try:
                        viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                          "\n".join(hud_lines(behavior)), ""))
                    except Exception:
                        pass
            viewer.sync()
            time.sleep(max(0.0, frame - (time.perf_counter() - start)))


def run_headless(model, data, behavior, tosses: int):
    t0 = time.perf_counter()
    while len(behavior.results) < tosses:
        behavior.step()
    wall = time.perf_counter() - t0
    print(behavior.summary())
    print(f"simulated {data.time:.1f} s in {wall:.1f} s ({data.time / wall:.1f}x real time)")


def record(model, data, behavior, path: str, seconds: float, fps: int = 30, width=1280, height=720):
    import imageio
    from PIL import Image, ImageDraw, ImageFont
    main = mujoco.Renderer(model, height, width)
    inset = mujoco.Renderer(model, 270, 400)
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
    try:
        font = ImageFont.truetype("consola.ttf", 18)
    except OSError:
        font = ImageFont.load_default()
    cam_close = mujoco.MjvCamera()
    writer = imageio.get_writer(path, fps=fps, quality=8, macro_block_size=8)
    trail, next_frame, n = [], 0.0, 0
    while data.time < seconds:
        behavior.step()
        if data.time >= next_frame:
            next_frame += 1 / fps
            trail.append(data.sensor("ball_pos").data.copy())
            del trail[:-12]
            main.update_scene(data, camera="side", scene_option=opt)
            add_trail(main.scene, trail)
            img = Image.fromarray(main.render())
            kiwi = data.xpos[model.body("base").id]
            cam_close.lookat[:] = kiwi + [0, 0, 0.28]
            cam_close.distance, cam_close.azimuth, cam_close.elevation = 1.25, 125, -18
            inset.update_scene(data, camera=cam_close, scene_option=opt)
            add_trail(inset.scene, trail)
            small = Image.fromarray(inset.render())
            img.paste(small, (width - 410, 10))
            draw = ImageDraw.Draw(img, "RGBA")
            draw.rectangle((width - 411, 9, width - 9, 281), outline=(255, 255, 255, 180), width=2)
            lines = hud_lines(behavior)
            draw.rectangle((10, 10, 430, 18 + 24 * len(lines)), fill=(0, 0, 0, 120))
            for k, line in enumerate(lines):
                draw.text((20, 16 + 24 * k), line, fill=(255, 255, 255, 255), font=font)
            writer.append_data(np.asarray(img))
            n += 1
    writer.close()
    print(f"wrote {path} ({n} frames)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drive", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("-n", "--tosses", type=int, default=20)
    ap.add_argument("--record", metavar="FILE.mp4")
    ap.add_argument("-s", "--seconds", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--wheel-model", default="sphere", choices=["sphere", "rollers"])
    ap.add_argument("--layout", default="x4", choices=["x4", "kiwi3"])
    ap.add_argument("--no-tilt", action="store_true")
    ap.add_argument("--moving", action="store_true", help="moving catches (2 m/s cap, clear-path gate)")
    ap.add_argument("--perception", default="truth", choices=["truth", "model"],
                    help="model: stereo cameras + tracking filter instead of the exact ball state after 0.25 s")
    ap.add_argument("--zone", type=float, default=0.37, help="tosses land within this distance of home (m)")
    ap.add_argument("--apex", type=float, nargs=2, metavar=("LO", "HI"), help="apex range above the release (m)")
    args = ap.parse_args()
    p, model, data, behavior = make_sim(args)
    if args.record:
        record(model, data, behavior, args.record, args.seconds)
    elif args.headless:
        run_headless(model, data, behavior, args.tosses)
    else:
        run_viewer(model, data, behavior)


if __name__ == "__main__":
    main()
