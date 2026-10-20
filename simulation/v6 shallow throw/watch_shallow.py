"""Watch, record or score the 6-7 m shallow throws (the scenario of studies/shallow_sweep.py) in 3D.

  python watch_shallow.py                              interactive viewer: thrower 6.5 m away, apex 1-1.5 m, seed 0
  python watch_shallow.py --dist 7 --apex 0.75         a 7 m throw with a 0.75 m apex (one value = fixed apex)
  python watch_shallow.py --seed 3 --tosses 6          another set of throws; stop after 6 tosses
  python watch_shallow.py --cell 6.5 1.0 0             replay a shallow_sweep.py cell exactly (distance, apex, seed index 0-2)
  python watch_shallow.py --robot "v5 robot, firmware adult"   the v5 robot on the same throws, to compare
  python watch_shallow.py --speed 0.25                 slow motion (viewer only)
  python watch_shallow.py --cam follow                 camera follows the robot (drag with the mouse to move it anyway)
  python watch_shallow.py --perception truth           exact ball state after 0.12 s: smoother in the viewer, no camera model
  python watch_shallow.py --no-return                  catch only, no return throw
  python watch_shallow.py --headless --tosses 12       no graphics: prints each toss and a summary
  python watch_shallow.py --record shots.mp4 -s 40     video (wide view + robot close-up + telemetry)
  python watch_shallow.py --ceiling 2.44 --apex 1.0    indoors: an 8 ft ceiling (balls bounce off it); the return
                                                       throw flattens to clear it (add --fixed-return to see v5's lob hit it)
  python watch_shallow.py --list                       the robots you can pass to --robot
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "studies"))

import run as R  # noqa: E402
import shallow_sweep as S  # noqa: E402
from catch_game import GameConfig  # noqa: E402
from params import SimParams  # noqa: E402
from robot_model import build_model, describe  # noqa: E402

DEFAULT_ROBOT = "v6 chute + track start, firmware adult"


def make_game(args):
    pkw, gkw = S.ROBOTS[args.robot]
    p = SimParams(home_xy=S.HOME, thrower_xy=(S.HOME[0] - args.dist - 0.3, 0.0), area_size=S.AREA,
                  ceiling_height=args.ceiling, **pkw)
    model = build_model(p)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lo, hi = (args.apex[0], args.apex[-1])
    cfg = dict(catch_zone_radius=0.0, zone_mode="ring", apex_range=(lo, hi), return_throws=not args.no_return,
               wait_between=args.wait, perception="vision", vision_cfg=S.EXPECTED, toss_spin=S.SPIN)
    cfg = {**cfg, **gkw}
    if args.ceiling and not args.fixed_return:
        cfg["ceiling"] = args.ceiling
    if args.ceiling and 1.0 + args.apex[-1] + p.ball_spec.radius > args.ceiling:
        print(f"note: a {args.apex[-1]} m apex above the 1 m release reaches the {args.ceiling} m ceiling")
    if args.perception:
        cfg["perception"] = args.perception
        if args.perception == "truth":
            cfg.setdefault("perception_delay", 0.12)
    game = S.HumanToss(model, data, p, cfg=GameConfig(**cfg), seed=args.seed, verbose=True, dist=args.dist)
    return p, model, data, game


def set_camera(cam, game, mode):
    if mode == "follow":
        cam.lookat[:] = game.d.xpos[game.m.body("base").id] + np.array([0.0, 0.0, 0.3])
    # "wide": fixed, set once


def init_camera(cam, mode):
    cam.lookat[:] = (-0.1, 0.0, 0.9)
    cam.distance, cam.azimuth, cam.elevation = 11.5, 90, -12
    if mode == "follow":
        cam.distance, cam.azimuth, cam.elevation = 2.4, 125, -16


def viewer(model, data, game, args):
    import mujoco.viewer
    trail = []
    with mujoco.viewer.launch_passive(model, data) as v:
        v.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        init_camera(v.cam, args.cam)
        frame = 1 / 60
        while v.is_running():
            t0 = time.perf_counter()
            stop = data.time + frame * args.speed
            while data.time < stop:
                game.step()
            trail.append(data.sensor("ball_pos").data.copy())
            del trail[:-45]
            set_camera(v.cam, game, args.cam)
            with v.lock():
                v.user_scn.ngeom = 0
                R.add_trail(v.user_scn, trail)
                try:
                    v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                 "\n".join(R.hud_lines(game)), ""))
                except Exception:
                    pass
            v.sync()
            if args.tosses and len(game.results) >= args.tosses and game.state == "wait":
                break
            time.sleep(max(0.0, frame - (time.perf_counter() - t0)))


def record(model, data, game, args, fps=30, width=1280, height=720):
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
    cam_wide, cam_close = mujoco.MjvCamera(), mujoco.MjvCamera()
    init_camera(cam_wide, "wide")
    writer = imageio.get_writer(args.record, fps=fps, quality=8, macro_block_size=8)
    trail, next_frame, n = [], 0.0, 0
    while data.time < args.seconds and not (args.tosses and len(game.results) >= args.tosses and game.state == "wait"):
        game.step()
        if data.time >= next_frame:
            next_frame += 1 / fps
            trail.append(data.sensor("ball_pos").data.copy())
            del trail[:-20]
            main.update_scene(data, camera=cam_wide, scene_option=opt)
            R.add_trail(main.scene, trail)
            img = Image.fromarray(main.render())
            cam_close.lookat[:] = data.xpos[model.body("base").id] + [0, 0, 0.28]
            cam_close.distance, cam_close.azimuth, cam_close.elevation = 1.4, 125, -18
            inset.update_scene(data, camera=cam_close, scene_option=opt)
            R.add_trail(inset.scene, trail)
            img.paste(Image.fromarray(inset.render()), (width - 410, 10))
            draw = ImageDraw.Draw(img, "RGBA")
            draw.rectangle((width - 411, 9, width - 9, 281), outline=(255, 255, 255, 180), width=2)
            lines = R.hud_lines(game)
            draw.rectangle((10, 10, 430, 18 + 24 * len(lines)), fill=(0, 0, 0, 120))
            for k, line in enumerate(lines):
                draw.text((20, 16 + 24 * k), line, fill=(255, 255, 255, 255), font=font)
            writer.append_data(np.asarray(img))
            n += 1
    writer.close()
    print(f"wrote {args.record} ({n} frames, {data.time:.1f} s simulated)")


def headless(data, game, args):
    t0 = time.perf_counter()
    while len(game.results) < (args.tosses or 6) or game.state != "wait":
        game.step()
        if data.time > 400:
            break
    print(game.summary())
    print(f"simulated {data.time:.1f} s in {time.perf_counter() - t0:.1f} s")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default=DEFAULT_ROBOT, choices=list(S.ROBOTS), metavar="NAME",
                    help="a key of shallow_sweep.ROBOTS (see --list)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--dist", type=float, default=6.5, help="thrower distance from the waiting spot (m)")
    ap.add_argument("--apex", type=float, nargs="+", default=[1.0, 1.5], metavar="M",
                    help="apex above the 1 m release: one value, or LO HI for a random apex per toss")
    ap.add_argument("--seed", type=int, default=0, help="seeds the landing points, apexes, spin and camera noise")
    ap.add_argument("--cell", type=float, nargs=3, metavar=("DIST", "APEX", "S"),
                    help="replay shallow_sweep.py's cell: distance, apex, seed index 0-2 (sets --dist --apex --seed, "
                         "no return throws, 1.2 s between tosses, 6 tosses; the same throws and outcomes as the sweep)")
    ap.add_argument("-n", "--tosses", type=int, default=0, help="stop after this many tosses (0 = keep going)")
    ap.add_argument("--no-return", action="store_true", help="catch only, no return throw")
    ap.add_argument("--wait", type=float, default=2.0, help="seconds between tosses")
    ap.add_argument("--perception", choices=["vision", "truth", "model"],
                    help="override the robot's perception (truth = exact ball state after 0.12 s)")
    ap.add_argument("--ceiling", type=float, help="ceiling height (m), e.g. 2.44 for 8 ft; return throws clear it")
    ap.add_argument("--fixed-return", action="store_true", help="with --ceiling: keep the 50 deg return lob")
    ap.add_argument("--cam", choices=["wide", "follow"], default="wide")
    ap.add_argument("--speed", type=float, default=1.0, help="viewer speed factor (0.25 = slow motion)")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--record", metavar="FILE.mp4")
    ap.add_argument("-s", "--seconds", type=float, default=30.0, help="video length")
    args = ap.parse_args()
    if args.list:
        print("\n".join(S.ROBOTS))
        return
    if args.cell:
        d_, a_, s_ = args.cell
        args.dist, args.apex, args.no_return, args.wait = d_, [a_], True, 1.2
        args.seed = 100 * int(s_) + 10 * S.DISTANCES.index(d_) + S.APEXES.index(a_)
        args.tosses = args.tosses or S.TOSSES
    p, model, data, game = make_game(args)
    print(describe(model, p))
    print(f"robot: {args.robot}; thrower {args.dist} m away; apex {args.apex}; seed {args.seed}")
    if args.record:
        record(model, data, game, args)
    elif args.headless:
        headless(data, game, args)
    else:
        viewer(model, data, game, args)


if __name__ == "__main__":
    main()
