"""Showcase video of the v5 design: the robot, good catches, bad catches, hopper shooting.

The catch clips replay real tosses from studies/catch_sweep.py ("physics, moving" robot,
vision model): the same cell, seed and toss number, fast-forwarded to that toss and
recorded, each followed by a 4x slow-motion close-up of the ball's arrival.

  python render_videos.py                 -> videos/v5_showcase.mp4 (and one mp4 per chapter)
"""

from __future__ import annotations

import itertools
import math
import sys
from pathlib import Path

import imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "studies"))

from catch_game import CatchGame, GameConfig  # noqa: E402
from params import SimParams  # noqa: E402
from robot_model import ball_names, build_model  # noqa: E402

OUT = HERE / "videos"
W, H, FPS = 1280, 720, 30
HOME = (1.0, 0.0)
DISTANCES, APEXES, OFFSETS = (1.5, 2.25, 3.0), (0.5, 1.0, 2.0, 3.0), (0.3, 0.6, 1.0)   # as studies/catch_sweep.py
CELLS = list(itertools.product(DISTANCES, APEXES, OFFSETS))


def font(size):
    for name in ("segoeui.ttf", "arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


F_BIG, F_MED, F_SMALL, F_MONO = font(52), font(30), font(22), None
try:
    F_MONO = ImageFont.truetype("consola.ttf", 18)
except OSError:
    F_MONO = font(18)


class Video:
    def __init__(self, path: Path):
        self.w = imageio.get_writer(str(path), fps=FPS, quality=8, macro_block_size=8)
        self.n = 0

    def add(self, img: Image.Image):
        self.w.append_data(np.asarray(img.convert("RGB")))
        self.n += 1

    def title(self, text: str, sub: str = "", seconds: float = 2.5, color=(255, 255, 255)):
        img = Image.new("RGB", (W, H), (18, 20, 26))
        dr = ImageDraw.Draw(img)
        dr.text((W // 2, H // 2 - 30), text, font=F_BIG, fill=color, anchor="mm")
        for k, line in enumerate(sub.split("\n") if sub else []):
            dr.text((W // 2, H // 2 + 40 + 36 * k), line, font=F_MED, fill=(190, 195, 205), anchor="mm")
        for _ in range(int(seconds * FPS)):
            self.add(img)

    def close(self):
        self.w.close()


def scene_opt():
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
    return opt


def add_sphere(scene, pos, rgba, size=0.008):
    if scene.ngeom >= scene.maxgeom:
        return
    mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([size, 0, 0]),
                        np.asarray(pos, float), np.eye(3).flatten(), np.array(rgba, np.float32))
    scene.ngeom += 1


def add_disc(scene, pos, radius, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_CYLINDER, np.array([radius, 0.006, 0.0]),
                        np.asarray(pos, float), np.eye(3).flatten(), np.array(rgba, np.float32))
    scene.ngeom += 1


def aim(cam, pos, lookat):
    """Point a free camera from pos at lookat (kept inside the netting, unlike an orbit distance)."""
    v = np.asarray(lookat, float) - np.asarray(pos, float)
    n = float(np.linalg.norm(v))
    cam.lookat[:] = lookat
    cam.distance = n
    cam.azimuth = math.degrees(math.atan2(v[1], v[0]))
    cam.elevation = math.degrees(math.asin(v[2] / n))


def close_view(cam, base, thrower_xy, dist=1.3, elev=-14):
    """Close-up of the robot from the arena-center side, 40 deg around, so the camera stays inside the netting
    (thrower_xy only breaks the tie when the robot is at the center)."""
    cx, cy = (0.0, 0.0) if math.hypot(base[0], base[1]) > 0.3 else thrower_xy
    az = math.degrees(math.atan2(base[1] - cy, base[0] - cx)) + 40
    cam.lookat[:] = base + np.array([0, 0, 0.32])
    cam.distance, cam.azimuth, cam.elevation = dist, az, elev


def trails(scene, tr):
    for path in tr:
        for k, pos in enumerate(path):
            add_sphere(scene, pos, (1.0, 0.85, 0.15, 0.6 * (k + 1) / len(path)))


def banner(img, text, color, sub=""):
    dr = ImageDraw.Draw(img, "RGBA")
    dr.rectangle((0, H - 74, W, H), fill=(0, 0, 0, 150))
    dr.text((24, H - 66), text, font=F_MED, fill=color)
    if sub:
        dr.text((24, H - 30), sub, font=F_SMALL, fill=(220, 220, 225))


def hud(img, lines):
    dr = ImageDraw.Draw(img, "RGBA")
    dr.rectangle((10, 10, 420, 18 + 22 * len(lines)), fill=(0, 0, 0, 120))
    for k, line in enumerate(lines):
        dr.text((20, 15 + 22 * k), line, font=F_MONO, fill=(255, 255, 255))


def inset(img, small):
    img.paste(small, (W - 410, 10))
    ImageDraw.Draw(img).rectangle((W - 411, 9, W - 9, 281), outline=(255, 255, 255), width=2)


def robot_hud(d, r, extra=()):
    tm = r.telemetry()
    v = float(np.hypot(*d.sensor("base_linvel").data[:2]))
    return [f"t {d.time:6.2f} s   base {v:4.2f} m/s", f"tilt {tm['tilt_deg']:+5.1f} deg   flywheels {tm['fly_rpm']:5.0f} rpm",
            f"battery {tm['v_bus']:5.2f} V {tm['i_bus']:5.1f} A", *extra]


# ---------------------------------------------------------------------------
# 1. design

def design_chapter(vid: Video):
    import hopper_rig
    vid.title("v5: the compact robot, no firmware limits", "4-wheel X base, deck in the axle plane, tilting catch/launch module\n"
              "plunger feed, stereo cameras on the payload plate")
    for p, n, label in ((SimParams(n_balls=6), 0, "catch module: 55 deg net funnel, pocket, plunger, flywheels"),
                        (SimParams(n_balls=6, hopper="tube"), 6, "hopper mode: magazine tube")):
        m, d, r, bodies, *_ = hopper_rig.load(p, n, settle=0.5)
        rnd = mujoco.Renderer(m, H, W)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0.0, 0.0, 0.28 if n else 0.22]
        cam.distance, cam.elevation = (1.6 if n else 1.35), -14
        opt = scene_opt()
        seq = [(0.0, "upright"), (15.0, "+15 deg: tilted into an incoming ball"), (-30.0, "-30 deg: 60 deg launch"),
               (0.0, "0 deg: 30 deg launch")]
        T = 9.0
        t0 = d.time
        nxt = t0
        while d.time - t0 < T:
            s = d.time - t0
            k = min(len(seq) - 1, int(s / (T / len(seq))))
            r.tilt_target = math.radians(seq[k][0])
            r.step()
            if d.time >= nxt:
                nxt += 1 / FPS
                cam.azimuth = -70 + 360 * s / T
                rnd.update_scene(d, camera=cam, scene_option=opt)
                img = Image.fromarray(rnd.render())
                aboard = sum(hopper_rig._aboard(d, r, p, bodies)) if n else 0
                banner(img, label + (f" ({aboard} balls aboard)" if n else ""), (255, 255, 255), f"tilt {seq[k][1]}")
                vid.add(img)
        rnd.close()
    drive_clip(vid)


def drive_clip(vid: Video):
    """Physics-limit moves: a 4 m dash along the X's fast axis and back, a strafe, a diagonal while turning, a spin."""
    from omni_control import wrap_angle
    from robot import Robot
    p = SimParams(home_xy=(-1.3, -1.35), thrower_xy=(-2.1, 1.7))     # thrower out of the way, in a corner
    m = build_model(p)
    d = mujoco.MjData(m)
    adr = m.joint("base").qposadr[0]
    d.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    m.vis.global_.fovy = 60
    r = Robot(m, d, p)
    steps = [((2.0, -1.35, 0.0), "3.3 m dash forward (the X drive's fast axis)"),
             ((2.0, 1.2, 0.0), "strafe left 2.5 m"),
             ((0.0, -0.3, math.pi), "diagonal back while turning 180 deg"),
             ((0.0, -0.3, math.pi), "spin in place, two turns")]
    rnd = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    opt = scene_opt()
    nxt, vmax, k, t_hold, turned, last_yaw = 0.0, 0.0, 0, 0.6, 0.0, 0.0
    while k < len(steps) and d.time < 20.0:
        pose = r.pose()
        tgt, label = steps[k]
        if d.time < t_hold:
            r.drive_to(pose, pose if k == 0 else steps[k - 1][0])
        elif label.startswith("spin"):
            turned += wrap_angle(pose[2] - last_yaw)
            rem = 4 * math.pi - turned
            r.drive_velocity((0.0, 0.0), math.copysign(min(16.0, math.sqrt(2 * 80 * abs(rem))), rem), pose[2])
            if abs(rem) < 0.02:
                k, t_hold = k + 1, d.time + 0.6
        else:
            r.drive_to(pose, tgt)
            if np.hypot(tgt[0] - pose[0], tgt[1] - pose[1]) < 0.01 and abs(wrap_angle(tgt[2] - pose[2])) < 0.02:
                k, t_hold = k + 1, d.time + 0.5
        last_yaw = pose[2]
        r.step()
        vmax = max(vmax, float(np.hypot(*d.sensor("base_linvel").data[:2])))
        if d.time >= nxt:
            nxt += 1 / FPS
            b = d.xpos[m.body("base").id]
            aim(cam, (0.4 * b[0], -1.9, 2.1), b + [0, 0, 0.2])
            rnd.update_scene(d, camera=cam, scene_option=opt)
            img = Image.fromarray(rnd.render())
            yr = abs(float(d.sensor("base_angvel").data[2]))
            hud(img, robot_hud(d, r, (f"yaw rate {yr:4.1f} rad/s", f"peak speed so far {vmax:4.2f} m/s")))
            banner(img, f"Omni drive at the physics limits: {steps[min(k, len(steps) - 1)][1]}", (255, 255, 255),
                   "no speed cap (back-EMF only), full traction table, no jerk limit; v3b capped this at 1.4 m/s")
            vid.add(img)
    rnd.close()


# ---------------------------------------------------------------------------
# 2-3. catches

def catch_clip(vid: Video, cell, toss: int, title: str, why: str, return_throws=False, seed=None):
    dist, apex, off = cell
    p = SimParams(home_xy=HOME, thrower_xy=(HOME[0] - dist - 0.3, 0.0))
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = GameConfig(catch_zone_radius=off, zone_mode="ring", apex_range=(apex, apex), return_throws=return_throws,
                     wait_between=0.8, catch_mode="moving", perception="vision")
    game = CatchGame(m, d, p, cfg=cfg, seed=CELLS.index(cell) if seed is None else seed, verbose=False)
    while not (len(game.results) == toss and game.state == "wait" and d.time > game.t_next - 0.5):
        game.step()
    main, small, close = mujoco.Renderer(m, H, W), mujoco.Renderer(m, 270, 400), mujoco.Renderer(m, H, W)
    cam_c = mujoco.MjvCamera()
    m.vis.global_.fovy = 62
    cam_m = mujoco.MjvCamera()                 # side view from inside the netting: thrower left, robot right
    aim(cam_m, ((p.thrower_xy[0] + HOME[0]) / 2 + 0.2, -1.85, 1.25), ((p.thrower_xy[0] + HOME[0]) / 2 + 0.2, 0.6, 0.75))
    opt = scene_opt()
    tr, nxt, nxt_c = [], d.time, d.time
    slow = []                                 # (t, image) close-ups at 120 Hz for the replay
    t_end = None
    vmax = 0.0
    while True:
        game.step()
        t = d.time
        if game.state in ("flight", "settle"):
            vmax = max(vmax, float(np.hypot(*d.sensor("base_linvel").data[:2])))
        done = len(game.results) > toss
        if done and t_end is None:
            res = game.results[toss]
            t_end = t + (0.6 if not return_throws else 4.2)
        if t_end is not None and t >= t_end:
            break
        base = d.xpos[m.body("base").id]
        arrived = game.state in ("flight", "settle") and game.cross_err is not None
        if t >= nxt_c and game.state in ("flight", "settle") and not done and not (arrived and t > game.t_in + 0.8):
            nxt_c += 1 / 120
            close_view(cam_c, base, p.thrower_xy, 1.3, -14)
            close.update_scene(d, camera=cam_c, scene_option=opt)
            trails(close.scene, [tr[-12:]] if tr else [])
            slow.append((t, Image.fromarray(close.render())))
            # keep 0.5 s before the arrival (or the last 0.5 s until it happens)
            t_keep = game.t_in - 0.5 if arrived else t - 0.5
            slow = [s for s in slow if s[0] >= t_keep]
        if t >= nxt:
            nxt += 1 / FPS
            tr.append(d.sensor("ball_pos").data.copy())
            del tr[:-14]
            main.update_scene(d, camera=cam_m, scene_option=opt)
            trails(main.scene, [tr])
            img = Image.fromarray(main.render())
            close_view(cam_c, base, p.thrower_xy, 1.25, -18)
            small.update_scene(d, camera=cam_c, scene_option=opt)
            trails(small.scene, [tr])
            inset(img, Image.fromarray(small.render()))
            hud(img, robot_hud(d, game.robot, (f"state {game.state}",)))
            if done:
                ok = game.results[toss].caught
                banner(img, ("CAUGHT  " if ok else "MISSED  ") + title, (120, 230, 120) if ok else (255, 110, 100), why)
            else:
                banner(img, title, (255, 255, 255), f"thrower {dist} m away, apex {apex} m, landing {off} m from the waiting spot")
            vid.add(img)
    res = game.results[toss]
    # 4x slow-motion close-up of the arrival
    t_arr = game.t_in if res.pos_error == res.pos_error else slow[-1][0] - 0.4
    clip = [s for s in slow if t_arr - 0.45 <= s[0] <= t_arr + 0.75]
    for t, img in clip:
        img = img.copy()
        dr = ImageDraw.Draw(img, "RGBA")
        dr.rectangle((0, 0, W, 50), fill=(0, 0, 0, 130))
        dr.text((20, 10), f"4x slow motion: ball arrives {res.arrival_speed:.1f} m/s, {res.arrival_deg:.0f} deg from vertical; "
                f"base at {res.catch_speed:.1f} m/s; funnel tilt {res.tilt_deg:.0f} deg", font=F_SMALL, fill=(255, 255, 255))
        ok = res.caught
        banner(img, ("CAUGHT" if ok else "MISSED") + f": funnel center {100 * res.pos_error:.1f} cm from the ball"
               if res.pos_error == res.pos_error else "MISSED", (120, 230, 120) if ok else (255, 110, 100), why)
        vid.add(img)
    for rr in (main, small, close):
        rr.close()
    return res


def good_chapter(vid: Video):
    vid.title("Good catches", "no firmware limits, moving catches, detailed vision model", color=(140, 235, 140))
    catch_clip(vid, (2.25, 2.0, 0.3), 0, "Lob, catch, and the return throw",
               "stop under the ball, tilt into it, dump, tilt back to 50 deg, throw back to the hands", return_throws=True)
    catch_clip(vid, (2.25, 0.5, 1.0), 4, "Line drive landing 1 m away, caught on the move",
               "too fast to stop under: the robot meets it at ~3 m/s (v3b's 1.4 m/s cap never got there)")
    catch_clip(vid, (3.0, 0.5, 1.0), 0, "Another 1 m line drive, moving catch",
               "braking curve to the slowest on-time speed, then braking with the ball aboard")


def bad_chapter(vid: Video):
    vid.title("Bad catches", "where the remaining misses come from", color=(255, 130, 120))
    catch_clip(vid, (2.25, 0.5, 1.0), 0, "Line drive landing 1 m away: can't get there",
               "0.8 s of flight, ~0.6 s after the first estimate: 1 m is beyond reach even at the traction limit")
    catch_clip(vid, (2.25, 3.0, 0.6), 1, "3 m lob, landed 3 cm from center: bounced out",
               "arrives at ~7.5 m/s nearly straight down; the padded pocket doesn't hold it (catch module, not drive)")
    catch_clip(vid, (2.25, 1.0, 1.0), 1, "Moving catch that bounced out",
               "the ball enters at an angle while the base moves at 1 m/s and climbs back out of the shallow cone")


# ---------------------------------------------------------------------------
# 4. hopper

def hopper_clip(vid: Video, mode: str, title: str, sub: str, seconds: float = 16.0, **ckw):
    import shoot_game as sg
    p = SimParams(hopper="tube", n_balls=5, home_xy=(0.0, 0.0), area_size=(10.0, 8.0), thrower_xy=(-6.0, 0.0))
    m = build_model(p)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    cfg = sg.ShootConfig(mode=mode, **ckw)
    game = sg.ShootGame(m, d, p, cfg, verbose=False)
    sg.load_tube(m, d, game.r, p, 6)
    game.t_state = d.time
    main, small = mujoco.Renderer(m, H, W), mujoco.Renderer(m, 270, 400)
    m.vis.global_.fovy = 60
    cam = mujoco.MjvCamera()
    aim(cam, (0.0, -3.0, 8.0), (0.0, 0.3, 0.0))     # high, inside the gym: the whole floor and every target
    cam_c = mujoco.MjvCamera()
    opt = scene_opt()
    bodies = [m.body(n).id for n in ball_names(p)]
    trs = {i: [] for i in range(len(bodies))}
    t0, nxt = d.time, d.time
    t_done = None
    while d.time - t0 < seconds:
        game.step()
        if game.done() and t_done is None:
            t_done = d.time
        if t_done is not None and d.time - t_done > 1.0:
            break
        if d.time >= nxt:
            nxt += 1 / FPS
            for i, b in enumerate(bodies):
                v = np.linalg.norm(d.cvel[b][3:])
                if v > 1.0:
                    trs[i].append(d.xpos[b].copy())
                    del trs[i][:-20]
            main.update_scene(d, camera=cam, scene_option=opt)
            for k, tg in enumerate(cfg.targets):
                hit = next((s for s in game.shots if s.target == tg and s.miss == s.miss), None)
                col = (0.2, 0.9, 0.3, 0.7) if hit is not None and hit.miss < 0.3 else \
                      ((1.0, 0.3, 0.2, 0.7) if hit is not None else (1.0, 0.55, 0.1, 0.7))
                add_disc(main.scene, (tg[0], tg[1], tg[2]), 0.3, col)
                add_sphere(main.scene, (tg[0], tg[1], tg[2] + 0.01), (1, 1, 1, 0.9), 0.03)
            trails(main.scene, [t for t in trs.values() if t])
            img = Image.fromarray(main.render())
            close_view(cam_c, d.xpos[m.body("base").id], (0.0, -6.0), 1.5, -15)
            small.update_scene(d, camera=cam_c, scene_option=opt)
            inset(img, Image.fromarray(small.render()))
            scored = [s for s in game.shots if s.miss == s.miss]
            last = (f"last shot: {s.distance:.1f} m target, missed by {100 * s.miss:.0f} cm"
                    if (s := (scored[-1] if scored else None)) else "")
            hud(img, robot_hud(d, game.r, (f"shots {len(game.shots)}/6, target {min(game.k + 1, 6)}",
                                           f"balls in the pocket: {'yes' if game.r.ball_held() else 'no'}")))
            banner(img, title, (255, 255, 255), last or sub)
            vid.add(img)
    main.close()
    small.close()
    return game


def hopper_chapter(vid: Video):
    vid.title("Hopper mode", "6 balls in the magazine tube, 6 targets 4-5 m away\n"
              "discs: orange = not yet, green = within 30 cm, red = missed")
    g1 = hopper_clip(vid, "stop", "Stop, aim, shoot: one ball per target", "the base spins to each target; flywheels braked while it turns")
    g2 = hopper_clip(vid, "move", "Shooting on the move (1.5 m/s loop)",
                     "launch solved with the base's own velocity; turns and braking can roll the waiting ball into the wheels")
    return g1, g2


def main():
    OUT.mkdir(exist_ok=True)
    chapters = [("1_design", design_chapter), ("2_good_catches", good_chapter), ("3_bad_catches", bad_chapter),
                ("4_hopper", hopper_chapter)]
    only = sys.argv[1:]
    for name, fn in chapters:
        if only and name not in only:
            continue
        vid = Video(OUT / f"v5_{name}.mp4")
        fn(vid)
        vid.close()
        print(f"wrote {OUT / f'v5_{name}.mp4'} ({vid.n / FPS:.0f} s)", flush=True)
    # one combined file
    if not only:
        out = imageio.get_writer(str(OUT / "v5_showcase.mp4"), fps=FPS, quality=8, macro_block_size=8)
        for name, _ in chapters:
            for frame in imageio.get_reader(str(OUT / f"v5_{name}.mp4")):
                out.append_data(frame)
        out.close()
        print(f"wrote {OUT / 'v5_showcase.mp4'}")


if __name__ == "__main__":
    main()
