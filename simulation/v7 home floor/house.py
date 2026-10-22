"""A one-storey house floor for the v7 robot (design.md 25), and a scene you drive with the keyboard or a gamepad.

The floor: wood living room and hallway, a raised tile kitchen behind reducers, wall-to-wall carpet in the bedroom
behind carpet trim, a marble threshold into the bathroom, a T-molding into the office, rugs with square edges
(12 mm living-room rug turned 15 deg, an 8 mm runner, a 6 mm bath mat) and a 16 mm plush rug. Heights are typical of
the parts sold for each (est). Friction differs by floor (wood 0.95, laminate 0.9, tile 0.85, est), with a light
random field on top. Partition walls (half height, so the camera sees over them) have doorways.

  python house.py                       drive the proposed v7 robot (--base v6 for the v6 base, --list for the bases)
  python house.py --base v6 --no-assist the v6 base, no terrain-aware firmware

Keys (the window must have focus):
  W / S, A / D     forward / back, strafe left / right (body frame; G switches to world frame)
  Q / E or <- / -> turn left / right            SHIFT   2 m/s (adult cap); otherwise 1 m/s
  SPACE            stop (ramped)                T       terrain assist on / off (speed caps + square-edge alignment)
  F                fire the ball                L       load a ball into the pocket
  R                back to the start            C       camera: follow / chase / top
  1-9              teleport to a spot (1 living-room rug, 2 kitchen reducer, 3 bedroom carpet, 4 bath threshold,
                   5 office T-molding, 6 plush rug, 7 runner, 8 bath mat, 9 hallway)
  mouse drag / wheel  orbit / zoom             ESC     quit
Gamepad (XInput layout): left stick translate, right stick turn, RT = 2 m/s, A fire, B load, X assist, Y camera,
START back to the start. Flight stick (e.g. Logitech Extreme 3D): stick translates, twist turns, the throttle slider
sets the top speed (1-2 m/s); trigger fire, thumb load, buttons 3/4 assist/camera, 8 back to the start.
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

from floor import Carpet, Threshold, Wall  # noqa: E402
from omni_control import wrap_angle  # noqa: E402
from params import SimParams  # noqa: E402

TILE = (0.86, 0.86, 0.83, 1)
CARPET = (0.78, 0.7, 0.58, 1)

# x: -6 .. 6, y: -4.5 .. 4.5. Living room (x < 0) and kitchen (x > 0) at the bottom, hallway across the middle
# (y 0.5 .. 2), bedroom, office and bathroom along the top.
HOUSE_FEATURES = (
    # kitchen: tile on a 6 mm underlayment, reducer strips (40 mm ramps) to the wood
    Carpet(0.0, -4.45, 5.95, 0.5, thickness=0.006, bevel=0.04, bevel_y=0.04, mu=0.85, crr=0.015, softness=0.003,
           rgba=TILE),
    # bedroom: wall-to-wall carpet, 12 mm, carpet trim (25 mm bevel) at the door
    Carpet(-5.95, 2.05, -1.05, 4.45, thickness=0.012, bevel=0.025, bevel_y=0.025, mu=0.85, crr=0.06, rgba=CARPET),
    # living room: a 12 mm rug, square edges, turned 15 deg; an 8 mm runner, turned -30 deg; a 16 mm plush rug
    Carpet(-4.6, -3.4, -1.6, -1.4, thickness=0.012, yaw_deg=15.0, rgba=(0.55, 0.16, 0.14, 1)),
    Carpet(-1.2, -1.2, -0.4, 0.4, thickness=0.008, yaw_deg=-30.0, rgba=(0.15, 0.3, 0.5, 1)),
    Carpet(-5.6, -0.6, -4.4, 0.4, thickness=0.016, crr=0.08, softness=0.015, rgba=(0.9, 0.9, 0.88, 1)),
    # doorways: marble threshold into the bathroom, T-molding into the office
    Threshold(3.45, 2.0, yaw_deg=90.0, length=0.9, height=0.012, top=0.03, ramp=0.02),
    Threshold(0.45, 2.0, yaw_deg=90.0, length=0.9, height=0.005, top=0.012, ramp=0.012),
    # bathroom: 6 mm bath mat
    Carpet(4.2, 3.0, 5.0, 3.5, thickness=0.006, crr=0.05, rgba=(0.35, 0.6, 0.55, 1)),
    # partition walls along y = 2 with doors (bedroom -3.45..-2.55, office 0..0.9, bath 3..3.9), and between rooms
    Wall(-6.0, 2.0, -3.45, 2.0), Wall(-2.55, 2.0, 0.0, 2.0), Wall(0.9, 2.0, 3.0, 2.0), Wall(3.9, 2.0, 6.0, 2.0),
    Wall(-1.0, 2.0, -1.0, 4.5), Wall(2.0, 2.0, 2.0, 4.5),
)
HOUSE_PATCHES = (
    (-6.0, 0.5, 6.0, 2.0, 0.9),          # hallway laminate
    (2.0, 2.0, 6.0, 4.5, 0.85),          # bathroom tile (at floor level)
)
SPOTS = {   # teleport spots: (x, y, yaw deg) just before an edge
    "1": (-3.1, -0.6, -90.0), "2": (-0.7, -2.0, 0.0), "3": (-3.0, 1.2, 90.0), "4": (3.45, 1.2, 90.0),
    "5": (0.45, 1.2, 90.0), "6": (-5.0, -1.4, 90.0), "7": (-0.8, -2.0, 90.0), "8": (4.6, 2.6, 90.0),
    "9": (-5.0, 1.25, 0.0),
}
START = (-3.0, 1.25, 0.0)

# bases (design.md 25): v6 as built in round 3, and the v7 proposal
BASES = {
    "v6": dict(),
    "v7": dict(wheel="Printed-127-2x5-36", suspension="pod"),
}


def house_params(base: str = "v7", assist: bool = True, wheel_model: str = "rollers", **kw) -> SimParams:
    return SimParams(**{**BASES[base], **kw}, wheel_model=wheel_model, area_size=(12.2, 9.2), home_xy=START[:2],
                     thrower_xy=(START[0] - 3.0, START[1]), floor_features=HOUSE_FEATURES, floor_patches=HOUSE_PATCHES,
                     floor_mu_field=(0.05, 0.6, 11), terrain_assist=assist, limits="firmware", base_max_speed=2.0,
                     wall_height=0.6)


class Driver:
    """The robot, a ball in its pocket, and teleop commands -> firmware velocity requests."""

    def __init__(self, p: SimParams):
        from robot import Robot
        from robot_model import build_model, describe
        self.p = p
        self.m = build_model(p)
        self.d = mujoco.MjData(self.m)
        self.r = Robot(self.m, self.d, p)
        self.describe = describe(self.m, p)
        self.base_adr = self.m.joint("base").qposadr[0]
        self.base_dof = self.m.joint("base").dofadr[0]
        j = self.m.joint("ball")
        self.bq, self.bv = j.qposadr[0], j.dofadr[0]
        self.tilt_peak = 0.0
        self.air_ms = self.air_run = 0.0
        self.fire_t = None
        self.teleport(START)

    def teleport(self, spot):
        m, d = self.m, self.d
        x, y, yaw = spot
        yaw = math.radians(yaw)
        d.qpos[self.base_adr:self.base_adr + 7] = [x, y, 0.004, math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        d.qvel[self.base_dof:self.base_dof + 6] = 0
        self.r.ctl.reset()
        self.r._hold_yaw = yaw
        mujoco.mj_forward(m, d)
        for _ in range(int(0.3 / m.opt.timestep)):
            self.r.drive_velocity((0.0, 0.0), 0.0, self.r.pose()[2])
            self.r.step()
        self.load_ball()
        self.tilt_peak = self.air_ms = 0.0

    def module_point(self, xyz):
        b = self.r.module
        return self.d.xpos[b] + self.d.xmat[b].reshape(3, 3) @ np.asarray(xyz, dtype=float)

    def load_ball(self):
        g = self.r.g
        pos = self.module_point((g.hold[0], 0, g.hold[2] + 0.003))
        self.d.qpos[self.bq:self.bq + 7] = [*pos, 1, 0, 0, 0]
        self.d.qvel[self.bv:self.bv + 6] = self.d.qvel[self.base_dof:self.base_dof + 6] * 0
        self.d.qvel[self.bv:self.bv + 3] = self.d.sensor("base_linvel").data

    def fire(self):
        if self.r.ball_held() and self.fire_t is None:
            self.r.fly_target = 2200 * 2 * math.pi / 60        # a short lob
            self.fire_t = self.d.time

    def step(self, v_body, w, assist: bool):
        r, d = self.r, self.d
        pose = r.pose()
        c, s = math.cos(pose[2]), math.sin(pose[2])
        v_world = (c * v_body[0] - s * v_body[1], s * v_body[0] + c * v_body[1])
        r.edges = self._edges if assist else []
        if not assist:
            r.ctl.max_speed = self.p.base_max_speed
            r.terrain_yaw = None
        if r.terrain_yaw is not None or abs(w) < 1e-3:
            r.drive_velocity(v_world, None, pose[2])          # firmware holds or aligns the heading
        else:
            r._hold_yaw = None
            r.drive_velocity(v_world, w, pose[2])
        if self.fire_t is not None:                           # spin up, trigger, spin down
            if d.time - self.fire_t > 0.35:
                r.gate_open = True
            if d.time - self.fire_t > 0.9:
                r.gate_open, r.fly_target, self.fire_t = False, 0.0, None
        r.step()
        from common import tilt_deg
        t = tilt_deg(d)
        self.tilt_peak = max(self.tilt_peak, t)
        self.air_run = self.air_run + self.m.opt.timestep if np.all(r.wheel_loads() < 0.5) else 0.0
        self.air_ms = max(self.air_ms, 1000 * self.air_run)

    @property
    def _edges(self):
        if not hasattr(self, "_e"):
            import floor
            self._e = floor.edges(self.p)
        return self._e

    def under_wheels(self) -> list:
        """The floor under each wheel (name)."""
        names = []
        for i in range(1, self.p.n_wheels + 1):
            xy = self.d.xpos[self.m.body(f"hub{i}").id][:2]
            name = "wood"
            for x0, y0, x1, y1, _ in HOUSE_PATCHES:
                if x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1:
                    name = "laminate" if y1 <= 2.0 else "tile"
            for f in HOUSE_FEATURES:
                if isinstance(f, Carpet) and f.contains(xy)[0]:
                    name = {TILE: "tile", CARPET: "carpet"}.get(f.rgba, f"rug {f.thickness * 1000:.0f} mm")
            names.append(name)
        return names


# -- the window ------------------------------------------------------------------------------

class _ScriptKeys:
    """Stands in for pygame.key.get_pressed() in --script runs: keys named like 'w', 'shift'."""

    def __init__(self, held):
        import pygame
        self.codes = {getattr(pygame, f"K_{k}") if k != "shift" else pygame.K_LSHIFT for k in held}

    def __getitem__(self, code):
        return code in self.codes

def run(args):
    import pygame
    p = house_params(args.base, assist=not args.no_assist, wheel_model=args.model)
    drv = Driver(p)
    if args.spot:
        drv.teleport(SPOTS[args.spot])
    print(drv.describe)
    W, H = args.width, args.height
    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption(f"omni-robot v7 house drive: {args.base} base")
    font = pygame.font.SysFont("consolas", 17)
    small = pygame.font.SysFont("consolas", 14)
    pads = []
    for k in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(k)
        js.init()
        if any(s in js.get_name() for s in ("KMJ", "3Dconnexion", "SpaceMouse")):
            print("ignoring", js.get_name())          # a 3D mouse's virtual joystick, not a controller
            continue
        pads.append(js)
        print("controller:", js.get_name(), f"({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)")
    ren = mujoco.Renderer(drv.m, H, W)
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = 200.0, -22.0, 2.2
    cam_mode = 0
    assist = not args.no_assist
    clock = pygame.time.Clock()
    dragging = False
    rt_factor = 1.0
    msg, msg_t = "", 0.0
    running = True
    script = list(args.script or [])          # testing: [(seconds, keys held), ...], then a screenshot
    t_script = drv.d.time
    held_script = set()
    phase = None
    while running:
        if args.script is not None:
            if phase is None or drv.d.time - t_script >= phase[0]:
                if not script:
                    pygame.image.save(screen, args.screenshot or "house.png")
                    print("pose", np.round(drv.r.pose(), 3), "peak tilt", round(drv.tilt_peak, 1), "air",
                          round(drv.air_ms), "ball", drv.r.ball_held(), "sim t", round(drv.d.time, 2))
                    break
                phase, t_script = script.pop(0), drv.d.time
                held_script = set(phase[1])
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                k = ev.key
                if k == pygame.K_ESCAPE:
                    running = False
                elif k == pygame.K_t:
                    assist = not assist
                    msg, msg_t = f"terrain assist {'ON' if assist else 'OFF'}", time.time()
                elif k == pygame.K_f:
                    drv.fire()
                elif k == pygame.K_l:
                    drv.load_ball()
                elif k == pygame.K_r:
                    drv.teleport(START)
                elif k == pygame.K_c:
                    cam_mode = (cam_mode + 1) % 3
                elif k == pygame.K_g:
                    args.world_frame = not args.world_frame
                    msg, msg_t = f"{'world' if args.world_frame else 'body'}-frame driving", time.time()
                elif pygame.K_1 <= k <= pygame.K_9:
                    drv.teleport(SPOTS[chr(k)])
            elif ev.type == pygame.JOYBUTTONDOWN:
                b = ev.button
                if b == 0:
                    drv.fire()
                elif b == 1:
                    drv.load_ball()
                elif b == 2:
                    assist = not assist
                    msg, msg_t = f"terrain assist {'ON' if assist else 'OFF'}", time.time()
                elif b == 3:
                    cam_mode = (cam_mode + 1) % 3
                elif b == 7:
                    drv.teleport(START)
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                dragging = True
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                dragging = False
            elif ev.type == pygame.MOUSEMOTION and dragging:
                cam.azimuth -= 0.3 * ev.rel[0]
                cam.elevation = float(np.clip(cam.elevation - 0.3 * ev.rel[1], -89, -3))
            elif ev.type == pygame.MOUSEWHEEL:
                cam.distance = float(np.clip(cam.distance * (0.9 ** ev.y), 0.6, 12.0))
        keys = pygame.key.get_pressed()
        if args.script is not None:
            keys = _ScriptKeys(held_script)
        fast = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        fwd = (keys[pygame.K_w] or keys[pygame.K_UP]) - (keys[pygame.K_s] or keys[pygame.K_DOWN])
        side = keys[pygame.K_a] - keys[pygame.K_d]
        turn = (keys[pygame.K_q] or keys[pygame.K_LEFT]) - (keys[pygame.K_e] or keys[pygame.K_RIGHT])
        vmax = 2.0 if fast else 1.0
        v = np.array([fwd, side], dtype=float)
        if np.linalg.norm(v) > 1:
            v /= np.linalg.norm(v)
        v *= vmax
        w = 2.5 * turn
        keyboard = bool(fwd or side or turn)
        for js in pads if not keyboard else []:
            dz = lambda a: 0.0 if abs(a) < 0.12 else (a - math.copysign(0.12, a)) / 0.88   # noqa: E731
            lx, ly = dz(js.get_axis(0)), dz(js.get_axis(1))
            rx = dz(js.get_axis(2)) if js.get_numaxes() > 2 else 0.0
            if js.get_numaxes() == 4:          # flight stick (Logitech Extreme 3D): twist turns, throttle slider = top speed
                jmax = 1.0 + (1.0 - js.get_axis(3)) / 2
            else:                              # gamepad (XInput): right stick turns, right trigger = 2 m/s
                jmax = 1.0 + ((js.get_axis(5) + 1) / 2 if js.get_numaxes() > 5 else 0.0)
            if abs(lx) + abs(ly) + abs(rx) > 0:
                v = np.array([-ly, -lx]) * jmax
                if np.linalg.norm(v) > jmax:
                    v *= jmax / np.linalg.norm(v)
                w = -3.0 * rx
        if keys[pygame.K_SPACE]:
            v, w = np.zeros(2), 0.0
        if args.world_frame:                       # world-frame: rotate the request into the body frame
            yaw = drv.r.pose()[2]
            c, s = math.cos(yaw), math.sin(yaw)
            v = np.array([c * v[0] + s * v[1], -s * v[0] + c * v[1]])
        # physics: one frame of simulated time, within ~85% of the frame's wall time (slow motion if the PC can't keep up)
        frame_dt = 1 / args.fps
        t_wall = time.perf_counter()
        target = drv.d.time + frame_dt
        while drv.d.time < target:
            drv.step(v, w, assist)
            if time.perf_counter() - t_wall > 0.85 * frame_dt:
                break
        dt_wall = time.perf_counter() - t_wall
        rt_factor = 0.9 * rt_factor + 0.1 * min(2.0, (drv.d.time - (target - frame_dt)) / max(dt_wall, 1e-6))
        # camera
        base = drv.d.xpos[drv.m.body("base").id]
        cam.lookat[:] = base + np.array([0.0, 0.0, 0.12])
        if cam_mode == 1:          # chase: behind the robot
            cam.azimuth = math.degrees(drv.r.pose()[2]) + 180.0 + 180.0
            cam.elevation = -18.0
        elif cam_mode == 2:        # top
            cam.elevation, cam.distance = -89.0, max(cam.distance, 5.0)
        ren.update_scene(drv.d, camera=cam, scene_option=opt)
        img = ren.render()
        surf = pygame.surfarray.make_surface(np.ascontiguousarray(np.transpose(img, (1, 0, 2))))
        screen.blit(surf, (0, 0))
        # HUD
        r = drv.r
        vel = drv.d.sensor("base_linvel").data[:2]
        from common import tilt_deg
        loads = r.wheel_loads()
        lines = [
            f"{args.base} base ({p.omni.name[:34]}, {'sprung pods' if p.suspension == 'pod' else 'roller compliance'})",
            f"speed {np.linalg.norm(vel):4.2f} m/s  (cap {r.ctl.max_speed:.1f})   tilt {tilt_deg(drv.d):4.1f} deg, "
            f"peak {drv.tilt_peak:4.1f}   airborne peak {drv.air_ms:3.0f} ms",
            f"assist {'ON ' if assist else 'off'}" + ("  ALIGNING to cross a square edge" if r.terrain_yaw is not None else "")
            + f"   ball {'aboard' if r.ball_held() else 'out'}   {'world' if args.world_frame else 'body'} frame"
            + f"   sim x{rt_factor:.2f} real time",
            "wheels: " + "  ".join(f"{n} {L:4.1f} N" for n, L in zip(drv.under_wheels(), loads)),
        ]
        pygame.draw.rect(screen, (0, 0, 0), (8, 8, 700, 10 + 22 * len(lines)))
        for k, line in enumerate(lines):
            screen.blit(font.render(line, True, (255, 255, 255)), (16, 12 + 22 * k))
        help_ = "WASD move  QE turn  SHIFT 2 m/s  SPACE stop  T assist  F fire  L load  R reset  C camera  1-9 spots  G frame"
        screen.blit(small.render(help_, True, (255, 255, 255), (0, 0, 0)), (12, H - 24))
        if msg and time.time() - msg_t < 2.0:
            screen.blit(font.render(msg, True, (255, 220, 60), (0, 0, 0)), (W // 2 - 120, 80))
        pygame.display.flip()
        clock.tick(args.fps)
    pygame.quit()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", default="v7", choices=list(BASES))
    ap.add_argument("--no-assist", action="store_true", help="start with the terrain-aware firmware off (T toggles)")
    ap.add_argument("--model", default="rollers", choices=["rollers", "sphere"],
                    help="wheel model (sphere: faster, but it can't get stuck on a square edge sideways)")
    ap.add_argument("--world-frame", action="store_true", help="sticks/keys move in the world frame, not the body frame")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=40)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--spot", choices=list(SPOTS), help="start at a teleport spot instead of the hallway")
    ap.add_argument("--script", help="testing: 'secs:keys,secs:keys' e.g. '1.5:w+shift,1:' then save --screenshot")
    ap.add_argument("--screenshot", help="testing: PNG written when the script ends")
    args = ap.parse_args()
    if args.script is not None:
        args.script = [(float(s.split(":")[0]), [k for k in s.split(":")[1].split("+") if k]) for s in args.script.split(",")]
    if args.list:
        for k, v in BASES.items():
            print(k, v)
        return
    run(args)


if __name__ == "__main__":
    main()
