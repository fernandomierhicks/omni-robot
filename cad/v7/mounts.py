"""Fixed parts that carry the tilting module and the cameras. ROBOT frame.

  yoke L / R      A-frames on the payload plate at y = +-(105..113) (the sim's yokes), outside the module's swept
                  volume. L: a 608 bearing for the module's 8 mm shaft, the flywheel ESC on its outer face.
                  R: the STS3250 in a window at the top, held by servo_cap; the RoboClaw on its outer face.
  cam_arm x 2     on the payload plate's top (3 x M3 each), cantilevered forward at |y| 89-103 to a post behind the
                  stereo bar (6061 flat bar 25 x 6 x 244); above the lidar's layer, beside the flywheels' sweep
  cam_mount x 2   30 deg wedges on the bar's front face for the OV9782 boards
Cameras: the sim puts them at (115, +-95, plate top + 20), where the flywheel motors pass at ~+15 deg of tilt. Here
the bar is at x 162-168, 12.5..37.5 mm over the plate: cameras ~55 mm forward and 12.5 mm up from the sim's spot.
"""

from __future__ import annotations

import math
from functools import lru_cache

from build123d import (Align, Box, Cylinder, Location, Part, Plane, Polygon, Pos, Rot, extrude)

import base as B
import dims as D
import hw
import pod as P
import vendor as V

CB = (Align.CENTER, Align.CENTER, Align.MIN)
PX, PZ = D.PIVOT_R                      # pivot (-10, 329.3)
YOKE_Y = (D.YOKE_Y, D.YOKE_Y + 8.0)     # 105 .. 113
BASE_X = (-50.0, 42.0)
TOP_HALF = 16.0
YOKE_SCREWS = B.YOKE_SCREWS_X
ESC_Z, RC_Z, ELRS_Z = D.PLATE_TOP + 90.5, D.PLATE_TOP + 57.5, D.PLATE_TOP + 16.5    # on the yokes' outer faces
BUCKS = [("12 V buck", "12 V 5 A step-down (Pololu D36V50F12 class) for the feed motors' RoboClaw and the STS3250",
          "pololu.com #4096 (~$30)", D.PLATE_TOP + 49.5),
         ("19 V buck", "19 V 4 A step-down for the Jetson dev kit (est)", "TBD (~$15)", D.PLATE_TOP + 22.5)]
BUCK_SIZE, BUCK_HOLES = (35.0, 25.0, 10.0), [(-15.0, 0.0), (15.0, 0.0)]
ESC_HOLES = [(dx, dz) for dx in (-15.25, 15.25) for dz in (-15.25, 15.25)]
RC_HOLES = [(dx, dz) for dx in (-29.2, 29.2) for dz in (-20.3, 20.3)]      # RoboClaw 2x7A (VERIFY)
BOSS_ESC, BOSS_RC = 6.0, 8.0


def _xz_prism(pts, y0, y1) -> Part:
    return extrude(Plane.XZ.offset(-y0) * Polygon(*pts, align=None), y1 - y0, dir=(0, 1, 0))


def _ycyl(r, x, z, y0, y1) -> Part:
    return Cylinder(r, y1 - y0, rotation=(-90, 0, 0), align=CB).moved(Pos(x, y0, z))


# --- servo geometry -----------------------------------------------------------------------------------------------------

def horn_y() -> float:
    """|y| of the servo horn's outer face (the module's -y boss end + the 3.5 mm horn)."""
    import module as M
    return M.PIVOT_BOSS_Y[1] + 3.5


SERVO_TOP_Y = None   # set below (needs module)


def servo_y():
    top = horn_y() + 1.8                  # room for the horn screws' heads
    return top, top + D.STS3250["h"]          # |y| of the case top (spline side) and bottom


SERVO_WIN = dict(x=(PX - 12.6, PX + 12.6), z=(PZ - 35.0, PZ + 11.6))      # case window (spline 11 mm from its top end)
SERVO_CAP_X = (PX - 21.0, PX + 21.0)


def servo_loc() -> Location:
    """STS3250 frame (spline +Z at the origin, case top at z = 0, long side +X) -> ROBOT: spline toward +y on the pivot
    axis, long side down."""
    top, _ = servo_y()
    return Pos(PX, -top, PZ) * Rot(-90, 0, 0) * Rot(0, 0, 90)


# --- yokes ----------------------------------------------------------------------------------------------------------------

def _yoke_body(servo: bool) -> Part:
    y0, y1 = YOKE_Y
    zt = PZ + (22.0 if servo else 16.0)
    w_top = TOP_HALF + (6.0 if servo else 0.0)
    p = _xz_prism([(BASE_X[0], D.PLATE_TOP), (BASE_X[1], D.PLATE_TOP), (PX + w_top, zt), (PX - w_top, zt)], y0, y1)
    z0, z1 = D.PLATE_TOP + 22.0, PZ - (62.0 if servo else 40.0)

    def edge(z):
        f = (z - D.PLATE_TOP) / (zt - D.PLATE_TOP)
        return BASE_X[0] + (PX - w_top - BASE_X[0]) * f + 12.0, BASE_X[1] + (PX + w_top - BASE_X[1]) * f - 12.0
    (a0, b0), (a1, b1) = edge(z0), edge(z1)
    p -= _xz_prism([(a0, z0), (b0, z0), (b1, z1), (a1, z1)], y0 - 1, y1 + 1)      # lightening window
    p += Box(BASE_X[1] - BASE_X[0] - 8, 8.0, 6.0, align=(Align.MIN, Align.MAX, Align.MIN)).moved(
        Pos(BASE_X[0] + 4, y0, D.PLATE_TOP))                                       # flange, inboard (pan heads in counterbores)
    for x in YOKE_SCREWS:
        p -= hw.cbore(6.0, top=D.PLATE_TOP + 6.0).moved(Pos(x, y0 - 4.0, 0))
    for x in (BASE_X[0] + 14, BASE_X[1] - 14):                                     # gussets
        p += extrude(Plane.YZ.offset(x - 2) * Polygon((y0 - 8, D.PLATE_TOP + 6), (y0, D.PLATE_TOP + 6),
                                                      (y0, D.PLATE_TOP + 30), align=None), 4.0, dir=(1, 0, 0))
    return p


@lru_cache(maxsize=None)
def yoke(side: int) -> Part:
    """side +1: yoke L (608 bearing, ESC); -1: yoke R (servo window, RoboClaw). Modelled at +y; R is mirrored."""
    y0, y1 = YOKE_Y
    servo = side < 0
    p = _yoke_body(servo)
    if not servo:
        p += _ycyl(16.0, PX, PZ, y0, y1)
        p -= _ycyl(11.0, PX, PZ, y0 - 0.01, y0 + 7.1)                              # 608 seat, from the module side
        p -= _ycyl(5.0, PX, PZ, y0, y1 + 1)
        for dx, dz in ESC_HOLES + [(bx, bz + z - ESC_Z) for *_n, z in BUCKS for bx, bz in BUCK_HOLES]:
            p += _ycyl(4.0, PX + dx, ESC_Z + dz, y1, y1 + BOSS_ESC)
            p -= _ycyl(D.TAP_M3 / 2, PX + dx, ESC_Z + dz, y1 - 6.0, y1 + BOSS_ESC + 0.1)
    else:
        wx, wz = SERVO_WIN["x"], SERVO_WIN["z"]
        zc = (wz[0] + wz[1]) / 2
        p += Box(wx[1] - wx[0] + 14, y1 - y0, wz[1] - wz[0] + 14, align=(Align.CENTER, Align.MIN, Align.CENTER)).moved(
            Pos(PX, y0, zc))
        p -= Box(wx[1] - wx[0], 40, wz[1] - wz[0], align=(Align.CENTER, Align.MIN, Align.CENTER)).moved(Pos(PX, y0 - 1, zc))
        for x in SERVO_CAP_X:
            p -= _ycyl(D.TAP_M3 / 2, x, zc, y0 - 0.1, y1 + 0.1)
        for dx, dz in RC_HOLES:
            p += _ycyl(4.0, PX + dx, RC_Z + dz, y1, y1 + BOSS_RC)
            p -= _ycyl(D.TAP_M3 / 2, PX + dx, RC_Z + dz, y1 - 6.0, y1 + BOSS_RC + 0.1)
    p.label = "yoke"
    if side > 0:
        return p
    m = p.mirror(Plane.XZ)
    m.label = "yoke"
    return m


@lru_cache(maxsize=None)
def servo_cap() -> Part:
    """Holds the STS3250's case bottom into yoke R's window: a plate on the case bottom with two legs to the yoke."""
    _, bot = servo_y()
    wz = SERVO_WIN["z"]
    zc = (wz[0] + wz[1]) / 2
    yo = bot + 4.0                                              # |y| of the cap's outer face
    p = Box(54.0, 4.0, wz[1] - wz[0], align=(Align.CENTER, Align.MIN, Align.CENTER)).moved(Pos(PX, bot, zc))
    for x in SERVO_CAP_X:
        p += Box(9.0, yo - YOKE_Y[1], 12.0, align=(Align.CENTER, Align.MIN, Align.CENTER)).moved(Pos(x, YOKE_Y[1], zc))
        p -= _ycyl(D.CLR_M3 / 2, x, zc, YOKE_Y[1] - 1, yo + 1) + _ycyl(D.CB_M3[0] / 2, x, zc, yo - D.CB_M3[1], yo + 1)
    p -= Box(14.0, 10, 8.0).moved(Pos(PX, bot + 2, wz[0] + 6))           # servo cable
    m = p.mirror(Plane.XZ)
    m.label = "servo_cap"
    return m


# --- stereo bar ------------------------------------------------------------------------------------------------------------
CAM_BAR_X = 165.0                                    # stereo bar centre (front face x 168)
CAM_Z = D.CAM_SIM[0][1][2] + D.CAM_RAISE             # plate top + 32.5
CAM_Y = D.CAM_BASELINE / 2                           # 95
BAR = dict(x=(CAM_BAR_X - 3.0, CAM_BAR_X + 3.0), z=(CAM_Z - 20.0, CAM_Z + 5.0), half_len=122.0)
POST_X = (146.0, BAR["x"][0])                        # the arm's post, right behind the bar
POST_TOP = CAM_Z + 2.0
POST_BAR_Z = (CAM_Z - 14.0, CAM_Z - 6.0)             # post -> bar screws (M3 tapped in the bar), at y = +-CAM_Y
ARM = dict(x0=44.0, y=(89.0, 103.0), h=14.0)         # beam on the plate, clear of the flywheel mounts (y <= 87) at
                                                     # full tilt and of the yokes' flanges (x <= 38)
ARM_FOOT = dict(x=(44.0, 72.0), y=(89.0, 105.0), t=6.0)   # outboard of the flywheel mounts (y <= 87) too
WEDGE_SCREWS_Y = (CAM_Y - 12.0, CAM_Y + 12.0)        # wedge -> bar, from behind the bar, beside the post


@lru_cache(maxsize=None)
def cam_arm() -> Part:
    """+y camera arm, ROBOT frame: a foot on the payload plate (3 x M3 into its tapped holes, heads in pockets), a
    14 mm beam forward over the plate's edge, and a post up behind the stereo bar (2 x M3 along +x into the bar)."""
    z0 = D.PLATE_TOP
    fx, fy = ARM_FOOT["x"], ARM_FOOT["y"]
    p = Box(fx[1] - fx[0], fy[1] - fy[0], ARM_FOOT["t"], align=(Align.MIN, Align.MIN, Align.MIN)).moved(Pos(fx[0], fy[0], z0))
    ay = ARM["y"]
    p += Box(POST_X[1] - ARM["x0"], ay[1] - ay[0], ARM["h"], align=(Align.MIN, Align.MIN, Align.MIN)).moved(Pos(ARM["x0"], ay[0], z0))
    p += Box(POST_X[1] - POST_X[0], 16.0, POST_TOP - z0, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(Pos(POST_X[0], CAM_Y, z0))
    seat = z0 + ARM_FOOT["t"] - D.CB_M3[1]
    for x, y in B.CAM_ARM_SCREWS:
        p -= hw.clear(ARM_FOOT["t"], top=z0 + ARM_FOOT["t"]).moved(Pos(x, y, 0))
        p -= Cylinder(3.5, 40.0, align=CB).moved(Pos(x, y, seat))
    for z in POST_BAR_Z:
        p -= Cylinder(D.CLR_M3 / 2, 40, rotation=(0, 90, 0)).moved(Pos(BAR["x"][0], CAM_Y, z))
        p -= Cylinder(D.CB_M3[0] / 2, 2 * D.CB_M3[1], rotation=(0, 90, 0)).moved(Pos(POST_X[0], CAM_Y, z))
    p.label = "cam_arm"
    return p


def cam_board_loc(side: int = 1) -> Location:
    """OV9782 board frame (back at z = 0, lens +Z) -> ROBOT: on the wedge's slope, looking forward, 30 deg up. 16 mm
    ahead of the bar, so the wedge behind the board's upper M2 holes is >= 4 mm deep."""
    return Pos(BAR["x"][1] + 16.0, side * CAM_Y, CAM_Z) * Rot(0, 90 - D.CAM_TILT, 0) * Rot(0, 0, 90)


@lru_cache(maxsize=None)
def cam_mount() -> Part:
    """Wedge: back on the bar's front face (2 x M3 from behind the bar), the camera board on its 30 deg face (4 x M2)."""
    x0 = BAR["x"][1]
    board = cam_board_loc()
    wedge = Box(46.0, 46.0, 30.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(board * Pos(0, 0, -0.01))
    wedge &= Box(60.0, 2 * 21.0, 50.0, align=(Align.MIN, Align.CENTER, Align.CENTER)).moved(Pos(x0, CAM_Y, CAM_Z))
    for y in WEDGE_SCREWS_Y:
        wedge -= Cylinder(D.TAP_M3 / 2, 8.0, rotation=(0, 90, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Pos(x0 - 0.01, y, CAM_Z))
    for dy in (-17.0, 17.0):
        for dz in (-17.0, 17.0):
            wedge -= hw.tap(8.0, d=D.TAP_M2).moved(board * Pos(dy, dz, 0.0))
    wedge -= Box(18.0, 18.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(board * Pos(0, 0, 0.01))   # sensor back
    wedge.label = "cam_mount"
    return wedge


def stereo_bar() -> Part:
    """6061 flat bar 25 x 6 x 244, ROBOT frame: clearance holes for the wedge screws, M3 tapped holes for the posts."""
    bx0, bx1 = BAR["x"]
    p = Box(bx1 - bx0, 2 * BAR["half_len"], BAR["z"][1] - BAR["z"][0], align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
        Pos(bx0, 0, BAR["z"][0]))
    for s in (1, -1):
        for y in WEDGE_SCREWS_Y:
            p -= Cylinder(D.CLR_M3 / 2, 20, rotation=(0, 90, 0)).moved(Pos(bx0, s * y, CAM_Z))
        for z in POST_BAR_Z:
            p -= Cylinder(1.25, 20, rotation=(0, 90, 0)).moved(Pos(bx0, s * CAM_Y, z))
    return p


def hardware() -> hw.Hardware:
    h = hw.Hardware()
    y0, y1 = YOKE_Y
    for s in (1, -1):
        for x in YOKE_SCREWS:                                       # yoke flanges -> plate
            h.screw("PAN", 10, Pos(x, s * (y0 - 4.0), D.PLATE_TOP + 6.0 - D.CB_M3[1]))
    h.part("608-2RS bearing", "608-2RS ball bearing 8 x 22 x 7 (tilt pivot, yoke L)", "generic; McMaster 6383K (608)",
           hw.bearing_shape(8.0, 22.0, 7.0), Pos(PX, y0 + 3.55, PZ) * hw.rot_to("y"))
    h.part("Feetech STS3250", "Feetech STS3250 serial-bus servo, 12 V, 12-bit absolute encoder (tilt, section 24)",
           "feetechrc.com / robotshop ($73.99)", V.sts3250(), servo_loc())
    _, bot = servo_y()
    zc = sum(SERVO_WIN["z"]) / 2
    for x in SERVO_CAP_X:                                           # cap -> yoke R (shank toward +y), all 8 mm of the yoke
        h.screw("PAN", 30, Pos(x, -(bot + 4.0 - D.CB_M3[1]), zc) * hw.rot_to("-y"))
    esc = V.pcb(42.0, 42.0, 7.0, [(dx, dz) for dx, dz in ESC_HOLES], "ESC 4-in-1", inset=9.0)
    h.part("ESC 8S 4-in-1 (AM32)", "8S AM32 4-in-1 ESC, bidirectional DShot, 30.5 x 30.5 mount (Pilotix 65A / TBS Lucid 8S)",
           "section 24 (~$56-73)", esc, Pos(PX, y1 + BOSS_ESC, ESC_Z) * hw.rot_to("y"))
    for dx, dz in ESC_HOLES:
        h.screw("PAN", 10, Pos(PX + dx, y1 + BOSS_ESC + 1.6, ESC_Z + dz) * hw.rot_to("y"))
    for name, desc, src, z in BUCKS:
        w, d, hh = BUCK_SIZE
        h.part(name, desc, src, V.pcb(w, d, hh, BUCK_HOLES, name, inset=6.0), Pos(PX, y1 + BOSS_ESC, z) * hw.rot_to("y"))
        for dx, dz in BUCK_HOLES:
            h.screw("PAN", 10, Pos(PX + dx, y1 + BOSS_ESC + 1.6, z + dz) * hw.rot_to("y"))
    h.part("ELRS receiver", "ELRS 2.4 GHz receiver (RadioMaster RP2 class), 3M dual-lock on yoke R, antenna up",
           "RadioMaster ($15)", V.board(22.0, 14.0, 5.0, "ELRS receiver"), Pos(PX, -(y1 + 0.5), ELRS_Z) * hw.rot_to("-y"))
    rc = V.pcb(68.6, 50.8, 15.0, [(dx, dz) for dx, dz in RC_HOLES], "RoboClaw 2x7A", inset=9.0)
    h.part("RoboClaw 2x7A", "Basicmicro RoboClaw 2x7A (feed motors' speed / position loops)", "basicmicro.com ($79.95)",
           rc, Pos(PX, -(y1 + BOSS_RC), RC_Z) * hw.rot_to("-y"))
    for dx, dz in RC_HOLES:
        h.screw("PAN", 12, Pos(PX + dx, -(y1 + BOSS_RC + 1.6), RC_Z + dz) * hw.rot_to("-y"))
    # stereo bar, arms, wedges, cameras
    bx0, bx1 = BAR["x"]
    h.part("stereo bar", "6061 aluminium flat bar 25 x 6 mm x 244 mm; 4 x 3.4 holes (wedges), 4 x M3 tapped (posts)",
           "McMaster 8975K (6061 flat bar) or 25 x 6 metric stock", stereo_bar(), Location())
    seat = D.PLATE_TOP + ARM_FOOT["t"] - D.CB_M3[1]
    for s in (1, -1):
        for x, y in B.CAM_ARM_SCREWS:                               # arm foot -> plate (6 mm: the plate's thickness)
            h.screw("PAN", 10, Pos(x, s * y, seat))
        for z in POST_BAR_Z:                                        # post -> bar (tapped)
            h.screw("PAN", 18, Pos(POST_X[0] + D.CB_M3[1], s * CAM_Y, z) * hw.rot_to("-x"))
        for y in WEDGE_SCREWS_Y:                                    # bar -> wedge, from behind the bar
            h.screw("PAN", 12, Pos(bx0, s * y, CAM_Z) * hw.rot_to("-x"))
        cam = cam_board_loc(s)
        h.part("OV9782 camera", "OV9782 global-shutter colour camera, M12 lens, 22-pin FFC (MCU-triggered)",
               "Arducam / Luxonis-class module (VERIFY driver at 1280x800 @ 120 fps)", V.camera(), cam)
        for dy in (-17.0, 17.0):
            for dz in (-17.0, 17.0):
                h.screw("PAN", 6, cam * Pos(dy, dz, 1.6), size="M2")
    return h


def printed() -> list[tuple[str, Part, str, int, str]]:
    cm = cam_mount()
    ca = cam_arm()
    return [("yoke_L", yoke(1), "PETG", 40, "on its outer face"),
            ("yoke_R", yoke(-1), "PETG", 40, "on its outer face (mirror of L with the servo window)"),
            ("servo_cap", servo_cap(), "PETG", 40, "plate down"),
            ("cam_arm_L", ca, "PETG", 60, "on its outer side face"),
            ("cam_arm_R", _mir(ca, "cam_arm"), "PETG", 60, "on its outer side face"),
            ("cam_mount_L", cm, "PETG", 60, "back face down"),
            ("cam_mount_R", _mir(cm, "cam_mount"), "PETG", 60, "back face down")]


def _mir(p, label):
    m = p.mirror(Plane.XZ)
    m.label = label
    return m
