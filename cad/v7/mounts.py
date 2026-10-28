"""Fixed parts that carry the tilting module and the boards on its yokes. ROBOT frame.

  yoke L / R      A-frames on the payload plate at y = +-(105..113) (the sim's yokes), outside the module's swept
                  volume. L: a 608 bearing for the module's 8 mm shaft, the flywheel ESC on its outer face.
                  R: the STS3250 in a window at the top, held by servo_cap; the RoboClaw on its outer face.
The cameras and the display are on the head at the plate's front edge (head.py, display.py).
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
YOKE_Y = (D.YOKE_Y, D.YOKE_Y + 10.0)    # 105 .. 115 (10 mm: the 608's shoulder was 0.9 mm in an 8 mm yoke)
FLANGE_W = 10.0                         # inboard flange: 1.8 mm round its counterbores (0.8 at 8 mm)
# No lightening window: the boards' bosses on the outer faces stood in it, joined to nothing (yoke L printed as six
# pieces), and its 12 mm legs were the thinnest columns carrying the module.
BASE_X = (-50.0, 42.0)
TOP_HALF = 16.0
YOKE_SCREWS = B.YOKE_SCREWS_X
ESC_Z, RC_Z, ELRS_Z = D.PLATE_TOP + 90.5, D.PLATE_TOP + 57.5, D.PLATE_TOP + 16.5    # on the yokes' outer faces
# the catch kit's boards ride on its yokes (the base's own, the Jetson's 19 V buck and the RC receiver, moved to the
# deck's front splice and the head's right cheek on Oct 25: they stay when the kit is swapped for the arm)
BUCKS = [("12 V buck", "12 V 5 A step-down (Pololu D36V50F12 class) for the feed motors' RoboClaw and the STS3250",
          "pololu.com #4096 (~$30)", D.PLATE_TOP + 49.5)]
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
    p += Box(BASE_X[1] - BASE_X[0] - 8, FLANGE_W, 6.0, align=(Align.MIN, Align.MAX, Align.MIN)).moved(
        Pos(BASE_X[0] + 4, y0, D.PLATE_TOP))                                       # flange, inboard (pan heads in counterbores)
    for x in YOKE_SCREWS:
        p -= hw.cbore(6.0, top=D.PLATE_TOP + 6.0).moved(Pos(x, y0 - B.YOKE_SCREW_IN, 0))
    for k in range(len(YOKE_SCREWS) - 1):                                          # gussets, between the screws
        x = (YOKE_SCREWS[k] + YOKE_SCREWS[k + 1]) / 2                              # (beside them they hid the heads)
        p += extrude(Plane.YZ.offset(x - 2) * Polygon((y0 - FLANGE_W, D.PLATE_TOP + 6), (y0, D.PLATE_TOP + 6),
                                                      (y0, D.PLATE_TOP + 34), align=None), 4.0, dir=(1, 0, 0))
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
        p -= hw.lead_in(11.0).moved(Pos(PX, y0, PZ) * hw.rot_to("-y"))
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


def hardware() -> hw.Hardware:
    h = hw.Hardware()
    y0, y1 = YOKE_Y
    for s in (1, -1):
        for x in YOKE_SCREWS:                                       # yoke flanges -> plate
            h.screw("PAN", 10, Pos(x, s * (y0 - B.YOKE_SCREW_IN), D.PLATE_TOP + 6.0 - D.CB_M3[1]))
    h.part("608-2RS bearing", "608-2RS ball bearing 8 x 22 x 7 (tilt pivot, yoke L)", "generic; McMaster 6383K (608)",
           hw.bearing_shape(8.0, 22.0, 7.0), Pos(PX, y0 + 3.55, PZ) * hw.rot_to("y"))
    h.part("Feetech STS3250", "Feetech STS3250 serial-bus servo, 12 V, 12-bit absolute encoder (tilt, section 24)",
           "feetechrc.com / robotshop ($73.99)", V.sts3250(), servo_loc())
    _, bot = servo_y()
    zc = sum(SERVO_WIN["z"]) / 2
    for x in SERVO_CAP_X:                                           # cap -> yoke R (shank toward +y), 8 mm into the yoke
        h.screw("PAN", 25, Pos(x, -(bot + 4.0 - D.CB_M3[1]), zc) * hw.rot_to("-y"))
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
    rc = V.pcb(68.6, 50.8, 15.0, [(dx, dz) for dx, dz in RC_HOLES], "RoboClaw 2x7A", inset=9.0)
    h.part("RoboClaw 2x7A", "Basicmicro RoboClaw 2x7A (feed motors' speed / position loops)", "basicmicro.com ($79.95)",
           rc, Pos(PX, -(y1 + BOSS_RC), RC_Z) * hw.rot_to("-y"))
    for dx, dz in RC_HOLES:
        h.screw("PAN", 12, Pos(PX + dx, -(y1 + BOSS_RC + 1.6), RC_Z + dz) * hw.rot_to("-y"))
    return h


def printed() -> list[tuple[str, Part, str, int, str]]:
    return [("yoke_L", yoke(1), "PETG", 40, "on its outer face"),
            ("yoke_R", yoke(-1), "PETG", 40, "on its outer face (mirror of L with the servo window)"),
            ("servo_cap", servo_cap(), "PETG", 40, "plate down")]
