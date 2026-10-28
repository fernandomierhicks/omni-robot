"""The arm kit (design.md 12.3, Oct 25): what replaces the catch module, its yokes and their boards when the robot
carries the SO-101. The base and the head stay; the camera bar turns to its arm index (40 deg down); the face and the
audio box move to the kit. ROBOT frame.

  arm_riser     printed hollow block on the plate's front edge (4 x M3 down through its floor into the plate, keyed
                by the plate's front dowel), 43.5 mm tall, behind the camera bar
  arm_adapter   printed 10 mm plate on the riser (4 x M3 x 20 into its corner bosses, heads counterbored), running
                forward over the camera bar (3 mm over the tube's highest corner at any index) to under the SO-101's
                base; the base on 4 x M3 (VERIFY its hole pattern on TheRobotStudio's STEP)
  bin           printed open box 120 x 120 x 70 on the plate's rear (4 x M3 into the audio box's holes), just clear of
                the plate's rear dowel
  face_upright  printed, one each side at |y| 64-72 (as the head's cheeks; a 4 mm foot against the bin): 2 x M3 into
                the bin's side walls, the
                face unit on 2 x M3 each side as on the cheeks, the shelf on top
  audio_shelf   printed 6 mm plate on the uprights: the audio box on it with its own 4 screws
  12 V buck     on the riser's left wall: the SO-101's servo bus (its bus board rides on the arm's base)

Why here (simulation/v11 sil/studies/layout_trade.py A6, with the SO-101's own meshes against the drawn head):
  - The arm's base and shoulder bracket reach 65 / 86 mm ahead of its base origin, 0-100 mm up: on the plate they
    meet the camera bar wherever the arm reaches past the bumper. The study's A5 (x 0.10 on the plate) checked only
    thin capsules along the links and missed that. On a 50 mm riser the base clears the bar's top by 3+ mm and the
    shoulder clears it at every pan; the base's front edge stays inside the bumper up to x 0.14.
  - At x 0.14: top-down floor grasps 0.25-0.41 m from the centre, a drop into the bin 10 deg off vertical, a stow
    folded sideways 0.37 m up.
  - The face over the bin's back, 0.40 m up, facing forward (A5's spot), every viewer sees it over the stowed arm.
"""

from __future__ import annotations

import math
from functools import lru_cache

from build123d import Align, Box, Cylinder, Part, Plane, Polygon, Pos, extrude

import base as B
import dims as D
import display as DP
import hw

CB = (Align.CENTER, Align.CENTER, Align.MIN)
MIN = (Align.MIN, Align.MIN, Align.MIN)

ARM_ORIGIN = (140.0, 0.0, 224.5)         # the SO-101's base frame (simulation's arm_mount x 0.14, arm_riser 0.05)
ARM_CG = (150.0, 0.0, 300.0)             # the arm's CG for the mass budget, stowed (est)
BASE_BELOW = 2.4                         # the base's printed underside sits this far under its frame's origin
RISER = dict(x=(62.0, 130.0), y=48.0, top=212.0, wall=4.0, floor=4.0)
ADAPTER = dict(x=(62.0, 205.0), y=56.0, top=ARM_ORIGIN[2] - BASE_BELOW, r=211.0)
RISER_BOSSES = [(x, s * 40.0) for x in (70.0, 122.0) for s in (-1, 1)]
RISER_SCREWS = [(75.0, 25.0), (75.0, -25.0)] + list(B.ARM_SCREWS)     # into the plate: (75, +-25) are on its grid
SO101_SCREWS = [(ARM_ORIGIN[0] + dx, s * 45.0) for dx in (-12.0, 52.0) for s in (-1, 1)]   # tapped for the arm's base's
                                                                                          # 4 x M3 (est, VERIFY)
BIN = dict(x=(-130.0, -10.0), y=60.0, top=D.PLATE_TOP + 70.0, wall=3.0, floor=4.0)
UPRIGHT_Y = (64.0, 72.0)
FOOT_SCREWS = [(-121.0, 225.0), (-101.0, 225.0)]       # upright -> the bin's side wall bosses (x, z)
SHELF = dict(x=(-176.0, -120.0), z=(450.0, 456.0), y=72.0)
SHELF_SCREWS = [(-170.0, s * 68.0) for s in (-1, 1)] + [(-160.0, s * 68.0) for s in (-1, 1)]   # clear of the face's top screw
BUCK = dict(x=110.0, z=195.0)            # the 12 V buck on the riser's +y wall (its long side along x)


def face_front_x(z: float) -> float:
    """x of the face's front plane at height z, at its arm seat."""
    x0, z0 = DP.SEATS["arm"]
    return x0 - (z - z0) * math.tan(math.radians(DP.LEAN))


def _box(x, y, z) -> Part:
    return Box(x[1] - x[0], y[1] - y[0], z[1] - z[0], align=MIN).moved(Pos(x[0], y[0], z[0]))


def _xz_prism(pts, y0, y1) -> Part:
    return extrude(Plane.XZ.offset(-y0) * Polygon(*pts, align=None), y1 - y0, dir=(0, 1, 0))


def _ycyl(r, y0, y1, x, z) -> Part:
    return Cylinder(r, y1 - y0, rotation=(-90, 0, 0), align=CB).moved(Pos(x, y0, z))


# --- the arm's riser and adapter -------------------------------------------------------------------------------------

@lru_cache(maxsize=None)
def riser() -> Part:
    x0, x1 = RISER["x"]
    y, t, z0, z1 = RISER["y"], RISER["wall"], D.PLATE_TOP, RISER["top"]
    p = _box((x0, x1), (-y, y), (z0, z1)) - _box((x0 + t, x1 - t), (-y + t, y - t), (z0 + RISER["floor"], z1 + 1))
    for x, yy in RISER_BOSSES:                                             # the adapter's screws, from the top
        p += Cylinder(4.5, z1 - z0, align=CB).moved(Pos(x, yy, z0))
        p -= hw.tap(12.0, top=z1, entry="top").moved(Pos(x, yy, 0))
    for x, yy in RISER_SCREWS:                                             # down into the plate, from inside
        p -= hw.clear(RISER["floor"], top=z0 + RISER["floor"]).moved(Pos(x, yy, 0))
    px, py = B.PIN_XY[0]                                                    # keyed on the plate's front dowel
    p -= Cylinder(2.1, 11.0, align=CB).moved(Pos(px, py, z0 - 0.01))
    for dx in (-15.0, 15.0):                                               # the 12 V buck's bosses on the +y wall
        p += _ycyl(4.0, y, y + 4.0, BUCK["x"] + dx, BUCK["z"])
        p -= hw.tap(8.0, top=0.0).moved(Pos(BUCK["x"] + dx, y + 4.0, BUCK["z"]) * hw.rot_to("y"))
    p -= _box((x0 + 20, x1 - 20), (-y - 1, -y + t + 1), (z0 + 12, z1 - 10))  # cables out the -y wall
    p.label = "arm_riser"
    return p


@lru_cache(maxsize=None)
def adapter() -> Part:
    x0, x1 = ADAPTER["x"]
    y = ADAPTER["y"]
    p = _box((x0, x1), (-y, y), (RISER["top"], ADAPTER["top"]))
    p &= Cylinder(ADAPTER["r"], 100, align=CB).moved(Pos(0, 0, 150))
    for x, yy in RISER_BOSSES:
        p -= hw.cbore(ADAPTER["top"] - RISER["top"], top=ADAPTER["top"]).moved(Pos(x, yy, 0))
    for x, yy in SO101_SCREWS:                                             # the SO-101's base, from above
        p -= hw.tap(ADAPTER["top"] - RISER["top"] + 0.1, top=ADAPTER["top"], entry="top").moved(Pos(x, yy, 0))
    p -= _box((x0 + 12, x0 + 40), (-12.0, 12.0), (RISER["top"] - 1, ADAPTER["top"] + 1))   # the arm's cables down
    p.label = "arm_adapter"
    return p


# --- the bin, the face's uprights, the shelf -------------------------------------------------------------------------

@lru_cache(maxsize=None)
def bin_() -> Part:
    x0, x1 = BIN["x"]
    y, t, z0, z1 = BIN["y"], BIN["wall"], D.PLATE_TOP, BIN["top"]
    p = _box((x0, x1), (-y, y), (z0, z1)) - _box((x0 + t, x1 - t), (-y + t, y - t), (z0 + BIN["floor"], z1 + 1))
    for x, yy in B.AUDIO_SCREWS:                                            # down into the plate, from inside
        p -= hw.clear(BIN["floor"], top=z0 + BIN["floor"]).moved(Pos(x, yy, 0))
    for x, z in FOOT_SCREWS:                                               # the uprights' screws: bosses inside
        for s in (1, -1):
            yo, yi = s * y, s * (y - t - 7.0)
            p += _ycyl(4.0, min(yo, yi), max(yo, yi), x, z)
            p -= hw.tap(10.0, top=0.0).moved(Pos(x, yo, z) * hw.rot_to("y" if s > 0 else "-y"))
    p.label = "bin"
    return p


def _upright_outline():
    """(x, z) outline of the +y upright: a foot on the bin's side, a strut beside the face, the shelf's seat."""
    fx = face_front_x
    return [(-128.0, 212.0), (-96.0, 212.0), (-96.0, 240.0), (fx(352.0) + 2.0, 352.0), (fx(SHELF["z"][0]), SHELF["z"][0]),
            (SHELF["x"][0], SHELF["z"][0]), (SHELF["x"][0], 430.0), (-166.0, 350.0), (-130.0, 238.0)]


@lru_cache(maxsize=None)
def upright(side: int) -> Part:
    y0, y1 = UPRIGHT_Y
    p = _xz_prism(_upright_outline(), y0, y1)
    p += _box((-128.0, -96.0), (BIN["y"], y0 + 0.5), (214.0, 238.0))          # the foot, against the bin's wall
    for x, z in FOOT_SCREWS + DP.seat_screws("arm"):                      # to the bin; the face's screws
        p -= _ycyl(D.CLR_M3 / 2, BIN["y"] - 1, y1 + 1, x, z)
    for x, y in SHELF_SCREWS:                                              # the shelf, from above
        if y > 0:
            p -= hw.tap(12.0, top=SHELF["z"][0], entry="top").moved(Pos(x, y, 0))
    p.label = "face_upright"
    if side < 0:
        p = p.mirror(Plane.XZ)
        p.label = "face_upright"
    return p


@lru_cache(maxsize=None)
def shelf() -> Part:
    x0, x1 = SHELF["x"]
    z0, z1 = SHELF["z"]
    p = _box((x0, x1), (-SHELF["y"], SHELF["y"]), (z0, z1))
    for x, y in SHELF_SCREWS:
        p -= hw.clear(z1 - z0, top=z1).moved(Pos(x, y, 0))
    for x, y in DP.audio_screws("shelf"):                                  # the audio box, from inside it
        p -= hw.tap(z1 - z0 + 0.1, top=z1).moved(Pos(x, y, 0))
    p -= _box((-150.0, -138.0), (-10.0, 10.0), (z0 - 1, z1 + 1))           # its USB cable down
    p.label = "audio_shelf"
    return p


# --- hardware ----------------------------------------------------------------------------------------------------------

def hardware() -> hw.Hardware:
    import vendor as V
    h = hw.Hardware()
    for x, y in RISER_SCREWS:                                              # riser -> plate (6 mm of it)
        h.screw("PAN", 10, Pos(x, y, D.PLATE_TOP + RISER["floor"]))
    for x, y in RISER_BOSSES:                                              # adapter -> riser bosses
        h.screw("PAN", 20, Pos(x, y, ADAPTER["top"] - D.CB_M3[1]))
    for x, y in B.AUDIO_SCREWS:                                            # bin -> plate
        h.screw("PAN", 10, Pos(x, y, D.PLATE_TOP + BIN["floor"]))
    for x, z in FOOT_SCREWS:                                               # uprights -> bin
        for s in (1, -1):
            h.screw("PAN", 20, Pos(x, s * UPRIGHT_Y[1], z) * hw.rot_to("y" if s > 0 else "-y"))
    for x, y in SHELF_SCREWS:                                              # shelf -> uprights
        h.screw("PAN", 16, Pos(x, y, SHELF["z"][1]))
    w, d, hh = 35.0, 25.0, 10.0
    yb = RISER["y"] + 4.0
    h.part("12 V buck (arm)", "12 V 5 A step-down for the SO-101's STS3215 bus (Pololu D36V50F12 class; the bus board "
           "rides on the arm's base)", "pololu.com #4096 (~$30)", V.pcb(w, d, hh, [(-15.0, 0.0), (15.0, 0.0)], "12 V buck",
                                                                       inset=6.0), Pos(BUCK["x"], yb, BUCK["z"]) * hw.rot_to("y"))
    for dx in (-15.0, 15.0):
        h.screw("PAN", 10, Pos(BUCK["x"] + dx, yb + 1.6, BUCK["z"]) * hw.rot_to("y"))
    return h


def printed() -> list[tuple[str, Part, str, int, str]]:
    return [("arm_riser", riser(), "PETG", 40, "floor down"),
            ("arm_adapter", adapter(), "PETG", 40, "flat"),
            ("bin", bin_(), "PETG", 20, "floor down"),
            ("face_upright_L", upright(1), "PETG", 40, "on its outer face"),
            ("face_upright_R", upright(-1), "PETG", 40, "on its outer face (mirror of L)"),
            ("audio_shelf", shelf(), "PETG", 30, "flat")]
