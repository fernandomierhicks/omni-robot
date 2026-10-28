"""The base, ROBOT frame: deck in the axle plane (design.md 20 round 2), payload plate, battery, electronics, bumpers.

Deck: a 140 mm centre piece and four lobes (front, left, rear, right) split at |x| or |y| = 70, where the deck is
still ~110 mm wide; the four wheel slots (141 mm wide, from r 88 out) leave only ~25 mm necks between the centre and
each lobe further out, so the joints sit inboard of the necks and the pod mounts and wheel houses stiffen the ring.
Joints: the front and rear ones under a 6 mm PETG splice plate (M3 into 2.8 mm holes in both pieces); the side ones
under the pod mounts, whose ties put two screws into the centre piece beside the five in the lobe (pod.TIE).
Payload plate: on six standoffs (dims.STANDOFF_XY): four printed, the front pair Ø8 aluminium rods (dims.ROD_XY). The
layer between the deck and the plate is the lidar's: its scan plane (150 mm) passes over the wheels at their bump stop
and under the plate, and only the standoffs cross it.
Things screwed down onto the centre deck stand on posts and bosses printed with it (tapped tops): the battery is right
under the centre deck, so no screw may come up from below there, and 6 mm of deck alone is too little thread.
"""

from __future__ import annotations

import math
from functools import lru_cache

from build123d import (Align, Box, Cylinder, Location, Part, Plane, Polygon, Pos, Rot, extrude, revolve, Axis)

import dims as D
import hw
import pod as P

CB = (Align.CENTER, Align.CENTER, Align.MIN)
CT = (Align.CENTER, Align.CENTER, Align.MAX)
SPLIT = 70.0
LOBES = {"front": 0.0, "left": 90.0, "rear": 180.0, "right": 270.0}


def pod_loc(th: float) -> Location:
    """POD -> ROBOT for wheel th, mirrored pods included (apply to shapes with pod_place)."""
    return P.pod_to_robot(th)


def pod_pt(th: float, x: float, y: float, z: float = 0.0):
    """A POD-frame point of wheel th in the ROBOT frame (mirror applied)."""
    if D.POD_MIRROR[th]:
        y = -y
    v = P.pod_to_robot(th) * Pos(x, y, z)
    return v.position.X, v.position.Y, v.position.Z


def pod_place(shape, th: float):
    """Place a POD-frame shape on wheel th (mirrored about the pod's x-z plane where needed)."""
    s = shape.mirror(Plane.XZ) if D.POD_MIRROR[th] else shape
    return s.moved(P.pod_to_robot(th))


# --- deck ---------------------------------------------------------------------------------------------------------------

def _deck_outline() -> Part:
    """The whole deck as one solid (before splitting): circle minus the four slots."""
    p = Cylinder(D.DECK_R, D.DECK_T, align=CB).moved(Pos(0, 0, D.DECK_Z))
    for th in D.WHEEL_ANGLES:     # T-slots: narrow where only the arm and cup pass, full width at the wheel
        loc = Location((0, 0, D.DECK_Z), (0, 0, th))
        p -= Box(400, 2 * D.SLOT_HW, 40, align=(Align.MIN, Align.CENTER, Align.CENTER)).moved(loc * Pos(D.SLOT_R1, 0, 0))
        p -= Box(400, 2 * D.SLOT_HW_IN, 40, align=(Align.MIN, Align.CENTER, Align.CENTER)).moved(loc * Pos(D.SLOT_R0, 0, 0))
    return p


def _deck_holes() -> list[tuple[float, float, float, str]]:
    """(x, y, dia, what) through the deck: pod mounts and wheel-house feet (screws from below), e-stop, leads."""
    holes = []
    for th in D.WHEEL_ANGLES:
        for x, y in P.MOUNT_SCREWS:
            px, py, _ = pod_pt(th, x, y)
            holes.append((px, py, D.CLR_M3, "pod mount"))
        for x, s in P.guard_feet(th in P.FRONT):
            px, py, _ = pod_pt(th, x, s * (D.HOUSE_Y + 4))
            holes.append((px, py, D.CLR_M3, "wheel guard"))
    for x, y in D.STANDOFF_XY:
        holes.append((x, y, D.CLR_M3, "standoff"))
    holes.append((*ESTOP_XY, ESTOP_DECK_HOLE, "e-stop contact block"))
    holes.append((-95.0, 0.0, 16.0, "battery leads"))
    for x, y in LIDAR_SCREWS:        # a driver reaches the lidar's M2.5 screws from under the deck, up the riser
        holes.append((x, y, 7.0, "lidar screw access"))
    return holes


SPLICE_SCREWS = [(-45.0, -SPLIT + 8), (-45.0, SPLIT + 8), (-15.0, -SPLIT + 8), (-15.0, SPLIT + 8),
                 (15.0, -SPLIT + 8), (15.0, SPLIT + 8), (45.0, -SPLIT + 8), (45.0, SPLIT + 8)]
# ^ in a "joint frame": u along the joint (x), v across it; the joint is at v = SPLIT; 4 screws each side


def _splice_pts(lobe_deg: float):
    """Screw points (x, y) of the splice over the joint toward lobe_deg (two rows, 8 mm each side of the split)."""
    a = math.radians(lobe_deg)
    c, s = math.cos(a), math.sin(a)
    out = []
    for u in (-24.0, -8.0, 8.0, 24.0):
        for v in (SPLIT - 9.0, SPLIT + 9.0):
            out.append((v * c - u * s, v * s + u * c))
    return out


def _lobe_region(name: str) -> Part:
    a = LOBES[name]
    pts = [(SPLIT, -SPLIT), (400.0, -400.0), (400.0, 400.0), (SPLIT, SPLIT)]     # x > SPLIT, between the diagonals
    region = extrude(Plane.XY.offset(D.DECK_Z - 1) * Polygon(*pts, align=None), D.DECK_T + 2, dir=(0, 0, 1))
    return region.rotate(Axis.Z, a)


def _piece_of(x: float, y: float) -> str:
    """Which deck piece (x, y) lies on."""
    if abs(x) <= SPLIT and abs(y) <= SPLIT:
        return "centre"
    a = math.degrees(math.atan2(y, x)) % 360
    return min(LOBES, key=lambda n: abs((a - LOBES[n] + 180) % 360 - 180))


def _drill(p: Part, holes) -> Part:
    for x, y, dia, _ in holes:
        p -= Cylinder(dia / 2, 40).moved(Pos(x, y, D.DECK_Z))
    return p


@lru_cache(maxsize=None)
def deck_piece(name: str) -> Part:
    """'centre' or a lobe name. ROBOT frame."""
    whole = _deck_outline()
    if name == "centre":
        p = whole & Box(2 * SPLIT, 2 * SPLIT, 60).moved(Pos(0, 0, D.DECK_Z))
    else:
        p = whole & _lobe_region(name)
    p = _drill(p, _deck_holes())
    for lobe in SPLICED:                                        # splice screws
        for x, y in _splice_pts(LOBES[lobe]):
            p -= hw.tap(D.DECK_T + 0.1, top=D.DECK_TOP).moved(Pos(x, y, 0))
    for lobe in LOBES.values():                                 # lobe bumpers' lip screws
        c, s = math.cos(math.radians(lobe)), math.sin(math.radians(lobe))
        for x, y in bumper_screws_lobe():
            p -= hw.tap(D.DECK_T + 0.1, top=D.DECK_TOP).moved(Pos(x * c - y * s, x * s + y * c, 0))
    for x, y in TAPPED_TOP:                                     # things screwed down onto the deck
        p -= hw.tap(D.DECK_T + 0.1, top=D.DECK_TOP).moved(Pos(x, y, 0))
    for x, y in TAPPED_BOTTOM:                                  # things screwed up under the deck
        p -= hw.tap(D.DECK_T + 0.1, top=D.DECK_TOP).moved(Pos(x, y, 0))
    for x, y in D.ROD_XY:                                       # the front rods' locating recesses
        p -= Cylinder(D.ROD_D / 2 + 0.2, D.ROD_RECESS + 0.01, align=CT).moved(Pos(x, y, D.DECK_TOP + 0.01))
    for x, y, r, h in DECK_POSTS:                               # printed posts / bosses with tapped tops
        if _piece_of(x, y) == name:
            p += Cylinder(r, h, align=CB).moved(Pos(x, y, D.DECK_TOP))
            p += hw.cove_ring(r, 2.0).moved(Pos(x, y, D.DECK_TOP))                 # fillet at the foot
            p -= hw.tap(h + D.DECK_T, top=D.DECK_TOP + h, entry="top").moved(Pos(x, y, 0))
    s = 0.5 * math.sqrt(2)              # 0.5 mm first-layer chamfers on the joint edges (the pieces print flat, and
    for v in (-SPLIT, SPLIT):           # a squashed first layer would keep the joints from closing)
        p -= Box(s, 2 * SPLIT + 2, s).rotate(Axis.Y, 45).moved(Pos(v, 0, D.DECK_Z))
        p -= Box(2 * SPLIT + 2, s, s).rotate(Axis.X, 45).moved(Pos(0, v, D.DECK_Z))
    p.label = f"deck_{name}"
    return p


SPLICE_T = 6.0
BUCK19_POSTS, BUCK19_POST_H = ((SPLIT, 15.0), (SPLIT, -15.0)), 6.0    # under the plate, behind the lidar, 10 mm short
                                                                         # of the Jetson; out of the scan plane


@lru_cache(maxsize=None)
def splice() -> Part:
    """6 mm PETG splice plate over one centre-lobe joint, modelled for the front joint (x = SPLIT). Pan heads on top."""
    p = Box(32.0, 62.0, SPLICE_T, align=CB).moved(Pos(SPLIT, 0, D.DECK_TOP))
    for x, y in _splice_pts(0.0):
        p -= hw.clear(SPLICE_T, top=D.DECK_TOP + SPLICE_T).moved(Pos(x, y, 0))
    for x, y in BUCK19_POSTS:              # the Jetson's 19 V buck on two posts, between the screws' heads (on the
        p += Cylinder(3.5, BUCK19_POST_H, align=CB).moved(Pos(x, y, D.DECK_TOP + SPLICE_T))      # rear splice too:
        p -= hw.tap(BUCK19_POST_H + 2.5, top=D.DECK_TOP + SPLICE_T + BUCK19_POST_H, entry="top").moved(Pos(x, y, 0))  # unused)
    p.label = "deck_splice"
    return p


# --- electronics layout on the deck (ROBOT frame) -------------------------------------------------------------------------

# The deck between the slots is small (the rear lobe is ~40 mm wide at its neck), so: the power board and BMS sit
# under the Jetson, which stands on 16 mm posts; the MCU board (with the BNO085 on it, design.md 7) under the rear lobe
# and the e-stop on it; the two bucks on yoke L (mounts.py). The display, speaker and microphones are in the display
# housing on the payload plate (display.py).
JETSON_XY = (0.0, 0.0)
JETSON_HOLES = [(x, y) for x in (-43.0, 43.0) for y in (-29.0, 29.0)]      # (VERIFY on the carrier)
JETSON_POST_H = 16.0
BOSS_H = 4.0                                                               # board bosses (the boards' undersides need air)
LIDAR_XY = (140.0, 0.0)                                                    # the sim's 0.14 m
LIDAR_HOLE_R = D.A2M12["holes"][2] / 2
LIDAR_SCREWS = [(LIDAR_XY[0] + LIDAR_HOLE_R * math.cos(math.radians(a)), LIDAR_HOLE_R * math.sin(math.radians(a)))
                for a in (45, 135, 225, 315)]
LIDAR_FOOT_R = 43.0              # riser feet: outside the riser's wall (r 36) and the lidar (r 38), so a driver
                                 # reaches them with the lidar on (at r 38 the riser's own wall covered them)
LIDAR_FOOT_ANGLES = (45, 160, 200, 315)       # the inner pair clear of the front standoffs (under the plate's edge:
LIDAR_FEET = [(LIDAR_XY[0] + LIDAR_FOOT_R * math.cos(math.radians(a)), LIDAR_FOOT_R * math.sin(math.radians(a)))
              for a in LIDAR_FOOT_ANGLES]   # they go in before the plate)
# e-stop: a 4 mm panel on a hollow riser. The e-stop is built into the riser on the bench (head through the panel,
# ring nut and contact block from the open underside), then the riser goes onto the deck with the contact block
# through a 30 mm hole (VERIFY the contact block's size). The 20 mm tube it had left no room for the ring nut.
ESTOP_XY, ESTOP_RISER = (-168.0, 0.0), 20.0
ESTOP_R, ESTOP_CAVITY_R, ESTOP_PANEL = 21.0, 17.0, 4.0
ESTOP_DECK_HOLE = 30.0
ESTOP_SCREWS = [(ESTOP_XY[0], ESTOP_XY[1] + 27.0 * s) for s in (-1, 1)]
SPLICED = ("front", "rear")
# name: (description, source, x, y, z, w, d, h, holes, top side up?)
BOARDS = {
    "power board": ("power board: e-stop MOSFET switch, 40 A fuse, XT60 in/out, CAN + 24 V to the payload (TBD)",
                    "custom / TBD", 0.0, 22.0, D.DECK_TOP, 60.0, 38.0, 14.0, [(dx, dy) for dx in (-26, 26) for dy in (-15, 15)], True),
    "BMS": ("7S 40 A BMS (Daly-class, est 60 x 38 x 12)", "TBD (VERIFY size)", 0.0, -22.0, D.DECK_TOP, 60.0, 38.0, 12.0,
            [(dx, dy) for dx in (-26, 26) for dy in (-15, 15)], True),
    "MCU board": ("MCU carrier: STM32G474, 2 x CAN-FD, UARTs, BNO085 (Adafruit #4754) on it; design.md 4 (TBD), under the deck",
                  "custom / TBD", -130.0, 0.0, D.DECK_Z, 40.0, 30.0, 14.0, [(dx, dy) for dx in (-16, 16) for dy in (-11, 11)], False),
}
# (x, y, radius, height): printed with the deck, tapped from the top (Jetson posts; bosses under the boards on top)
DECK_POSTS = ([(JETSON_XY[0] + x, JETSON_XY[1] + y, 4.5, JETSON_POST_H) for x, y in JETSON_HOLES] +
              [(b[2] + dx, b[3] + dy, 3.5, BOSS_H) for b in BOARDS.values() if b[9] for dx, dy in b[8]])
TAPPED_TOP = LIDAR_FEET + ESTOP_SCREWS
BATTERY_TRAY_SCREWS = [(-83.0, -16.0), (-83.0, 16.0), (83.0, -16.0), (83.0, 16.0)]   # between the splice's rows
TAPPED_BOTTOM = BATTERY_TRAY_SCREWS + [(b[2] + dx, b[3] + dy) for b in BOARDS.values() if not b[9] for dx, dy in b[8]]


@lru_cache(maxsize=None)
def board_post() -> Part:
    """4 mm printed spacer under a board hung below the deck (the MCU). Base at z = 0."""
    p = Cylinder(3.5, 4.0, align=CB) - hw.clear(5.0, top=4.0)
    p.label = "board_spacer"
    return p


@lru_cache(maxsize=None)
def standoff() -> Part:
    """Printed payload standoff, deck top to plate underside: a Ø12 column with Ø20 end flanges, 3 mm fillets into
    them, tapped 12 deep at each end (an M3 pan head up through the deck, an M3 flat head down through the plate).
    Base at z = 0. Print standing, 100 % infill. In bending (the upper robot's sway) the root is the weak spot, hence
    the fillets; axially a Ø12 PETG column holds > 2 kN against buckling."""
    h = D.PLATE_BOT - D.DECK_TOP
    fd, ft = D.STANDOFF_FLANGE
    r = D.STANDOFF_D / 2
    p = Cylinder(r, h, align=CB) + Cylinder(fd / 2, ft, align=CB) + Cylinder(fd / 2, ft, align=CT).moved(Pos(0, 0, h))
    p += hw.cove_ring(r, D.FILLET).moved(Pos(0, 0, ft)) + hw.cove_ring(r, D.FILLET).moved(Pos(0, 0, h - ft) * Rot(180, 0, 0))
    p -= hw.tap(12.0, top=12.0, entry="bottom") + hw.tap(12.0, top=h, entry="top")
    p -= hw.edge_chamfer(fd / 2, 0.5, 0.0, up=False)                       # first layer
    p.label = "standoff"
    return p


@lru_cache(maxsize=None)
def rod() -> Part:
    """The front pair's standoff: 6061 rod Ø8, cut to the gap + both recesses, M3 tapped 10 deep at each end (2.5 mm
    drill). Base at z = 0."""
    h = D.PLATE_BOT - D.DECK_TOP + 2 * D.ROD_RECESS
    p = Cylinder(D.ROD_D / 2, h, align=CB)
    p -= Cylinder(1.25, 10.0, align=CB).moved(Pos(0, 0, -0.01)) + Cylinder(1.25, 10.0, align=CT).moved(Pos(0, 0, h + 0.01))
    p.label = "standoff rod"
    return p


@lru_cache(maxsize=None)
def estop_riser() -> Part:
    """Raises the e-stop 20 mm so its contact block ends 40 mm above the floor: a 4 mm panel for the e-stop (22.5 mm
    hole) on a hollow body (the ring nut and the contact block go on from below, on the bench), a 4 mm flange with
    two M3 into the deck. ROBOT frame."""
    x, y = ESTOP_XY
    p = Cylinder(ESTOP_R, ESTOP_RISER, align=CB).moved(Pos(x, y, D.DECK_TOP))
    p += Box(14.0, 66.0, 4.0, align=CB).moved(Pos(x, y, D.DECK_TOP))
    p -= Cylinder(ESTOP_CAVITY_R, ESTOP_RISER - ESTOP_PANEL + 0.01, align=CB).moved(Pos(x, y, D.DECK_TOP - 0.01))
    p -= Cylinder(11.25, 40).moved(Pos(x, y, D.DECK_TOP))
    for sx, sy in ESTOP_SCREWS:
        p -= hw.clear(4.0, top=D.DECK_TOP + 4.0).moved(Pos(sx, sy, 0))
    p.label = "estop_riser"
    return p


@lru_cache(maxsize=None)
def lidar_riser() -> Part:
    """Raises the A2M12 so its scan plane is at LIDAR_SCAN_Z (over the wheels at their bump stop, under the plate).
    ROBOT frame."""
    h = lidar_base_z() - D.DECK_TOP
    x0, y0 = LIDAR_XY
    p = Cylinder(D.A2M12["d"] / 2 - 2, h, align=CB).moved(Pos(x0, y0, D.DECK_TOP))
    p -= Cylinder(LIDAR_HOLLOW_R, h - 4.0, align=CB).moved(Pos(x0, y0, D.DECK_TOP - 0.01))         # hollow, 4 mm top
    p -= Cylinder(10.0, h + 2).moved(Pos(x0, y0, D.DECK_TOP + h))                                  # cable up
    p -= Box(30, 24, h - 10, align=(Align.MAX, Align.CENTER, Align.MIN)).moved(Pos(x0 - 20, y0, D.DECK_TOP))  # cable out
    for a, (x, y) in zip(LIDAR_FOOT_ANGLES, LIDAR_FEET):                                          # feet on lugs
        lug = Box(LIDAR_FOOT_R - 28.0, 12.0, 4.0, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
            Pos(x0, y0, D.DECK_TOP) * Rot(0, 0, a) * Pos(28.0, 0, 0)) + Cylinder(6.0, 4.0, align=CB).moved(Pos(x, y, D.DECK_TOP))
        p += lug
        p -= hw.clear(4.0, top=D.DECK_TOP + 4.0).moved(Pos(x, y, 0))
    for x, y in LIDAR_SCREWS:                                                                       # lidar's 4 x M2.5
        p -= hw.clear(h + 1, top=D.DECK_TOP + h, d=D.CLR_M25).moved(Pos(x, y, 0))
    p.label = "lidar_riser"
    return p


LIDAR_HOLLOW_R = 28.0            # the M2.5 screws (r 25) are driven up from under the deck through this hollow


def lidar_base_z() -> float:
    return D.LIDAR_SCAN_Z - D.A2M12["scan_z"]


# --- battery ----------------------------------------------------------------------------------------------------------------

@lru_cache(maxsize=None)
def battery_tray() -> Part:
    """Cradle under the centre deck for the flat 7S1P pack (153 x 72 x 23): a 3 mm box closed by the deck, the pack
    captured between them (no straps). ROBOT frame. Its floor is the robot's belly (~35 mm up). The 2 mm walls and
    the strap slots that cut the floor through left it 1 mm thin."""
    b = D.BATTERY
    t = 3.0
    z_top = D.DECK_Z
    p = Box(b["l"] + 2 * t, b["w"] + 2 * t, b["h"] + t, align=CT).moved(Pos(0, 0, z_top))
    p -= Box(b["l"], b["w"], b["h"] + 1, align=CT).moved(Pos(0, 0, z_top + 0.01))
    p += Box(176.0, 64.0, 4.0, align=CT).moved(Pos(0, 0, z_top)) - Box(b["l"], b["w"], 10).moved(Pos(0, 0, z_top))
    p -= Box(20.0, 24.0, b["h"] - 4.0, align=CB).moved(Pos(-b["l"] / 2, 0, z_top - b["h"]))   # leads out the -x end (deck
                                                                   # hole at x -95), under the flange and its screws
    for x, y in BATTERY_TRAY_SCREWS:
        p -= hw.clear(5.0, top=z_top).moved(Pos(x, y, 0))
    p.label = "battery_tray"
    return p


# --- payload plate ----------------------------------------------------------------------------------------------------------

def plate_screws():
    """ROBOT (x, y) of the payload plate's screws into the six standoffs (flat heads, flush with the top)."""
    return list(D.STANDOFF_XY)


PIN_XY = ((87.5, 37.5), (-37.5, -62.5))           # 4 mm dowels: a module fits only one way (kept from v2)
YOKE_SCREWS_X = (-40.0, -4.0, 32.0)                # the yokes' flange screws (mounts.py), YOKE_SCREW_IN inboard of
YOKE_SCREW_IN = 5.0                                # the yokes' inner faces
AUDIO_SCREWS = ((-80.0, 30.0), (-80.0, -30.0), (-106.0, 22.0), (-106.0, -22.0))     # the audio box (display.py)
ARM_SCREWS = ((105.0, 25.0), (105.0, -25.0))      # the arm kit's riser, beside two grid holes (arm_kit.py)


def head_screws():
    """The head's cheek pads (head.py), both sides."""
    import head as H
    return [(x, s * y) for x, y in H.HEAD_SCREWS for s in (1, -1)]
THUMBSCREW_XY = ((50.0, 50.0), (50.0, -50.0), (-50.0, 50.0), (-50.0, -50.0))


@lru_cache(maxsize=None)
def payload_plate() -> Part:
    """PAYLOAD contract kept from v2 (25 mm grid, two dowels, centre pass-through), now M3 on 2.8 mm holes. A full
    disc: it rides above the wheels and the lidar. ROBOT frame, z = PLATE_BOT .. PLATE_TOP."""
    p = Cylinder(D.PLATE_R, D.PLATE_T, align=CB).moved(Pos(0, 0, D.PLATE_BOT))
    for x, y in plate_screws():
        p -= hw.csink(D.PLATE_T, top=D.PLATE_TOP).moved(Pos(x, y, 0))
    for x, y in D.ROD_XY:                              # the front rods' locating recesses
        p -= Cylinder(D.ROD_D / 2 + 0.2, D.ROD_RECESS + 0.01, align=CB).moved(Pos(x, y, D.PLATE_BOT - 0.01))
    head = head_screws()
    for i in range(-4, 5):
        for j in range(-4, 5):
            x, y = 25.0 * i, 25.0 * j
            r = math.hypot(x, y)
            if r > 98 or r < 20:
                continue
            if any(math.hypot(x - sx, y - sy) < 9 for sx, sy in plate_screws() + head + list(AUDIO_SCREWS) + list(ARM_SCREWS)):
                continue
            p -= hw.tap(D.PLATE_T + 0.1, top=D.PLATE_TOP).moved(Pos(x, y, 0))
    for x in YOKE_SCREWS_X:
        for s in (1, -1):
            p -= hw.tap(D.PLATE_T + 0.1, top=D.PLATE_TOP).moved(Pos(x, s * (D.YOKE_Y - YOKE_SCREW_IN), 0))
    for x, y in head + list(AUDIO_SCREWS) + list(ARM_SCREWS):
        p -= hw.tap(D.PLATE_T + 0.1, top=D.PLATE_TOP).moved(Pos(x, y, 0))
    for x, y in PIN_XY:
        p -= Cylinder(3.95 / 2, 20).moved(Pos(x, y, D.PLATE_BOT))
    p -= Cylinder(14.0, 20).moved(Pos(0, 0, D.PLATE_BOT))
    p.label = "payload_plate"
    return p


# --- bumpers ------------------------------------------------------------------------------------------------------------------

BUMPER_Z = (42.0, 95.0)
SKIRT_Z = 20.0


def _bumper_profile():
    """(r, z) outline: a hollow D, 2.5 mm TPU walls, outer face at DECK_R + BUMPER_T, plus a 1.5 mm skirt to 20 mm."""
    r0, r1 = D.DECK_R + 0.2, D.DECK_R + D.BUMPER_T
    z0, z1 = BUMPER_Z
    w = 2.5
    return [(r0, z0), (r1 - 4, z0), (r1 - 4 + 1.5, z0 - 0.01), (r1 - 4 + 1.5, SKIRT_Z), (r1 - 4, SKIRT_Z), (r1 - 4, z0 - 0.0)]


def _arc(r0, r1, z0, z1, a0, a1) -> Part:
    pts = [(r0, z0), (r1, z0), (r1, z1), (r0, z1)]
    return revolve(Plane.XZ * Polygon(*pts, align=None), Axis.Z, a1 - a0).rotate(Axis.Z, a0)


def bumper_span(kind: str):
    """(start, end) angles of one bumper arc centred at 0: 'lobe' between two slots, 'corner' over a wheel house."""
    half_slot = math.degrees(math.asin((D.HOUSE_Y + 6) / D.DECK_R))
    return (-(45 - half_slot), 45 - half_slot) if kind == "lobe" else (-half_slot, half_slot)


@lru_cache(maxsize=None)
def bumper(kind: str) -> Part:
    """TPU 95A bumper arc, modelled centred on angle 0. 'lobe' arcs screw down through an inner lip onto the deck;
    'corner' arcs screw radially into the wheel house's front panel. The 1.5 mm skirt reaches 20 mm above the floor
    (design.md 23.4)."""
    a0, a1 = bumper_span(kind)
    r0, r1 = D.DECK_R + 0.2, D.DECK_R + D.BUMPER_T
    z0, z1 = BUMPER_Z
    shell = _arc(r0, r1, z0, z1, a0, a1) - _arc(r0 + 2.5, r1 - 2.5, z0 + 2.5, z1 - 2.5, a0 - 1, a1 + 1)
    shell += _arc(r1 - 4.0, r1 - 2.5, SKIRT_Z, z0 + 0.5, a0, a1)                   # skirt
    if kind == "lobe":
        shell += _arc(D.DECK_R - 14.0, r0 + 0.5, D.DECK_TOP, D.DECK_TOP + 3.0, a0 + 2, a1 - 2)  # lip on the deck
        shell -= _arc(r0 - 0.1, r0 + 2.6, D.DECK_Z - 0.2, D.DECK_TOP + 0.2, a0 - 1, a1 + 1) - _arc(0, 0.01, 0, 0.01, 0, 1)
        for a in _lin(a0 + 5, a1 - 5, 3):
            x, y = D.polar(D.DECK_R - 7.0, a)
            shell -= hw.clear(3.2, top=D.DECK_TOP + 3.0).moved(Pos(x, y, 0))
    else:
        for a in (a0 + 4, 0.0, a1 - 4):
            for z in (55.0, 85.0):
                x, y = D.polar(D.DECK_R, a)
                shell -= Cylinder(D.CLR_M3 / 2, 8, rotation=(0, 90, 0)).moved(Location((x, y, z), (0, 0, a)))
                x2, y2 = D.polar(r1 - 2.5, a)
                shell -= Cylinder(3.5, 6, rotation=(0, 90, 0)).moved(Location((x2, y2, z), (0, 0, a)))
    shell.label = f"bumper_{kind}"
    return shell


def bumper_screws_lobe():
    a0, a1 = bumper_span("lobe")
    return [D.polar(D.DECK_R - 7.0, a) for a in _lin(a0 + 5, a1 - 5, 3)]


def bumper_screws_corner():
    a0, a1 = bumper_span("corner")
    return [(a, z) for a in (a0 + 4, 0.0, a1 - 4) for z in (55.0, 85.0)]


def _lin(a, b, n):
    return [a + (b - a) * k / (n - 1) for k in range(n)]


# --- hardware ------------------------------------------------------------------------------------------------------------

def hardware() -> hw.Hardware:
    """Base hardware in the ROBOT frame (the pods' own is in pod.py / wheel.py, the module's in module.py)."""
    import vendor as V
    h = hw.Hardware()
    # deck splices (front and rear joints; the pod mounts span the side joints)
    for lobe in SPLICED:             # M3 x 12: all 6 mm of the deck
        for x, y in _splice_pts(LOBES[lobe]):
            h.screw("PAN", 12, Pos(x, y, D.DECK_TOP + SPLICE_T))
    # pod mounts and wheel guards: screws up from under the deck (pod mounts: 8 mm into the mount)
    for th in D.WHEEL_ANGLES:
        for x, y in P.MOUNT_SCREWS:
            px, py, _ = pod_pt(th, x, y)
            h.screw("PAN", 14, Pos(px, py, D.DECK_Z) * hw.rot_to("-z"))
        for x, s in P.guard_feet(th in P.FRONT):              # M3 x 14: 7.8 mm into the 8 mm foot
            px, py, _ = pod_pt(th, x, s * (D.HOUSE_Y + 4))
            h.screw("PAN", 14, Pos(px, py, D.DECK_Z) * hw.rot_to("-z"))
    # payload plate on its six standoffs: pan heads up through the deck, flat heads down through the plate
    for x, y in D.STANDOFF_XY:
        h.screw("PAN", 14, Pos(x, y, D.DECK_Z) * hw.rot_to("-z"))
        h.screw("FLAT", 14, Pos(x, y, D.PLATE_TOP))
    for x, y in D.ROD_XY:
        h.part("standoff rod 8 x 98", "6061 aluminium rod Ø8 x 98 mm, M3 tapped 10 deep at both ends (front standoffs; "
               "or Ø8 M3 female round standoffs of that length, VERIFY)", "McMaster 8974K (6061 rod, 8 mm) + M3 tap",
               rod(), Pos(x, y, D.DECK_TOP - D.ROD_RECESS))
    for x, y in PIN_XY:          # flush with the plate's underside (the front one is next to a standoff)
        hw.pin(h, 4.0, 16.0, Pos(x, y, D.PLATE_BOT + 8.0), "4 mm dowel pin x 16 (module keying), pressed into the plate",
               "McMaster 91585A (4 mm x 16 dowel)")
    # battery, its tray, straps
    h.part("battery 7S1P P50B", "7S1P Molicel P50B 21700 pack (5 Ah, 25.2 V), flat row, wrapped, XT60 + balance lead",
           "Molicel INR21700-P50B x 7 + nickel strip / pack builder (~$90)", V.battery(), Pos(0, 0, D.DECK_Z))
    for x, y in BATTERY_TRAY_SCREWS:
        h.screw("PAN", 10, Pos(x, y, D.DECK_Z - 4.0) * hw.rot_to("-z"))
    # Jetson on the deck's posts, boards on its bosses (M3 x 12 through carrier and foot: 6.4 mm into the post)
    h.part("Jetson Orin Nano Super dev kit", "NVIDIA Jetson Orin Nano Super developer kit (design.md 4)", "NVIDIA / Arrow ($399 est)",
           V.jetson(), Pos(*JETSON_XY, D.DECK_TOP + JETSON_POST_H))
    for x, y in JETSON_HOLES:
        h.screw("PAN", 12, Pos(JETSON_XY[0] + x, JETSON_XY[1] + y, D.DECK_TOP + JETSON_POST_H + 4.0 + 1.6))
    for name, (desc, src, x, y, z, w, d, hh, holes, up) in BOARDS.items():
        if up:
            h.part(name, desc, src, V.pcb(w, d, hh, holes, name, inset=7.0), Pos(x, y, z + BOSS_H))
            for dx, dy in holes:
                h.screw("PAN", 10, Pos(x + dx, y + dy, z + BOSS_H + 1.6))
        else:     # hangs under the deck on spacers, face down (M3 x 12: all 6 mm of the deck)
            h.part(name, desc, src, V.pcb(w, d, hh, holes, name, inset=7.0), Pos(x, y, z - 4.0) * Rot(180, 0, 0))
            for dx, dy in holes:
                h.screw("PAN", 12, Pos(x + dx, y + dy, z - 4.0 - 1.6) * hw.rot_to("-z"))
    # the Jetson's 19 V buck on the front splice's posts (a base part: it stays when the payload kits are swapped)
    zb = D.DECK_TOP + SPLICE_T + BUCK19_POST_H
    h.part("19 V buck", "19 V 4 A step-down for the Jetson dev kit (est)", "TBD (~$15)",
           V.pcb(25.0, 35.0, 10.0, [(0.0, -15.0), (0.0, 15.0)], "19 V buck", inset=6.0), Pos(SPLIT, 0, zb))
    for x, y in BUCK19_POSTS:
        h.screw("PAN", 10, Pos(x, y, zb + 1.6))
    # lidar on its riser
    zl = lidar_base_z()
    h.part("RPLIDAR A2M12", "Slamtec RPLIDAR A2M12 (design.md 7, Fernando's choice)", "Slamtec / robotshop (~$229)",
           V.a2m12(), Pos(*LIDAR_XY, zl))
    for x, y in LIDAR_SCREWS:        # up through the riser's top (driver through the deck's access holes)
        h.screw("PAN", 8, Pos(x, y, zl - 4.0) * hw.rot_to("-z"), size="M2.5")
    for x, y in LIDAR_FEET:          # heads on the 4 mm lugs, all 6 mm of the deck
        h.screw("PAN", 10, Pos(x, y, D.DECK_TOP + 4.0))
    # e-stop on its riser
    h.part("e-stop 22 mm", "22 mm mushroom e-stop, 1 NC contact, 40 mm head, ring nut (design.md 14)", "e.g. IDEC / generic ($12)",
           V.estop(ESTOP_PANEL), Pos(*ESTOP_XY, D.DECK_TOP + ESTOP_RISER))
    for x, y in ESTOP_SCREWS:
        h.screw("PAN", 10, Pos(x, y, D.DECK_TOP + 4.0))
    # bumpers
    for lobe in LOBES.values():
        for x, y in bumper_screws_lobe():
            c, s = math.cos(math.radians(lobe)), math.sin(math.radians(lobe))
            h.screw("PAN", 10, Pos(x * c - y * s, x * s + y * c, D.DECK_TOP + 3.0))
    for th in D.WHEEL_ANGLES:
        for a, z in bumper_screws_corner():
            h.screw("PAN", 10, Location((0, 0, z), (0, 0, th + a)) * Pos(D.DECK_R + 2.8, 0, 0) * hw.rot_to("x"))
    return h
