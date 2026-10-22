"""The base, ROBOT frame: deck in the axle plane (design.md 20 round 2), payload plate, battery, electronics, bumpers.

Deck: a 140 mm centre piece and four lobes (front, left, rear, right) split at |x| or |y| = 70, where the deck is
still ~110 mm wide; the four wheel slots (141 mm wide, from r 88 out) leave only ~25 mm necks between the centre and
each lobe further out, so the splices sit inboard of the necks and the pod mounts and wheel houses stiffen the ring.
Splices: a 4 mm PETG plate on top of each joint, M3 into 2.8 mm holes in both pieces.
Payload plate: on six printed standoffs (dims.STANDOFF_XY). The layer between the deck and the plate is the lidar's:
its scan plane (150 mm) passes over the wheels at their bump stop and under the plate, and only the standoffs cross it.
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
    holes.append((*ESTOP_XY, 22.5, "e-stop"))
    holes.append((-95.0, 0.0, 16.0, "battery leads"))
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
    for x, y, r, h in DECK_POSTS:                               # printed posts / bosses with tapped tops
        if _piece_of(x, y) == name:
            p += Cylinder(r, h, align=CB).moved(Pos(x, y, D.DECK_TOP))
            p -= hw.tap(h + D.DECK_T, top=D.DECK_TOP + h).moved(Pos(x, y, 0))
    p.label = f"deck_{name}"
    return p


@lru_cache(maxsize=None)
def splice() -> Part:
    """4 mm PETG splice plate over one centre-lobe joint, modelled for the front joint (x = SPLIT)."""
    p = Box(32.0, 62.0, 4.0, align=CB).moved(Pos(SPLIT, 0, D.DECK_TOP))
    for x, y in _splice_pts(0.0):
        p -= hw.cbore(4.0, top=D.DECK_TOP + 4.0, cb=(6.0, 0.01)).moved(Pos(x, y, 0))
    p.label = "deck_splice"
    return p


# --- electronics layout on the deck (ROBOT frame) -------------------------------------------------------------------------

# The deck between the slots is small (the rear lobe is ~40 mm wide at its neck), so: the power board and BMS sit
# under the Jetson, which stands on 16 mm posts; the MCU board (with the BNO085 on it, design.md 7), the e-stop and the
# ELRS receiver on the rear lobe; the speaker under the rear lobe facing down; the two bucks on yoke L (mounts.py).
JETSON_XY = (0.0, 0.0)
JETSON_HOLES = [(x, y) for x in (-43.0, 43.0) for y in (-29.0, 29.0)]      # (VERIFY on the carrier)
JETSON_POST_H = 16.0
BOSS_H = 4.0                                                               # board bosses (the boards' undersides need air)
LIDAR_XY = (140.0, 0.0)                                                    # the sim's 0.14 m
LIDAR_HOLE_R = D.A2M12["holes"][2] / 2
LIDAR_FEET = [(LIDAR_XY[0] + 38.0 * math.cos(math.radians(a)), 38.0 * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]
ESTOP_XY, ESTOP_RISER = (-175.0, 0.0), 20.0
SPLICED = ("front", "rear")
# name: (description, source, x, y, z, w, d, h, holes, top side up?)
BOARDS = {
    "power board": ("power board: e-stop MOSFET switch, 40 A fuse, XT60 in/out, CAN + 24 V to the payload (TBD)",
                    "custom / TBD", 0.0, 22.0, D.DECK_TOP, 60.0, 38.0, 14.0, [(dx, dy) for dx in (-26, 26) for dy in (-15, 15)], True),
    "BMS": ("7S 40 A BMS (Daly-class, est 60 x 38 x 12)", "TBD (VERIFY size)", 0.0, -22.0, D.DECK_TOP, 60.0, 38.0, 12.0,
            [(dx, dy) for dx in (-26, 26) for dy in (-15, 15)], True),
    "MCU board": ("MCU carrier: STM32G474, 2 x CAN-FD, UARTs, BNO085 (Adafruit #4754) on it; design.md 4 (TBD), under the deck",
                  "custom / TBD", -142.0, 0.0, D.DECK_Z, 40.0, 30.0, 14.0, [(dx, dy) for dx in (-16, 16) for dy in (-11, 11)], False),
    "speaker": ("28 mm 8 ohm speaker + MAX98357A I2S amp", "Adafruit #4227 / #3006 (~$10)", -138.0, 0.0,
                D.DECK_TOP, 30.0, 30.0, 14.0, [(-13.0, 0.0), (13.0, 0.0)], True),        # clear of the MCU's holes below
}
# (x, y, radius, height): printed with the deck, tapped from the top (Jetson posts; bosses under the boards on top)
DECK_POSTS = ([(JETSON_XY[0] + x, JETSON_XY[1] + y, 4.5, JETSON_POST_H) for x, y in JETSON_HOLES] +
              [(b[2] + dx, b[3] + dy, 3.5, BOSS_H) for b in BOARDS.values() if b[9] for dx, dy in b[8]])
TAPPED_TOP = (LIDAR_FEET + [(ESTOP_XY[0], ESTOP_XY[1] + 18.0 * s) for s in (-1, 1)])
BATTERY_TRAY_SCREWS = [(-83.0, -24.0), (-83.0, 24.0), (83.0, -24.0), (83.0, 24.0)]
TAPPED_BOTTOM = BATTERY_TRAY_SCREWS + [(b[2] + dx, b[3] + dy) for b in BOARDS.values() if not b[9] for dx, dy in b[8]]


@lru_cache(maxsize=None)
def board_post() -> Part:
    """4 mm printed spacer under a board hung below the deck (the MCU). Base at z = 0."""
    p = Cylinder(3.5, 4.0, align=CB) - hw.clear(5.0, top=4.0)
    p.label = "board_spacer"
    return p


@lru_cache(maxsize=None)
def standoff() -> Part:
    """Printed payload standoff, deck top to plate underside: a Ø12 column with Ø20 end flanges, tapped 12 deep at
    each end (an M3 pan head up through the deck, an M3 flat head down through the plate). Base at z = 0. Print
    standing, 100 % infill."""
    h = D.PLATE_BOT - D.DECK_TOP
    fd, ft = D.STANDOFF_FLANGE
    p = Cylinder(D.STANDOFF_D / 2, h, align=CB) + Cylinder(fd / 2, ft, align=CB) + Cylinder(fd / 2, ft, align=CT).moved(Pos(0, 0, h))
    p -= hw.tap(12.0, top=12.0) + hw.tap(12.0, top=h)
    p.label = "standoff"
    return p


@lru_cache(maxsize=None)
def estop_riser() -> Part:
    """Raises the e-stop 20 mm so its contact block ends 40 mm above the floor. ROBOT frame."""
    x, y = ESTOP_XY
    p = Cylinder(15.0, ESTOP_RISER, align=CB).moved(Pos(x, y, D.DECK_TOP))
    p += Box(12.0, 48.0, 4.0, align=CB).moved(Pos(x, y, D.DECK_TOP))
    p -= Cylinder(11.25, 40).moved(Pos(x, y, D.DECK_TOP))
    for s in (-1, 1):
        p -= hw.cbore(4.0, top=D.DECK_TOP + 4.0).moved(Pos(x, y + 18.0 * s, 0))
    p.label = "estop_riser"
    return p


@lru_cache(maxsize=None)
def lidar_riser() -> Part:
    """Raises the A2M12 so its scan plane is at LIDAR_SCAN_Z (over the wheels at their bump stop, under the plate).
    ROBOT frame."""
    h = lidar_base_z() - D.DECK_TOP
    x0, y0 = LIDAR_XY
    p = Cylinder(D.A2M12["d"] / 2 - 2, h, align=CB).moved(Pos(x0, y0, D.DECK_TOP))
    p -= Cylinder(D.A2M12["d"] / 2 - 11, h - 4.0, align=CB).moved(Pos(x0, y0, D.DECK_TOP - 0.01))  # hollow, 4 mm top
    p -= Cylinder(10.0, h + 2).moved(Pos(x0, y0, D.DECK_TOP + h))                                  # cable up
    p -= Box(30, 24, h - 10, align=(Align.MAX, Align.CENTER, Align.MIN)).moved(Pos(x0 - 20, y0, D.DECK_TOP))  # cable out / tool access
    for x, y in LIDAR_FEET:
        p += Cylinder(5.0, 4.0, align=CB).moved(Pos(x, y, D.DECK_TOP))                              # feet
        p -= hw.cbore(4.0, top=D.DECK_TOP + 4.0).moved(Pos(x, y, 0))
    for a in (45, 135, 225, 315):                                                                   # lidar's 4 x M2.5
        x, y = x0 + LIDAR_HOLE_R * math.cos(math.radians(a)), y0 + LIDAR_HOLE_R * math.sin(math.radians(a))
        p -= hw.clear(h + 1, top=D.DECK_TOP + h, d=D.CLR_M25).moved(Pos(x, y, 0))
    p.label = "lidar_riser"
    return p


def lidar_base_z() -> float:
    return D.LIDAR_SCAN_Z - D.A2M12["scan_z"]


# --- battery ----------------------------------------------------------------------------------------------------------------

@lru_cache(maxsize=None)
def battery_tray() -> Part:
    """Cradle under the centre deck for the flat 7S1P pack (153 x 72 x 23) and its BMS beside it, two straps.
    ROBOT frame. Its floor is the robot's belly (~36 mm up)."""
    b = D.BATTERY
    t = 2.0
    z_top = D.DECK_Z
    zb = z_top - b["h"] - t
    p = Box(b["l"] + 2 * t, b["w"] + 2 * t, b["h"] + t, align=CT).moved(Pos(0, 0, z_top))
    p -= Box(b["l"], b["w"], b["h"] + 1, align=CT).moved(Pos(0, 0, z_top + 0.01))
    p += Box(176.0, 64.0, 4.0, align=CT).moved(Pos(0, 0, z_top)) - Box(b["l"], b["w"], 10).moved(Pos(0, 0, z_top))
    for x in (-40.0, 40.0):                                        # strap slots in the floor
        p -= Box(22.0, b["w"] + 10, 10).moved(Pos(x, 0, zb))
    p -= Box(20.0, 30.0, b["h"]).moved(Pos(-b["l"] / 2, 0, z_top - b["h"] / 2))    # leads out the -x end (deck hole at x -95)
    for x, y in BATTERY_TRAY_SCREWS:
        p -= hw.clear(5.0, top=z_top).moved(Pos(x, y, 0))
    p.label = "battery_tray"
    return p


# --- payload plate ----------------------------------------------------------------------------------------------------------

def plate_screws():
    """ROBOT (x, y) of the payload plate's screws into the six standoffs (flat heads, flush with the top)."""
    return list(D.STANDOFF_XY)


PIN_XY = ((87.5, 37.5), (-37.5, -62.5))           # 4 mm dowels: a module fits only one way (kept from v2)
YOKE_SCREWS_X = (-40.0, -4.0, 32.0)                # the yokes' flange screws (mounts.py)
CAM_ARM_SCREWS = ((52.0, 94.0), (68.0, 94.0), (52.0, 102.0))   # +y camera arm's foot (mounts.py), mirrored for -y
THUMBSCREW_XY = ((50.0, 50.0), (50.0, -50.0), (-50.0, 50.0), (-50.0, -50.0))


@lru_cache(maxsize=None)
def payload_plate() -> Part:
    """PAYLOAD contract kept from v2 (25 mm grid, two dowels, centre pass-through), now M3 on 2.8 mm holes. A full
    disc: it rides above the wheels and the lidar. ROBOT frame, z = PLATE_BOT .. PLATE_TOP."""
    p = Cylinder(D.PLATE_R, D.PLATE_T, align=CB).moved(Pos(0, 0, D.PLATE_BOT))
    for x, y in plate_screws():
        p -= hw.csink(D.PLATE_T, top=D.PLATE_TOP).moved(Pos(x, y, 0))
    cam = [(x, s * y) for x, y in CAM_ARM_SCREWS for s in (1, -1)]
    for i in range(-4, 5):
        for j in range(-4, 5):
            x, y = 25.0 * i, 25.0 * j
            r = math.hypot(x, y)
            if r > 98 or r < 20:
                continue
            if any(math.hypot(x - sx, y - sy) < 9 for sx, sy in plate_screws() + cam):
                continue
            p -= hw.tap(D.PLATE_T + 0.1, top=D.PLATE_TOP).moved(Pos(x, y, 0))
    for x in YOKE_SCREWS_X:
        for s in (1, -1):
            p -= hw.tap(D.PLATE_T + 0.1, top=D.PLATE_TOP).moved(Pos(x, s * (D.YOKE_Y - 4.0), 0))
    for x, y in cam:
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
    for lobe in SPLICED:
        for x, y in _splice_pts(LOBES[lobe]):
            h.screw("PAN", 10, Pos(x, y, D.DECK_TOP + 4.0))
    # pod mounts and wheel guards: screws up from under the deck
    for th in D.WHEEL_ANGLES:
        for x, y in P.MOUNT_SCREWS:
            px, py, _ = pod_pt(th, x, y)
            h.screw("PAN", 12, Pos(px, py, D.DECK_Z) * hw.rot_to("-z"))
        for x, s in P.guard_feet(th in P.FRONT):              # M3 x 14: 7.8 mm into the 8 mm foot
            px, py, _ = pod_pt(th, x, s * (D.HOUSE_Y + 4))
            h.screw("PAN", 14, Pos(px, py, D.DECK_Z) * hw.rot_to("-z"))
    # payload plate on its six standoffs: pan heads up through the deck, flat heads down through the plate
    for x, y in D.STANDOFF_XY:
        h.screw("PAN", 14, Pos(x, y, D.DECK_Z) * hw.rot_to("-z"))
        h.screw("FLAT", 14, Pos(x, y, D.PLATE_TOP))
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
    # lidar on its riser
    zl = lidar_base_z()
    h.part("RPLIDAR A2M12", "Slamtec RPLIDAR A2M12 (design.md 7, Fernando's choice)", "Slamtec / robotshop (~$229)",
           V.a2m12(), Pos(*LIDAR_XY, zl))
    for a in (45, 135, 225, 315):
        x, y = LIDAR_XY[0] + LIDAR_HOLE_R * math.cos(math.radians(a)), LIDAR_XY[1] + LIDAR_HOLE_R * math.sin(math.radians(a))
        h.screw("PAN", 8, Pos(x, y, zl - 4.0) * hw.rot_to("-z"), size="M2.5")      # up through the riser's top
    for x, y in LIDAR_FEET:
        h.screw("PAN", 8, Pos(x, y, D.DECK_TOP + 4.0 - D.CB_M3[1]))
    # e-stop on its riser
    h.part("e-stop 22 mm", "22 mm mushroom e-stop, 1 NC contact, 40 mm head (design.md 14)", "e.g. IDEC / generic ($12)",
           V.estop(), Pos(*ESTOP_XY, D.DECK_TOP + ESTOP_RISER))
    for s in (-1, 1):
        h.screw("PAN", 10, Pos(ESTOP_XY[0], ESTOP_XY[1] + 18.0 * s, D.DECK_TOP + 4.0 - D.CB_M3[1]))
    # bumpers
    for lobe in LOBES.values():
        for x, y in bumper_screws_lobe():
            c, s = math.cos(math.radians(lobe)), math.sin(math.radians(lobe))
            h.screw("PAN", 10, Pos(x * c - y * s, x * s + y * c, D.DECK_TOP + 3.0))
    for th in D.WHEEL_ANGLES:
        for a, z in bumper_screws_corner():
            h.screw("PAN", 10, Location((0, 0, z), (0, 0, th + a)) * Pos(D.DECK_R + 2.8, 0, 0) * hw.rot_to("x"))
    return h
