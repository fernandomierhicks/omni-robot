"""Wheel pods (design.md 25.6: 15 mm travel, ~3 N/mm at the wheel, zeta ~0.6), POD frame, x inward.

Layout (one identical pod per wheel; rotational symmetry):
  - motor_cup:  holds the DM-2325 by its face (4 x M3 flat heads from the outside) inside the wheel; a channel in its bore
                takes the motor's leads from the mid-body connector to the inner end.
  - arm:        trailing arm. Bearing boss (61808 inner race) bolted to the cup's inner end (4 x M3 x 22), pivot tube
                with two 623s at PIVOT_YZ, shock boss at SHOCK_LO_YZ. The wheel and motor ride on it (unsprung).
  - pod_mount:  on the clockwise lobe: the pivot clevis and the shock tower (the payload plate has its own standoffs).
  - wheel_guard: open-top guard at the slot's outer end (Fernando, Oct 21: the wheel needs no cover on top): the curved
                front panel that closes the slot down to the skirt line (design.md 23.4) and carries the corner bumper,
                and two side walls on the lobes. It stays under the lidar's scan plane.
Mirrored pods (front-left, rear-right) use mirrored prints of the arm, cup, mount and guard (`_M`); the wheel and
all purchased parts are the same parts, turned (assembly.py).
"""

from __future__ import annotations

import math
from functools import lru_cache

from build123d import (Align, Box, Cylinder, Location, Part, Plane, Polygon, Pos, Rot, extrude)

import dims as D
import hw
import vendor as V

CB = (Align.CENTER, Align.CENTER, Align.MIN)


def x_cyl(r: float, x0: float, x1: float) -> Part:
    lo, hi = min(x0, x1), max(x0, x1)
    return Cylinder(r, hi - lo, rotation=(0, 90, 0)).moved(Pos((lo + hi) / 2, 0, 0))


def yz_cyl(r: float, y: float, z: float, x0: float, x1: float) -> Part:
    return x_cyl(r, x0, x1).moved(Pos(0, y, z))


def hull(circles, n=24):
    """Convex hull (y, z) of circles [(y, z, r)], as a polygon point list."""
    pts = [(y + r * math.cos(2 * math.pi * k / n), z + r * math.sin(2 * math.pi * k / n)) for y, z, r in circles for k in range(n)]
    pts = sorted(set((round(a, 4), round(b, 4)) for a, b in pts))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def yz_plate(pts, x0: float, x1: float) -> Part:
    """Prism of a (y, z) outline between x0 and x1."""
    return extrude(Plane.YZ.offset(x0) * Polygon(*pts, align=None), x1 - x0, dir=(1, 0, 0))


MOTOR_LOC = Pos(D.X_FACE, 0, 0) * Rot(0, -90, 0)       # motor frame -> POD: shaft outward (-x), connector up (+z)
# arm -> cup screws: they pass through the arm's bearing boss (r 20, the 61808's bore), so they sit on r 17 (1.4 mm of
# boss over their 3.2 mm holes, 1.3 mm of cup wall inside their taps; at r 18.15 the boss skin was 0.15 mm), at angles
# that keep them off the connector channel (top) and the arm's shock boss (31 deg)
CUP_SCREW_R = 17.0
CUP_SCREWS = [(CUP_SCREW_R * math.cos(math.radians(a)), CUP_SCREW_R * math.sin(math.radians(a))) for a in (58.0, 148.0, 238.0, 328.0)]
CUP_SCREW_CLR = 3.2
FACE_HOLES = [(7.8, 7.8), (-7.8, 7.8), (-7.8, -7.8), (7.8, -7.8)]       # (y, z): the motor's 4 x M3 on PCD 22, 45 deg off the connector
PLUG_W = 12.0                   # connector channel: the motor slides in from the inner end with its 9 mm connector
                                # housing (and the plugged-in cable) in it (8 mm jammed it); VERIFY with the real plug
PLUG_Z = (14.0, 20.0)


@lru_cache(maxsize=None)
def motor_cup() -> Part:
    r_o = D.CUP_OD / 2
    p = x_cyl(r_o, D.X_CUP_OUT, D.X_CUP_IN) - x_cyl(D.CUP_BORE / 2, D.X_FACE, D.X_CUP_IN + 1)
    # pilot pocket (2 mm). It leaves 0.5 mm webs to the face screws' holes: the motor's own PCD 22 holes sit 2.5 mm
    # outside its Ø17 pilot. Not on the load path (the heads clamp the solid outer 1 mm); walls.py lists them.
    p -= x_cyl(D.DM_PILOT[0] / 2 + 0.3, D.X_FACE - D.DM_PILOT[1] - 0.1, D.X_FACE + 0.1)
    p -= x_cyl(7.5, D.X_CUP_OUT - 1, D.X_FACE)                                           # shaft and seal through the 1 mm left
    for y, z in FACE_HOLES:          # face screws: pan heads on the outer face (countersinks broke into the pilot bore)
        p -= hw.clear(4.0).moved(Pos(D.X_CUP_OUT, y, z) * hw.rot_to("-x"))
    x0, x1 = D.X_FACE + D.DM_PLUG[0] - 1.5, D.X_FACE + D.DM_PLUG[1] + 1.5            # connector pocket (top)
    p -= Box(x1 - x0, PLUG_W + 1.0, PLUG_Z[1] - PLUG_Z[0], align=(Align.MIN, Align.CENTER, Align.MIN)).moved(Pos(x0, 0, PLUG_Z[0]))
    p -= Box(D.X_CUP_IN + 1 - x0, PLUG_W, PLUG_Z[1] - PLUG_Z[0], align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
        Pos(x0, 0, PLUG_Z[0]))                                                          # lead channel to the inner end
    for y, z in CUP_SCREWS:                                                             # arm screws, inner end
        p -= hw.tap(12.0).moved(Pos(D.X_CUP_IN, y, z) * hw.rot_to("x"))           # (a countersink came 0.8 mm from the bore)
    p -= hw.edge_chamfer(r_o, 0.5, 0.0).moved(Pos(D.X_CUP_OUT, 0, 0) * hw.rot_to("-x"))   # first layer (end wall down)
    p.label = "motor_cup"
    return p


ARM_CIRCLES = [(0.0, 0.0, 27.0), (D.PIVOT_YZ[0], D.PIVOT_YZ[1], 9.0), (D.SHOCK_LO_YZ[0], D.SHOCK_LO_YZ[1], 7.5)]
ARM_CLEAR = (D.SLOT_HW_IN - 2.0, D.POD_Z_DECK + 2.5)   # over the deck beside the inner slot, the arm stays above z 5.5


@lru_cache(maxsize=None)
def arm() -> Part:
    xa0, xa1 = D.X_ARM
    p = yz_plate(hull(ARM_CIRCLES), xa0, xa1)
    p -= Box(40, 100, 40, align=(Align.CENTER, Align.MIN, Align.MAX)).moved(Pos((xa0 + xa1) / 2, ARM_CLEAR[0], ARM_CLEAR[1]))
    # bearing boss: 61808 inner race on x 41..48, shoulder 48..49
    p += x_cyl(D.BRG_61808[0] / 2, D.X_BRG[0], D.X_BRG[1]) + x_cyl(23.0, D.X_BRG[1], xa0 + 0.5)
    p -= x_cyl(8.0, D.X_BRG[0] - 1, xa1 + 1)                                              # lead hole
    p -= Box(4.0, 9.0, 12.0, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(Pos(D.X_BRG[0] - 0.1, 0, 6.0))   # from the cup's channel
    for y, z in CUP_SCREWS:
        p -= (hw.clear(xa1 - D.X_BRG[0], d=CUP_SCREW_CLR) + Cylinder(D.CB_M3[0] / 2, D.CB_M3[1] + 0.01, align=(
            Align.CENTER, Align.CENTER, Align.MAX))).moved(Pos(xa1, y, z) * hw.rot_to("x"))
    p -= hw.edge_chamfer(D.BRG_61808[0] / 2, 0.5).moved(Pos(D.X_BRG[0], 0, 0) * hw.rot_to("-x"))   # lead-in for the 61808
    # pivot tube with a 623 at each end
    py, pz = D.PIVOT_YZ
    t0, t1 = D.X_PIVOT_TUBE
    p += yz_cyl(9.0, py, pz, t0, t1)
    p -= yz_cyl(D.BRG_623[1] / 2, py, pz, t0 - 1, t0 + D.BRG_623[2]) + yz_cyl(D.BRG_623[1] / 2, py, pz, t1 - D.BRG_623[2], t1 + 1)
    p -= yz_cyl(3.0, py, pz, t0, t1)
    p -= hw.lead_in(D.BRG_623[1] / 2).moved(Pos(t0, py, pz) * hw.rot_to("-x"))
    p -= hw.lead_in(D.BRG_623[1] / 2).moved(Pos(t1, py, pz) * hw.rot_to("x"))
    # shock boss, tapped from the shock's side
    sy, sz = D.SHOCK_LO_YZ
    p += yz_cyl(6.5, sy, sz, xa0, D.SHOCK_X[0] - 0.5)
    p -= yz_cyl(D.TAP_M3 / 2, sy, sz, D.SHOCK_X[0] - 8.3, D.SHOCK_X[0])            # 2 mm short of the plate's face
    # fillets on the load path: pivot tube and shock boss into the plate's inner face, the deck-clearance notch
    plate_face = yz_plate(hull(ARM_CIRCLES), xa1, xa1 + 4.0)
    notch = Box(40, 100, 40, align=(Align.CENTER, Align.MIN, Align.MAX)).moved(Pos(xa1, ARM_CLEAR[0], ARM_CLEAR[1]))
    p += (hw.cove_ring(9.0, D.FILLET).moved(Pos(xa1, py, pz) * hw.rot_to("x")) & plate_face) - notch
    p += hw.cove_ring(6.5, 1.5).moved(Pos(xa1, sy, sz) * hw.rot_to("x"))
    p += hw.cove_at(D.FILLET, (xa0, ARM_CLEAR[0], ARM_CLEAR[1]), (0, 1, 0), (0, 0, -1), (1, 0, 0), xa1 - xa0) & \
        yz_plate(hull(ARM_CIRCLES), xa0, xa1)
    p.label = "pod_arm"
    return p


Z0 = D.POD_Z_DECK
OUTER_EAR = dict(x=D.X_EAR[0], y=(38.0, 58.0))          # stands on the deck beside the inner slot
WEB = dict(x=D.X_EAR[1], y=(28.0, 64.0))                # inner ear, shock tower, plate post (centre deck)
FOOT = dict(x=(68.5, 80.0), y=(26.0, 66.0), t=7.0)     # beside the pivot tube (x < 68); 7 mm (the pivot nut is above)
TIE = dict(x=(80.0, 100.0), y=(24.0, 42.0), t=10.0)    # foot extension onto the centre deck piece: the side lobes are
                                                       # joined to the centre only through the pod mounts
BRIDGE = dict(y=(44.0, 52.0), z=(26.0, 32.0))           # ties the outer ear to the web over the pivot tube
# (x, y) up from under the deck, M3 x 14 (8 mm of thread each): two in the outer ear, two in the web, one in a boss on
# its foot, two in the tie, which land on the centre piece 6 mm from the joint (base.py)
MOUNT_SCREWS = [(44.5, 43.0), (44.5, 53.0), (77.0, 34.0), (71.5, 58.0), (71.5, 38.0), (90.0, 28.0), (96.0, 36.0)]
MOUNT_BOSSES = [(77.0, 34.0)]                          # in the foot beyond the web: 10 mm for the thread
MOUNT_TAP = 10.0
WEB_TOP = D.SHOCK_HI_YZ[1] + 8.0                       # the web ends over the shock's top eye (robot z 127.5)


def _box(x, y, z) -> Part:
    return Box(x[1] - x[0], y[1] - y[0], z[1] - z[0], align=(Align.MIN, Align.MIN, Align.MIN)).moved(Pos(x[0], y[0], z[0]))


@lru_cache(maxsize=None)
def pod_mount() -> Part:
    py, pz = D.PIVOT_YZ
    sy, sz = D.SHOCK_HI_YZ
    ex, wx = OUTER_EAR["x"], WEB["x"]
    ft = FOOT["t"]
    p = _box(ex, OUTER_EAR["y"], (Z0, Z0 + 6.0))                                           # outer foot
    p += _box(ex, (py - 8.0, py + 8.0), (Z0, BRIDGE["z"][1]))                              # outer ear
    p += _box((ex[0], wx[1]), BRIDGE["y"], BRIDGE["z"])                                    # bridge over the pivot tube
    p += _box(FOOT["x"], FOOT["y"], (Z0, Z0 + ft))                                         # web foot
    p += _box(TIE["x"], TIE["y"], (Z0, Z0 + TIE["t"]))                                     # tie onto the centre piece
    for x, y in MOUNT_BOSSES:
        p += Cylinder(5.0, MOUNT_TAP, align=CB).moved(Pos(x, y, Z0))
    p += _box(wx, WEB["y"], (Z0, WEB_TOP))                                                 # web
    p += yz_cyl(4.5, sy, sz, D.SHOCK_X[1] + 0.5, wx[0] + 0.5)                              # spacer to the shock eye
    # fillets: ear and web into their feet, the bridge into the ear and the web, the spacer into the web (the strips
    # by the web's foot stop short of the pivot's nyloc)
    for y0, y1 in ((WEB["y"][0], py - 5.0), (py + 5.0, WEB["y"][1])):
        p += hw.cove_at(D.FILLET, (wx[1], y0, Z0 + ft), (1, 0, 0), (0, 0, 1), (0, 1, 0), y1 - y0)
    for y, s in ((py - 8.0, -1), (py + 8.0, 1)):
        p += hw.cove_at(2.0, (ex[0], y, Z0 + 6.0), (0, s, 0), (0, 0, 1), (1, 0, 0), ex[1] - ex[0])
    by = BRIDGE["y"]
    p += hw.cove_at(2.0, (ex[1], by[0], BRIDGE["z"][0]), (1, 0, 0), (0, 0, -1), (0, 1, 0), by[1] - by[0])
    p += hw.cove_at(2.0, (wx[0], by[0], BRIDGE["z"][0]), (-1, 0, 0), (0, 0, -1), (0, 1, 0), by[1] - by[0])
    p += hw.cove_at(D.FILLET, (wx[0], by[0], BRIDGE["z"][1]), (-1, 0, 0), (0, 0, 1), (0, 1, 0), by[1] - by[0])
    p += hw.cove_ring(4.5, 2.0).moved(Pos(wx[0], sy, sz) * hw.rot_to("-x"))
    p -= yz_cyl(D.CLR_M3 / 2, py, pz, ex[0] - 1, wx[1] + 1)
    p -= yz_cyl(D.CLR_M3 / 2, sy, sz, D.SHOCK_X[1], wx[1] + 1)
    for x, y in MOUNT_SCREWS:
        p -= hw.tap(MOUNT_TAP, top=Z0 + MOUNT_TAP, entry="bottom").moved(Pos(x, y, 0))
    p.label = "pod_mount"
    return p


def _deck_circle_x(y: float, r: float = D.DECK_R) -> float:
    """POD x of the deck's outer circle at pod y (the circle is centred at x = L_WHEEL in the POD frame)."""
    return D.L_WHEEL - math.sqrt(r * r - y * y)


def robot_to_pod_front(x: float, y: float):
    """A ROBOT point near the front lobe -> (x, y) in a front pod's frame (FL mirrored and FR give the same)."""
    a = (x + abs(y)) / math.sqrt(2)
    t = (x - abs(y)) / math.sqrt(2)
    return D.L_WHEEL - a, -t


HOUSE_FEET = [(-14.0, 1), (16.0, 1), (-14.0, -1), (16.0, -1)]       # (x, sign y): screws up from under the deck
HOUSE_FOOT_X0 = -20.0                                                # feet stay inside r 190 (the bumper lip is outside)
GUARD_PANEL_T = 5.0
GUARD_TOP = 100.0 - D.WHEEL_R                                        # POD z of the guard's top edge (robot 100: just
                                                                     # over the bumper, 50 under the scan plane)
LIDAR_POD = robot_to_pod_front(140.0, 0.0)                           # the A2M12's axis in a front pod's frame
LIDAR_CLEAR = 42.0
FRONT = (45.0, 315.0)                                                # the front wheels' angles


def guard_feet(front: bool):
    """The guard's foot screws (x, sign y), POD frame; a front guard loses the one the lidar riser's clearance cuts."""
    if not front:
        return HOUSE_FEET
    return [(x, s) for x, s in HOUSE_FEET if math.dist((x, s * (D.HOUSE_Y + 4)), LIDAR_POD) > LIDAR_CLEAR + 6.0]


_HALF_SLOT = math.degrees(math.asin((D.HOUSE_Y + 6) / D.DECK_R))
CORNER_BUMPER_SCREWS = [(a, z) for a in (-_HALF_SLOT + 4, 0.0, _HALF_SLOT - 4) for z in (55.0, 85.0)]   # (deg off the
# wheel's diagonal, robot z): radial M3 screws through the corner bumper into the guard's front panel


@lru_cache(maxsize=None)
def wheel_guard(front: bool = False) -> Part:
    xo = _deck_circle_x(D.HOUSE_Y + 3) - 4.0                         # front panel inner face at the walls
    x_in = D.HOUSE_X_IN
    y_leg = D.HOUSE_Y
    deck_circle = Cylinder(D.DECK_R, 400).moved(Pos(D.L_WHEEL, 0, 0))
    p = None
    for s in (1, -1):                                               # side walls on the lobes, with feet
        w = Box(x_in - xo, 6.0, GUARD_TOP - D.POD_Z_DECK, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
            Pos(xo, s * (y_leg + 3.0), D.POD_Z_DECK))
        w += Box(x_in - HOUSE_FOOT_X0, 10.0, 8.0, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(
            Pos(HOUSE_FOOT_X0, s * (y_leg + 4), D.POD_Z_DECK))
        p = w if p is None else p + w
    # front panel: an arc of the deck circle (POD: centred at x = L) from the skirt line up to the top edge; 5 mm, it
    # takes the corner bumper's hits
    r0, r1 = D.DECK_R - GUARD_PANEL_T, D.DECK_R
    x_corner = D.L_WHEEL - math.sqrt(r0 * r0 - (y_leg - 5.0) ** 2) - 0.5  # inside the panel over the fillet's width
    for s in (1, -1):                                               # fillets: side walls into the front panel
        p += hw.cove_at(5.0, (x_corner, s * y_leg, D.POD_Z_DECK), (1, 0, 0), (0, -s, 0), (0, 0, 1), GUARD_TOP - D.POD_Z_DECK)
    yw = y_leg + 6
    ang = math.asin(yw / r1)
    pts = [(D.L_WHEEL - r1 * math.cos(s), r1 * math.sin(s)) for s in _lin(-ang, ang, 21)] + \
          [(D.L_WHEEL - r0 * math.cos(s), r0 * math.sin(s)) for s in _lin(ang, -ang, 21)]
    p += extrude(Plane.XY.offset(D.PANEL_Z0) * Polygon(*pts, align=None), GUARD_TOP - D.PANEL_Z0, dir=(0, 0, 1))
    # the lobes' edges pass through the panel's ends (deck slab beyond the slot)
    p -= Box(400, 30.0, D.DECK_T + 0.4, align=(Align.CENTER, Align.MIN, Align.CENTER)).moved(Pos(0, D.SLOT_HW - 0.2, 0))
    p -= Box(400, 30.0, D.DECK_T + 0.4, align=(Align.CENTER, Align.MAX, Align.CENTER)).moved(Pos(0, -(D.SLOT_HW - 0.2), 0))
    for a_, zr in CORNER_BUMPER_SCREWS:                             # bosses for the corner bumper's radial screws
        loc = Pos(D.L_WHEEL, 0, zr - D.WHEEL_R) * Rot(0, 0, 180 + a_)
        p += Cylinder(4.5, 6.5, rotation=(0, 90, 0)).moved(loc * Pos(r0 - 6.0 + 3.25, 0, 0))
        p -= Cylinder(D.TAP_M3 / 2, 12.0, rotation=(0, 90, 0)).moved(loc * Pos(r0 - 6.5 + 6.0, 0, 0))   # through the boss
    for x, s in guard_feet(front):
        p -= hw.tap(8.0, top=D.POD_Z_DECK + 8.0).moved(Pos(x, s * (y_leg + 4), 0))
    p &= deck_circle
    if front:                                                      # keep clear of the A2M12's riser on the front lobe
        p -= Cylinder(LIDAR_CLEAR, 400).moved(Pos(*LIDAR_POD, 0))
        for x, s in HOUSE_FEET:                                    # a dropped foot: its strips beside the wall go too
            if (x, s) not in guard_feet(True):                     # (the lidar cut left them 0.2 mm thin)
                for ya, yb in ((y_leg - 1.1, y_leg + 0.01), (y_leg + 5.99, y_leg + 9.1)):
                    p -= Box(x_in - x + 12.0, yb - ya, 8.2, align=(Align.MIN, Align.MIN, Align.MIN)).moved(
                        Pos(x - 12.0, ya if s > 0 else -yb, D.POD_Z_DECK - 0.1))
    p.label = "wheel_guard"
    return p


def _lin(a, b, n):
    return [a + (b - a) * k / (n - 1) for k in range(n)]


def hardware_unsprung() -> hw.Hardware:
    """Pod hardware that rides on the arm (POD frame, static pose; the wheel's own is in wheel.hardware)."""
    h = hw.Hardware()
    h.part("Damiao DM-2325", "Damiao DM-S2325-1EC planetary BLDC + FOC driver, 1:25, CAN (section 24)",
           "aifitlab.com / Foxtech, $86", V.dm2325(trim=True), MOTOR_LOC)
    for y, z in FACE_HOLES:          # pan heads on the cup's face, under the #1999 hub's flange (6 mm away)
        h.screw("PAN", 8, Pos(D.X_CUP_OUT, y, z) * hw.rot_to("-x"))
    for y, z in CUP_SCREWS:                                        # arm -> cup
        h.screw("PAN", 22, Pos(D.X_ARM[1] - D.CB_M3[1], y, z) * hw.rot_to("x"))
    py, pz = D.PIVOT_YZ
    t0, t1 = D.X_PIVOT_TUBE
    for xc in (t0 + D.BRG_623[2] / 2, t1 - D.BRG_623[2] / 2):
        hw.bearing(h, "623", Pos(xc, py, pz) * hw.rot_to("x"))
    sy, sz = D.SHOCK_LO_YZ                                         # lower shock bolt, into the arm's tapped boss
    h.washer(Pos(D.SHOCK_X[1], sy, sz) * hw.rot_to("x"))
    h.screw("PAN", 14, Pos(D.SHOCK_X[1] + 0.5, sy, sz) * hw.rot_to("x"))
    return h


@lru_cache(maxsize=None)
def _shim() -> Part:
    return Cylinder(3.0, 0.5, align=CB) - Cylinder(1.6, 2)


def hardware_sprung() -> hw.Hardware:
    """Pod hardware fixed to the chassis (POD frame): the pivot bolt and shims, the shock's top bolt."""
    h = hw.Hardware()
    py, pz = D.PIVOT_YZ
    (e0, e1), (i0, i1) = D.X_EAR
    t1 = D.X_PIVOT_TUBE[1]
    for xs in (e1, t1):          # M3 x 40: ear, shim, 623, tube, 623, shim, ear, nyloc
        h.part("shim 3x6x0.5", "M3 shim washer 3 x 6 x 0.5, DIN 988", "McMaster 98055A (shim washer, metric)",
               _shim(), Pos(xs, py, pz) * hw.rot_to("x"))
    h.screw("PAN", 40, Pos(e0, py, pz) * hw.rot_to("-x"))
    h.nut(Pos(i1, py, pz) * hw.rot_to("x"), kind="nyloc")
    ty, tz = D.SHOCK_HI_YZ
    h.screw("PAN", 20, Pos(D.SHOCK_X[0], ty, tz) * hw.rot_to("-x"))
    h.nut(Pos(i1, ty, tz) * hw.rot_to("x"), kind="nyloc")
    return h


def shock_item(bump: float = 0.0) -> hw.Hardware:
    """The shock between its lower eye (on the arm, moved by bump) and its fixed upper eye, POD frame."""
    h = hw.Hardware()
    lo = arm_motion(bump) * Pos(0, *D.SHOCK_LO_YZ)
    sy, sz = lo.position.Y, lo.position.Z
    ty, tz = D.SHOCK_HI_YZ
    ang = math.degrees(math.atan2(ty - sy, tz - sz))
    h.part("RC shock 50 mm", "1/16-scale RC oil shock, 50 mm eye to eye, 10 mm stroke, M3 eyes; spring ~9 N/mm (est)",
           "e.g. 1/16 RC (WLtoys 144001-class) shock; VERIFY length, stroke, spring",
           V.shock(math.hypot(ty - sy, tz - sz)), Pos(sum(D.SHOCK_X) / 2, sy, sz) * Rot(-ang, 0, 0))
    return h


def shock_travel() -> dict:
    """Shock length and the wheel travel its stroke allows (motion ratio from the geometry)."""
    ty, tz = D.SHOCK_HI_YZ

    def length(b):
        lo = arm_motion(b) * Pos(0, *D.SHOCK_LO_YZ)
        return math.hypot(ty - lo.position.Y, tz - lo.position.Z)
    l0, l1 = length(0.0), length(1.0)
    mr = l0 - l1
    return dict(static=l0, motion_ratio=mr, bump_travel=(l0 - (D.SHOCK_L - D.SHOCK_STROKE)) / mr,
                droop_travel=(D.SHOCK_L - l0) / mr, spring_at_shock=D.POD_RATE / mr ** 2)


def printed() -> list[tuple[str, Part, str, int, str]]:
    return [("motor_cup", motor_cup(), "PETG", 60, "end wall down"),
            ("pod_arm", arm(), "PETG", 60, "outer face (x 49) down"),
            ("pod_mount", pod_mount(), "PETG", 50, "base down"),
            ("wheel_guard", wheel_guard(), "PETG", 30, "front panel down")]


def pod_to_robot(th_deg: float, bump: float = 0.0) -> Location:
    """POD frame -> ROBOT for the wheel at th_deg (the static pose; bump moves only the unsprung parts)."""
    th = math.radians(th_deg)
    return Location((D.L_WHEEL * math.cos(th), D.L_WHEEL * math.sin(th), D.WHEEL_R), (0, 0, th_deg + 180))


def arm_motion(bump: float) -> Location:
    """Location of the unsprung parts (arm, cup, motor, wheel) in the POD frame when the wheel is `bump` mm above its
    static height: a rotation about the pivot axis."""
    py, pz = D.PIVOT_YZ
    a = math.degrees(math.asin(bump / math.hypot(py, pz)))      # small angle about the pivot
    return Pos(0, py, pz) * Rot(-a, 0, 0) * Pos(0, -py, -pz)
