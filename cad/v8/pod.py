"""Wheel pods (design.md 25.6: 15 mm travel, ~3 N/mm at the wheel, zeta ~0.6), POD frame, x inward.

Round 2 (Fernando, Oct 27: "the 4 wheels need bearings and maybe even shock absorbers on both sides; a cantilever wheel
assembly is going to break; thick strong parts"). Every load path now has a support on each side of the wheel:

  - the wheel turns on a 61808 at each end (v8): the inner one on the inner plate's boss, the outer one on hub A's boss
    in the outer plate;
  - the fork (inner plate, outer plate, a 10 x 20 6061 bar under the deck, and the wheel between the plates) pivots on
    one 8 mm steel shaft beside the wheel, in a 608 in each plate; the shaft is held by an ear on each side of the
    wheel, both in the wheel guard (the inner ear pins it with an M3 through a cross hole);
  - a shock on each side: the inner one from the inner plate to the pod mount's tower, the outer one from the outer
    plate to a boss on the guard's front panel, so the outer side is sprung by the chassis, not through the fork;
  - the plates are 12 mm (inner) and 9.5 mm with a 10.5 mm housing (outer), the bar 10 mm (v8: 8, 6.5, 6).

Parts:
  - motor_cup:  holds the DM-2325 by its face (4 x M3 pan heads from the outside) inside the wheel; a channel in its bore
                takes the motor's leads from the mid-body connector to the inner end.
  - arm:        the fork's inner plate. Bearing boss (61808 inner race) bolted to the cup's inner end (4 x M3 x 25), a
                608 in its pivot boss, the inner shock's boss, a leg under the deck to the bar.
  - outer_plate: the fork's outer plate: the outer 61808's housing, a 608 in its pivot boss, the outer shock's boss, a
                lug under the deck to the bar.
  - fork bar:   6061 flat bar 10 x 20 on edge under the deck, two M3 into each plate's lug.
  - pod_mount:  the inner shock's tower on the deck (a foot, a tie onto the centre deck piece: the side lobes are joined
                to the centre only through the pod mounts).
  - wheel_guard: open-top guard at the slot's outer end: the curved front panel (the corner bumper's backing and the
                outer shock's top), a side wall on each lobe; the pivot side's wall carries both ears.
Mirrored pods (front-left, rear-right) use mirrored prints of the arm, cup, mount, guard and outer plate (`_M`); the
wheel and all purchased parts are the same parts, turned (assembly.py).
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


def _box(x, y, z) -> Part:
    return Box(x[1] - x[0], y[1] - y[0], z[1] - z[0], align=(Align.MIN, Align.MIN, Align.MIN)).moved(Pos(x[0], y[0], z[0]))


MOTOR_LOC = Pos(D.X_FACE, 0, 0) * Rot(0, -90, 0)       # motor frame -> POD: shaft outward (-x), connector up (+z)
# arm -> cup screws: they pass through the arm's bearing boss (r 20, the 61808's bore), so they sit on r 17 (1.4 mm of
# boss over their 3.2 mm holes, 1.3 mm of cup wall inside their taps; at r 18.15 the boss skin was 0.15 mm), at angles
# that keep them off the connector channel (top) and the arm's shock boss
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


# --- the fork --------------------------------------------------------------------------------------------------------

PY, PZ = D.PIVOT_YZ
PIV_R = D.BRG_608[1] / 2 + 2.5                           # pivot bosses round the 608s (r 13.5)
ARM_CIRCLES = [(0.0, 0.0, 30.0), (PY, PZ, PIV_R), (D.SHOCK_IN[0][0], D.SHOCK_IN[0][1], 7.5)]
ARM_CLEAR = (D.SLOT_HW_IN - 2.0, D.POD_Z_DECK + 2.5)    # over the deck beside the inner slot, the arm stays above z 5.5
OUT_CLEAR = (D.FORK_LUG_Y[1], PZ - PIV_R)                          # over the lobe beside the wide slot, the outer plate stays above
                                                        # its pivot boss's underside (the guard's low wall is under it)
FORK_SCREW_X = (sum(D.X_OPLATE) / 2, sum(D.X_ARM) / 2)  # the bar's screws: outer plate, inner plate (POD x)


def _608_seat(x0: float, x1: float, open_at: float) -> Part:
    """Cutter: a 608 seat between x0 and x1 at the pivot, open at the face x = open_at (a lead-in there), and a 16 mm
    hole on through the rest (the race's lip; the shaft passes)."""
    lo, hi = min(x0, x1), max(x0, x1)
    c = yz_cyl(D.BRG_608[1] / 2, PY, PZ, lo - (1.0 if open_at < lo + 1 else 0.0), hi + (1.0 if open_at > hi - 1 else 0.0))
    c += hw.lead_in(D.BRG_608[1] / 2).moved(Pos(open_at, PY, PZ) * hw.rot_to("-x" if open_at < (lo + hi) / 2 else "x"))
    return c


@lru_cache(maxsize=None)
def arm() -> Part:
    """The fork's inner plate (12 mm)."""
    xa0, xa1 = D.X_ARM
    p = yz_plate(hull(ARM_CIRCLES), xa0, xa1)
    p -= Box(40, 100, 40, align=(Align.CENTER, Align.MIN, Align.MAX)).moved(Pos((xa0 + xa1) / 2, ARM_CLEAR[0], ARM_CLEAR[1]))
    # the leg under the deck, out to the bar's lug; the sloped top runs it into the hull inside the narrow slot. Its top
    # is 10 mm under the deck's underside where it passes under the lobe: the fork swings about a shaft above and beside
    # it, so at the bump stop that part rises 6 mm (at 5 mm it met the deck)
    bz0, bz1 = D.FORK_BAR["z"]
    p += yz_plate([(10.0, bz0), (D.FORK_LUG_Y[1], bz0), (D.FORK_LUG_Y[1], bz1), (57.0, bz1), (52.0, -13.0), (34.0, -13.0), (20.0, -2.0)],
                  xa0, xa1)
    # bearing boss: 61808 inner race on x 41..48, shoulder 48..49
    p += x_cyl(D.BRG_61808[0] / 2, D.X_BRG[0], D.X_BRG[1]) + x_cyl(23.0, D.X_BRG[1], xa0 + 0.5)
    p -= x_cyl(8.0, D.X_BRG[0] - 1, xa1 + 1)                                              # lead hole
    p -= Box(4.0, 9.0, 12.0, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(Pos(D.X_BRG[0] - 0.1, 0, 6.0))   # from the cup's channel
    for y, z in CUP_SCREWS:
        p -= (hw.clear(xa1 - D.X_BRG[0], d=CUP_SCREW_CLR) + Cylinder(D.CB_M3[0] / 2, D.CB_M3[1] + 0.01, align=(
            Align.CENTER, Align.CENTER, Align.MAX))).moved(Pos(xa1, y, z) * hw.rot_to("x"))
    p -= hw.edge_chamfer(D.BRG_61808[0] / 2, 0.5).moved(Pos(D.X_BRG[0], 0, 0) * hw.rot_to("-x"))   # lead-in for the 61808
    # pivot: a 608 pressed in from the outer face (it stands 0.5 proud, its inner race shimmed to the inner ear)
    p -= _608_seat(D.X_PIV_IN[0], D.X_PIV_IN[1], D.X_PIV_IN[0]) + yz_cyl(8.0, PY, PZ, D.X_PIV_IN[1] - 0.1, xa1 + 1)
    # inner shock's boss, tapped from the shock's side
    sy, sz = D.SHOCK_IN[0]
    p += yz_cyl(6.5, sy, sz, xa0, D.SHOCK_X[0] - 0.5)
    p -= yz_cyl(D.TAP_M3 / 2, sy, sz, D.SHOCK_X[0] - 9.0, D.SHOCK_X[0])
    p += hw.cove_ring(6.5, 1.0).moved(Pos(xa1, sy, sz) * hw.rot_to("x"))
    p += hw.cove_at(D.FILLET, (xa0, ARM_CLEAR[0], ARM_CLEAR[1]), (0, 1, 0), (0, 0, -1), (1, 0, 0), xa1 - xa0) & \
        yz_plate(hull(ARM_CIRCLES), xa0, xa1)
    for z in D.FORK_SCREW_Z:                                                       # the fork bar's two screws
        p -= hw.tap(9.0, top=D.FORK_LUG_Y[1], entry="top").moved(Pos(FORK_SCREW_X[1], 0, z) * hw.rot_to("y"))
    p.label = "pod_arm"
    return p


@lru_cache(maxsize=None)
def outer_plate() -> Part:
    """The fork's outer plate: the outer 61808's housing (bore 52, a 3 mm lip outboard) on a 9.5 mm web over the
    wheel's outer face, up to its pivot boss (a 608) and down to the lug the bar screws to; the outer shock's boss on
    its outer face. POD frame; it rides with the fork (unsprung)."""
    x0, x1 = D.X_OUTER
    w0, w1 = D.X_OPLATE
    bz0, bz1 = D.FORK_BAR["z"]
    ly0, ly1 = D.FORK_LUG_Y
    r_h = D.BRG_61808[1] / 2 + 5.0                                          # housing r 31
    sy, sz = D.SHOCK_OUT[0]
    p = x_cyl(r_h, x0, x1)
    p += yz_plate(hull([(0.0, 0.0, r_h), (PY, PZ, PIV_R), (sy, sz, 6.5)] +
                       [(y, z, 0.01) for y in (ly0 - 8.0, ly1) for z in (bz0, bz1)]), w0, w1)
    p -= Box(30, 60, 60, align=(Align.CENTER, Align.MIN, Align.MAX)).moved(Pos((w0 + w1) / 2, OUT_CLEAR[0], OUT_CLEAR[1]))
    p += yz_plate(hull([(PY, PZ, PIV_R), (PY - 8.0, PZ - 4.0, PIV_R)]), w0, w1)          # the boss, whole again
    p -= x_cyl(r_h + 0.5, x1, w1 + 1.0)                                     # off hub A's end plate (r 30, x > -43)
    b0, b1 = D.X_BRG_OUT
    p -= x_cyl(D.BRG_61808[1] / 2, b0, x1 + 0.1)                            # the 61808's outer race
    p -= x_cyl(D.BRG_61808[1] / 2 - 3.0, x0 - 1, b0 + 0.1)                  # the lip (clear of the inner race)
    p -= hw.lead_in(D.BRG_61808[1] / 2).moved(Pos(x1, 0, 0) * hw.rot_to("x"))
    p -= hw.edge_chamfer(r_h, 0.5, 0.0, up=False).moved(Pos(x0, 0, 0) * hw.rot_to("x"))   # first layer (prints lip down)
    # pivot: a 608 pressed in from the inner face (flush; a shim to the outer ear, 0.5 mm off the plate), a lip outboard
    p -= _608_seat(D.X_PIV_OUT[0], D.X_PIV_OUT[1], w1) + yz_cyl(8.0, PY, PZ, w0 - 1, D.X_PIV_OUT[0] + 0.1)
    # outer shock's boss on the outer face, tapped from the shock's side through into the web
    p += yz_cyl(6.5, sy, sz, D.SHOCK_X_OUT[1] + 0.5, w0 + 0.5)
    p -= yz_cyl(D.TAP_M3 / 2, sy, sz, D.SHOCK_X_OUT[1] + 0.5 - 0.01, w1 - 2.0)          # 2 mm short of the inner face
    p += hw.cove_ring(6.5, 1.0).moved(Pos(w0, sy, sz) * hw.rot_to("-x"))
    for z in D.FORK_SCREW_Z:
        p -= hw.tap(9.0, top=ly1, entry="top").moved(Pos(FORK_SCREW_X[0], 0, z) * hw.rot_to("y"))
    p.label = "outer_plate"
    return p


def fork_bar_loc() -> Location:
    """The bar's centre in the POD frame (fork_bar() is modelled centred, so the mirrored pods turn it into place)."""
    return Pos(*(sum(D.FORK_BAR[k]) / 2 for k in "xyz"))


@lru_cache(maxsize=None)
def fork_bar() -> Part:
    """6061 flat bar 10 x 20 on edge, cut to length, four 3.4 mm cross holes. Centred on its own frame."""
    (x0, x1), (y0, y1), (z0, z1) = D.FORK_BAR["x"], D.FORK_BAR["y"], D.FORK_BAR["z"]
    p = Box(x1 - x0, y1 - y0, z1 - z0)
    c = fork_bar_loc().position
    for x in FORK_SCREW_X:
        for z in D.FORK_SCREW_Z:
            p -= Cylinder(D.CLR_M3 / 2, 20, rotation=(90, 0, 0)).moved(Pos(x - c.X, 0, z - c.Z))
    p.label = "fork bar"
    return p


def shaft_loc() -> Location:
    """The shaft's centre in the POD frame (shaft() is modelled centred, so the mirrored pods turn it into place)."""
    return Pos(sum(D.SHAFT) / 2, PY, PZ)


@lru_cache(maxsize=None)
def shaft() -> Part:
    """The pivot shaft: 8 mm steel rod, cut to length, a 3.4 mm cross hole at the inner ear's pin. Centred on its
    own frame, axis along x."""
    s0, s1 = D.SHAFT
    c = sum(D.SHAFT) / 2
    p = x_cyl(D.BRG_608[0] / 2, s0 - c, s1 - c) - Cylinder(D.CLR_M3 / 2, 20).moved(Pos(D.PIN_X - c, 0, 0))
    p.label = "shaft 8 x 104"
    return p


# --- the inner shock's tower -----------------------------------------------------------------------------------------

Z0 = D.POD_Z_DECK
TOWER = dict(x=D.X_TOWER, y=(22.0, 46.0))
FOOT = dict(x=(D.X_TOWER[0], 92.0), y=(18.0, 50.0), t=10.0)
TIE = dict(x=(92.0, 104.0), y=(20.0, 38.0), t=10.0)     # the foot's extension onto the centre deck piece
# (x, y) up from under the deck, M3 x 14, 10 mm into the foot (the tie's two land on the centre piece, base.py)
MOUNT_SCREWS = [(86.0, 22.0), (86.0, 46.0), (90.0, 30.0), (97.0, 26.0), (100.0, 33.5)]   # >= 6 mm off the deck's joint
MOUNT_TAP = 10.0
WEB_TOP = D.SHOCK_IN[1][1] + 6.0                         # over the shock's top eye (robot z 127.8: under 128)


@lru_cache(maxsize=None)
def pod_mount() -> Part:
    sy, sz = D.SHOCK_IN[1]
    wx = TOWER["x"]
    p = _box(FOOT["x"], FOOT["y"], (Z0, Z0 + FOOT["t"])) + _box(TIE["x"], TIE["y"], (Z0, Z0 + TIE["t"]))
    p += _box(wx, TOWER["y"], (Z0, WEB_TOP))
    p += yz_cyl(4.5, sy, sz, D.SHOCK_X[1] + 0.5, wx[0] + 0.5)                       # spacer to the shock eye
    p += hw.cove_ring(4.5, 2.0).moved(Pos(wx[0], sy, sz) * hw.rot_to("-x"))
    for y0, y1 in (TOWER["y"],):                                                      # the web into its foot
        p += hw.cove_at(D.FILLET, (wx[1], y0, Z0 + FOOT["t"]), (1, 0, 0), (0, 0, 1), (0, 1, 0), y1 - y0)
    p += hw.cove_at(D.FILLET, (wx[0], TOWER["y"][1], Z0 + FOOT["t"]), (0, 1, 0), (0, 0, 1), (1, 0, 0), wx[1] - wx[0])
    p -= yz_cyl(D.CLR_M3 / 2, sy, sz, D.SHOCK_X[1], wx[1] + 1)
    for x, y in MOUNT_SCREWS:
        p -= hw.tap(MOUNT_TAP, top=Z0 + MOUNT_TAP, entry="bottom").moved(Pos(x, y, 0))
    p.label = "pod_mount"
    return p


# --- the wheel guard: front panel, side walls, both ears ---------------------------------------------------------------

def _deck_circle_x(y: float, r: float = D.DECK_R) -> float:
    """POD x of the deck's outer circle at pod y (the circle is centred at x = L_WHEEL in the POD frame)."""
    return D.L_WHEEL - math.sqrt(r * r - y * y)


def robot_to_pod_front(x: float, y: float):
    """A ROBOT point near the front lobe -> (x, y) in a front pod's frame (FL mirrored and FR give the same)."""
    a = (x + abs(y)) / math.sqrt(2)
    t = (x - abs(y)) / math.sqrt(2)
    return D.L_WHEEL - a, -t


HOUSE_FEET = [(-14.0, 1), (16.0, 1), (-14.0, -1), (16.0, -1)]       # (x, sign y): screws up from under the deck
EAR_FEET = [(D.PIN_X, 1), (sum(D.X_EAR_OUT) / 2, 1)]                # one under each ear
HOUSE_FOOT_X0 = -20.0                                                # feet stay inside r 190 (the bumper lip is outside)
GUARD_PANEL_T = 5.0
GUARD_TOP = 128.0 - D.WHEEL_R - 0.5                                  # POD z of the panel's top (robot 127.5): it carries
                                                                     # the outer shock's top eye; 22 under the scan plane
WALL_TOP = D.PIVOT_YZ[1] + 14.0                                      # the side walls and the ears (the pin's head on top)
LOW_WALL = OUT_CLEAR[1] - 1.5                                        # the pivot side's wall under the outer plate's boss
EAR_Y = (PY - 10.0, D.HOUSE_Y + 6.0)
LIDAR_POD = robot_to_pod_front(140.0, 0.0)                           # the A2M12's axis in a front pod's frame
LIDAR_CLEAR = 42.0
FRONT = (45.0, 315.0)                                                # the front wheels' angles
ROLLER_CLEAR = D.WHEEL_R + 2.0                                       # the outer ear stays this far from the axle


def guard_feet(front: bool):
    """The guard's foot screws (x, sign y), POD frame; a front guard loses the one the lidar riser's clearance cuts."""
    feet = HOUSE_FEET + EAR_FEET
    if not front:
        return feet
    return [(x, s) for x, s in feet if math.dist((x, s * (D.HOUSE_Y + 4)), LIDAR_POD) > LIDAR_CLEAR + 6.0]


_HALF_SLOT = math.degrees(math.asin((D.HOUSE_Y + 6) / D.DECK_R))
CORNER_BUMPER_SCREWS = [(0.0, 55.0), (0.0, 85.0)] + [(s * (_HALF_SLOT - 4), z) for s in (-1, 1) for z in (72.0, 88.0)]
# ^ (deg off the wheel's diagonal, robot z): radial M3 screws through the corner bumper into the guard's front panel


def _panel_x(y: float, inner: bool = True) -> float:
    return _deck_circle_x(y, D.DECK_R - (GUARD_PANEL_T if inner else 0.0))


@lru_cache(maxsize=None)
def wheel_guard(front: bool = False) -> Part:
    xo = _deck_circle_x(D.HOUSE_Y + 3) - 4.0                         # front panel inner face at the walls
    x_in = D.HOUSE_X_IN
    y_leg = D.HOUSE_Y
    deck_circle = Cylinder(D.DECK_R, 400).moved(Pos(D.L_WHEEL, 0, 0))
    eo, ei = D.X_EAR_OUT, D.X_EAR_IN
    # side walls on the lobes, with feet. The far side (-y) runs full height; the pivot side (+y) is full height between
    # the ears and on to the inner ear, low under the outer plate's pivot boss
    p = _box((xo, x_in), (-(y_leg + 6.0), -y_leg), (Z0, WALL_TOP))
    p += _box((HOUSE_FOOT_X0, x_in), (-(y_leg + 9.0), -(y_leg - 1.0)), (Z0, Z0 + 8.0))
    p += _box((eo[1] - 0.01, ei[0] + 0.01), (y_leg, y_leg + 6.0), (Z0, WALL_TOP))
    p += _box((xo, eo[0] + 0.01), (y_leg, y_leg + 6.0), (Z0, LOW_WALL))
    p += _box((eo[0], ei[1]), (y_leg - 1.0, y_leg + 9.0), (Z0, Z0 + 8.0))       # the feet, both ears' too (the
    # bumper's lip is beyond the outer ear)
    # the ears: round the shaft, down to the deck (the outer one beside the rollers, cut back to r 65.5 from the axle)
    for (a, b), top in ((ei, WALL_TOP), (eo, WALL_TOP - 4.0)):
        ear = _box((a, b), EAR_Y, (Z0, top))
        if b < 0:
            ear -= x_cyl(ROLLER_CLEAR, a - 1, b + 1)
        p += ear
        p -= yz_cyl(D.BRG_608[0] / 2 + 0.05, PY, PZ, a - 1, b + 1)                  # the shaft's bore (slip)
        p -= hw.lead_in(D.BRG_608[0] / 2).moved(Pos(a, PY, PZ) * hw.rot_to("-x"))
        p -= hw.lead_in(D.BRG_608[0] / 2).moved(Pos(b, PY, PZ) * hw.rot_to("x"))
    p -= hw.clear(WALL_TOP - PZ, top=WALL_TOP).moved(Pos(D.PIN_X, PY, 0))            # the pin: clear to the shaft,
    p -= hw.tap(11.0, top=PZ - 3.0).moved(Pos(D.PIN_X, PY, 0))                      # thread under it (from in the bore)
    # front panel: an arc of the deck circle (POD: centred at x = L) from the skirt line up to the top edge; 5 mm, it
    # takes the corner bumper's hits and the outer shock's top
    r0, r1 = D.DECK_R - GUARD_PANEL_T, D.DECK_R
    x_corner = D.L_WHEEL - math.sqrt(r0 * r0 - (y_leg - 5.0) ** 2) - 0.5  # inside the panel over the fillet's width
    for s, h in ((1, LOW_WALL), (-1, WALL_TOP)):                    # fillets: side walls into the front panel
        p += hw.cove_at(5.0, (x_corner, s * y_leg, D.POD_Z_DECK), (1, 0, 0), (0, -s, 0), (0, 0, 1), h - D.POD_Z_DECK)
    yw = y_leg + 6
    ang = math.asin(yw / r1)
    pts = [(D.L_WHEEL - r1 * math.cos(s), r1 * math.sin(s)) for s in _lin(-ang, ang, 21)] + \
          [(D.L_WHEEL - r0 * math.cos(s), r0 * math.sin(s)) for s in _lin(ang, -ang, 21)]
    p += extrude(Plane.XY.offset(D.PANEL_Z0) * Polygon(*pts, align=None), GUARD_TOP - D.PANEL_Z0, dir=(0, 0, 1))
    # the outer shock's top: a boss on the panel's inner face, tapped through the panel; under it a hole to drive the
    # lower eye's screw (the corner bumper covers both)
    (ly, lz), (uy, uz) = D.SHOCK_OUT
    xp = _panel_x(uy)
    p += yz_cyl(4.5, uy, uz, D.SHOCK_X_OUT[0] - 0.5, _panel_x(uy - 4.5) - 1.5)          # into the curved panel
    p -= yz_cyl(D.TAP_M3 / 2, uy, uz, D.SHOCK_X_OUT[0] + 0.5, _panel_x(uy, False) - 1.0)
    p -= yz_cyl(3.5, ly, lz, _panel_x(ly, False) - 1, _panel_x(ly) + 1)
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


# --- hardware ----------------------------------------------------------------------------------------------------------

def _shim8() -> Part:
    return Cylinder(7.0, 0.5, align=CB) - Cylinder(4.1, 2)


def hardware_unsprung() -> hw.Hardware:
    """Pod hardware that rides on the fork (POD frame, static pose; the wheel's own is in wheel.hardware)."""
    h = hw.Hardware()
    h.part("Damiao DM-2325", "Damiao DM-S2325-1EC planetary BLDC + FOC driver, 1:25, CAN (section 24)",
           "aifitlab.com / Foxtech, $86", V.dm2325(trim=True), MOTOR_LOC)
    for y, z in FACE_HOLES:          # pan heads on the cup's face, under the #1999 hub's flange (6 mm away)
        h.screw("PAN", 8, Pos(D.X_CUP_OUT, y, z) * hw.rot_to("-x"))
    for y, z in CUP_SCREWS:                                        # inner plate -> cup: 7.9 mm into it
        h.screw("PAN", 25, Pos(D.X_ARM[1] - D.CB_M3[1], y, z) * hw.rot_to("x"))
    for x0, x1 in (D.X_PIV_IN, D.X_PIV_OUT):                       # the fork's pivot bearings
        hw.bearing(h, "608", Pos((x0 + x1) / 2, PY, PZ) * hw.rot_to("x"))
    sy, sz = D.SHOCK_IN[0]                                         # inner shock's lower bolt, into the plate's boss
    h.washer(Pos(D.SHOCK_X[1], sy, sz) * hw.rot_to("x"))
    h.screw("PAN", 14, Pos(D.SHOCK_X[1] + 0.5, sy, sz) * hw.rot_to("x"))
    sy, sz = D.SHOCK_OUT[0]                                        # outer shock's lower bolt, into the outer plate
    h.screw("PAN", 16, Pos(D.SHOCK_X_OUT[0], sy, sz) * hw.rot_to("-x"))
    # the bar, two M3 x 18 at each end through it (10 mm) into the inner plate's leg and the outer plate's lug (8 mm)
    h.part("fork bar 10 x 20", f"6061 flat bar 10 x 20 mm, {D.FORK_BAR['x'][1] - D.FORK_BAR['x'][0]:.0f} mm long, 4 x 3.4 mm "
           "cross holes (the wheel fork's link under the deck)", "McMaster 8975K (6061 bar 3/8 x 3/4 in) or 10 x 20 metric "
           "stock", fork_bar(), fork_bar_loc())
    for x in FORK_SCREW_X:
        for z in D.FORK_SCREW_Z:
            h.screw("PAN", 18, Pos(x, D.FORK_BAR["y"][1], z) * hw.rot_to("y"))
    return h


def hardware_sprung() -> hw.Hardware:
    """Pod hardware fixed to the chassis (POD frame): the pivot shaft, its pin and shims, the shocks' top bolts."""
    h = hw.Hardware()
    h.part("shaft 8 x 104", "8 mm steel shaft (h6 linear shaft or drill rod) cut to 104 mm, one 3.4 mm cross hole 52.25 "
           "mm from the outer end (the fork's pivot)", "McMaster 6112K (8 mm precision shaft) + drill", shaft(), shaft_loc())
    for xs in (D.X_EAR_IN[1], D.X_EAR_OUT[0]):                     # 0.5 mm shims: the 608s' inner races to the ears
        h.part("shim 8x14x0.5", "shim washer 8 x 14 x 0.5, DIN 988 (the pivot 608s' inner races to the ears)",
               "McMaster 98055A (shim washer, metric)", _shim8(), Pos(xs, PY, PZ) * hw.rot_to("x" if xs > 0 else "-x"))
    h.screw("PAN", 25, Pos(D.PIN_X, PY, WALL_TOP))                # the pin: through the ear and the shaft, into the ear
    ty, tz = D.SHOCK_IN[1]                                         # inner shock's top: tower, nyloc
    h.screw("PAN", 20, Pos(D.SHOCK_X[0], ty, tz) * hw.rot_to("-x"))
    h.nut(Pos(D.X_TOWER[1], ty, tz) * hw.rot_to("x"), kind="nyloc")
    ty, tz = D.SHOCK_OUT[1]                                        # outer shock's top: into the panel's boss
    h.screw("PAN", 16, Pos(D.SHOCK_X_OUT[1], ty, tz) * hw.rot_to("x"))
    return h


def _shock_len(lo_yz, hi_yz, bump: float) -> float:
    lo = arm_motion(bump) * Pos(0, *lo_yz)
    return math.hypot(hi_yz[0] - lo.position.Y, hi_yz[1] - lo.position.Z)


def shock_item(bump: float = 0.0) -> hw.Hardware:
    """Both shocks between their lower eyes (on the fork, moved by bump) and their fixed upper eyes, POD frame."""
    h = hw.Hardware()
    for (lo_yz, (ty, tz)), xs in ((D.SHOCK_IN, D.SHOCK_X), (D.SHOCK_OUT, D.SHOCK_X_OUT)):
        lo = arm_motion(bump) * Pos(0, *lo_yz)
        sy, sz = lo.position.Y, lo.position.Z
        ang = math.degrees(math.atan2(ty - sy, tz - sz))
        h.part("RC shock 50 mm", "1/16-scale RC oil shock, 50 mm eye to eye, 10 mm stroke, M3 eyes; spring ~6 N/mm (est, "
               "pod.shock_travel)", "e.g. 1/16 RC (WLtoys 144001-class) shock; VERIFY length, stroke, spring",
               V.shock(math.hypot(ty - sy, tz - sz)), Pos(sum(xs) / 2, sy, sz) * Rot(-ang, 0, 0))
    return h


def shock_travel() -> dict:
    """Each shock's static length, motion ratio and the wheel travel its stroke allows; the spring both need for the
    wheel rate (k (mr_in^2 + mr_out^2) = POD_RATE)."""
    out, mr2 = {}, 0.0
    for name, (lo, hi) in (("inner", D.SHOCK_IN), ("outer", D.SHOCK_OUT)):
        l0, l1 = _shock_len(lo, hi, 0.0), _shock_len(lo, hi, 1.0)
        mr = l0 - l1
        mr2 += mr * mr
        out.update({f"{name}_static": l0, f"{name}_motion_ratio": mr,
                    f"{name}_bump_travel": (l0 - (D.SHOCK_L - D.SHOCK_STROKE)) / mr, f"{name}_droop_travel": (D.SHOCK_L - l0) / mr})
    out["spring_at_each_shock"] = D.POD_RATE / mr2
    return out


def printed() -> list[tuple[str, Part, str, int, str]]:
    return [("motor_cup", motor_cup(), "PETG", 60, "end wall down"),
            ("pod_arm", arm(), "PETG", 60, "outer face (x 49) down"),
            ("outer_plate", outer_plate(), "PETG", 60, "housing lip down"),
            ("pod_mount", pod_mount(), "PETG", 50, "base down"),
            ("wheel_guard", wheel_guard(), "PETG", 40, "front panel down")]


def pod_to_robot(th_deg: float, bump: float = 0.0) -> Location:
    """POD frame -> ROBOT for the wheel at th_deg (the static pose; bump moves only the unsprung parts)."""
    th = math.radians(th_deg)
    return Location((D.L_WHEEL * math.cos(th), D.L_WHEEL * math.sin(th), D.WHEEL_R), (0, 0, th_deg + 180))


def arm_motion(bump: float) -> Location:
    """Location of the unsprung parts (the fork, cup, motor, wheel) in the POD frame when the wheel is `bump` mm above
    its static height: a rotation about the pivot shaft (its lever: the shaft's horizontal distance from the axle)."""
    a = math.degrees(math.asin(bump / PY))
    return Pos(0, PY, PZ) * Rot(-a, 0, 0) * Pos(0, -PY, -PZ)
