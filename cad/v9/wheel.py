"""The v9 wheel: 140 mm omni, two staggered rows of six 36 mm rollers.
POD frame: X along the axle inward, Z up. Three PLA hubs split on the row
planes hold twelve M3 × 50 screw axles. Keyed medium-TPU sleeves fit separate PLA
cores. The 1 mm coned tips lie outside the alternating contact transition.
Hub A bolts directly to the AK40-10's output flange (3 x M2.5, PCD 27);
its large access bore admits the heads and driver. Two owned 6804 bearings use 20 mm support interfaces, to be integrated
after the wheel bench test. Hub A has a removable bearing cartridge.
"""

from __future__ import annotations

import math
from functools import lru_cache

from build123d import (Align, Axis, Box, Cylinder, Location, Part, Plane, Polygon, Pos, Rot, RegularPolygon, chamfer, fillet, extrude, revolve)

import dims as D
import hw
import vendor as V

R, d = D.WHEEL_R, D.ROLLER_AXIS_R
HL, CONE = D.ROLLER_HALF, D.ROLLER_CONE
R_END = 8.0                       # roller radius at the tip of the cone
DRUM = (30.0, 33.0)               # drum bore / outer radius in the roller zones
X_ZONE = 36.6                     # rollers stay within |x| < 38 (row plane +-18)
SPOKE_HX = 12.0                   # spoke half thickness along the axle
JOINT_R = 38.0                    # radius of the hub joint screws
N = D.ROLLERS_PER_ROW
PITCH = 360.0 / N
PIN_CHAMFER = 0.4
BRG = (20.0, 32.0, 7.0)  # owned 6804 bearings; wheel-only interfaces
JOURNAL_R = 4.0
JOURNAL_LENGTH = 1.5               # stand the core body off the PLA cheeks
END_FLOAT = 0.6                   # total travel between the two rigid stops
STOP_S = HL + JOURNAL_LENGTH + END_FLOAT / 2
ROLLER_CLEARANCE = 1.3            # radial relief; verified as surface distance below
MIN_BODY_CLEARANCE = 0.5          # excludes the intended journal/thrust contact
SPOKE_EDGE_BREAK = 1.0
WEB_ROOT_FILLET = 2.0             # additive coves at both sides of the thin cheek neck
POCKET_TIP_R = R_END + 1.0         # end relief follows the core tip, not the TPU barrel


def r_barrel(s: float) -> float:
    return math.sqrt(R * R - s * s) - d


def _profile(offset: float = 0.0, tread: bool = False, core: bool = False):
    """(radius, s) outline of the roller (offset grows it). tread: only the barrel shell; core: minus the tread."""
    s_b = HL - CONE
    if tread:
        outer = [(r_barrel(s), s) for s in _lin(-s_b, s_b, 15)]
        inner = [(r_barrel(s) - D.TREAD_T, s) for s in _lin(s_b, -s_b, 15)]
        return outer + inner
    sub = D.TREAD_T if core else 0.0
    bore = 1.8
    pts = [(bore, -HL - offset), (R_END + offset, -HL - offset)]
    pts += [(r_barrel(s) - sub + offset, s) for s in _lin(-s_b, s_b, 15)]
    pts += [(R_END + offset, HL + offset), (bore, HL + offset)]
    return pts


def _lin(a, b, n):
    return [a + (b - a) * k / (n - 1) for k in range(n)]


def _revolve(pts) -> Part:
    return revolve(Plane.XZ * Polygon(*pts, align=None), Axis.Z)


@lru_cache(maxsize=None)
def roller_core() -> Part:
    """PLA roller core, axis Z; three keyways stop the tread creeping around it."""
    p = _revolve(_profile(core=True)) - Cylinder(1.8, 2 * HL + 2)
    for key in tread_keys():
        p -= key
    for sign in (-1, 1):
        p += Cylinder(JOURNAL_R, JOURNAL_LENGTH, align=(Align.CENTER,Align.CENTER,Align.MIN)).moved(Pos(0,0,sign*HL) * Rot(180 if sign < 0 else 0,0,0))
    p -= Cylinder(1.8, 2 * (HL + JOURNAL_LENGTH) + 2)
    p.label = "roller_core"
    return p


@lru_cache(maxsize=None)
def roller_tread() -> Part:
    """Separate medium TPU sleeve. Barrel shape retains it axially; keys resist creep.

    Nominal fit first: softness / shrinkage is not known until the fit coupons
    are printed. No chemical bond between PLA and TPU is required.
    """
    p = _revolve(_profile(tread=True))
    for key in tread_keys():
        p += key & _revolve(_profile(core=True, offset=0.2))
    p.label = "roller_tread"
    return p


def tread_keys():
    """Open axial grooves under the tread, terminating 2 mm before its ends."""
    return [Box(8.0, 2.4, 30.0).moved(Rot(0, 0, angle) * Pos(17.0, 0, 0))
            for angle in (0, 120, 240)]


@lru_cache(maxsize=None)
def _roller_env(clear: float) -> Part:
    return _revolve([(0.0, -HL - clear)] + _profile(offset=clear)[1:-1] + [(0.0, HL + clear)])


@lru_cache(maxsize=None)
def _roller_pocket() -> Part:
    """Running relief with rounded end corners, leaving reinforcing web coves.

    Rounding the *cutter* removes less PLA at the cheek neck: these are
    additive root fillets, not bevels cut into the already thin triangle.
    The barrel-sized cut stops before the thrust block: taper the relief to
    the core tip so it cannot scoop a slot under the triangular support.
    The blend starts beyond the TPU's furthest axial travel.
    """
    sb = HL-CONE
    body_end = HL + END_FLOAT/2
    p = _revolve([(0,-STOP_S), (POCKET_TIP_R,-STOP_S)]
                 + [(r_barrel(max(-sb,min(sb,t)))+ROLLER_CLEARANCE,t)
                    for t in _lin(-body_end,body_end,37)]
                 + [(POCKET_TIP_R,STOP_S), (0,STOP_S)])
    ends = [e for e in p.edges() if e.geom_type.name == "CIRCLE"
            and abs(abs(e.center().Z)-STOP_S) < 1e-5]
    assert len(ends) == 2, "Both roller-pocket end rings must be rounded"
    return fillet(ends, WEB_ROOT_FILLET)


def roller_locs(row: int) -> list[Location]:
    """Roller j of row (0 = outer, x = -20; 1 = inner, x = +20): axis along local Z, centre at local (0, d, 0)."""
    return [Pos(D.ROW_X[row], 0, 0) * Rot(PITCH * j + PITCH / 2 * row, 0, 0) * Pos(0, d, 0) for j in range(N)]


def _spokes(row: int) -> Part:
    """Six spokes: clipped, bevelled noses retain the broad roots and axle blocks."""
    out = None
    for j in range(N):
        g = PITCH * j + PITCH / 2 * row + PITCH / 2            # between roller j and j + 1
        prism = extrude(Plane.YZ * Polygon((DRUM[0] - 1, -14.0), (56.0, -14.0), (60.0, -10.0), (60.0, 10.0), (56.0, 14.0), (DRUM[0] - 1, 14.0),
                                           align=None), SPOKE_HX, both=True)
        # Beyond the joint pads, taper to the axle blocks' 12 mm width.
        # The old full-width cheeks left pointed fins beside the roller ends.
        taper = extrude(Plane.XY * Polygon((-SPOKE_HX, DRUM[0]-1), (SPOKE_HX, DRUM[0]-1),
                                           (SPOKE_HX, 45), (6, 60), (-6, 60), (-SPOKE_HX, 45),
                                           align=None), 15, both=True)
        prism &= taper
        # Restrict edge breaks to the nose: chamfering the drum intersection
        # would interfere with the structural 2 mm root fillets below.
        prism = chamfer([e for e in prism.edges() if e.center().Y >= 56.0], SPOKE_EDGE_BREAK)
        prism = prism.moved(Pos(D.ROW_X[row], 0, 0) * Rot(g, 0, 0))
        out = prism if out is None else out + prism
    for y, z in _joint_pts(row):
        out += _x_cyl(7.0, D.ROW_X[row] - SPOKE_HX, D.ROW_X[row] + SPOKE_HX).moved(Pos(0,y,z))
        out += hw.cove_ring(7.0,2.0).moved(Pos(D.ROW_X[row]-SPOKE_HX+2,y,z)*hw.rot_to("x"))
    for loc in roller_locs(row):
        for lo, hi in ((-23.5,-20.0),(20.0,23.5)):
            block = Cylinder(6.0, hi-lo, align=(Align.CENTER,Align.CENTER,Align.MIN)).moved(Pos(0,0,lo))
            out += block.moved(loc)
    for loc in roller_locs(row):
        out -= _roller_env(0.8).moved(loc)
        out -= Cylinder(1.7, 51).moved(loc)
        out -= Cylinder(3.6, 45, align=(Align.CENTER,Align.CENTER,Align.MAX)).moved(loc*Pos(0,0,-23.5))
        out -= extrude(RegularPolygon(5.8 / math.sqrt(3),6),2.6).moved(loc*Pos(0,0,23.5))
        out -= Cylinder(3.6,45,align=(Align.CENTER,Align.CENTER,Align.MIN)).moved(loc*Pos(0,0,26.1))
    return out




def _joint_pts(row: int):
    for j in range(N):
        g = math.radians(PITCH * j + PITCH / 2 * row + PITCH / 2)
        yield JOINT_R * math.cos(g), JOINT_R * math.sin(g)


def _x_cyl(r: float, x0: float, x1: float) -> Part:
    lo, hi = min(x0, x1), max(x0, x1)
    return Cylinder(r, hi - lo, rotation=(0, 90, 0)).moved(Pos((lo + hi) / 2, 0, 0))


def _joint_holes(row: int, x0: float, x1: float, head: bool) -> Part:
    """Joint screw holes along x through the row's spokes between x0 (head face if head) and x1."""
    out = None
    for y, z in _joint_pts(row):
        if head:   # clearance + pan-head counterbore at x0
            h = _x_cyl(D.CLR_M3 / 2, x0, x1) + _x_cyl(D.CB_M3[0] / 2, x0, x0 + math.copysign(D.CB_M3[1], x1 - x0))
        else:
            h = _x_cyl(D.TAP_M3 / 2, x0, x1)
        h = h.moved(Pos(0, y, z))
        out = h if out is None else out + h
    return out


def _drum(x0: float, x1: float) -> Part:
    return Cylinder(DRUM[1], x1 - x0, rotation=(0, 90, 0)).moved(Pos((x0 + x1) / 2, 0, 0)) - \
        Cylinder(DRUM[0], x1 - x0 + 1, rotation=(0, 90, 0)).moved(Pos((x0 + x1) / 2, 0, 0))


def _slab(x0, x1):
    return Box(x1 - x0, 200, 200).moved(Pos((x0 + x1) / 2, 0, 0))


@lru_cache(maxsize=None)
def hub(which: str) -> Part:
    """'A' outer (x -41 .. -20), 'B' middle (-20 .. 20), 'C' inner (20 .. 48)."""
    xo, xi = D.ROW_X
    x_flange = D.X_FACE                                      # motor flange contact plane
    if which == "A":
        p = _drum(-X_ZONE, xo) + (_spokes(0) & _slab(xo - SPOKE_HX, xo))
        p += _x_cyl(DRUM[1], D.X_DRUM_OUT, -X_ZONE)                  # end plate, outside the rollers' reach
        p += _drum(-X_ZONE - 0.5, x_flange)
        p += _x_cyl(DRUM[1], *D.X_DRIVE_PLATE)            # ... and its centre up to the hub flange
        p -= _x_cyl(16.5, -42, D.X_DRIVE_PLATE[0])
        for angle in (30,150,270):
            y,z=D.polar(26,angle)
            p += _x_cyl(4.5,-41,-31).moved(Pos(0,y,z))
            p -= _x_cyl(1.4,-41.1,-31).moved(Pos(0,y,z))
        for angle in D.AK_OUT_ANGLES:
            y,z=D.polar(D.AK_OUT_PCD/2,angle)
            p -= _x_cyl(D.CLR_M25/2,D.X_DRIVE_PLATE[0]-.1,x_flange+.1).moved(Pos(0,y,z))
        p -= _joint_holes(0, xo - SPOKE_HX, xo, head=True)
    elif which == "B":
        p = _drum(xo, xi) + (_spokes(0) & _slab(xo, xo + SPOKE_HX)) + (_spokes(1) & _slab(xi - SPOKE_HX, xi))
        p -= _joint_holes(0, xo, xo + SPOKE_HX - 1.5, head=False)            # 1.5 mm floors (10.2 mm of thread used)
        p -= _joint_holes(1, xi, xi - SPOKE_HX + 1.5, head=False)
    else:
        bx0, bx1 = D.X_BRG
        shoulder_start = bx0 - 2.5  # full 2.5 mm axial bearing stop, not a thin printed skin
        p = _drum(xi, shoulder_start) + (_spokes(1) & _slab(xi, xi + SPOKE_HX))
        p += _x_cyl(DRUM[1], shoulder_start - 0.5, bx1) - _x_cyl(13.5, shoulder_start - 1, bx1 + 1)
        p -= _x_cyl(16.0, bx0, bx1 + 0.1)              # bearing seat (outer race)
        p -= hw.lead_in(16.0).moved(Pos(bx1, 0, 0) * hw.rot_to("x"))   # lead-in, pressed from the arm side
        p -= _x_cyl(13.5, shoulder_start - 1, bx0 + 0.01)                    # shoulder on the outer race's outer side
        p -= _joint_holes(1, xi + SPOKE_HX, xi, head=True)
    # Concave seams: round the perpendicular drum/spoke intersections.
    seams = [e for e in p.edges() if e.geom_type.name == "LINE" and e.length > 5
             and 32.7 < math.hypot(e.center().Y,e.center().Z) < 33.3]
    if seams:
        p = fillet(seams, 2.0)
    # Recut the *finished filleted* hub around the tread's complete axial
    # envelope, through the entire axial travel. Only the small journal ends
    # may touch the stops at +/-STOP_S; the cone and tread never act as stops.
    # Open the pocket past the old sharp lip at +/-19.5 mm.
    for row in (0,1):
        for loc in roller_locs(row):
            p -= _roller_pocket().moved(loc)
    p.label = f"wheel_hub_{which}"
    return p


def hardware() -> hw.Hardware:
    """Wheel hardware in the POD frame: pins, the two joints' screws, three direct flange screws and two 6804 bearings."""
    h = hw.Hardware()
    for row in (0, 1):
        for loc in roller_locs(row):
            h.screw("PAN",50,loc*Pos(0,0,-23.5)*Rot(180,0,0))
            h.nut(loc*Pos(0,0,23.5))
        xo, xi = D.ROW_X
        for y, z in _joint_pts(row):
            if row == 0:   # head in hub A's counterbore, shank pointing inward (+x) into hub B
                h.screw("PAN", 20, Pos(xo - SPOKE_HX + D.CB_M3[1], y, z) * hw.rot_to("-x"))
            else:          # head in hub C's counterbore, shank pointing outward (-x) into hub B
                h.screw("PAN", 20, Pos(xi + SPOKE_HX - D.CB_M3[1], y, z) * hw.rot_to("x"))
    for angle in D.AK_OUT_ANGLES:
        y, z = D.polar(D.AK_OUT_PCD / 2, angle)
        h.screw("PAN", 6, Pos(D.X_DRIVE_PLATE[0], y, z) * hw.rot_to("-x"), size="M2.5")
    h.part("bearing 6804", "6804 20 x 32 x 7 mm", "owned; SKF 61804 dimensions", hw.bearing_shape(*BRG), Pos(sum(D.X_BRG)/2,0,0)*hw.rot_to("x"))
    h.part("bearing 6804", "6804 20 x 32 x 7 mm", "owned; SKF 61804 dimensions", hw.bearing_shape(*BRG), Pos(-47,0,0)*hw.rot_to("x"))
    for angle in (30,150,270):
        y,z=D.polar(26,angle)
        h.screw("PAN",12,Pos(-44,y,z)*hw.rot_to("-x"))
    return h


def printed() -> list[tuple[str, Part, str, int, str]]:
    """(name, part in POD frame, material, infill %, print orientation) for one wheel, with per-wheel qty."""
    return [("wheel_cartridge", bearing_cartridge(), "PLA", 80, "flange down; housing up"),
            ("wheel_hub_A", hub("A"), "PLA", 60, "bearing boss down"),
            ("wheel_hub_B", hub("B"), "PLA", 60, "a split face down (pin channels up on the other face)"),
            ("wheel_hub_C", hub("C"), "PLA", 60, "bearing housing up, split face down")]


@lru_cache(maxsize=None)
def bearing_cartridge():
    p = _x_cyl(33,-44,-41) + _x_cyl(22,-51,-43.5)
    p -= _x_cyl(16,-51.1,-43.5)
    p -= _x_cyl(13.5,-43.6,-40.9)
    for angle in (30,150,270):
        y,z=D.polar(26,angle)
        p -= _x_cyl(1.7,-44.1,-40.9).moved(Pos(0,y,z))
    p.label="wheel_cartridge"
    return p
