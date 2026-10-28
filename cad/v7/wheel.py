"""The v7 wheel: printed 127 mm omni, two staggered rows of five 36 mm rollers (design.md 25.3), built around an
in-wheel Damiao DM-2325. POD frame (x along the axle, inward +).

  - Rollers: barrel profile r(s) = sqrt(R^2 - s^2) - d, so the wheel's envelope is a true circle; coned ends
    (design.md 25.3: without cones neighbours overlap near the hub). PETG core on a 3 mm steel pin, TPU 95A tread
    sleeve stretched over the barrel.
  - Hub: three PETG prints split on the two row planes (x = -20, +20), so each roller pin lies in a split plane and
    is clamped in two half-channels. Hub A (outer) bolts to a Pololu #1999 hub on the motor's 6 mm D shaft; hub C
    (inner) carries a 61808 bearing on the arm's boss. Five M3 x 20 pan heads per joint, through the spokes.
  - The hub is a drum (r 23-26.5) around the motor cup, so the motor sits inside the wheel and nothing of the drive
    reaches inboard of the arm (the sim's motor-behind-the-wheel layout reached r 36 mm, through the centre deck).
"""

from __future__ import annotations

import math
from functools import lru_cache

from build123d import (Align, Axis, Box, Cylinder, Location, Part, Plane, Polygon, Pos, Rot, extrude, revolve)

import dims as D
import hw
import vendor as V

R, d = D.WHEEL_R, D.ROLLER_AXIS_R
HL, CONE = D.ROLLER_HALF, D.ROLLER_CONE
R_END = 8.0                       # roller radius at the tip of the cone
DRUM = (23.0, 26.5)               # drum bore / outer radius in the roller zones
X_ZONE = 38.6                     # rollers stay within |x| < 38 (row plane +-18)
SPOKE_HX = 12.0                   # spoke half thickness along the axle
JOINT_R = 36.0                    # radius of the hub joint screws
N = D.ROLLERS_PER_ROW
PITCH = 360.0 / N
PIN_CHAMFER = 0.4


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
    bore = 1.6
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
    """PETG roller core, axis Z, centred. 3.2 mm bore for the steel pin."""
    p = _revolve(_profile(core=True)) - Cylinder(1.6, 2 * HL + 2)
    p.label = "roller_core"
    return p


@lru_cache(maxsize=None)
def roller_tread() -> Part:
    """TPU 95A tread sleeve over the core's barrel."""
    p = _revolve(_profile(tread=True))
    p.label = "roller_tread"
    return p


@lru_cache(maxsize=None)
def _roller_env(clear: float) -> Part:
    return _revolve([(0.0, -HL - clear)] + _profile(offset=clear)[1:-1] + [(0.0, HL + clear)])


def roller_locs(row: int) -> list[Location]:
    """Roller j of row (0 = outer, x = -20; 1 = inner, x = +20): axis along local Z, centre at local (0, d, 0)."""
    return [Pos(D.ROW_X[row], 0, 0) * Rot(PITCH * j + PITCH / 2 * row, 0, 0) * Pos(0, d, 0) for j in range(N)]


def _spokes(row: int) -> Part:
    """The row's five spokes (both halves), carved around the rollers with 1 mm clearance, with the pins' holes."""
    out = None
    for j in range(N):
        g = PITCH * j + PITCH / 2 * row + PITCH / 2            # between roller j and j + 1
        prism = extrude(Plane.YZ * Polygon((DRUM[0] + 1, -8.0), (58.0, -11.0), (58.0, 11.0), (DRUM[0] + 1, 8.0),
                                           align=None), SPOKE_HX, both=True)
        prism = prism.moved(Pos(D.ROW_X[row], 0, 0) * Rot(g, 0, 0))
        out = prism if out is None else out + prism
    for loc in roller_locs(row):
        out -= _roller_env(1.0).moved(loc)
        out -= Cylinder(D.PIN_D / 2 + 0.03, D.PIN_L + 0.2).moved(loc)           # pin half-channels after the split
        # 0.4 mm x 45 deg chamfers on the channels' edges in the split face (a diamond across the split): the hubs
        # print split face down, and the first layer's squash would otherwise pinch the pin
        s = (D.PIN_D / 2 + 0.03 + PIN_CHAMFER) * math.sqrt(2)
        out -= Box(s, s, D.PIN_L + 0.2).rotate(Axis.Z, 45).moved(loc)
    # (where a spoke's face meets a roller's clearance envelope at a grazing angle it ends in a knife edge, 0.1-0.3 mm
    # along the cut-out: no load there, and the slicer drops it; walls.py lists them)
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
    x_flange = D.X_HUB - 9.2                                      # the #1999 hub's mounting face (-37.2)
    if which == "A":
        p = _drum(-X_ZONE, xo) + (_spokes(0) & _slab(xo - SPOKE_HX, xo))
        p += _x_cyl(30.0, D.X_DRUM_OUT, -X_ZONE)                  # end plate, outside the rollers' reach
        p += _x_cyl(DRUM[1], -X_ZONE - 0.5, x_flange)            # ... and its centre up to the hub flange
        p -= _x_cyl(4.0, D.X_DRUM_OUT - 1, x_flange + 1)          # shaft end (at -40)
        for k in range(6):                                        # 6 x M3 pan heads into the hub's tapped holes
            y, z = D.polar(D.HUB_1999["pcd"] / 2, 60 * k)
            p -= (_x_cyl(D.CLR_M3 / 2, D.X_DRUM_OUT - 1, x_flange + 1) + _x_cyl(3.1, D.X_DRUM_OUT - 1, D.X_DRUM_OUT + 1.8)).moved(
                Pos(0, y, z))
        p -= _joint_holes(0, xo - SPOKE_HX, xo, head=True)
        p -= hw.edge_chamfer(30.0, 0.5).moved(Pos(D.X_DRUM_OUT, 0, 0) * hw.rot_to("-x"))   # first layer
    elif which == "B":
        p = _drum(xo, xi) + (_spokes(0) & _slab(xo, xo + SPOKE_HX)) + (_spokes(1) & _slab(xi - SPOKE_HX, xi))
        p -= _joint_holes(0, xo, xo + SPOKE_HX - 1.5, head=False)            # 1.5 mm floors (10.2 mm of thread used)
        p -= _joint_holes(1, xi, xi - SPOKE_HX + 1.5, head=False)
    else:
        p = _drum(xi, X_ZONE) + (_spokes(1) & _slab(xi, xi + SPOKE_HX))
        bx0, bx1 = D.X_BRG
        p += _x_cyl(30.0, X_ZONE, bx1) - _x_cyl(DRUM[0], X_ZONE - 1, bx1 + 1)
        p -= _x_cyl(D.BRG_61808[1] / 2, bx0, bx1 + 0.1)              # bearing seat (outer race)
        p -= hw.lead_in(D.BRG_61808[1] / 2).moved(Pos(bx1, 0, 0) * hw.rot_to("x"))   # lead-in, pressed from the arm side
        p -= _x_cyl(24.6, X_ZONE - 1, bx0 + 0.01)                    # shoulder on the outer race's outer side
        p -= _joint_holes(1, xi + SPOKE_HX, xi, head=True)
    p.label = f"wheel_hub_{which}"
    return p


def hardware() -> hw.Hardware:
    """Wheel hardware in the POD frame: pins, the two joints' screws, the #1999 hub and its screws, the 61808."""
    h = hw.Hardware()
    for row in (0, 1):
        for loc in roller_locs(row):
            hw.pin(h, D.PIN_D, D.PIN_L, loc, "3 mm hardened steel pin, 58 mm (cut from 3 mm drill rod)",
                   "McMaster 8893K (3 mm O1 drill rod, cut) or 3 x 60 dowel ground down")
        xo, xi = D.ROW_X
        for y, z in _joint_pts(row):
            if row == 0:   # head in hub A's counterbore, shank pointing inward (+x) into hub B
                h.screw("PAN", 20, Pos(xo - SPOKE_HX + D.CB_M3[1], y, z) * hw.rot_to("-x"))
            else:          # head in hub C's counterbore, shank pointing outward (-x) into hub B
                h.screw("PAN", 20, Pos(xi + SPOKE_HX - D.CB_M3[1], y, z) * hw.rot_to("x"))
    # the #1999 hub: flange face at x -37.2 toward the wheel's end plate, set-screw boss toward the motor
    h.part("Pololu #1999 hub", "Pololu universal aluminium mounting hub, 6 mm shaft, 6 x M3 (pair: #1999)",
           "pololu.com #1999 ($8.95 per 2)", V.hub_1999(), Pos(D.X_HUB - 9.2, 0, 0) * hw.rot_to("-x"))
    for k in range(6):
        y, z = D.polar(D.HUB_1999["pcd"] / 2, 60 * k)
        h.screw("PAN", 8, Pos(D.X_DRUM_OUT + 1.8, y, z) * hw.rot_to("-x"))
    # the hub's two set screws (boss and flange, on one radial line), onto the shaft's flat
    h.screw("SSCS", 3, Pos(D.X_HUB - 2.1, 0, 0) * Rot(60, 0, 0) * Pos(0, 0, 6.0))
    h.screw("SSCS", 10, Pos(D.X_HUB - 6.7, 0, 0) * Rot(60, 0, 0) * Pos(0, 0, 12.7))
    hw.bearing(h, "61808", Pos(sum(D.X_BRG) / 2, 0, 0) * hw.rot_to("x"))
    return h


def printed() -> list[tuple[str, Part, str, int, str]]:
    """(name, part in POD frame, material, infill %, print orientation) for one wheel, with per-wheel qty."""
    return [("wheel_hub_A", hub("A"), "PETG", 60, "outer end plate down"),
            ("wheel_hub_B", hub("B"), "PETG", 60, "a split face down (pin channels up on the other face)"),
            ("wheel_hub_C", hub("C"), "PETG", 60, "bearing housing up, split face down")]
