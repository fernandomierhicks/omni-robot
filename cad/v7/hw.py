"""Hardware: fasteners modelled to ISO dimensions, bearings, pins, and the hole cutters that go with them.

Every screw is placed with screw(kind, length, loc). loc puts the fastener's frame in the part's frame:
origin at the underside of the head (the bearing face), shank along -Z. Nuts and washers: origin at the face that
seats on the part, body along +Z (away from the part). So a screw through a plate whose top is at z = t is
screw("PAN", L, Pos(0, 0, t)) and its nut on the far side is nut(Location((0, 0, 0), (180, 0, 0))).

Shanks are modelled at 2.6 mm (the M3 minor diameter): they sit in 2.8 mm thread-forming holes without counting as
interference, and a screw too long for its hole shows up in the clash check. Each screw also carries a probe: a thin
tube around its shank, between the tapped-hole and clearance-hole radii. Material inside the probe is thread the
screw cuts or engages (a 2.8 mm hole, a nut, a vendor's tapped hole); a 3.4 mm clearance hole leaves it empty. The
build's engagement check measures it (Fernando, Oct 21: >= 6 mm in printed parts).

Screws are pan head (ISO 7045 / 14583) and flat head (ISO 7046 / 14584, 90 deg): Fernando's stock, no socket heads.
Identical hardware shares one shape, so the assembly STEP instances it (one part file per screw size in SolidWorks).

McMaster numbers: McMaster's site can't be read here, so the BOM gives the McMaster product family and the exact
size; the full part number is a lookup (marked so in BOM.md).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache

from build123d import (Align, Axis, Box, Cone, Cylinder, Location, Part, Pos, RegularPolygon, Rot, extrude)

import dims as D

CB = (Align.CENTER, Align.CENTER, Align.MIN)
CT = (Align.CENTER, Align.CENTER, Align.MAX)

# kind: (description, McMaster family, {size: (head dia, head height)})
SCREWS = {
    "PAN": ("M3 pan head screw, ISO 7045 (Phillips) / ISO 14583 (Torx), 18-8 stainless",
            "McMaster 92000A (18-8 SS pan head Phillips, metric)", {"M2": (4.0, 1.3), "M2.5": (5.0, 1.5), "M3": (5.6, 1.8)}),
    "FLAT": ("M3 flat head screw 90 deg, ISO 7046 (Phillips) / ISO 14584 (Torx), 18-8 stainless",
             "McMaster 92010A (18-8 SS flat head Phillips, metric)", {"M2": (3.8, 1.2), "M2.5": (4.7, 1.5), "M3": (5.5, 1.65)}),
    "SSCS": ("M3 set screw, cup point, ISO 4029", "McMaster 92015A (18-8 SS set screw, metric)", {}),
}
SHANK_R = {"M2": 0.8, "M2.5": 1.0, "M3": 1.2}   # modelled shank: under the minor diameter (fits tapped holes in vendor STEPs)
PROBE_R = {"M2": (0.95, 1.05), "M2.5": (1.2, 1.3), "M3": (1.5, 1.6)}   # between the tap and clearance radii
STD_LENGTHS = (3, 4, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 25, 30, 35, 40, 45, 50)


@dataclass
class HwItem:
    key: str           # BOM line key
    desc: str
    source: str
    shape: Part
    loc: Location      # in the owning part's frame
    probe: Part | None = None   # screws: the engagement probe, same frame as shape
    size: str = ""


@lru_cache(maxsize=None)
def _screw_shape(kind: str, length: float, size: str) -> tuple[Part, Part | None]:
    """(screw, probe), origin at the head's bearing face (flat heads: the top face, flush with the part), shank -Z."""
    sr = SHANK_R[size]
    if kind == "SSCS":
        shape = Cylinder(sr, length, align=CT)
        h0 = 0.0
    else:
        hd, hh = SCREWS[kind][2][size]
        if kind == "FLAT":
            shape = Cone(sr, hd / 2, hh, align=CT) + Cylinder(sr, length - hh, align=CT).moved(Pos(0, 0, -hh))
            h0 = hh
        else:
            shape = Cylinder(hd / 2, hh, align=CB) + Cylinder(sr, length, align=CT)
            h0 = 0.0
    shape.label = f"{size}x{length:g} {kind}"
    r0, r1 = PROBE_R[size]
    probe = (Cylinder(r1, length - h0, align=CT) - Cylinder(r0, length + 1, align=CT).moved(Pos(0, 0, 0.5))).moved(Pos(0, 0, -h0))
    return shape, probe


@lru_cache(maxsize=None)
def _nut_shape(kind: str) -> Part:
    af, h = {"hex": (5.5, 2.4), "nyloc": (5.5, 4.0)}[kind]
    shape = extrude(RegularPolygon(af / 2 / math.cos(math.pi / 6), 6), h) - Cylinder(1.4, 3 * h)
    shape.label = f"M3 {kind} nut"
    return shape


@lru_cache(maxsize=None)
def _washer_shape() -> Part:
    shape = Cylinder(3.5, 0.5, align=CB) - Cylinder(1.6, 2)
    shape.label = "M3 washer"
    return shape


class Hardware:
    """Collects hardware for one printed part or subassembly (in that part's frame)."""

    def __init__(self):
        self.items: list[HwItem] = []

    def screw(self, kind: str, length: float, loc: Location, size="M3"):
        if length not in STD_LENGTHS:
            raise ValueError(f"non-standard {size} length {length}")
        desc, src, _ = SCREWS[kind]
        if size != "M3":
            src = src.replace("metric", f"metric, {size}")
        shape, probe = _screw_shape(kind, float(length), size)
        self.items.append(HwItem(f"{size} x {length} {kind}", desc.replace("M3", size), src, shape, loc, probe, size))

    def nut(self, loc: Location, kind="hex", size="M3"):
        desc, src = {"hex": ("M3 hex nut, ISO 4032, 18-8 stainless", "McMaster 91828A (18-8 SS hex nut, metric)"),
                     "nyloc": ("M3 nylon-insert locknut, ISO 10511, 18-8 stainless",
                               "McMaster 93625A (18-8 SS nylon-insert locknut, metric)")}[kind]
        self.items.append(HwItem(f"{size} {kind} nut", desc, src, _nut_shape(kind), loc))

    def washer(self, loc: Location, size="M3"):
        self.items.append(HwItem("M3 washer", "M3 flat washer, ISO 7089, 18-8 stainless",
                                 "McMaster 93475A (18-8 SS washer, metric)", _washer_shape(), loc))

    def part(self, key: str, desc: str, source: str, shape: Part, loc: Location):
        """Any other purchased hardware (bearings, pins, springs) placed in this part."""
        shape.label = key
        self.items.append(HwItem(key, desc, source, shape, loc))

    def extend(self, other: "Hardware", loc: Location = Location()):
        for it in other.items:
            self.items.append(HwItem(it.key, it.desc, it.source, it.shape, loc * it.loc, it.probe, it.size))


# --- hole cutters (solids to subtract), modelled at the origin, axis along Z ----------------------------------------------

def tap(depth: float, top: float = 0.0, d: float = D.TAP_M3, entry: str | None = None) -> Part:
    """Thread-forming hole of `depth` going down from z = top. entry 'top' / 'bottom': a 0.5 mm 90 deg countersink
    where the screw enters (it starts straight, and the printed lip can't hold the joint open). Only where the hole
    has thread to spare: the countersink isn't thread."""
    h = Cylinder(d / 2, depth + 0.02, align=CT).moved(Pos(0, 0, top + 0.01))
    if entry == "top":
        h += Cone(d / 2 + D.TAP_CSK, d / 2, D.TAP_CSK, align=CT).moved(Pos(0, 0, top + 0.01))
    elif entry == "bottom":
        h += Cone(d / 2, d / 2 + D.TAP_CSK, D.TAP_CSK, align=CB).moved(Pos(0, 0, top - depth - 0.01))
    return h


def clear(length: float, top: float = 0.0, d: float = D.CLR_M3) -> Part:
    return Cylinder(d / 2, length + 0.02, align=CT).moved(Pos(0, 0, top + 0.01))


def cbore(length: float, top: float = 0.0, cb=D.CB_M3) -> Part:
    """Through hole with a pan-head counterbore at the top face (z = top)."""
    return clear(length, top) + Cylinder(cb[0] / 2, cb[1] + 0.01, align=CT).moved(Pos(0, 0, top + 0.01))


def csink(length: float, top: float = 0.0) -> Part:
    """Through hole with a 90 deg countersink at z = top (ISO 7046 M3 head, which sits 0.5 mm under the face)."""
    return clear(length, top) + Cone(0.0, D.CSK_M3 / 2, D.CSK_M3 / 2, align=CT).moved(Pos(0, 0, top + 0.01))


def nut_trap(top: float = 0.0, depth: float = D.NUT_M3[1], af: float = D.NUT_M3[0], angle: float = 30.0) -> Part:
    return extrude(RegularPolygon(af / 2 / math.cos(math.pi / 6), 6), depth + 0.01).rotate(Axis.Z, angle).moved(
        Pos(0, 0, top - depth))


def lead_in(r: float, c: float = 0.5, z: float = 0.0, up: bool = True) -> Part:
    """Cutter: a c x 45 deg lead-in chamfer at the mouth of a bore of radius r whose face is at z = z (the bore goes
    -Z from the face if up, +Z if not). For bearing seats and press fits."""
    cone = Cone(r, r + c, c + 0.01, align=CB if up else CT)
    return (cone if up else cone.rotate(Axis.X, 180)).moved(Pos(0, 0, z - c if up else z + c))


def edge_chamfer(r: float, c: float = 0.5, z: float = 0.0, up: bool = True) -> Part:
    """Cutter: a c x 45 deg chamfer on the outer edge of a boss / disc of radius r, at its end face z (the solid is
    below z if up, above it if not)."""
    ring = Cylinder(r + 1.0, c + 0.01, align=CT if up else CB).moved(Pos(0, 0, z + (0.005 if up else -0.005)))
    keep = Cone(r, r - c, c + 0.01, align=CT if up else CB).moved(Pos(0, 0, z)) if up else \
        Cone(r - c, r, c + 0.01, align=CB).moved(Pos(0, 0, z))
    return ring - keep


def cove(r: float, length: float) -> Part:
    """r x r square minus the quarter circle centred at (r, r), along +Z from 0 to length: a fillet's material at an
    inside corner, or the cutter that rounds an outside edge (cove_at places it)."""
    e = 0.05                  # reaches 0.05 mm into the two faces (coplanar faces break OCC's fuse)
    return Box(r + e, r + e, length, align=(Align.MIN, Align.MIN, Align.MIN)).moved(Pos(-e, -e, 0)) - \
        Cylinder(r, length + 2, align=CB).moved(Pos(r, r, -1))


def cove_ring(r0: float, rf: float, outward: bool = True) -> Part:
    """Fillet ring at the foot of a boss of radius r0 standing on the plane z = 0 (boss along +Z, face below):
    the material between the boss, the face and a torus of radius rf. outward=False: the boss is a bore of radius r0
    (material outside it), and the ring rounds the bore's mouth when subtracted (the face is the solid's top)."""
    n = 10
    if outward:
        pts = [(r0, 0.0), (r0 + rf, 0.0)] + [(r0 + rf - rf * math.sin(math.pi / 2 * k / n), rf - rf * math.cos(math.pi / 2 * k / n))
                                            for k in range(1, n + 1)]
    else:
        pts = [(r0, 0.0), (r0 + rf, 0.0)] + [(r0 + rf - rf * math.sin(math.pi / 2 * k / n), -rf + rf * math.cos(math.pi / 2 * k / n))
                                            for k in range(1, n + 1)]
    from build123d import Plane, Polygon, revolve
    return revolve(Plane.XZ * Polygon(*pts, align=None), Axis.Z)


def cove_at(r: float, corner, u, v, w, length: float) -> Part:
    """cove(r) placed at a straight edge: corner = a point on the edge, u and v = unit directions away from the edge
    along its two faces, w = the edge direction (the strip runs from corner along w for length). For an inside corner
    u and v point into the empty quadrant (add it: a fillet); for an outside edge they point into the solid (subtract
    it: a round)."""
    from build123d import Plane, Vector
    u, v, w, c = Vector(*u), Vector(*v), Vector(*w), Vector(*corner)
    z = u.cross(v)                        # the frame (u, v, u x v) is right-handed; if u x v runs against w, the strip
    if z.dot(w) < 0:                      # starts at the far end and runs back over the same edge
        c = c + w * length
    return cove(r, length).moved(Location(Plane(c, x_dir=u, z_dir=z)))


@lru_cache(maxsize=None)
def bearing_shape(d: float, D_: float, b: float) -> Part:
    """Ball bearing envelope, axis Z, centred on z (faces at +-b/2)."""
    return Cylinder(D_ / 2, b) - Cylinder(d / 2, b + 1)


@lru_cache(maxsize=None)
def _pin_shape(d: float, length: float) -> Part:
    return Cylinder(d / 2, length)


def bearing(hw: Hardware, size: str, loc: Location):
    d, D_, b, desc, src = {
        "61808": (*D.BRG_61808, "61808-2RS thin-section ball bearing 40 x 52 x 7", "generic (VXB / Amazon); McMaster 5972K (search 40 mm ID x 52 mm OD)"),
        "623": (*D.BRG_623, "623-2RS ball bearing 3 x 10 x 4", "generic; McMaster 6661K (search 3 mm ID x 10 mm OD)"),
    }[size]
    hw.part(f"bearing {size}-2RS", desc, src, bearing_shape(d, D_, b), loc)


def pin(hw: Hardware, d: float, length: float, loc: Location, desc: str, source: str):
    """A plain pin along Z, centred on its origin."""
    hw.part(f"pin {d:g} x {length:g}", desc, source, _pin_shape(d, length), loc)


def rot_to(axis: str) -> Location:
    """Rotation that turns a part's +Z into the given axis ('x', '-x', 'y', '-y', 'z', '-z')."""
    return {"z": Location(), "-z": Rot(180, 0, 0), "x": Rot(0, 90, 0), "-x": Rot(0, -90, 0),
            "y": Rot(-90, 0, 0), "-y": Rot(90, 0, 0)}[axis]
