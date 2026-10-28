"""The whole robot as placed items (ROBOT frame), at a module tilt and a wheel bump.

Item kinds: printed, purchased (vendor parts, bearings, pins, materials), fastener. Groups: base, pod<k> (sprung) /
pod<k>u (unsprung), module, mounts (yokes, cheeks, the face, the audio box), head (the camera bar and what turns with
it: placed at an index pitch, 30 = catch, -40 = arm; the face only at the catch index). The printed-part registry (unique parts, their export frame, material, quantity)
comes from the same calls, so the print list and the assembly can't disagree.

Mirrored pods (front-left, rear-right): the handed prints (arm, cup, mount, guard) are mirror images (`_M` parts).
Every purchased part is the same part, turned into place (a real DM-2325 isn't mirrored), and so is the wheel: its
mirror image is the same wheel turned half a roller pitch about the axle. Identical parts share one shape, so the
STEP instances them.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache

from build123d import Box, Location, Part, Plane, Pos, Rot
from OCP.gp import gp_Trsf

import base as B
import dims as D
import arm_kit as AK
import display as DP
import head as H
import hw
import module as M
import mounts as MT
import pod as P
import vendor as V
import wheel as WH

GROUP_NAMES = {"base": "base", "module": "module (tilting)", "mounts": "mounts (yokes, cheeks, face, audio)",
               "head": "head (camera bar, indexed)"}
_MARKER = Box(0.1, 0.1, 0.1)     # placed like a hardware item to read its ROBOT-frame location
WHEEL_TURN = Rot(180.0 / D.ROLLERS_PER_ROW, 0, 0)        # half a roller pitch about the axle (POD x)
POD_NAMES = {45.0: "FL", 135.0: "RL", 225.0: "RR", 315.0: "FR"}


def group_label(group: str) -> str:
    """Human name of an item group: 'pod0' -> 'pod FL chassis', 'pod0u' -> 'pod FL wheel'."""
    if group.startswith("pod"):
        k = int(group[3:].rstrip("u"))
        return f"pod {POD_NAMES[D.WHEEL_ANGLES[k]]} {'wheel' if group.endswith('u') else 'chassis'}"
    return GROUP_NAMES.get(group, group)


@dataclass
class Item:
    name: str
    kind: str              # printed / purchased / fastener / foam / fabric / ball
    group: str
    shape: Part            # placed, ROBOT frame
    key: str = ""          # BOM / print-list key
    desc: str = ""
    source: str = ""
    material: str = ""
    probe: Part | None = None    # screws: engagement probe, placed like shape
    size: str = ""
    loc: Location | None = None  # hardware: its frame in the ROBOT frame (screws: head's bearing face, shank -Z)


@dataclass
class PrintedPart:
    name: str
    shape: Part            # in its contract frame (ROBOT / POD / MODULE), for STEP export
    material: str
    infill: int
    orientation: str
    qty: int = 0
    frame: str = "ROBOT"


@dataclass
class Robot:
    items: list = field(default_factory=list)
    printed: dict = field(default_factory=dict)

    def add_printed(self, name, shape_local, placed, material, infill, orientation, group, frame):
        pp = self.printed.get(name)
        if pp is None:
            pp = self.printed[name] = PrintedPart(name, shape_local, material, infill, orientation, 0, frame)
        pp.qty += 1
        self.items.append(Item(f"{name}#{pp.qty}", "printed", group, placed, name, material=material))

    def add_hw(self, hwset: hw.Hardware, place, group: str):
        """place(shape, loc) -> the shape placed in the ROBOT frame (loc: its location in the set's frame)."""
        for it in hwset.items:
            kind = "fastener" if it.key.startswith(("M3 ", "M2 ", "M2.5 ")) else "purchased"
            probe = place(it.probe, it.loc) if it.probe is not None else None
            self.items.append(Item(it.key, kind, group, place(it.shape, it.loc), it.key, it.desc, it.source,
                                   probe=probe, size=it.size, loc=place(_MARKER, it.loc).location))


def module_loc(tilt: float) -> Location:
    """MODULE -> ROBOT at a tilt (deg; + tips the funnel toward the thrower)."""
    return Pos(D.PIVOT_R[0], 0, D.PIVOT_R[1]) * Rot(0, tilt, 0) * Pos(-D.PIVOT_M[0], 0, -D.PIVOT_M[1])


def _conj(loc: Location) -> Location:
    """M L M, M the mirror y -> -y: L's position and orientation mirrored about the x-z plane, as a proper rigid
    motion. For a part symmetric about its own x-z plane, part.moved(_conj(L)) is the mirror image of part.moved(L)."""
    t = loc.wrapped.Transformation()
    s = (1.0, -1.0, 1.0)
    vals = [t.Value(i, j) * s[i - 1] * (s[j - 1] if j < 4 else 1.0) for i in range(1, 4) for j in range(1, 5)]
    g = gp_Trsf()
    g.SetValues(*vals)
    return Location(g)


_MIRRORS: dict = {}


def mirrored(shape: Part) -> Part:
    """The shape's mirror image about its frame's x-z plane, made once per shape (so its placements share it)."""
    key = id(shape)
    if key not in _MIRRORS:
        m = shape.mirror(Plane.XZ)
        m.label = shape.label
        _MIRRORS[key] = (shape, m)
    return _MIRRORS[key][1]


@lru_cache(maxsize=None)
def _roller() -> tuple:
    return WH.roller_core(), WH.roller_tread()


def build(tilt: float = 0.0, bump: float = 0.0, parts: str = "all", index: float | None = None,
          kit: str = "catch") -> Robot:
    """parts: 'all', or a comma list of groups to include ('base', 'pods', 'module', 'mounts', 'head'). kit: the
    payload, "catch" (the module, its yokes and their boards; the face on the head's cheeks, the audio box on the
    plate) or "arm" (arm_kit.py: the SO-101 on its riser, the bin, the face and the audio box over it; no module).
    index: the camera bar's pitch (deg), by default the kit's; between the stops (the swap) the face is off."""
    want = set(parts.split(",")) if parts != "all" else {"base", "pods", "module", "mounts", "head"}
    if kit == "arm":
        want.discard("module")
    index = H.INDEX[kit] if index is None else index
    seat = kit if abs(index - H.INDEX[kit]) < 1e-6 else None
    audio = "plate" if kit == "catch" else "shelf"
    r = Robot()
    at = (lambda s, loc: s.moved(loc))
    if "base" in want:
        for name in ("centre", *B.LOBES):
            s = B.deck_piece(name)
            r.add_printed(f"deck_{name}", s, s, "PETG", 30, "flat (posts and bosses up)", "base", "ROBOT")
        sp = B.splice()
        for lobe in B.SPLICED:
            r.add_printed("deck_splice", sp, sp.rotate(_zaxis(), B.LOBES[lobe]), "PETG", 60, "flat", "base", "ROBOT")
        for name, fn, inf, orient in (("battery_tray", B.battery_tray, 30, "flange up"), ("payload_plate", B.payload_plate, 30, "flat"),
                                      ("lidar_riser", B.lidar_riser, 30, "top down"), ("estop_riser", B.estop_riser, 40, "flange down")):
            s = fn()
            r.add_printed(name, s, s, "PETG", inf, orient, "base", "ROBOT")
        so = B.standoff()
        for x, y in D.STANDOFF_XY[len(D.ROD_XY):]:
            r.add_printed("standoff", so, so.moved(Pos(x, y, D.DECK_TOP)), "PETG", 100, "standing", "base", "local")
        bp = B.board_post()
        for nm, (desc, src, x, y, z, w, d, hh, holes, up) in B.BOARDS.items():
            if up:
                continue                    # boards on top stand on the deck's printed bosses
            for dx, dy in holes:
                r.add_printed("board_spacer", bp, bp.moved(Pos(x + dx, y + dy, z - 4.0)), "PETG", 100, "flat", "base", "local")
        for kind, angles in (("lobe", B.LOBES.values()), ("corner", D.WHEEL_ANGLES)):
            s = B.bumper(kind)
            for a in angles:
                r.add_printed(f"bumper_{kind}", s, s.rotate(_zaxis(), a), "TPU 95A", 20, "inner lip down", "base", "ROBOT")
        r.add_hw(B.hardware(), at, "base")
    if "pods" in want:
        core, tread = _roller()
        m = P.arm_motion(bump)
        for k, th in enumerate(D.WHEEL_ANGLES):
            mir = D.POD_MIRROR[th]
            sfx = "_M" if mir else ""
            pod = P.pod_to_robot(th)
            g, gu = f"pod{k}", f"pod{k}u"

            def put(s, loc=Location(), pod=pod, mir=mir):
                """A part symmetric about its own x-z plane at POD loc: turned into place on the mirrored pods."""
                return s.moved(pod * (_conj(loc) if mir else loc))

            def put_h(s, loc=Location(), pod=pod, mir=mir):
                """A handed print at POD loc: its mirror image on the mirrored pods."""
                return (mirrored(s) if mir else s).moved(pod * (_conj(loc) if mir else loc))

            def put_w(s, loc=Location(), pod=pod, mir=mir):
                """A wheel part at its POD loc. The wheel's mirror image is the same wheel turned half a roller pitch
                about the axle, so the mirrored pods get the same prints, turned."""
                return s.moved(pod * ((_conj(m) * WHEEL_TURN) if mir else m) * loc)

            def reg(s, mir=mir):
                return mirrored(s) if mir else s
            # sprung (chassis)
            front = th in (45.0, 315.0)
            for nm, s, inf, orient in (("pod_mount", P.pod_mount(), 50, "base down"),
                                       ("wheel_guard_front" if front else "wheel_guard", P.wheel_guard(front=front), 30,
                                        "front panel down")):
                r.add_printed(nm + sfx, reg(s), put_h(s), "PETG", inf, orient, g, "POD")
            r.add_hw(P.hardware_sprung(), put, g)
            r.add_hw(P.shock_item(bump), put, g)
            # unsprung (arm, cup, motor, wheel)
            for nmp, s, mat, inf, orient in P.printed():
                if nmp in ("pod_mount", "wheel_guard"):
                    continue
                r.add_printed(nmp + sfx, reg(s), put_h(s, m), mat, inf, orient, gu, "POD")
            for nmp, s, mat, inf, orient in WH.printed():
                r.add_printed(nmp, s, put_w(s), mat, inf, orient, gu, "POD")
            for row in (0, 1):
                for loc in WH.roller_locs(row):
                    r.add_printed("roller_core", core, put_w(core, loc), "PETG", 20, "on end", gu, "local")
                    r.add_printed("roller_tread", tread, put_w(tread, loc), "TPU 95A", 100, "on end", gu, "local")
            r.add_hw(WH.hardware(), put_w, gu)
            r.add_hw(P.hardware_unsprung(), lambda s, loc, put=put: put(s, m * loc), gu)
    if "module" in want:
        ml = module_loc(tilt)
        for nmp, s, mat, inf, orient in M.printed():
            r.add_printed(nmp, s, s.moved(ml), mat, inf, orient, "module", "MODULE")
        for nmp, s in M.pads():
            r.items.append(Item(nmp, "foam", "module", s.moved(ml), "catch-pad foam 6 mm",
                                "6 mm low-resilience (slow-recovery) PU foam sheet, cut to the templates in out/step/pad_*.step",
                                "McMaster 8643K (slow-recovery polyurethane foam, 1/4 in) (est)"))
        r.items.append(Item("funnel_net", "fabric", "module", M.net().moved(ml), "funnel net",
                            "grippy, soft mesh, ~0.25 m2, sewn to the 1:1 pattern and laced to the collar and rim",
                            "fabric store (design.md 8: mesh choice open)"))
        r.add_hw(M.hardware(), lambda s, loc, ml=ml: s.moved(ml * loc), "module")
        r.items.append(Item("ball", "ball", "module", V.ball().moved(ml * Pos(D.HOLD[0], 0, D.HOLD[1])), "foam ball 74 mm",
                            "74 mm foam pickleball (design.md 2)", "any"))
    if "mounts" in want:
        if kit == "catch":                                     # the module's yokes and their boards
            for nmp, s, mat, inf, orient in MT.printed():
                r.add_printed(nmp, s, s, mat, inf, orient, "mounts", "ROBOT")
            r.add_hw(MT.hardware(), at, "mounts")
        else:                                                  # the arm kit
            for nmp, s, mat, inf, orient in AK.printed():
                r.add_printed(nmp, s, s, mat, inf, orient, "mounts", "ROBOT")
            r.add_hw(AK.hardware(), at, "mounts")
            r.items.append(Item("SO-101 arm", "proxy", "mounts", Box(1.0, 1.0, 1.0).moved(Pos(*AK.ARM_CG)), "SO-101 arm",
                                "TheRobotStudio SO-101 follower arm, 6 x Feetech STS3215 12 V, Waveshare bus servo board on its "
                                "base; on the arm kit's adapter with 4 x M3 (VERIFY its hole pattern). Not modelled as a solid: "
                                "build.py checks its own meshes against the CAD (arm_clearance)",
                                "TheRobotStudio kit / Seeed / PartaBot (~$120-250)"))
        for nmp, s, mv, mat, inf, orient in DP.printed(seat, audio):    # the face, the audio box at their seats
            r.add_printed(nmp, s, s.moved(mv), mat, inf, orient, "mounts", "ROBOT")
        r.add_hw(DP.hardware(seat, audio), at, "mounts")
        for nmp, s, mat, inf, orient in H.printed():
            if nmp.startswith("cheek"):
                r.add_printed(nmp, s, s, mat, inf, orient, "mounts", "ROBOT")
        r.add_hw(H.cheek_hardware(index), at, "mounts")
    if "head" in want:
        mv = H.index_loc(index)
        for nmp, s, mat, inf, orient in H.printed():
            if not nmp.startswith("cheek"):
                r.add_printed(nmp, s, s.moved(mv), mat, inf, orient, "head", "ROBOT")
        r.add_hw(H.hardware(index), at, "head")
    return r


def _zaxis():
    from build123d import Axis
    return Axis.Z
