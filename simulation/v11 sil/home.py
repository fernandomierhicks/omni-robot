"""The home world (Nov 24): the real house, single storey, from its permit drawings (home_extract.py -> home_plan.json).

What comes from the drawings (sheet A-1, 1/4" = 1'-0", vector; measured, not traced):
  walls     114 boxes, their thickness as drawn (outer 0.13-0.21 m, inner 0.13-0.16 m)
  doors     18 hinged doors: hinge, width (0.48-0.92 m) and swing from the drawn arcs and leaves; the den's two French
            pairs (0.92 m leaves); the sliding glass door from the den to the pool
  windows   24 spans in the outer walls (pane lines), with the corrections in WINDOW_FIX
  fixtures  toilets, tubs, the shower, vanities and sinks, washer and dryer, laundry and kitchen counters, the island,
            fridge, oven tower, cooktops, the double-sided fireplace and the built-ins beside it, the water heater,
            closets (bifold / bypass doors shut, pocket doors open)
What is est (not on the drawings; Fernando to correct): heights (walls 2.44 m, sills 0.91, door heads 2.03, counters
0.91), which floors are tile (the wet rooms and the kitchen; wood elsewhere), the furniture (FURNITURE: plausible
pieces in plausible places, to be replaced by what the house really has), and the "corner built-in" in bathroom 2 (a
chamfered outline on the sheet: a tub or a bench).

Floors told by Fernando (Nov 24): the family room's floor is raised above the rest: at each of its two entries (from
the hall, from the hallway) a 0.5 in square step, then a 20 deg ramp over ~6 in, so 68 mm up (a floor.Platform and two
floor.Ramps; the run taken as horizontal, est). A square rug in the middle of the living room, 0.375 in thick (8 x 8 ft,
square edges: est). What stands on them (the family room's furniture, built-ins, the den's open French doors; the coffee table)
is set down on them; the robot and the toys spawn on them (floor.height_at).

The den is 6.5 in (165 mm) below the family room (the sheet's "-6.5"" at its doors): off limits, the robot can't climb out
of it or go down into it. What the cameras see is the real drop (Nov 25): in this world the physics floor plane is in
group 3 (not drawn), and the floor drawn is planes round the den plus the den's own floor 165 mm under the family
room's, with skirts down to it, so the D435 measures the step as it would in the house. The physics floor stays one
plane at z 0, so the robot can't fall in: an invisible barrier just inside the den's doors (group 4: the robot's body
stops at the edge; the lidar and the cameras don't see it) and in_den() tell whether it would have driven off the step.
The generated map's places mark the den a no-go zone, so Nav2 plans around it on that map; a new map (Explore) has no
zones: only the camera's drop marks (depth_layer:=height, the default) keep the robot off the step then.

Frame: metres, origin at the middle of the house's walls, x along the sheet's right (north), y along its top (west,
the pool side); the house spans x -9.6..9.6, y -9.93..9.69.

  python home.py                 MuJoCo's viewer on the house (no robot code running)
  python home.py --top out.png   a top view (renders/)
  python home.py --map           ros/maps/home.{pgm,yaml}, home_origin.json, home.places.json (rooms, places, the den
                                 a no-go zone), the map frame = the robot's START (as a SLAM session started there)
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from assets import _f

HERE = Path(__file__).resolve().parent
PLAN = json.loads((HERE / "home_plan.json").read_text())

WALL_H = 2.44            # m: 8 ft ceilings (est)
SILL, HEAD = 0.91, 2.03  # windows' sill and head, doors' head (est: 36 in, 6'-8")
DEN_DROP = 0.165         # m: the den's floor below the family room's (the sheet's "-6.5"" at their doors)
# Fernando (Nov 24): the family room's floor is higher than the rest; at each of its entries a 0.5 in square step,
# then a 20 deg ramp over ~6 in (taken as its horizontal run) up to it: 12.7 + 152.4 tan 20 = 68 mm (est from his
# description, not measured)
FAMILY_STEP = 0.0127
FAMILY_RAMP_RUN = 0.1524
FAMILY_RAMP_DEG = 20.0
FAMILY_H = FAMILY_STEP + FAMILY_RAMP_RUN * math.tan(math.radians(FAMILY_RAMP_DEG))
# the platform: the family room to its walls, and on under the den's doorways to the den's step
FAMILY_FLOOR = (-5.13, -2.88, 0.94, 2.33)
# Fernando (Nov 24): a square rug in the middle of the living room, 0.375 in thick (Nov 25). Its size est: 8 x 8 ft,
# square edges
RUG_SIDE, RUG_T = 2.44, 0.375 * 0.0254
# the den's floor, below the house's floor (z 0): seen by the cameras only (the physics floor stays one plane)
DEN_Z = FAMILY_H - DEN_DROP
WOOD = (0.72, 0.55, 0.36, 1)             # the raised floor and its ramps (est colour)
AREA = (21.4, 21.4)      # SimParams.area_size: the sim's own boundary walls clear of the house

# rooms (world, inside the walls): the console's room names and the clutter's places. Rectangles; the L-shaped ones
# are their main part
ROOMS = {
    "bedroom 1": (-9.40, -9.79, -5.34, -4.72),
    "bathroom 1": (-9.40, -4.58, -6.94, -2.48),
    "laundry": (-9.40, -2.35, -6.46, -0.60),
    "walk-in closet": (-9.40, 0.18, -6.46, 2.19),
    "hall": (-6.25, -5.34, -5.34, 2.19),
    "bedroom 2": (-9.45, 2.33, -3.99, 7.18),
    "bathroom 2": (-9.45, 7.33, -3.99, 9.55),
    "den": (-3.83, 2.33, 0.94, 8.38),
    "family room": (-5.13, -2.88, 0.94, 2.19),
    "living room": (-5.13, -7.63, 0.94, -3.02),
    "entry": (1.12, -6.08, 2.87, -2.52),
    "dining room": (3.03, -6.33, 6.37, -2.52),
    "hallway": (1.12, -2.38, 6.37, -1.39),
    "bedroom 3": (1.12, -1.25, 5.05, 1.46),
    "bathroom 3": (5.21, -1.33, 6.37, 1.46),
    "kitchen": (6.53, -7.58, 9.32, -1.48),
    "pantry": (6.53, -1.33, 7.57, 0.23),
    "back hall": (7.72, -1.33, 9.32, 1.46),
}
DEN = ROOMS["den"]
OFF_LIMITS = ("den",)
TILE_ROOMS = ("bathroom 1", "bathroom 2", "bathroom 3", "laundry", "kitchen", "pantry", "back hall", "entry")   # est
TILE_MU = 0.85           # rollers on tile (est, as house.py)

# the doors, by their hinge on the sheet (home_plan.json "arcs"); outside: shut unless home_doors says otherwise, the
# others open as drawn
DOORS = {
    "bathroom 2 outside": dict(hinge=(-8.393, 9.685), outside=True),
    "water heater closet": dict(hinge=(-9.588, 7.856), outside=True),
    "bathroom 2": dict(hinge=(-7.225, 7.321)),
    "bedroom 2": dict(hinge=(-5.396, 2.19)),
    "den 1": dict(hinge=(-3.313, 2.19)),
    "den 2": dict(hinge=(-1.484, 2.19)),
    "den 3": dict(hinge=(-1.23, 2.19)),
    "den 4": dict(hinge=(0.598, 2.19)),
    "patio": dict(hinge=(8.879, 1.581), outside=True),
    "pantry": dict(hinge=(7.71, -0.287)),
    "laundry": dict(hinge=(-6.438, -0.734)),
    "kitchen side": dict(hinge=(9.464, -1.288), outside=True),
    "bathroom 3": dict(hinge=(6.314, -1.339)),
    "linen closet": dict(hinge=(-6.235, -2.004)),
    "coat closet": dict(hinge=(2.846, -2.394)),
    "bathroom 1": dict(hinge=(-6.412, -2.965)),
    "bedroom 1": dict(hinge=(-5.372, -5.48)),
    "front": dict(hinge=(1.59, -6.076), outside=True),
}
# windows the extraction got wrong: (found span, replacement or None)
WINDOW_FIX = (
    ([-9.499, -2.966, -9.499, -2.484], None),                                  # no pane pair: a fixture's edge
    ([-7.506, 9.62, -4.127, 9.62], [-5.32, 9.62, -4.15, 9.62]),                # bathroom 2: the vanity's back + its window
)
# closets with sliding / folding doors, shut (a panel in the opening, a head over it)
PANELS = {
    "hall closet": (-6.30, -5.34, -6.26, -3.96),          # bifold
    "bedroom 3 closet": (4.30, -1.16, 4.34, 0.15),         # bypass
}
# heads over the openings with no hinged door (no wider than a doorway): pocket doors (open) and cased openings
OPENINGS = (
    (-8.68, 2.19, -7.90, 2.33),          # bedroom 2 -> walk-in closet (pocket door)
    (1.12, -1.39, 1.95, -1.25),          # hallway -> bedroom 3 (pocket door)
    (-5.34, 0.29, -5.13, 1.15),          # hall -> family room
    (0.94, -2.52, 1.12, -1.43),          # family room -> hallway
    (6.37, -6.46, 6.53, -5.41),          # dining room -> kitchen
    (6.37, -2.52, 6.53, -1.33),          # hallway -> kitchen
    (7.59, -1.48, 8.63, -1.33),          # kitchen -> back hall
    (-6.35, -5.34, -6.22, -3.96),        # the hall closet's head
    (4.24, -1.16, 4.40, 0.15),           # bedroom 3's closet's head
)

START = (-2.2, 0.5, 0.0)                 # the family room, facing east (x, y, yaw deg): the map frame's origin
SPOTS = {                                # teleport spots (x, y, yaw deg)
    "family room": START, "living room": (-2.0, -4.6, -90.0), "kitchen": (7.9, -4.6, -90.0),
    "dining room": (3.6, -3.2, 0.0), "entry": (2.0, -5.3, 90.0), "hall": (-5.8, -1.0, 90.0),
    "bedroom 1": (-6.6, -6.4, 180.0), "bedroom 2": (-6.6, 3.2, 90.0), "bedroom 3": (3.6, -0.6, 90.0),
    "bathroom 1": (-7.4, -3.4, 180.0), "back hall": (8.4, 0.0, 90.0), "den doors": (-1.4, 1.0, 90.0),
}
ARM_AREA = ((-0.7, -1.2), 0.55)          # the arm world's things: the family room's floor ahead of START
THROWER_PARK = (30.0, -30.0)             # the arena's thrower figure (robot_model._thrower): out of every sensor's sight
PLACES = {                               # the console's named places (world x, y, yaw deg)
    "family room": START[:2] + (0.0,), "sofa": (-0.6, 0.9, 0.0), "fireplace": (-2.1, -1.2, -90.0),
    "living room": (-2.0, -4.6, -90.0), "front door": (2.0, -5.4, -90.0), "dining table": (4.7, -3.1, -90.0),
    "kitchen island": (7.9, -5.7, -90.0), "fridge": (7.7, -2.9, 180.0), "laundry": (-7.3, -1.1, 180.0),
    "bedroom 1": (-6.6, -6.4, 180.0), "bedroom 2": (-6.6, 3.2, 90.0), "bedroom 3": (3.6, -0.6, 90.0),
    "bathroom 1": (-7.4, -3.4, 180.0), "bathroom 3": (5.8, -0.9, 90.0), "back door": (8.4, 0.9, 90.0),
}


def in_den(xy, margin: float = 0.0) -> bool:
    """A point in the den (margin m outside its walls counts too)."""
    x0, y0, x1, y1 = DEN
    return x0 - margin <= xy[0] <= x1 + margin and y0 - margin <= xy[1] <= y1 + margin


def room_at(xy) -> str | None:
    for name, (x0, y0, x1, y1) in ROOMS.items():
        if x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1:
            return name
    return None


def tile_patches() -> tuple:
    return tuple((*ROOMS[r], TILE_MU) for r in TILE_ROOMS)


def rug_rect() -> tuple:
    x0, y0, x1, y1 = ROOMS["living room"]
    cx, cy, h = (x0 + x1) / 2, (y0 + y1) / 2, RUG_SIDE / 2
    return cx - h, cy - h, cx + h, cy + h


def floor_features() -> tuple:
    """SimParams.floor_features: the family room's raised floor and the ramps at its two entries (from the hall, x
    -5.34..-5.13, and from the hallway, x 0.94..1.12; each across its opening, its top at the room's wall face), and
    the living room's rug."""
    from floor import Carpet, Platform, Ramp
    x0, y0, x1, y1 = FAMILY_FLOOR
    kw = dict(step=FAMILY_STEP, run=FAMILY_RAMP_RUN, height=FAMILY_H, mu=0.9, rgba=WOOD)
    return (Platform(x0, y0, x1, y1, thickness=FAMILY_H, mu=0.9, crr=0.02, softness=0.003, rgba=WOOD),
            Ramp(-5.13 - FAMILY_RAMP_RUN, (0.29 + 1.15) / 2, 0.0, length=1.15 - 0.29, **kw),      # from the hall
            Ramp(0.94 + FAMILY_RAMP_RUN, (-2.52 - 1.43) / 2, 180.0, length=2.52 - 1.43, **kw),    # from the hallway
            Carpet(*rug_rect(), thickness=RUG_T, mu=0.85, crr=0.06, rgba=(0.42, 0.33, 0.46, 1)))


def floor_z(xy) -> float:
    """The floor's height at a point (the family room's platform, the rug), for setting things down in build."""
    if FAMILY_FLOOR[0] <= xy[0] <= FAMILY_FLOOR[2] and FAMILY_FLOOR[1] <= xy[1] <= FAMILY_FLOOR[3]:
        return FAMILY_H
    x0, y0, x1, y1 = rug_rect()
    return RUG_T if x0 <= xy[0] <= x1 and y0 <= xy[1] <= y1 else 0.0


# the rooms the live world's clutter goes in (sim_world: not the den, the hall, a closet or a bathroom)
CLUTTER_ROOMS = ("bedroom 1", "bedroom 2", "bedroom 3", "family room", "living room", "dining room", "kitchen")


def door_centres() -> list:
    """The middle of every doorway (hinged doors and the open ones in OPENINGS), for keeping things out of them."""
    out = [tuple((np.array(a["hinge"]) + np.array(a["shut"])) / 2) for a in PLAN["arcs"]]
    out += [((x0 + x1) / 2, (y0 + y1) / 2) for x0, y0, x1, y1 in OPENINGS]
    return out


# -- the plan, corrected -----------------------------------------------------------------------------------------------
def windows() -> list:
    """[(x0, y0, x1, y1)] along the walls' middles: the extraction's, corrected and merged where two wall boxes held
    the same window."""
    out = []
    for w in PLAN["windows"]:
        fix = next((f for f in WINDOW_FIX if np.allclose(f[0], w, atol=0.02)), None)
        if fix is not None:
            w = fix[1]
        if w is None:
            continue
        horiz = abs(w[1] - w[3]) < 1e-6
        for k, v in enumerate(out):
            if horiz == (abs(v[1] - v[3]) < 1e-6) and abs((v[1] if horiz else v[0]) - (w[1] if horiz else w[0])) < 0.12:
                a0, a1 = (v[0], v[2]) if horiz else (v[1], v[3])
                b0, b1 = (w[0], w[2]) if horiz else (w[1], w[3])
                if b0 < a1 and a0 < b1:
                    lo, hi = min(a0, b0), max(a1, b1)
                    out[k] = [lo, v[1], hi, v[3]] if horiz else [v[0], lo, v[2], hi]
                    break
        else:
            out.append(list(w))
    return out


def doors(p=None) -> list:
    """[(name, hinge, shut end, open end, width, open 0..1, outside)] from the plan's arcs, matched by hinge."""
    over = dict(getattr(p, "home_doors", ()) or ())
    for k in over:
        if k not in DOORS:
            raise ValueError(f"home_doors: no door {k!r} (doors: {', '.join(DOORS)})")
    out = []
    for name, info in DOORS.items():
        arc = min(PLAN["arcs"], key=lambda a: math.dist(a["hinge"], info["hinge"]))
        if math.dist(arc["hinge"], info["hinge"]) > 0.05:
            raise ValueError(f"home_plan.json has no swing at {name}'s hinge {info['hinge']}")
        outside = info.get("outside", False)
        frac = float(over.get(name, 0.0 if outside else 1.0))
        out.append((name, np.array(arc["hinge"]), np.array(arc["shut"]), np.array(arc["open"]), arc["width"],
                    min(max(frac, 0.0), 1.0), outside))
    return out


# -- geometry ------------------------------------------------------------------------------------------------------------
MATERIALS = {   # name: (rgba, specular, emission)
    "hm_wall": ((0.93, 0.92, 0.88, 1), 0.05, 0.0), "hm_trim": ((0.97, 0.97, 0.95, 1), 0.2, 0.0),
    "hm_door": ((0.96, 0.95, 0.92, 1), 0.2, 0.0), "hm_outer_door": ((0.42, 0.27, 0.16, 1), 0.3, 0.0),
    "hm_glass": ((0.7, 0.82, 0.9, 0.28), 0.9, 0.0), "hm_tile": ((0.7, 0.68, 0.63, 1), 0.12, 0.0),
    "hm_den_floor": ((0.5, 0.4, 0.3, 1), 0.2, 0.0), "hm_step": ((0.16, 0.13, 0.11, 1), 0.1, 0.0),
    "hm_porcelain": ((0.97, 0.97, 0.96, 1), 0.6, 0.0), "hm_cabinet": ((0.88, 0.86, 0.8, 1), 0.2, 0.0),
    "hm_counter": ((0.25, 0.25, 0.27, 1), 0.5, 0.0), "hm_steel": ((0.74, 0.75, 0.77, 1), 0.7, 0.0),
    "hm_dark": ((0.12, 0.12, 0.13, 1), 0.3, 0.0), "hm_brick": ((0.55, 0.3, 0.22, 1), 0.05, 0.0),
    "hm_wood": ((0.55, 0.38, 0.22, 1), 0.2, 0.0), "hm_wood_dark": ((0.3, 0.2, 0.12, 1), 0.2, 0.0),
    "hm_fabric": ((0.36, 0.42, 0.52, 1), 0.02, 0.0), "hm_fabric2": ((0.6, 0.55, 0.47, 1), 0.02, 0.0),
    "hm_fabric3": ((0.45, 0.25, 0.22, 1), 0.02, 0.0), "hm_bedding": ((0.92, 0.92, 0.9, 1), 0.02, 0.0),
    "hm_blanket": ((0.62, 0.68, 0.74, 1), 0.02, 0.0), "hm_mirror": ((0.8, 0.85, 0.88, 1), 1.0, 0.0),
    "hm_screen": ((0.05, 0.06, 0.08, 1), 0.8, 0.0), "hm_lamp": ((0.98, 0.94, 0.8, 1), 0.1, 0.4),
    "hm_plant": ((0.2, 0.45, 0.2, 1), 0.05, 0.0), "hm_water_heater": ((0.85, 0.85, 0.82, 1), 0.4, 0.0),
}


class _G:
    """Geoms in the world body. Physical ones (the robot hits them, the lidar sees them) unless decor; names prop_*
    except the walls (tag_sim puts its tags on unnamed-prop boxes)."""

    def __init__(self):
        self.g: list[str] = []
        self.dz = 0.0                 # the floor under what's being built (the family room's platform, the rug)

    def _up(self, *pts):
        return [(x, y, z + self.dz) for x, y, z in pts]

    def _name(self, kind):
        return f'name="{kind}_{len(self.g)}"'

    def _attrs(self, decor, group):
        if decor:
            return f'class="decor"{"" if group is None else f" group={chr(34)}{group}{chr(34)}"}'
        return f'contype="1" conaffinity="1" group="{0 if group is None else group}" friction="0.6 0.005 0.0001"'

    def box(self, lo, hi, mat, decor=False, group=None, kind="prop_home"):
        lo, hi = (np.asarray(v, float) for v in self._up(lo, hi))
        c, h = (lo + hi) / 2, np.maximum(np.abs(hi - lo) / 2, 0.0005)
        self.g.append(f'<geom {self._name(kind)} type="box" {self._attrs(decor, group)} material="{mat}" '
                      f'pos="{_f(*c)}" size="{_f(*h)}"/>')

    def obox(self, centre, yaw, half, mat, decor=False, group=None, kind="prop_home"):
        """A box turned yaw (rad) about z: centre (x, y, z), half sizes (along, across, up)."""
        q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        centre = self._up(centre)[0]
        self.g.append(f'<geom {self._name(kind)} type="box" {self._attrs(decor, group)} material="{mat}" '
                      f'pos="{_f(*centre)}" quat="{_f(*q)}" size="{_f(*half)}"/>')

    def cyl(self, a, b, r, mat, decor=False, group=None):
        a, b = self._up(a, b)
        self.g.append(f'<geom {self._name("prop_home")} type="cylinder" {self._attrs(decor, group)} material="{mat}" '
                      f'size="{_f(r)}" fromto="{_f(*a, *b)}"/>')

    def cap(self, a, b, r, mat, decor=False):
        a, b = self._up(a, b)
        self.g.append(f'<geom {self._name("prop_home")} type="capsule" {self._attrs(decor, None)} material="{mat}" '
                      f'size="{_f(r)}" fromto="{_f(*a, *b)}"/>')

    def sph(self, c, r, mat, decor=True):
        c = self._up(c)[0]
        self.g.append(f'<geom {self._name("prop_home")} type="sphere" {self._attrs(decor, None)} material="{mat}" '
                      f'size="{_f(r)}" pos="{_f(*c)}"/>')

    def table(self, x0, y0, x1, y1, h, mat="hm_wood", leg=0.025, top=0.03):
        self.box((x0, y0, h - top), (x1, y1, h), mat)
        for x, y in ((x0 + 0.05, y0 + 0.05), (x1 - 0.05, y0 + 0.05), (x0 + 0.05, y1 - 0.05), (x1 - 0.05, y1 - 0.05)):
            self.cyl((x, y, 0.0), (x, y, h - top), leg, mat)

    def chair(self, x, y, facing_deg, mat="hm_wood"):
        """A dining chair: seat, four legs, a back on the side away from facing_deg."""
        s = 0.21
        self.table(x - s, y - s, x + s, y + s, 0.46, mat, leg=0.017, top=0.04)
        a = math.radians(facing_deg + 180.0)
        bx, by = x + s * math.cos(a), y + s * math.sin(a)
        self.obox((bx, by, 0.7), math.radians(facing_deg) + math.pi / 2, (s, 0.015, 0.24), mat)

    def stool(self, x, y, h=0.66, mat="hm_wood_dark"):
        self.cyl((x, y, h - 0.04), (x, y, h), 0.17, mat)
        for k in range(4):
            a = math.pi / 4 + k * math.pi / 2
            self.cyl((x + 0.12 * math.cos(a), y + 0.12 * math.sin(a), 0.0),
                     (x + 0.08 * math.cos(a), y + 0.08 * math.sin(a), h - 0.04), 0.012, mat)

    def sofa(self, x0, y0, x1, y1, back: str, mat="hm_fabric", seat=0.45, top=0.85, arm=0.62):
        """A sofa in (x0, y0)-(x1, y1), its back along the side named 'n', 's', 'e' or 'w'; arms at the two ends."""
        self.box((x0, y0, 0.08), (x1, y1, seat), mat)
        for (cx, cy) in ((x0 + 0.05, y0 + 0.05), (x1 - 0.05, y0 + 0.05), (x0 + 0.05, y1 - 0.05), (x1 - 0.05, y1 - 0.05)):
            self.cyl((cx, cy, 0.0), (cx, cy, 0.08), 0.02, "hm_wood_dark")
        d = 0.2
        backs = {"n": ((x0, y1 - d), (x1, y1)), "s": ((x0, y0), (x1, y0 + d)),
                 "e": ((x1 - d, y0), (x1, y1)), "w": ((x0, y0), (x0 + d, y1))}
        (a0, b0), (a1, b1) = backs[back]
        self.box((a0, b0, seat), (a1, b1, top), mat)
        if back in ("n", "s"):
            self.box((x0, y0, seat), (x0 + 0.15, y1, arm), mat)
            self.box((x1 - 0.15, y0, seat), (x1, y1, arm), mat)
        else:
            self.box((x0, y0, seat), (x1, y0 + 0.15, arm), mat)
            self.box((x0, y1 - 0.15, seat), (x1, y1, arm), mat)

    def bed(self, x0, y0, x1, y1, head: str):
        """Frame, mattress, blanket, headboard on side head ('n', 's', 'e', 'w')."""
        self.box((x0, y0, 0.08), (x1, y1, 0.32), "hm_wood")
        self.box((x0 + 0.03, y0 + 0.03, 0.32), (x1 - 0.03, y1 - 0.03, 0.55), "hm_bedding")
        hb = {"n": ((x0, y1 - 0.06), (x1, y1)), "s": ((x0, y0), (x1, y0 + 0.06)),
              "e": ((x1 - 0.06, y0), (x1, y1)), "w": ((x0, y0), (x0 + 0.06, y1))}[head]
        self.box((*hb[0], 0.0), (*hb[1], 1.05), "hm_wood")
        foot = {"n": (x0, y0, x1, y1 - 0.65), "s": (x0, y0 + 0.65, x1, y1),
                "e": (x0, y0, x1 - 0.65, y1), "w": (x0 + 0.65, y0, x1, y1)}[head]
        self.box((foot[0] + 0.01, foot[1] + 0.01, 0.5), (foot[2] - 0.01, foot[3] - 0.01, 0.59), "hm_blanket", decor=True)
        for cx, cy in ((x0 + 0.04, y0 + 0.04), (x1 - 0.04, y0 + 0.04), (x0 + 0.04, y1 - 0.04), (x1 - 0.04, y1 - 0.04)):
            self.box((cx - 0.035, cy - 0.035, 0.0), (cx + 0.035, cy + 0.035, 0.08), "hm_wood")

    def toilet(self, cx, cy, toward_deg):
        """A toilet: the tank against the wall behind it, the bowl toward toward_deg."""
        a = math.radians(toward_deg)
        u = np.array([math.cos(a), math.sin(a)])
        tank = np.array([cx, cy]) - u * 0.25
        bowl = np.array([cx, cy]) + u * 0.08
        self.obox((*tank, 0.58), a, (0.1, 0.24, 0.2), "hm_porcelain")
        self.obox((*tank, 0.19), a, (0.1, 0.12, 0.19), "hm_porcelain")
        self.cyl((*bowl, 0.0), (*bowl, 0.4), 0.19, "hm_porcelain")
        self.cyl((*bowl, 0.4), (*bowl, 0.42), 0.2, "hm_trim", decor=True)

    def body(self) -> str:
        return "".join(self.g)


def _wall_boxes(G: _G):
    """The plan's walls, cut where a window is (sill, glass, head), full height elsewhere."""
    wins = windows()
    for k, (x0, y0, x1, y1) in enumerate(PLAN["walls"]):
        horiz = (x1 - x0) >= (y1 - y0)
        a0, a1 = (x0, x1) if horiz else (y0, y1)
        c0, c1 = (y0, y1) if horiz else (x0, x1)
        cuts = []
        for w in wins:
            wh = abs(w[1] - w[3]) < 1e-6
            if wh != horiz:
                continue
            line = w[1] if horiz else w[0]
            if not (c0 - 0.08 <= line <= c1 + 0.08):
                continue
            lo, hi = sorted((w[0], w[2]) if horiz else (w[1], w[3]))
            lo, hi = max(lo, a0), min(hi, a1)
            if hi - lo > 0.02:
                cuts.append((lo, hi))
        edges = [a0]
        for lo, hi in sorted(cuts):
            edges += [lo, hi]
        edges.append(a1)

        def piece(s0, s1, z0, z1, mat="hm_wall", kind="home_wall", decor=False, group=None, thin=None):
            if s1 - s0 < 1e-3:
                return
            cc0, cc1 = (c0, c1) if thin is None else ((c0 + c1) / 2 - thin, (c0 + c1) / 2 + thin)
            lo = (s0, cc0, z0) if horiz else (cc0, s0, z0)
            hi = (s1, cc1, z1) if horiz else (cc1, s1, z1)
            G.box(lo, hi, mat, decor=decor, group=group, kind=kind)

        for i in range(0, len(edges) - 1):
            s0, s1 = edges[i], edges[i + 1]
            if i % 2 == 0:                                    # solid
                piece(s0, s1, 0.0, WALL_H)
            else:                                             # a window: sill, glass, head, a sill ledge
                piece(s0, s1, 0.0, SILL, kind="home_sill")
                piece(s0, s1, HEAD, WALL_H, kind="prop_home_head")
                piece(s0, s1, SILL, HEAD, mat="hm_glass", kind="prop_home_glass", group=2, thin=0.006)
                piece(s0 - 0.03, s1 + 0.03, SILL - 0.02, SILL, mat="hm_trim", kind="prop_home_trim", decor=True,
                      thin=(c1 - c0) / 2 + 0.03)


def _sliders(G: _G):
    """The den's sliding glass door (shut): a head, a track, two glass panels in frames."""
    for name, (x0, y0, x1, y1) in PLAN["sliders"].items():
        lo, hi = min(x0, x1), max(x0, x1)
        t = 0.07
        G.box((lo, y0 - t, HEAD), (hi, y0 + t, WALL_H), "hm_wall", kind="prop_home_head")
        G.box((lo, y0 - t, 0.0), (hi, y0 + t, 0.02), "hm_steel")
        mid = (lo + hi) / 2
        for a, b, dy in ((lo, mid + 0.04, -0.02), (mid - 0.04, hi, 0.02)):
            G.box((a, y0 + dy - 0.006, 0.02), (b, y0 + dy + 0.006, HEAD), "hm_glass", group=2, kind="prop_home_glass")
            for s in (a, b - 0.04):
                G.box((s, y0 + dy - 0.02, 0.02), (s + 0.04, y0 + dy + 0.02, HEAD), "hm_trim")


def _doors(G: _G, p):
    ds = doors(p)
    shut_ends = [e for _, _, e, _, _, _, _ in ds]
    for name, h, e, o, width, frac, outside in ds:
        us = (e - h) / np.linalg.norm(e - h)
        uo = (o - h) / np.linalg.norm(o - h)
        th = frac * math.pi / 2
        d = math.cos(th) * us + math.sin(th) * uo
        L = width - 0.01
        t = 0.045 if outside else 0.035
        c = h + d * L / 2
        yaw = math.atan2(d[1], d[0])
        G.dz = floor_z(c)                                 # the den's leaves stand on the family room's floor
        G.obox((c[0], c[1], 0.012 + (HEAD - 0.022) / 2), yaw, (L / 2, t / 2, (HEAD - 0.022) / 2),
               "hm_outer_door" if outside else "hm_door")
        n = np.array([-d[1], d[0]])
        knob = h + d * (L - 0.06)
        for sgn in (1, -1):
            G.sph((*(knob + sgn * n * (t / 2 + 0.03)), 0.92), 0.03, "hm_steel")
        G.dz = 0.0
        # the head over the opening, as thick as the wall beside it; the jambs (the frame between the wall's end and
        # the leaf, ~5 cm on the sheet), not where a French pair's leaves meet
        (lo, hi) = _head_box(h, e)
        G.box(lo, hi, "hm_wall", kind="prop_home_head")
        horiz = abs(h[1] - e[1]) < 1e-3
        c0, c1 = (lo[1], hi[1]) if horiz else (lo[0], hi[0])
        meets = any(np.linalg.norm(e - f) < 0.1 for f in shut_ends if f is not e)
        for a, b in ((h - us * 0.07, h), (e, e + us * 0.07)) if not meets else ((h - us * 0.07, h),):
            s0, s1 = sorted((a[0], b[0]) if horiz else (a[1], b[1]))
            G.box((s0, c0, 0.0) if horiz else (c0, s0, 0.0), (s1, c1, HEAD) if horiz else (c1, s1, HEAD), "hm_trim")


def _head_box(h, e):
    """The head over an opening from h to e: the wall box beside it gives its thickness."""
    horiz = abs(h[1] - e[1]) < 1e-3
    lo, hi = sorted((h[0], e[0]) if horiz else (h[1], e[1]))
    line = h[1] if horiz else h[0]
    best = None
    for x0, y0, x1, y1 in PLAN["walls"]:
        if ((x1 - x0) >= (y1 - y0)) != horiz:
            continue
        c0, c1 = (y0, y1) if horiz else (x0, x1)
        a0, a1 = (x0, x1) if horiz else (y0, y1)
        if c0 - 0.03 <= line <= c1 + 0.03 and (abs(a1 - lo) < 0.3 or abs(a0 - hi) < 0.3) and c1 - c0 > 0.08:
            best = (c0, c1)
            break
    c0, c1 = best or (line - 0.07, line + 0.07)
    if horiz:
        return (lo, c0, HEAD), (hi, c1, WALL_H)
    return (c0, lo, HEAD), (c1, hi, WALL_H)


def _floors(G: _G):
    """Tile in the wet rooms (visual; their friction is SimParams.floor_patches), the den's floor, its step."""
    for r in TILE_ROOMS:
        x0, y0, x1, y1 = ROOMS[r]
        G.box((x0, y0, 0.0004), (x1, y1, 0.0012), "hm_tile", decor=True)
    x0, y0, x1, y1 = DEN
    # the floor the cameras see (the physics plane is group 3 in this world): the house's round the den, the den's
    # own 165 mm under the family room's, and skirts from it up to the floors and walls round it (decor)
    ax, ay = AREA[0] / 2, AREA[1] / 2
    for k, (a0, b0, a1, b1) in enumerate(((-ax, -ay, ax, y0), (-ax, y1, ax, ay), (-ax, y0, x0, y1), (x1, y0, ax, y1))):
        G.g.append(f'<geom name="prop_home_floor_{k}" type="plane" class="decor" group="0" material="floor" '
                   f'pos="{_f((a0 + a1) / 2, (b0 + b1) / 2, 0.0)}" size="{_f((a1 - a0) / 2, (b1 - b0) / 2, 0.05)}"/>')
    G.box((x0, y0, DEN_Z - 0.01), (x1, y1, DEN_Z), "hm_den_floor", decor=True)
    sk = 0.12
    G.box((x0 - sk, y0 - sk, DEN_Z), (x0, y1 + sk, 0.0), "hm_wall", decor=True)
    G.box((x1, y0 - sk, DEN_Z), (x1 + sk, y1 + sk, 0.0), "hm_wall", decor=True)
    G.box((x0, y1, DEN_Z), (x1, y1 + sk, 0.0), "hm_wall", decor=True)
    G.box((x0, y0 - sk, DEN_Z), (x1, y0, 0.0), "hm_step", decor=True)       # the step's face under the platform's
    # the edge, in the doorways: on the family room's raised floor, which runs to the den's step
    G.box((x0, y0 - 0.14, FAMILY_H + 0.0004), (x1, y0 + 0.002, FAMILY_H + 0.0016), "hm_step", decor=True)
    # the step's stand-in: the robot's body stops here (group 4: no lidar, no camera)
    G.g.append(f'<geom name="home_den_step" type="box" contype="1" conaffinity="1" group="4" rgba="1 0.3 0 0.3" '
               f'pos="{_f((x0 + x1) / 2, y0 + 0.05, 0.3)}" size="{_f((x1 - x0) / 2, 0.02, 0.3)}"/>')


def _fixtures(G: _G):
    """What the sheet draws in the rooms (positions measured; heights est)."""
    # bathroom 2: toilet room, shower, double vanity, the corner built-in, the water heater
    G.toilet(-9.03, 8.3, 90.0)
    G.box((-8.71, 7.33, 0.0), (-7.36, 8.25, 0.1), "hm_tile")                    # shower curb / pan
    G.box((-8.71, 8.24, 0.1), (-7.36, 8.26, 2.0), "hm_glass", group=2)          # its glass
    G.box((-7.37, 7.33, 0.1), (-7.35, 8.25, 2.0), "hm_glass", group=2)
    G.box((-7.45, 9.0, 0.0), (-5.45, 9.55, 0.83), "hm_cabinet")                 # vanity
    G.box((-7.47, 8.98, 0.83), (-5.43, 9.55, 0.87), "hm_counter")
    for x in (-6.98, -5.93):
        G.cyl((x, 9.3, 0.85), (x, 9.3, 0.875), 0.2, "hm_porcelain", decor=True)
    G.box((-7.4, 9.545, 1.1), (-5.5, 9.55, 1.95), "hm_mirror", decor=True)
    # the corner built-in (assets_xml's hm_corner: the sheet's chamfered outline; a tub or a bench, 0.55 m est)
    G.g.append(f'<geom name="prop_home_{len(G.g)}" type="mesh" mesh="hm_corner" contype="1" conaffinity="1" '
               f'material="hm_porcelain"/>')
    G.cyl((-9.18, 7.62, 0.0), (-9.18, 7.62, 1.5), 0.24, "hm_water_heater")
    # bedroom 2's walk-in closet: a low shoe shelf along the outer wall
    G.box((-9.40, 0.3, 0.0), (-9.05, 2.05, 0.45), "hm_wood")
    G.box((-9.40, 0.3, 1.0), (-9.30, 2.05, 1.75), "hm_fabric2", decor=True)    # clothes hanging
    # laundry: washer, dryer, the utility counter and its sink
    for x0, x1 in ((-9.03, -8.22), (-8.19, -7.38)):
        G.box((x0, -2.33, 0.0), (x1, -1.64, 0.9), "hm_porcelain")
        G.cyl(((x0 + x1) / 2, -1.99, 0.9), ((x0 + x1) / 2, -1.99, 0.905), 0.2, "hm_steel", decor=True)
    G.box((-9.40, -0.60, 0.0), (-8.18, -0.06, 0.87), "hm_cabinet")
    G.box((-9.40, -0.62, 0.87), (-8.16, -0.06, 0.91), "hm_counter")
    G.box((-9.0, -0.5, 0.905), (-8.6, -0.15, 0.912), "hm_steel", decor=True)
    # bathroom 1: toilet, tub, double vanity, the linen cabinet
    G.toilet(-9.02, -4.25, 90.0)
    G.box((-8.43, -4.58, 0.0), (-6.94, -3.83, 0.55), "hm_porcelain")
    G.box((-8.33, -4.48, 0.5), (-7.04, -3.93, 0.552), "hm_steel", decor=True)  # the water's surface stand-in
    G.box((-9.40, -2.98, 0.0), (-7.98, -2.48, 0.83), "hm_cabinet")
    G.box((-9.40, -3.0, 0.83), (-7.96, -2.48, 0.87), "hm_counter")
    for x in (-9.08, -8.33):
        G.cyl((x, -2.72, 0.85), (x, -2.72, 0.875), 0.2, "hm_porcelain", decor=True)
    G.box((-9.395, -2.95, 1.1), (-9.39, -2.55, 1.9), "hm_mirror", decor=True)
    G.box((-7.85, -3.41, 0.0), (-7.36, -2.94, 2.0), "hm_cabinet")
    # bathroom 3: toilet, pedestal sink
    G.toilet(5.565, 1.12, -90.0)
    G.cyl((5.36, -0.28, 0.0), (5.36, -0.28, 0.78), 0.08, "hm_porcelain")
    G.box((5.21, -0.5, 0.78), (5.66, -0.05, 0.86), "hm_porcelain")
    # kitchen: the west run (counter + range, oven tower, fridge), the east run (sink, dishwasher), the island
    G.box((6.53, -5.45, 0.0), (7.13, -4.08, 0.87), "hm_cabinet")
    G.box((6.53, -5.47, 0.87), (7.15, -4.08, 0.91), "hm_counter")
    G.box((6.6, -4.98, 0.91), (7.12, -4.12, 0.92), "hm_dark", decor=True)       # the six-burner cooktop
    G.box((6.53, -4.98, 1.55), (6.95, -4.12, 1.75), "hm_steel")                  # its hood
    G.box((6.53, -5.45, 1.42), (6.88, -4.98, 2.13), "hm_cabinet")                # uppers
    G.box((6.53, -4.08, 0.0), (7.15, -3.32, 2.13), "hm_cabinet")                 # oven tower
    G.box((7.15, -3.95, 0.75), (7.16, -3.45, 1.45), "hm_dark", decor=True)
    G.box((6.53, -3.32, 0.0), (7.26, -2.41, 1.78), "hm_steel")                   # fridge
    G.box((8.71, -5.39, 0.0), (9.32, -1.48, 0.87), "hm_cabinet")
    G.box((8.69, -5.41, 0.87), (9.32, -1.48, 0.91), "hm_counter")
    G.box((8.8, -3.7, 0.9), (9.2, -3.0, 0.912), "hm_steel", decor=True)          # sink
    G.box((8.705, -4.5, 0.05), (8.71, -3.9, 0.8), "hm_steel", decor=True)        # dishwasher front
    G.box((8.97, -5.39, 1.42), (9.32, -1.48, 2.13), "hm_cabinet")                # uppers
    G.box((7.42, -6.89, 0.0), (8.44, -6.25, 0.87), "hm_cabinet")                 # island
    G.box((7.38, -6.93, 0.87), (8.48, -6.21, 0.91), "hm_counter")
    # back hall: the bar counter, the second cooktop
    G.box((8.71, -0.45, 0.0), (9.32, 0.64, 0.87), "hm_cabinet")
    G.box((8.69, -0.45, 0.87), (9.32, 0.64, 0.91), "hm_counter")
    G.box((7.16, 0.38, 0.0), (7.78, 1.46, 0.87), "hm_cabinet")
    G.box((7.16, 0.38, 0.87), (7.80, 1.46, 0.91), "hm_counter")
    G.box((7.22, 0.6, 0.91), (7.72, 1.3, 0.92), "hm_dark", decor=True)
    # family room / living room: the double-sided fireplace, its hearth, the built-ins beside it
    G.box((-3.16, -2.88, 0.0), (-1.05, -1.75, WALL_H), "hm_brick")
    G.dz = FAMILY_H                                                              # the family room's side
    G.box((-2.71, -1.76, 0.25), (-1.50, -1.745, 1.0), "hm_dark", decor=True)     # the family room's firebox
    G.dz = 0.0
    G.box((-2.71, -3.035, 0.25), (-1.50, -3.02, 1.0), "hm_dark", decor=True)     # the living room's
    G.box((-3.01, -3.23, 0.0), (-1.21, -3.02, 0.15), "hm_brick")                 # its raised hearth
    G.dz = FAMILY_H                                                              # on the raised floor
    G.box((-4.53, -2.88, 0.0), (-3.16, -2.28, 0.9), "hm_cabinet")
    G.box((-1.05, -2.88, 0.0), (0.94, -2.28, 0.9), "hm_cabinet")
    G.box((-5.13, -2.28, 0.0), (-4.53, 0.26, 0.9), "hm_cabinet")
    G.dz = 0.0
    # closets with folding / sliding doors, shut
    for x0, y0, x1, y1 in PANELS.values():
        G.box((x0, y0, 0.0), (x1, y1, HEAD), "hm_door")
    for lo_hi in OPENINGS:
        x0, y0, x1, y1 = lo_hi
        G.box((x0, y0, HEAD), (x1, y1, WALL_H), "hm_wall", kind="prop_home_head")


# est: plausible furniture in plausible places (not on the drawings); the doors' swings and the walkways kept clear
def _furniture(G: _G):
    # family room (on its raised floor): sofa along the east wall facing west, armchair, the TV on the west built-in, a
    # lamp. No coffee table since Nov 25 (removed at Fernando's request: the robot wedged under its 0.42 m top, which
    # neither the lidar nor the D435 sees; it was G.table(-1.0, -0.65, -0.4, 0.45, 0.42))
    G.dz = FAMILY_H
    G.sofa(0.0, -1.25, 0.9, 1.05, "e")
    G.sofa(-2.35, -1.65, -1.55, -0.85, "s", mat="hm_fabric2")
    G.box((-5.0, -1.5, 0.9), (-4.95, -0.3, 1.6), "hm_screen")
    G.cyl((-4.85, 1.85, 0.0), (-4.85, 1.85, 0.03), 0.15, "hm_dark")
    G.cyl((-4.85, 1.85, 0.03), (-4.85, 1.85, 1.5), 0.012, "hm_steel")
    G.cyl((-4.85, 1.85, 1.5), (-4.85, 1.85, 1.72), 0.16, "hm_lamp", decor=True)
    G.dz = 0.0
    # living room: sofa under the window facing the fireplace, two armchairs, a coffee table, a side table, a plant
    G.sofa(-3.3, -7.55, -0.9, -6.65, "s")
    G.dz = RUG_T                                                                 # the coffee table on the rug
    G.table(-2.6, -6.15, -1.6, -5.55, 0.42)
    G.dz = 0.0
    G.sofa(-4.85, -6.2, -4.05, -5.4, "w", mat="hm_fabric3")
    G.sofa(-0.25, -6.6, 0.55, -5.8, "e", mat="hm_fabric3")
    G.table(-0.75, -7.55, -0.35, -7.15, 0.55)
    G.cyl((-4.8, -7.3, 0.0), (-4.8, -7.3, 0.35), 0.16, "hm_dark")
    G.sph((-4.8, -7.3, 0.75), 0.35, "hm_plant")
    # dining room: table and six chairs, a sideboard
    G.table(3.85, -4.9, 5.55, -3.9, 0.75)
    for x in (4.2, 4.7, 5.2):
        G.chair(x, -5.25, 90.0)
        G.chair(x, -3.55, -90.0)
    G.box((3.9, -2.95, 0.0), (5.5, -2.52, 0.85), "hm_wood_dark")
    # kitchen: three stools tucked in at the island (Nov 26: at y -5.95, pulled 0.3 m out, their legs left 0.55 m to the
    # counter runs' ends on both sides, a walkway the robot passed only before the legs were mapped and then couldn't
    # plan back through, hm24 / hm27; tucked: ~0.65 m)
    for x in (7.6, 7.93, 8.26):
        G.stool(x, -6.06)
    # entry: a bench by the door
    G.box((2.4, -5.0, 0.0), (2.85, -4.0, 0.45), "hm_wood")
    # bedroom 1: queen bed between the west windows, nightstands, a dresser
    G.bed(-9.4, -8.13, -7.37, -6.61, "w")
    G.box((-9.4, -8.6, 0.0), (-8.95, -8.16, 0.55), "hm_wood")
    G.box((-9.4, -6.58, 0.0), (-8.95, -6.14, 0.55), "hm_wood")
    G.cyl((-9.17, -6.36, 0.55), (-9.17, -6.36, 0.8), 0.09, "hm_lamp", decor=True)
    G.box((-8.9, -5.22, 0.0), (-7.5, -4.74, 0.8), "hm_wood")
    # bedroom 2: king bed against the east wall, nightstands, a dresser, an armchair by the window
    G.bed(-6.02, 3.79, -3.99, 5.72, "e")
    G.box((-4.44, 3.3, 0.0), (-3.99, 3.75, 0.55), "hm_wood")
    G.box((-4.44, 5.76, 0.0), (-3.99, 6.21, 0.55), "hm_wood")
    G.box((-7.9, 2.33, 0.0), (-6.5, 2.8, 0.8), "hm_wood")
    G.sofa(-9.35, 6.2, -8.55, 7.0, "w", mat="hm_fabric2")
    # bedroom 3: full bed against the west wall, a desk under the window and its chair
    G.bed(1.12, -0.55, 3.02, 0.82, "w")
    G.table(3.4, 0.86, 4.6, 1.44, 0.75)
    G.chair(4.0, 0.5, 90.0)
    # den (off limits; seen through its doors): sofa, TV, armchair, on its sunken floor
    G.dz = DEN_Z
    G.sofa(-3.75, 4.2, -2.85, 6.4, "w")
    G.box((0.85, 4.7, 0.0), (0.94, 5.9, 0.5), "hm_wood_dark")
    G.box((0.9, 4.75, 0.8), (0.93, 5.85, 1.45), "hm_screen")
    G.sofa(-1.0, 7.3, -0.2, 8.1, "n", mat="hm_fabric3")
    G.dz = 0.0


def assets_xml(p=None) -> str:
    out = [f'<material name="{k}" rgba="{_f(*rgba)}" specular="{sp}" emission="{em}"/>' for k, (rgba, sp, em) in
           MATERIALS.items()]
    v = [(-5.21, 7.33), (-3.99, 7.33), (-3.99, 9.39), (-4.45, 9.39), (-5.21, 8.6)]
    verts = [c for x, y in v for c in (x, y, 0.0)] + [c for x, y in v for c in (x, y, 0.55)]
    out.append(f'<mesh name="hm_corner" vertex="{_f(*verts)}"/>')
    return "\n    ".join(out)


def worldbody_xml(p=None) -> str:
    G = _G()
    _wall_boxes(G)
    _sliders(G)
    _doors(G, p)
    _floors(G)
    _fixtures(G)
    _furniture(G)
    return G.body()


# -- the map Nav2 / AMCL can use straight away, and the console's places ------------------------------------------------
def occupancy(m, d, res: float = 0.05, z: float | None = None, area: tuple = AREA):
    """(grid bool[h, w] True = occupied, x0, y0) at the lidar's height z: every static geom the lidar sees (groups 0,
    1, 3) and the glass (group 2: the lidar sees through it, the robot can't drive through it) that spans z, its
    cross-section at z rasterized (boxes and meshes by their oriented extent, round things by radius). area: the
    world's (x, y) size about the origin (course.py passes its own)."""
    import mujoco
    if z is None:
        z = float(d.site_xpos[m.site("lidar").id][2])
    x0, y0 = -area[0] / 2, -area[1] / 2
    nx, ny = int(round(area[0] / res)), int(round(area[1] / res))
    X, Y = np.meshgrid(x0 + (np.arange(nx) + 0.5) * res, y0 + (np.arange(ny) + 0.5) * res)
    grid = np.zeros((ny, nx), dtype=bool)
    static = m.body_weldid[m.geom_bodyid] == 0
    for g in range(m.ngeom):
        if not static[g] or m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        if m.geom_group[g] not in (0, 1, 3) and not (m.geom_group[g] == 2 and m.geom_contype[g]):
            continue
        c, R, s = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
        t = m.geom_type[g]
        if t == mujoco.mjtGeom.mjGEOM_MESH:
            lo, hi = m.geom_aabb[g][:3] - m.geom_aabb[g][3:], m.geom_aabb[g][:3] + m.geom_aabb[g][3:]
            cz = c + R @ ((lo + hi) / 2)
            half = (hi - lo) / 2
            c, s, t = cz, half, mujoco.mjtGeom.mjGEOM_BOX
        if t == mujoco.mjtGeom.mjGEOM_BOX:
            hz = abs(R[2]) @ s
            if not (c[2] - hz <= z <= c[2] + hz):
                continue
            dx, dy, dz = X - c[0], Y - c[1], z - c[2]
            inside = np.ones_like(grid)
            for k in range(3):                               # inside along each of the box's axes, at height z
                inside &= np.abs(dx * R[0, k] + dy * R[1, k] + dz * R[2, k]) <= s[k] + res / 2
            grid |= inside
        elif t in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
            hz = s[1] + (s[0] if t == mujoco.mjtGeom.mjGEOM_CAPSULE else 0.0)
            ax = R[:, 2]
            if abs(ax[2]) > 0.9:                             # upright
                if c[2] - hz <= z <= c[2] + hz:
                    grid |= np.hypot(X - c[0], Y - c[1]) <= s[0] + res / 2
            else:
                a, b = c - ax * s[1], c + ax * s[1]
                if min(a[2], b[2]) - s[0] <= z <= max(a[2], b[2]) + s[0]:
                    ab = (b - a)[:2]
                    tt = np.clip(((X - a[0]) * ab[0] + (Y - a[1]) * ab[1]) / max(ab @ ab, 1e-9), 0, 1)
                    grid |= np.hypot(X - a[0] - tt * ab[0], Y - a[1] - tt * ab[1]) <= s[0] + res / 2
        elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
            if abs(c[2] - z) <= s[0]:
                r = math.sqrt(s[0] ** 2 - (c[2] - z) ** 2)
                grid |= np.hypot(X - c[0], Y - c[1]) <= r + res / 2
    return grid, x0, y0


def write_map(out_dir: Path, name: str = "home", base: str = "drawn", res: float = 0.05):
    """<name>.pgm / .yaml (AMCL, map_server), <name>_origin.json, <name>.places.json in the map frame (the robot's
    START: where a SLAM session started there would put it). Free = inside the house, unknown = outside."""
    import mujoco
    from scipy import ndimage as ndi
    from bridge_core import CoreConfig, model_params
    from robot_model import build_model
    m = build_model(model_params(CoreConfig(base=base, world="home")))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    # the scan plane above the lower floor, where most of the house is (in the bare model the robot stands at z = 0,
    # not yet set down on the family room's raised floor: Driver.teleport does that)
    try:
        z = float(d.site_xpos[m.site("lidar").id][2])
    except KeyError:
        z = 0.15
    # the robot is in the model: rasterize without its geoms (they aren't static)
    occ, x0, y0 = occupancy(m, d, res, z)
    sx, sy, syaw = START
    c, s = math.cos(math.radians(syaw)), math.sin(math.radians(syaw))
    if abs(syaw) > 1e-9:
        raise NotImplementedError("write_map: START's yaw must be 0 (the grid is axis-aligned in the map frame)")
    lab, _ = ndi.label(~occ)
    inside = lab == lab[int((sy - y0) / res), int((sx - x0) / res)]
    img = np.full(occ.shape, 205, dtype=np.uint8)           # unknown
    img[inside] = 254
    img[occ] = 0
    out_dir.mkdir(parents=True, exist_ok=True)
    h, w = img.shape
    with open(out_dir / f"{name}.pgm", "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(img[::-1].tobytes())                         # PGM rows top-down; the grid's row 0 is y0
    (out_dir / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nmode: trinary\nresolution: {res:.3f}\norigin: [{x0 - sx:.3f}, {y0 - sy:.3f}, 0]\n"
        f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    (out_dir / f"{name}_origin.json").write_text(json.dumps(dict(world_from_map=[sx, sy, math.radians(syaw)])))

    def to_map(x, y):
        return [round(x - sx, 3), round(y - sy, 3)]

    rooms = {r: [to_map(a, b), to_map(c_, b), to_map(c_, e), to_map(a, e)] for r, (a, b, c_, e) in ROOMS.items()}
    places = {k: [*to_map(x, y), round(math.radians(yaw), 4)] for k, (x, y, yaw) in PLACES.items()}
    zones = {}
    for r in OFF_LIMITS:
        a, b, c_, e = ROOMS[r]
        g = 0.12                                             # past its doorway's edge: the step itself
        zones[f"{r} (step down)"] = [to_map(a, b - g), to_map(c_, b - g), to_map(c_, e), to_map(a, e)]
    # floor edges (the console's slow bands, Nav2's speed filter): the family room's ramps (their square step at the
    # foot) and the rug's four edges
    from floor import Ramp
    edges = {}
    for k, f in enumerate(f for f in floor_features() if isinstance(f, Ramp)):
        c_, s_ = math.cos(math.radians(f.yaw_deg)), math.sin(math.radians(f.yaw_deg))
        hl = f.length / 2
        edges[f"family room ramp {k + 1}"] = dict(points=[to_map(f.x + s_ * hl, f.y - c_ * hl),
                                                          to_map(f.x - s_ * hl, f.y + c_ * hl)],
                                                  height_mm=round(f.step * 1000, 1), speed=1.0, kind="step")
    a, b, c_, e = rug_rect()
    edges["living room rug"] = dict(points=[to_map(a, b), to_map(c_, b), to_map(c_, e), to_map(a, e), to_map(a, b)],
                                    height_mm=round(RUG_T * 1000, 1), speed=1.0, kind="rug")
    path = out_dir / f"{name}.places.json"
    old = json.loads(path.read_text()) if path.exists() else {}
    old.setdefault("routes", {})
    old.update(rooms=rooms, places={**places, **old.get("places", {})}, zones={**old.get("zones", {}), **zones},
               edges={**old.get("edges", {}), **edges})
    path.write_text(json.dumps(old, indent=1))
    print(f"{name}: {w} x {h} cells at {res} m (lidar plane {z:.3f} m), {inside.sum() * res * res:.0f} m2 free; "
          f"{len(rooms)} rooms, {len(places)} places, {len(edges)} floor edges, no-go: {', '.join(zones)} -> {out_dir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--top", metavar="PNG", help="render a top view to renders/PNG")
    ap.add_argument("--map", action="store_true", help="write ros/maps/home.* (AMCL map, origin, places)")
    ap.add_argument("--base", default="drawn")
    a = ap.parse_args()
    if a.map:
        write_map(HERE.parents[1] / "ros" / "maps", base=a.base)
        return
    import mujoco
    from bridge_core import CoreConfig, model_params
    from robot_model import build_model
    m = build_model(model_params(CoreConfig(base=a.base, world="home")))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    print(f"home: {m.ngeom} geoms")
    if a.top:
        m.vis.global_.offwidth = m.vis.global_.offheight = 1800
        m.vis.global_.orthographic = 1                      # a plan view: fovy is the view's height in m
        m.vis.global_.fovy = 20.6
        r = mujoco.Renderer(m, 1800, 1800)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = (0.0, -0.1, 0.0)
        cam.distance, cam.azimuth, cam.elevation = 30.0, 90.0, -90.0
        opt = mujoco.MjvOption()
        r.update_scene(d, cam, opt)
        out = HERE / "renders" / a.top
        out.parent.mkdir(exist_ok=True)
        from PIL import Image
        Image.fromarray(r.render()).save(out)
        print(f"-> {out}")
        return
    import mujoco.viewer
    mujoco.viewer.launch(m, d)


if __name__ == "__main__":
    main()
