"""cad/v15 MID-DECK pod, layer 5 of 5 (pod.py -> pod_lean.py -> pod_rx.py -> pod_p4.py -> pod_mid.py; tools/README.md). The floor deck sits
above the pivot and below the spring towers (robot z 80..100, pod z +10..+30); the pod hangs under it, bolted up from below with the threads
in the deck (Fernando, Nov 14 2026; PROPOSAL). build123d, STEP review only.

    C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B pod_mid.py [build|check|export|render|all]      (checks: validation/pod_mid; STEPs: step_export/pod_mid)

From pod_p4.py: the fork, the pivot housings and rings, the beam and its droop lug (no shim: exact-length bosses on the 627s' inner races).
 * HANGERS: 14 mm plates outboard of the arms under the towers (their base width), tapering to 8 behind the pivot, z -10..+10, printed upright;
   each has the pivot boss on the inner race and an M3 x 35 pivot screw from its outer face into the beam.
 * Load path: the springs push the towers up and the pivot pulls the hangers and beam down (about 2x the wheel load). Ahead of the pivot,
   2 x M3 x 50 per side go up from the hanger's underside through hanger, deck and the tower's foot tab into a hex nut in the tab's top: a
   clamped stack, so the spring loop closes through these bolts and the deck carries only the net wheel load. Behind the pivot, 2 x M3 x 30
   per hanger up into the deck.
 * The BEAM's top is the deck's underside: 7 x M3 x 30 up from below into the deck (the 2 front ones driven with an L-key over the fork's bar)
   and an M3 x 50 up the droop lug's length.
 * TOWERS stand on the deck over the hangers (the springs and plungers push from them onto the arms' pads; since Nov 19 plunger_study.py's M3
   set: a PETG cap's R8 dome on the pad, an M3 x 45 button head in it, a printed Ø4.9 sleeve in each Ø5.0 guide, the overhang's underside the
   bump stop); one M3 x 40 down through each into the deck keeps it there with the pod out (the plungers' nylocs keep each spring's preload).
 * The pod drops out downward (motor unplugged through the deck's opening, the bolts and screws out from below): everything of it above the
   deck's underside passes through the deck's opening (CLR all round, every pose).
 * rx_deck here is a local stand-in for the robot deck (chassis_mid.py): a 20 mm plate with the pod's opening, for the pod-level checks.
"""
import json
import math
import pickle
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import union_all
from shapely.geometry import Polygon

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import pod_p4 as P4                                                    # noqa: E402
import pod_rx as R                                                    # noqa: E402
from pod_rx import box, along_x, cyl, min_dist, PIV, PIVOT_AXIS, POSES, A_DROOP, A_BUMP   # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
import hardware as H                                                   # noqa: E402
import pod_lean as L                                                   # noqa: E402
import base as B                                                       # noqa: E402
import wheel_motor as W                                                # noqa: E402

HERE = TOOLS.parent / 'validation' / 'pod_mid'
STEP_OUT = TOOLS.parent / 'step_export' / 'pod_mid'   # print STEPs + the assembly (checks and renders stay in validation/pod_mid)
NAME = 'pod_mid'
P = R.P
DECK_Z = (10.0, 30.0)              # pod frame (robot 80..100): over the pivot boss (r 8) + 2, under the towers' overhang (Z_TBOT 34)
HANGER_T = 14.0                    # Nov 14 (Fernando): the towers' base width (P4's frames: 8)
OUT_H = (R.OUTER_ARM[0] - R.STATIC_GAP - HANGER_T, R.OUTER_ARM[0] - R.STATIC_GAP)   # (outer face, inner face) = (-64.4, -50.4)
IN_H = (R.INNER_ARM[0] + R.STATIC_GAP + HANGER_T, R.INNER_ARM[0] + R.STATIC_GAP)    # (69.7, 55.7)
HANGER = dict(y=(29.5, 101.0), z=(-10.0, DECK_Z[0]), r_bot=4.0, r_top=1.0)  # front at 29.5: under the tower's front tab
HANGER_SCREW_Y = (96.5,)           # behind the pivot: M3 x 30 up into the deck (ahead of it: the long bolts into the towers), on the 8 mm part
                                   # (Nov 14, Fernando: at 91 / 96 their counterbores ran into each other; 8 apart. Nov 15 (Fernando: two very
                                   # close together, keep the far one): only the one at 96.5; the one at 88.5 is gone)
HANGER_REAR_T, HANGER_TAPER_Y = 8.0, (80.0, 87.0)   # Nov 14: 14 mm under the tower, tapering to 8 behind the pivot: the 14 mm back corner set
                                   # the robot's radius (r 232 -> Ø475; 8 mm: r 227, Ø463). Printed upright (underside on the bed) so the plan is free
BEAM_FRONT = [(P4.X_MID - 30.0, 94.0), (P4.X_MID + 30.0, 94.0)]   # Nov 15 evening (Fernando: 2 per side, diagonal, clear of the bar, + the middle)
BEAM_SCREWS = [(P4.X_MID - 16.0, 107.5), (P4.X_MID, 107.5), (P4.X_MID + 16.0, 107.5)] + BEAM_FRONT
                                   # each side: one at the front corner (y 94, behind the bar) and one at the back (y 107.5) inboard of it; the
                                   # middle one at the back. Was 7 (3 at the back, 2 beside the lug, 2 in front)
                                   # Nov 14 (Fernando: more screws): the two at y 99.5 beside the lug; round c: two at the front (y 86, over the
                                   # fork's bar: an L-key job). Nov 15 (Fernando: can't reach them, the bar is in the way): moved back to y 94,
                                   # beside the droop lug, so a straight driver from below passes behind the bar in every pose (checks.json
                                   # beam_front_screws_driver_to_fork)
BEAM_DRIVER_R, BEAM_DRIVER_L = 3.25, 60.0   # the driver checked under those two: the head's radius, this long down from the head
LUG_BOLT = (P4.X_MID, 93.6)        # Nov 14 (Fernando): an M3 x 50 up the lug's length (counterbored in its underside), through the beam into the
LUG_BOLT_L, LUG_CB_Z = 50.0, -27.6 # deck: it keeps the lug's layers in compression (the lug bends across them). Counterbore floor (pod z)
UP_SCREW_L = 30.0                  # M3 x 30 pan up from below: through the hanger / beam (Ø3.4, head counterbored) into the deck's Ø2.8 pilot
TOWER_OUT = 0.0                    # the tower's base = the hanger's width, right over it
PIVOT_SCREW_L_MID = 35.0           # the 14 mm hanger (P4's 8 mm frame: M3 x 30)
TOWER_Y = (P.Y_S - 12.0, P.Y_S + P4.TOWER_HALF_Y)
TOWER_TAB = dict(dy=12.0, t=12.0)  # a foot tab in front of and behind each tower, over the hanger, at deck level. Nov 14 (Fernando): 12 tall
                                   # (was 8): stronger, and the M3 x 50's end stands only 0.9 over the tab top (was 4.9), the nut under it
TOWER_SCREW_Y = (TOWER_Y[0] - TOWER_TAB['dy'] / 2, TOWER_Y[1] + TOWER_TAB['dy'] / 2)   # one long bolt in each tab (39 mm apart)
TOWER_SCREW_L = 50.0               # Nov 14 (Fernando: torque them; his longest M3 is 50): head counterbored in the hanger's underside, clearance
                                   # through hanger, deck and tab (20 + 20 + 12), an M3 hex nut in a 5.5 AF x 2.5 recess in the tab's top
TOWER_FIX_L = 40.0                 # Nov 14 (Fernando: the towers stay put with the pod out): one M3 down from the tower's top (counterbored)
TOWER_FIX_THREAD = 12.0            # through it into the deck (Ø2.8), in the middle of the tower. Nov 15: the taller tower (two springs in series):
                                   # an M3 x 40 (was x 30) in a counterbore deep enough for 12 mm of thread in the deck
NUT_WELL_AF = 5.7                  # Nov 15: the long bolts' nuts drop down a hex well from the tower's top onto its floor at the old tab-top height
                                   # (the tabs are full height now, so the tower prints top down without support); 0.1 a side over the nut
# Nov 15 (Fernando): shallow, wide alignment features where the pod meets the deck: a boss on the tower's base into a pocket in the deck's top, a
# boss on each hanger's and the beam's top into a pocket in the deck's underside (and the separate droop lug's boss into the beam)
ALIGN_H, ALIGN_CLR, ALIGN_DEPTH = 1.5, 0.15, 1.7   # boss height; pocket over the boss per side; pocket depth
ALIGN_INSET = 3.0                  # a boss stays this far inside the part's face (and the deck's edge: the checks report the walls)
ALIGN_R = 1.5                      # its plan corners
# Nov 15 (Fernando: the front right wheel's beam interferes with the deck's dovetail): the beam's top is relieved where the neighbouring deck
# piece's dovetail tab can lie over it (FR's beam under RR's tab on the 270 deg seam: pod-frame line x + y = 145.8). Cut on every beam (each hand
# the same part), so it lands on deck that is its own piece only; chassis_mid also keeps every tab off the pods' seats
BEAM_RELIEF = dict(line=143.0, depth='through')   # the beam cut away where x + y >= line (pod frame). Nov 15 evening (Fernando): through the
                                   # beam (was its top 1 mm), so that deck piece can come off while pod 2 and its deck stay
CLR = 2.0                          # the deck's opening round anything of the pod above its underside (every pose). Nov 14 review: 2.0 (was
                                   # 1.0), the wheel rule (pod_rx.CLR_WHEEL): the pod sits on screws in Ø3.4 holes (+-0.3), the deck is four
                                   # lapped prints, the treads wear and swell
CLR_HARNESS_DECK = 4.0             # Nov 14: round the XT30 + leads (they move with the fork; cad/solidworks.md 16: ~4 mm to what moves)
LOCAL = dict(x=(-78.0, 84.0), y=(-95.0, 122.0))   # the stand-in deck's extent


def hside(side):
    fr, arm = (OUT_H, R.OUTER_ARM) if side == 'o' else (IN_H, R.INNER_ARM)
    xo, xi = fr
    return fr, arm, xo, xi, (1.0 if xi > xo else -1.0)


# ---- hangers -----------------------------------------------------------------------------------------------------------------------------
def on_plane(e, axis, v):
    """the edge lies in the plane <axis> = v (axis 0, 1, 2)"""
    return all(abs(tuple(e.position_at(t))[axis] - v) < 1e-3 for t in (0, 0.5, 1))


def bed_chamfer(s, axis, v, info, key, c=R.BED_CHAMFER):
    """chamfer the edges of the bed face (<axis> = v); if OCC refuses, only its straight edges, else leave them (recorded)"""
    edges = [e for e in s.edges() if on_plane(e, axis, v)]
    for sel in (edges, [e for e in edges if e.geom_type.name == 'LINE'], [e for e in edges if e.geom_type.name == 'LINE' and e.length > 3.0]):
        try:
            out = b.chamfer(sel, c)
            if out.is_valid and len(out.solids()) == 1:
                info[key] = f'chamfer {c:g} on {len(sel)} of {len(edges)} bed edges'
                return out
        except Exception:                                              # noqa: BLE001
            pass
    info[key] = 'bed chamfer refused'
    return s


def align_boss(plan, z_face, up):
    """a shallow alignment boss: the plan (pod frame) ALIGN_H proud of the face z = z_face, toward up (+-1)"""
    z0, z1 = sorted((z_face - up * 0.01, z_face + up * ALIGN_H))
    return B.solid_of(plan, z0, z1)


def rounded_box(x0, x1, y0, y1, r=ALIGN_R):
    from shapely.geometry import box as sbox
    x0, x1 = sorted((x0, x1))
    return sbox(x0, y0, x1, y1).buffer(-r, 16, join_style=2).buffer(r, 16)


def hanger_boss_plan(side):
    """the hanger's alignment boss (pod frame plan): over its 14 mm part, ALIGN_INSET inside its faces and the deck's opening edge (the deck
    covers the hanger to 1 mm inside its inner face)"""
    fr, arm, xo, xi, s = hside(side)                                   # (it ends short of the rear long bolt: no hole across its edge)
    return rounded_box(xo + s * ALIGN_INSET, xi - s * (ALIGN_INSET + 1.0), HANGER['y'][0] + ALIGN_INSET, TOWER_SCREW_Y[1] - M3_CLR / 2 - 1.5)


def build_hanger(side, info):
    """the pivot hanger: 14 mm under the tower tapering to 8 behind the pivot, its top the deck's underside; the pivot boss, the pivot screw's
    counterbore and snug hole, 3 up-screws (Ø3.4 through, counterbored in its underside), the alignment boss on its top.
    Nov 15 (Fernando: the holes perpendicular to the bed): printed on its outer face, so the pivot boss, its spigot and the pivot screw's hole
    stand vertical (the up-screws' holes are then round horizontal roofs)"""
    from shapely.geometry import Polygon
    fr, arm, xo, xi, s = hside(side)
    (y0, y1), (z0, z1) = HANGER['y'], HANGER['z']
    xt = xi - s * HANGER_REAR_T                                        # the rear part's outer face
    ta, tb = HANGER_TAPER_Y
    plan = Polygon([(xi, y0), (xi, y1), (xt, y1), (xt, tb), (xo, ta), (xo, y0)])
    plan = plan.buffer(-2.0, 16, join_style=2).buffer(2.0, 16).buffer(4.0, 16).buffer(-4.0, 16)   # corners R2, the taper's inside bend R4
    import shapely
    h = B.solid_of(shapely.set_precision(plan, 0.01), z0, z1)          # on a 0.01 grid: the outer face is exactly x = xo (the bed)
    h = bed_chamfer(h, 0, xo, info, f'hanger_{side}_bed')               # the outer face on the bed (first: OCC refused it after the rounds)
    h = R.fillet_try(h, [e for e in h.edges() if on_plane(e, 2, z1) and abs(e.center().X - xo) > 1.0], (HANGER['r_top'], 0.6), info, f'hanger_{side}_top')
    h = R.fillet_try(h, [e for e in h.edges() if on_plane(e, 2, z0) and abs(e.center().X - xo) > 1.0], (1.0, 0.6), info, f'hanger_{side}_bottom')
    h = h + R.static_boss(arm, 'outer', shim=False) + align_boss(hanger_boss_plan(side), z1, 1)
    _, b_lip = R.brg_x(arm)
    h = h - along_x(R.CB_D / 2, *R.xs(xo - s * 0.1, xo + s * R.CB_DEPTH), *PIV) - along_x(M3_SNUG / 2, *R.xs(xo - s * 0.1, b_lip + s * (R.SPIGOT_L + 0.1)), *PIV)
    for x, y in [(hanger_x(side, y), y) for y in HANGER_SCREW_Y + TOWER_SCREW_Y]:
        h = h - cyl(M3_CLR / 2, z1 - z0 + ALIGN_H + 1, (x, y, (z0 + z1 + ALIGN_H) / 2)) - cyl(R.CB_D / 2, R.CB_DEPTH + 0.1, (x, y, z0 + R.CB_DEPTH / 2 - 0.05))
    return h


def hanger_x(side, y):
    """the x of a hanger screw: the middle of the 14 mm front (the long bolts), of the 8 mm rear (the screws behind the pivot)"""
    fr, arm, xo, xi, s = hside(side)
    return (xo + xi) / 2 if y < HANGER_TAPER_Y[0] else xi - s * HANGER_REAR_T / 2


def relief_plan():
    """BEAM_RELIEF's region (pod frame plan)"""
    L_ = BEAM_RELIEF['line']
    return Polygon([(L_ - 400.0, 400.0), (400.0, L_ - 400.0), (400.0, 400.0)])


def beam_boss_plan():
    """the beam's alignment boss (pod frame plan): its top less ALIGN_INSET (4 at the ends, where the deck's opening round the fork's pivot
    lobes comes 1 mm over them; 5 at the front, the wheel's opening), less the relief and ALIGN_INSET round it"""
    (x0, x1), (y0, y1) = P4.BEAM['x'], P4.BEAM['y']                     # its back edge short of the front screws (y 94): the rear pods' beams lie
    g = rounded_box(x0 + 4.0, x1 - 4.0, y0 + 5.0, BEAM_FRONT[0][1] - M3_CLR / 2 - 1.5)     # under the side fins' rear feet, whose screws (from
    g = g.difference(relief_plan().buffer(ALIGN_INSET))                 # below) sit over the beam's back half
    return g.buffer(-ALIGN_R, 16).buffer(ALIGN_R, 16)


def lug_xy():
    """(lx0, lx1, ly0, ly1) of the separate droop lug's top (on the beam's underside), pod frame"""
    lx0, lx1 = P4.X_MID - P4.LUG_HALF_W, P4.X_MID + P4.LUG_HALF_W
    z0 = P4.BEAM['z'][0]
    lug = box(lx0, lx1, *P4.LUG['y'], *P4.LUG['z']).rotate(PIVOT_AXIS, A_DROOP) & box(lx0 - 5, lx1 + 5, 50, 130, z0 - 0.3, z0)
    bb = lug.bounding_box()
    return lx0, lx1, bb.min.Y, bb.max.Y


def lug_boss_plan():
    lx0, lx1, ly0, ly1 = lug_xy()
    return rounded_box(lx0 + 2.5, lx1 - 2.5, ly0 + 2.5, ly1 - 2.5, 1.0)


def build_beam(info):
    """P4's beam (Nov 15: rebuilt here). The inboard support of both pivots, between the arms behind the wheel; its seven screws up through it
    into the deck (Ø3.4, counterbored in its underside), the lug bolt's hole, the alignment boss on its top, the relief under the neighbouring
    deck piece's dovetail tab. Nov 15 (Fernando: the holes perpendicular to the bed): printed standing on its outer (-x) end, so the pivot line
    and the inner boss's spigot stand vertical. For that its outer boss is a separate pin (build_pin) and the droop lug a separate part
    (build_lug) bolted under it (the lug bolt), located by a boss in a pocket in the beam's underside"""
    x0, x1 = P4.BEAM['x']
    y0, y1 = P4.BEAM['y']
    z0, z1 = P4.BEAM['z']
    k = box(x0, x1, y0, y1, z0, z1)
    vert = [e for e in k.edges() if abs(e.position_at(1).Z - e.position_at(0).Z) > 1]
    k = R.fillet_try(k, [e for e in vert if e.center().Y > (y0 + y1) / 2], (P4.BEAM_R_BACK,), info, 'beam_back_corners')
    vert = [e for e in k.edges() if abs(e.position_at(1).Z - e.position_at(0).Z) > 1 and e.center().Y < y0 + 1]
    k = R.fillet_try(k, vert, (P4.BEAM_R_FRONT,), info, 'beam_front_corners')
    try:                                                               # the top, on the deck's underside: chamfered
        k = b.chamfer([e for e in k.edges() if on_plane(e, 2, z1) and not on_plane(e, 0, x0)], 0.5)
    except Exception:                                                  # noqa: BLE001
        info['beam_top_chamfer'] = 'refused'
    k = R.fillet_try(k, [e for e in k.edges() if on_plane(e, 2, z0) and not on_plane(e, 0, x0)], (1.0, 0.6), info, 'beam_bottom_edges')
    k = bed_chamfer(k, 0, x0, info, 'beam_bed')                        # the outer end on the bed
    k = k + R.static_boss(R.INNER_ARM, 'wheel')                        # the inner boss (on top as printed); the outer one is the pin
    k = k.clean()
    k = R.fillet_try(k, [e for e in k.edges() if e.geom_type.name == 'CIRCLE' and abs(e.radius - R.BOSS_D / 2) < 1e-3
                         and abs(e.center().X - R.static_end_x(R.INNER_ARM)) < 1e-3], (1.0, 0.6), info, 'beam_boss_root')
    k = k - B.solid_of(relief_plan().intersection(rounded_box(x0 - 5, x1 + 5, y0 - 5, y1 + 5, 0.1)), z0 - 1.0, z1 + 1.0)   # through (Nov 15
                                                                       # evening: the neighbouring deck piece comes off with the pod in)
    k = k + align_boss(beam_boss_plan(), z1, 1)
    k = k - B.solid_of(lug_boss_plan().buffer(ALIGN_CLR, 16), z0 - 0.1, z0 + ALIGN_DEPTH)   # the lug's pocket
    k = k - along_x(M3_SOFT_PILOT / 2, x0 - 8, x1 + 8, *PIV)          # both pivot screws (a long thread: Ø2.9), now along the print axis
    zt = z1 + ALIGN_H
    for x, y in BEAM_SCREWS:
        k = k - cyl(M3_CLR / 2, zt - z0 + 1, (x, y, (z0 + zt) / 2)) - cyl(R.CB_D / 2, R.CB_DEPTH + 0.1, (x, y, z0 + R.CB_DEPTH / 2 - 0.05))
    x, y = LUG_BOLT                                                    # the lug bolt: clearance through the beam
    return k - cyl(M3_CLR / 2, zt - z0 + 2, (x, y, (z0 + zt) / 2))


def build_pin():
    """Nov 15: the beam's outer boss as a separate pin (the beam prints standing on that end): Ø10 from the beam's end to the 627's inner race,
    the Ø7.2 spigot 3 into its bore, snug Ø3.2 on the pivot screw (its steel core) which centres it and clamps hanger - race - pin - beam.
    Prints on its flange face, the spigot up"""
    arm = R.OUTER_ARM
    xo, xw, s = R.frame_x(arm)
    b_ring, _ = R.brg_x(arm)
    xe = R.static_end_x(arm)
    p = along_x(R.BOSS_D / 2, *R.xs(xe, b_ring), *PIV) + along_x(R.SPIGOT_D / 2, *R.xs(b_ring - s * 0.01, b_ring + s * R.SPIGOT_L), *PIV)
    p = p.clean()
    try:
        p = b.chamfer([e for e in p.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().X - (b_ring + s * R.SPIGOT_L)) < 1e-3
                       and abs(e.radius - R.SPIGOT_D / 2) < 1e-3], 0.3)          # a lead-in on the spigot's tip
    except Exception:                                                  # noqa: BLE001
        pass
    return p - along_x(M3_SNUG / 2, *R.xs(xe + 0.1, b_ring + s * (R.SPIGOT_L + 0.1)), *PIV)


def build_lug(info):
    """Nov 15: the droop stop's lug as a separate part under the beam (P4's lug: 24 wide on the centre plane, its front face the fork bar's back
    face at droop). Its top on the beam's underside with a boss in the beam's pocket (it cannot turn), the lug bolt (M3 x 50) up through it and
    the beam into the deck. Printed on its outer (-x) face, so the bar's push bends it in the layer planes (it used to bend across them)"""
    lx0, lx1 = P4.X_MID - P4.LUG_HALF_W, P4.X_MID + P4.LUG_HALF_W
    z0 = P4.BEAM['z'][0]
    lug = box(lx0, lx1, *P4.LUG['y'], *P4.LUG['z']).rotate(PIVOT_AXIS, A_DROOP) & box(lx0 - 5, lx1 + 5, 50, 130, -60, z0)
    lug = R.fillet_try(lug, [e for e in lug.edges() if not on_plane(e, 2, z0) and not on_plane(e, 0, lx0)], (1.5, 1.0, 0.6), info, 'lug_edges')
    lug = bed_chamfer(lug, 0, lx0, info, 'lug_bed')
    lug = lug + align_boss(lug_boss_plan(), z0, 1)
    x, y = LUG_BOLT
    return lug - cyl(M3_CLR / 2, 60.0, (x, y, z0 - 25.0)) - cyl(R.CB_D / 2, 6.0, (x, y, LUG_CB_Z - 3.0))


# ---- towers (on the deck) --------------------------------------------------------------------------------------------------------------------
def tower_boss_plan(side):
    """the tower's alignment boss (pod frame plan) under its base: ALIGN_INSET inside its outer face and ends, + 1 more at its inner face (the
    deck's opening edge is 1 mm inside it)"""
    fr, arm, xo, xi, s = hside(side)
    xb, xe = xo - s * TOWER_OUT, xi
    ty0, ty1 = TOWER_Y
    return rounded_box(xb + s * ALIGN_INSET, xe - s * (ALIGN_INSET + 1.0), ty0 - TOWER_TAB['dy'] + ALIGN_INSET, ty1 + TOWER_TAB['dy'] - ALIGN_INSET)


def tower_stop_z(y):
    """the bump stop's surface under a tower (pod y -> z): the pad's top at bump (pod_rx.rail_line)"""
    return R.rail_line(P.Y_S - 14.0, P.Y_S + 14.0)(y)


def m3():
    """plunger_study (the M3 plunger set, baseline since Nov 19), imported here, not at the top: it imports this module"""
    import plunger_study as PS
    return PS


_M3_PLUNGER = {}


def m3_plunger_at_origin():
    """one M3 plunger (cap + sleeve + steel) fused, axis at x = y = 0, pod z at nominal; built once"""
    if 'p' not in _M3_PLUNGER:
        PS = m3()
        p = PS.cap_m3() + PS.sleeve()
        for s in PS.hardware().values():
            p = p + s
        _M3_PLUNGER['p'] = p
    return _M3_PLUNGER['p']


def m3_spring(xc, y, lift):
    """the bore's two springs in series as one tube (the 8.2 OD's OD and ID) from the seat washer (lifted) up to the tower's ceiling; the coils
    are drawn only in the review STEPs and SolidWorks (sw_robot_parts.m3_plunger)"""
    PS = m3()
    sp = PS.SPRINGS[PS.IN_BORES[0]]
    z0 = PS.Z_SEAT + lift
    return cyl(sp['od'] / 2, P.Z_CEIL - z0, (xc, y, (P.Z_CEIL + z0) / 2)) - cyl(sp['od'] / 2 - sp['w'], P.Z_CEIL - z0 + 1.0, (xc, y, (P.Z_CEIL + z0) / 2))


def build_tower(side, info, stop=True):
    """a spring tower standing on the deck: its base on the deck's top over the hanger, its overhang over the arm (underside at Z_TBOT, the bump
    rails under it), pod.py's spring bores (two springs in series since Nov 15) and plunger guides.
    Nov 15 (Fernando: the holes perpendicular to the bed): printed TOP DOWN, so the spring bores, the plunger guides and every screw hole stand
    vertical; for that the foot tabs at each end run the tower's full height (no overhang under them) and the long bolts' nuts drop down a hex
    well from the top onto the well's floor at the old tab-top height. The alignment boss under its base (printed last, on top).
    stop (Nov 18, Fernando, tools/plunger_study.py; the baseline since Nov 19): the overhang's whole underside IS the bump stop's surface (the
    pad's top at bump, tower_stop_z), its full length, so the spring bores run down to it (no separate rails); drawn before the rounds like the
    rest. Nov 19 (Fernando: "make the M3 the baseline towers now"): the bores Ø plunger_study.BORE_D (8.8) and the guides printed
    Ø GUIDE_PRINT_D (5.0, drilled 13/64") for the M3 plunger's Ø4.9 sleeve (were Ø8.6 / Ø4.4 for the M4 bolt)"""
    fr, arm, xo, xi, s = hside(side)
    xc = P4.ARM_C[0] if side == 'o' else P4.ARM_C[1]
    xb = xo - s * TOWER_OUT                                            # the base's outer face
    xe = xi                                                            # the deck's opening edge (1 mm off the arm's outer face)
    ty0, ty1 = TOWER_Y
    dy = TOWER_TAB['dy']
    line_b = R.rail_line(P.Y_S - 14.0, P.Y_S + 14.0)
    if stop:
        t = (R.P.yz_poly([(ty0, line_b(ty0)), (ty1, line_b(ty1)), (ty1, P.Z_TT), (ty0, P.Z_TT)], *R.xs(xb, arm[1]))
             + box(*R.xs(xb, xe), ty0, ty1, DECK_Z[1], P.Z_TT))
    else:
        t = box(*R.xs(xb, arm[1]), ty0, ty1, P.Z_TBOT, P.Z_TT) + box(*R.xs(xb, xe), ty0, ty1, DECK_Z[1], P.Z_TT)
        # the bump rails (the hard stop: the arm's pad lands on them at bump) under the overhang, either side of the springs. Nov 14 (Fernando:
        # the stub was added after the rounds and left sharp): joined BEFORE the edge rounds, so they are rounded with the rest of the tower
        pad_front_at_bump = float(P.rot_pt((P.Y_S - 14.0, PAD_Z), A_BUMP)[0]) + 0.5
        wy = WASHER_OD / 2 + RAIL_CLEAR
        ax = R.xs(xe, arm[1])              # flush with the base's face (to the arm's outer face, a 1 mm ledge under the overhang refused every round)
        for ry in ((max(ty0, pad_front_at_bump), P.Y_PAIR[0] - wy), (P.Y_PAIR[1] + wy, ty1)):
            if ry[1] - ry[0] >= 1.0:
                t = t + R.P.yz_poly([(ry[0], line_b(ry[0])), (ry[1], line_b(ry[1])), (ry[1], P.Z_TBOT + 0.5), (ry[0], P.Z_TBOT + 0.5)], *ax)
    t = t.clean()
    zb, ztop = DECK_Z[1], P.Z_TT
    z_low = min(line_b(ty0), line_b(ty1)) - 0.1 if stop else P.Z_TBOT - 0.1   # the underside's lowest point

    def sel(t):                                                        # (not the overhang's inner corner against the base's face, at x = xe)
        return [e for e in t.edges() if not on_plane(e, 2, ztop) and not on_plane(e, 2, zb)
                and not (abs(e.center().X - xe) < 0.2 and abs(e.position_at(0).X - e.position_at(1).X) < 1e-3 and e.center().Z < P.Z_TBOT + 0.2)]

    def vertical(e):
        p0, p1 = e.position_at(0), e.position_at(1)
        return abs(p1.Z - p0.Z) > 1 and abs(p1.X - p0.X) < 1e-3 and abs(p1.Y - p0.Y) < 1e-3
    # Nov 15: rounded in two groups, the horizontal edges first, then the vertical ones (straight lines left): rounded all together, the outer
    # tower's mirror image (the other hand) came out invalid
    t, d1 = L.round_edges_each(t, [e for e in sel(t) if not vertical(e)], 2.0)
    t, d2 = L.round_edges_each(t, [e for e in sel(t) if vertical(e) and e.geom_type.name == 'LINE'], 2.0)
    info[f'tower_{side}_edges'] = f'R2 on {d1} horizontal + {d2} vertical'
    t = bed_chamfer(t, 2, ztop, info, f'tower_{side}_bed')             # the top on the bed
    # Nov 15 evening (Fernando: the long nut wells too deep, an M3 x 50 does not reach far and the nut is hard to fit; stubs as before, the middle
    # screw keeps the part together): the foot tab at each end is a 12 mm stub again, the nut in a recess in its top. Rounded on their own (OCC
    # refused the body + both) and joined to the body's ends. Printed top down, each stub's top hangs: support under the two stubs
    zt = zb + TOWER_TAB['t']
    for k, (ya, yb, yfree) in enumerate(((ty0 - dy, ty0 + 1.0, ty0 - dy), (ty1 - 1.0, ty1 + dy, ty1 + dy))):
        tab = box(*R.xs(xb, xe), ya, yb, zb, zt)
        free = [e for e in tab.edges() if e.center().Z > zb + 0.1
                and (abs(e.center().Y - yfree) < 1e-3 or (abs(e.center().Z - zt) < 1e-3 and abs(e.position_at(1).Y - e.position_at(0).Y) > 1))]
        tab = R.fillet_try(tab, free, (2.0, 1.5, 1.0), info, f'tower_{side}_tab{k}')
        t = (t + tab).clean()
    info[f'tower_{side}_mirror_valid'] = bool(t.mirror(b.Plane.XZ).is_valid)
    t = t + align_boss(tower_boss_plan(side), zb, -1)
    PS = m3()
    for y in P.Y_PAIR:
        t = t - cyl(PS.BORE_D / 2, P.Z_CEIL - z_low + 0.1, (xc, y, (P.Z_CEIL + z_low) / 2 - 0.05))
        t = t - cyl(PS.GUIDE_PRINT_D / 2, GUIDE_LEN + 0.2, (xc, y, P.Z_CEIL + GUIDE_LEN / 2))
    xm = (xb + xe) / 2
    for y in TOWER_SCREW_Y:                                            # the long bolts: clearance through the stub, the nut in a recess in its top
        t = t - cyl(M3_CLR / 2, zt - zb + ALIGN_H + 1.0, (xm, y, (zt + zb - ALIGN_H) / 2)) - H.hex_recess((xm, y, zt), (0, 0, -1))
    xf, yf = tower_fix_xy(side)                                        # the short screw down through the tower (counterbored from its top)
    zh = tower_fix_head_z()
    t = t - cyl(M3_CLR / 2, ztop - zb + ALIGN_H + 1.0, (xf, yf, (ztop + zb - ALIGN_H) / 2)) - cyl(R.CB_D / 2, ztop - zh + 0.1, (xf, yf, (ztop + zh) / 2 + 0.05))
    return t


def tower_fix_head_z():
    """the tower's short screw: its head's seat (pod z), for TOWER_FIX_THREAD of thread in the deck"""
    return DECK_Z[1] - TOWER_FIX_THREAD + TOWER_FIX_L


def tower_fix_xy(side=None):
    """the towers' short screws (pod frame xy): one per tower, in its foot tab"""
    out = []
    for sd in ('oi' if side is None else side):
        fr, arm, xo, xi, s = hside(sd)
        out.append(((xo - s * TOWER_OUT + xi) / 2, (TOWER_Y[0] + TOWER_Y[1]) / 2))
    return out if side is None else out[0]


def tower_screw_xy():
    out = []
    for side in 'oi':
        fr, arm, xo, xi, s = hside(side)
        xm = (xo - s * TOWER_OUT + xi) / 2
        out += [(xm, y) for y in TOWER_SCREW_Y]
    return out


def up_screw_xy():
    """hanger and beam screws (pod frame xy): up from below into the deck"""
    out = []
    for side in 'oi':
        fr, arm, xo, xi, s = hside(side)
        out += [(hanger_x(side, y), y) for y in HANGER_SCREW_Y]
    return out + list(BEAM_SCREWS) + [LUG_BOLT]


# ---- the pod's footprints and the local deck ---------------------------------------------------------------------------------------------
def envelope_plan(a, z_cut=None, dx=0.5):
    """plan (pod frame) of the wheel's swept envelope (every roller angle: pod_rx's revolved (x, r) profile) at fork angle a, above z_cut
    (None: all heights). The rollers' pose in the CAD is one angle only, so the opening must take the envelope"""
    from shapely.geometry import LineString, box as sbox
    env = R.wheel_envelope(None)
    ya, za = (float(v) for v in P.rot_pt((0.0, 0.0), a))
    x0, _, x1, _ = env.bounds
    rects = []
    for x in np.arange(x0, x1 + dx, dx):
        cut = env.intersection(sbox(x, -1.0, x + dx, 500.0))             # the bin's largest r (its mid-line undershot by up to 0.25)
        if cut.is_empty:
            continue
        rm = cut.bounds[3]
        dz = 0.0 if z_cut is None else max(0.0, z_cut - za)
        if rm <= dz:
            continue
        h = math.sqrt(rm * rm - dz * dz)
        rects.append(sbox(x, ya - h, x + dx, ya + h))
    return union_all(rects).buffer(0.01)


def footprints(parts, rm, rs, unit, cache_dir=HERE):
    """plan polygons (pod frame): 'above' = anything of the pod that moves or comes out with it, above the deck's underside, every pose (the deck's
    opening); 'moving' = the swept moving parts, all heights; 'pod_static' = hangers + beam + their screws (nominal); 'seats' = the hangers' and
    beam's tops; 'towers' = the towers' footprint. Cached against the sources"""
    srcs = [TOOLS / n for n in ('pod_mid.py', 'pod_p4.py', 'pod_rx.py', 'pod.py', 'pod_lean.py', 'wheel_motor.py', 'dims.py')]
    key = [round(s_.stat().st_mtime) for s_ in srcs]
    cache = cache_dir / 'pod_fp.pkl'
    if cache.exists():
        got = pickle.loads(cache.read_bytes())
        if got[0] == key and 'align_top' in got[1] and len(got[1]['up_screws']) == len(up_screw_xy()):                     # (a process holding older code can write the current key)
            return got[1]
    exact = ('pod_fork', 'rx_hanger', 'rx_beam', 'rx_tower', 'rx_lug', 'rx_pin')
    above, mov, pst, seats, tw, hz = [], [], [], [], [], []
    for pn, a in POSES.items():
        above.append(envelope_plan(a, DECK_Z[0] - 0.5))
        mov.append(envelope_plan(a))
        sh, moving, _ = assembly(parts, rm, rs, unit, a, deck=False)
        for k, s_ in sh.items():
            ex = k.startswith(exact)
            if k in moving:
                for lst, z0, z1 in ((mov, None, None), (above, DECK_Z[0] - 0.5, 300.0)):
                    g = B.proj(s_, ex, z0, z1)
                    if g is not None:
                        lst.append(g)
                if k.startswith(R.HARNESS):                            # Nov 14: the plug faces up, its leads come up through the deck
                    g = B.proj(s_, True, DECK_Z[0] - CLR_HARNESS_DECK, 300.0)
                    if g is not None:
                        hz.append(g)
            elif pn == 'nominal':
                if k.startswith(('rx_hanger', 'rx_beam', 'rx_lug', 'rx_pin', 'bolt_up', 'bolt_pivot', 'bolt_lug')):
                    g = B.proj(s_, ex)
                    if g is not None:
                        pst.append(g)
                    if k.startswith(('rx_hanger', 'rx_beam')):
                        g = B.proj(s_, True, DECK_Z[0] - 0.3, DECK_Z[0] + 0.01)
                        if g is not None:
                            seats.append(g)
                if k.startswith('rx_tower'):
                    g = B.proj(s_, True)
                    if g is not None:
                        tw.append(g)
    fp = dict(above=union_all(above), moving=union_all(mov), pod_static=union_all(pst), seats=union_all(seats), towers=union_all(tw),
              harness=union_all(hz) if hz else None,
              up_screws=up_screw_xy(), tower_screws=tower_screw_xy(), tower_fix=tower_fix_xy(),
              align_top=union_all([tower_boss_plan(sd) for sd in 'oi']),                                # Nov 15: pockets in the deck's top
              align_bottom=union_all([hanger_boss_plan(sd) for sd in 'oi'] + [beam_boss_plan()]))    # ... and its underside
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache.write_bytes(pickle.dumps((key, fp)))
    return fp


def between_arms(fp):
    """Nov 14 (Fernando: of no use): the deck between the two arms' slots, ahead of the beam, is left open (pod frame plan). It was a tongue held
    at the beam end only, with no screw or seat on it; open, the motor's leads (XT30 facing up) go straight up through it. Clipped to the
    convex hull of the pod's opening, so the deck's outline does not grow"""
    from shapely.geometry import box as sbox
    return sbox(R.OUTER_ARM[0], fp['above'].bounds[1], R.INNER_ARM[0], P4.BEAM['y'][0] - 1.0).intersection(fp['above'].convex_hull)


def opening(fp):
    """the deck's opening for one pod (pod frame plan): everything of the pod above the deck's underside in every pose + CLR, and the space
    between the arms ahead of the beam"""
    g = fp['above'].buffer(CLR, 32).union(between_arms(fp))
    if fp.get('harness') is not None:                                  # the XT30 and its leads: CLR_HARNESS_DECK all round (cad/solidworks.md 16)
        g = g.union(fp['harness'].buffer(CLR_HARNESS_DECK, 32))
    return g.union(tie_relief())


def tie_relief():
    """Nov 14 (harness): the motor plate's harness hole is at deck height, 1 mm from the opening's edge: the deck steps back beside it, so the
    zip tie's strap (through the hole, round the plate's top edge and the motor bundle lying on it) has room in every pose (pod frame plan)"""
    from shapely.geometry import box as sbox
    y, _, d = R.HARNESS_HOLE
    return sbox(R.INNER_ARM[0], y - TIE_RELIEF[1], R.INNER_ARM[0] + TIE_RELIEF[0], y + TIE_RELIEF[1])


TIE_RELIEF = (4.0, 4.0)            # the relief beside the harness hole: 4 out from the plate's outer face, +-4 along it (the strap is 3.6 wide)


def build_deck_local(fp):
    """the stand-in deck (pod frame): a 20 mm plate round the pod, less its opening; pilots for the up-screws, the towers' screws counterbored
    from below"""
    from shapely.geometry import box as sbox
    g = sbox(LOCAL['x'][0], LOCAL['y'][0], LOCAL['x'][1], LOCAL['y'][1]).difference(opening(fp))
    d = B.solid_of(g, *DECK_Z)
    z0, z1 = DECK_Z
    for x, y in fp['up_screws']:
        d = d - cyl(M3_PILOT / 2, z1 - z0 + 1, (x, y, (z0 + z1) / 2))
    for x, y in fp['tower_screws']:                                     # the long bolts pass through the deck
        d = d - cyl(M3_CLR / 2, z1 - z0 + 1, (x, y, (z0 + z1) / 2))
    for x, y in fp['tower_fix']:                                        # the towers' short screws thread into it
        d = d - cyl(M3_PILOT / 2, z1 - z0 + 1, (x, y, (z0 + z1) / 2))
    d = d - B.solid_of(fp['align_top'].buffer(ALIGN_CLR, 16), z1 - ALIGN_DEPTH, z1 + 1.0)      # Nov 15: the alignment pockets
    return d - B.solid_of(fp['align_bottom'].buffer(ALIGN_CLR, 16), z0 - 1.0, z0 + ALIGN_DEPTH)


# ---- records, parts, assembly --------------------------------------------------------------------------------------------------------------
def static_recs():
    out = []
    for side in 'oi':
        fr, arm, xo, xi, s = hside(side)
        out.append(dict(kind='pan', size=3.0, L=PIVOT_SCREW_L_MID, p=(xo + s * R.CB_DEPTH, *PIV), d=(s, 0.0, 0.0), name=f'bolt_pivot_{side}'))
    zh = HANGER['z'][0] + R.CB_DEPTH
    nh = 2 * len(HANGER_SCREW_Y)
    pts = up_screw_xy()
    for k, (x, y) in enumerate(pts[:-1]):
        z = zh if k < nh else P4.BEAM['z'][0] + R.CB_DEPTH
        out.append(dict(kind='pan', size=3.0, L=UP_SCREW_L, p=(x, y, z), d=(0, 0, 1.0), name=f'bolt_up_{k + 1}'))
    out.append(dict(kind='pan', size=3.0, L=LUG_BOLT_L, p=(*LUG_BOLT, LUG_CB_Z), d=(0, 0, 1.0), name='bolt_lug'))
    for k, (x, y) in enumerate(tower_screw_xy()):
        out.append(dict(kind='pan', size=3.0, L=TOWER_SCREW_L, p=(x, y, zh), d=(0, 0, 1.0), name=f'bolt_tower_{k + 1}'))
        out.append(dict(kind='hexnut', size=3.0, L=2.4, p=(x, y, DECK_Z[1] + TOWER_TAB['t'] - H.NUT_RECESS[1]), d=(0, 0, 1.0), name=f'nut_tower_{k + 1}'))
    for k, (x, y) in enumerate(tower_fix_xy()):
        out.append(dict(kind='pan', size=3.0, L=TOWER_FIX_L, p=(x, y, tower_fix_head_z()), d=(0, 0, -1.0), name=f'bolt_towerfix_{k + 1}'))
    return out


XT30_NOTCH = dict(depth=2.0, half_w=2.0)   # Nov 15: the V-notch in the motor plate's rim where the motor's XT30 lines up (the 4 rear holes take it
                                   # 4 ways round): this deep from the rim, this half-wide at it


def plug_azimuth():
    """the motor's XT30 azimuth about the axle (pod y-z plane, deg): the plug housing's centroid"""
    return P.lead_azimuth()


def rim_r(part, az, x, r0=15.0, r1=60.0):
    """the outer radius of the plate `part` along azimuth az at x (pod frame)"""
    c, s_ = math.cos(math.radians(az)), math.sin(math.radians(az))
    last = None
    for r in np.arange(r0, r1, 0.1):
        if part.is_inside(b.Vector(x, r * c, r * s_)):
            last = r
        elif last is not None and r > last + 1.0:
            break
    return last


def xt30_notch(inner):
    """the V-notch cutter through the motor plate at the plug's azimuth"""
    az = plug_azimuth()
    xo, xw = R.INNER_ARM
    rr = rim_r(inner, az, (xo + xw) / 2)
    d, hw = XT30_NOTCH['depth'], XT30_NOTCH['half_w']
    c, s_ = math.cos(math.radians(az)), math.sin(math.radians(az))
    t = (-s_, c)
    pts = [((rr - d) * c, (rr - d) * s_), ((rr + 1.0) * c + t[0] * (hw + 1.0), (rr + 1.0) * s_ + t[1] * (hw + 1.0)),
           ((rr + 1.0) * c - t[0] * (hw + 1.0), (rr + 1.0) * s_ - t[1] * (hw + 1.0))]
    return P.yz_poly(pts, *R.xs(xo + 0.1, xw - 0.1)), az, rr


def build(info=None):
    info = {} if info is None else info
    outer, inner, recs_moving = P4.build_fork(info)
    notch, az, rr = xt30_notch(inner)
    inner = inner - notch
    info['xt30_notch'] = dict(azimuth_deg=round(az, 1), rim_r=round(rr, 1), depth=XT30_NOTCH['depth'])
    parts = dict(pod_fork_outer=outer, pod_fork_inner=inner, rx_hanger_outer=build_hanger('o', info), rx_hanger_inner=build_hanger('i', info),
                 rx_beam=build_beam(info), rx_pin=build_pin(), rx_lug=build_lug(info), rx_tower_outer=build_tower('o', info),
                 rx_tower_inner=build_tower('i', info), rx_ring_outer=R.build_ring(R.OUTER_ARM), rx_ring_inner=R.build_ring(R.INNER_ARM),
                 pod_retainer=L.build_retainer())
    recs_moving = (recs_moving + R.ring_screw_recs(R.OUTER_ARM, 'o') + R.ring_screw_recs(R.INNER_ARM, 'i') + R.motor_screw_recs()
                   + [R.retainer_screw_rec()])
    return parts, recs_moving, static_recs(), info


# Nov 15 (Fernando: beam, tower and hanger print in another plane, their holes perpendicular to the bed): the hangers on their outer faces, the
# beam on its outer end, the towers top down; the new pin on its flange, the lug on its outer face
PRINTED_UP = dict(pod_fork_outer=(1, 0, 0), pod_fork_inner=(-1, 0, 0), rx_hanger_outer=(1, 0, 0), rx_hanger_inner=(-1, 0, 0), rx_beam=(1, 0, 0),
                  rx_pin=(-1, 0, 0), rx_lug=(1, 0, 0), rx_tower_outer=(0, 0, -1), rx_tower_inner=(0, 0, -1), rx_deck=(0, 0, 1),
                  rx_ring_outer=(1, 0, 0), rx_ring_inner=(-1, 0, 0), pod_retainer=(1, 0, 0))
STATIC = ('rx_hanger_outer', 'rx_hanger_inner', 'rx_beam', 'rx_pin', 'rx_lug', 'rx_tower_outer', 'rx_tower_inner')
STATIC_PRINTED = ('rx_hanger_outer', 'rx_hanger_inner', 'rx_beam', 'rx_pin', 'rx_lug', 'rx_tower_outer', 'rx_tower_inner', 'rx_deck')


# ---- assembly marks (Nov 15, Fernando): the pod's number and IN / OUT on its big parts, an up arrow on the forks, the XT30 mark -----------------
def marks(key, n):
    """the marks on part `key` of pod number n (labels.engrave specs, drawn pod frame)"""
    side = 'OUT' if key.endswith('outer') else 'IN'
    if key.startswith('rx_hanger'):
        fr, arm, xo, xi, s = hside('o' if side == 'OUT' else 'i')
        return [dict(text=f'{n} {side}', size=5.0, origin=((xo + xi) / 2, 56.0, HANGER['z'][0]), normal=(0, 0, -1), up=(1, 0, 0))]
    if key.startswith('rx_tower'):
        fr, arm, xo, xi, s = hside('o' if side == 'OUT' else 'i')
        return [dict(text=f'{n} {side}', size=6.0, origin=(xo - s * TOWER_OUT, sum(TOWER_Y) / 2, (DECK_Z[1] + P.Z_TT) / 2), normal=(-s, 0, 0), up=(0, 0, 1))]
    if key == 'rx_beam':
        # Nov 18 (Fernando: "in, out and up on the beam, it's confusing"): OUT points to its outer end (-x: the pin, the outer arm), IN to its
        # inner end (+x: the boss, the motor plate), UP to the deck. On the back face (seen from behind, u = -x) and on the underside (u = +x)
        (x0, x1), z0, yb = P4.BEAM['x'], P4.BEAM['z'][0], P4.BEAM['y'][1]
        back = dict(origin=(P4.X_MID, yb, 1.0), normal=(0, 1, 0), up=(0, 0, 1))
        under = dict(origin=(P4.X_MID, 100.0, z0), normal=(0, 0, -1), up=(0, -1, 0))
        return [dict(text=f'{n}', size=6.0, origin=(P4.X_MID - 10.0, 104.0, z0), normal=(0, 0, -1), up=(0, -1, 0)),
                dict(text='UP', size=5.0, arrow_to=(0, 0, 1), prefer=(0.0, 0.0), **back),
                dict(text='OUT', size=5.0, arrow_to=(-1, 0, 0), prefer=(P4.X_MID - x0 - 12.0, 0.0), **back),
                dict(text='IN', size=5.0, arrow_to=(1, 0, 0), prefer=(P4.X_MID - x1 + 14.0, 0.0), **back),
                dict(text='OUT', size=4.5, arrow_to=(-1, 0, 0), prefer=(x0 - P4.X_MID + 11.0, 0.0), **under),
                dict(text='IN', size=4.5, arrow_to=(1, 0, 0), prefer=(x1 - P4.X_MID - 11.0, 0.0), **under)]
    if key == 'pod_fork_outer':
        return [dict(text=f'{n} OUT', size=6.0, arrow=True, origin=(R.OUTER_ARM[0], 40.0, 0.0), normal=(-1, 0, 0), up=(0, 0, 1))]
    if key == 'pod_fork_inner':
        az = plug_azimuth()
        c, s_ = math.cos(math.radians(az)), math.sin(math.radians(az))
        o = (R.INNER_ARM[0], 0.0, 0.0)                                 # u = y, v = z on this face
        return [dict(text=f'{n} IN', size=6.0, arrow=True, origin=o, normal=(1, 0, 0), up=(0, 0, 1), prefer=(48.0, 2.0)),
                dict(text='XT30', size=4.0, arrow=True, origin=o, normal=(1, 0, 0), up=(0, c, s_), prefer=(0.0, 19.0))]   # the arrow points
                                                                       # out at the plug's azimuth, toward the rim's notch
    return []


LABELLED = ('rx_hanger_outer', 'rx_hanger_inner', 'rx_tower_outer', 'rx_tower_inner', 'rx_beam', 'pod_fork_outer', 'pod_fork_inner')
_LAB, _REG = {}, {}


def labelled(parts, i):
    """the pod parts of pod i (base.NAMES order) with its marks, in the drawn pod frame (a mirrored pod's marks are cut so they read right once
    the part is mirrored: labels.py); the other parts unchanged. Cached per process"""
    import labels as LB
    name = B.NAMES[i]
    if i not in _LAB:
        out, rep = dict(parts), {}
        for k in LABELLED:
            out[k], rep[k] = LB.engrave(parts[k], marks(k, LB.NUMBER[name]), mirrored=B.mirrored(i), region_cache=_REG, key=k)
        _LAB[i] = (out, rep)
    return _LAB[i][0]


def label_report():
    return {B.NAMES[i]: v[1] for i, v in sorted(_LAB.items())}


def assembly(parts, recs_moving, recs_static, unit, a, deck=True):
    moving = dict(unit)
    moving.update({k: parts[k] for k in ('pod_fork_outer', 'pod_fork_inner', 'rx_ring_outer', 'rx_ring_inner', 'pod_retainer')})
    moving['bearing_627_outer'] = R.bearing_627(R.OUTER_ARM)
    moving['bearing_627_inner'] = R.bearing_627(R.INNER_ARM)
    for r in recs_moving:
        moving[r['name']] = H.solid(r)
    out = {k: (v.rotate(PIVOT_AXIS, a) if a else v) for k, v in moving.items()}
    for k in STATIC + (('rx_deck',) if deck and 'rx_deck' in parts else ()):
        out[k] = parts[k]
    for r in recs_static:
        out[r['name']] = H.solid(r)
    lifts = {}
    PS = m3()                                                          # Nov 19: the M3 plunger (cap's R8 dome on the pad) and its springs
    for side, xc in (('o', P4.ARM_C[0]), ('i', P4.ARM_C[1])):
        for j, y in enumerate(P.Y_PAIR):
            lift = PS.lift_of(a, y)[0]
            lifts[f'{side}{j + 1}'] = lift
            out[f'plunger_{side}{j + 1}'] = m3_plunger_at_origin().moved(b.Location((xc, y, lift)))
            out[f'spring_{side}{j + 1}'] = m3_spring(xc, y, lift)
    return out, set(moving), lifts


def pose_gaps(sh, a):
    fork = R.FORK + ('rx_ring_outer', 'rx_ring_inner')
    drivers = [cyl(BEAM_DRIVER_R, BEAM_DRIVER_L, (x, y, P4.BEAM['z'][0] + R.CB_DEPTH - BEAM_DRIVER_L / 2)) for x, y in BEAM_FRONT]
    return dict(bar_to_lug_droop_stop=min_dist(sh['pod_fork_outer'], sh['rx_lug']),
                beam_front_screws_driver_to_fork=min(min_dist(d, sh[f]) for d in drivers for f in fork),   # Nov 15: a Ø6.5 driver straight down
                beam_front_screws_driver_to_lug=min(min_dist(d, sh['rx_lug']) for d in drivers),           # from each head, 60 long
                rails_to_pads=min(min_dist(sh[f], sh[k]) for f, k in (('rx_tower_outer', 'pod_fork_outer'), ('rx_tower_inner', 'pod_fork_inner'))),
                deck_to_fork_and_rings=min(min_dist(sh['rx_deck'], sh[f]) for f in fork),
                hangers_to_fork=min(min_dist(sh[h], sh[f]) for h in ('rx_hanger_outer', 'rx_hanger_inner') for f in fork),
                towers_base_to_fork=min(min_dist(sh[t] & box(-200, 200, -200, 200, DECK_Z[1] - 0.1, P.Z_TBOT - 0.2), sh[f])
                                        for t in ('rx_tower_outer', 'rx_tower_inner') for f in fork),
                plunger_top_washer_gap=m3().TOP_GAP + min(m3().lift_of(a, y)[0] for y in P.Y_PAIR))   # the lower top washer over the
                                                                       # tower's top (it lands there with the pod out)


def check(parts, rm, rs, unit, info, fp):
    span = abs(sum(R.brg_x(R.INNER_ARM)) / 2 - sum(R.brg_x(R.OUTER_ARM)) / 2)
    rep = R.run_checks(NAME, parts, rm, rs, unit, info, assembly, STATIC_PRINTED, PRINTED_UP, pose_gaps, R.front_springs_report(),
                       R.hand_loads(SPRING_ARM, False, 4, bearing_span=span))
    seat = fp['seats']
    rep['deck'] = dict(z_pod=DECK_Z, z_robot=[v + R_WHEEL for v in DECK_Z], opening_clearance_mm=CLR,
                       hanger_and_beam_tops_on_deck_pct=round(100 * seat.difference(opening(fp)).area / seat.area, 1),
                       pod_out_downward='the opening is the plan of everything of the pod above the deck underside in every pose, + CLR')
    rep['droop_stop'] = P4.droop_stop_report()
    rep['cartridge_screw_access'] = cartridge_access(parts['pod_fork_outer'])
    # Nov 15 review
    deck_solid = shapely_box_local().difference(opening(fp))
    walls = {}
    for nm, g in (('towers (deck top)', fp['align_top']), ('hangers + beam (deck underside)', fp['align_bottom'])):
        pk = g.buffer(ALIGN_CLR, 16)
        walls[nm] = dict(pocket_area_mm2=round(pk.area, 1), inside_deck=bool(deck_solid.buffer(0.01).contains(pk)),
                         min_wall_to_opening_mm=round(min(opening(fp).distance(q) for q in getattr(pk, 'geoms', [pk])), 2))
    rep['alignment'] = dict(boss_h=ALIGN_H, pocket_depth=ALIGN_DEPTH, clearance_per_side=ALIGN_CLR, walls=walls,
                            lug='the separate droop lug: a boss in a pocket in the beam\'s underside, the lug bolt through both',
                            note='shallow, wide bosses (cad/solidworks.md 18): the tower base, each hanger top, the beam top into deck pockets')
    rep['beam_relief'] = dict(BEAM_RELIEF, area_mm2=round(relief_plan().intersection(rounded_box(*P4.BEAM['x'], *P4.BEAM['y'], 0.1)).area, 1),
                              why='FR\'s beam corner lay under RR\'s dovetail tab (270 deg seam); chassis_mid checks every tab against the seats')
    rep['print_orientation'] = {k: dict(up=PRINTED_UP[k]) for k in STATIC_PRINTED + ('pod_fork_outer', 'pod_fork_inner')}
    rep['towers'] = dict(springs_in_series=SPRINGS_IN_SERIES, spring_bore_mm=round(P.Z_CEIL - P.Z_TBOT, 2), top_z_pod=round(P.Z_TT, 2),
                         top_z_robot=round(P.Z_TT + R_WHEEL, 2), height_over_deck=round(P.Z_TT - DECK_Z[1], 2),
                         plunger=(f'M3 x {m3().SCREW_L:g} button head dome down in a PETG cap (R{m3().DOME_R:g} on the pad), printed '
                                  f'Ø{m3().SLEEVE_D} sleeve in a Ø{m3().GUIDE_PRINT_D} guide (13/64" drill), 2 washers + nyloc on top '
                                  '(plunger_study.py, baseline Nov 19)'),
                         spring_bore_d=m3().BORE_D, bump_stop='the overhang\'s underside (stop=True)',
                         fix_screw=f'M3 x {TOWER_FIX_L:g}, head seat {tower_fix_head_z():.1f} (counterbore {P.Z_TT - tower_fix_head_z():.1f} deep), '
                                   f'{TOWER_FIX_THREAD:g} mm in the deck',
                         stack_preload_mm=STACK_PRELOAD,
                         long_bolt_nut=f'in a 5.5 AF x 2.5 recess in the top of each {TOWER_TAB["t"]:g} mm stub tab (Nov 15 evening; the hex wells are gone)')
    for i in range(4):
        labelled(parts, i)
    rep['marks'] = dict(numbers='1 FL, 2 FR, 3 RL, 4 RR (top view, front up)', per_pod=label_report(), xt30_notch=info.get('xt30_notch'))
    return rep


def shapely_box_local():
    from shapely.geometry import box as sbox
    return sbox(LOCAL['x'][0], LOCAL['y'][0], LOCAL['x'][1], LOCAL['y'][1])


def cartridge_access(fork_outer, step=10):
    """Nov 14: the wheel cartridge's 3 flat heads (r CART_R) driven from outside past the outer arm, with fork + cartridge put on the hub as
    one unit: at which wheel azimuths (pod y-z plane, 0 = toward the pivot) a head (Ø5.5) + 0.5 has a clear straight path out along -x"""
    clear = []
    for az in range(0, 360, step):
        y, z = W.at_angle(CART_R, az)
        path = along_x(2.75 + 0.5, R.OUTER_ARM[0] - 20.0, X_LEFT - 0.2, y, z)
        if R.overlap(path, fork_outer) < 1e-3:
            clear.append(az)
    return dict(head_r=CART_R, clear_azimuths_deg=clear, n_clear=len(clear), of=360 // step,
                note='turn the wheel to bring each screw into a clear azimuth; the three are 120 deg apart')


def render(parts, rm, rs, unit):
    HERE.mkdir(parents=True, exist_ok=True)
    for f in HERE.glob('*.png'):
        f.unlink()
    sh, moving, _ = assembly(parts, rm, rs, unit, 0.0)
    hide = ('roller_core', 'roller_sleeve', 'bearing_687', 'bolt_1', 'bolt_2', 'bolt_cartridge')
    view = {k: v for k, v in sh.items() if not k.startswith(hide)}
    R.iso(view, HERE / 'iso_motor_side.png', 235, 22, 'v15 mid-deck pod: hangers and beam under the deck, towers on it (motor side, nominal)')
    R.iso(view, HERE / 'iso_outer_side.png', 55, 22, 'v15 mid-deck pod: outer side')
    nodeck = {k: v for k, v in view.items() if not k.startswith('rx_deck')}
    R.iso(nodeck, HERE / 'iso_nodeck.png', 215, 25, 'deck hidden: hangers, beam, towers, fork, wheel')
    pod_only = {k: v for k, v in nodeck.items() if not k.startswith(('rx_tower', 'plunger', 'spring', 'bolt_tower'))}
    R.iso(pod_only, HERE / 'iso_pod_module.png', 215, 25, 'the module that drops out: fork, wheel, motor, hangers, beam (towers and springs stay)')
    R.iso({k: v for k, v in view.items() if not k.startswith(('roller', 'bearing_687', 'ak40', 'hub', 'tread', 'bolt_cart'))}, HERE / 'iso_below.png', 200, -35,
          'from below: the up-screws into the deck', size=(12, 9))
    sec = {k: v for k, v in sh.items() if not k.startswith(('roller_', 'bearing_687', 'bolt_1', 'bolt_2'))}
    R.section(sec, HERE / 'section_pivot_plane.png', 'z0', ((-75, 80), (25, 125)), 'section z 0 (pivot plane): hanger - 627 - beam - 627 - hanger')
    R.section(sec, HERE / 'section_spring_outer.png', ('x', P4.ARM_C[0]), ((-60, 125), (-45, 75)), f'section x {P4.ARM_C[0]}: tower on the deck, springs, arm, hanger')
    R.section(sec, HERE / 'section_hanger_outer.png', ('x', sum(OUT_H) / 2), ((-60, 125), (-45, 75)), 'section through the outer hanger: up-screws into the deck')
    for nm, a in (('droop', A_DROOP), ('bump', A_BUMP)):
        s2, _, _ = assembly(parts, rm, rs, unit, a)
        s2 = {k: v for k, v in s2.items() if not k.startswith(('roller_', 'bearing_687', 'bolt_1', 'bolt_2'))}
        R.section(s2, HERE / f'section_spring_{nm}.png', ('x', P4.ARM_C[0]), ((-80, 125), (-75, 90)), f'{nm}: section x {P4.ARM_C[0]}')


def main(stage):
    parts, rm, rs, info = build()
    for k, v in parts.items():
        print(k, len(v.solids()), v.is_valid, round(v.volume), flush=True)
    print(info, flush=True)
    if stage == 'build':
        return
    unit = R.wheel_unit()
    R.wheel_envelope(unit)
    fp = footprints(parts, rm, rs, unit)
    parts['rx_deck'] = build_deck_local(fp)
    HERE.mkdir(parents=True, exist_ok=True)
    if stage in ('check', 'all'):
        R.jdump(HERE / 'checks.json', check(parts, rm, rs, unit, info, fp))
        print('checks written', flush=True)
    if stage in ('export', 'all'):
        i_fr = B.NAMES.index('FR')                                     # the drawn hand, as pod 2 (FR)
        lab = labelled(parts, i_fr)
        sh, _, _ = assembly(lab, rm, rs, unit, 0.0)
        names = {r['name'] for r in rm + rs}
        printed = {k: v for k, v in parts.items() if k not in LABELLED}
        printed['rx_lug_m'] = parts['rx_lug'].mirror(b.Plane.XZ)       # the lug's other hand (the rest of the unmarked parts are round)
        print('exported', R.export(STEP_OUT, printed, {k: v for k, v in sh.items() if k not in names}, rm + rs, NAME), flush=True)
        PS = m3()                                                      # Nov 19: the M3 plunger's printed parts (4 of each per pod), in
        zt = PS.Z_HEAD_APEX + PS.CAP_M3['rim']                         # their print orientation (as plunger_study): the cap pocket down,
        R.write_part(PS.cap_m3().mirror(b.Plane.XY).moved(b.Location((0, 0, zt))), (1.0, 0.85, 0.0),   # the sleeve standing (PETG,
                     STEP_OUT / 'parts' / 'plunger_cap_m3.step', 'plunger_cap_m3')                     # SolidWorks' colours)
        R.write_part(PS.sleeve().moved(b.Location((0, 0, -PS.Z_SEAT))), (0.0, 0.85, 1.0), STEP_OUT / 'parts' / 'plunger_sleeve_m3.step',
                     'plunger_sleeve_m3')
        import labels as LB                                            # Nov 15: the marked parts once per pod, in that pod's hand
        for i, n in enumerate(B.NAMES):
            li = labelled(parts, i)
            for k in LABELLED:
                s = li[k].mirror(b.Plane.XZ) if B.mirrored(i) else li[k]
                R.write_part(s, R.colour(k), STEP_OUT / 'parts' / f'{k}_{LB.NUMBER[n]}_{n}.step', f'{k}_{LB.NUMBER[n]}')
    if stage in ('render', 'all'):
        render(parts, rm, rs, unit)
        print('rendered', flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
