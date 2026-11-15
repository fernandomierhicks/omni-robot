"""cad/v15 MID-DECK pod (Nov 14 2026, PROPOSAL, Fernando): the floor deck sits above the pivot and below the spring towers (robot z 80..100,
pod z +10..+30), and the pod hangs under it, bolted up from below with the threads in the deck. build123d, STEP review only.

    C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B pod_mid.py [build|check|export|render|all]      (checks: validation/pod_mid; STEPs: step_export/pod_mid)

From pod_p4.py (cad/v14 Pod_reimagined_4): the fork, the pivot housings, rings, the beam and its droop lug (no shims since Nov 14: exact-length bosses).
 * HANGERS replace P4's side frames: 14 mm plates outboard of the arms (the towers' base width: thinner bought no compactness, Fernando),
   z -10..+10, y 34..99, each with the pivot boss on the inner race (no shim this side) and an M3 x 35 pivot screw from its outer face into
   the beam. Ahead of the pivot, 2 x M3 x 50 go up from the hanger's underside through the hanger and the deck into the tower above it
   (clearance in the hanger and the deck, the thread in the tower): one stack, hanger - deck - tower, clamped. Behind the pivot, 2 x M3 x 30
   up into the deck. The pivot's loads go straight into the floor.
 * The BEAM's top is the deck's underside (z +10): 3 x M3 x 30 up from below into the deck (it keeps the droop lug).
 * TOWERS stand on the deck right over the hangers (separate printed parts, held by the hangers' long bolts): pod.py's springs and plungers
   push from them onto the arms' pads. With the pod out they rest loose on the deck (the plungers' nylocs keep each spring's preload in its
   tower).
 * The pod drops out downward (motor unplugged, the 11 hanger / beam screws and bolts out from below): everything of it above the deck's underside
   passes through the deck's opening in every pose.
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

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import pod_p4 as P4                                                    # noqa: E402  (sets the arm outline: pads, waist)
import pod_rx as R                                                     # noqa: E402
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
HANGER_SCREW_Y = (88.5, 96.5)      # behind the pivot: M3 x 30 up into the deck (ahead of it: the long bolts into the towers), on the 8 mm part
                                   # (Nov 14, Fernando: at 91 / 96 their counterbores ran into each other; 8 apart now)
HANGER_REAR_T, HANGER_TAPER_Y = 8.0, (80.0, 87.0)   # Nov 14: 14 mm under the tower, tapering to 8 behind the pivot: the 14 mm back corner set
                                   # the robot's radius (r 232 -> Ø475; 8 mm: r 227, Ø463). Printed upright (underside on the bed) so the plan is free
BEAM_SCREWS = [(P4.X_MID - 22.0, 107.5), (P4.X_MID, 107.5), (P4.X_MID + 22.0, 107.5), (P4.X_MID - 30.0, 99.5), (P4.X_MID + 30.0, 99.5),
               (P4.X_MID - 18.0, 86.0), (P4.X_MID + 18.0, 86.0)]   # y 86: off the pivot pilot (y 80.6, through the beam)
                                   # Nov 14 (Fernando: more screws): the two at y 99.5 beside the lug; round c: two at the front corners (the
                                   # front had none), head in the beam's underside, thread in the deck as the rest. The fork's bar is ~12 under
                                   # them: put them in the beam before the fork, drive them with an L-key (gap per pose: checks.json)
LUG_BOLT = (P4.X_MID, 93.6)        # Nov 14 (Fernando): an M3 x 50 up the lug's length (counterbored in its underside), through the beam into the
LUG_BOLT_L, LUG_CB_Z = 50.0, -27.6 # deck: it keeps the lug's layers in compression (the lug bends across them). Counterbore floor (pod z)
UP_SCREW_L = 30.0                  # M3 x 30 pan up from below: through the hanger / beam (Ø3.4, head counterbored) into the deck's Ø2.8 pilot
TOWER_OUT = 0.0                    # the tower's base = the hanger's width, right over it
PIVOT_SCREW_L_MID = 35.0           # the 14 mm hanger (P4's 8 mm frame: M3 x 30)
R.LEAD_TIE = False                 # Nov 14 (Fernando): no zip-tie tunnel in the motor plate's rim (its support can't be taken out); instead
R.HARNESS_HOLE = (26.0, 18.0, 4.6) # a Ø4.6 hole straight through the plate (x, its print axis: no support) ahead of the spring pads and the
                                   # inner hanger's front (y 29.5), 8 mm under the arm's top edge: the leads are tied there, then run with a
                                   # service loop onto the deck. 4.4 mm of wall to the nearest motor screw's counterbore, 2.3 to the can pocket
R.SHIM_T = 0.0                     # Nov 14 (Fernando): no shim washer; the beam's bosses are printed to the exact length (they bear on the 627s'
                                   # inner races): a pivot that binds gets its boss face sanded, one with play a shim added at assembly
P4.LUG_HALF_W = 12.0               # Nov 14 (Fernando: the stop was excessive): a 24 mm lug on the centre plane (was the beam's width, 72)
P4.LUG = dict(y=(P4.BAR['y'][1], P4.BAR['y'][1] + 12.0), z=(-31.0, -4.0))  # down to the flat of the bar's back face (z -32..-24); 12 thick for
                                   # the lug bolt's counterbore (was 9)
TOWER_Y = (P.Y_S - 12.0, P.Y_S + P4.TOWER_HALF_Y)
TOWER_TAB = dict(dy=12.0, t=12.0)  # a foot tab in front of and behind each tower, over the hanger, at deck level. Nov 14 (Fernando): 12 tall
                                   # (was 8): stronger, and the M3 x 50's end stands only 0.9 over the tab top (was 4.9), the nut under it
TOWER_SCREW_Y = (TOWER_Y[0] - TOWER_TAB['dy'] / 2, TOWER_Y[1] + TOWER_TAB['dy'] / 2)   # one long bolt in each tab (39 mm apart)
TOWER_SCREW_L = 50.0               # Nov 14 (Fernando: torque them; his longest M3 is 50): head counterbored in the hanger's underside, clearance
                                   # through hanger, deck and tab (20 + 20 + 12), an M3 hex nut in a 5.5 AF x 2.5 recess in the tab's top
TOWER_FIX_L = 30.0                 # Nov 14 (Fernando: the towers stay put with the pod out): one M3 x 30 down from the tower's top (counterbored)
                                   # through it into the deck (Ø2.8), in the middle of the tower
CLR = 1.0                          # the deck's opening round anything of the pod above its underside (every pose)
CLR_HARNESS_DECK = 4.0             # Nov 14: round the XT30 + leads (they move with the fork; cad/solidworks.md 16: ~4 mm to what moves)
LOCAL = dict(x=(-78.0, 84.0), y=(-95.0, 122.0))   # the stand-in deck's extent


def hside(side):
    fr, arm = (OUT_H, R.OUTER_ARM) if side == 'o' else (IN_H, R.INNER_ARM)
    xo, xi = fr
    return fr, arm, xo, xi, (1.0 if xi > xo else -1.0)


# ---- hangers -----------------------------------------------------------------------------------------------------------------------------
def build_hanger(side, info):
    """the pivot hanger: 8 mm plate (outer face on the bed), its top the deck's underside, the pivot boss, the pivot screw's counterbore and
    snug hole, 4 up-screws (Ø3.4 through, counterbored in its underside)"""
    from shapely.geometry import Polygon
    fr, arm, xo, xi, s = hside(side)
    (y0, y1), (z0, z1) = HANGER['y'], HANGER['z']
    xt = xi - s * HANGER_REAR_T                                        # the rear part's outer face
    ta, tb = HANGER_TAPER_Y
    plan = Polygon([(xi, y0), (xi, y1), (xt, y1), (xt, tb), (xo, ta), (xo, y0)])
    plan = plan.buffer(-2.0, 16, join_style=2).buffer(2.0, 16).buffer(4.0, 16).buffer(-4.0, 16)   # corners R2, the taper's inside bend R4
    h = B.solid_of(plan, z0, z1)
    h = R.fillet_try(h, [e for e in h.edges() if all(abs(e.position_at(t).Z - z1) < 1e-3 for t in (0, 0.5, 1))], (HANGER['r_top'], 0.6), info, f'hanger_{side}_top')
    try:                                                               # the underside prints on the bed: chamfered
        h = b.chamfer([e for e in h.edges() if all(abs(e.position_at(t).Z - z0) < 1e-3 for t in (0, 0.5, 1))], 0.6)
    except Exception:                                                  # noqa: BLE001
        info[f'hanger_{side}_bed_chamfer'] = 'refused'
    h = h + R.static_boss(arm, 'outer', shim=False)
    _, b_lip = R.brg_x(arm)
    h = h - along_x(R.CB_D / 2, *R.xs(xo - s * 0.1, xo + s * R.CB_DEPTH), *PIV) - along_x(M3_SNUG / 2, *R.xs(xo - s * 0.1, b_lip + s * (R.SPIGOT_L + 0.1)), *PIV)
    for x, y in [(hanger_x(side, y), y) for y in HANGER_SCREW_Y + TOWER_SCREW_Y]:
        h = h - cyl(M3_CLR / 2, z1 - z0 + 1, (x, y, (z0 + z1) / 2)) - cyl(R.CB_D / 2, R.CB_DEPTH + 0.1, (x, y, z0 + R.CB_DEPTH / 2 - 0.05))
    return h


def hanger_x(side, y):
    """the x of a hanger screw: the middle of the 14 mm front (the long bolts), of the 8 mm rear (the screws behind the pivot)"""
    fr, arm, xo, xi, s = hside(side)
    return (xo + xi) / 2 if y < HANGER_TAPER_Y[0] else xi - s * HANGER_REAR_T / 2


def build_beam(info):
    """P4's beam (bosses, the droop lug), its three screws now up through it into the deck (Ø3.4, counterbored in its underside)"""
    k = P4.build_beam(info)
    z0, z1 = P4.BEAM['z']
    for x, y in BEAM_SCREWS:
        k = k - cyl(M3_CLR / 2, z1 - z0 + 1, (x, y, (z0 + z1) / 2)) - cyl(R.CB_D / 2, R.CB_DEPTH + 0.1, (x, y, z0 + R.CB_DEPTH / 2 - 0.05))
    x, y = LUG_BOLT                                                    # the lug bolt: clearance up the lug and the beam, counterbored below
    k = k - cyl(M3_CLR / 2, z1 - (LUG_CB_Z - 5.0), (x, y, (z1 + LUG_CB_Z - 5.0) / 2)) - cyl(R.CB_D / 2, 6.0, (x, y, LUG_CB_Z - 3.0))
    return k


# ---- towers (on the deck) --------------------------------------------------------------------------------------------------------------------
def build_tower(side, info):
    """a spring tower standing on the deck: its base on the deck's top past the hanger, its overhang over the arm (underside at Z_TBOT, the bump
    rails under it), pod.py's spring bores and plunger guides, 2 pilots for the screws up through the deck"""
    fr, arm, xo, xi, s = hside(side)
    xc = P4.ARM_C[0] if side == 'o' else P4.ARM_C[1]
    xb = xo - s * TOWER_OUT                                            # the base's outer face
    xe = xi                                                            # the deck's opening edge (1 mm off the arm's outer face)
    ty0, ty1 = TOWER_Y
    t = box(*R.xs(xb, arm[1]), ty0, ty1, P.Z_TBOT, P.Z_TT) + box(*R.xs(xb, xe), ty0, ty1, DECK_Z[1], P.Z_TBOT + 0.5)
    # the bump rails (the hard stop: the arm's pad lands on them at bump) under the overhang, either side of the springs. Nov 14 (Fernando: the
    # stub was added after the rounds and left sharp): joined BEFORE the edge rounds, so they are rounded with the rest of the tower
    line_b = R.rail_line(P.Y_S - 14.0, P.Y_S + 14.0)
    pad_front_at_bump = float(P.rot_pt((P.Y_S - 14.0, PAD_Z), A_BUMP)[0]) + 0.5
    wy = WASHER_OD / 2 + RAIL_CLEAR
    ax = R.xs(xe, arm[1])                  # flush with the base's face (to the arm's outer face, a 1 mm ledge under the overhang refused every round)
    for ry in ((max(ty0, pad_front_at_bump), P.Y_PAIR[0] - wy), (P.Y_PAIR[1] + wy, ty1)):
        if ry[1] - ry[0] >= 1.0:
            t = t + R.P.yz_poly([(ry[0], line_b(ry[0])), (ry[1], line_b(ry[1])), (ry[1], P.Z_TBOT + 0.5), (ry[0], P.Z_TBOT + 0.5)], *ax)
    t = t.clean()
    t = R.fillet_try(t, [e for e in t.edges() if not R.on_x(e, xb) and e.center().Z > DECK_Z[1] + 0.1
                         and not (abs(e.center().Z - P.Z_TBOT) < 0.2 and abs(e.center().X - xe) < 0.2)], (2.0, 1.5, 1.0, 0.6), info, f'tower_{side}_edges')
    # the foot tabs (the long bolts), rounded on their own (all at once, OCC refused the body + both tabs) and joined to the body's ends
    for k, (ya, yb, yfree) in enumerate(((ty0 - TOWER_TAB['dy'], ty0 + 1.0, ty0 - TOWER_TAB['dy']), (ty1 - 1.0, ty1 + TOWER_TAB['dy'], ty1 + TOWER_TAB['dy']))):
        zt = DECK_Z[1] + TOWER_TAB['t']
        tab = box(*R.xs(xb, xe), ya, yb, DECK_Z[1], zt)
        free = [e for e in tab.edges() if not R.on_x(e, xb) and e.center().Z > DECK_Z[1] + 0.1
                and (abs(e.center().Y - yfree) < 1e-3 or (abs(e.center().Z - zt) < 1e-3 and abs(e.position_at(1).Y - e.position_at(0).Y) > 1))]
        tab = R.fillet_try(tab, free, (2.0, 1.5, 1.0), info, f'tower_{side}_tab{k}')
        t = (t + tab).clean()
    try:
        t = b.chamfer([e for e in t.edges() if R.on_x(e, xb)], R.BED_CHAMFER)
    except Exception:                                                  # noqa: BLE001
        pass
    for y in P.Y_PAIR:
        t = t - cyl((SPRING_HOLE_D + 0.6) / 2, P.Z_CEIL - P.Z_TBOT + 0.1, (xc, y, (P.Z_CEIL + P.Z_TBOT) / 2 - 0.05))
        t = t - cyl(GUIDE_R, GUIDE_LEN + 0.2, (xc, y, P.Z_CEIL + GUIDE_LEN / 2))
    xm = (xb + xe) / 2
    zt = DECK_Z[1] + TOWER_TAB['t']
    for y in TOWER_SCREW_Y:                                            # the long bolts: clearance through the tab, the nut in a recess in its top
        t = t - cyl(M3_CLR / 2, TOWER_TAB['t'] + 1.0, (xm, y, (zt + DECK_Z[1]) / 2)) - H.hex_recess((xm, y, zt), (0, 0, -1))
    xf, yf = tower_fix_xy(side)                                        # the short screw down through the tower (counterbored in its top)
    t = t - cyl(M3_CLR / 2, P.Z_TT - DECK_Z[1] + 1.0, (xf, yf, (P.Z_TT + DECK_Z[1]) / 2)) - cyl(R.CB_D / 2, R.CB_DEPTH + 0.1, (xf, yf, P.Z_TT - R.CB_DEPTH / 2 + 0.05))
    return t


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
        if got[0] == key and 'harness' in got[1] and len(got[1]['up_screws']) == len(up_screw_xy()):                       # (a process holding older code can write the current key)
            return got[1]
    exact = ('pod_fork', 'rx_hanger', 'rx_beam', 'rx_tower')
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
                if k.startswith(('rx_hanger', 'rx_beam', 'bolt_up', 'bolt_pivot')):
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
              up_screws=up_screw_xy(), tower_screws=tower_screw_xy(), tower_fix=tower_fix_xy())
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
    return d


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
        out.append(dict(kind='pan', size=3.0, L=TOWER_FIX_L, p=(x, y, P.Z_TT - R.CB_DEPTH), d=(0, 0, -1.0), name=f'bolt_towerfix_{k + 1}'))
    return out


def build(info=None):
    info = {} if info is None else info
    outer, inner, recs_moving = P4.build_fork(info)
    parts = dict(pod_fork_outer=outer, pod_fork_inner=inner, rx_hanger_outer=build_hanger('o', info), rx_hanger_inner=build_hanger('i', info),
                 rx_beam=build_beam(info), rx_tower_outer=build_tower('o', info), rx_tower_inner=build_tower('i', info),
                 rx_ring_outer=R.build_ring(R.OUTER_ARM), rx_ring_inner=R.build_ring(R.INNER_ARM),
                 pod_retainer=L.build_retainer())
    recs_moving = (recs_moving + R.ring_screw_recs(R.OUTER_ARM, 'o') + R.ring_screw_recs(R.INNER_ARM, 'i') + R.motor_screw_recs()
                   + [R.retainer_screw_rec()])
    return parts, recs_moving, static_recs(), info


PRINTED_UP = dict(pod_fork_outer=(1, 0, 0), pod_fork_inner=(-1, 0, 0), rx_hanger_outer=(0, 0, 1), rx_hanger_inner=(0, 0, 1), rx_beam=(0, 0, -1),
                  rx_tower_outer=(1, 0, 0), rx_tower_inner=(-1, 0, 0), rx_deck=(0, 0, 1), rx_ring_outer=(1, 0, 0), rx_ring_inner=(-1, 0, 0),
                  pod_retainer=(1, 0, 0))
STATIC = ('rx_hanger_outer', 'rx_hanger_inner', 'rx_beam', 'rx_tower_outer', 'rx_tower_inner')
STATIC_PRINTED = ('rx_hanger_outer', 'rx_hanger_inner', 'rx_beam', 'rx_tower_outer', 'rx_tower_inner', 'rx_deck')


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
    for side, xc in (('o', P4.ARM_C[0]), ('i', P4.ARM_C[1])):
        for j, y in enumerate(P.Y_PAIR):
            lift = P.plunger_lift(a, y)[0]
            lifts[f'{side}{j + 1}'] = lift
            out[f'plunger_{side}{j + 1}'] = P.plunger(xc, y).moved(b.Location((0, 0, lift)))
            out[f'spring_{side}{j + 1}'] = P.die_spring(xc, y, P.Z_CEIL - P.Z_SB - lift).moved(b.Location((0, 0, lift)))
    return out, set(moving), lifts


def pose_gaps(sh, a):
    fork = R.FORK + ('rx_ring_outer', 'rx_ring_inner')
    return dict(bar_to_lug_droop_stop=min_dist(sh['pod_fork_outer'], sh['rx_beam']),
                rails_to_pads=min(min_dist(sh[f], sh[k]) for f, k in (('rx_tower_outer', 'pod_fork_outer'), ('rx_tower_inner', 'pod_fork_inner'))),
                deck_to_fork_and_rings=min(min_dist(sh['rx_deck'], sh[f]) for f in fork),
                hangers_to_fork=min(min_dist(sh[h], sh[f]) for h in ('rx_hanger_outer', 'rx_hanger_inner') for f in fork),
                towers_base_to_fork=min(min_dist(sh[t] & box(-200, 200, -200, 200, DECK_Z[1] - 0.1, P.Z_TBOT - 0.2), sh[f])
                                        for t in ('rx_tower_outer', 'rx_tower_inner') for f in fork),
                plunger_nut_gap=NUT_GAP + min(P.plunger_lift(a, y)[0] for y in P.Y_PAIR),
                beam_front_screw_heads_to_fork=min(min_dist(sh[f'bolt_up_{2 * len(HANGER_SCREW_Y) + len(BEAM_SCREWS) - j}'], sh['pod_fork_outer'])
                                                   for j in (0, 1)))   # round c: the L-key's room under the beam's front screws


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
    return rep


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
        sh, _, _ = assembly(parts, rm, rs, unit, 0.0)
        names = {r['name'] for r in rm + rs}
        print('exported', R.export(STEP_OUT, parts, {k: v for k, v in sh.items() if k not in names}, rm + rs, NAME), flush=True)
    if stage in ('render', 'all'):
        render(parts, rm, rs, unit)
        print('rendered', flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')
