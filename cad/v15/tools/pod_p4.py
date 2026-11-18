"""v15 pod library, layer 4 of 5 (pod.py -> pod_lean.py -> pod_rx.py -> pod_p4.py -> pod_mid.py; tools/README.md): cad/v14 Pod_reimagined_4's
fork (two 12 mm arm plates, the lower bar, a flat pad on each arm under its springs) and its beam (the inboard support of both pivots, with the
droop stop's lug). pod_mid.py adds the hangers, the towers and the deck. Library only, no entry point (P4's own side frames and deck stand-in
were the triangle study's: studies/triangle).

 * DROOP STOP: the bar's back face (28 below the pivot) lands on a lug that hangs from the beam, on the centre plane (v15: 24 wide, 12 thick).
 * Each pivot is supported on BOTH sides: the hanger's boss on the inner race's outer face, the beam's boss on its wheel-side face (no shim
   since Nov 14: pod_rx.SHIM_T). One M3 along the axis from the hanger threads into the beam: it clamps hanger - race - beam.
"""
import sys
from pathlib import Path

import build123d as b

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import pod_rx as R                                                     # noqa: E402
from pod_rx import box, along_x, cyl, PIV, PIVOT_AXIS, A_DROOP   # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403

P = R.P
ARM_C = (sum(R.OUTER_ARM) / 2, sum(R.INNER_ARM) / 2)                 # spring axes: the arms' centre planes
TOWER_HALF_Y = 15.0                                                    # the spring tower's half length behind the spring pair's centre
X_MID = 2.85                                                           # half way between the arms' centres
# fork: proposal 3's bar (its back face is the droop stop's moving half: 28 below the pivot, it moves back ~1 mm from nominal to droop)
BAR = dict(y=(74.0, 87.0), z=(-36.0, -20.0))
BAR_CIRCLES = [(78.0, -32.0, 4.0), (83.0, -32.0, 4.0), (78.0, BAR['z'][1] - 4.0, 4.0), (83.0, BAR['z'][1] - 4.0, 4.0)]
INSET = 3.0
ARM_EXTRA = [(y, z, r + INSET) for y, z, r in BAR_CIRCLES[:2]] + R.pad_circles()
TIE_LINES = [(80.5, -28.0, 45, 50)]
# the beam (static)
BEAM =dict(x=(R.static_end_x(R.OUTER_ARM), R.static_end_x(R.INNER_ARM)), y=(73.6, 112.0), z=(-8.0, 10.0))   # v15: back at 112 (was 118)
BEAM_R_FRONT, BEAM_R_BACK = 0.8, 4.0  # plan corners: the bosses (Ø10 at y 80.6) come within 2 mm of the front face, so its corners stay small
LUG = dict(y=(BAR['y'][1], BAR['y'][1] + 12.0), z=(-31.0, -4.0))     # the droop stop: hangs from the beam behind the bar; its front face is the
                                   # bar's back face as it stands at droop (the box turned by the droop angle about the pivot)
BEAM_SCREWS = [(X_MID - 22.0, 107.5), (X_MID + 22.0, 107.5)]   # Ø2.8 pilots P4 drew; pod_mid.BEAM_SCREWS drills all seven through Ø3.4
LUG_HALF_W = 12.0                  # Nov 14 (Fernando: the stop was excessive): a 24 mm lug on the centre plane (None: the beam's width).
                                   # LUG: down to the flat of the bar's back face (z -32..-24); 12 thick for the lug bolt's counterbore (was 9)


# ---- fork --------------------------------------------------------------------------------------------------------------------------------
def build_fork(info):
    oc, ic, recs = R.tie_lines_holes(TIE_LINES)
    outer = R.outer_arm(ARM_EXTRA, info, members=(R.member(BAR_CIRCLES),))
    outer = outer + R.tenon(*BAR['y'], *BAR['z'])
    for c in oc:
        outer = outer - c
    inner = R.motor_plate(ARM_EXTRA, info) - R.tenon(*BAR['y'], *BAR['z'], grow=R.TENON_CLR)
    for c in ic:
        inner = inner - c
    return outer, inner, list(recs)


# ---- the beam: the inboard support of both pivots ------------------------------------------------------------------------------------------
def build_beam(info):
    """between the arms, behind the wheel: a boss on each 627's wheel-side face (through the ring's bore, bearing on the race: pod_rx.SHIM_T 0),
    one Ø2.9 line for both pivot screws, the droop stop's lug hanging behind the fork's bar, its back on the deck's cross member. Prints top
    face down (the lug standing up)"""
    x0, x1 = BEAM['x']
    y0, y1 = BEAM['y']
    z0, z1 = BEAM['z']
    at_z = lambda e, z: all(abs(e.position_at(t).Z - z) < 1e-3 for t in (0, 0.5, 1))   # noqa: E731
    k = box(x0, x1, y0, y1, z0, z1)
    vert = [e for e in k.edges() if abs(e.position_at(1).Z - e.position_at(0).Z) > 1]
    k = R.fillet_try(k, [e for e in vert if e.center().Y > (y0 + y1) / 2], (BEAM_R_BACK,), info, 'beam_back_corners')
    vert = [e for e in k.edges() if abs(e.position_at(1).Z - e.position_at(0).Z) > 1 and e.center().Y < y0 + 1]
    k = R.fillet_try(k, vert, (BEAM_R_FRONT,), info, 'beam_front_corners')
    try:                                                               # the top prints on the bed: chamfered
        k = b.chamfer([e for e in k.edges() if at_z(e, z1)], 0.5)
    except Exception:                                                  # noqa: BLE001
        info['beam_bed_chamfer'] = 'refused'
    # the droop stop's lug, 3 mm in from the beam's ends (so its joins are interior loops, clear of the bosses and the ring screws)
    lx0, lx1 = (x0 + 3.0, x1 - 3.0) if LUG_HALF_W is None else (X_MID - LUG_HALF_W, X_MID + LUG_HALF_W)
    lug = box(lx0, lx1, *LUG['y'], *LUG['z']).rotate(PIVOT_AXIS, A_DROOP)
    k = (k + lug).clean()
    in_lug = lambda e, g=0.0: lx0 - g < e.center().X < lx1 + g and LUG['y'][0] - 1 - g < e.center().Y < LUG['y'][1] + 2 + g   # noqa: E731
    k = R.fillet_try(k, [e for e in k.edges() if at_z(e, z0) and in_lug(e, 0.1)], (2.0, 1.5, 1.0), info, 'lug_root')
    k = R.fillet_try(k, [e for e in k.edges() if e.center().Z < z0 - 3.0 and in_lug(e, 0.5)], (1.5, 1.0), info, 'lug_edges')
    k = R.fillet_try(k, [e for e in k.edges() if at_z(e, z0) and not in_lug(e, 3.0)], (1.0, 0.6), info, 'beam_bottom_edges')
    for arm in (R.OUTER_ARM, R.INNER_ARM):
        k = k + R.static_boss(arm, 'wheel')
    k = k.clean()
    k = R.fillet_try(k, [e for e in k.edges() if e.geom_type.name == 'CIRCLE' and abs(e.radius - R.BOSS_D / 2) < 1e-3
                         and any(abs(e.center().X - R.static_end_x(a)) < 1e-3 for a in (R.OUTER_ARM, R.INNER_ARM))], (1.0, 0.6), info, 'beam_boss_roots')
    k = k - along_x(M3_SOFT_PILOT / 2, x0 - 8, x1 + 8, *PIV)
    for x, y in BEAM_SCREWS:
        k = k - cyl(M3_PILOT / 2, z1 - z0 + 1, (x, y, (z0 + z1) / 2))   # along the print axis: Ø2.8
    return k


def droop_stop_report():
    """where the droop stop is and how it is drawn"""
    return dict(where='the fork bar\'s back face (28 below the pivot) lands on the face of a lug hanging from the beam, on the centre plane',
                drawn='the lug is a box turned by the droop angle about the pivot, so its face is the bar\'s back face at droop',
                travel_mm='nominal -> droop: the bar\'s back face moves back about 0.75 (top) to 1.3 (bottom)',
                replaces='the droop screws and their arc slots')
