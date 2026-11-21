"""The spring plunger: the M3 set (baseline Nov 19 2026, Fernando: "make the M3 the baseline towers now"; designed Nov 18 in
cad/v15/tools/plunger_study.py, whose M4 cap try-out stays there).

    python -B plunger.py        the report (validation/plunger/report.json) and the plunger + towers at droop / nominal / bump
                                (step_export/plunger/plunger_<pose>.step: towers, plungers, coil springs)

Why: pushing the suspension by hand, each M4 plunger tilted in its guide (threads on PLA, Ø4.4 hole) and the bolt's head ground the fork's
pad (Fernando, Nov 18). From the pad up (Fernando: "the M3 needs 2 washers on top and the head + washer at the bottom where it contacts"): a PETG
cap whose R8 dome touches the pad, the M3 button head (5.5 x 1.8; SCREW_L, x 50 since Nov 20) dome down in it, an M3 washer on the head (the spring seat), the printed
sleeve up through the springs and the tower's guide to TOP_GAP over the tower's top, two M3 washers and the nyloc. Steel takes every load:
the sleeve is only in compression between the washers, so the guide sees smooth plastic from droop to bump. Since Nov 20 (the 7.85 x 4.25
springs) the sleeve is Ø4.0 with a Ø3.0 bore its whole length (0.5 wall, the M3 threads through it; was Ø4.9) and the guide is printed Ø4.1,
opened with a 4.2 drill (0.2 clearance; was Ø5.0 / 13/64"). With the pod out the top washers land on the tower's top (the stop). The towers
(pod_mid.build_tower, stop surface underside) have Ø8.3 spring bores (Nov 20; were Ø8.8).

pod_mid draws the plunger (cap + sleeve + steel) and one tube per bore for the springs, lifted by lift_of; sw_robot_parts / robot_step draw
each part on its own (sw_robot_parts.m3_pieces) and the coils. Springs (coil): flat wire, two in series per bore sharing the stack's length;
their radial width and coil count are ASSUMED (SPRINGS: VERIFY); solid length = coils x wire.
"""
import json
import math
import sys
from pathlib import Path

import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pod as P                                                        # noqa: E402
import hardware as H                                                   # noqa: E402
from dims import PAD_Z, PLUNGER_DOME_R, SPRINGS_IN_SERIES                                               # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT, VAL = ROOT / 'step_export' / 'plunger', ROOT / 'validation' / 'plunger'

# ---- springs -----------------------------------------------------------------------------------------------------------------------------
# Nov 18 (Fernando's picture of the yellow 20 mm spring): flat (die-spring) wire 1.0 mm thick along the axis with 1.5 mm gaps between the coils
# at free length (pitch 2.5), a closed turn at each end. Its radial width is not in the picture: 1.5 ASSUMED (the common "8 x 5 x 20" yellow
# spring; VERIFY: which drill shank slides through it gives the ID)
# Nov 20 (Fernando): the suspension springs are now 7.85 OD x 4.25 ID x 20.2 free (radial width 1.8 from OD / ID). Their wire thickness along
# the axis and free gap are not given: 1.0 / 1.5 ASSUMED as the yellow springs' (VERIFY: they set the solid length and the margin at bump).
SPRINGS = [dict(name='8.2 OD', od=8.2, L0=20.0, t=1.0, w=1.5, gap=1.5),
           dict(name='7.6 OD', od=7.6, L0=20.0, t=1.0, w=1.5, gap=1.5),
           # Nov 20 (Fernando): "12 mm is when spring coils touch each other": solid 12.0 MEASURED. The coil count is not: drawn as 1.0 wire
           # with a 0.72 free gap (10 active + 2 closed turns = 12 x 1.0), which gives that solid length
           dict(name='7.85 x 4.25 (Nov 20)', od=7.85, L0=20.2, t=1.0, w=1.8, gap=0.72,
                wire='solid 12.0 measured; 12 turns of 1.0 wire ASSUMED x 1.8 radial (from the OD / ID given)')]
CLOSED = 1.0                       # closed turns at each end (pitch = the wire's thickness)
IN_BORES = (2, 2)                  # which spring each bore holds (Nov 20: the 7.85 x 4.25, round the Ø4.0 sleeve)


def spring_numbers(sp):
    """active / total turns and solid length from the free length, the wire's thickness and the free gap"""
    na = (sp['L0'] - sp['t'] - 2 * CLOSED * sp['t']) / (sp['t'] + sp['gap'])
    return dict(active_turns=round(na, 2), total_turns=round(na + 2 * CLOSED, 2), solid_mm=round((na + 2 * CLOSED) * sp['t'], 2),
                id=round(sp['od'] - 2 * sp['w'], 2), travel_to_solid_mm=round(na * sp['gap'], 2))


# ---- the plunger ----------------------------------------------------------------------------------------------------------------------------
BORE_D = 8.3                       # the tower's spring bores (Nov 20, Fernando: 8.3, 0.45 round the 7.85 spring; was 8.8 for the 8.2 spring)
# Nov 20 (Fernando: "the screw can thread onto the sleeve so sleeve inner diam can be 3.0 mm. Make the sleeve be 0.4 mm wall"; he gauges the
# printed part): a Ø3.0 bore the whole length, the M3 threads through it; first Ø3.8 outside (0.4 wall), then as below.
# Was Ø4.9 (Nov 18) with a Ø2.9 x 12 thread at its top and Ø3.2 snug below, in a Ø5.0 guide opened with a 13/64" drill.
# Nov 20, second try (Fernando: "make the sleeve a little wider"; the slicer: "empty initial layer"): 0.5 wall, Ø4.0 (0.125 round inside the
# springs' 4.25 ID), no end chamfers (the 0.2 chamfer on the 0.4 wall left a 0.2 ring on the bed, under one line's width)
SLEEVE_BORE, SLEEVE_WALL = 3.0, 0.5
SLEEVE_D = SLEEVE_BORE + 2 * SLEEVE_WALL   # 4.0
GUIDE_PRINT_D, GUIDE_DRILL = 4.3, ('4.3 mm', 4.3)   # the tower's guide, printed vertical (Nov 20, Fernando: 4.3 for the M4; was 4.1 drilled
                                                     # 4.2), cleaned out with a 4.3 drill if it prints under: 0.3 clearance on Ø4.0
SCREW_TIP_PAST_NUT = 1.0           # the screw's tip at least this far past the nyloc's top (SCREW_L below, from the tower's height)
TOP_GAP = 1.5                      # the lower top washer over the tower's top at nominal

# Nov 20 (Fernando: "regenerate them for a M4 screw, no sleeve"): KIND picks the plunger set. 'M4': an ISO 7380 M4 button head, its dome straight
# on the pad (no cap: a cup round the 7.6 head would be over Ø8.3 and could not enter the bores at bump), a DIN 433 small M4 washer as the spring
# seat (Ø8.0: it enters the Ø8.3 bore at bump; a DIN 125 Ø9 would not), the bare M4 up through the springs (0.125 round it in their 4.25 ID) and
# the tower's guide, two DIN 125 M4 washers and an M4 nyloc on top. 'M3' (Nov 18-20): the PETG cap, the M3 button head, the printed sleeve.
KIND = 'M4'
SETS = dict(
    M4=dict(screw_max=50.0, head=(7.6, 2.2), hex=2.5, shaft_d=4.0, seat_washer=(8.0, 4.3, 0.5), top_washer=(9.0, 4.3, 0.8), nyloc=(7.0, 5.0), cap=False,
            sleeve=False),
    M3=dict(head=(5.5, 1.8), hex=2.0, shaft_d=SLEEVE_D, seat_washer=(6.85, 3.4, 0.6), top_washer=(6.85, 3.4, 0.6), nyloc=(5.5, 4.0), cap=True,
            sleeve=True))
S = SETS[KIND]
SIZE = 4.0 if KIND == 'M4' else 3.0
SHAFT_D = S['shaft_d']             # what slides in the tower's guide (the M4's threads; the M3's sleeve)
HEAD_D, HEAD_K = S['head']         # M4: ISO 7380 7.6 x 2.2; M3: Fernando's button heads 5.5 x 1.8 (Nov 18); the dome down
HEAD_R = ((HEAD_D / 2) ** 2 + HEAD_K ** 2) / (2 * HEAD_K)   # the head dome's sphere (M4 R 4.38, M3 R 3.0)
HEAD_HEX = S['hex']                # its hex socket, at the dome's apex
DOME_R = PLUNGER_DOME_R            # what touches the pad: the M4 head's own dome (R 4.38), or the M3 cap's R8; in dims.py (pod.Z_CEIL uses its lift)
# Nov 18 (Fernando: "model the PETG cap for the M3 head"): the head sits dome down in a PETG cup whose pocket is the head's own sphere (+0.05),
# so the load spreads over the dome, not the socket's rim; the head stands 0.2 proud of the rim (the washer bears on the head). The cup's bottom
# is an R8 dome CAP_M3['apex'] under the head's apex; it is what touches the pad. Prints pocket down. (M3 set only)
CAP_DOME_R = 8.0                   # the M3 cap's dome on the pad
CAP_M3 = dict(od=8.0, apex=1.2, rim=SETS['M3']['head'][1] - 0.2, seat_clr=0.05)
SEAT_WASHER = S['seat_washer']     # (OD, ID, t) on the head: the spring seat; it rises into the Ø8.3 bore at bump
TOP_WASHER = S['top_washer']       # two under the nyloc
NYLOC_AF, NYLOC_H = S['nyloc']

Z_HEAD_APEX = PAD_Z + (CAP_M3['apex'] if S['cap'] else 0.0)   # the head's dome apex (nominal, pod z): on the pad, or in the M3 cap
Z_HEAD_TOP = Z_HEAD_APEX + HEAD_K  # the head's bearing face
Z_SEAT = Z_HEAD_TOP + SEAT_WASHER[2]
Z_TOPW = P.Z_TT + TOP_GAP          # the lower top washer's underside (the M3 sleeve's top)
Z_SLEEVE_TOP = Z_TOPW
Z_NUT = Z_TOPW + 2 * TOP_WASHER[2]
SCREW_L = math.ceil((Z_NUT + NYLOC_H + SCREW_TIP_PAST_NUT - Z_HEAD_TOP) / 5.0) * 5.0   # the shortest that reaches (5 mm steps)
assert SCREW_L <= S.get('screw_max', 1e9), f'M{SIZE:g} x {SCREW_L:g}: longer than he has (Nov 20: M4 x 50 at most)'
assert Z_HEAD_TOP + SCREW_L >= Z_NUT + NYLOC_H + SCREW_TIP_PAST_NUT, f'M{SIZE:g} x {SCREW_L:g} too short for the towers'
Z_TIP = Z_HEAD_TOP + SCREW_L
assert abs(Z_SEAT - P.Z_SEAT_M3) < 1e-9, 'dims.PLUNGER_SEAT_OVER_PAD no longer matches the plunger'
assert abs(DOME_R - (HEAD_R if not S['cap'] else 8.0)) < 1e-3, 'dims.PLUNGER_DOME_R no longer matches the plunger'

GREY, PETG, STEEL, SPRING_COL = (0.33, 0.33, 0.33), (0.85, 0.85, 0.85), H.MAGENTA, (0.80, 0.80, 0.82)


def lift_of(a, y, R=DOME_R):
    """the plunger's rise at fork angle a (its dome, radius R, on the pad): (rise, where it touches along the pad from the point that was under
    it at nominal, how far the contact is from the dome's apex)"""
    p0 = P.rot_pt((y, PAD_Z), a)
    n = P.rot_pt((y, PAD_Z + 1.0), a) - p0
    zc = p0[1] + (R - n[0] * (y - p0[0])) / n[1]
    contact = (y, zc) - R * n
    along = float((contact - p0) @ [n[1], -n[0]])
    return float(zc - (PAD_Z + R)), along, float(abs(contact[0] - y))


def dome_disc(d, apex_z, h_top):
    """a Ø d disc from its R CAP_DOME_R dome's apex at apex_z (the lowest point) up to h_top (the M3 cap)"""
    sph = b.Solid.make_sphere(CAP_DOME_R).moved(b.Location((0, 0, apex_z + CAP_DOME_R)))
    cyl = b.Solid.make_cylinder(d / 2, h_top - apex_z, b.Plane(origin=(0, 0, apex_z)))
    return cyl & sph


RELIEF, RELIEF_FROM = 0.2, 0.5     # the cap's pocket: a straight wall head_d / 2 + RELIEF from half the head's height up (the head's dome bears
                                   # on the conformal lower half, outside its socket)
WASHER_WALL = 0.6                  # Nov 18 (Fernando: the caps don't stay put): the cap's wall round the washer (Nov 20: 0.6, was 0.75, so the
                                   # Ø8.05 cap enters the Ø8.3 bore at bump with 0.125 round it)
WASHER_TOP_UNDER = 0.1             # the wall's top this far under the washer's top (the spring bears on the washer, not on the cap)
LEDGE_UNDER = 0.2                  # the cap's floor round the head this far under the head's top (the washer bears on the head, not the cap)


def head_sphere(head_d, head_k):
    """the sphere of a domed (button / pan) head d x k"""
    return ((head_d / 2) ** 2 + head_k ** 2) / (2 * head_k)


def dome_cap(head_d, head_k, od, apex, z0=0.0, seat_clr=0.05, washer=None, wall=WASHER_WALL):
    """a PETG cup for a domed screw head, dome down: its own R DOME_R dome's apex at z0 (on the pad), the head's apex `apex` above it in a pocket
    that is the head's sphere (+ seat_clr) round its lower half and a straight relief round its upper half.
    Nov 18 (Fernando: the caps work but won't stay put; the head in its spherical pocket is a ball joint, so the sliding pad rocked the cap off
    it): the wall rises past the head round the washer (washer = (od, id, t)): a Ø washer-OD pocket (it prints a little under: the washer presses
    in snug) from LEDGE_UNDER under the head's top to WASHER_TOP_UNDER under the washer's top. The washer, clamped on the head by the spring,
    keeps the cap from tilting off, and its snug fit holds the cap on with the pod out. od is then the washer's + 2 x wall"""
    R = head_sphere(head_d, head_k)
    za = z0 + apex                                                     # the head's apex
    zh = za + head_k                                                   # the head's top (the washer's seat)
    if washer:
        od = washer[0] + 2 * wall
        zr = zh + washer[2] - WASHER_TOP_UNDER                         # the rim, round the washer
    else:
        zr = zh - 0.2
    s = dome_disc(od, z0, zr)
    s = s - b.Solid.make_sphere(R + seat_clr).moved(b.Location((0, 0, za + R + seat_clr)))   # the head's dome bears on its lower half
    s = s - b.Solid.make_cylinder(head_d / 2 + RELIEF, zr - (za + RELIEF_FROM * head_k) + 1.0,   # a straight relief round its upper half,
                                  b.Plane(origin=(0, 0, za + RELIEF_FROM * head_k)))          # so the printed pocket takes the head
    if washer:
        s = s - b.Solid.make_cylinder(washer[0] / 2, zr - (zh - LEDGE_UNDER) + 1.0, b.Plane(origin=(0, 0, zh - LEDGE_UNDER)))
    try:
        s = b.fillet([e for e in s.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().Z - zr) < 1e-3 and e.radius > od / 2 - 0.1], 0.3)
    except Exception:                                                  # noqa: BLE001
        pass
    return s


def cap_m3():
    """(M3 set) installed frame, pod z at nominal: the cup's dome apex on the pad (PAD_Z), the head's apex CAP_M3['apex'] above it in a spherical
    pocket"""
    c, m3 = CAP_M3, SETS['M3']
    return dome_cap(*m3['head'], c['od'], c['apex'], PAD_Z, c['seat_clr'], washer=m3['seat_washer'])


def sleeve():
    """the printed sleeve, pod frame at nominal (axis x = y = 0): Z_SEAT .. Z_SLEEVE_TOP, Ø SLEEVE_D with a Ø SLEEVE_BORE bore through (the M3
    threads through it); plain square ends: a chamfer on so thin a wall leaves the slicer an empty first layer"""
    L = Z_SLEEVE_TOP - Z_SEAT
    s = b.Solid.make_cylinder(SLEEVE_D / 2, L, b.Plane(origin=(0, 0, Z_SEAT)))
    return s - b.Solid.make_cylinder(SLEEVE_BORE / 2, L + 0.2, b.Plane(origin=(0, 0, Z_SEAT - 0.1)))


def printed_parts():
    """the plunger's printed parts, installed (pod z at nominal, axis at 0, 0): {name: solid}; none for the M4 set"""
    out = {}
    if S['cap']:
        out['plunger_cap'] = cap_m3()
    if S['sleeve']:
        out['plunger_sleeve'] = sleeve()
    return out


def ring(od, idd, t, z0):
    return b.Solid.make_cylinder(od / 2, t, b.Plane(origin=(0, 0, z0))) - b.Solid.make_cylinder(idd / 2, t + 0.2, b.Plane(origin=(0, 0, z0 - 0.1)))


def hardware(lift=0.0):
    """the steel of one plunger (pod frame, axis at 0, 0), lifted by `lift`: {name: solid}. The thread is drawn as its major diameter"""
    z0 = Z_HEAD_APEX                                                   # the head's dome apex
    head = (b.Solid.make_cylinder(HEAD_D / 2, HEAD_K, b.Plane(origin=(0, 0, z0)))
            & b.Solid.make_sphere(HEAD_R).moved(b.Location((0, 0, z0 + HEAD_R))))
    head = head - b.extrude(b.Plane(origin=(0, 0, z0 - 0.1)) * b.RegularPolygon(HEAD_HEX / math.sqrt(3), 6), amount=1.2)
    nut = b.extrude(b.Plane(origin=(0, 0, Z_NUT)) * b.RegularPolygon(NYLOC_AF / math.sqrt(3), 6), amount=NYLOC_H)
    nut = nut - b.Solid.make_cylinder(SIZE / 2, NYLOC_H + 0.2, b.Plane(origin=(0, 0, Z_NUT - 0.1)))
    out = dict(screw=head + b.Solid.make_cylinder(SIZE / 2, SCREW_L, b.Plane(origin=(0, 0, Z_HEAD_TOP))),
               washer_seat=ring(*SEAT_WASHER, Z_HEAD_TOP),
               washer_top_1=ring(*TOP_WASHER, Z_TOPW), washer_top_2=ring(*TOP_WASHER, Z_TOPW + TOP_WASHER[2]),
               nyloc=nut)
    return {k: v.moved(b.Location((0, 0, lift))) for k, v in out.items()}


def coil(sp, z0, length):
    """a spring as a coil of flat wire (sp['w'] radial x sp['t'] along the axis): CLOSED turns at each end (pitch = the thickness, +4 % so the
    turns do not fuse), the active turns evenly pitched over the rest; the wire's bottom at z0, its top at z0 + length (OD sp['od']).
    Returns (solid, active pitch, gap between active coils)"""
    t, w = sp['t'], sp['w']
    R = (sp['od'] - w) / 2                                             # the section's middle
    na = spring_numbers(sp)['active_turns']
    pe = t * 1.04
    p = (length - t - 2 * CLOSED * pe) / na

    def prof(e):
        o, tg = e.position_at(0), e.tangent_at(0)
        rad = b.Vector(o.X, o.Y, 0).normalized()
        return b.Plane(origin=o, x_dir=rad, z_dir=tg) * b.Rectangle(w, t)
    parts, z, ang = [], z0 + t / 2, 0.0
    for turns, pitch in ((CLOSED, pe), (na, p), (CLOSED, pe)):
        hx = b.Helix(pitch, turns * pitch, R, center=(0, 0, z)).rotate(b.Axis.Z, ang)
        parts.append(b.sweep(prof(hx), hx, is_frenet=True))
        z += turns * pitch
        ang += turns * 360.0
    return b.Compound(parts), p, p - t


def assembly(towers, a, name):
    """the towers, plungers (steel, and the M3 set's cap and sleeve) and coil springs at fork angle a, written to step_export/plunger; returns the spring report"""
    from step_colour import write_step
    items, rep = {}, []
    for side, (t, xc) in towers.items():
        items[f'rx_tower_{side}'] = (t, GREY)
        for j, y in enumerate(P.Y_PAIR):
            lift = lift_of(a, y)[0]
            at = b.Location((xc, y, lift))
            for k, s in printed_parts().items():
                items[f'{k}_{side}{j}'] = (s.moved(at), PETG)
            for k, s in hardware().items():
                items[f'{k}_{side}{j}'] = (s.moved(at), STEEL)
            sp = SPRINGS[IN_BORES[j]]                                  # the two in a bore share the stack (equal rates)
            stack = P.Z_CEIL - (Z_SEAT + lift)
            each = stack / SPRINGS_IN_SERIES
            for k in range(SPRINGS_IN_SERIES):
                c, pitch, gap = coil(sp, Z_SEAT + lift + k * each, each)
                items[f'spring_{side}{j}_{k}'] = (c.moved(b.Location((xc, y, 0))), SPRING_COL)
            if side == 'o':
                solid_len = spring_numbers(sp)['solid_mm']
                rep.append(dict(bore=j, spring=sp['name'], lift_mm=round(lift, 2), stack_mm=round(stack, 2), each_mm=round(each, 2),
                                compressed_each_mm=round(sp['L0'] - each, 2), to_solid_each_mm=round(each - solid_len, 2),
                                active_pitch=round(pitch, 2), gap_between_coils=round(gap, 2)))
    write_step(items, OUT / f'plunger_{name}.step', f'plunger_{name}')
    return rep


def main():
    import pod_mid as PM
    OUT.mkdir(parents=True, exist_ok=True)
    VAL.mkdir(parents=True, exist_ok=True)
    towers = {side: (PM.build_tower(side, {}), PM.P4.ARM_C[0] if side == 'o' else PM.P4.ARM_C[1]) for side in ('o', 'i')}
    poses = {}
    for name, a in (('droop', PM.A_DROOP), ('nominal', 0.0), ('bump', PM.A_BUMP)):
        poses[name] = assembly(towers, a, name)
        print(name, json.dumps(poses[name]), flush=True)
    lift_max = max(lift_of(PM.A_BUMP, y)[0] for y in P.Y_PAIR)
    sp_rep = [dict(name=s['name'], od=s['od'], wire=s.get('wire', f"{s['t']} thick (picture) x {s['w']} radial (ASSUMED)"), free_gap=s['gap'],
                   **spring_numbers(s), shaft_clearance_in_id=round(s['od'] - 2 * s['w'] - SHAFT_D, 2),
                   bore_clearance=round(BORE_D - s['od'], 2), seat_washer_covers_wire=bool(SEAT_WASHER[0] >= s['od'] - 0.2),
                   drawn=[IN_BORES[j] == SPRINGS.index(s) for j in range(len(IN_BORES))])
              for s in SPRINGS]
    sz = f'M{SIZE:g}'
    hw = hardware()
    rep = dict(
        kind=KIND,
        stack_from_the_pad=([f'{sz} x {SCREW_L:g} button head ({HEAD_D:g} x {HEAD_K:g}, R{HEAD_R:.2f} dome) dome down on the pad']
                            if not S['cap'] else ['PETG cap (R8 dome on the pad)', f'{sz} x {SCREW_L:g} button head, dome down in the cap'])
                           + [f'{sz} washer {SEAT_WASHER[0]:g} x {SEAT_WASHER[2]:g} (the spring seat)']
                           + ([f'printed sleeve Ø{SLEEVE_D:g}'] if S['sleeve'] else [f'the bare {sz} through the springs and the guide'])
                           + [f'2 x {sz} washer {TOP_WASHER[0]:g} x {TOP_WASHER[2]:g}', f'{sz} nyloc'],
        z_pod=dict(head=[round(Z_HEAD_APEX, 2), round(Z_HEAD_TOP, 2)], seat=round(Z_SEAT, 2), top_washers=round(Z_TOPW, 2),
                   nut=[round(Z_NUT, 2), round(Z_NUT + NYLOC_H, 2)], screw_tip=round(Z_TIP, 2), tip_past_nut_mm=round(Z_TIP - Z_NUT - NYLOC_H, 2)),
        in_the_guide=dict(shaft_d=SHAFT_D, what='the M4 thread (major diameter; no sleeve)' if not S['sleeve'] else 'the printed sleeve'),
        guide=dict(length=round(P.Z_TT - P.Z_CEIL, 2), printed_d=GUIDE_PRINT_D, drill=GUIDE_DRILL, clearance_mm=round(GUIDE_DRILL[1] - SHAFT_D, 3),
                   tilt_deg=round(math.degrees(math.atan((GUIDE_DRILL[1] - SHAFT_D) / (P.Z_TT - P.Z_CEIL))), 2)),
        head_radial_clearance_in_bore_mm=round((BORE_D - HEAD_D) / 2, 3),
        seat_washer_radial_clearance_in_bore_mm=round((BORE_D - SEAT_WASHER[0]) / 2, 3),
        seat_washer_to_tower_at_bump_mm=round(P.Z_TBOT - (Z_SEAT + lift_max), 2),
        dome_contact=f'R{DOME_R:.2f}: the contact stays within {max(lift_of(a, y)[2] for a in (PM.A_DROOP, PM.A_BUMP) for y in P.Y_PAIR):.2f} mm of its apex',
        nut_top_at_bump_robot_z=round(Z_NUT + NYLOC_H + lift_max + PM.R_WHEEL, 1),
        screw_tip_at_bump_robot_z=round(Z_TIP + lift_max + PM.R_WHEEL, 1),
        steel_valid={k: bool(v.is_valid) for k, v in hw.items()},
        springs=sp_rep, bore_d=BORE_D, poses=poses)
    (VAL / 'report.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
    print(json.dumps({k: v for k, v in rep.items() if k != 'poses'}, indent=1), flush=True)


if __name__ == '__main__':
    main()
