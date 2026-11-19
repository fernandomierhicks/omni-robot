"""Nov 18 (Fernando): the spring plungers. PROPOSAL / try-out, not in the pod build yet.

    python -B plunger_study.py cap        the M4 cap to print and try now
    python -B plunger_study.py m3         the M3 plunger proposal: sleeve, towers with the bigger bores and the smooth guide, assemblies
    python -B plunger_study.py all

What he saw: pushing the suspension by hand, each M4 plunger tilts in its guide (threads on PLA, Ø4.4 hole), and the bolt's head grinds the fork's
pad. Writes step_export/plunger/ (print STEPs in their print orientation, the assemblies in the pod frame) and validation/plunger/report.json.

1. **cap_m4** (to try now): a PETG cup under the M4 pan head (7.4 x 2.5, Fernando): the head drops into a Ø7.7 pocket 2.2 deep (the head stands
   0.3 proud so the washer and spring bear on the head, not the rim), and the cup's bottom is a smooth R8 dome; 1.6 mm of plastic at its apex.
   Ø9.4: 0.8 mm off the bump rails (±5.5 from the axis). Prints pocket down (the dome is the top skin). It raises the spring seat 1.6 mm.

2. **M3 plunger** (proposal; Fernando, second round: "the M3 needs 2 washers on top and the head + washer at the bottom where it contacts, or the
   printed part would break"): steel takes every load. From the pad up: the M3 x 45 pan head (it touches the pad), an M3 large washer (DIN 9021,
   9 x 0.8: the spring seat, wide enough for both springs' coils), the printed sleeve (Ø4.9, smooth, up through the springs and the tower's guide
   to TOP_GAP over the tower's top), two M3 washers (7 x 0.5) and the M3 nyloc. The sleeve is only in compression between the washers; it threads
   on the screw over its top THREAD_L (Ø2.9: a long thread, cad/solidworks.md 18), Ø3.2 snug below (the steel core along it), so the guide sees
   smooth plastic from droop to bump. When the pod is out the top washers land on the tower's top (the stop). Prints standing, either end down
   (the hole runs through). Towers: the spring bores Ø8.8 (Fernando's springs: 8.2 and 7.6 OD x 20 free, two in series) and the guide printed
   Ø5.0, opened with a 13/64" drill (5.16 mm: ~0.26 clearance on the Ø4.9 sleeve). The towers are where they are (no height change).

Springs drawn as coils (round wire on a helix, one closed turn at each end, the rest evenly pitched) at the length each one has in the pose: the
two in a bore share the stack's length (equal rates). Wire and coil count are ASSUMED (SPRINGS: VERIFY); solid length = coils x wire.
"""
import json
import math
import sys
from pathlib import Path

import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pod as P                                                        # noqa: E402
import pod_mid as PM                                                   # noqa: E402
import hardware as H                                                   # noqa: E402
from dims import PAD_Z                                                 # noqa: E402
from step_colour import write_part, write_step                         # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT, VAL = ROOT / 'step_export' / 'plunger', ROOT / 'validation' / 'plunger'

# ---- 1. the M4 cap ------------------------------------------------------------------------------------------------------------------------
DOME_R = 8.0
CAP_D = 9.4                        # 0.8 off the bump rails (they stand 5.5 from the plunger's axis)
CAP = dict(head_d=7.4, head_k=2.5, apex=1.2)       # M4 pan (domed) head 7.4 x 2.5 (Fernando); 1.2 of PETG under its apex, as the M3 cap
M4_WASHER = (9.0, 4.3, 0.8)        # the M4 plunger's washer on its head (DIN 125 M4 as dims.WASHER_OD / _T: VERIFY Fernando's): the cap wraps it

# ---- 2. the M3 plunger ----------------------------------------------------------------------------------------------------------------------
# Nov 18 (Fernando's picture of the yellow 20 mm spring): flat (die-spring) wire 1.0 mm thick along the axis with 1.5 mm gaps between the coils
# at free length (pitch 2.5), a closed turn at each end. Its radial width is not in the picture: 1.5 ASSUMED (the common "8 x 5 x 20" yellow
# spring; VERIFY: which drill shank slides through it gives the ID)
SPRINGS = [dict(name='8.2 OD', od=8.2, L0=20.0, t=1.0, w=1.5, gap=1.5),
           dict(name='7.6 OD', od=7.6, L0=20.0, t=1.0, w=1.5, gap=1.5)]
CLOSED = 1.0                       # closed turns at each end (pitch = the wire's thickness)
IN_BORES = (0, 0)                  # which spring each bore holds (the 7.6's ID, 4.6 with a 1.5 wire, does not clear the Ø4.9 sleeve)


def spring_numbers(sp):
    """active / total turns and solid length from the free length, the wire's thickness and the free gap"""
    na = (sp['L0'] - sp['t'] - 2 * CLOSED * sp['t']) / (sp['t'] + sp['gap'])
    return dict(active_turns=round(na, 2), total_turns=round(na + 2 * CLOSED, 2), solid_mm=round((na + 2 * CLOSED) * sp['t'] + sp['t'] * 0, 2),
                id=round(sp['od'] - 2 * sp['w'], 2), travel_to_solid_mm=round(na * sp['gap'], 2))
BORE_D = 8.8                       # the bigger spring's OD + 0.6
SLEEVE_D = 4.9                     # Nov 18 (Fernando: slightly under 5): inside the 7.6 spring's ID (5.6 with a 1.0 mm wire)
GUIDE_PRINT_D, GUIDE_DRILL = 5.0, ('13/64"', 5.159)
THREAD_L, PILOT_THREAD, PILOT_SNUG = 12.0, 2.9, 3.2
SCREW_L = 45.0                     # M3 x 45 pan head (6.5 x 2.5, hex drive)
HEAD_D, HEAD_K = 5.5, 1.8          # Fernando, Nov 18: his M3 button (dome) heads for this, 5.5 x 1.8, the dome down on the pad
HEAD_R = ((HEAD_D / 2) ** 2 + HEAD_K ** 2) / (2 * HEAD_K)   # the dome's sphere (R 3.0)
HEAD_HEX = 2.0                     # its hex socket, at the dome's apex


# Nov 18 (Fernando: "model the PETG cap for the M3 head"): the button head sits dome down in a PETG cup whose pocket is the head's own sphere
# (+0.05), so the load spreads over the dome, not the socket's rim; the head stands 0.2 proud of the rim (the washer bears on the head). The
# cup's bottom is the R8 dome of the M4 cap, CAP_M3['apex'] under the head's apex; it is what touches the pad. Prints pocket down.
CAP_M3 = dict(od=8.0, apex=1.2, rim=HEAD_K - 0.2, seat_clr=0.05)


def lift_of(a, y, R=DOME_R):
    """pod.plunger_lift for this plunger's dome (the cap's R DOME_R): (rise of the plunger at fork angle a, where it touches along the pad from
    the point that was under it at nominal, how far the contact is from the dome's apex)"""
    p0 = P.rot_pt((y, PAD_Z), a)
    n = P.rot_pt((y, PAD_Z + 1.0), a) - p0
    zc = p0[1] + (R - n[0] * (y - p0[0])) / n[1]
    contact = (y, zc) - R * n
    along = float((contact - p0) @ [n[1], -n[0]])
    return float(zc - (PAD_Z + R)), along, float(abs(contact[0] - y))


SEAT_WASHER = (6.85, 3.4, 0.6)     # Fernando, Nov 18: his M3 washers (OD, ID, t); one on the head (the spring seat: it fits the Ø8.8 bore, so it
TOP_WASHER = (6.85, 3.4, 0.6)      # may rise into it at bump), two under the nyloc
NYLOC_H = 4.0
TOP_GAP = 1.5                      # the lower top washer over the tower's top at nominal (as the M4 nyloc's gap)

Z_HEAD_APEX = PAD_Z + CAP_M3['apex']   # the head's dome apex, in the cap (nominal, pod z)
Z_HEAD_TOP = Z_HEAD_APEX + HEAD_K  # the head's bearing face
Z_SEAT = Z_HEAD_TOP + SEAT_WASHER[2]
Z_SLEEVE_TOP = P.Z_TT + TOP_GAP
Z_NUT = Z_SLEEVE_TOP + 2 * TOP_WASHER[2]
Z_TIP = Z_HEAD_TOP + SCREW_L

GREY, PETG, STEEL, SPRING_COL = (0.33, 0.33, 0.33), (0.85, 0.85, 0.85), H.MAGENTA, (0.80, 0.80, 0.82)


def dome_disc(d, apex_z, h_top):
    """a Ø d disc from its R DOME_R dome's apex at apex_z (the lowest point) up to h_top"""
    sph = b.Solid.make_sphere(DOME_R).moved(b.Location((0, 0, apex_z + DOME_R)))
    cyl = b.Solid.make_cylinder(d / 2, h_top - apex_z, b.Plane(origin=(0, 0, apex_z)))
    return cyl & sph


RELIEF, RELIEF_FROM = 0.2, 0.5     # the caps' pocket: a straight wall head_d / 2 + RELIEF from half the head's height up (the head's dome
                                   # bears on the conformal lower half, outside its socket)


def head_sphere(head_d, head_k):
    """the sphere of a domed (button / pan) head d x k"""
    return ((head_d / 2) ** 2 + head_k ** 2) / (2 * head_k)


WASHER_WALL = 0.75                 # Nov 18 (Fernando: the caps don't stay put): the cap's wall round the washer
WASHER_TOP_UNDER = 0.1             # the wall's top this far under the washer's top (the spring bears on the washer, not on the cap)
LEDGE_UNDER = 0.2                  # the cap's floor round the head this far under the head's top (the washer bears on the head, not the cap)


def dome_cap(head_d, head_k, od, apex, z0=0.0, seat_clr=0.05, washer=None, wall=WASHER_WALL):
    """a PETG cup for a domed screw head, dome down: its own R DOME_R dome's apex at z0 (on the pad), the head's apex `apex` above it in a
    pocket that is the head's sphere (+ seat_clr) round its lower half and a straight relief round its upper half.
    Nov 18 (Fernando: the caps work but won't stay put; the head in its spherical pocket is a ball joint, so the sliding pad rocked the cap off
    it): the wall rises past the head round the washer (washer = (od, id, t)): a Ø washer-OD pocket (it prints a little under: the washer
    presses in snug) from LEDGE_UNDER under the head's top to WASHER_TOP_UNDER under the washer's top. The washer, clamped on the head by the
    spring, keeps the cap from tilting off, and its snug fit holds the cap on with the pod out. od is then the washer's + 2 x wall"""
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


def cap_m4():
    """installed frame: the cap's dome apex at z 0 (on the pad), the pocket up. Nov 18 (Fernando: the M4 heads are domed too): the pocket is
    the head's sphere, as the M3 cap's (it was a flat Ø7.7 pocket, so a domed head bore only on its socket's rim)"""
    return dome_cap(CAP['head_d'], CAP['head_k'], CAP_D, CAP['apex'], washer=M4_WASHER, wall=0.6)   # Ø10.2: 0.4 off the rails


def cap_m3():
    """installed frame, pod z at nominal: the cup's dome apex on the pad (PAD_Z), the head's apex CAP_M3['apex'] above it in a spherical pocket"""
    c = CAP_M3
    return dome_cap(HEAD_D, HEAD_K, c['od'], c['apex'], PAD_Z, c['seat_clr'], washer=SEAT_WASHER)


def sleeve():
    """the printed sleeve, pod frame at nominal (axis x = y = 0): Z_SEAT .. Z_SLEEVE_TOP; Ø2.9 over its top THREAD_L, Ø3.2 below (through)"""
    L = Z_SLEEVE_TOP - Z_SEAT
    s = b.Solid.make_cylinder(SLEEVE_D / 2, L, b.Plane(origin=(0, 0, Z_SEAT)))
    try:
        s = b.chamfer(s.edges().filter_by(b.GeomType.CIRCLE), 0.3)     # lead-ins at both ends
    except Exception:                                                  # noqa: BLE001
        pass
    s = s - b.Solid.make_cylinder(PILOT_SNUG / 2, L - THREAD_L + 0.1, b.Plane(origin=(0, 0, Z_SEAT - 0.05)))
    s = s - b.Solid.make_cylinder(PILOT_THREAD / 2, THREAD_L + 0.2, b.Plane(origin=(0, 0, Z_SLEEVE_TOP - THREAD_L - 0.05)))
    return s


def ring(od, idd, t, z0):
    return b.Solid.make_cylinder(od / 2, t, b.Plane(origin=(0, 0, z0))) - b.Solid.make_cylinder(idd / 2, t + 0.2, b.Plane(origin=(0, 0, z0 - 0.1)))


def hardware(lift=0.0):
    """the steel of one plunger (pod frame, axis at 0, 0), lifted by `lift`: {name: solid}"""
    up = (0, 0, 1)
    z0 = Z_HEAD_APEX                                                   # the head's dome apex (in the cap)
    head = (b.Solid.make_cylinder(HEAD_D / 2, HEAD_K, b.Plane(origin=(0, 0, z0)))
            & b.Solid.make_sphere(HEAD_R).moved(b.Location((0, 0, z0 + HEAD_R))))
    head = head - b.extrude(b.Plane(origin=(0, 0, z0 - 0.1)) * b.RegularPolygon(HEAD_HEX / math.sqrt(3), 6), amount=1.2)
    out = dict(screw=head + b.Solid.make_cylinder(1.5, SCREW_L, b.Plane(origin=(0, 0, Z_HEAD_TOP))),
               washer_seat=ring(*SEAT_WASHER, Z_HEAD_TOP),
               washer_top_1=ring(*TOP_WASHER, Z_SLEEVE_TOP), washer_top_2=ring(*TOP_WASHER, Z_SLEEVE_TOP + TOP_WASHER[2]),
               nyloc=H.solid(dict(kind='nut', size=3.0, L=0.0, p=(0, 0, Z_NUT), d=up)))
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


EXTEND_TO_STOP = True              # Nov 18 (Fernando: "lengthen the tower from the bottom, following the hard stop's surface"): the overhang's
                                   # underside comes down to the bump rails' surface (the pad's top at bump) over its whole length, so the two
                                   # rails become one stop face and the bores run down to it: the springs are guided that much lower


TOWER_INFO = {}


def stop_line():
    """the bump stop's underside (pod y -> z): the pad's top at bump (pod_rx.rail_line, as the rails)"""
    return PM.R.rail_line(P.Y_S - 14.0, P.Y_S + 14.0)


def tower_m3(side):
    """today's tower (pod_mid.build_tower, unlabelled) with the bores opened to BORE_D and the guides to GUIDE_PRINT_D; with EXTEND_TO_STOP the
    overhang's underside filled down to the stop surface (from the pad's front edge at bump to the tower's back, between the base's face and the
    arm's outer face, as the rails), the bores through it"""
    info = {}
    t = PM.build_tower(side, info, stop=EXTEND_TO_STOP)                # stop: the underside drawn as the stop surface before the rounds
    TOWER_INFO[side] = info
    xc = PM.P4.ARM_C[0] if side == 'o' else PM.P4.ARM_C[1]
    z_bot = P.Z_TBOT
    if EXTEND_TO_STOP:
        z_bot = min(PM.tower_stop_z(y) for y in PM.TOWER_Y) - 0.5
    for y in P.Y_PAIR:
        t = t - b.Solid.make_cylinder(BORE_D / 2, P.Z_CEIL - z_bot + 0.2, b.Plane(origin=(xc, y, z_bot - 0.1)))
        t = t - b.Solid.make_cylinder(GUIDE_PRINT_D / 2, P.Z_TT - P.Z_CEIL + 1.0, b.Plane(origin=(xc, y, P.Z_CEIL - 0.5)))
    return t, xc


def report_cap():
    s = cap_m4()
    bb = s.bounding_box()
    od = bb.max.X - bb.min.X
    return dict(part='cap_m4', solids=len(s.solids()), valid=s.is_valid, volume_mm3=round(s.volume, 1), height=round(bb.max.Z - bb.min.Z, 2),
                od=round(od, 2), head=[CAP['head_d'], CAP['head_k']], washer=M4_WASHER,
                pocket=f"the head's sphere R{head_sphere(CAP['head_d'], CAP['head_k']) + 0.05:.2f} (lower half), the washer's Ø{M4_WASHER[0]} round it",
                dome_r=DOME_R, apex=CAP['apex'], seat_raise_mm=CAP['apex'],
                print='pocket down on the bed (the dome the top skin); PETG', rails_clearance_mm=round(5.5 - od / 2, 2))


def assembly(towers, sl, a, name):
    """the towers, plungers (sleeve + steel) and coil springs at fork angle a; returns (items, spring report)"""
    items, rep = {}, []
    for side, (t, xc) in towers.items():
        items[f'rx_tower_{side}'] = (t, GREY)
        for j, y in enumerate(P.Y_PAIR):
            lift = lift_of(a, y)[0]
            at = b.Location((xc, y, lift))
            items[f'sleeve_{side}{j}'] = (sl.moved(at), PETG)
            items[f'cap_{side}{j}'] = (cap_m3().moved(at), PETG)
            for k, s in hardware().items():
                items[f'{k}_{side}{j}'] = (s.moved(at), STEEL)
            sp = SPRINGS[IN_BORES[j]]                                  # the two in a bore share the stack (equal rates)
            stack = P.Z_CEIL - (Z_SEAT + lift)
            each = stack / 2
            for k in range(2):
                c, pitch, gap = coil(sp, Z_SEAT + lift + k * each, each)
                items[f'spring_{side}{j}_{k}'] = (c.moved(b.Location((xc, y, 0))), SPRING_COL)
            if side == 'o':
                solid_len = spring_numbers(sp)['solid_mm']
                rep.append(dict(bore=j, spring=sp['name'], lift_mm=round(lift, 2), stack_mm=round(stack, 2), each_mm=round(each, 2),
                                compressed_each_mm=round(sp['L0'] - each, 2), to_solid_each_mm=round(each - solid_len, 2),
                                active_pitch=round(pitch, 2), gap_between_coils=round(gap, 2)))
    write_step(items, OUT / f'plunger_m3_{name}.step', f'plunger_m3_{name}')
    return items, rep


def main(stage):
    OUT.mkdir(parents=True, exist_ok=True)
    VAL.mkdir(parents=True, exist_ok=True)
    rep = {}
    if stage in ('cap', 'all'):
        s = cap_m4()
        bb = s.bounding_box()
        write_part(s.mirror(b.Plane.XY).moved(b.Location((0, 0, bb.max.Z))), PETG, OUT / 'plunger_cap_m4_print.step', 'plunger_cap_m4')
        rep['cap_m4'] = report_cap()
        print(json.dumps(rep['cap_m4']), flush=True)
    if stage in ('m3', 'all'):
        for f in ('plunger_m3_print.step', 'plunger_m3_towers_assembly.step'):   # the first round's files (domed foot, screw from the top)
            (OUT / f).unlink(missing_ok=True)
        sl = sleeve()
        write_part(sl.moved(b.Location((0, 0, -Z_SEAT))), PETG, OUT / 'plunger_m3_sleeve_print.step', 'plunger_m3_sleeve')
        cp = cap_m3()
        zt = Z_HEAD_APEX + CAP_M3['rim']
        write_part(cp.mirror(b.Plane.XY).moved(b.Location((0, 0, zt))), PETG, OUT / 'plunger_cap_m3_print.step', 'plunger_cap_m3')   # pocket down
        towers = {}
        for side in ('o', 'i'):
            t, xc = tower_m3(side)
            write_part(t, GREY, OUT / f'rx_tower_{"outer" if side == "o" else "inner"}_m3.step', f'rx_tower_{side}_m3')
            towers[side] = (t, xc)
        poses = {}
        for name, a in (('droop', PM.A_DROOP), ('nominal', 0.0), ('bump', PM.A_BUMP)):
            items, sp = assembly(towers, sl, a, name)
            poses[name] = sp
            print(name, json.dumps(sp), flush=True)
        lift_max = max(lift_of(PM.A_BUMP, y)[0] for y in P.Y_PAIR)
        sp_rep = [dict(name=s['name'], od=s['od'], wire=f"{s['t']} thick (picture) x {s['w']} radial (ASSUMED)", free_gap=s['gap'],
                       **spring_numbers(s), sleeve_clearance_in_id=round(s['od'] - 2 * s['w'] - SLEEVE_D, 2),
                       bore_clearance=round(BORE_D - s['od'], 2), seat_washer_covers_wire=bool(SEAT_WASHER[0] >= s['od'] - 0.2),
                       drawn=[IN_BORES[j] == SPRINGS.index(s) for j in range(len(IN_BORES))])
                  for s in SPRINGS]
        rep['plunger_m3'] = dict(
            stack_from_the_pad=[f"PETG cap (Ø{CAP_M3['od']}, R{DOME_R:g} dome, {CAP_M3['apex']} under the head's apex): it touches the pad",
                                'M3 x 45 button head (5.5 x 1.8, R3 dome), dome down in the cap', 'M3 washer 6.85 x 0.6 (the spring seat)',
                                'printed sleeve Ø4.9', '2 x M3 washer 6.85 x 0.6', 'M3 nyloc'],
            cap=dict(solids=len(cp.solids()), valid=cp.is_valid, volume_mm3=round(cp.volume, 1), od=CAP_M3['od'], height=round(zt - PAD_Z, 2),
                     pocket=f"the head's sphere R{HEAD_R + CAP_M3['seat_clr']:.2f}, rim Ø{2 * math.sqrt((HEAD_R + CAP_M3['seat_clr']) ** 2 - (HEAD_R - CAP_M3['seat_clr'] - CAP_M3['rim']) ** 2):.2f}",
                     head_proud_mm=round(HEAD_K - CAP_M3['rim'], 2), to_rails_mm=round(5.5 - CAP_M3['od'] / 2, 2),
                     top_at_bump_to_tower_mm=round(P.Z_TBOT - (zt + lift_max), 2), print='pocket down on the bed (the dome the top skin); PETG'),
            seat_washer_on_spring=f'the washer covers the spring\'s wire from r {SEAT_WASHER[1] / 2:.2f} to {SEAT_WASHER[0] / 2:.2f}; the 8.2 spring\'s '
                                  f'wire runs r {(SPRINGS[0]["od"] - 2 * SPRINGS[0]["w"]) / 2:.2f}..{SPRINGS[0]["od"] / 2:.2f} (radial width ASSUMED)',
            z_pod=dict(head=[PAD_Z, round(Z_HEAD_TOP, 2)], seat=round(Z_SEAT, 2), sleeve_top=round(Z_SLEEVE_TOP, 2), nut=[round(Z_NUT, 2), round(Z_NUT + NYLOC_H, 2)],
                       screw_tip=round(Z_TIP, 2), tip_past_nut_mm=round(Z_TIP - Z_NUT - NYLOC_H, 2)),
            seat_vs_today_mm=round(Z_SEAT - P.Z_SB, 2),
            sleeve=dict(d=SLEEVE_D, length=round(Z_SLEEVE_TOP - Z_SEAT, 2), thread=f'Ø{PILOT_THREAD} x {THREAD_L} at its top', snug=f'Ø{PILOT_SNUG} below',
                        wall_at_thread_mm=round((SLEEVE_D - PILOT_THREAD) / 2, 2), loads='compression only, between the steel washers'),
            guide=dict(length=round(P.Z_TT - P.Z_CEIL, 2), printed_d=GUIDE_PRINT_D, drill=GUIDE_DRILL, clearance_mm=round(GUIDE_DRILL[1] - SLEEVE_D, 3),
                       tilt_deg=round(math.degrees(math.atan((GUIDE_DRILL[1] - SLEEVE_D) / (P.Z_TT - P.Z_CEIL))), 2),
                       sleeve_fills_guide='from droop (-1.15) to bump (+6.43): the sleeve runs from the seat to 1.5 over the tower'),
            seat_washer_to_tower_at_bump_mm=round(P.Z_TBOT - (Z_SEAT + lift_max), 2),
            seat_washer_to_rails_mm=round(5.5 - SEAT_WASHER[0] / 2, 2),
            dome_contact=dict(cap=f'R{DOME_R:g}: the contact stays within '
                                  f'{max(lift_of(a, y)[2] for a in (PM.A_DROOP, PM.A_BUMP) for y in P.Y_PAIR):.2f} mm of its apex',
                              bare_head=f'M3 button R{HEAD_R:.2f}: within {max(lift_of(a, y, HEAD_R)[2] for a in (PM.A_DROOP, PM.A_BUMP) for y in P.Y_PAIR):.2f} '
                                        f'mm of its apex, inside the {HEAD_HEX} hex socket\'s rim (r {HEAD_HEX / 2:.2f}): the pad would ride on the socket\'s edge'),
            nut_top_at_bump_robot_z=round(Z_NUT + NYLOC_H + lift_max + PM.R_WHEEL, 1),
            screw_tip_at_bump_robot_z=round(Z_TIP + lift_max + PM.R_WHEEL, 1),
            springs=sp_rep, bore_d=BORE_D, wall_between_bores_mm=round(P.SPRING_PITCH - BORE_D, 2) if hasattr(P, 'SPRING_PITCH') else None,
            poses=poses)
        print(json.dumps({k: v for k, v in rep['plunger_m3'].items() if k != 'poses'}, indent=1), flush=True)
    old = json.loads((VAL / 'report.json').read_text()) if (VAL / 'report.json').exists() else {}
    old.update(rep)
    (VAL / 'report.json').write_text(json.dumps(old, indent=1), encoding='utf-8')


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'all')
