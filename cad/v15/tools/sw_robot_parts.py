"""Part STEPs + manifest for the SolidWorks viewing assembly of the whole v15 robot: the MID DECK (chassis_mid.py: four deck pieces, bumper,
lidar pedestal, the generated fins (fins.py), the battery under the deck and its clips), four mid-deck pods (pod_mid.py), upper.py's second floor, arm, electronics, bin deck and face,
the D435 on the arm riser. omni-robot-sim interpreter; sw_robot.py builds the assemblies from them. Every input is read from cad/v15
(cad/solidworks.md section 17); nothing comes from another version's folder.

    python -B sw_robot_parts.py

Writes cad/v15/solidworks/step_parts/<part>.step (one coloured solid each, purchased parts may be several) and solidworks/manifest.json.
A part whose STEP comes out the same keeps its file and mtime, so `sw_robot.py import` re-imports only the parts that changed.
Base and upper parts are written in the robot frame (inserted at the origin), pod parts in the pod frame, once per hand: the 45 and 225 deg
pods (FL, RR) are the mirror image (suffix _m; the mirrored motor is the whole motor mirrored, not a real opposite-hand plug position).
The roller's parts are written in their own frame, one pose per roller.
Pod structure (v15): STATIC = hangers, beam (its droop lug), the two towers on the deck and the four springs (nominal length);
UNSPRUNG = fork halves, the two 627s (their outer races in the arms), rings, 6804 retainer, the wheel and the motor; the four
plungers are separate (they slide in the towers' guides, their domes on the arms' pads). The fork hinges on the static parts' Ø7.2 spigots in
the 627s' bores.
Fasteners: every screw and nut is a record (hardware.py), placed as an instance of one magenta library part per type and length (fst_*):
the pod's in its static / unsprung / wheel sub-assemblies, the rest in the top-level fasteners group.
The wiring harness (Nov 14): harness.py's leads and plugs, group 'harness' (routes: harness_route.py on the mid deck).
"""
import json
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import chassis_mid as CM                                               # noqa: E402
import fins as F                                                       # noqa: E402
import hardware as H                                                   # noqa: E402
import palette as PAL                                                  # noqa: E402
import pod as P                                                        # noqa: E402
import pod_mid as PM                                                   # noqa: E402
import pod_p4 as P4                                                    # noqa: E402
import pod_rx as R                                                     # noqa: E402
import print_beds as PB                                                # noqa: E402
import upper as U                                                      # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part                                     # noqa: E402

RSW = W.ROOT / 'solidworks'
STEPS = RSW / 'step_parts'
S_POD = np.diag([1.0, -1.0, 1.0, 1.0])                                 # the mirror hand: pod y -> -y (base.place3d)
POD_STATIC = ('rx_hanger_outer', 'rx_hanger_inner', 'rx_beam', 'rx_pin', 'rx_lug', 'rx_tower_outer', 'rx_tower_inner')   # Nov 15: + the beam's pin, the lug
POD_FORK = ('pod_fork_outer', 'pod_fork_inner', 'bearing_627_outer', 'bearing_627_inner', 'rx_ring_outer', 'rx_ring_inner', 'pod_retainer')   # Nov 14: no leads' tie
POD_WHEEL = ('wheel_hub', 'wheel_cartridge', 'wheel_cartridge_cap')   # Nov 15: the cartridge is a cup + cap (the 6804 held both ways)
# Nov 15 (Fernando: numbers and IN / OUT on the big parts): these carry their pod's marks, so they are written once per pod (<part>_<pod>, already
# in that pod's hand); the rest once per hand as before
POD_LABELLED = PM.LABELLED
POD_MOTOR = ('ak40_stator', 'ak40_rotor', 'xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg')
PLUNGERS = {f'{side}{j + 1}': (xc, y) for side, xc in (('o', P4.ARM_C[0]), ('i', P4.ARM_C[1])) for j, y in enumerate(P.Y_PAIR)}
LIMIT_KEY = 'i1'                                                       # the travel limit rides on this plunger (motor side, far from the pivot)
# Nov 18 (Fernando: "pick up the latest towers that have M3 + sleeves + plungers"): the SolidWorks assembly shows plunger_study.py's M3 set,
# which the pod build (pod.py / pod_mid.py, so robot_assembly.step and the pod checks) does not draw yet: the towers with the stop-surface
# underside and the Ø8.8 bores / Ø5.0 guides (plunger_study.tower_m3, marked per pod as before), each plunger one moving part (its PETG cap,
# the M3 x 45 button head, the seat washer, the printed Ø4.9 sleeve, two washers and the nyloc; it slides on the sleeve in the guide), two
# 8.2 OD coil springs per bore at nominal (plunger_study.coil). 'm4' gives the pod build's set
PLUNGER_SET = 'm3'
# Nov 19 (Fernando: "bright colors to the M3 sleeve and plunger cap"): the plunger stays one part (one sphere for the pad mate), its faces
# painted by the piece they belong to: the PETG cap yellow, the printed sleeve cyan, the steel magenta (springs stay green, towers grey)
PLUNGER_CAP_RGB, PLUNGER_SLEEVE_RGB = (1.0, 0.85, 0.0), (0.0, 0.85, 1.0)
PAINT = {}                                                             # stem -> [(solid, rgb)]: a face takes the colour of the piece it bounds


def painter(body, pieces):
    """face -> rgb: a point on the face nudged 0.05 into the body, coloured as the piece that holds it (the first at 0 distance)"""
    def rgb(face):
        f = b.Face(face)
        p = f.position_at(0.5, 0.5)
        n = f.normal_at(p)
        q = min((p + n * 0.05, p - n * 0.05), key=lambda v: body.distance_to(v))
        return min(pieces, key=lambda sc: sc[0].distance_to(q))[1]
    return rgb
# vendor part frame -> robot frame: part Z -> robot x, X -> y, Y -> z (the RPLIDAR is Y-up; its Laser_link has its x along part +Z)
PERM = np.array([[0, 0, 1.0, 0], [1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 0, 1.0]])
LIDAR_FOOT_Y = -9.3                                                    # vendor RPLIDAR: base Y -9.3, Laser_link (scan plane) Y 22


def mirror(s):
    return s.mirror(b.Plane.XZ)


def PM_number(name):
    import labels as LB
    return LB.NUMBER[name]


def mirror_rec(r):
    return dict(r, p=(r['p'][0], -r['p'][1], r['p'][2]), d=(r['d'][0], -r['d'][1], r['d'][2]))


def pod_parts():
    """pod-frame parts at nominal (one solid each), the plungers and springs, the roller's parts (core, sleeve, tread, 687) in their own frame;
    the pod's fastener records (static, unsprung)"""
    parts, rm, rs, unit, _ = CM.pod_build()
    if PLUNGER_SET == 'm3':
        import plunger_study as PS
        parts = dict(parts, rx_tower_outer=PS.tower_m3('o')[0], rx_tower_inner=PS.tower_m3('i')[0])
        PM._LAB.clear()                                                # the marks go on these towers (labelled() caches per pod
        PM._REG.clear()                                                # and the marks' regions per part)
    sh, moving, _ = PM.assembly(parts, rm, rs, unit, 0.0, deck=False)
    once = {k: sh[k] for k in POD_STATIC + POD_FORK + POD_WHEEL + POD_MOTOR}
    # one-body 6804 ring in the cartridge (the reference STEP has separate races and balls: SolidWorks opens it as an assembly)
    once['bearing_6804_left'] = W.along_x(R_BRG, X_PLATE_FLOOR, X_PLATE_FLOOR + 7.0) - W.along_x(10.0, X_PLATE_FLOOR - 1, X_PLATE_FLOOR + 8.0)
    for k, (xs, y) in PLUNGERS.items():
        if PLUNGER_SET == 'm3':
            at = b.Location((xs, y, 0))
            fill = b.Solid.make_cylinder(3.6, PS.Z_SEAT - PAD_Z - 1.0, b.Plane(origin=(0, 0, PAD_Z + 1.0)))   # the head's pocket filled:
                                                                       # the cap's R8 dome stays the body's only sphere (the pad mate)
            paint = [(s, H.MAGENTA) for s in PS.hardware().values()] + [(PS.cap_m3(), PLUNGER_CAP_RGB), (PS.sleeve(), PLUNGER_SLEEVE_RGB),
                                                                        (fill, PLUNGER_CAP_RGB)]
            body = paint[0][0]
            for s, _ in paint[1:]:
                body = body + s
            once[f'spring_plunger_{k}'] = body.moved(at)
            PAINT[f'spring_plunger_{k}'] = [(s.moved(at), c) for s, c in paint]
            sp = PS.SPRINGS[PS.IN_BORES[0]]
            each = (P.Z_CEIL - PS.Z_SEAT) / 2
            once[f'die_spring_{k}'] = b.Compound([PS.coil(sp, PS.Z_SEAT + j * each, each)[0] for j in range(2)]).moved(at)
        else:
            once[f'spring_plunger_{k}'] = P.plunger(xs, y)
            once[f'die_spring_{k}'] = P.die_spring(xs, y, P.Z_CEIL - P.Z_SB)
    per_pod = {}
    for i, name in enumerate(B.NAMES):                                 # the marked parts, each pod's own (pod_mid.labelled), in its hand
        lab = PM.labelled(parts, i)
        for k in POD_LABELLED:
            once.pop(k, None)
            per_pod[f'{k}_{name}'] = mirror(lab[k]) if B.mirrored(i) else lab[k]
    wm, _ = W.build_all()
    return once, per_pod, {k: wm['_' + k] for k in (*W.ROLLER_PARTS, 'bearing_687')}, rm, rs


def wheel_fasteners():
    """pod-frame records of the wheel: hub-to-rotor M2.5s, the roller axles, the cartridge's flat heads"""
    wheel = []
    for a in FLANGE_ANGLES:                                            # hub flange to the rotor's M2.5 holes (2 deep in our model: VERIFY)
        y, z = W.at_angle(FLANGE_R, a)
        wheel.append(H.pan((FLANGE_FRONT, y, z), (1, 0, 0), 5.0, 'bolt_hub_rotor', size=2.5))
    for row, k, M in W.roller_frames():                                # M3 x 50 roller axles (wheel_motor.build_all's placement)
        p = (M @ np.array([0, 0, SEAT, 1.0]))[:3]
        wheel.append(H.pan(tuple(p), tuple(-M[:3, 2]), 50.0, f'bolt_axle_{row}_{k}'))
    for a in CART_ANGLES:                                              # cartridge: M3 x 12 flat heads from the left face
        y, z = W.at_angle(CART_R, a)
        wheel.append(dict(kind='flat', size=3.0, L=12.0, p=(X_LEFT + CSK_EXTRA, y, z), d=(1.0, 0, 0), name='bolt_cartridge'))
    return wheel


def vendor_poses():
    """robot-frame pose of the vendor lidar: its base on the pedestal top at fins.LIDAR_X (the pedestal stands on the mid deck; the lidar
    stays at v14's height)"""
    lid = PERM.copy()
    lid[:3, 3] = (F.LIDAR_X, 0.0, F.LIDAR_BASE_Z - LIDAR_FOOT_Y)
    return dict(lidar=lid.tolist())


def bed_of(stem, s):
    """(axis 0..2, sign, offset along the axis in the part's frame, supports, why) or None"""
    b_ = PB.bed(stem)
    if b_ is None:
        return None
    up, sup, why = b_
    ax = 'xyz'.index(up[1])
    bb = s.bounding_box()
    lo, hi = (bb.min.X, bb.min.Y, bb.min.Z)[ax], (bb.max.X, bb.max.Y, bb.max.Z)[ax]
    return [ax, 1 if up[0] == '+' else -1, round(lo if up[0] == '+' else hi, 3), sup, why]


WRITTEN = {}                                                           # stem -> True if the STEP changed this run


def write_step_part(s, rgb, stem):
    WRITTEN[stem] = write_part(s, rgb, STEPS / f'{stem}.step', stem, keep_same=True)


def main():
    STEPS.mkdir(parents=True, exist_ok=True)
    beds = {}

    def out(stem, s, rgb):
        write_step_part(s, rgb, stem)
        bd = bed_of(stem, s)
        if bd:
            beds[stem] = bd

    once, per_pod, local, rm, rs = pod_parts()
    for k, s in once.items():
        rgb = H.MAGENTA if k.startswith(('washer', 'spring_plunger', 'bearing_')) else R.colour(k)
        if k in PAINT:
            out(k, s, painter(s, PAINT[k]))
            out(k + '_m', mirror(s), painter(mirror(s), [(mirror(p), c) for p, c in PAINT[k]]))
            continue
        out(k, s, rgb)
        out(k + '_m', mirror(s), rgb)
    for k, s in per_pod.items():
        out(k, s, R.colour(k))
    for k, s in local.items():                                         # mirrored about the roller's own x-z plane: placed by S M S
        rgb = H.MAGENTA if k.startswith('bearing_') else R.colour(k)
        out(k, s, rgb)
        out(k + '_m', mirror(s), rgb)
    cparts, chw, cg = CM.built()
    cparts = dict(cparts, **cg['bought'])                              # + the battery and its straps (bought, under the deck)
    for k, s in cparts.items():
        out(k, s, CM.colour(k))
    import d435_riser as D                                             # the riser camera's fields of view (500 mm frustums, see-through)
    env = D.fov_envelopes()
    for k, s in env.items():
        out(k, s, D.colour(k))
    printed, bought, vendor, uhw, uinfo = U.build()                    # the upper robot (v14's, unchanged in place)
    for k, s in printed.items():
        out(k, s, U.colour(k))
    for k, s in bought.items():
        out(k, s, U.colour(k))
    vend_inst, done = {}, {}
    for k, (path, T) in vendor.items():
        stem = ('arm_' + Path(path).stem.replace('SO-101_', '')) if k.startswith('arm_') else k
        if stem not in done:
            s = b.import_step(str(path))
            rgb = U.COL['servo'] if 'STS3215' in stem else (PAL.colour(stem) or U.colour(k))
            write_step_part(s, rgb, stem)
            done[stem] = 1
            if stem.startswith('arm_') and 'STS3215' not in stem:
                beds[stem] = [None, None, None, 'per the SO-101 repo', "print as TheRobotStudio's print-ready STL files are oriented (no plane added)"]
            elif PB.bed(stem):
                beds[stem] = bed_of(stem, s.moved(U.mat_loc(np.eye(4))))
        vend_inst[k] = (stem, np.asarray(T, float).tolist())
    # fasteners: library parts and instances
    lib = {}
    fst = dict(robot=[], static=[], static_m=[], unsprung=[], unsprung_m=[], wheel=[], wheel_m=[])
    for k, s in chw.items():
        r = H.from_solid(s, k)
        if r is None:
            print('  not a fastener record:', k)
            continue
        fst['robot'].append(r)
    fst['robot'] += uhw
    for grp, recs in (('static', rs), ('unsprung', rm), ('wheel', wheel_fasteners())):
        fst[grp] += recs
        fst[grp + '_m'] += [mirror_rec(r) for r in recs]
    inst = {}
    for grp, recs in fst.items():
        inst[grp] = []
        for i, r in enumerate(recs):
            nm = H.part_name(r)
            if nm not in lib:
                lib[nm] = r
                write_step_part(H.part_solid(r), H.MAGENTA, f'fst_{nm}')
            inst[grp].append([f"{r['name']}_{i}", f'fst_{nm}', H.pose(r).tolist()])
    count = {}
    for grp, recs in fst.items():
        if grp.endswith('_m'):
            continue
        mult = 4 if grp != 'robot' else 1                               # per pod x 4 pods (both hands)
        for r in recs:
            count[H.part_name(r)] = count.get(H.part_name(r), 0) + mult
    rollers = {f'{row}_{k}': M for row, k, M in W.roller_frames()}
    pods = {}
    for i, name in enumerate(B.NAMES):
        A, xh, yh, m = B.frame(i)
        T = np.eye(4)
        T[:2, 0], T[:2, 1], T[:2, 3], T[2, 3] = xh, yh, A, R_WHEEL
        pods[name] = dict(mirrored=bool(m), pose=T.tolist(), number=PM_number(name))
    if PLUNGER_SET == 'm3':
        import plunger_study as PS
        lift_fn, shank_r, guide_r = PS.lift_of, PS.SLEEVE_D / 2, PS.GUIDE_PRINT_D / 2   # the sleeve in the printed guide
    else:
        lift_fn, shank_r, guide_r = P.plunger_lift, SHANK_R, GUIDE_R
    lift = {pose: {pk: float(lift_fn(a, y)[0]) for pk, (_, y) in PLUNGERS.items()} for pose, a in R.POSES.items()}
    eye = np.eye(4).tolist()
    groups = dict(frame=sorted(k for k in cparts if k.startswith('deck_')),
                  mounts=sorted(k for k in cparts if k.startswith(('pedestal', 'support'))),
                  bumper=sorted(k for k in cparts if k.startswith('bumper')),
                  battery=sorted(k for k in cparts if k.startswith(('battery', 'strap_battery', 'bracket_battery'))),
                  reference=sorted(env))
    left = set(cparts) | set(env)
    left -= {k for g in groups.values() for k in g}
    if left:
        raise RuntimeError(f'base parts in no group: {sorted(left)}')
    groups = {g: [[k, k, eye] for k in ks] for g, ks in groups.items()}
    groups['upper'] = [[k, k, eye] for k in sorted(printed)]
    groups['electronics'] = [[k, k, eye] for k in sorted(bought)] + [[k, st, T] for k, (st, T) in vend_inst.items() if not k.startswith('arm_')]
    groups['arm'] = [[k, st, T] for k, (st, T) in vend_inst.items() if k.startswith('arm_')]
    groups['fasteners'] = inst['robot']
    # Nov 14 (v15, from v14): the wiring harness (harness.py: the leads along harness_route.py's routes, the plugs upper.py does not draw, the
    # XT90 pair, the lidar's adapter), purchased items for review; part names carry the prefix harness_
    import harness as HN
    hitems, _, _ = HN.build()
    for k, s in hitems.items():
        out(f'harness_{k}', s, HN.colour(k))
    groups['harness'] = [[f'harness_{k}', f'harness_{k}', eye] for k in sorted(hitems)]
    # each lead's own end plugs (it runs into the plug's back, as its build123d check allows): sw_robot.py's interference lists those overlaps
    # as the lead's termination, not findings
    harness_own = {}
    for cn in HN.CONNECTIONS:
        ends = [q for q in (cn.get('a'), cn.get('b')) if isinstance(q, str)]
        harness_own[f'harness_lead_{cn["id"]}'] = sorted({f'harness_plug_{q}' for q in ends} | {HN.DRAWN_PLUG[q] for q in ends if q in HN.DRAWN_PLUG})
    harness_own['harness_lead_pb_xt90'] = ['harness_plug_xt90_pair', 'harness_lead_BAT_PB']   # the board's XT90 lead: the pair under the
                                                                       # deck to the hole, then routed (BAT_PB) to the board
    man = dict(
        harness_own=harness_own,
        groups=groups, pods=pods, mirror=S_POD.tolist(), vendor=vendor_poses(), print_bed=beds,
        pod_fasteners={g: v for g, v in inst.items() if g != 'robot'}, fastener_count=dict(sorted(count.items())),
        rollers={k: M.tolist() for k, M in rollers.items()},
        roller_bearings={tag: T.tolist() for tag, T in W.bearing_frames()},   # the two 687s in the roller frame
        rollers_m={k: (S_POD @ M @ S_POD).tolist() for k, M in rollers.items()},
        pivot=[PIVOT_Y, PIVOT_Z], b627_bore_r=B627_BORE / 2, spigot_r=R.SPIGOT_D / 2, poses_deg=R.POSES,
        pod=dict(static=list(POD_STATIC), fork=list(POD_FORK), wheel=list(POD_WHEEL) + ['bearing_6804_left'], motor=list(POD_MOTOR),
                 axial_pair=['pod_fork_inner', 'rx_hanger_inner'], pivot_static='rx_hanger_outer', labelled=list(POD_LABELLED)),
        plungers=dict(axes={k: list(v) for k, v in PLUNGERS.items()}, shank_r=shank_r, guide_r=guide_r, pad_z=PAD_Z, set=PLUNGER_SET,
                      guide_part={k: ('rx_tower_outer' if k.startswith('o') else 'rx_tower_inner') for k in PLUNGERS},
                      pad_part={k: ('pod_fork_outer' if k.startswith('o') else 'pod_fork_inner') for k in PLUNGERS},
                      column_top_z=P.Z_TT, limit_key=LIMIT_KEY, limit_part='rx_tower_inner', lift=lift),
    )
    (RSW / 'manifest.json').write_text(json.dumps(man, indent=1), encoding='utf-8')
    for f in STEPS.glob('*.step'):                                     # parts that no longer exist
        if f.stem not in WRITTEN:
            try:
                f.unlink()
            except PermissionError:
                print('  could not remove (locked):', f.name)
    changed = sorted(k for k, v in WRITTEN.items() if v)
    print(len(WRITTEN), 'part STEPs and the manifest in', RSW, f'({len(changed)} changed)')
    print('fasteners per robot:', json.dumps(man['fastener_count']))


if __name__ == '__main__':
    main()
