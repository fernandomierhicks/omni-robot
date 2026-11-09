"""Part STEPs + manifest for the SolidWorks viewing assembly of the whole v13 robot (layout F: chassis.py + four lean pods, pod_lean.py,
upper.py's second floor, arm, electronics, bin deck and face, the D435 on the arm riser). omni-robot-sim interpreter; sw_robot.py builds the
assemblies from them. Every input is read from cad/v13 (cad/solidworks.md section 17); nothing comes from another version's folder.

    python -B sw_robot_parts.py

Writes cad/v13/solidworks/step_parts/<part>.step (one coloured solid each, purchased parts may be several) and solidworks/manifest.json.
Nov 4: a part whose STEP comes out the same (all but the header's time stamp) keeps its file and mtime, so `sw_robot.py import` (which
re-imports a part whose SLDPRT is older than its STEP) re-imports only the parts that changed; the run prints which those are.
Chassis and upper parts are written in the robot frame (inserted at the origin), pod parts in the pod frame, once per hand: the 45 and
225 deg pods (FL, RR) are the mirror image, as in chassis.py's STEP (mirrored copies carry the suffix _m; the mirrored motor is the whole
motor mirrored, not a real opposite-hand plug position). Roller core / tread, the SO-101's parts and other vendor models are written in
their own frame, one pose per instance.
Fasteners (Nov 4): every screw, dowel and nut is a record (hardware.py), placed as an instance of one magenta library part per type and
length (fst_*.step): the pod's in its static / unsprung / wheel sub-assemblies, everything else in the top-level fasteners group. The
spring plungers stay bodies (the suspension mates need them); the pivot dowel and washer are magenta too.
Print beds (Nov 4): manifest['print_bed'][part] = (axis, sign, offset, supports, why): sw_robot.py adds the PRINT_BED plane there.
The lidar is the vendor SolidWorks part (sw_robot.VENDOR, reference/parts/RPLIDAR.SLDPRT), not a STEP: the manifest gives its pose in the robot
frame. The D435 (d435_source.step in the riser's pocket), its plug + lead and the arm's parts are STEP parts like the rest; the D435's depth / RGB
fields of view are the see-through envelopes of d435_riser.fov_envelopes() in the reference group.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import chassis as C                                                    # noqa: E402
import first_floor as FF                                               # noqa: E402
import hardware as H                                                   # noqa: E402
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import print_beds as PB                                                # noqa: E402
import upper as U                                                      # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part                                     # noqa: E402

RSW = W.ROOT / 'solidworks'
STEPS = RSW / 'step_parts'
S_POD = np.diag([1.0, -1.0, 1.0, 1.0])                                 # the mirror hand: pod y -> -y (base.place3d)
POD_KEEP = ('wheel_hub', 'wheel_cartridge', 'bearing_6804_left', 'ak40_stator', 'ak40_rotor', 'xt30_housing', 'xt30_pin_1', 'xt30_pin_2',
            'lead_pos', 'lead_neg', 'pod_fork_outer', 'pod_fork_inner', 'bearing_627_outer', 'bearing_627_inner', 'pod_frame_inner',
            'pod_keel_outer', 'pod_sleeve', 'dowel_inner', 'washer_pivot_outer')
ENV_VENDOR = ('env_lidar', 'env_arm_riser')                            # replaced by the vendor lidar; round 13: the riser envelope too
                                                                       # (the real riser and adapter are in the upper group: they overlapped)
# vendor part frame -> robot frame: part Z -> robot x, X -> y, Y -> z (the RPLIDAR is Y-up; its Laser_link has its x along part +Z)
PERM = np.array([[0, 0, 1.0, 0], [1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 0, 1.0]])
LIDAR_FOOT_Y = -9.3                                                    # vendor RPLIDAR: base Y -9.3, Laser_link (scan plane) Y 22


def mirror(s):
    return s.mirror(b.Plane.XZ)


def mirror_rec(r):
    return dict(r, p=(r['p'][0], -r['p'][1], r['p'][2]), d=(r['d'][0], -r['d'][1], r['d'][2]))


def pod_parts():
    """pod-frame parts at nominal (one solid each), the roller core / tread in their own frame"""
    parts, _ = L.build_static()
    unit = P.wheel_unit()
    sh, _ = L.posed(parts, unit, 0.0)
    once = {k: sh[k] for k in POD_KEEP}
    # one-body 6804 ring in the cartridge (the reference STEP has separate races and balls: SolidWorks opens it as an assembly)
    once['bearing_6804_left'] = W.along_x(R_BRG, X_PLATE_FLOOR, X_PLATE_FLOOR + 7.0) - W.along_x(10.0, X_PLATE_FLOOR - 1, X_PLATE_FLOOR + 8.0)
    for k, (xs, y) in L.PLUNGERS.items():
        once[f'spring_plunger_{k}'] = P.plunger(xs, y)
        once[f'die_spring_{k}'] = P.die_spring(xs, y, P.Z_CEIL - P.Z_SB)
    wm, _ = W.build_all()
    return once, dict(roller_core=wm['_roller_core'], roller_tread=wm['_roller_tread'])


def pod_fasteners():
    """pod-frame records: static (outer pivot screw, the six hex nuts for the chassis screws), unsprung (tie bolts, motor screws, hub-to-rotor M2.5), wheel (roller axles,
    cartridge)"""
    static = [H.from_solid(L.outer_screw(), 'bolt_pivot_outer')] + L.nuts()                # round 13: the column bolt comes up from under
                                                                                         # the chassis (chassis.py); the pod keeps its hex nuts
    unsprung = [H.from_solid(s, k) for k, s in L.tie_bolts().items()]
    head_x = L.MOTOR_PLATE_X[0] + L.MOTOR_SCREW_L - REAR_ENGAGE
    for a in REAR_HOLES_AZ:                                            # fork inner half to the motor's rear face
        y, z = W.at_angle(REAR_HOLES_R, a)
        unsprung.append(H.pan((head_x, y, z), (-1, 0, 0), L.MOTOR_SCREW_L, 'bolt_motor_rear'))
    wheel = []
    for a in FLANGE_ANGLES:                                            # hub flange to the rotor's M2.5 holes (2 deep in our model: VERIFY)
        y, z = W.at_angle(FLANGE_R, a)
        wheel.append(H.pan((FLANGE_FRONT, y, z), (1, 0, 0), 5.0, 'bolt_hub_rotor', size=2.5))
    for row, k, M in W.roller_frames():                                # M3 x 50 roller axles (wheel_motor.build_all's placement)
        p = (M @ np.array([0, 0, SEAT, 1.0]))[:3]
        wheel.append(H.pan(tuple(p), tuple(-M[:3, 2]), 50.0, f'bolt_axle_{row}_{k}'))
    for a in CART_ANGLES:                                              # cartridge: M3 x 12 flat heads from the left face
        y, z = W.at_angle(CART_R, a)
        wheel.append(dict(kind='flat', size=3.0, L=12.0, p=(X_LEFT, y, z), d=(1.0, 0, 0), name='bolt_cartridge'))
    return dict(static=static, unsprung=unsprung, wheel=wheel)


def vendor_poses():
    """robot-frame pose of the vendor lidar: its base on the pedestal top at chassis.LIDAR_X (v13: the D435 is a STEP part placed by
    d435_riser.build_mount, so it needs no pose here)"""
    lid = PERM.copy()
    lid[:3, 3] = (C.LIDAR_X, 0.0, FF.LIDAR_BASE_Z - LIDAR_FOOT_Y)
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
    """Nov 4: an unchanged part keeps its STEP file and mtime, so `sw_robot.py import` re-imports only the parts that changed"""
    WRITTEN[stem] = write_part(s, rgb, STEPS / f'{stem}.step', stem, keep_same=True)


def main():
    STEPS.mkdir(parents=True, exist_ok=True)
    beds = {}

    def out(stem, s, rgb):
        write_step_part(s, rgb, stem)
        bd = bed_of(stem, s)
        if bd:
            beds[stem] = bd

    once, local = pod_parts()
    for k, s in once.items():
        rgb = H.MAGENTA if k.startswith(('dowel', 'washer', 'spring_plunger')) else L.colour(k)
        out(k, s, rgb)
        out(k + '_m', mirror(s), rgb)
    for k, s in local.items():                                         # mirrored about the roller's own x-z plane: placed by S M S
        out(k, s, L.colour(k))
        out(k + '_m', mirror(s), L.colour(k))
    cparts, chw, env, _ = C.build(FF.pod_geometry())
    for k, s in cparts.items():
        out(k, s, C.colour(k))
    env = {k: s for k, s in env.items() if k not in ENV_VENDOR}
    import d435_riser as D                                             # v13: the riser camera's fields of view (500 mm frustums, see-through)
    env.update(D.fov_envelopes())
    for k, s in env.items():
        out(k, s, C.colour(k))                                         # round 15: the D435's FOV frustums blue / green
    # the upper robot
    printed, bought, vendor, uhw, uinfo = U.build()
    for k, s in printed.items():
        out(k, s, U.colour(k))
    for k, s in bought.items():
        out(k, s, U.colour(k))
    vend_inst, done = {}, {}
    for k, (path, T) in vendor.items():
        stem = ('arm_' + Path(path).stem.replace('SO-101_', '')) if k.startswith('arm_') else k
        if stem not in done:
            s = b.import_step(str(path))
            rgb = U.COL['servo'] if 'STS3215' in stem else (U.COL['arm'] if stem.startswith('arm_') else U.colour(k))
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
    for n, (k, s) in enumerate(chw.items()):
        r = H.from_solid(s, k)
        if r is None:
            print('  not a fastener record:', k)
            continue
        fst['robot'].append(r)
    fst['robot'] += uhw
    for grp, recs in pod_fasteners().items():
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
        pods[name] = dict(mirrored=bool(m), pose=T.tolist())
    lift = {pose: {pk: P.plunger_lift(a, y)[0] for pk, (_, y) in L.PLUNGERS.items()} for pose, a in L.POSES.items()}
    eye = np.eye(4).tolist()
    groups = dict(frame=sorted(k for k in cparts if k.startswith(('hub', 'edge', 'node'))),
                  mounts=sorted(k for k in cparts if k.startswith(('pedestal', 'bracket', 'support'))),
                  bumper=sorted(k for k in cparts if k.startswith('bumper')),
                  reference=sorted(env))
    left = set(cparts) | set(env)
    left -= {k for g in groups.values() for k in g}
    if left:
        raise RuntimeError(f'chassis parts in no group: {sorted(left)}')
    groups = {g: [[k, k, eye] for k in ks] for g, ks in groups.items()}
    groups['upper'] = [[k, k, eye] for k in sorted(printed)]
    groups['electronics'] = [[k, k, eye] for k in sorted(bought)] + [[k, st, T] for k, (st, T) in vend_inst.items() if not k.startswith('arm_')]
    groups['arm'] = [[k, st, T] for k, (st, T) in vend_inst.items() if k.startswith('arm_')]
    groups['fasteners'] = inst['robot']
    man = dict(
        groups=groups, pods=pods, mirror=S_POD.tolist(), vendor=vendor_poses(), print_bed=beds,
        pod_fasteners={g: v for g, v in inst.items() if g != 'robot'}, fastener_count=dict(sorted(count.items())),
        rollers={k: M.tolist() for k, M in rollers.items()},
        rollers_m={k: (S_POD @ M @ S_POD).tolist() for k, M in rollers.items()},
        pivot=[PIVOT_Y, PIVOT_Z], b627_bore_r=B627_BORE / 2, dowel_r=L.DOWEL_D / 2, poses_deg=L.POSES,
        plungers=dict(axes={k: list(v) for k, v in L.PLUNGERS.items()}, shank_r=SHANK_R, guide_r=GUIDE_R, pad_z=PAD_Z,
                      column_top_z=P.Z_TT, limit_key='inner_a1', lift=lift),
    )
    (RSW / 'manifest.json').write_text(json.dumps(man, indent=1), encoding='utf-8')
    for f in STEPS.glob('*.step'):                                     # parts that no longer exist (and the .new.step of a first try)
        if f.stem not in WRITTEN:
            try:
                f.unlink()
            except PermissionError:                                    # Dropbox syncing it: the next run removes it
                print('  could not remove (locked):', f.name)
    changed = sorted(k for k, v in WRITTEN.items() if v)
    print(len(WRITTEN), 'part STEPs and the manifest in', RSW, f'({len(changed)} changed: {", ".join(changed) or "none"})')
    print('fasteners per robot:', json.dumps(man['fastener_count']))


if __name__ == '__main__':
    main()
