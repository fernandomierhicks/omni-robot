"""Independent STEP interface checks and floor IK, with outputs confined to CAD."""
import json
import sys
from pathlib import Path
import build123d as b
import numpy as np
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform

ROOT = Path(__file__).resolve().parents[2]
REPO = ROOT.parents[1]
sys.path.insert(0, str(REPO / 'ros/src/omni_arm'))
from omni_arm.kinematics import ArmConfig, ArmKinematics, DOWN

ROWS = json.loads((ROOT / 'reference/robot_sim_instances.json').read_text())
CACHE = {}


def transformed(shape, M):
    # Use the exact rigid matrix. OCCT/build123d Euler rotations use a different
    # convention from scipy's extrinsic xyz, especially for folded arm links.
    M = np.array(M)
    tr = gp_Trsf()
    tr.SetValues(*[float(x) for x in M[:3, :].ravel()])
    return b.Compound(BRepBuilderAPI_Transform(shape.wrapped, tr, True).Shape())


def part(product):
    if product not in CACHE:
        CACHE[product] = b.import_step(ROOT / 'step_parts' / (product + '.step'))
    return CACHE[product]


def item(row):
    return transformed(part(row['product']), row['matrix'])


def intersection_volume(a, c):
    if box_distance(a, c) > 1e-5:
        return 0.0
    if a.distance_to(c) > 1e-5:
        return 0.0
    common = a.intersect(c)
    if common is None:
        return 0.0
    return float(common.volume) if hasattr(common, 'volume') else sum(float(s.volume) for s in common)


def box_distance(a, c):
    ba, bc = a.bounding_box(), c.bounding_box()
    delta = np.maximum(np.maximum(np.array(list(ba.min)) - np.array(list(bc.max)),
                                  np.array(list(bc.min)) - np.array(list(ba.max))), 0)
    return float(np.linalg.norm(delta))


def main():
    # Snapshot the existing robot config; never run robot_export (writes ROS/fw).
    cfg_path = ROOT / 'reference/robot_arm_sim_snapshot.json'
    cfg_path.write_text((REPO / 'ros/src/omni_arm/omni_arm/config/robot_arm.json').read_text())
    kin = ArmKinematics(ArmConfig(cfg_path))
    assert abs(kin.fk(np.zeros(6))['arm_base'][2, 3] * 1000 - 224.5) < .001

    selected = [r for r in ROWS if not r['fastener'] and
                (r['product'] in ('payload_plate', 'RPLIDAR_A2M12', 'REC_arm_adapter', 'bin',
                                  'REC_arm_riser', 'REC_cheek_L', 'REC_cheek_R') or
                 r['group'] == 'head (camera bar, indexed)')]
    selected = [r for r in selected if not r['product'].startswith('label_')]
    shapes = [(r, item(r)) for r in selected]
    interfaces = []
    for i, (a, sa) in enumerate(shapes):
        for c, sc in shapes[i + 1:]:
            # Independent exact BREP checks at the revised interfaces.
            d = float(sa.distance_to(sc))
            v = intersection_volume(sa, sc) if d < 1e-5 else 0.0
            row = dict(a=a['name'], b=c['name'], clearance_mm=d, overlap_mm3=v)
            interfaces.append(row)
            if v > .1:
                print('INTERFACE OVERLAP', row, flush=True)
    report = dict(interfaces=interfaces, floor_poses=[], arm_checks=[],
                  nominal_arm_base_mm=224.5, note='Exact STEP checks, nominal suspension; no physical validation.')
    (ROOT / 'validation/sim_reconciliation_checks.json').write_text(json.dumps(report, indent=2))
    print('interfaces', len(interfaces), flush=True)

    links = json.loads((ROOT / 'reference/so101_links.json').read_text())
    arm = {l['body']: b.Compound(children=[transformed(part(p['product']), p['matrix']) for p in l['parts']]) for l in links}
    obstacles = [(r['name'], s) for r, s in shapes]
    # Include the retained rear mast and its actual housing (not the simulation's old boxes).
    for r in ROWS:
        if not r['fastener'] and r['group'].startswith('face mast') and not r['product'].startswith('label_'):
            obstacles.append((r['name'], item(r)))
    bar_active=(ROOT/'validation/d435_bar_mount_solidworks.json').exists()
    d435 = ROOT / ('validation/d435_bar_mount_geometry.json' if bar_active else 'validation/d435_integration_geometry.json')
    if d435.exists():
        dr = json.loads(d435.read_text())
        obstacles.append(('RealSense D435', transformed(b.import_step(ROOT / 'reference/d435_source.step'), dr['camera_source_to_robot'])))
        obstacles.append(('RealSense bar saddle' if bar_active else 'RealSense nose wedge', part('REC_D435_bar_saddle' if bar_active else 'REC_D435_wedge')))
    pose_list = [('high', [0, -.218, -1.268, -.207, 0, -.17]),
                 ('drop', [0, -1.3265, -1.5548, -1.6581, 0, -.17])]
    for x in (.27, .31, .34):
        target = np.array([x, 0, .0083])  # actual fingertip ~2 mm above the floor
        q = kin.ik(target, DOWN, yaw=0.0, flip_ok=False)
        if q is None:
            raise RuntimeError(f'No floor IK at {x}')
        q[5] = -.17
        assert np.all(q >= kin.lo - 1e-6) and np.all(q <= kin.hi + 1e-6)
        fk = kin.fk(q)
        row = dict(target_mm=(target * 1000).tolist(), q_rad=q.tolist(),
                   tool_mm=(fk['tool'][:3, 3] * 1000).tolist(),
                   down_cos=float(fk['tool'][:3, 0] @ DOWN), coarse_clear=kin.clear(q))
        report['floor_poses'].append(row)
        pose_list.append((f'floor_{int(x*1000)}', q))
        print('FLOOR IK', row, flush=True)
    for name, q in pose_list:
        frames = kin.fk(q)
        hits, minimum = [], float('inf')
        floor_min = float('inf')
        posed = {}
        for l in links:
            body = l['body']
            frame = frames[body].copy()
            frame[:3, 3] *= 1000
            transform = frame @ np.linalg.inv(np.array(l['frame']))
            s = transformed(arm[body], transform)
            posed[body] = s
            floor_min = min(floor_min, s.bounding_box().min.Z)
            if body == 'arm_base':
                continue     # Its underside intentionally seats on the adapter.
            for obs_name, obstacle in obstacles:
                # Disjoint exact AABBs certify separation without the expensive
                # multi-body BREP distance calculation. Only overlapping boxes
                # need the exact narrow-phase check.
                d = box_distance(s, obstacle)
                if d < 1e-5:
                    d = float(s.distance_to(obstacle))
                minimum = min(minimum, d)
                if d < 1e-5:
                    v = intersection_volume(s, obstacle)
                    if v > 1.0:
                        hits.append(dict(link=body, obstacle=obs_name, overlap_mm3=v))
        self_hits = []
        for i, l in enumerate(links):
            for other in links[i+1:]:
                if l['parent'] == other['body'] or other['parent'] == l['body']:
                    continue     # mating horn/servo interfaces intentionally overlap
                v = intersection_volume(posed[l['body']], posed[other['body']])
                if v > 2.0:
                    self_hits.append(dict(a=l['body'], b=other['body'], overlap_mm3=v))
        row = dict(pose=name, min_robot_clearance_lower_bound_mm=minimum, lowest_mesh_z_mm=floor_min,
                   overlaps=hits, nonadjacent_self_overlaps=self_hits)
        report['arm_checks'].append(row)
        print('ARM', row, flush=True)
        (ROOT / 'validation/sim_reconciliation_checks.json').write_text(json.dumps(report, indent=2))
    report['pass'] = not any(r['overlap_mm3'] > .1 for r in interfaces) and not any(r['overlaps'] or r['nonadjacent_self_overlaps'] for r in report['arm_checks'])
    (ROOT / 'validation/sim_reconciliation_checks.json').write_text(json.dumps(report, indent=2))
    print('PASS', report['pass'], flush=True)


if __name__ == '__main__':
    main()
