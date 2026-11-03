"""Drive every SO-101 joint of V10H_robot.SLDASM through its limit mate and compare the real rotation with the mate value.

    cadenv python test_arm_motion.py       (nothing is saved)
For each joint: the angle mate's dimension is set to its minimum + 0.5, the delivered value and maximum - 0.5 (degrees); the link
must turn by exactly that amount about the joint axis, the other links must stay, and no mate may report an error.
"""
import sys
from importlib import import_module
from subasm import *

RB = import_module('41_robot')


def main():
    sw = connect()
    discard(sw, 'V10H_robot.SLDASM')
    a = RB.load(sw)
    q = dict(zip(RB.BODIES, RB.POSES[RB.DELIVERED_POSE]))
    X = RB.link_transforms(q)
    plan = {r['joint']: r for r in json.loads((ROOT / 'validation' / 'arm_angle_mates.json').read_text())}
    rows, ok = [], True
    for l in RB.LINKS:
        j = l.get('joint')
        if not j:
            continue
        C = l['body']
        r = plan[j['name']]
        f = wrap(a.asm.FeatureByName(r['mate']), sld.IFeature)
        dim = RB.mate_dim(f)
        u = X[l['parent']][:3, :3] @ np.array(j['axis'])
        u /= np.linalg.norm(u)
        others = {b: placement(a.comp_of(b)) for b in RB.W0 if b != C}
        R0 = placement(a.comp_of(C))[:, :3].copy()
        v0 = r['value_deg']
        for target in (r['min_deg'] + 0.5, v0, r['max_deg'] - 0.5):
            dim.SetSystemValue3(math.radians(target), 1, None)
            a.model.EditRebuild3()
            got = math.degrees(RB.rot_about(R0, placement(a.comp_of(C))[:, :3], u))
            want = r['sign'] * (target - v0)
            moved = max(float(np.abs(placement(a.comp_of(b)) - p).max()) for b, p in others.items() if RB.PAR.get(b) != C and b not in descendants(C))
            errs = M.mate_errors(a.model)
            row = dict(joint=j['name'], mate_deg=round(target, 1), turned_deg=round(got, 2), expected_deg=round(want, 2), others_moved_mm=round(moved, 4), mate_errors=errs)
            row['pass'] = abs(got - want) < 0.2 and not errs and moved < 0.01
            ok &= row['pass']
            rows.append(row)
            print(row, flush=True)
        dim.SetSystemValue3(math.radians(v0), 1, None)
        a.model.EditRebuild3()
    discard(sw, 'V10H_robot.SLDASM')
    (ROOT / 'validation' / 'arm_motion.json').write_text(json.dumps(rows, indent=1))
    print('PASS' if ok else 'FAIL')


def descendants(b):
    out, todo = set(), [b]
    while todo:
        x = todo.pop()
        for k, p in RB.PAR.items():
            if p == x and k not in out:
                out.add(k)
                todo.append(k)
    return out


if __name__ == '__main__':
    main()
