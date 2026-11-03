"""Where can each SO-101 joint be mated? Lists coaxial cylinders and perpendicular planes of the two links at each joint.

    cadenv python 42_arm_explore.py          (needs the arm link sub-assemblies: 40_robot_subassemblies.py arm_links)
"""
import sys
from subasm import *
from importlib import import_module

R = import_module('40_robot_subassemblies')
LINKS = R.LINKS
NAMES = R.LINK_NAMES


def main():
    sw = connect()
    SP = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'validation' / 'arm_explore.SLDASM'
    a = Asm(sw, SP)
    for l in LINKS:
        a.add(l['body'], ASSEMBLIES / f'V10H_{NAMES[l["body"]]}.SLDASM', np.eye(4), fixed=True)
    a.refresh()
    out = {}
    for l in LINKS:
        j = l.get('joint')
        if not j:
            continue
        u = np.array(j['axis'], float)
        u /= np.linalg.norm(u)
        anc = np.array(j['anchor'], float)
        P, C = l['parent'], l['body']
        rec = dict(parent=P, child=C, axis=u.tolist(), anchor=anc.tolist(), cyl={}, planes={})
        for tag, body in (('parent', P), ('child', C)):
            fs = a.mater.faces(a.name_of(body))
            cyls = []
            for f in fs:
                if f['kind'] == 'cyl' and abs(abs(f['axis'] @ u) - 1) < 1e-5:
                    d = f['pt'] - anc
                    off = float(np.linalg.norm(d - (d @ u) * u))
                    if off < 0.3:
                        lo = float(((f['aabb'][0] + f['aabb'][1]) / 2 - anc) @ u)
                        ext = float(np.abs(((f['aabb'][1] - f['aabb'][0]) / 2) @ np.abs(u)))
                        cyls.append(dict(r=round(f['r'], 3), off=round(off, 3), mid=round(lo, 2), half=round(ext, 2), area=round(f['area'], 1)))
            pls = []
            for f in fs:
                if f['kind'] == 'plane' and abs(abs(f['n'] @ u) - 1) < 1e-5:
                    pls.append(dict(s=round(float(f['n'] @ u) * 1.0, 0), pos=round(float(f['off'] * (f['n'] @ u)) - float(anc @ u), 3), area=round(f['area'], 1),
                                    ctr=(f['ctr'] - anc).round(1).tolist()))
            rec['cyl'][tag] = sorted(cyls, key=lambda x: -x['area'])[:8]
            rec['planes'][tag] = sorted(pls, key=lambda x: -x['area'])[:8]
        out[C] = rec
        print('\n=== joint', j['name'], P, '->', C)
        for tag in ('parent', 'child'):
            print(' ', tag, 'cyl:', rec['cyl'][tag])
            print(' ', tag, 'planes:', rec['planes'][tag])
    (ROOT / 'validation' / 'arm_explore.json').write_text(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
