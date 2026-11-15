"""Pod strength check, step 3 (fe_pod.py combine): every wheel load the sim found (studies/pod_loads.py, the 5 ms peaks,
pod frame) through the unit-load FE of the fork, the inner frame and the keel, plus hand checks of the steel and the
bearings. Linear superposition; read the numbers as +-30 %.

A wheel's contact force F (pod frame) acts at (x_c, 0, z_c): x_c = -+D_ROW (both rows tried, the worse kept), z_c the
contact's height (-70 on the floor, -70 + h on an obstacle h high). The hub rides on the left 6804 (x_stub_brg) and the
AK40's output bearing (x_motor_brg): their radial loads balance F and its moment about the axle (two-support statics);
the axial load is split evenly. The motor's reaction torque (the sim's, both signs tried) goes into the plate.
"""
import json
import math
from pathlib import Path

import numpy as np

import fe_pod as FP

G = 9.81
CATS = (('A static, robot standing (creep)', None),
        ('B flat floor <= 1 m/s, hard starts and stops', lambda r: r['obstacle'] == 'flat' and r['speed'] <= 1.0),
        ('C 12 mm threshold or rug edge <= 1 m/s', lambda r: '12 mm' in r['obstacle'] and r['speed'] <= 1.0),
        ('D 18-25 mm threshold <= 1 m/s', lambda r: r['obstacle'] in ('threshold 18 mm', 'threshold 25 mm') and r['speed'] <= 1.0),
        ('E anything at 2 m/s (the speed cap)', lambda r: r['speed'] >= 2.0))
HEIGHT = {'flat': 0.0, 'threshold 12 mm': 12.0, 'threshold 18 mm': 18.0, 'threshold 25 mm': 25.0, 'rug edge 12 mm (square)': 12.0}
# bearings (typical catalogue values for the size, VERIFY on the parts bought): static C0 / dynamic C, N
BRG = {'627 (pivots)': (1370.0, 3300.0), '6804 (hub, stub)': (2470.0, 4000.0), '687 (roller)': (500.0, 1150.0)}
# printed PLA (typical, VERIFY on Fernando's prints): MPa
PLA = dict(in_layer=45.0, across_layers=20.0, interlayer_shear=15.0, fatigue_in_layer=15.0, creep=8.0)
# the fork's split joint (the halves meet on x = 42.7): the bar's end face 14 (y) x 29.5 (z) round a 9 x 24.5 tenon,
# clamped by the two inner M3 x 50 tie bolts (inner half in Ø3.4, threaded into the outer half's pilot)
R_AVG = 1.5      # mm: stresses are read as volume averages over this radius (critical distance; ~4 extrusion widths)
JOINT = dict(c=(42.7, 80.0, -14.75), b_y=14.0, h_z=29.5, tenon_area=9.0 * 24.5, bolts=2)
STEEL = dict(M3_88_yield=640.0, M3_minor_d=2.387, dowel_d=7.0, dowel_yield=500.0)


def split(F, tau, xc, zc, xl, xr):
    """the two hub bearings' loads (force ON the fork) for a contact force F at (xc, 0, zc)"""
    Fx, Fy, Fz = F
    My = zc * Fx - xc * Fz
    Mz = xc * Fy
    A = np.array([[1.0, 1.0], [xl, xr]])
    Lz, Rz = np.linalg.solve(A, [Fz, -My])
    Ly, Ry = np.linalg.solve(A, [Fy, Mz])
    return np.array([Fx / 2, Ly, Lz]), np.array([Fx / 2, Ry, Rz]), 1000.0 * tau      # tau N m -> N mm (the FE's units)


def vm(s):
    return np.sqrt(0.5 * ((s[0] - s[1]) ** 2 + (s[1] - s[2]) ** 2 + (s[2] - s[0]) ** 2) + 3 * (s[3] ** 2 + s[4] ** 2 + s[5] ** 2))


def main():
    pt = json.loads((FP.VAL / 'patches.json').read_text())
    xl, xr, dr = pt['x_stub_brg'], pt['x_motor_brg'], pt['d_row']
    rows = json.loads(FP.SIM_LOADS.read_text())
    mass = rows[0]['mass']
    W0 = mass * G / 4
    vecs = {c: [] for c, _ in CATS}
    vecs[CATS[0][0]].append(dict(F=[0.0, 0.0, W0], tau=0.0, zc=-70.0, src='static, a quarter of %.2f kg' % mass))
    for r in rows:
        for i, w in enumerate(r['wheels']):
            at = w['pod_5ms']['at_peak']
            if at is None:
                continue
            for c, f in CATS[1:]:
                if f(r):
                    vecs[c].append(dict(F=at['F_pod'], tau=at['torque'], zc=-70.0 + HEIGHT[r['obstacle']],
                                        src=f"{r['obstacle']} {r['direction']:g} deg {r['speed']:g} m/s wheel {i + 1}"))
    # the drive's own worst: stall torque against something (4.2 N m = 60 N at the rim) with 1.5 x the static load
    vecs[CATS[1][0]].append(dict(F=[0.0, 60.0, 1.5 * W0], tau=4.2, zc=-70.0, src='stall torque, rim force toward the pivot'))
    vecs[CATS[1][0]].append(dict(F=[0.0, -60.0, 1.5 * W0], tau=4.2, zc=-70.0, src='stall torque, rim force away from the pivot'))

    unit = {p: json.loads((FP.VAL / f'unit_{p}.json').read_text()) for p in ('fork', 'frame', 'keel')}
    data = {p: np.load(FP.SCRATCH / f'pod_fe_{p}.npz') for p in ('fork', 'frame', 'keel')}

    def fork_coeffs(L_, R_, tau):
        c = dict(stub_Fx=L_[0], stub_Fy=L_[1], stub_Fz=L_[2], motor_Fx=R_[0], motor_Fy=R_[1], motor_Fz=R_[2], motor_Tx=tau)
        return np.array([c[n] for n in unit['fork']['names']])

    def reactions(coef):
        out = {}
        for nm, a in zip(unit['fork']['names'], coef):
            for k, v in unit['fork']['reactions'][nm].items():
                out[k] = out.get(k, 0.0) + a * np.asarray(v, float)
        return out

    from scipy.spatial import cKDTree
    trees = {p: cKDTree(data[p]['cent']) for p in data}
    vols = {p: data[p]['vol'] for p in data}

    def avg_peak(part, val, mask, k=300):
        """the highest volume average of `val` (per element) over a sphere of R_AVG round any of the k largest
        candidates: a re-entrant corner's singular peak (mesh-dependent, unbounded) read at a critical distance"""
        d = data[part]
        cand = np.argsort(-np.where(mask, val, -np.inf))[:k]
        best, at = -np.inf, None
        for i in cand:
            nb = trees[part].query_ball_point(d['cent'][i], R_AVG)
            a = float((val[nb] * vols[part][nb]).sum() / vols[part][nb].sum())
            if a > best:
                best, at = a, i
        return best, [round(float(q_), 1) for q_ in d['cent'][at]]

    def peak(part, coef):
        d = data[part]
        S = np.tensordot(coef.astype(np.float32), d['S'], axes=1)          # (6, nel, nqp)
        Sm = S.mean(axis=2)                                                 # element means
        ok = d['dist'] > FP.EXCL
        v_raw = vm(S).max(axis=1)
        v, v_at = avg_peak(part, vm(Sm), ok)
        sxx, sxx_at = avg_peak(part, Sm[0], ok)
        # the layers (every pod part prints on an x face): peel sigma_xx (tension only) and interlayer shear
        # tau = |(s_xy, s_xz)|, together: (s_xx+ / across_layers)^2 + (tau / interlayer_shear)^2 (1 = the layers part)
        tau = np.hypot(Sm[4], Sm[5])
        f = (np.clip(Sm[0], 0, None) / PLA['across_layers']) ** 2 + (tau / PLA['interlayer_shear']) ** 2
        fl, fl_at = avg_peak(part, f, ok)
        il = int(np.argmin(np.linalg.norm(d['cent'] - np.array(fl_at), axis=1)))
        nb = trees[part].query_ball_point(d['cent'][il], R_AVG)
        w = vols[part][nb] / vols[part][nb].sum()
        if part != 'fork':                                                  # bonded foot: z tension just over the plate
            foot, _ = avg_peak(part, Sm[2], d['cent'][:, 2] < -6.5)        # = the foot prying up between its screws
        else:
            foot = 0.0
        return dict(vm=v, vm_at=v_at, sxx=sxx, sxx_at=sxx_at, layers=float(np.sqrt(max(fl, 0.0))), layers_at=fl_at,
                    layers_sxx=float((Sm[0][nb] * w).sum()), layers_tau=float((tau[nb] * w).sum()),
                    foot_pull=foot, vm_raw=float(np.where(ok, v_raw, -1).max()))

    def evaluate(F, tau_nm, zc, xc):
        """one wheel load F (pod frame, N) at (xc, 0, zc) with motor torque tau_nm (N m): the three parts' peaks and
        the hand checks"""
        F = np.asarray(F, float)
        L_, R_, tau = split(F, tau_nm, xc, zc, xl, xr)
        cf = fork_coeffs(L_, R_, tau)
        rx = reactions(cf)
        springs = [float(rx[f'spring_{i}']) for i in range(4)]
        fi, fo = np.asarray(rx['seat_inner'], float), np.asarray(rx['seat_outer'], float)
        cfr = np.array([dict(pivot_Fx=fi[0], pivot_Fy=fi[1], pivot_Fz=fi[2],
                             **{f'seat_{i}_Fz': springs[i] for i in range(4)})[n] for n in unit['frame']['names']])
        cke = np.array([dict(pivot_Fx=fo[0], pivot_Fy=fo[1], pivot_Fz=fo[2])[n] for n in unit['keel']['names']])
        out = dict(fork=peak('fork', cf), frame=peak('frame', cfr), keel=peak('keel', cke))
        fo_r, fi_r = math.hypot(fo[1], fo[2]), math.hypot(fi[1], fi[2])
        Z3 = math.pi * STEEL['M3_minor_d'] ** 3 / 32
        Z7 = math.pi * STEEL['dowel_d'] ** 3 / 32
        m_mot = np.cross(np.array([xr - 44.7, 0.0, 0.0]), R_)            # the can's moment on its floor
        # the split joint: everything that acts on the inner half (motor, inner 627, the four springs) crosses it
        cj = np.array(JOINT['c'])
        ext = [(R_, np.array([xr, 0.0, 0.0])), (-fi, np.array([pt['fork']['x_seat_inner'], *pt['pivot']]))]
        ext += [(np.array([0.0, 0.0, -springs[i]]), np.array([p_['x'], p_['y'], p_['z']])) for i, p_ in enumerate(pt['fork']['pads'])]
        Fe = sum(fv for fv, _ in ext)
        Me = sum(np.cross(r - cj, fv) for fv, r in ext) + np.array([tau, 0.0, 0.0])
        Fj, Mj = -Fe, -Me
        n_open = max(0.0, -Fj[0])                       # the outer half pulling the inner half toward -x
        out['hand'] = {
            'split joint: clamp to stay shut N (both tie bolts, no tenon)': n_open + 6 * abs(Mj[1]) / JOINT['h_z'] + 6 * abs(Mj[2]) / JOINT['b_y'],
            'split joint: tenon root bending MPa (tenon alone, across its layers)': abs(Mj[2]) / (JOINT['h_z'] - 5) / 9.0 ** 2 * 6 + abs(Mj[1]) / 9.0 / (JOINT['h_z'] - 5) ** 2 * 6 + n_open / JOINT['tenon_area'],
            'split joint: tenon shear MPa (V / 220 mm2)': math.hypot(Fj[1], Fj[2]) / JOINT['tenon_area'],
            'outer pivot M3 bending, MPa (627 4.4 mm out of the keel)': fo_r * 4.4 / Z3,
            'inner dowel bending, MPa (627 4.6 mm out of the frame)': fi_r * 4.6 / Z7,
            '627 outer radial N': fo_r, '627 inner radial N': fi_r,
            '6804 radial N': math.hypot(L_[1], L_[2]), 'motor bearing radial N': math.hypot(R_[1], R_[2]),
            '687 radial N (one roller, near bearing 1.05x)': 1.05 * math.hypot(F[0], F[2]),
            '687 axial N (along the roller)': abs(F[1]),
            'M2.5 motor screw tension N (moment / 2 r)': float(np.hypot(m_mot[1], m_mot[2]) / (2 * 23.5)),
            'spring max N (one plunger)': max(springs),
        }
        return out

    def record(worst, out, key, F):
        for part in ('fork', 'frame', 'keel'):
            k2 = f'{part} vm raw corner peak (singular, mesh-dependent)'
            if k2 not in worst or out[part]['vm_raw'] > worst[k2]['value']:
                worst[k2] = dict(value=round(out[part]['vm_raw'], 1), case=key)
            for q in ('vm', 'sxx', 'layers'):
                k2 = f'{part} {q}'
                if k2 not in worst or out[part][q] > worst[k2]['value']:
                    worst[k2] = dict(value=round(out[part][q], 2), at=out[part][q + '_at'], case=key,
                                     F_pod=[round(float(v), 1) for v in F])
                    if q == 'layers':
                        worst[k2].update(sxx=round(out[part]['layers_sxx'], 1), tau=round(out[part]['layers_tau'], 1))
            if part != 'fork':
                k2 = f'{part} foot pull MPa (bonded in the model)'
                if k2 not in worst or out[part]['foot_pull'] > worst[k2]['value']:
                    worst[k2] = dict(value=round(out[part]['foot_pull'], 2), case=key)
        for k, v in out['hand'].items():
            if k not in worst or v > worst[k]['value']:
                worst[k] = dict(value=round(float(v), 1), case=key, F_pod=[round(float(q), 1) for q in F])

    res = {}
    for cat, _ in CATS:
        worst = {}
        for vct in vecs[cat]:
            for xc in (-dr, dr):
                for sgn in ((1.0, -1.0) if vct['tau'] else (1.0,)):
                    out = evaluate(vct['F'], sgn * vct['tau'], vct['zc'], xc)
                    key = vct['src'] + f' x_c {xc:+.2f}' + (f' tau {sgn * vct["tau"]:+.1f}' if vct['tau'] else '')
                    record(worst, out, key, vct['F'])
        res[cat] = dict(n_loads=len(vecs[cat]), worst=worst)
    # sensitivities: 100 N in each direction alone (worse row; axial and in-plane at a 12 mm edge's height, z -58)
    sens = {}
    for name, F, zc in (('100 N up (floor)', (0, 0, 100.0), -70.0), ('100 N toward the pivot (edge, z -58)', (0, 100.0, 0), -58.0),
                        ('100 N away from the pivot (edge, z -58)', (0, -100.0, 0), -58.0),
                        ('100 N along the axle, toward the motor (edge, z -58)', (100.0, 0, 0), -58.0),
                        ('100 N along the axle, outward (edge, z -58)', (-100.0, 0, 0), -58.0)):
        worst = {}
        for xc in (-dr, dr):
            record(worst, evaluate(F, 0.0, zc, xc), f'x_c {xc:+.2f}', F)
        sens[name] = worst
    rep = dict(note=__doc__.strip().splitlines()[0], static_wheel_load_N=round(W0, 1), pla_MPa=PLA, bearings_C0_C_N=BRG,
               steel=STEEL, fe=dict(E=FP.E, nu=FP.NU, excl_mm=FP.EXCL, meshes={p: {k: unit[p][k] for k in ('H', 'tets', 'dofs')} for p in unit}),
               x_stub_brg=xl, x_motor_brg=xr, layers_note='layers = sqrt((s_xx+ / across_layers)^2 + (tau / interlayer_shear)^2): 1 = the layers part',
               categories=res, per_100N=sens)
    (FP.VAL / 'results.json').write_text(json.dumps(rep, indent=1))
    for block in (res, sens):
        for cat in block:
            w = block[cat]['worst'] if 'worst' in block[cat] else block[cat]
            print()
            print(cat)
            for k, v in w.items():
                print(f'  {k:62s} {v["value"]:8.1f}  {v.get("at", "")}  {v["case"][:60]}')


if __name__ == '__main__':
    main()
