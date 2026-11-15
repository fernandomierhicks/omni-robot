"""Pod strength check, pictures (Nov 12 2026): the fork's surface coloured by the layer-separation index of fe_pod.py's
unit cases, superposed for a chosen wheel load.

    <FE venv> python -B fe_pod_plot.py mesh       re-mesh fork.step exactly as fe_pod.py solve fork 3.0 did -> SCRATCH/pod_fe_fork_surf.npz
    <sim venv> python -B fe_pod_plot.py plot      -> validation/pod_fe/fork_layers_*.png (needs matplotlib)

layers = sqrt((sigma_xx+ / 20)^2 + (tau_x / 15)^2), element means (as fe_pod_combine, without its 1.5 mm averaging, so
sharp corners read high); 1 = the layers part (typical printed PLA, VERIFY). Grey: within 3 mm of a load or support patch
(the load model's own artefacts).
"""
import json
import sys
from pathlib import Path

import numpy as np

import fe_pod as FP

H = 3.0
SPLIT_X = 42.7
SIM = FP.SIM_LOADS


def mesh():
    X, T = FP.mesh_step(FP.VAL / 'fork.step', H)
    d = np.load(FP.SCRATCH / 'pod_fe_fork.npz')
    assert len(T) == d['cent'].shape[0], ('mesh differs from the solve', len(T), d['cent'].shape)
    cent = X[T].mean(axis=1)
    assert np.abs(cent - d['cent']).max() < 1e-3, 'element order differs'
    faces = np.concatenate([T[:, [0, 1, 2]], T[:, [0, 1, 3]], T[:, [0, 2, 3]], T[:, [1, 2, 3]]])
    owner = np.tile(np.arange(len(T)), 4)
    key = np.sort(faces, axis=1)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    b = cnt[inv.ravel()] == 1
    np.savez_compressed(FP.SCRATCH / 'pod_fe_fork_surf.npz', X=X, tri=faces[b], owner=owner[b])
    print('surface triangles', int(b.sum()))


def split(F, tau, xc, zc, xl, xr):
    Fx, Fy, Fz = F
    My = zc * Fx - xc * Fz
    Mz = xc * Fy
    A = np.array([[1.0, 1.0], [xl, xr]])
    Lz, Rz = np.linalg.solve(A, [Fz, -My])
    Ly, Ry = np.linalg.solve(A, [Fy, Mz])
    return np.array([Fx / 2, Ly, Lz]), np.array([Fx / 2, Ry, Rz]), 1000.0 * tau


def cases():
    """(title, F pod, tau N m, z_c, x_c): the loads drawn"""
    rows = json.loads(SIM.read_text())
    W0 = rows[0]['mass'] * 9.81 / 4

    def worst_axial(obstacle, speed):
        best = None
        for r in rows:
            if r['obstacle'] == obstacle and r['speed'] == speed:
                for w in r['wheels']:
                    at = w['pod_5ms']['at_peak']
                    if at and (best is None or abs(at['F_pod'][0]) > abs(best[0][0])):
                        best = (at['F_pod'], at['torque'], r['direction'])
        return best
    out = [('standing (a quarter of the robot)', [0, 0, W0], 0.0, -70.0)]
    fl = None                          # flat 1 m/s: the wheel load with the largest in-plane push
    for r in rows:
        if r['obstacle'] == 'flat' and r['speed'] == 1.0:
            for w in r['wheels']:
                at = w['pod_5ms']['at_peak']
                if at and (fl is None or np.hypot(*at['F_pod'][1:]) > np.hypot(*fl[0][1:])):
                    fl = (at['F_pod'], at['torque'])
    out.append(('flat floor 1 m/s, hard stop (sim 5 ms peak)', fl[0], fl[1], -70.0))
    out.append(('100 N along the axle at a 12 mm edge', [-100.0, 0, 0], 0.0, -58.0))
    F, t, d = worst_axial('threshold 12 mm', 0.5)
    out.append((f'12 mm threshold at 0.5 m/s ({d:g} deg, sim 5 ms peak)', F, t, -58.0))
    return out


def field(d, unit, pt, F, tau, zc, xc):
    L_, R_, tq = split(np.asarray(F, float), tau, xc, zc, pt['x_stub_brg'], pt['x_motor_brg'])
    c = dict(stub_Fx=L_[0], stub_Fy=L_[1], stub_Fz=L_[2], motor_Fx=R_[0], motor_Fy=R_[1], motor_Fz=R_[2], motor_Tx=tq)
    coef = np.array([c[n] for n in unit['names']], np.float32)
    S = np.tensordot(coef, d['S'], axes=1).mean(axis=2)
    tau_x = np.hypot(S[4], S[5])
    return np.sqrt((np.clip(S[0], 0, None) / 20.0) ** 2 + (tau_x / 15.0) ** 2)


def plot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    d = np.load(FP.SCRATCH / 'pod_fe_fork.npz')
    s = np.load(FP.SCRATCH / 'pod_fe_fork_surf.npz')
    unit = json.loads((FP.VAL / 'unit_fork.json').read_text())
    pt = json.loads((FP.VAL / 'patches.json').read_text())
    X, tri, own = s['X'], s['tri'], s['owner']
    P = X[tri]
    n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    excl = d['dist'][own] < FP.EXCL
    cmap = plt.get_cmap('turbo')
    VMAX = 1.5
    views = (('outside face (from -x)', 5, 180, None), ('from the wheel side (-y), below', -25, -60, None),
             ('from behind (+y), below', -25, 120, None),
             ('ZOOM: outer arm meets the bar (from the wheel side, below)', -20, -50, ((-55, -15), (55, 95), (-40, 5))))
    for ci, (title, F, tau, zc) in enumerate(cases()):
        best = None
        for xc in (-pt['d_row'], pt['d_row']):
            v = field(d, unit, pt, F, tau, zc, xc)
            m = np.where(d['dist'] > FP.EXCL, v, 0).max()
            if best is None or m > best[0]:
                best = (m, v, xc)
        m, v, xc = best
        vs = v[own]
        ok = d['dist'] > FP.EXCL
        top = np.argsort(-np.where(ok, v, -1))[:1]
        at = d['cent'][top[0]]
        fig = plt.figure(figsize=(16, 13))
        for k, (vt, el, az, box) in enumerate(views):
            ax = fig.add_axes([0.01 + 0.45 * (k % 2), 0.48 - 0.46 * (k // 2), 0.45, 0.44], projection='3d')
            th, ph = np.radians(el), np.radians(az)
            view = np.array([np.cos(th) * np.cos(ph), np.cos(th) * np.sin(ph), np.sin(th)])
            shade = 0.5 + 0.5 * np.abs(n @ view)
            col = cmap(np.clip(vs / VMAX, 0, 1))
            col[excl] = (0.6, 0.6, 0.6, 1.0)
            col[:, :3] *= shade[:, None]
            sel = np.ones(len(P), bool)
            if box is not None:
                cc = P.mean(axis=1)
                for a_ in range(3):
                    sel &= (cc[:, a_] > box[a_][0]) & (cc[:, a_] < box[a_][1])
                lo, hi = np.array([b_[0] for b_ in box]), np.array([b_[1] for b_ in box])
            else:
                lo, hi = X.min(axis=0), X.max(axis=0)
            pc = Poly3DCollection(P[sel], facecolors=col[sel], edgecolors='none', linewidths=0)
            ax.add_collection3d(pc)
            mid, r = (lo + hi) / 2, (hi - lo).max() / 2 * 0.78
            ax.set_xlim(mid[0] - r, mid[0] + r); ax.set_ylim(mid[1] - r, mid[1] + r); ax.set_zlim(mid[2] - r, mid[2] + r)
            ax.set_box_aspect((1, 1, 1))
            ax.view_init(elev=el, azim=az)
            ax.set_axis_off()
            ax.scatter(*at, s=260, facecolors='none', edgecolors='k', linewidths=2.5, depthshade=False)
            # axes tripod: x (axle, toward the motor) red, y (toward the pivot) green, z up blue
            o = lo + 0.05 * (hi - lo)
            L = 0.15 * r
            for e, cl, lb in ((0, 'r', 'x motor'), (1, 'g', 'y pivot'), (2, 'b', 'z up')):
                dv = np.zeros(3); dv[e] = L
                ax.plot(*np.c_[o, o + dv], color=cl, lw=2)
                ax.text(*(o + 1.25 * dv), lb, color=cl, fontsize=8)
            ax.set_title(vt, fontsize=11)
        fig.suptitle(f'Fork, layer-separation index (1 = layers part; typical PLA 20 MPa peel / 15 MPa interlayer shear)\n'
                     f'{title}: F pod = ({F[0]:.0f}, {F[1]:.0f}, {F[2]:.0f}) N, contact row x {xc:+.2f}; '
                     f'peak {m:.2f} at ({at[0]:.0f}, {at[1]:.0f}, {at[2]:.0f}) (element value, corners read high)', fontsize=12)
        sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, VMAX))
        cb = fig.colorbar(sm, cax=fig.add_axes([0.92, 0.25, 0.015, 0.5]))
        cb.set_label('layer index (grey: next to a load / support patch)')
        out = FP.VAL / f'fork_layers_{ci}.png'
        fig.savefig(out, dpi=90)
        plt.close(fig)
        print(out, round(float(m), 2), at)


if __name__ == '__main__':
    {'mesh': mesh, 'plot': plot}[sys.argv[1]]()
