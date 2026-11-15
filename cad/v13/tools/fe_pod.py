"""Pod strength check, step 2 (Nov 12 2026): linear elastic P2 tets (gmsh + scikit-fem + pyamg, the venv from
requirements-fe.txt, as fe_hub.py) on the pod's three printed load paths, unit loads only; step 3 (`combine`) scales and
sums them for every wheel load the sim found (simulation/v11 sil/studies/pod_loads.py).

    python -B fe_pod.py solve [fork|frame|keel] [H]     unit cases -> SCRATCH/pod_fe_<part>.npz (big: not in the repo)
    python -B fe_pod.py combine                          -> validation/pod_fe/results.json

Inputs: validation/pod_fe/*.step and patches.json (pod_fe_prep.py, the CAD interpreter). Pod frame: x along the axle toward
the motor, y toward the pivot, z up; mm, N, MPa.

Model (preliminary; read the numbers as +-30 %, the peaks at sharp corners as mesh-dependent):
  * PLA, E 3.5 GPa, nu 0.36, solid (100 % infill): a part printed with 3-4 walls and sparse infill is weaker in its core;
    the peaks sit at the surface, which the walls carry.
  * fork = both halves fused (the tie bolts and the tenon as a bonded joint). Loads on it: the hub's left 6804 on the axle
    stub (uniform over the stub's seat), the AK40's output bearing (x_motor_brg) carried by the can to its pocket: the force
    and its moment about the pocket floor as a linear pressure on the floor, the motor's reaction torque the same way.
    Supports: both 627 seats hold the bore's mean y, z displacement (stiff; the fork turns and tilts on them, SEAT below),
    the outer seat also its mean x; each
    plunger dome a 6.1 N/mm spring on a Ø6 patch of its pad. The springs carry the moment about the pivot; nothing else
    does, so the spring and pivot loads are statics.
  * frame and keel: foot bonded to the plate (the screws clamp it); the pivot's load on the dowel bore / the M3's pilot as
    a force plus its moment (the 627 sits ~10.6 mm outboard of each) by a linear traction; each spring's push on its seat.
  * every part prints on an x face, so sigma_xx is the stress across the layers.
"""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
VAL = ROOT / 'validation' / 'pod_fe'
REPO = ROOT.parents[1]
SIM_LOADS = REPO / 'simulation' / 'v11 sil' / 'studies' / 'results' / 'pod_loads.json'
SCRATCH = Path(__import__('os').environ.get('POD_FE_SCRATCH', __import__('tempfile').gettempdir())) / 'pod_fe'   # ~100 MB
E, NU = 3500.0, 0.36                 # PLA (typical, assumed)
KP = 2.0e4                           # N/mm^3 bearing-seat penalty (~10x E / h)
TOL = 0.2                            # mm, patch selection
EXCL = 3.0                           # mm: stresses within this of a load / support patch are not reported (load-model artefacts)
# the 627 seats: 'mean' (default since Nov 12 evening) holds each bore's mean displacement, so the fork can tilt in its
# bearings as a single deep-groove 627 lets it; 'penalty' (the first run, results_seat_penalty.json) held every point of
# the 7 mm bore, which clamped the tilt and pushed an axial wheel load's moment into the outer seat as a local couple
# (a large interlayer shear beside it) instead of as a pair of radial loads on both 627s through the bar
SEAT = __import__('os').environ.get('POD_FE_SEAT', 'mean')


# ---- step 2: solve ---------------------------------------------------------------------------------------------------
def mesh_step(step, hmax):
    import gmsh
    for a2, a3, mx in ((6, 1, hmax), (1, 10, hmax), (1, 10, 0.8 * hmax)):
        gmsh.initialize()
        gmsh.option.setNumber('General.Terminal', 0)
        try:
            gmsh.model.occ.importShapes(str(step))
            gmsh.model.occ.synchronize()
            for k, v in (('Mesh.MeshSizeMax', mx), ('Mesh.MeshSizeMin', 0.3 * mx), ('Mesh.MeshSizeFromCurvature', 12),
                         ('Mesh.Algorithm', a2), ('Mesh.Algorithm3D', a3)):
                gmsh.option.setNumber(k, v)
            gmsh.model.mesh.generate(3)
            et, en = gmsh.model.mesh.getElementsByType(4)
            if len(et) == 0:
                raise RuntimeError('empty mesh')
            nt, xyz, _ = gmsh.model.mesh.getNodes()
            idx = np.full(int(max(nt)) + 1, -1, dtype=np.int64)
            idx[np.array(nt, dtype=np.int64)] = np.arange(len(nt))
            X = np.array(xyz).reshape(-1, 3)
            T = idx[np.array(en, dtype=np.int64).reshape(-1, 4)]
            used = np.unique(T)
            remap = np.full(len(X), -1, dtype=np.int64)
            remap[used] = np.arange(len(used))
            print('mesher ok', a2, a3, mx, flush=True)
            return X[used], remap[T]
        except Exception as ex:                    # noqa: BLE001
            print('mesher failed', a2, a3, mx, str(ex)[:80], flush=True)
        finally:
            gmsh.finalize()
    raise SystemExit('no mesher setting worked')


def selector(p):
    """facet-midpoint test for a patch dict"""
    k = p['kind']
    if k == 'cyl':
        ay, az = p['axis']
        return lambda x: ((np.abs(np.hypot(x[1] - ay, x[2] - az) - p['r']) < TOL) & (x[0] > p['x'][0] - TOL) & (x[0] < p['x'][1] + TOL))
    if k == 'xplane':
        ay, az = p['axis']
        return lambda x: (np.abs(x[0] - p['x']) < TOL) & (np.hypot(x[1] - ay, x[2] - az) < p['r_max'] + TOL)
    if k == 'zplane':
        return lambda x: np.abs(x[2] - p['z']) < TOL
    if k == 'zdisc':
        return lambda x: (np.abs(x[2] - p['z']) < TOL) & (np.hypot(x[0] - p['x'], x[1] - p['y']) < p['r'])
    if k == 'zannulus':
        return lambda x: ((np.abs(x[2] - p['z']) < TOL) & (np.hypot(x[0] - p['x'], x[1] - p['y']) < p['r_out'] + TOL)
                          & (np.hypot(x[0] - p['x'], x[1] - p['y']) > p['r_in'] - TOL))
    raise ValueError(k)


def solve(part, H):
    from skfem import (Basis, BilinearForm, ElementTetP2, ElementVector, FacetBasis, LinearForm, MeshTet)
    from skfem.helpers import ddot, sym_grad, trace
    from scipy.sparse.linalg import cg
    from scipy.spatial import cKDTree
    import pyamg
    pt = json.loads((VAL / 'patches.json').read_text())
    spec = pt[part]
    t0 = time.time()
    X, T = mesh_step(VAL / f'{part}.step', H)
    mesh = MeshTet(X.T.copy(), T.T.copy())
    elem = ElementVector(ElementTetP2())
    basis = Basis(mesh, elem, intorder=2)                 # exact for the stiffness of straight P2 tets (4 points)
    print(part, 'nodes', len(X), 'tets', len(T), 'dofs', basis.N, 'mesh s', round(time.time() - t0, 1), flush=True)
    lam, mu = E * NU / ((1 + NU) * (1 - 2 * NU)), E / (2 * (1 + NU))

    @BilinearForm
    def stiff(u, v, w):
        return lam * trace(sym_grad(u)) * trace(sym_grad(v)) + 2 * mu * ddot(sym_grad(u), sym_grad(v))

    K = stiff.assemble(basis)
    print(part, 'K assembled s', round(time.time() - t0, 1), flush=True)

    def fbasis(p):
        f = mesh.facets_satisfying(selector(p), boundaries_only=True)
        if len(f) == 0:
            raise SystemExit(f'{part}: patch {p} found no facets')
        fb = FacetBasis(mesh, elem, facets=f)
        x = fb.global_coordinates().value
        dx = fb.dx
        A = float(dx.sum())
        c = np.array([(x[i] * dx).sum() for i in range(3)]) / A
        return fb, f, A, c, x, dx

    def penalty_radial(p, axial=False):
        fb, f, A, c, _, _ = fbasis(p)
        ay, az = p['axis']

        @BilinearForm
        def pen(u, v, w):
            ry, rz = w.x[1] - ay, w.x[2] - az
            rr = np.sqrt(ry ** 2 + rz ** 2)
            ny, nz = ry / rr, rz / rr
            out = KP * (u[1] * ny + u[2] * nz) * (v[1] * ny + v[2] * nz)
            if axial:
                out = out + KP * u[0] * v[0]
            return out
        return pen.assemble(fb), fb, A

    def spring(p, k_total):
        fb, f, A, c, _, _ = fbasis(p)

        @BilinearForm
        def spr(u, v, w):
            return (k_total / A) * u[2] * v[2]
        return spr.assemble(fb), fb, A

    def traction(p, F, M=(0.0, 0.0, 0.0), about=None):
        """force F (uniform) + moment M about `about` (default the patch centroid): t = F / A + Omega x (x - c), with
        Omega = I^-1 (M + (c - about) x F ... ) so that the patch's resultant is F at `about` plus M"""
        fb, f, A, c, x, dx = fbasis(p)
        F, M = np.asarray(F, float), np.asarray(M, float)
        o = c if about is None else np.asarray(about, float)
        Mc = M + np.cross(o - c, F)                       # the same load about the centroid
        r = x - c[:, None, None]
        r2 = (r ** 2).sum(axis=0)
        I = np.zeros((3, 3))
        for i in range(3):
            for j in range(3):
                I[i, j] = ((r2 * (i == j) - r[i] * r[j]) * dx).sum()
        Om = np.linalg.solve(I, Mc) if np.linalg.norm(Mc) > 0 else np.zeros(3)

        @LinearForm
        def lf(v, w):
            rr = [w.x[i] - c[i] for i in range(3)]
            t = [F[i] / A + (Om[(i + 1) % 3] * rr[(i + 2) % 3] - Om[(i + 2) % 3] * rr[(i + 1) % 3]) for i in range(3)]
            return t[0] * v[0] + t[1] * v[1] + t[2] * v[2]
        return lf.assemble(fb), f

    def mean_support(p, comps):
        """a stiff spring on the patch's MEAN displacement in each of `comps`: the 627's rigid race on its balls holds
        the bore's centre but not its tilt (a single deep-groove bearing takes next to no moment)"""
        from scipy.sparse import csr_matrix
        fb, f, A, c, _, _ = fbasis(p)
        rows = []
        for cc in comps:
            @LinearForm
            def avg(v, w, cc=cc):
                return v[cc] / A
            rows.append(avg.assemble(fb))
        Bm = csr_matrix(np.array(rows))
        return (KP * A) * (Bm.T @ Bm).tocsr(), fb, A, rows

    supports, loads, dirich, patch_facets = [], {}, None, []
    seat_rows = {}
    if part == 'fork':
        for name, axial in (('seat_outer', True), ('seat_inner', False)):
            if SEAT == 'mean':
                comps = (0, 1, 2) if axial else (1, 2)
                Kp, fb, A, rows_ = mean_support(spec[name], comps)
                seat_rows[name] = (KP * A, dict(zip(comps, rows_)))
            else:
                Kp, fb, A = penalty_radial(spec[name], axial)
            K = K + Kp
            supports.append((name, fb, spec[name]))
            patch_facets.append(fb.find)
        for i, pad in enumerate(spec['pads']):
            p = dict(kind='zdisc', **pad)
            Ks, fb, A = spring(p, spec['spring_rate'])
            K = K + Ks
            supports.append((f'spring_{i}', fb, p))
            patch_facets.append(fb.find)
        xb = pt['x_motor_brg']
        for k, e in enumerate(np.eye(3)):
            loads[f'stub_F{"xyz"[k]}'], f = traction(spec['stub'], e)
            patch_facets.append(f)
        floor = spec['motor_floor']
        for k, e in enumerate(np.eye(3)):
            # the can carries the bearing's force from x_motor_brg on the axis to its pocket floor: force + moment
            loads[f'motor_F{"xyz"[k]}'], f = traction(floor, e, about=(xb, 0.0, 0.0))
        loads['motor_Tx'], f = traction(floor, (0.0, 0.0, 0.0), M=(1.0, 0.0, 0.0))
        patch_facets.append(f)
    else:
        foot = spec['foot']
        ff = mesh.facets_satisfying(selector(foot), boundaries_only=True)
        dirich = basis.get_dofs(ff).all()
        patch_facets.append(ff)
        key = 'dowel' if part == 'frame' else 'pin'
        xl = spec['x_load']
        ay, az = pt['pivot']
        for k, e in enumerate(np.eye(3)):
            loads[f'pivot_F{"xyz"[k]}'], f = traction(spec[key], e, about=(xl, ay, az))
        patch_facets.append(f)
        if part == 'frame':
            for i, s in enumerate(spec['seats']):
                loads[f'seat_{i}_Fz'], f = traction(dict(kind='zannulus', **s), (0.0, 0.0, 1.0))
                patch_facets.append(f)
    print(part, 'assembled, loads', list(loads), 's', round(time.time() - t0, 1), flush=True)

    ndof = K.shape[0]
    free = np.setdiff1d(np.arange(ndof), dirich) if dirich is not None else np.arange(ndof)
    Kf = K[free][:, free].tocsr()
    # the six rigid-body modes as AMG's near-null space (P2 vector dofs are interleaved: dof i is component i % 3), so
    # the soft spring mode (the fork turning on its pivots against 6.1 N/mm springs) does not stall the iterations
    loc = basis.doflocs
    comp = np.arange(ndof) % 3
    B = np.zeros((ndof, 6))
    for c in range(3):
        B[comp == c, c] = 1.0
    xm_ = loc.mean(axis=1)
    xr_, yr_, zr_ = loc[0] - xm_[0], loc[1] - xm_[1], loc[2] - xm_[2]
    B[comp == 1, 3], B[comp == 2, 3] = -zr_[comp == 1], yr_[comp == 2]      # about x
    B[comp == 0, 4], B[comp == 2, 4] = zr_[comp == 0], -xr_[comp == 2]      # about y
    B[comp == 0, 5], B[comp == 1, 5] = -yr_[comp == 0], xr_[comp == 1]      # about z
    ml = pyamg.smoothed_aggregation_solver(Kf, B=B[free], max_coarse=500)
    Mpre = ml.aspreconditioner(cycle='V')
    sb = basis                                            # stress output at its 4 points per tet
    lamb, mub = lam, mu
    names = list(loads)
    S = None
    react = {}
    for li, nm in enumerate(names):
        f = loads[nm]
        uf, info = cg(Kf, f[free], M=Mpre, rtol=1e-6, maxiter=6000)
        res = float(np.linalg.norm(Kf @ uf - f[free]) / max(np.linalg.norm(f[free]), 1e-30))
        u = np.zeros(ndof)
        u[free] = uf
        G = sb.interpolate(u).grad
        eps = 0.5 * (G + np.swapaxes(G, 0, 1))
        tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
        sig = 2 * mub * eps + lamb * tr * np.eye(3)[:, :, None, None]
        voigt = np.stack([sig[0, 0], sig[1, 1], sig[2, 2], sig[1, 2], sig[0, 2], sig[0, 1]]).astype(np.float32)
        if S is None:
            S = np.zeros((len(names),) + voigt.shape, dtype=np.float32)
        S[li] = voigt
        rr = {}
        if part == 'fork':
            for sname, fb, p in supports:
                ub = fb.interpolate(u)
                x = fb.global_coordinates().value
                dx = fb.dx
                val = ub.value
                if sname.startswith('spring'):
                    A = float(dx.sum())
                    rr[sname] = float((spec['spring_rate'] / A * val[2] * dx).sum())   # + : the pad pushed up: spring loaded
                elif sname in seat_rows:
                    kk, rws = seat_rows[sname]
                    rr[sname] = [float(kk * (rws[cc] @ u)) if cc in rws else 0.0 for cc in range(3)]
                else:
                    ay, az = p['axis']
                    ry, rz = x[1] - ay, x[2] - az
                    rn = np.sqrt(ry ** 2 + rz ** 2)
                    ny, nz = ry / rn, rz / rn
                    un = val[1] * ny + val[2] * nz
                    Fv = [float((KP * val[0] * dx).sum()) if sname == 'seat_outer' else 0.0,
                          float((KP * un * ny * dx).sum()), float((KP * un * nz * dx).sum())]
                    rr[sname] = Fv                         # force the fork puts on the bearing (= its load)
        react[nm] = rr
        print(f'{part} {nm}: cg {info} res {res:.1e} vm max {float(np.sqrt(0.5 * ((voigt[0] - voigt[1]) ** 2 + (voigt[1] - voigt[2]) ** 2 + (voigt[2] - voigt[0]) ** 2) + 3 * (voigt[3] ** 2 + voigt[4] ** 2 + voigt[5] ** 2)).max()):.3f} MPa/N'
              f' {json.dumps({k: (np.round(v, 3).tolist() if isinstance(v, list) else round(v, 3)) for k, v in rr.items()})}', flush=True)
    cent = X[T].mean(axis=1)
    pts = mesh.p[:, np.unique(mesh.facets[:, np.concatenate(patch_facets)])].T
    dist, _ = cKDTree(pts).query(cent)
    SCRATCH.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(SCRATCH / f'pod_fe_{part}.npz', S=S, names=np.array(names), cent=cent.astype(np.float32),
                        dist=dist.astype(np.float32), vol=np.abs(np.linalg.det(np.stack([X[T[:, i]] - X[T[:, 0]] for i in (1, 2, 3)], axis=1))) / 6)
    meta = dict(part=part, H=H, nodes=int(len(X)), tets=int(len(T)), dofs=int(basis.N), names=names, reactions=react,
                seconds=round(time.time() - t0, 1))
    (VAL / f'unit_{part}.json').write_text(json.dumps(meta, indent=1))
    print('saved', part, round(time.time() - t0, 1), 's', flush=True)


if __name__ == '__main__':
    if __import__('os').environ.get('POD_FE_TRACE'):
        import faulthandler
        faulthandler.dump_traceback_later(60, repeat=True)
    mode = sys.argv[1] if len(sys.argv) > 1 else 'solve'
    if mode == 'solve':
        solve(sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 2.5)
    elif mode == 'combine':
        import fe_pod_combine
        fe_pod_combine.main()
