"""cad/v13: stress in a wheel-hub spoke under the tangential drive force on a roller's thrust journal (Nov 11 whole-CAD review).

Linear elastic P2 tets (gmsh + scikit-fem + pyamg) on a sector of the real hub from hub_sector.py: the far x plane and the two azimuth cuts fixed (the drum shell),
the journal annulus of the roller that pushes on the spoke (r 1.7..4 about its axis, on its thrust face z = +-ZF) loaded with F, tangential to the wheel, uniform
('full') or only on its outer / inner half ('outer' / 'inner': the drive force acts at the tread, so the real pressure leans outward). E 2 GPa, nu 0.38 (PETG, assumed).

    python -B fe_hub.py SECTOR.STEP ROW AZ H_FINE [full|outer|inner] [F_N=58.6] [OUT.json]       (a venv from requirements-fe.txt, not the CAD interpreter)

Prints and saves the peak von Mises away from the load patch, the top clusters (position, r, azimuth), the peak in a r 38-46 band round the spoke, the peak
sigma_xx (across the layers: the hub prints x up) and the first principal stress. Results of the Nov 11 runs: validation/wheel_motor/hub_fe/.
"""
import sys, json, math, time
import numpy as np
import gmsh
from skfem import *
from skfem.helpers import sym_grad, ddot, trace
from scipy.sparse.linalg import cg
import pyamg

STEP, ROW, AZ, HF = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
PATTERN = sys.argv[5] if len(sys.argv) > 5 else 'full'
F = float(sys.argv[6]) if len(sys.argv) > 6 else 58.6
OUT = sys.argv[7] if len(sys.argv) > 7 else None
HALF = 35.0
E, NU = 2000.0, 0.38                       # MPa, PETG (assumed; stress barely depends on it)
D_ROW, R_AXIS, ZF, JR, N = 17.25, 52.0, 20.55, 4.0, 6


def frames():
    out = []
    x = -D_ROW if ROW == 1 else D_ROW
    for k in range(1, N + 1):
        th = math.radians(60 * (k - 1) + (0 if ROW == 1 else 30))
        c, s = math.cos(th), math.sin(th)
        M = np.eye(4)
        M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = [1, 0, 0], [0, c, s], [0, -s, c], [x, R_AXIS * c, R_AXIS * s]
        if k % 2 == 0:
            M = M @ np.diag([-1.0, 1.0, -1.0, 1.0])
        out.append((k, math.degrees(th), M))
    return out


# the roller face that pushes on the spoke at AZ: nearest face centre in azimuth
best = None
for k, th, M in frames():
    for s in (1, -1):
        p = M @ np.array([0, 0, s * ZF, 1.0])
        a = math.degrees(math.atan2(p[2], p[1]))
        d = abs((a - AZ + 180) % 360 - 180)
        if best is None or d < best[0]:
            best = (d, k, th, s, M)
_, K, TH, S, M = best
Minv = np.linalg.inv(M)
ZG = M[:3, 2] * S                           # global direction of the push on the spoke (from the roller toward the face)
print(f'row {ROW} spoke az {AZ}: roller k={K} (th {TH}) face s={S}, push direction {np.round(ZG, 3)}', flush=True)

a = math.radians(AZ)
xc = D_ROW if ROW == 2 else -D_ROW


def mesh_once(a2, a3, mx):
    """a fresh gmsh session per attempt (a failed PLC leaves the session unusable); fine ball on the spoke, a little inside the roller axis radius (the neck is at r 42)"""
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    gmsh.model.occ.importShapes(STEP)
    gmsh.model.occ.synchronize()
    ball = gmsh.model.mesh.field.add('Ball')
    for k, v in dict(VIn=HF, VOut=mx, XCenter=xc, YCenter=47.0 * math.cos(a), ZCenter=47.0 * math.sin(a), Radius=13.0, Thickness=14.0).items():
        gmsh.model.mesh.field.setNumber(ball, k, v)
    gmsh.model.mesh.field.setAsBackgroundMesh(ball)
    for k, v in (('Mesh.MeshSizeFromPoints', 0), ('Mesh.MeshSizeExtendFromBoundary', 0), ('Mesh.MeshSizeFromCurvature', 0), ('Mesh.MeshSizeMax', mx),
                 ('Mesh.Algorithm', a2), ('Mesh.Algorithm3D', a3)):
        gmsh.option.setNumber(k, v)
    try:
        gmsh.model.mesh.generate(3)
        et, en = gmsh.model.mesh.getElementsByType(4)
        if len(et) == 0:
            raise RuntimeError('empty mesh')
        nt, xyz, _ = gmsh.model.mesh.getNodes()
        tags = np.array(nt, dtype=np.int64)
        X = np.array(xyz).reshape(-1, 3)
        idx = {t: i for i, t in enumerate(tags)}
        T = np.array([[idx[n] for n in row] for row in np.array(en, dtype=np.int64).reshape(-1, 4)])
        return X, T
    finally:
        gmsh.finalize()


t0 = time.time()
X = None
for a2, a3, mx in ((6, 1, 3.2), (1, 10, 3.2), (1, 10, 2.4)):          # Delaunay first; MeshAdapt + HXT meshes the head-pocket spokes
    try:
        X, T = mesh_once(a2, a3, mx)
        print('mesher ok: 2D', a2, '3D', a3, 'max', mx, flush=True)
        break
    except Exception as ex:                    # noqa: BLE001
        print('mesher failed: 2D', a2, '3D', a3, 'max', mx, '-', str(ex)[:70], flush=True)
if X is None:
    raise SystemExit('no mesher setting worked')
print('nodes', len(X), 'tets', len(T), 'mesh s', round(time.time() - t0, 1), flush=True)

mesh = MeshTet(X.T.copy(), T.T.copy())
elem = ElementVector(ElementTetP2())
basis = Basis(mesh, elem)
lam = E * NU / ((1 + NU) * (1 - 2 * NU))
mu = E / (2 * (1 + NU))


@BilinearForm
def stiff(u, v, w):
    return lam * trace(sym_grad(u)) * trace(sym_grad(v)) + 2 * mu * ddot(sym_grad(u), sym_grad(v))


t0 = time.time()
K = stiff.assemble(basis)
print('assembled', K.shape, 'nnz', K.nnz, 's', round(time.time() - t0, 1), flush=True)

xfix = -9.0 if ROW == 2 else -30.0
a0, a1 = math.radians(AZ - HALF), math.radians(AZ + HALF)


def fixed(x):
    az = np.arctan2(x[2], x[1])
    d = (az - a) % (2 * math.pi)
    d = np.where(d > math.pi, d - 2 * math.pi, d)                       # signed azimuth from the spoke
    # on a cut plane: the plane through the axis at the sector's edge (distance to it ~0), or the far x plane
    r = np.hypot(x[1], x[2])
    on_edge = (np.abs(np.abs(d) - math.radians(HALF)) < math.radians(0.15)) & (r > 25.0)
    return on_edge | (np.abs(x[0] - xfix) < 1e-3)


D = basis.get_dofs(lambda x: fixed(x)).all()
print('fixed dofs', len(D), flush=True)


def on_patch(x):
    p = np.vstack([x, np.ones(x.shape[1])])
    L = Minv @ p
    rho = np.hypot(L[0], L[1])
    ok = (np.abs(L[2] - S * ZF) < 0.06) & (rho < JR + 0.15)
    if PATTERN == 'outer':
        ok &= L[1] > 0
    elif PATTERN == 'inner':
        ok &= L[1] < 0
    return ok


facets = mesh.facets_satisfying(on_patch)
fb = FacetBasis(mesh, elem, facets=facets)
Ap = float(np.sum(fb.dx))
print('patch facets', len(facets), 'area', round(Ap, 1), 'of', round(math.pi * (JR ** 2 - 1.7 ** 2), 1), flush=True)
dirn = ZG


@LinearForm
def load(v, w):
    return (F / Ap) * (dirn[0] * v[0] + dirn[1] * v[1] + dirn[2] * v[2])


f = load.assemble(fb)
free = np.setdiff1d(np.arange(K.shape[0]), D)
Kf = K[free][:, free].tocsr()
t0 = time.time()
ml = pyamg.smoothed_aggregation_solver(Kf, B=None, max_coarse=500)
M_ = ml.aspreconditioner(cycle='V')
uf, info = cg(Kf, f[free], M=M_, rtol=1e-7, maxiter=3000)
res = float(np.linalg.norm(Kf @ uf - f[free]) / np.linalg.norm(f[free]))
print('cg info', info, 'rel residual', res, 's', round(time.time() - t0, 1), flush=True)
u = np.zeros(K.shape[0])
u[free] = uf
G = basis.interpolate(u).grad
eps = 0.5 * (G + np.swapaxes(G, 0, 1))
tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
sig = 2 * mu * eps + lam * tr * np.eye(3)[:, :, None, None]
vm = np.sqrt(0.5 * ((sig[0, 0] - sig[1, 1]) ** 2 + (sig[1, 1] - sig[2, 2]) ** 2 + (sig[2, 2] - sig[0, 0]) ** 2) + 3 * (sig[0, 1] ** 2 + sig[1, 2] ** 2 + sig[0, 2] ** 2))
vme = vm.max(axis=1)
cent = X[T].mean(axis=1)
L = np.linalg.inv(M) @ np.vstack([cent.T, np.ones(len(cent))])      # centroids in the loaded roller's frame
near_patch = (np.hypot(L[0], L[1]) < JR + 3.0) & (np.abs(L[2] - S * ZF) < 3.0)
# principal stresses and sigma_xx (across the layers: the hub prints x up) per element max over qp
sxx = sig[0, 0].max(axis=1)
cand = np.argsort(-vme)[:400]
mp = np.array([max(np.linalg.eigvalsh(sig[:, :, e, q])[-1] for q in range(sig.shape[3])) for e in cand])
ur = np.hypot(cent[:, 1], cent[:, 2])
ang = np.degrees(np.arctan2(cent[:, 2], cent[:, 1]))


def clusters(vals, mask, n=6, cell=4.0):
    order = np.argsort(-np.where(mask, vals, -1))[:120]
    seen = {}
    for i in order:
        if not mask[i]:
            continue
        kk = tuple(np.round(cent[i] / cell).astype(int))
        if kk not in seen:
            seen[kk] = (round(float(vals[i]), 2), [round(float(v), 1) for v in cent[i]], dict(r=round(float(ur[i]), 1), az=round(float(ang[i]), 1)))
    return sorted(seen.values(), key=lambda t: -t[0])[:n]


neck = (ur > 38.0) & (ur < 46.0) & (np.abs(((ang - AZ + 180) % 360) - 180) < 8.0)
out = dict(row=ROW, az=AZ, pattern=PATTERN, F=F, E=E, H_fine=HF, tets=len(T), nodes=int(len(X)), cg_residual=res, patch_area=Ap,
           vm_max_all=round(float(vme.max()), 2), vm_max_away_from_patch=round(float(vme[~near_patch].max()), 2),
           vm_top_away=clusters(vme, ~near_patch), vm_top_neck_band_r38_46=clusters(vme, neck, 3),
           sigma_xx_max_across_layers=round(float(sxx[~near_patch].max()), 2), sigma_xx_min=round(float(sig[0, 0].min(axis=1)[~near_patch].min()), 2),
           first_principal_top_among_top400_vm=round(float(mp.max()), 2),
           push_dir=[round(float(v), 3) for v in ZG])
print(json.dumps(out, indent=1))
if OUT:
    json.dump(out, open(OUT, 'w'), indent=1)
