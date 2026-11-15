"""cad/v13 (Nov 11 whole-CAD review; venv from requirements-fe.txt): stiffness of one bumper quarter (3 mm PETG shell on 4 tabs): radial push on the outer wall, deflection per newton and peak von Mises.
Linear elastic, P2 tets, tab screws modelled as fixed discs. Material E / nu are assumptions (printed PETG)."""
import sys, json, math
import numpy as np
import gmsh
from skfem import *
from skfem.helpers import dot, sym_grad, ddot, eye, trace
from scipy.sparse.linalg import cg
import pyamg

import os
from pathlib import Path
STEP = os.environ.get('BUMPER_STEP', str(Path(__file__).resolve().parents[1] / 'step_export' / 'chassis' / 'parts' / 'bumper_q1.step'))   # BUMPER_STEP: another variant's STEP
E, NU = 2000.0, 0.38                       # MPa, PETG (assumed; PLA ~3000)
SCREWS = [(-24.3, 215.5), (-68.6, 206.0), (-211.9, 52.4), (-216.2, 30.2)]
H = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
LOAD_ANGLE = float(sys.argv[2]) if len(sys.argv) > 2 else 137.3
LOAD_Z = float(sys.argv[3]) if len(sys.argv) > 3 else 65.0
F = 10.0                                   # N, total radial push (inward)

gmsh.initialize()
gmsh.option.setNumber('General.Terminal', 0)
gmsh.model.occ.importShapes(STEP)
gmsh.model.occ.synchronize()
gmsh.option.setNumber('Mesh.MeshSizeMax', H)
gmsh.option.setNumber('Mesh.MeshSizeMin', 0.6)
gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 12)
gmsh.option.setNumber('Mesh.Algorithm3D', 1)
gmsh.model.mesh.generate(3)
nt, xyz, _ = gmsh.model.mesh.getNodes()
tags = np.array(nt, dtype=np.int64)
X = np.array(xyz).reshape(-1, 3)
idx = {t: i for i, t in enumerate(tags)}
et, en = gmsh.model.mesh.getElementsByType(4)
T = np.array([[idx[n] for n in row] for row in np.array(en, dtype=np.int64).reshape(-1, 4)])
gmsh.finalize()
print('nodes', len(X), 'tets', len(T), flush=True)

mesh = MeshTet(X.T.copy(), T.T.copy())
elem = ElementVector(ElementTetP2())
basis = Basis(mesh, elem)
lam = E * NU / ((1 + NU) * (1 - 2 * NU))
mu = E / (2 * (1 + NU))


@BilinearForm
def stiff(u, v, w):
    return lam * trace(sym_grad(u)) * trace(sym_grad(v)) + 2 * mu * ddot(sym_grad(u), sym_grad(v))


K = stiff.assemble(basis)


def fixed(x):
    m = np.zeros(x.shape[1], bool)
    for sx, sy in SCREWS:
        m |= (np.hypot(x[0] - sx, x[1] - sy) < 5.5) & (x[2] < 45.2)
    return m


D = basis.get_dofs(lambda x: fixed(x)).all()
# load: pressure patch on the outer wall (r >= 224.9) round (LOAD_ANGLE, LOAD_Z), radius 10 mm
a = math.radians(LOAD_ANGLE)
c = np.array([225.15 * math.cos(a), 225.15 * math.sin(a), LOAD_Z])


def on_patch(x):
    r = np.hypot(x[0], x[1])
    return (r > 224.9) & (np.linalg.norm(x - c[:, None], axis=0) < 10.0)


facets = mesh.facets_satisfying(on_patch)
fb = FacetBasis(mesh, elem, facets=facets)
area = float(fb.dx.sum()) if hasattr(fb, 'dx') else None
Ap = float(np.sum(fb.dx))
print('patch area', Ap, 'facets', len(facets), flush=True)


@LinearForm
def load(v, w):
    x = w.x
    r = np.sqrt(x[0] ** 2 + x[1] ** 2)
    n = np.array([-x[0] / r, -x[1] / r, 0 * r])        # inward
    return (F / Ap) * (n[0] * v[0] + n[1] * v[1] + n[2] * v[2])


f = load.assemble(fb)
free = np.setdiff1d(np.arange(K.shape[0]), D)
Kf = K[free][:, free].tocsr()
ml = pyamg.smoothed_aggregation_solver(Kf, B=None, max_coarse=500)
M = ml.aspreconditioner(cycle='V')
uf, info = cg(Kf, f[free], M=M, rtol=1e-7, maxiter=4000)
print('cg info', info, 'rel residual', float(np.linalg.norm(Kf @ uf - f[free]) / np.linalg.norm(f[free])), flush=True)
u = np.zeros(K.shape[0])
u[free] = uf
# displacement at the load point: average of inward radial displacement over the patch nodes (P1 dofs)
ux = u[basis.nodal_dofs]                                 # (3, nnodes) for the P1 vertices
r = np.hypot(X[:, 0], X[:, 1])
inward = -(ux[0] * X[:, 0] + ux[1] * X[:, 1]) / r
patch = (r > 224.9) & (np.linalg.norm(X - c, axis=1) < 10.0)
d_patch = float(inward[patch].mean())
d_max = float(inward.max())
# von Mises at quadrature points
G = basis.interpolate(u).grad                            # (3,3,nel,nqp)
eps = 0.5 * (G + np.swapaxes(G, 0, 1))
tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
sig = 2 * mu * eps + lam * tr * np.eye(3)[:, :, None, None]
vm = np.sqrt(0.5 * ((sig[0, 0] - sig[1, 1]) ** 2 + (sig[1, 1] - sig[2, 2]) ** 2 + (sig[2, 2] - sig[0, 0]) ** 2) + 3 * (sig[0, 1] ** 2 + sig[1, 2] ** 2 + sig[0, 2] ** 2))
vme = vm.max(axis=1)
cent = X[T].mean(axis=1)
order = np.argsort(-vme)[:60]
# cluster top elements by location (round to 5 mm)
seen = {}
for i in order:
    k = tuple(np.round(cent[i] / 6).astype(int))
    if k not in seen:
        seen[k] = (round(float(vme[i]), 2), [round(float(v), 1) for v in cent[i]])
top = sorted(seen.values(), key=lambda t: -t[0])[:6]
res = dict(h=H, load_angle=LOAD_ANGLE, load_z=LOAD_Z, F=F, E=E, patch_area=Ap, defl_patch_mm=round(d_patch, 3), defl_mm_per_N=round(d_patch / F, 4),
           max_inward_mm=round(d_max, 3), vm_max_MPa=round(float(vme.max()), 2), top_vm=top)
print(json.dumps(res, indent=1))
json.dump(res, open(sys.argv[4] if len(sys.argv) > 4 else 'fe_bumper_out.json', 'w'), indent=1)
