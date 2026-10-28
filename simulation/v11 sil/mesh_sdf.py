"""Distance to a set of watertight meshes, on a voxel grid: how close the SO-101's own meshes come to the robot's parts
as cad/v7 draws them (design.md 12.2; the reach studies' capsules miss the arm's base and shoulder bracket).

    sdf = MeshSDF([tris_a, tris_b, ...], lo, hi, h=0.001)   # triangles (n, 3, 3) m; the box lo..hi; voxel h
    sdf.clearance(points)                                     # m from each point to the nearest part (<= 0 inside)

The parts are filled by z-ray parity per (x, y) column (the meshes must be closed: build123d's STLs are), then
scipy's Euclidean distance transform gives every empty voxel's distance to the nearest filled one. Accurate to about
one voxel; points outside the box are reported far away.
"""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt


def read_stl(path, scale: float = 0.001) -> np.ndarray:
    """Triangles (n, 3, 3) of a binary or ASCII STL, times scale (the CAD's are in mm)."""
    raw = Path(path).read_bytes()
    if raw[:5] == b"solid" and b"facet" in raw[:400]:
        v = [list(map(float, ln.split()[1:4])) for ln in raw.decode("ascii", "ignore").splitlines()
             if ln.strip().startswith("vertex")]
        return np.asarray(v, float).reshape(-1, 3, 3) * scale
    n = struct.unpack_from("<I", raw, 80)[0]
    rec = np.frombuffer(raw, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), n, 84)
    return rec["v"].astype(float) * scale


class MeshSDF:
    def __init__(self, meshes, lo, hi, h: float = 0.001):
        self.lo = np.asarray(lo, float)
        self.h = h
        self.n = np.ceil((np.asarray(hi, float) - self.lo) / h).astype(int)
        occ = np.zeros(self.n, bool)
        for tris in meshes:
            self._fill(occ, np.asarray(tris, float))
        self.occ = occ
        self.dist = distance_transform_edt(~occ, sampling=h)

    def _fill(self, occ, tris):
        nx, ny, nz = self.n
        lo, h = self.lo, self.h
        # column centres, nudged off the grid lines so no ray runs exactly through an edge or a vertex
        eps = np.array([1.3e-7, 2.9e-7])
        hits = {}
        p = tris[:, :, :2]
        x0 = np.clip(np.ceil((p[:, :, 0].min(1) - lo[0]) / h - 0.5).astype(int), 0, nx)
        x1 = np.clip(np.floor((p[:, :, 0].max(1) - lo[0]) / h - 0.5).astype(int), -1, nx - 1)
        y0 = np.clip(np.ceil((p[:, :, 1].min(1) - lo[1]) / h - 0.5).astype(int), 0, ny)
        y1 = np.clip(np.floor((p[:, :, 1].max(1) - lo[1]) / h - 0.5).astype(int), -1, ny - 1)
        for k in np.nonzero((x1 >= x0) & (y1 >= y0))[0]:
            ix, iy = np.meshgrid(np.arange(x0[k], x1[k] + 1), np.arange(y0[k], y1[k] + 1), indexing="ij")
            ix, iy = ix.ravel(), iy.ravel()
            cx = lo[0] + (ix + 0.5) * h + eps[0]
            cy = lo[1] + (iy + 0.5) * h + eps[1]
            a, b, c = tris[k]
            d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(d) < 1e-18:
                continue                                  # edge-on to the rays
            l1 = ((b[1] - c[1]) * (cx - c[0]) + (c[0] - b[0]) * (cy - c[1])) / d
            l2 = ((c[1] - a[1]) * (cx - c[0]) + (a[0] - c[0]) * (cy - c[1])) / d
            l3 = 1.0 - l1 - l2
            ok = (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
            z = l1 * a[2] + l2 * b[2] + l3 * c[2]
            for i, j, zz in zip(ix[ok], iy[ok], z[ok]):
                hits.setdefault((i, j), []).append(zz)
        for (i, j), zs in hits.items():
            zs = np.sort(zs)
            for z0, z1 in zip(zs[0::2], zs[1::2]):
                k0 = max(0, int(np.ceil((z0 - lo[2]) / h - 0.5)))
                k1 = min(nz, int(np.floor((z1 - lo[2]) / h - 0.5)) + 1)
                if k1 > k0:
                    occ[i, j, k0:k1] = True

    def clearance(self, pts) -> np.ndarray:
        """Distance (m) from each point to the nearest part: <= 0 inside one; 1.0 outside the grid's box."""
        idx = np.floor((np.asarray(pts, float) - self.lo) / self.h).astype(int)
        inside = np.all((idx >= 0) & (idx < self.n), axis=1)
        out = np.full(len(idx), 1.0)
        i = idx[inside]
        d = self.dist[i[:, 0], i[:, 1], i[:, 2]]
        out[inside] = np.where(self.occ[i[:, 0], i[:, 1], i[:, 2]], -self.h / 2, d - self.h / 2)
        return out
