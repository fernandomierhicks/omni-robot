"""Plan and side views of one pod (pod frame) with its deck attachment points marked: validation/pod_attachment_points.png.

    python -B 61_pod_attachment_figure.py       (omni-robot-sim interpreter; build123d, matplotlib)
Guard from validation/human/wheel_guard.step (the lowered guard), forks and tower from reference/, wheel as a cylinder envelope, attachment
points from validation/master_layout.json (Tower_feet; M3 pilots in the feet's underside at z 3).
"""
import json
from pathlib import Path
import numpy as np
import build123d as b
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.colors import to_rgb

ROOT = Path(__file__).resolve().parents[2]
layout = json.loads((ROOT / 'validation' / 'master_layout.json').read_text())['patterns']
parts = [(ROOT / 'validation/human/wheel_guard.step', '#7fa5a8'), (ROOT / 'reference/pod_mount.step', '#6cb36c'),
         (ROOT / 'reference/inner_fork_6804.step', '#d88a3a'), (ROOT / 'reference/outer_fork_6804.step', '#c9a63a'),
         (None, '#444444')]
meshes = []
for p, col in parts:
    s = b.import_step(p) if p else b.Cylinder(70, 74).rotate(b.Axis.Y, 90)
    v, t = s.tessellate(0.4, 0.3)
    meshes.append((np.array([[q.X, q.Y, q.Z] for q in v]), np.array(t), col))


def view(ax, u, v_, d, flip=1):
    polys = []
    for V, T, col in meshes:
        for t in V[T]:
            n = np.cross(t[1] - t[0], t[2] - t[0]); nn = np.linalg.norm(n)
            if nn:
                polys.append((flip * t[:, d].mean(), t[:, [u, v_]], tuple(np.array(to_rgb(col)) * (0.55 + 0.45 * abs(n[d] / nn)))))
    polys.sort(key=lambda p: p[0])
    ax.add_collection(PolyCollection([p[1] for p in polys], facecolors=[p[2] for p in polys], edgecolors='none'))
    ax.set_aspect('equal'); ax.autoscale(); ax.grid(True, lw=.3)


fig, axs = plt.subplots(1, 2, figsize=(15, 7))
view(axs[0], 0, 1, 2)
# the guard feet as drawn in the guard (12_master.py / 56_level_pivot_deck.py FEET_NEW): the stale validation/master_layout.json still has the pre-level-pivot pivot-side feet
GUARD = [(-14, -87.5), (16, -87.5), (-31.5, 94.5), (-14, 94.5), (16, 94.5), (30, 94.5)]
pts = GUARD + layout['Tower_feet']['points']
axs[0].scatter([p[0] for p in pts], [p[1] for p in pts], c='red', s=40, zorder=5, label='M3 deck screws (11)')
axs[0].set(title='Pod plan (pod frame: x radial inward, y toward the pivot side)', xlabel='x mm', ylabel='y mm'); axs[0].legend()
view(axs[1], 0, 2, 1)
axs[1].axhline(3, color='red', lw=1); axs[1].text(-70, 5, 'deck-top plane z 3 (feet underside)', color='red')
axs[1].set(title='Pod side view', xlabel='x mm', ylabel='z mm')
fig.tight_layout(); fig.savefig(ROOT / 'validation' / 'pod_attachment_points.png', dpi=100)
print('wrote validation/pod_attachment_points.png', len(pts), 'points')
