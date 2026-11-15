"""One picture with the three Pod_reimagined proposals side by side (their renders), cad/v14/Pod_reimagined_compare.png.   python -B pod_rx_compare.py"""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                         # noqa: E402
import matplotlib.image as mpimg                                        # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
VIEWS = (('iso_motor_side.png', 'motor side'), ('iso_outer_side.png', 'outer side'))
BACK = {1: 'iso_frame_back.png', 2: 'iso_back.png', 3: 'iso_frames_back.png'}
TITLES = {1: '1  COMPACT: closed fork frame, one block, springs behind the pivot',
          2: '2  MEDIUM: one block for the pivots, C-frames carry today\'s front springs',
          3: '3  RELAXED: outboard side frames + tie, front springs (classic)'}
fig, axs = plt.subplots(3, 3, figsize=(21, 17))
for r, i in enumerate((1, 2, 3)):
    for c, fn in enumerate([v[0] for v in VIEWS] + [BACK[i]]):
        ax = axs[r][c]
        ax.axis('off')
        f = ROOT / f'Pod_reimagined_{i}' / fn
        if f.exists():
            img = mpimg.imread(f)
            h, w = img.shape[:2]
            ax.imshow(img[int(h * 0.06):, :])
        if c == 0:
            ax.set_title(TITLES[i], loc='left', fontsize=13)
fig.tight_layout()
fig.savefig(ROOT / 'Pod_reimagined_compare.png', dpi=70)
print('written', ROOT / 'Pod_reimagined_compare.png')
