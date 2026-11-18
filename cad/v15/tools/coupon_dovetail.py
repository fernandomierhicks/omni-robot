"""cad/v15 dovetail fit coupon (Nov 15 evening, Fernando: the deck's dovetail laps are far too loose at 0.1; try 0.05 and 0). build123d; a bench
print, not part of the robot.

    python -B coupon_dovetail.py [build|check|export|render|all]        (omni-robot-sim interpreter)

Two bars, printed flat on the bed as the deck pieces are (the tab and the socket are both in the deck's bottom half, so both start at the bed,
elephant's foot and all):
 * TAB: one male dovetail of the deck's own shape (chassis_mid.TAB: 16 wide at the root, 22 at the end, 20 deep, sharp corners as drawn in
   the deck) on a short bar, CDT_T thick;
 * SOCKETS: a bar with two sockets cut as the deck cuts them (the tab grown by the clearance per face, mitred corners): 0.05 and 0 per face
   (Fernando), engraved on the bar's top beside each socket. The bars meet along the seam with no gap, as the deck pieces now do.
Slide the tab into each socket from above (as the upper deck piece drops onto the lower one). Pick the tightest that goes together by hand
without forcing and does not rock; put it into chassis_mid.TAB['clr'] (and KEY['clr']). Print in the deck's filament and settings.
Dimensions: dims.CDT_*, the tab's shape from chassis_mid.TAB. Printed Nov 17 (Fernando): 0.05 fits best, now the deck's clearance. Since
then the tab's corners and the sockets' lips carry the deck's R0.5 fillets (the printed coupon had sharp corners).
"""
import json
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely.geometry import Polygon, box as sbox

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import palette as PAL                                                  # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import CDT_CLR, CDT_T, CDT_WALL, CDT_BACK, CDT_LABEL         # noqa: E402
from step_colour import write_part                                     # noqa: E402

OUT = W.ROOT / 'step_export' / 'coupon_dovetail'
VAL = W.ROOT / 'validation' / 'coupon_dovetail'
TAB = dict(neck=8.0, flare=11.0, depth=13.0, root=1.0, r=0.5)          # chassis_mid.TAB's shape (Nov 16: depth 20 -> 13, the bed; Nov 17: R0.5 on the tab's
                                                                       # corners and the sockets' lips) (repeated: importing chassis_mid builds nothing,
                                                                       # but pulls the whole pod; check() compares them)
PITCH = 2 * TAB['flare'] + CDT_WALL + 2 * max(CDT_CLR)
N = len(CDT_CLR)
L_BAR = N * PITCH + CDT_WALL
X0 = -L_BAR / 2
GAP = 2.0                                                              # the two bars side by side on the bed (as printed)


def tab_poly(x, clr=0.0):
    """the deck's tab (chassis_mid.tab_poly, its seam on y = 0, reaching +y), centred on x, grown by clr per face (mitred, as the deck's socket)"""
    T = TAB
    p = Polygon([(x - T['neck'], -T['root']), (x + T['neck'], -T['root']), (x + T['flare'], T['depth']), (x - T['flare'], T['depth'])])
    p = round_convex(p, T['r'])
    return p.buffer(clr, join_style=2) if clr else p


def round_convex(g, r, where=None):
    """as chassis_mid.round_convex: g's outside corners rounded to r, everywhere or only inside `where`"""
    if r <= 0:
        return g
    o = g.buffer(-r, 16, join_style=1).buffer(r, 16, join_style=1)
    return o if where is None else g.difference(where.difference(o))


def cell_x(n):
    return X0 + CDT_WALL + (n - 0.5) * PITCH - CDT_WALL / 2


def build():
    """one male tab on a short bar (its seam edge at y = 0, the tab reaching +y, centred on cell 1) and the sockets bar (its seam edge at y = 0,
    the bar over y 0 .. depth + back, one socket per CDT_CLR); try the male in each socket. For printing they lie side by side (export)"""
    x1 = cell_x(1)
    tabs = sbox(x1 - PITCH / 2, -CDT_BACK, x1 + PITCH / 2, 0.0).union(tab_poly(x1))
    sock = sbox(X0, 0.0, X0 + L_BAR, TAB['depth'] + CDT_BACK)
    for n in range(1, N + 1):
        cut = tab_poly(cell_x(n), CDT_CLR[n - 1])
        sock = round_convex(sock.difference(cut), TAB['r'], cut.buffer(3 * TAB['r']).intersection(sbox(X0 - 1, -3 * TAB['r'], X0 + L_BAR + 1, 3 * TAB['r'])))
    t = B.solid_of(tabs.buffer(0), 0.0, CDT_T)
    s = B.solid_of(sock.buffer(0), 0.0, CDT_T)
    for n in range(1, N + 1):                                          # the clearance engraved on the socket bar's top, beside its socket
        txt = f'{CDT_CLR[n - 1]:.2f}' if CDT_CLR[n - 1] else '0'
        pl = b.Plane(origin=(cell_x(n), TAB['depth'] + CDT_BACK / 2, CDT_T + 0.1), x_dir=(1, 0, 0), z_dir=(0, 0, 1))
        s = s - b.extrude(pl * b.Text(txt, font_size=CDT_LABEL[0], align=(b.Align.CENTER, b.Align.CENTER)), amount=CDT_LABEL[1] + 0.1, dir=(0, 0, -1))
    t.label, s.label = 'coupon_dovetail_tabs', 'coupon_dovetail_sockets'
    return dict(coupon_dovetail_tabs=t, coupon_dovetail_sockets=s)


def check(shapes):
    import chassis_mid as CM                                            # the deck's own tab, to compare
    t, s = shapes['coupon_dovetail_tabs'], shapes['coupon_dovetail_sockets']
    rep = dict(tab_shape_matches_deck=all(abs(TAB[k] - CM.TAB[k]) < 1e-9 for k in TAB), deck_clr_today=CM.TAB['clr'],
               parts={k: dict(solids=len(v.solids()), valid=bool(v.is_valid), volume_cm3=round(v.volume / 1000, 2),
                              bbox=[round(q, 1) for q in v.bounding_box().size]) for k, v in shapes.items()}, cells={})
    for n in range(1, N + 1):
        x = cell_x(n)
        tn = t.moved(b.Location((x - cell_x(1), 0, 0)))                 # the male, in socket n
        win = b.Box(PITCH, TAB['depth'] + 4, CDT_T + 2).moved(b.Location((x, TAB['depth'] / 2, CDT_T / 2)))
        tc, sc = tn & win, s & win
        t_ = tn
        flank = b.Box(PITCH, TAB['depth'] - 2.0, CDT_T + 2).moved(b.Location((x, TAB['depth'] / 2, CDT_T / 2)))   # off the seam (the bars meet
        rep['cells'][n] = dict(clearance_per_face=CDT_CLR[n - 1], overlap_mm3=round(W.overlap(tc, sc), 4),        # there, as the deck's pieces)
                               flank_gap_measured=round((t_ & flank).distance_to(s & flank), 3), seam_contact=bool(tc.distance_to(sc) < 1e-6),
                               wall_beside_socket=round(PITCH - 2 * (TAB['flare'] + CDT_CLR[n - 1]), 2))
    rep['fits_bed'] = max(L_BAR, 2 * (TAB['depth'] + CDT_BACK) + GAP) <= 256
    rep['how'] = 'slide each tab into its socket from above; pick the tightest that goes in by hand and does not rock'
    return rep


def export(shapes):
    """each bar alone (as printed: flat, bed at z 0), and both side by side on one bed"""
    OUT.mkdir(parents=True, exist_ok=True)
    rgb = PAL.colour('deck_FL') or (0.86, 0.86, 0.82)
    for k, v in shapes.items():
        write_part(v, rgb, OUT / f'{k}.step', k)
    plate = b.Compound([shapes['coupon_dovetail_tabs'].moved(b.Location((0, -GAP / 2 - (TAB['depth'] + 0.0), 0))),
                        shapes['coupon_dovetail_sockets'].moved(b.Location((0, GAP / 2 + CDT_BACK, 0)))])
    write_part(plate, rgb, OUT / 'coupon_dovetail_bed.step', 'coupon_dovetail_bed')


def render(shapes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 4.5))
    for k, col in (('coupon_dovetail_tabs', (0.85, 0.55, 0.15)), ('coupon_dovetail_sockets', (0.55, 0.6, 0.65))):
        g = B.proj(shapes[k], True, CDT_T / 2 - 0.1, CDT_T / 2 + 0.1)
        for q in getattr(g, 'geoms', [g]):
            ax.fill(*q.exterior.xy, color=col, alpha=0.8)
            for h in q.interiors:
                ax.fill(*h.xy, color='w')
    for n in range(1, N + 1):
        ax.annotate(f'clr {CDT_CLR[n - 1]:g}', (cell_x(n), TAB['depth'] + CDT_BACK + 2), ha='center', fontsize=9)
    ax.set_aspect('equal')
    ax.grid(lw=0.3)
    ax.set_title('dovetail coupon, mated (orange: tabs bar, grey: sockets bar); print both flat', fontsize=10)
    VAL.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(VAL / 'coupon_dovetail_plan.png', dpi=110)
    plt.close(fig)


def main(stage):
    shapes = build()
    for k, v in shapes.items():
        print(k, len(v.solids()), v.is_valid, round(v.volume / 1000, 2), [round(q, 1) for q in v.bounding_box().size], flush=True)
    if stage in ('check', 'all'):
        VAL.mkdir(parents=True, exist_ok=True)
        rep = check(shapes)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps(rep, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(shapes)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(shapes)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'all')
