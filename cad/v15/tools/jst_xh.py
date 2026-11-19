"""JST XH wire-side plugs (XHP-n housings) from JST's own XHP-4 STEP (Fernando, Nov 18 2026).

    python -B jst_xh.py            (omni-robot-sim interpreter, from tools/): checks and writes step_export/jst_xh/XHP-{2,3,4,6}.step

Only the XHP-4 STEP is on hand (reference/parts/JST_XHP-4.stp, JST's CAD data: for this robot's design only, not to be passed on). The
other sizes are cut from it at the 2.5 mm pitch lines and re-stacked: its two end cells (each one contact plus the housing's end wall)
kept, its middle cell (one contact, 2.5 mm wide; the two middle cells are identical, 51.174 mm^3 each) repeated n - 2 times. Every size
then has the catalogue width B = 2.5 (n - 1) + 4.8 (reference/harness/jst_xh_catalogue_eXH.pdf p4: XHP-2 7.3, -3 9.8, -4 12.3, -6 17.3),
depth 5.7 and height 7.75 (7.5 + the 0.25 lip).

Frame of xhp(n): contacts along x, centred on x = 0; the depth (5.7) centred on y = 0; z = 0 the mating face (the lip end, into the
header), z = 7.75 the wire end (the contact cavities). Mated on a top-entry B*B-XH-A header the plug's top stands 9.8 mm over the board
(catalogue p2), so its mating face is at MATED_Z = 2.05 over the board; the wires leave its top along +z.
Which end of the XHP-4 is contact 1 is not marked in the STEP: x is not oriented to it (VERIFY on a plug if that ever matters).
"""
from functools import lru_cache
from pathlib import Path

import build123d as b

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'reference' / 'parts' / 'JST_XHP-4.stp'
OUT = ROOT / 'step_export' / 'jst_xh'
PITCH = 2.5
H = 7.75                                    # housing height, mating face to wire end
MATED_Z = 9.8 - H                           # mating face over the board when plugged into a top-entry header (catalogue: 9.8 mated)
NATURAL = (0.93, 0.92, 0.87)                # PA 6, natural (white)

# the XHP-4 STEP's own x: housing -8.549 .. 3.751, contacts at -6.149, -3.649, -1.149, 1.351 (2.4 in from each end)
_X0, _X1 = -8.549, 3.751
_CUTS = (-4.899, -2.399, 0.101)              # the pitch lines between the contacts


def catalogue_width(n):
    return PITCH * (n - 1) + 4.8


@lru_cache(maxsize=None)
def _src():
    s = b.import_step(str(SRC)).solids()
    assert len(s) == 1, f'{SRC.name}: {len(s)} solids'
    bb = s[0].bounding_box()
    assert abs(bb.size.X - 12.3) < 0.01 and abs(bb.size.Y - 5.7) < 0.01 and abs(bb.size.Z - H) < 0.01, bb.size
    return s[0]


def _slab(x0, x1):
    """the XHP-4 between x0 and x1"""
    box = b.Box(x1 - x0, 40, 40, align=(b.Align.MIN, b.Align.CENTER, b.Align.CENTER)).moved(b.Location((x0, 0, H / 2)))
    out = _src() & box
    return out.solids()[0]


@lru_cache(maxsize=None)
def xhp(n):
    """the XHP-n housing (n >= 2), in this module's frame"""
    assert n >= 2
    left = _slab(_X0 - 1, _CUTS[0])                   # end cell: end wall + the contact at -6.149
    mid = _slab(_CUTS[1], _CUTS[2])                   # middle cell: the contact at -1.149
    right = _slab(_CUTS[2], _X1 + 1)                  # end cell: the contact at 1.351 + end wall
    parts = [right]
    for k in range(n - 2):                            # middle cells toward -x from the right end cell
        parts.append(mid.moved(b.Location((-PITCH * k, 0, 0))))
    parts.append(left.moved(b.Location((-PITCH * (n - 4), 0, 0))))    # the left end cell after them (n = 4: where it was)
    body = parts[0]
    for p in parts[1:]:
        body = body.fuse(p)
    body = body.clean()
    sol = body.solids()
    assert len(sol) == 1, f'XHP-{n}: {len(sol)} solids'
    s = sol[0]
    bb = s.bounding_box()
    s = s.moved(b.Location((-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z)))
    return s


def mated(n, at=(0.0, 0.0, 0.0)):
    """the XHP-n as plugged into a top-entry header whose centre on the board's top face is `at` (board frame, z up)"""
    return xhp(n).moved(b.Location((at[0], at[1], at[2] + MATED_Z)))


HEADER_TOP = 7.0                            # a top-entry B*B-XH-A header's top over the board (catalogue p5)
IN_HEADER = HEADER_TOP - MATED_Z            # how far the mated plug sits below the header's top (4.95)


def xh_plug(p, d, n, in_header=IN_HEADER):
    """the XHP-n plugged in at a port (harness.obox's frame): p where the plug meets its device (a header's top), d the unit direction
    the plug points out (its wires leave along d), the contacts across (harness: w, horizontal; y when d is vertical), the depth on the
    third axis. The plug's mating face sits in_header behind p, its wire end 7.75 - in_header (2.8) in front of it."""
    import numpy as np
    import wheel_motor as W
    d = np.asarray(d, float) / np.linalg.norm(d)
    a = np.cross((0, 0, 1.0), d) if abs(d[2]) < 0.9 else np.array([0, 1.0, 0])
    a = a / np.linalg.norm(a)
    u = np.cross(d, a)
    M = np.zeros((3, 4))
    M[:, 0], M[:, 1], M[:, 2], M[:, 3] = a, u, d, np.asarray(p, float) - d * in_header   # plug x (contacts) -> a, y -> u, z -> d
    return xhp(n).moved(W.loc(M))


def check(sizes=(2, 3, 4, 6)):
    rows = []
    v4 = _src().volume
    mid = _slab(_CUTS[1], _CUTS[2]).volume
    for n in sizes:
        s = xhp(n)
        bb = s.bounding_box()
        want = catalogue_width(n)
        vol_want = v4 + (n - 4) * mid
        ok = s.is_valid and abs(bb.size.X - want) < 0.01 and abs(bb.size.Y - 5.7) < 0.01 and abs(bb.size.Z - H) < 0.01 \
            and abs(s.volume - vol_want) < 0.01 * vol_want
        rows.append((n, round(bb.size.X, 3), want, round(bb.size.Y, 3), round(bb.size.Z, 3), round(s.volume, 2), round(vol_want, 2), ok))
    return rows


def main():
    import step_colour as SC
    OUT.mkdir(parents=True, exist_ok=True)
    rows = check()
    print('n  width  catalogue  depth  height  volume  expected  ok')
    for r in rows:
        print('  '.join(str(x) for x in r))
    assert all(r[-1] for r in rows), 'a plug failed its check'
    for n in (2, 3, 4, 6):
        SC.write_part(xhp(n), NATURAL, OUT / f'XHP-{n}.step', f'XHP-{n}')
    print(f'-> {OUT}')


if __name__ == '__main__':
    main()
