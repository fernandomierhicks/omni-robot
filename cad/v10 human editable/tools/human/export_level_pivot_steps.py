"""STEP exports of the 3D PRINTED parts the Nov 2 level fork pivot changed (no fasteners, washers, nuts, bearings, shafts or the
aluminium fork bar). Run in the cadenv Python.

    cadenv python export_level_pivot_steps.py

step_export/level_pivot/parts/: the native pod parts (right hand, for FR / RL) from SolidWorks, their mirror images
(left hand, for FL / RR) from assemblies/mirror/, and the robot's cut deck pieces, deck bridge and re-holed TPU corner
bumper (build123d parts from 56_level_pivot_deck.py, copied as written).
"""
import shutil
from common import *

OUTD = ROOT / 'step_export' / 'level_pivot' / 'parts'
OUTD.mkdir(parents=True, exist_ok=True)
NATIVE = ['inner_fork_6804', 'outer_fork_6804', 'wheel_guard', 'pod_mount', 'bench_plate']
MIRRORED = ['inner_fork_6804', 'outer_fork_6804', 'wheel_guard', 'pod_mount']           # the bench plate is a single fixture
ROBOT = ['REC_deck_left_outer_pivot', 'REC_deck_left_inner_pivot', 'REC_deck_right_outer_pivot', 'REC_deck_right_inner_pivot',
         'REC_deck_bridge', 'REC_bumper_corner_pivot']


def save_step(sw, m, path):
    sw.ActivateDoc3(m.GetTitle(), False, 0, 0)
    ok, e, w = m.Extension.SaveAs3(str(path), 0, 3, None, None, 0, 0)
    if not ok or e:
        raise RuntimeError('STEP export failed ' + str(path) + ' ' + str((e, w)))


def main():
    sw = connect()
    out = []
    for key in NATIVE:
        m = open_owned(sw, key)
        save_step(sw, m, OUTD / ('V10H_' + key + '.step'))
        sw.CloseDoc(m.GetTitle())
        out.append(key)
    for key in MIRRORED:
        path = ROOT / 'assemblies' / 'mirror' / ('v10h_' + key + '-mirror.SLDPRT')
        raw, e, w = sw.OpenDoc6(str(path), 1, 1, '', 0, 0)
        if raw is None:
            raise RuntimeError('cannot open ' + str(path))
        m = wrap(raw, sld.IModelDoc2)
        save_step(sw, m, OUTD / ('V10H_' + key + '_left_hand.step'))
        sw.CloseDoc(m.GetTitle())
        out.append(key + ' (left hand)')
    for stem in ROBOT:
        shutil.copy2(ROOT / 'step_parts' / (stem + '.step'), OUTD / (stem + '.step'))
        out.append(stem)
    print('exported', out, flush=True)


if __name__ == '__main__':
    main()
