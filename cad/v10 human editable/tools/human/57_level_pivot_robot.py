"""Nov 2 2026 level fork pivot, robot level: bring 56_level_pivot_deck.py's parts into SolidWorks and rebuild what uses them.

    cadenv python -B 57_level_pivot_robot.py import|deck|bumper|fasteners|robot      (one stage per SolidWorks session)

import     the new STEP parts (cut deck pieces, deck bridge, re-holed corner bumper) -> parts/step/*.SLDPRT
deck / bumper / fasteners   rebuild V10H_base_deck / _bumper / _fasteners from reference/robot_sim_instances.json
robot [names]   open each robot assembly (default all four), rebuild, report mate errors, save (pods and base are referenced files)
"""
import importlib
import sys
from subasm import *

RG = importlib.import_module('40_robot_subassemblies')
NEW = ['REC_deck_left_outer_pivot', 'REC_deck_left_inner_pivot', 'REC_deck_right_outer_pivot', 'REC_deck_right_inner_pivot',
       'REC_deck_bridge', 'REC_bumper_corner_pivot']
ROBOTS = ['V10H_robot.SLDASM', 'V10H_robot_FOV.SLDASM', 'V10H_robot_FOV_side.SLDASM', 'V10H_robot_floor_reach.SLDASM']
REPORT = ROOT / 'validation' / 'level_pivot_robot_solidworks.json'


def log(key, value):
    rep = json.loads(REPORT.read_text()) if REPORT.exists() else {}
    rep[key] = value
    REPORT.write_text(json.dumps(rep, indent=2))


def import_parts(sw):
    mod = importlib.import_module('32_import_steps')
    out = {n: mod.import_one(sw, n) for n in NEW}
    print(out, flush=True)
    log('import', out)


def group(sw, name):
    {'deck': lambda: RG.fixed_assembly(sw, 'base_deck', RG.pick('base', False, lambda i: i['product'].startswith(RG.DECK))),
     'bumper': lambda: RG.fixed_assembly(sw, 'base_bumper', RG.pick('base', False, lambda i: i['product'].startswith(RG.BUMPER))),
     'fasteners': lambda: RG.fixed_assembly(sw, 'base_fasteners', RG.pick('base', True))}[name]()
    log('base_' + name, 'rebuilt')


def robots(sw):
    out = {}
    for name in ROBOTS:
        path = ROOT / name
        raw, e, w = sw.OpenDoc6(str(path), 2, 1, '', 0, 0)
        if raw is None:
            out[name] = f'cannot open {(e, w)}'
            continue
        m = wrap(raw, sld.IModelDoc2)
        sw.ActivateDoc3(m.GetTitle(), False, 0, 0)
        m.ForceRebuild3(False)
        errors = M.mate_errors(m)
        ok, e2, w2 = m.Save3(1, 0, 0)
        out[name] = dict(components=len(M.comps(m)), mate_errors=errors, saved=bool(ok and not e2))
        print(name, out[name], flush=True)
        sw.CloseDoc(m.GetTitle())
    rep = json.loads(REPORT.read_text()) if REPORT.exists() else {}
    log('robots', {**rep.get('robots', {}), **out})


def main():
    sw = connect()
    stage = sys.argv[1]
    if stage == 'import':
        import_parts(sw)
    elif stage in ('deck', 'bumper', 'fasteners'):
        group(sw, stage)
    elif stage == 'robot':
        global ROBOTS
        ROBOTS = sys.argv[2:] or ROBOTS
        robots(sw)


if __name__ == '__main__':
    main()
