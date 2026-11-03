"""Convert step_parts/*.step into SolidWorks part files in parts/step/ (imported geometry: not human editable, by request).

    cadenv python 32_import_steps.py [name ...]      (no names: every STEP not yet converted)
"""
import sys, json, time
from subasm import *

SRC = ROOT / 'step_parts'
DST = ROOT / 'parts' / 'step'
DST.mkdir(parents=True, exist_ok=True)


def import_one(sw, stem):
    out = DST / (stem + '.SLDPRT')
    if out.exists():
        return 'exists'
    step = SRC / (stem + '.step')
    idata = sw.GetImportFileData(str(step))
    d, err = sw.LoadFile4(str(step), '', idata, 0)
    if d is None:
        raise RuntimeError(f'cannot import {step} ({err})')
    m = wrap(d, sld.IModelDoc2)
    if m.GetType() != 1:
        title = m.GetTitle()
        sw.CloseDoc(title)
        raise RuntimeError(f'{stem} imported as an assembly (several products): split it first')
    bodies_n = len(bodies(m))
    ok, e, w = m.Extension.SaveAs3(str(out), 0, 1, None, None, 0, 0)
    sw.CloseDoc(m.GetTitle())
    if not ok or e:
        raise RuntimeError(f'save {out} {(e, w)}')
    return f'{bodies_n} bodies'


def main():
    sw = connect()
    names = sys.argv[1:] or sorted(p.stem for p in SRC.glob('*.step'))
    log = {}
    for i, n in enumerate(names):
        t0 = time.time()
        try:
            log[n] = import_one(sw, n)
        except Exception as e:                           # noqa: BLE001
            log[n] = 'FAILED ' + str(e)
        print(f'[{i + 1}/{len(names)}] {n}: {log[n]} ({time.time() - t0:.0f} s)', flush=True)
    (ROOT / 'validation' / 'step_import.json').write_text(json.dumps(log, indent=1))
    bad = {k: v for k, v in log.items() if v.startswith('FAILED')}
    print('failures:', bad)


if __name__ == '__main__':
    main()
