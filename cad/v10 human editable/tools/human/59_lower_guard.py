"""Nov 2 2026: lower the wheel guard's pivot-side wall and ears (Fernando: the part that goes radially into the robot does not need to be that tall;
its height set the deck bridge's, which was too tall to use).

    cadenv python -B 59_lower_guard.py [pivot_wall_top low_wall_top outer_ear_top]        (default 20 24 26)

Adds the global `pivot_wall_top`, points the Pivot Side Wall extrude and the Inner Ear sketch at it, and lowers `outer_ear_top` (26) and
`low_wall_top` (24). The outer ear must stay taller than the outer low wall: at or below it the bore-entry chamfer on the ear's outer face fails (warning 13).
The panel-root fillet needs the low wall to stay over 20 mm tall. The far wall (`wall_top`, 57) stays: it braces the impact panel. Rebuilds, audits and saves V10H_wheel_guard; its mirror part follows when it is rebuilt.
"""
import sys
from common import *

TOP, LOW, OUTER = (float(v) for v in sys.argv[1:4]) if sys.argv[3:] else (20.0, 24.0, 26.0)
sw = connect()
try:
    m = open_owned(sw, 'wheel_guard')
    sw.ActivateDoc3(m.GetTitle(), False, 0, 0)
    em = wrap(m.GetEquationMgr(), sld.IEquationMgr)

    def find(name):
        for i in range(em.GetCount()):
            if em.Equation(i).split('=', 1)[0].strip().strip('"') == name:
                return i
        return None

    def put(name, rhs, comment=None):
        i = find(name)
        old = em.Equation(i)
        c = comment or (old.split("'", 1)[1] if "'" in old else '')
        txt = '"' + name + '" = ' + rhs + ((" '" + c) if c else '')
        em.SetEquation(i, txt)
        got = em.Equation(find(name)).split("'", 1)[0].replace(' ', '')
        if got != txt.split("'", 1)[0].replace(' ', ''):
            raise RuntimeError('equation not accepted: ' + txt + ' -> ' + got)

    if find('pivot_wall_top') is None:
        at = find('outer_ear_top')           # before its first user: an equation cannot see a later global
        if em.Add2(at, '"pivot_wall_top"= %gmm \'Pivot side wall and inner ear top (Nov 2: lowered from 57 so the deck bridge can be low)' % TOP, False) < 0:
            raise RuntimeError('cannot add pivot_wall_top')
    else:
        put('pivot_wall_top', '%gmm' % TOP)
    put('D1@Pivot Side Wall Extrude', '"pivot_wall_top" - "deck_height"')
    put('D5@Inner Ear Sketch', '"pivot_wall_top"')
    put('outer_ear_top', '%gmm' % OUTER)
    put('low_wall_top', '%gmm' % LOW)
    print('values', [(n, em.Value(find(n))) for n in ('pivot_wall_top', 'outer_ear_top', 'D1@Pivot Side Wall Extrude', 'D5@Inner Ear Sketch', 'D5@Outer Ear Sketch', 'low_wall_top')])
    v0 = volume_mm3(m)
    em.EvaluateAll()
    m.ForceRebuild3(False)
    print('volume', v0, '->', volume_mm3(m))
    r = finish(sw, m, DesignEqs(m), 'wheel_guard', OUT / 'V10H_wheel_guard.SLDPRT')
    print('audit pass', r['pass'])
finally:
    sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, True)
