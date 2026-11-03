"""The mirrored pod (front-left and rear-right wheels): SolidWorks' own Mirror Components on the finished pod.

    cadenv python 44_mirror_pod.py

A work assembly holds the pod at the origin and mirrors it about its Top Plane (the pod's x-z plane, y -> -y: v9's mirrored pods).
SolidWorks writes an opposite-hand copy of every part and sub-assembly of the pod into assemblies/mirror/ (suffix -MIRROR), mates
included. The mirrored top level is assemblies/mirror/v10h_wheel_pod-mirror.SLDASM; the robot uses it at the 45 and 225 degree corners.
Re-run it after the pod changes (it overwrites assemblies/mirror/).
"""
import sys
from subasm import *

WORK = ASSEMBLIES / '_work'
MIRROR = ASSEMBLIES / 'mirror'


def main():
    sw = connect()
    WORK.mkdir(exist_ok=True)
    MIRROR.mkdir(exist_ok=True)
    a = Asm(sw, WORK / 'mirror_work.SLDASM')
    a.add('Pod', ROOT / 'V10H_wheel_pod.SLDASM', np.eye(4), fixed=True)
    a.refresh()
    a.model.ClearSelection2(True)
    a.model.Extension.SelectByID2('Top Plane', 'PLANE', 0, 0, 0, False, 0, None, 0)
    plane = wrap(a.model.SelectionManager, sld.ISelectionMgr).GetSelectedObject6(1, -1)
    a.model.ClearSelection2(True)
    V = lambda t, x: wc.VARIANT(pythoncom.VT_ARRAY | t, x)
    a.asm.MirrorComponents3(plane, V(pythoncom.VT_DISPATCH, []), V(pythoncom.VT_I4, []), False,
                            V(pythoncom.VT_DISPATCH, [a.parts['Pod']._oleobj_]), False,
                            V(pythoncom.VT_BSTR, [str(MIRROR / 'V10H_wheel_pod-MIRROR.SLDASM')]), 1, '-MIRROR', str(MIRROR), 1, False, False, False)             # ImportOptions 1 = include the solid bodies (0 writes empty parts)
    a.model.EditRebuild3()
    top = M.comps(a.model, True)
    print('components', len(M.comps(a.model)), list(top), flush=True)
    a.save(check=False)
    for n, cp in top.items():
        print(n, cp.GetPathName())


if __name__ == '__main__':
    main()
