"""SolidWorks COM helpers for cad/v13's assembly scripts (sw_robot.py, subasm.py, mate_api.py). Run with C:/Users/ferna/cadenv/Scripts/python.exe.

Copied from cad/v10's tools/sw_native.py and trimmed to the generic helpers (the v10 NativePart authoring class and its V10_ file names stay in
v10). Everything cad/v13 needs is read from cad/v13 (cad/solidworks.md section 17: files come from the current version's folder).
connect() remembers the two settings it changes (the modal Modify popup, CommandInProgress) and release() puts them back: call it when a
script finishes, so Fernando's session is left as it was found.
"""
from pathlib import Path
import sys
import pythoncom
import win32com.client as wc
from win32com.client import gencache, constants as c

sys.stdout.reconfigure(encoding='utf-8', line_buffering=True)
sld = gencache.EnsureModule('{83A33D31-27C5-11CE-BFD4-00400513BB57}', 0, 31, 0)
gencache.EnsureModule('{4687F359-55D0-4CD3-B6CF-2EB42C11F989}', 0, 31, 0)
ROOT = Path(__file__).resolve().parents[1]                              # cad/v13
MM = .001
_PRIOR = {}


def wrap(o, cls):
    if o is None:
        return None
    d = o._oleobj_
    try:
        d = d.QueryInterface(cls.CLSID, pythoncom.IID_IDispatch)
    except pythoncom.com_error:
        pass
    return cls(d)


def connect():
    try:
        raw = wc.GetActiveObject('SldWorks.Application')
    except pythoncom.com_error:
        raw = wc.Dispatch('SldWorks.Application')
    sw = sld.ISldWorks(raw._oleobj_)
    sw.Visible = True
    _PRIOR['cmd'] = sw.CommandInProgress
    _PRIOR['dim'] = sw.GetUserPreferenceToggle(c.swInputDimValOnCreate)
    sw.CommandInProgress = True
    sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, False)
    return sw


def release(sw):
    """give back what connect() changed (the popup preference and CommandInProgress)"""
    try:
        sw.SetUserPreferenceToggle(c.swInputDimValOnCreate, _PRIOR.get('dim', True))
        sw.CommandInProgress = _PRIOR.get('cmd', False)
    except Exception:                                                  # noqa: BLE001  (SolidWorks already gone)
        pass


def arr(x):
    return wc.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, x)


def last(model):
    recent = wrap(model.FeatureByPositionReverse(0), sld.IFeature)
    if recent is not None:
        return recent
    f = wrap(model.FirstFeature(), sld.IFeature)
    result = None
    while f:
        result = f
        f = wrap(f.GetNextFeature(), sld.IFeature)
    return result


def bodies(model):
    return [wrap(b, sld.IBody2) for b in (wrap(model, sld.IPartDoc).GetBodies2(0, False) or [])]


def placement(cp):
    """4 x 4 (mm) pose of a component in its parent assembly"""
    import numpy as np
    a = np.array(cp.Transform2.ArrayData, float)
    return np.column_stack([a[:9].reshape(3, 3).T, a[9:12] * 1000])
