"""Native SolidWorks authoring. Run with C:/Users/ferna/cadenv/Scripts/python.exe.

No STEP import or imported BREP is used by this module.
"""
from pathlib import Path
import sys, math, json, time
import pythoncom
import win32com.client as wc
from win32com.client import gencache, constants as c

sys.stdout.reconfigure(encoding='utf-8', line_buffering=True)
sld = gencache.EnsureModule('{83A33D31-27C5-11CE-BFD4-00400513BB57}',0,31,0)
gencache.EnsureModule('{4687F359-55D0-4CD3-B6CF-2EB42C11F989}',0,31,0)
ROOT = Path(__file__).resolve().parents[1]
MM=.001

def wrap(o, cls):
    if o is None: return None
    d=o._oleobj_
    try: d=d.QueryInterface(cls.CLSID,pythoncom.IID_IDispatch)
    except pythoncom.com_error: pass
    return cls(d)

def connect():
    try: raw=wc.GetActiveObject('SldWorks.Application')
    except pythoncom.com_error: raw=wc.Dispatch('SldWorks.Application')
    sw=sld.ISldWorks(raw._oleobj_); sw.Visible=True
    sw.CommandInProgress=True
    sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,False)
    return sw

def arr(x): return wc.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,x)

def last(model):
    recent=wrap(model.FeatureByPositionReverse(0),sld.IFeature)
    if recent is not None:return recent
    f=wrap(model.FirstFeature(),sld.IFeature); result=None
    while f:
        result=f; f=wrap(f.GetNextFeature(),sld.IFeature)
    return result

def bodies(model): return [wrap(b,sld.IBody2) for b in (wrap(model,sld.IPartDoc).GetBodies2(0,False) or [])]

def edges(body):
    # Bypass pywin32's recursive dynamic dispatch/type-info lookup for each
    # member of the large edge array. These are already known IEdge objects.
    raw=body._oleobj_.InvokeTypes(125,0,1,(12,0),())
    return [sld.IEdge(e.QueryInterface(sld.IEdge.CLSID,pythoncom.IID_IDispatch)) for e in (raw or [])]

def curve(edge):
    raw=edge._oleobj_.InvokeTypes(1,0,1,(9,0),())
    return sld.ICurve(raw.QueryInterface(sld.ICurve.CLSID,pythoncom.IID_IDispatch))

def rebuild(model):
    # Refresh derived sketch dimensions before evaluating consumers. SolidWorks
    # otherwise evaluates cross-part dimension equations one rebuild too early.
    model.ForceRebuild3(False)
    eq=wrap(model.GetEquationMgr(),sld.IEquationMgr)
    if eq.GetCount():eq.EvaluateAll()
    model.ForceRebuild3(False)

def placement(cp):
    import numpy as np
    a=np.array(cp.Transform2.ArrayData,float)
    return np.column_stack([a[:9].reshape(3,3).T,a[9:12]*1000])

class NativePart:
    def __init__(self,sw,name):
        self.sw=sw; self.name=name; self.sketches=[]; self.serial=0
        for raw in sw.GetDocuments() or []:
            doc=wrap(raw,sld.IModelDoc2)
            title=doc.GetTitle().replace('.SLDPRT','')
            if title in ('V10_'+name,'INCOMPLETE_'+name):sw.CloseDoc(doc.GetTitle())
        template=sw.GetUserPreferenceStringValue(c.swDefaultTemplatePart)
        if not template: raise RuntimeError('No default SolidWorks part template')
        self.model=wrap(sw.NewDocument(template,0,0,0),sld.IModelDoc2)
        self.model.SetUserPreferenceIntegerValue(c.swUnitsLinear, c.swMM)
        self.model.SetTitle2('V10_'+name)
        self.model.FeatureManager.EnableFeatureTree=False
        self.model.FeatureManager.EnableFeatureTreeWindow=False

    def sketch(self,profile,name,axis=None):
        m=self.model; m.ClearSelection2(True)
        if not m.Extension.SelectByID2('Front Plane','PLANE',0,0,0,False,0,None,0): raise RuntimeError('front plane')
        sm=m.SketchManager; sm.InsertSketch(True); sm.AddToDB=True; sm.DisplayWhenAdded=False
        if profile['kind']=='circle':
            sm.CreateCircleByRadius(*[v*MM for v in profile.get('center',[0,0])],0,profile['radius']*MM)
        else:
            for pts in profile['loops']:
                for a,b in zip(pts,pts[1:]+pts[:1]):
                    if math.dist(a,b)<1e-7: continue
                    if sm.CreateLine(a[0]*MM,a[1]*MM,0,b[0]*MM,b[1]*MM,0) is None: raise RuntimeError('line')
        ax=None
        if axis:
            a,b=axis
            ax=sm.CreateCenterLine(a[0]*MM,a[1]*MM,0,b[0]*MM,b[1]*MM,0)
        sm.AddToDB=False; sm.DisplayWhenAdded=True
        sk=wrap(sm.ActiveSketch,sld.ISketch)
        # Explicit coordinates avoid FullyDefineSketch's silent under-defined result.
        for obj in sk.GetSketchPoints2() or []:
            pt=wrap(obj,sld.ISketchPoint)
            x,y=pt.X,pt.Y
            if abs(x)+abs(y)<1e-10:
                m.ClearSelection2(True);pt.Select4(False,None)
                m.Extension.SelectByID2('Point1@Origin','EXTSKETCHPOINT',0,0,0,True,0,None,0)
                m.SketchAddConstraints('sgCOINCIDENT')
            else:
                for index,value in enumerate((x,y)):
                    m.ClearSelection2(True);pt.Select4(False,None)
                    m.Extension.SelectByID2('Point1@Origin','EXTSKETCHPOINT',0,0,0,True,0,None,0)
                    if abs(value)<1e-10:
                        m.SketchAddConstraints('sgVERTICALPOINTS2D' if index==0 else 'sgHORIZONTALPOINTS2D')
                    else:
                        dd=(m.AddHorizontalDimension2 if index==0 else m.AddVerticalDimension2)(x+.004,y+.004,0)
                        if dd is None: raise RuntimeError('coordinate dimension failed')
        if profile['kind']=='circle':
            m.ClearSelection2(True)
            segment=wrap(sk.GetSketchSegments()[0],sld.ISketchSegment);segment.Select4(False,None)
            if m.AddDiameterDimension2(profile['radius']*MM+0.004,0.004,0) is None: raise RuntimeError('diameter dimension failed')
        code=0
        status=sk.GetConstrainedStatus()
        self.sketches.append({'name':name,'status':status,'fully_define_result':code})
        sm.InsertSketch(True)
        f=last(m); f.Name=name
        self.sketches[-1]['name']=f.Name
        m.EditRebuild3()
        status=wrap(f.GetSpecificFeature2(),sld.ISketch).GetConstrainedStatus()
        self.sketches[-1]['status']=status
        if status != c.swFullyConstrained: raise RuntimeError(f'{name}: sketch status={status}, result={code}')
        return f

    def extrude(self,profile,z0,z1,name,merge=False):
        m=self.model; sk=self.sketch(profile,name+'_Sketch'); m.ClearSelection2(True); sk.Select2(False,0)
        f=m.FeatureManager.FeatureExtrusion3(True,False,False,0,0,(z1-z0)*MM,0,False,False,False,False,0,0,False,False,False,False,merge,True,True,3 if abs(z0)>1e-9 else 0,abs(z0)*MM,z0<0)
        if f is None: raise RuntimeError(name+' extrude failed')
        f.Name=name; return f

    def save(self,expected_volume=None):
        m=self.model; m.ForceRebuild3(False)
        m.FeatureManager.EnableFeatureTree=True
        m.FeatureManager.EnableFeatureTreeWindow=True
        wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=True
        bs=bodies(m)
        check=[b.Check2() for b in bs]
        if any(check): raise RuntimeError('Invalid native body '+str(check))
        if len(bs)!=1:raise RuntimeError('Expected one final solid, got '+str(len(bs)))
        vol=sum(b.GetMassProperties(1)[3]*1e9 for b in bs)
        mass_tolerance=max(.5,(expected_volume or 0)*(1e-4 if self.name in ('wheel_hub_A','wheel_hub_B','wheel_hub_C') else 1e-5))
        if expected_volume is not None and abs(vol-expected_volume)>mass_tolerance:
            raise RuntimeError(f'Volume mismatch before saving: {vol-expected_volume}')
        m.ShowNamedView2('*Isometric',7); m.ViewZoomtofit2()
        path=ROOT/'parts'/('V10_'+self.name+'.SLDPRT')
        ok,err,warn=m.Extension.SaveAs3(str(path),0,1,None,None,0,0)
        if not ok or err: raise RuntimeError(f'save {err}/{warn}')
        # Export a disposable check surface for independent OCC comparison.
        step=ROOT/'validation'/('V10_'+self.name+'.step')
        exported,exerr,exwarn=m.Extension.SaveAs3(str(step),0,3,None,None,0,0)
        if not exported or exerr:raise RuntimeError(f'check export {exerr}/{exwarn}')
        data={'part':self.name,'path':str(path),'bodies':len(bs),'checks':check,'sketches':self.sketches,'volume_mm3':vol,
              'mass_integration_tolerance_mm3':mass_tolerance,'reference_volume_mm3':expected_volume}
        (ROOT/'validation'/(self.name+'.json')).write_text(json.dumps(data,indent=2))
        print(json.dumps({k:v for k,v in data.items() if k!='sketches'}|{'fully_constrained_sketches':len(data['sketches'])}),flush=True)
        return data

if __name__=='__main__':
    sw=connect()
    p=NativePart(sw,'probe')
    p.extrude({'kind':'circle','radius':4},-49,49,'Shaft')
    p.save()
