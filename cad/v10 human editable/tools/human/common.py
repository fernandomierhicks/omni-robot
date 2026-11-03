"""Managed design-intent models. Never closes or modifies the example document."""
from swhelpers import *
from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'parts/human';OUT.mkdir(exist_ok=True)
REPORTS=ROOT/'validation/human';REPORTS.mkdir(exist_ok=True)
REGISTRY=REPORTS/'managed_documents.json'

def open_owned(sw,key):
    """Load one registered task part; callers close it before loading the next."""
    record=json.loads(REGISTRY.read_text())[key];path=Path(record['path']).resolve()
    for raw in sw.GetDocuments() or []:
        m=wrap(raw,sld.IModelDoc2)
        if m.GetPathName() and Path(m.GetPathName()).resolve()==path:
            sw.ActivateDoc3(m.GetTitle(),False,0,0);return m
    if len(sw.GetDocuments() or [])>=5:raise RuntimeError('Part-work document limit reached; save and close task-owned documents first')
    raw,e,w=sw.OpenDoc6(str(path),1,1,'',0,0)
    if raw is None:raise RuntimeError('Cannot open '+str(path)+' '+str((e,w)))
    m=wrap(raw,sld.IModelDoc2);sw.ActivateDoc3(m.GetTitle(),False,0,0);return m

def close_owned(sw,m,save=False):
    paths={Path(r['path']).resolve() for r in json.loads(REGISTRY.read_text()).values()}
    if not m.GetPathName() or Path(m.GetPathName()).resolve() not in paths:raise RuntimeError('Refusing to close an unmanaged document')
    if m.GetSaveFlag():
        if not save:raise RuntimeError('Refusing to discard unsaved changes in '+m.GetTitle())
        sw.ActivateDoc3(m.GetTitle(),False,0,0);ok,e,w=m.Save3(1,0,0)
        if not ok or e:raise RuntimeError('Cannot save before closing '+str((e,w)))
    sw.CloseDoc(m.GetTitle())

class DesignEqs(Eqs):
    def var(self,name,value,unit='mm',comment=''):
        if not comment:raise RuntimeError('Every control variable needs a comment: '+name)
        for i in range(self.mgr.GetCount()):
            old=self.mgr.Equation(i)
            if old.split('=',1)[0].strip()=='"'+name+'"':
                literal=old.split('=',1)[1].split("'",1)[0].replace(' ','')
                if literal==str(value)+unit and "'" in old:return
        super().var(name,value,unit,comment)
        if not any(self.mgr.Equation(i).split('=',1)[0].strip()=='"'+name+'"' for i in range(self.mgr.GetCount())):
            raise RuntimeError('Global variable refused (possibly a reserved name): '+name)

def start(sw,key,first_feature):
    registry=json.loads(REGISTRY.read_text()) if REGISTRY.exists() else {}
    path=OUT/('V10H_'+key+'.SLDPRT')
    title='V10H_'+key
    matches=[]
    for raw in sw.GetDocuments() or []:
        d=wrap(raw,sld.IModelDoc2)
        if d.GetTitle().removesuffix('.SLDPRT')==title:matches.append(d)
    if key in registry:
        if matches:m=matches[0]
        else:
            raw,e,w=sw.OpenDoc6(str(path),1,1,'',0,0)
            if raw is None:raise RuntimeError('Cannot resume own part '+key)
            m=wrap(raw,sld.IModelDoc2)
        if m.GetPathName() and Path(m.GetPathName())!=path:raise RuntimeError('Document identity collision')
        sw.ActivateDoc3(m.GetTitle(),False,0,0);delete_from(m,first_feature)
    else:
        if matches or path.exists():raise RuntimeError('Unmanaged document exists; refusing to overwrite '+str(path))
        m=wrap(sw.NewDocument(sw.GetUserPreferenceStringValue(c.swDefaultTemplatePart),0,0,0),sld.IModelDoc2)
        m.SetTitle2(title);m.SetUserPreferenceIntegerValue(c.swUnitsLinear,c.swMM)
        registry[key]={'path':str(path),'title':title,'first_feature':first_feature}
        REGISTRY.write_text(json.dumps(registry,indent=2))
    sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,False)
    sw.CommandInProgress=True
    m.FeatureManager.EnableFeatureTree=False
    m.FeatureManager.EnableFeatureTreeWindow=False
    wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=False
    return m,DesignEqs(m),path

def drop_globals(m,names):
    """Delete global variables a rebuilt stage no longer uses (e.g. an alias to a master control that was removed)."""
    em=wrap(m.GetEquationMgr(),sld.IEquationMgr)
    for i in range(em.GetCount()-1,-1,-1):
        if em.Equation(i).split('=',1)[0].strip().strip('"') in names:em.Delete(i)

def link_feature(E,f,expressions):
    dims=feature_own_dims(f)
    if len(dims)!=len(expressions):raise RuntimeError(f'{f.Name}: expected {len(expressions)} dimensions, got {dims}')
    for dim,expression in zip(dims,expressions):E.link(dim,expression)

def audit_report(m):
    seen=set();sketches=[];references=[];errors=[];design=[];dimensions=[]
    def walk(f,sub=False):
        while f:
            if f.Name not in seen:
                seen.add(f.Name);typ=f.GetTypeName2()
                derived=f.Name.endswith('-Shared Layout')
                if not derived:dimensions.extend(feature_own_dims(f))
                if typ=='ProfileFeature':
                    if derived:references.append({'name':f.Name,'read_only':True,'constraint_source':'V10H_master_layout.SLDPRT'})
                    else:sketches.append({'name':f.Name,'status':wrap(f.GetSpecificFeature2(),sld.ISketch).GetConstrainedStatus()})
                if f.GetErrorCode2()[0]:errors.append([f.Name,f.GetErrorCode2()[0]])
                if typ in ('Revolution','Extrusion','ICE','Cut','CirPattern','MirrorPattern','Fillet','Chamfer','CombineBodies','MoveCopyBody'):
                    design.append({'name':f.Name,'type':typ,'dimensions':feature_own_dims(f)})
                walk(wrap(f.GetFirstSubFeature(),sld.IFeature),True)
            f=wrap(f.GetNextSubFeature() if sub else f.GetNextFeature(),sld.IFeature)
    walk(wrap(m.FirstFeature(),sld.IFeature))
    mgr=wrap(m.GetEquationMgr(),sld.IEquationMgr);equations=[mgr.Equation(i) for i in range(mgr.GetCount())]
    linked={t.split('=',1)[0].strip().strip('"') for t in equations if '=' in t}
    unlinked=[name for name in dimensions if name not in linked]
    checks=[b.Check2() for b in bodies(m)]
    report={'sketches':sketches,'derived_reference_sketches':references,'feature_errors':errors,'body_checks':checks,'volume_mm3':volume_mm3(m),
            'dead_equations':dead_equations(m),'unlinked_dimensions':unlinked,'equations':equations,'design_features':design}
    report['pass']=not errors and not report['dead_equations'] and not unlinked and checks==[0] and all(s['status']==3 for s in sketches)
    return report

def finish(sw,m,E,key,path,expected_volume=None):
    sw.ActivateDoc3(m.GetTitle(),False,0,0)
    E.solve();m.ForceRebuild3(False);report=audit_report(m)
    if expected_volume is not None:
        report['expected_volume_mm3']=expected_volume;report['volume_error_mm3']=report['volume_mm3']-expected_volume
        report['pass'] &= abs(report['volume_error_mm3'])<max(.02,expected_volume*2e-5)
    (REPORTS/(key+'.json')).write_text(json.dumps(report,indent=2))
    if not report['pass']:raise RuntimeError('Human-model audit failed: '+key+' '+str({k:v for k,v in report.items() if k in ('feature_errors','dead_equations','unlinked_dimensions','body_checks','volume_error_mm3')}))
    m.FeatureManager.EnableFeatureTree=True
    m.FeatureManager.EnableFeatureTreeWindow=True
    wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=True
    m.ShowNamedView2('*Isometric',7);m.ViewZoomtofit2()
    ok,e,w=m.Extension.SaveAs3(str(path),0,1,None,None,0,0)
    if not ok or e:raise RuntimeError('Native save failed '+str((e,w)))
    ok,e,w=m.Extension.SaveAs3(str(REPORTS/(key+'.step')),0,3,None,None,0,0)
    if not ok or e:raise RuntimeError('Check export failed '+str((e,w)))
    print(key,'SAVED',len(report['sketches']),'fully constrained sketches;',len(report['design_features']),'design features;',report['volume_mm3'],'mm3',flush=True)
    return report

def ring(sw,key,inside,outside,width,mid=False,description=''):
    m,E,path=start(sw,key,'Ring Sketch')
    E.var('bore_diameter',inside,'mm','Inner diameter; bearing or shaft interface')
    E.var('outside_diameter',outside,'mm','Outer diameter of '+description)
    E.var('width',width,'mm','Axial width of '+description)
    s=Sketch(m,'Front Plane','Ring Sketch')
    for diameter,expr in [(outside,'outside_diameter'),(inside,'bore_diameter')]:
        circ=s.circle(0,0,diameter/2);s.rel('sgCOINCIDENT',circ.GetCenterPoint2(),origin=True);s.ddim(circ,expr,(diameter/2+4,4))
    s.finish(E)
    if s.status()!=3:raise RuntimeError('Ring sketch is not fully constrained')
    m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,width,mid=mid,name='Ring Extrude')
    link_feature(E,f,['width'])
    E.solve();m.ForceRebuild3(False)
    initial=volume_mm3(m);test_width=width+.2
    try:
        set_var(m,'width',test_width)
        expected_delta=math.pi*(outside**2-inside**2)*.2/4
        if abs(volume_mm3(m)-initial-expected_delta)>max(.02,initial*2e-5):raise RuntimeError('Width control did not update the ring')
    finally:set_var(m,'width',width)
    return finish(sw,m,E,key,path,math.pi*(outside**2-inside**2)*width/4)

def orthogonal_profile(m,E,plane,name,vertices,axis='horizontal'):
    """A dimension per distinct ordinate, with aligned shared coordinates.

    vertices: (x_mm, y_mm, positive_x_expression, positive_y_expression).
    The outline is one meaningful half section, not a boolean primitive list.
    """
    s=Sketch(m,plane,name);points=[]
    for a,b in zip(vertices,vertices[1:]+vertices[:1]):
        line=s.line(a[0],a[1],b[0],b[1]);points.append(wrap(line.GetStartPoint2(),sld.ISketchPoint))
    for coordinate in (0,1):
        groups={}
        for i,v in enumerate(vertices):groups.setdefault(round(v[coordinate],9),[]).append(i)
        for value,indices in groups.items():
            first=points[indices[0]]
            for index in indices[1:]:s.rel('sgVERTICALPOINTS2D' if coordinate==0 else 'sgHORIZONTALPOINTS2D',first,points[index])
            if abs(value)<1e-9:s.rel('sgVERTICALPOINTS2D' if coordinate==0 else 'sgHORIZONTALPOINTS2D',first,origin=True)
            else:
                expression=vertices[indices[0]][coordinate+2]
                if not expression:raise RuntimeError('Missing named coordinate expression')
                at=(vertices[indices[0]][0]+4,vertices[indices[0]][1]+4)
                (s.hdim if coordinate==0 else s.vdim)(first,None,expression,at,origin_b=True)
    if axis:construction_axis(s,E,axis)
    s.finish(E)
    if s.status()!=3:raise RuntimeError(name+' constraint status '+str(s.status()))
    if axis:select_revolve(s)
    return s

def full_revolve(m,E,profile,name,new_body=False):
    E.var('full_turn',360,'deg','Full circular extent for revolves and patterns')
    f=revolve(m,as_new_body=new_body,name=name);link_feature(E,f,['full_turn']);return f

def patterned(m,E,axis,seed,count,count_expr,name):
    f=circ_pattern(m,axis,seed,count,name)
    dims=feature_own_dims(f)
    for d in dims:
        value=wrap(m.Parameter(d),sld.IDimension).SystemValue
        E.link(d,count_expr if abs(value-count)<1e-8 else 'full_turn')
    return f

def polar_hole(m,E,plane,name,radius,angle,diameter,radius_expr,angle_expr,diameter_expr):
    s=polar_circle_sketch(m,E,plane,name,radius,angle,diameter,radius_expr,angle_expr,diameter_expr);s.finish(E)
    if s.status()!=3:raise RuntimeError(name+' is not fully constrained')
    m.ClearSelection2(True);s.feat.Select2(False,0);return s

def construction_axis(s,E,axis='vertical'):
    E.var('axis_reference_halfspan',10.123,'mm','Half display length of the construction axis; avoids sharing a profile endpoint')
    ax=s.cline(-10.123 if axis=='horizontal' else 0,0 if axis=='horizontal' else -10.123,10.123 if axis=='horizontal' else 0,0 if axis=='horizontal' else 10.123)
    s.rel('sgHORIZONTAL2D' if axis=='horizontal' else 'sgVERTICAL2D',ax)
    first=wrap(ax.GetStartPoint2(),sld.ISketchPoint);last=wrap(ax.GetEndPoint2(),sld.ISketchPoint)
    s.rel('sgHORIZONTALPOINTS2D' if axis=='horizontal' else 'sgVERTICALPOINTS2D',first,origin=True)
    for point in (first,last):(s.hdim if axis=='horizontal' else s.vdim)(point,None,'axis_reference_halfspan',(5,5),origin_b=True)

def select_revolve(s):
    m=s.m;m.ClearSelection2(True);s.feat.Select2(False,0)
    sk=wrap(s.feat.GetSpecificFeature2(),sld.ISketch)
    segment=next(wrap(r,sld.ISketchSegment) for r in sk.GetSketchSegments() if wrap(r,sld.ISketchSegment).ConstructionGeometry)
    segment.Select4(True,mark_data(m,4))

def arc_controls(s,arc,E,centre_expression,radius_expression):
    centre=wrap(arc.GetCenterPoint2(),sld.ISketchPoint)
    s.rel('sgHORIZONTALPOINTS2D',centre,origin=True)
    s.hdim(centre,None,centre_expression,(-25,-5),origin_b=True)
    s.rdim(arc,radius_expression,(25,5))
