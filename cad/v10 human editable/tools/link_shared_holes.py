"""Apply native master references, retaining independent fit diameters/depths."""
from sw_native import *
import numpy as np
from update_native import open_part

LINK='LINK_Master_layout'

def imported_parameter(name):
    d,sk=name.split('@',1)
    return d+'@'+sk+'-'+LINK

def imported_sketch(part,name):
    f=wrap(part.FeatureByName(name+'-'+LINK),sld.IFeature)
    if f is None:raise RuntimeError('Missing imported master sketch '+name)
    return wrap(f.GetSpecificFeature2(),sld.ISketch)

def apply_part(sw,name,items,plan,master):
    m=open_part(sw,name);part=wrap(m,sld.IPartDoc)
    report_path=ROOT/'validation'/(name+'.json');old=json.loads(report_path.read_text())
    link=wrap(part.FeatureByName(LINK),sld.IFeature)
    finished=ROOT/'validation'/('LINKS_'+name+'.json')
    previous=json.loads(finished.read_text())['links'] if link is not None and finished.exists() else []
    completed={r['feature'] for r in previous}
    items=[r for r in items if r['feature'] not in completed]
    if link is not None and not items:
        print('MASTER LINKS already saved',name,flush=True);sw.CloseDoc(m.GetTitle());return
    if link is None:
        link=wrap(part.InsertPart3(str(ROOT/'parts/V10_master_layout.SLDPRT'),64|1024,''),sld.IFeature)
        if link is None:raise RuntimeError('Cannot derive master into '+name)
        link.Name=LINK
        if not m.Extension.ReorderFeature(LINK,old['sketches'][0]['name'],2):raise RuntimeError('Cannot place master before geometry')
    m.FeatureManager.EnableFeatureTree=False;m.FeatureManager.EnableFeatureTreeWindow=False
    wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=False
    eq=wrap(m.GetEquationMgr(),sld.IEquationMgr);eq.AutomaticRebuild=False;eq.AutomaticSolveOrder=True
    changed=list(previous)
    for ordinal,item in enumerate(items,1):
        sf=wrap(part.FeatureByName(item['sketch']),sld.IFeature)
        tf=wrap(part.FeatureByName(item['translation']),sld.IFeature)
        if sf is None or tf is None:raise RuntimeError('Missing native feature '+str(item))
        if item['kind']=='circle':
            ms=imported_sketch(part,item['master_feature'])
            ref=wrap(ms.GetSketchSegments()[item['point_index']+1],sld.ISketchSegment)
            m.ClearSelection2(True);sf.Select2(False,0);m.SketchManager.InsertSketch(True)
            sk=wrap(m.SketchManager.ActiveSketch,sld.ISketch);rm=wrap(sk.RelationManager,sld.ISketchRelationManager)
            for raw in rm.GetRelations(0) or []:
                r=wrap(raw,sld.ISketchRelation)
                if r.GetRelationType()==9:
                    if not rm.DeleteRelation(r):raise RuntimeError('Cannot remove old coordinate lock')
            own=wrap(sk.GetSketchSegments()[0],sld.ISketchSegment)
            m.ClearSelection2(True);own.Select4(False,None);ref.Select4(True,None);m.SketchAddConstraints('sgCONCENTRIC')
            status=sk.GetConstrainedStatus();m.SketchManager.InsertSketch(True)
            if status!=3:raise RuntimeError('Master-linked circle under-constrained '+item['sketch'])
            axis=np.array(item['axis_local']);original=np.array(item['t']);axial=axis*np.dot(axis,original)
            fd=wrap(tf.GetDefinition(),sld.IMoveCopyBodyFeatureData)
            if not fd.AccessSelections(m,None):raise RuntimeError('Cannot edit hole axial position')
            fd.TransformX=float(axial[0])*MM;fd.TransformY=float(axial[1])*MM;fd.TransformZ=float(axial[2])*MM
            if not tf.ModifyDefinition(fd,m,None):raise RuntimeError('Cannot preserve axial depth')
            method='native_concentric_to_derived_master'
        else:
            # Conical leads and hexagonal pockets follow the same centre through
            # native equations; their local section and axial depth stay editable.
            q0=plan['patterns'][item['pattern']]['points'][item['point_index']]
            U=np.array(item['world_to_part_U']);t=np.array(item['t'])
            dims=master['references']['patterns'][item['pattern']]['coordinate_dimensions'][item['point_index']]
            datum=master['references']['patterns'][item['pattern']]['datum_dimensions']
            # Move/Copy dimensions store unsigned distances and reject negative
            # equations. Use a positive coordinate frame with an equal native
            # compensation move, so a shared location can cross either axis.
            active=np.max(abs(U),axis=1)>1e-10
            offset=active.astype(float)*1000
            if not m.FeatureManager.EditRollback(4,tf.Name):raise RuntimeError('Cannot position coordinate frame')
            fd=wrap(tf.GetDefinition(),sld.IMoveCopyBodyFeatureData)
            if not fd.AccessSelections(m,None):raise RuntimeError('Cannot edit coordinate frame')
            fd.TransformX=float(t[0]+offset[0])*MM;fd.TransformY=float(t[1]+offset[1])*MM;fd.TransformZ=float(t[2]+offset[2])*MM
            if not tf.ModifyDefinition(fd,m,None):raise RuntimeError('Coordinate frame failed')
            m.EditRebuild3()
            far=[b for b in bodies(m) if all(b.GetMassProperties(1)[i]>.5 for i in range(3) if active[i])]
            if len(far)!=1:raise RuntimeError('Ambiguous coordinate-frame tool body')
            m.ClearSelection2(True);sd=wrap(wrap(m.SelectionManager,sld.ISelectionMgr).CreateSelectData(),sld.ISelectData);sd.Mark=1
            far[0].Select2(False,sd)
            cf=m.FeatureManager.InsertMoveCopyBody2(*[float(-v)*MM for v in offset],0,0,0,0,0,0,0,False,1)
            if cf is None:raise RuntimeError('Cannot restore coordinate frame')
            cf.Name='FRAME_'+item['translation']
            if not m.FeatureManager.EditRollback(1,''):raise RuntimeError('Cannot restore feature history')
            for axis in range(3):
                if not active[axis]:continue
                dst=f'D{axis+1}@'+item['translation']
                pd=wrap(m.Parameter(dst),sld.IDimension)
                if pd is None:raise RuntimeError('Missing move dimension '+dst)
                if abs(pd.SystemValue*1000-t[axis]-offset[axis])>1e-5:raise RuntimeError('Unexpected move dimension order '+dst)
                terms=[f'{t[axis]+offset[axis]:.12g}mm']
                for j in range(2):
                    if abs(U[axis,j])>1e-10:
                        terms.append(f'({U[axis,j]:.12g})*("{imported_parameter(dims[j])}" - "{imported_parameter(datum[j])}" - ({q0[j]:.12f}mm))')
                equation=f'"{dst}" = '+' + '.join(terms)
                if eq.Add2(-1,equation,False)<0:raise RuntimeError('Placement equation failed '+equation)
            method='native_equation_to_same_master'
        changed.append({'feature':item['feature'],'sketch':item['sketch'],'pattern':item['pattern'],'point_index':item['point_index'],'method':method})
        if ordinal%10==0:print('Master-linked tools',name,ordinal,'/',len(items),flush=True)
    eq.EvaluateAll();eq.AutomaticRebuild=True;m.ForceRebuild3(False)
    from audit_native import walk_features
    m.ClearSelection2(True);selected=0
    for f in walk_features(m):
        if f.GetTypeName2()=='ProfileFeature' and f.Name.endswith('-'+LINK):
            f.Select2(selected>0,0);selected+=1
    if selected:m.BlankSketch()
    m.ClearSelection2(True)
    p=object.__new__(NativePart);p.sw=sw;p.name=name;p.model=m;p.sketches=old['sketches']
    for row in p.sketches:
        f=wrap(part.FeatureByName(row['name']),sld.IFeature)
        if f is None:raise RuntimeError('Lost sketch '+row['name'])
        status=wrap(f.GetSpecificFeature2(),sld.ISketch).GetConstrainedStatus();row['status']=status
        if status!=3:raise RuntimeError('Under-constrained sketch after linking '+row['name'])
    data=json.loads((ROOT/'reference'/(name+'.json')).read_text());p.save(expected_volume=data['volume_mm3'])
    (ROOT/'validation'/('LINKS_'+name+'.json')).write_text(json.dumps({'part':name,'master':str(ROOT/'parts/V10_master_layout.SLDPRT'),'links':changed,'equation_count':eq.GetCount()},indent=2))
    print('MASTER LINKS',name,len(changed),flush=True)
    sw.CloseDoc(m.GetTitle())

if __name__=='__main__':
    sw=connect();plan=json.loads((ROOT/'reference/shared_holes.json').read_text());master=json.loads((ROOT/'validation/master_layout.json').read_text())
    items=json.loads((ROOT/'reference/shared_hole_candidates.json').read_text())
    if items['missing_parts']:raise RuntimeError('Missing slave parts: '+str(items['missing_parts']))
    names=sys.argv[1:] or sorted({c['part'] for c in items['candidates']})
    for name in names:
        try:apply_part(sw,name,[c for c in items['candidates'] if c['part']==name],plan,master)
        except Exception:
            path=ROOT/'parts'/('V10_'+name+'.SLDPRT')
            for raw in sw.GetDocuments() or []:
                doc=wrap(raw,sld.IModelDoc2)
                if doc.GetPathName().lower()==str(path).lower():sw.CloseDoc(doc.GetTitle())
            raise
