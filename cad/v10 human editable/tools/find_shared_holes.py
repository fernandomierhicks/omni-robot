"""Read native feature parameters and identify shared fastener tool axes."""
from sw_native import *
import numpy as np
from scipy.spatial.transform import Rotation

def features(m):
    f=wrap(m.FirstFeature(),sld.IFeature)
    while f:
        yield f;f=wrap(f.GetNextFeature(),sld.IFeature)

def scan(sw,plan,part_names=None):
    candidates=[];missing=[]
    part_names=part_names or sorted({p for line in plan['lines'] for p in line['parts']})
    for name in part_names:
        path=ROOT/'parts'/('V10_'+name+'.SLDPRT')
        if not path.exists():missing.append(name);continue
        d,e,w=sw.OpenDoc6(str(path),1,1,'',0,0)
        m=wrap(d,sld.IModelDoc2);ff=list(features(m));A=np.array(plan['frames'][name]);AR=A[:3,:3];AT=A[:3,3]
        for pos,f in enumerate(ff):
            fname=f.Name;typ=f.GetTypeName2();kind='extrusion' if typ in ('Extrusion','ICE') else 'revolved_profile' if typ=='Revolution' else None
            if not kind:continue
            sub=wrap(f.GetFirstSubFeature(),sld.IFeature)
            while sub and sub.GetTypeName2()!='ProfileFeature':sub=wrap(sub.GetNextSubFeature(),sld.IFeature)
            if not sub:continue
            sk=wrap(sub.GetSpecificFeature2(),sld.ISketch);segments=[wrap(x,sld.ISketchSegment) for x in (sk.GetSketchSegments() or [])]
            segments=[x for x in segments if not x.ConstructionGeometry]
            if kind=='extrusion':kind='circle' if len(segments)==1 else 'polygon'
            center=np.zeros(3)
            if kind=='circle':
                if len(segments)!=1:continue
                arc=wrap(segments[0],sld.ISketchArc);radius=arc.GetRadius()*1000
                pt=wrap(arc.GetCenterPoint2(),sld.ISketchPoint);center=np.array([pt.X,pt.Y,0])*1000
            else:
                pts=[]
                for seg in segments:
                    line=wrap(seg,sld.ISketchLine);q=wrap(line.GetStartPoint2(),sld.ISketchPoint);pts.append([q.X*1000,q.Y*1000,0])
                pts=np.array(pts)
                if kind=='polygon':
                    if len(pts)!=6:continue
                    center=pts.mean(0);rr=np.linalg.norm(pts-center,axis=1)
                    if max(rr)-min(rr)>.01:continue
                    radius=float(np.mean(rr))
                else:radius=float(max(abs(pts[:,0])))
            R=np.eye(3);t=np.zeros(3);translation=None
            for follow in ff[pos+1:]:
                if follow.GetTypeName2()!='MoveCopyBody':break
                fd=wrap(follow.GetDefinition(),sld.IMoveCopyBodyFeatureData)
                if fd.Copy:break
                if fd.TransformType==2:R=Rotation.from_euler('XYZ',[fd.TransformX,fd.TransformY,fd.TransformZ]).as_matrix()
                elif fd.TransformType==1:t=np.array([fd.TransformX,fd.TransformY,fd.TransformZ])*1000;translation=follow.Name
            axis_local=R[:,1 if kind in ('cone','revolved_profile') else 2];axis=AR@axis_local
            point=AR@(t+R@center)+AT
            hits=[]
            for line in plan['lines']:
                if name not in line['parts']:continue
                dd=np.array(line['axis']);delta=point-np.array(line['point'])
                if abs(abs(axis@dd)-1)>1e-6 or np.linalg.norm(delta-(delta@dd)*dd)>.001:continue
                if min(abs(radius-r) for r in line['radii'])>.015:continue
                hits.append(line)
            if not hits:continue
            if len(hits)!=1:raise RuntimeError('Ambiguous hole '+name+' '+fname)
            line=hits[0]
            if translation is None:raise RuntimeError('Shared hole has no translation feature '+name+' '+fname)
            U=np.column_stack([line['u'],line['v']]);C=R.T@AR.T@U;k=-R.T@AR.T@AT
            candidates.append({'part':name,'feature':fname,'sketch':sub.Name,'translation':translation,'kind':kind,'radius_mm':radius,
                               'pattern':line['pattern'],'point_index':line['index'],'line':line['name'],'R':R.tolist(),'t':t.tolist(),
                               'projection':np.round(C[:2,:],10).tolist(),'projection_offset':np.round(k[:2],10).tolist(),
                               'world_to_part_U':np.round(AR.T@U,10).tolist(),'axis_local':axis_local.tolist(),
                               'dynamic_origin':'pivot_shaft' if name=='pivot_shaft' else None})
        print(name,'shared tools',sum(c['part']==name for c in candidates),flush=True)
        sw.CloseDoc(m.GetTitle())
    return {'candidates':candidates,'missing_parts':missing}

if __name__=='__main__':
    sw=connect();plan=json.loads((ROOT/'reference/shared_holes.json').read_text());data=scan(sw,plan,sys.argv[1:])
    if sys.argv[1:]:
        old=json.loads((ROOT/'reference/shared_hole_candidates.json').read_text())
        def signature(r):return json.dumps([r['pattern'],r['projection'],r['projection_offset'],r.get('dynamic_origin')],sort_keys=True)
        masters={signature(r):r['master_feature'] for r in old['candidates'] if r['kind']=='circle' and 'master_feature' in r}
        for row in data['candidates']:
            if row['kind']=='circle':row['master_feature']=masters[signature(row)]
        data['candidates']=[r for r in old['candidates'] if r['part'] not in sys.argv[1:]]+data['candidates']
        data['missing_parts']=[n for n in old['missing_parts'] if n not in sys.argv[1:]]+data['missing_parts']
    (ROOT/'reference/shared_hole_candidates.json').write_text(json.dumps(data,indent=2));print('Total',len(data['candidates']),'missing',data['missing_parts'])
