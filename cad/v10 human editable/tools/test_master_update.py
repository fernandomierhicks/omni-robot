"""Change a real master dimension, verify slaves, restore and save the chain."""
from sw_native import *
from update_native import open_part
import numpy as np

def centres(m,features):
    part=wrap(m,sld.IPartDoc);out={}
    for name in features:
        f=wrap(part.FeatureByName(name),sld.IFeature);sk=wrap(f.GetSpecificFeature2(),sld.ISketch)
        arc=wrap(sk.GetSketchSegments()[0],sld.ISketchArc);pt=wrap(arc.GetCenterPoint2(),sld.ISketchPoint)
        out[name]=[pt.X*1000,pt.Y*1000,sk.GetConstrainedStatus()]
    return out

if __name__=='__main__':
    sw=connect();master=open_part(sw,'master_layout');meta=json.loads((ROOT/'validation/master_layout.json').read_text())
    candidates=json.loads((ROOT/'reference/shared_hole_candidates.json').read_text())['candidates']
    items=[x for x in candidates if x['pattern']=='Cradle_to_fork' and x['point_index']==0 and x['kind']=='circle']
    models={name:open_part(sw,name) for name in sorted({x['part'] for x in items})}
    names={name:[x['sketch'] for x in items if x['part']==name] for name in models}
    before={name:centres(m,names[name]) for name,m in models.items()}
    param=meta['references']['patterns']['Cradle_to_fork']['coordinate_dimensions'][0][1]
    dim=wrap(master.Parameter(param),sld.IDimension);value=dim.SystemValue
    report={'dimension':param,'delta_mm':1,'before':before}
    try:
        if dim.SetSystemValue3(value+.001,1,None)!=0:raise RuntimeError('Master edit rejected')
        master.ForceRebuild3(False);master.Save3(1,0,0)
        for m in models.values():m.ForceRebuild3(False)
        after={name:centres(m,names[name]) for name,m in models.items()};report['after']=after
        for name in before:
            for sk,b in before[name].items():
                a=after[name][sk]
                if abs(a[0]-b[0])>1e-5 or abs(a[1]-b[1]-1)>1e-5 or a[2]!=3:raise RuntimeError('Slave did not follow '+name+' '+sk)
        report['pass']=True
    finally:
        dim.SetSystemValue3(value,1,None);master.ForceRebuild3(False);master.Save3(1,0,0)
        for m in models.values():m.ForceRebuild3(False);m.Save3(1,0,0)
        restored={name:centres(m,names[name]) for name,m in models.items()};report['restored']=restored
        report['restoration_pass']=all(np.max(abs(np.array(restored[n][s][:2])-np.array(before[n][s][:2])))<1e-5 for n in before for s in before[n])
        (ROOT/'validation/master_update_test.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
