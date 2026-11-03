"""Small native dimension edits and check-export refresh, without rebuilding parts."""
from sw_native import *

def open_part(sw,name):
    path=ROOT/'parts'/('V10_'+name+'.SLDPRT')
    d,e,w=sw.OpenDoc6(str(path),1,1,'',0,0)
    if d is None:raise RuntimeError('Open failed '+name)
    m=wrap(d,sld.IModelDoc2);sw.ActivateDoc3(m.GetTitle(),False,0,0)
    if Path(m.GetPathName()).resolve()!=path.resolve():raise RuntimeError('Wrong document path')
    return m

def refresh(sw,name,edit_diameter=None):
    m=open_part(sw,name)
    old=json.loads((ROOT/'validation'/(name+'.json')).read_text())
    if edit_diameter:
        old_d,new_d=edit_diameter;changed=[]
        for item in old['sketches']:
            dim=wrap(m.Parameter('D1@'+item['name']),sld.IDimension)
            if dim and abs(dim.SystemValue-old_d*.001)<1e-8:
                status=dim.SetSystemValue3(new_d*.001,1,None)
                if status!=0:raise RuntimeError('Dimension edit failed '+str(status))
                changed.append(item['name'])
        if len(changed)!=1:raise RuntimeError('Expected exactly one target diameter, got '+str(changed))
        print('Edited native dimension',name,changed,old_d,'->',new_d,flush=True)
    p=object.__new__(NativePart);p.sw=sw;p.name=name;p.model=m;p.sketches=old['sketches']
    for item in p.sketches:
        f=wrap(wrap(m,sld.IPartDoc).FeatureByName(item['name']),sld.IFeature)
        sk=wrap(f.GetSpecificFeature2(),sld.ISketch)
        item['status']=sk.GetConstrainedStatus()
        if item['status']!=3:raise RuntimeError('Sketch no longer fully defined '+item['name'])
    ref=json.loads((ROOT/'reference'/(name+'.json')).read_text())
    return p.save(expected_volume=ref['volume_mm3'])

if __name__=='__main__':
    sw=connect()
    for name in sys.argv[1:]:refresh(sw,name,(26,23) if name in ('inner_fork_6804','outer_fork_6804') else None)
