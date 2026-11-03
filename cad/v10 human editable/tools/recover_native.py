from sw_native import *

sw=connect()
for name in sys.argv[1:]:
    path=ROOT/'validation'/('INCOMPLETE_'+name+'.SLDPRT')
    d,e,w=sw.OpenDoc6(str(path),1,1,'',0,0);m=wrap(d,sld.IModelDoc2);sw.ActivateDoc3(m.GetTitle(),False,0,0)
    p=object.__new__(NativePart);p.sw=sw;p.model=m;p.name=name;p.sketches=[]
    seen=set()
    def walk(f,sub=False):
        while f:
            if f.Name not in seen:
                seen.add(f.Name)
                if f.GetTypeName2()=='ProfileFeature':
                    sk=wrap(f.GetSpecificFeature2(),sld.ISketch);status=sk.GetConstrainedStatus()
                    if status!=3:raise RuntimeError('Underconstrained '+f.Name)
                    p.sketches.append({'name':f.Name,'status':status,'fully_define_result':0})
                walk(wrap(f.GetFirstSubFeature(),sld.IFeature),True)
            f=wrap(f.GetNextSubFeature() if sub else f.GetNextFeature(),sld.IFeature)
    walk(wrap(m.FirstFeature(),sld.IFeature))
    data=json.loads((ROOT/'reference'/(name+'.json')).read_text())
    p.save(expected_volume=data['volume_mm3'])
