"""Read-only feature/constraint audit of final native files."""
from sw_native import *

def walk_features(m):
    seen=set()
    def walk(f,sub=False):
        while f:
            if f.Name not in seen:
                seen.add(f.Name);yield f
                yield from walk(wrap(f.GetFirstSubFeature(),sld.IFeature),True)
            f=wrap(f.GetNextSubFeature() if sub else f.GetNextFeature(),sld.IFeature)
    yield from walk(wrap(m.FirstFeature(),sld.IFeature))

def audit(sw,path):
    d,e,w=sw.OpenDoc6(str(path),1,1,'',0,0)
    if d is None:raise RuntimeError('Cannot read '+str(path))
    m=wrap(d,sld.IModelDoc2);types={};sketches=[];references=[];errors=[]
    for f in walk_features(m):
        typ=f.GetTypeName2();types[typ]=types.get(typ,0)+1
        if typ in ('ProfileFeature','3DProfileFeature'):
            sk=wrap(f.GetSpecificFeature2(),sld.ISketch)
            row={'name':f.Name,'status':sk.GetConstrainedStatus()}
            if f.Name.endswith('-LINK_Master_layout'):
                # Stock-feature guide sketches are read-only generated geometry,
                # not locally solvable/editable sketches. Audit their actual
                # driving sketch in the external master instead of fixing a copy.
                md,_,_=sw.OpenDoc6(str(ROOT/'parts/V10_master_layout.SLDPRT'),1,1,'',0,0)
                mf=wrap(wrap(wrap(md,sld.IModelDoc2),sld.IPartDoc).FeatureByName(f.Name.removesuffix('-LINK_Master_layout')),sld.IFeature)
                row['source_status']=wrap(mf.GetSpecificFeature2(),sld.ISketch).GetConstrainedStatus() if mf else None
                row['definition']='read_only_Stock_reference_to_master'
                references.append(row)
            else:sketches.append(row)
        code,warn=f.GetErrorCode2(0)
        if code and not f.IsSuppressed():errors.append([f.Name,code,warn])
    bs=bodies(m);bad=[b.Check2() for b in bs if b.Check2()!=0]
    master=path.stem=='V10_master_layout'
    report={'path':str(path),'feature_types':types,'sketches':sketches,'reference_sketches':references,'feature_errors':errors,'body_errors':bad,
            'solid_count':len(bs),'imported_brep_count':types.get('Imported',0)+types.get('ImportedBody',0)}
    report['pass']=not errors and not bad and all(s['status']==3 for s in sketches) and all(s['source_status']==3 for s in references) and len(bs)==(0 if master else 1) and report['imported_brep_count']==0
    print(path.name,'PASS' if report['pass'] else 'FAIL','sketches',len(sketches),'errors',errors[:3],flush=True)
    sw.CloseDoc(m.GetTitle())
    return report

if __name__=='__main__':
    sw=connect();rows=[]
    paths=[ROOT/'parts'/('V10_'+n+'.SLDPRT') for n in sys.argv[1:]] if sys.argv[1:] else sorted((ROOT/'parts').glob('*.SLDPRT'))
    for path in paths:
        if 'probe' not in path.stem and not path.name.startswith('~$'):
            rows.append(audit(sw,path))
            (ROOT/'validation/native_feature_audit.json').write_text(json.dumps(rows,indent=2))
    (ROOT/'validation/native_feature_audit.json').write_text(json.dumps(rows,indent=2))
    if any(not r['pass'] for r in rows):raise SystemExit(1)
