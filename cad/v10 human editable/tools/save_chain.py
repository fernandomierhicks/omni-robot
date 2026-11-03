"""Rebuild/save only v10 documents, master then parts then assemblies."""
from sw_native import *
from audit_native import walk_features
import mate_api as M

def save_chain(sw):
    documents=[]
    for raw in sw.GetDocuments() or []:
        m=wrap(raw,sld.IModelDoc2);path=Path(m.GetPathName())
        if not m.GetPathName() or not path.is_relative_to(ROOT):continue
        if 'INCOMPLETE_' in path.name or 'probe' in path.name:continue
        documents.append((path,m))
    def order(row):
        p,m=row
        return (0 if p.stem=='V10_master_layout' else 1 if p.suffix.upper()=='.SLDPRT' else 2 if p.stem=='V10_dummy_motor' else 3,str(p))
    result=[]
    for path,m in sorted(documents,key=order):
        sw.ActivateDoc3(m.GetTitle(),False,0,0);rebuild(m)
        if path.suffix.upper()=='.SLDASM':
            if M.mate_errors(m):raise RuntimeError('Mate errors in '+str(path))
        else:
            for f in walk_features(m):
                if f.GetTypeName2() in ('ProfileFeature','3DProfileFeature') and not f.Name.endswith('-LINK_Master_layout') and wrap(f.GetSpecificFeature2(),sld.ISketch).GetConstrainedStatus()!=3:
                    raise RuntimeError('Under-defined sketch '+str(path)+' '+f.Name)
                if f.GetErrorCode2(0)[0] and not f.IsSuppressed():raise RuntimeError('Feature error '+str(path)+' '+f.Name)
        ok,e,w=m.Save3(1,0,0)
        if not ok or e:raise RuntimeError('Save failed '+str(path)+' '+str((e,w)))
        result.append(str(path));print('Saved',path.name,flush=True)
    (ROOT/'validation/saved_chain.json').write_text(json.dumps(result,indent=2))
    return result

if __name__=='__main__':save_chain(connect())
