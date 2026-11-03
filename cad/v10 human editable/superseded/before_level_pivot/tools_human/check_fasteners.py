"""Apply the repository's engagement/access checks to the native part exports."""
from pathlib import Path
import sys,json,importlib.util
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT.parent/'v9'))
import build123d as b
from OCP.gp import gp_Trsf
import assembly as A,hw
spec=importlib.util.spec_from_file_location('v9_build_checks',ROOT.parent/'v9/build.py');B=importlib.util.module_from_spec(spec);spec.loader.exec_module(B)

def items():
    manifest=json.loads((ROOT/'reference/assembly_manifest.json').read_text());shapes={};out=[]
    for row in manifest['parts']:
        name=row['part']
        if name not in shapes:shapes[name]=b.import_step(ROOT/'validation/human'/(name+'.step'))
        tr=gp_Trsf();tr.SetValues(*[v for r in row['matrix'][:3] for v in r]);loc=b.Location(tr)
        kind='fastener' if name.startswith(('bolt_','nut_','washer_')) else 'purchased' if name.startswith('bearing_') or name=='pivot_shaft' else 'printed'
        key=name;probe=None;size=''
        if name.startswith('bolt_'):
            _,size,length,head=name.split('_');size=size.replace('p','.');key=f'{size} x {length} {head}'
            probe=hw._screw_shape(head,float(length),size)[1].moved(loc)
        elif name.startswith('nut_'):key='M3 '+name.split('_')[-1]+' nut'
        elif name=='pivot_shaft':key='shaft 7 x 98'
        out.append(A.Item(name=row['key'],kind=kind,group=row['parent'] or row['role'],shape=shapes[name].moved(loc),key=key,
                          probe=probe,size=size,loc=loc))
    return out

if __name__=='__main__':
    its=items();print('Checking',len(its),'native components',flush=True)
    eng=B.engagement(its);print('Thread engagement:',len(eng),'screws;',len([r for r in eng if not r['ok']]),'short',flush=True)
    (ROOT/'validation/human/fastener_engagement.json').write_text(json.dumps(eng,indent=2))
    access=B.access(its);bad=[r for r in access if r['problems']]
    (ROOT/'validation/human/fastener_access.json').write_text(json.dumps(access,indent=2))
    print('Access:',len(access),'screws;',len(bad),'problems',flush=True)
    print(json.dumps({'short':[r for r in eng if not r['ok']],'access_problems':bad},indent=2),flush=True)
    if any(not r['ok'] for r in eng) or bad:raise SystemExit(1)

