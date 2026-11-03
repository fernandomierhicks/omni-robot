"""Catch repeated primitives SolidWorks left with generic feature names."""
from find_shared_holes import *

if __name__=='__main__':
    sw=connect();plan=json.loads((ROOT/'reference/shared_holes.json').read_text());path=ROOT/'reference/shared_hole_candidates.json';old=json.loads(path.read_text())
    names=sys.argv[1:] or sorted({r['part'] for r in old['candidates']})
    new=scan(sw,plan,names)
    existing={(r['part'],r['feature']) for r in old['candidates']}
    def signature(r):return json.dumps([r['pattern'],r['projection'],r['projection_offset'],r.get('dynamic_origin')],sort_keys=True)
    masters={signature(r):r['master_feature'] for r in old['candidates'] if r['kind']=='circle'}
    additional=[r for r in new['candidates'] if (r['part'],r['feature']) not in existing]
    for r in additional:
        if r['kind']=='circle':r['master_feature']=masters[signature(r)]
    old['candidates']+=additional
    path.write_text(json.dumps(old,indent=2));(ROOT/'validation/additional_shared_tools.json').write_text(json.dumps(additional,indent=2))
    print('Additional shared tools',len(additional),flush=True)
