"""Verify the same master edit moves both countersinks and clearance pilots."""
from sw_native import *
from update_native import open_part
from verify_shared_tools import position
import numpy as np

if __name__=='__main__':
    sw=connect();master=open_part(sw,'master_layout');meta=json.loads((ROOT/'validation/master_layout.json').read_text())
    items=[r for r in json.loads((ROOT/'reference/shared_hole_candidates.json').read_text())['candidates'] if r['pattern']=='Motor_cover' and r['point_index']==0]
    models={n:open_part(sw,n) for n in sorted({r['part'] for r in items})}
    for m in models.values():rebuild(m)
    before={r['part']+'/'+r['feature']:position(models[r['part']],r) for r in items}
    dim=wrap(master.Parameter(meta['references']['patterns']['Motor_cover']['coordinate_dimensions'][0][0]),sld.IDimension);v=dim.SystemValue
    report={'delta_master_u_mm':.2,'tools':[]}
    try:
        dim.SetSystemValue3(v+.0002,1,None);master.ForceRebuild3(False);master.Save3(1,0,0)
        for m in models.values():rebuild(m)
        for r in items:
            delta=position(models[r['part']],r)-before[r['part']+'/'+r['feature']];expected=np.array(r['world_to_part_U'])@np.array([.2,0])
            row={'part':r['part'],'feature':r['feature'],'kind':r['kind'],'actual_delta_mm':delta.tolist(),'expected_delta_mm':expected.tolist(),'pass':bool(max(abs(delta-expected))<1e-6)}
            report['tools'].append(row)
        report['pass']=all(r['pass'] for r in report['tools'])
        print(json.dumps(report),flush=True)
        if not report['pass']:raise RuntimeError('Countersink/clearance master propagation failed')
    finally:
        dim.SetSystemValue3(v,1,None);master.ForceRebuild3(False);master.Save3(1,0,0)
        for m in models.values():rebuild(m);m.Save3(1,0,0)
        report['restored']=all(bool(max(abs(position(models[r['part']],r)-before[r['part']+'/'+r['feature']]))<1e-6) for r in items)
        (ROOT/'validation/cover_master_update_test.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
