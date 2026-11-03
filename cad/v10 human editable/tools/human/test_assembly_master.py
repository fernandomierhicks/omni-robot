from common import *
from importlib import import_module
T=import_module('test_master_updates')
sys.path.insert(0,str(ROOT/'tools'))
import mate_api as M
from sw_native import placement
import numpy as np

sw=connect();raw,e,w=sw.OpenDoc6(str(ROOT/'V10H_wheel_pod.SLDASM'),2,1,'',0,0)
if raw is None:raise RuntimeError('Cannot open pod assembly')
assembly=wrap(raw,sld.IModelDoc2)
parts={}
for key in ('master_layout','motor_cup','inner_fork_6804'):
    raw,e,w=sw.OpenDoc6(str(OUT/('V10H_'+key+'.SLDPRT')),1,1,'',0,0)
    if raw is None:raise RuntimeError('Cannot open shared-layout consumer')
    parts[key]=wrap(raw,sld.IModelDoc2)
def rebuild_all():
    for key in ('motor_cup','inner_fork_6804'):T.rebuild(parts[key])
    sw.ActivateDoc3(assembly.GetTitle(),False,0,0);assembly.ForceRebuild3(False);assembly.EditRebuild3()
rebuild_all();names=json.loads((REPORTS/'V10H_wheel_pod_progress.json').read_text());cp=M.comps(assembly)[names['bolt_M3_25_PAN_1']];initial=placement(cp)
report={'master_control':'cradle_radius','nominal_mm':6.5,'test_mm':6.4,'component':cp.Name2}
try:
    T.set_master(parts['master_layout'],'cradle_radius',6.4);rebuild_all();actual=placement(cp);delta=actual[:,3]-initial[:,3]
    report.update(actual_delta_mm=delta.tolist(),mate_errors=M.mate_errors(assembly));report['pass']=bool(np.max(abs(delta-np.array([0,-.1,0])))<.002 and not report['mate_errors'])
    if not report['pass']:raise RuntimeError('Mated fastener did not follow the master')
finally:
    T.set_master(parts['master_layout'],'cradle_radius',6.5);rebuild_all();report['restored']=bool(np.max(abs(placement(cp)-initial))<.002)
    for key in ('motor_cup','inner_fork_6804'):
        m=parts[key];sw.ActivateDoc3(m.GetTitle(),False,0,0);ok,e,w=m.Save3(1,0,0)
        if not ok or e:raise RuntimeError('Cannot save restored consumer')
    sw.ActivateDoc3(assembly.GetTitle(),False,0,0);ok,e,w=assembly.Save3(1,0,0)
    if not ok or e:raise RuntimeError('Cannot save restored pod')
    (REPORTS/'assembly_master_update.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report),flush=True)
