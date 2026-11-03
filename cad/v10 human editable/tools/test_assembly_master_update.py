"""A real master edit must also reposition the mated assembly fastener."""
from sw_native import *
from update_native import open_part
from save_chain import save_chain
import mate_api as M
import numpy as np

if __name__=='__main__':
    sw=connect();raw,_,_=sw.OpenDoc6(str(ROOT/'V10_wheel_pod.SLDASM'),2,1,'',0,0);assembly=wrap(raw,sld.IModelDoc2)
    master=open_part(sw,'master_layout');cup=open_part(sw,'motor_cup');fork=open_part(sw,'inner_fork_6804')
    names=json.loads((ROOT/'validation/V10_wheel_pod_progress.json').read_text());cp=M.comps(assembly)[names['bolt_M3_25_PAN_1']]
    for m in (master,cup,fork,assembly):rebuild(m)
    initial=placement(cp);dim=wrap(master.Parameter('D4@MASTER_Cradle_to_fork'),sld.IDimension);value=dim.SystemValue
    report={'component':cp.Name2,'master_delta_y_mm':.2}
    try:
        dim.SetSystemValue3(value+.0002,1,None);rebuild(master);master.Save3(1,0,0)
        for m in (cup,fork,assembly):rebuild(m)
        actual=placement(cp);delta=actual[:,3]-initial[:,3];errors=M.mate_errors(assembly)
        report.update(actual_delta_mm=delta.tolist(),mate_errors=errors,pass_=bool(np.max(abs(delta-np.array([0,.2,0])))<.002 and not errors))
        if not report['pass_']:raise RuntimeError('Assembly fastener did not follow the master hole')
    finally:
        dim.SetSystemValue3(value,1,None);rebuild(master);master.Save3(1,0,0)
        for m in (cup,fork,assembly):rebuild(m)
        report['restored']=bool(np.max(abs(placement(cp)-initial))<.002)
        save_chain(sw)
        (ROOT/'validation/assembly_master_update.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
