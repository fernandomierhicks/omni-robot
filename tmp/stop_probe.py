import sys
from pathlib import Path
sys.path.insert(0,str(Path('simulation/v11 sil').resolve()))
import test_v11 as T
import numpy as np
_, PR=T._sil()
rig=T.sil_rig()
p,m,d,r,link=rig
T.sil_run(rig,2,lambda t: PR.Command(drive_mode=PR.DRIVE_VEL_BODY,a=1.,flags=PR.CMD_HOLD_YAW))
t0=d.time
rows=[]
while d.time-t0<2:
    link.poll(d.time);r.step()
    rows.append((d.time-t0,np.linalg.norm(d.sensor('base_linvel').data[:2]),*r.fw.outputs.dbg_v_cmd))
a=np.array(rows)
for dt in (.001,.01,.05,.1):
    k=round(dt/m.opt.timestep)
    v=a[::k,1];u=np.linalg.norm(a[::k,2:4],axis=1)
    print(dt,'physical',max(-np.diff(v)/dt),'command',max(-np.diff(u)/dt))
np.save('tmp/stop_probe.npy',a)
