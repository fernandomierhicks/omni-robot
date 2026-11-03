"""With the pod at several suspension heights and wheel angles: the stator stays on the fork, the rotor turns with the wheel,
and no dummy-motor part interferes with any other component."""
from common import *
sys.path.insert(0,str(ROOT/'tools'))
import mate_api as M
import numpy as np
from importlib import import_module
from sw_native import placement
T=import_module('test_assembly_motion')
MOTOR=('dummy_motor_stator','dummy_motor_rotor','washer_M3_large','bearing_687')

def motor_interferences(m):
    asm=wrap(m,sld.IAssemblyDoc);mgr=wrap(asm.InterferenceDetectionManager,sld.IInterferenceDetectionMgr)
    mgr.TreatCoincidenceAsInterference=False;mgr.TreatSubAssembliesAsComponents=True;mgr.IncludeMultibodyPartInterferences=False
    mgr.MakeInterferingPartsTransparent=False;mgr.CreateFastenersFolder=False;mgr.IgnoreHiddenBodies=True;mgr.ShowIgnoredInterferences=False
    mgr.UseTransform=False;mgr.NonInterferingComponentDisplay=0
    out=[]
    for raw in mgr.GetInterferences() or []:
        i=wrap(raw,sld.IInterference);names=[wrap(x,sld.IComponent2).Name2 for x in i.Components]
        if True:out.append((names,round(i.Volume*1e9,3)))
    mgr.Done();return out

sw=connect();raw,e,w=sw.OpenDoc6(str(ROOT/'V10H_wheel_pod.SLDASM'),2,1,'',0,0)
m=wrap(raw,sld.IModelDoc2);sw.ActivateDoc3(m.GetTitle(),False,0,0)
progress=json.loads((REPORTS/'V10H_wheel_pod_progress.json').read_text());cs=M.comps(m);cp=lambda k:cs[progress[k]]
plane=lambda k,p:f'{p}@{cp(k).Name2}@V10H_wheel_pod'
py,pz=json.loads((ROOT/'reference/assembly_manifest.json').read_text())['pivot']
report={'rows':[]}
base=motor_interferences(m);report['baseline_nominal']=base;print('nominal',base,flush=True)
sus,sdim=T.angle_driver(sw,m,plane('Inner_fork','Top Plane'),plane('Guard','Front Plane'),90,'VERIFY_Suspension')
spin,pdim=T.angle_driver(sw,m,plane('Hub_B','Top Plane'),plane('Inner_fork','Front Plane'),90,'VERIFY_Spin')
try:
    for bump in (-3,7.5,15):
        for ang in (90,150):
            a=math.asin((bump-pz)/math.hypot(py,pz))+math.atan2(pz,py)
            sdim.SetSystemValue3(math.pi/2+a,1,None);pdim.SetSystemValue3(math.radians(ang),1,None);m.EditRebuild3()
            st=placement(cp('Motor_stator'));ro=placement(cp('Motor_rotor'));fk=placement(cp('Inner_fork'))
            row={'bump_mm':bump,'wheel_deg':ang,'stator_rel_fork_err':float(np.max(abs(np.linalg.inv(np.vstack([fk,[0,0,0,1]]))@np.vstack([st,[0,0,0,1]])-np.linalg.inv(np.vstack([np.array(json.loads((REPORTS/'V10H_wheel_pod.json').read_text())['components'][cp('Inner_fork').Name2]['transform']),[0,0,0,1]]))@np.vstack([np.array(json.loads((REPORTS/'V10H_wheel_pod.json').read_text())['components'][cp('Motor_stator').Name2]['transform']),[0,0,0,1]])))),
                 'rotor_axis_change_vs_stator':float(np.max(abs((np.linalg.inv(np.vstack([st,[0,0,0,1]]))@np.vstack([ro,[0,0,0,1]]))[:3,:3]-np.eye(3)))),'mate_errors':M.mate_errors(m),'interferences':motor_interferences(m)}
            report['rows'].append(row);print(row,flush=True)
finally:
    sdim.SetSystemValue3(math.pi/2,1,None);pdim.SetSystemValue3(math.pi/2,1,None);m.EditRebuild3();T.remove(m,spin);T.remove(m,sus)
report['pass']=not base and all(not r['interferences'] and not r['mate_errors'] for r in report['rows'])
if any(f.Name.startswith('VERIFY_') for f in M.mates_of(m)):raise RuntimeError('Temporary driver left in assembly')
ok,e,w=m.Save3(1,0,0)
(REPORTS/'pod_interference_all.json').write_text(json.dumps(report,indent=2));print('PASS' if report['pass'] else 'FAIL',flush=True)
