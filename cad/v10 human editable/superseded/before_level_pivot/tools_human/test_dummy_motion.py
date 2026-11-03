from common import *
from importlib import import_module
T=import_module('test_assembly_motion')
sys.path.insert(0,str(ROOT/'tools'))
import mate_api as M
import numpy as np
from sw_native import placement

sw=connect();path=ROOT/'V10H_dummy_motor.SLDASM';raw,e,w=sw.OpenDoc6(str(path),2,1,'',0,0)
if raw is None:raise RuntimeError('Cannot open dummy assembly')
m=wrap(raw,sld.IModelDoc2);sw.ActivateDoc3(m.GetTitle(),False,0,0);asm=wrap(m,sld.IAssemblyDoc)
names=json.loads((REPORTS/'V10H_dummy_motor_progress.json').read_text());cs=M.comps(m);cp=lambda k:cs[names[k]]
plane=lambda k,p:f'{p}@{cp(k).Name2}@V10H_dummy_motor'
report={'spin':[],'end_float':[]};initial=placement(cp('Rotor'))
f,dim=T.angle_driver(sw,m,plane('Rotor','Right Plane'),plane('Stator','Top Plane'),90,'VERIFY_DummySpin')
try:
    for angle in (105,125,90):
        dim.SetSystemValue3(math.radians(angle),1,None);m.EditRebuild3();actual=placement(cp('Rotor'));errors=M.mate_errors(m)
        row={'angle_deg':angle,'rotation_change':float(np.max(abs(actual[:,:3]-initial[:,:3]))),'mate_errors':errors};row['pass']=not errors and (angle==90 or row['rotation_change']>.01);report['spin'].append(row)
        if not row['pass']:raise RuntimeError('Dummy spin test failed')
finally:dim.SetSystemValue3(math.pi/2,1,None);m.EditRebuild3();T.remove(m,f)
mater=M.Mater(sw,m);before=set(f.Name for f in M.mates_of(m));mater._select([plane('Rotor','Front Plane'),plane('Stator','Rotor Axial Datum Plane')])
mate,error=asm.AddMate5(5,c.swMateAlignCLOSEST,True,.01,0,0,0,0,0,0,0,False,False,0)
m.ClearSelection2(True);m.EditRebuild3();new=[f for f in M.mates_of(m) if f.Name not in before]
if mate is None or error!=1 or len(new)!=1:raise RuntimeError('Cannot add dummy float test driver')
f=new[0];f.Name='VERIFY_DummyFloat';md=wrap(f.GetSpecificFeature2(),sld.IMate2);dim=wrap(wrap(md.DisplayDimension,sld.IDisplayDimension).GetDimension2(0),sld.IDimension)
try:
    for z in (.05,.25,0):
        dim.SetSystemValue3((10+z)*MM,1,None);m.EditRebuild3();actual=placement(cp('Rotor'));errors=M.mate_errors(m)
        row={'requested_z_mm':z,'actual_z_mm':float(actual[2,3]),'mate_errors':errors};row['pass']=abs(actual[2,3]-z)<.002 and not errors;report['end_float'].append(row);print(row,flush=True)
        if not row['pass']:raise RuntimeError('Dummy outward float test failed')
finally:dim.SetSystemValue3(.01,1,None);m.EditRebuild3();T.remove(m,f)
report['restored']=bool(np.max(abs(placement(cp('Rotor'))-initial))<.002);report['pass']=report['restored'] and all(r['pass'] for group in ('spin','end_float') for r in report[group])
ok,e,w=m.Save3(1,0,0)
if not ok or e:raise RuntimeError('Cannot save restored dummy assembly')
(REPORTS/'dummy_motion.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
if not report['pass']:raise SystemExit(1)
