"""Interference check of the two-part dummy motor at both ends of the rotor end float and several spin angles."""
from common import *
sys.path.insert(0,str(ROOT/'tools'))
import mate_api as M
from importlib import import_module
T=import_module('test_assembly_motion')

def interferences(m):
    asm=wrap(m,sld.IAssemblyDoc);mgr=wrap(asm.InterferenceDetectionManager,sld.IInterferenceDetectionMgr)
    mgr.TreatCoincidenceAsInterference=False;mgr.TreatSubAssembliesAsComponents=True;mgr.IncludeMultibodyPartInterferences=False
    mgr.MakeInterferingPartsTransparent=False;mgr.CreateFastenersFolder=False;mgr.IgnoreHiddenBodies=True;mgr.ShowIgnoredInterferences=False
    mgr.UseTransform=False;mgr.NonInterferingComponentDisplay=0
    found=[]
    for raw in mgr.GetInterferences() or []:
        i=wrap(raw,sld.IInterference);names=[wrap(x,sld.IComponent2).Name2 for x in i.Components];found.append((names,i.Volume*1e9))
    mgr.Done();return found

sw=connect();raw,e,w=sw.OpenDoc6(str(ROOT/'V10H_dummy_motor.SLDASM'),2,1,'',0,0)
m=wrap(raw,sld.IModelDoc2);sw.ActivateDoc3(m.GetTitle(),False,0,0);asm=wrap(m,sld.IAssemblyDoc)
names=json.loads((REPORTS/'V10H_dummy_motor_progress.json').read_text());cs=M.comps(m);cp=lambda k:cs[names[k]]
plane=lambda k,p:f'{p}@{cp(k).Name2}@V10H_dummy_motor'
report={'interferences':[]}
mater=M.Mater(sw,m);before=set(f.Name for f in M.mates_of(m));mater._select([plane('Rotor','Front Plane'),plane('Stator','Rotor Axial Datum Plane')])
mate,error=asm.AddMate5(5,c.swMateAlignCLOSEST,True,.01,0,0,0,0,0,0,0,False,False,0)
m.ClearSelection2(True);m.EditRebuild3();new=[f for f in M.mates_of(m) if f.Name not in before]
f=new[0];f.Name='VERIFY_Float';md=wrap(f.GetSpecificFeature2(),sld.IMate2);dim=wrap(wrap(md.DisplayDimension,sld.IDisplayDimension).GetDimension2(0),sld.IDimension)
spin,sdim=T.angle_driver(sw,m,plane('Rotor','Right Plane'),plane('Stator','Top Plane'),90,'VERIFY_Spin')
try:
    for z in (0,.15,.3):
        for ang in (90,150):
            dim.SetSystemValue3((10+z)*MM,1,None);sdim.SetSystemValue3(math.radians(ang),1,None);m.EditRebuild3()
            row={'float_mm':z,'angle_deg':ang,'interferences':interferences(m)};report['interferences'].append(row);print(row,flush=True)
finally:
    sdim.SetSystemValue3(math.pi/2,1,None);dim.SetSystemValue3(.01,1,None);m.EditRebuild3();T.remove(m,spin);T.remove(m,f)
report['pass']=all(not r['interferences'] for r in report['interferences'])
ok,e,w=m.Save3(1,0,0)
(REPORTS/'dummy_interference.json').write_text(json.dumps(report,indent=2));print('PASS' if report['pass'] else 'FAIL',flush=True)
