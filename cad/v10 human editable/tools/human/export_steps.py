"""SolidWorks STEP exports of the 3D PRINTED parts only (no fasteners, washers, nuts or bearings). Part files; the assemblies are built by export_wheel_step.py. Run in the cadenv Python."""
from common import *
OUTD=ROOT/'step_export';(OUTD/'dummy_motor').mkdir(parents=True,exist_ok=True);(OUTD/'wheel_no_hubs'/'parts').mkdir(parents=True,exist_ok=True)
PARTS={'dummy_motor':['dummy_motor_stator','dummy_motor_rotor'],
       'wheel_no_hubs/parts':['roller_core','roller_tread','wheel_cartridge']}
sw=connect()
for folder,keys in PARTS.items():
    for key in keys:
        m=open_owned(sw,key);sw.ActivateDoc3(m.GetTitle(),False,0,0)
        ok,e,w=m.Extension.SaveAs3(str(OUTD/folder/('V10H_'+key+'.step')),0,3,None,None,0,0)
        if not ok or e:raise RuntimeError('STEP export failed '+key+' '+str((e,w)))
        sw.CloseDoc(m.GetTitle());print('exported',folder,key,flush=True)
