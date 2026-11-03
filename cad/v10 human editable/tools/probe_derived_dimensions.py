from sw_native import *
sw=connect();m=wrap(sw.ActiveDoc,sld.IModelDoc2);part=wrap(m,sld.IPartDoc)
f=wrap(part.FeatureByName('MASTER_Cradle_to_fork-LINK_Master_layout'),sld.IFeature)
print('Feature',f)
dd=wrap(f.GetFirstDisplayDimension(),sld.IDisplayDimension)
while dd:
    d=wrap(dd.GetDimension2(0),sld.IDimension);print('DIM',d.FullName,d.SystemValue)
    dd=wrap(f.GetNextDisplayDimension(dd),sld.IDisplayDimension)
for name in ('D1@MASTER_Cradle_to_fork-LINK_Master_layout','D3@MASTER_Cradle_to_fork-LINK_Master_layout','D1@MASTER_Cradle_to_fork'):
    d=wrap(m.Parameter(name),sld.IDimension);print('PARAM',name,d, d.SystemValue if d else None)
