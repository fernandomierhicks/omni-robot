from emit_recipe import *
sw=connect();data=json.loads((ROOT/'reference/fork_bar.json').read_text());data['name']='move_probe'
p=Emitter(sw,data);names=p.emit(2)
for b in bodies(p.model):print('BOX',b.GetBodyBox(),'MASS',b.GetMassProperties(1))
f=wrap(p.model.FirstFeature(),sld.IFeature)
while f:
    if 'MoveCopy' in f.GetTypeName2() or 'Rotate' in f.Name or 'Translate' in f.Name:
        d=wrap(f.GetDefinition(),sld.IMoveCopyBodyFeatureData);print('def',f.Name,f.GetErrorCode2(0),d.TransformType,d.TransformX,d.TransformY,d.TransformZ)
    f=wrap(f.GetNextFeature(),sld.IFeature)
