from emit_recipe import *
sw=connect();data=json.loads((ROOT/'reference/wheel_hub_A.json').read_text());data['name']='chamfer_probe'
p=Emitter(sw,data);p.emit(8)
print('VOLUME',sum(b.GetMassProperties(1)[3]*1e9 for b in bodies(p.model)))
f=last(p.model);d=wrap(f.GetDefinition(),sld.IChamferFeatureData2)
print('CHAMFER',d.Type,d.EqualDistance,d.GetEdgeChamferDistance(0),d.GetEdgeChamferDistance(1))
print(p.model.Extension.SaveAs3(str(ROOT/'validation/chamfer_probe.step'),0,3,None,None,0,0))
