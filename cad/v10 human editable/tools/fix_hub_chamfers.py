from sw_native import *
sw=connect();path=ROOT/'validation/INCOMPLETE_wheel_hub_A.SLDPRT'
raw,e,w=sw.OpenDoc6(str(path),1,1,'',0,0);m=wrap(raw,sld.IModelDoc2);sw.ActivateDoc3(m.GetTitle(),False,0,0)
ff=[];f=wrap(m.FirstFeature(),sld.IFeature)
while f:
    if f.GetTypeName2()=='Chamfer':ff.append(f.Name)
    f=wrap(f.GetNextFeature(),sld.IFeature)
for name in ff:
    f=wrap(wrap(m,sld.IPartDoc).FeatureByName(name),sld.IFeature);d=wrap(f.GetDefinition(),sld.IChamferFeatureData2)
    if not d.AccessSelections(m,None):raise RuntimeError('access '+name)
    d.Type=2;d.EqualDistance=True;d.SetEdgeChamferDistance(0,.001);d.SetEdgeChamferDistance(1,.001);d.TangentPropagation=False
    ok=f.ModifyDefinition(d,m,None);m.EditRebuild3();d=wrap(f.GetDefinition(),sld.IChamferFeatureData2)
    print(name,ok,f.GetErrorCode2(0),d.Type,d.GetEdgeChamferDistance(0),d.GetEdgeChamferDistance(1),flush=True)
    if not ok or f.GetErrorCode2(0)[0]:raise RuntimeError('chamfer edit failed')
state=json.loads((ROOT/'validation/RESUME_wheel_hub_A.json').read_text())
bs=bodies(m)
if len(bs)!=1:raise RuntimeError('body count')
bs[0].Name=state['resume']['bodies'][0]
print('volume',bs[0].GetMassProperties(1)[3]*1e9)
print(m.Extension.SaveAs3(str(path),0,1,None,None,0,0))
print(m.Extension.SaveAs3(str(ROOT/'validation/hub_A_prefillet_chamfer_fixed.step'),0,3,None,None,0,0))
