"""Reapply missed spoke cutters to the already-native, joined hub A.

The starting STEP is measurement-only. The emitted recipe starts with an
existing native body and creates native sketches/features for every cutter.
"""
import capture_recipe as R
import build123d as b
import wheel as W
import math,json

state_path=R.ROOT/'validation/RESUME_wheel_hub_A.json';state=json.loads(state_path.read_text())
shape=b.import_step(R.ROOT/'validation/hub_A_prefillet_chamfer_fixed.step')
shape=R.record(shape,{'op':'existing','bodies':state['resume']['bodies']})
for loc in W.roller_locs(0):
    tools=[W._roller_env(.8),b.Cylinder(1.7,51),
           b.Cylinder(3.6,45,align=(b.Align.CENTER,b.Align.CENTER,b.Align.MAX)).moved(b.Pos(0,0,-23.5)),
           b.extrude(b.RegularPolygon(5.8/math.sqrt(3),6),2.6).moved(b.Pos(0,0,23.5)),
           b.Cylinder(3.6,45,align=(b.Align.CENTER,b.Align.CENTER,b.Align.MIN)).moved(b.Pos(0,0,26.1))]
    for tool in tools:shape-=tool.moved(loc)
offset=len(state['nodes'])
for item in R.NODES:
    item=dict(item);item['id']+=offset
    for key in ('a','b'):
        if key in item:item[key]+=offset
    state['nodes'].append(item)
for item in state['nodes'][:offset]:
    if item['op']=='fillet' and item.get('a')==333:item['a']=offset+R.node(shape)
state['repair_note']='Native equal-distance chamfers corrected; spoke cutters reapplied after joining to avoid ambiguous symmetric-body ordering.'
state_path.write_text(json.dumps(state))
print('Repair continuation ready',len(R.NODES),'nodes, expected prefillet volume',shape.volume)
