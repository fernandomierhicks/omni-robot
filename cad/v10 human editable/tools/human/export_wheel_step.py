"""Hub-free wheel as one STEP assembly of the 3D printed parts only: rollers (core + tread) and the bearing cartridge, at their nominal
positions in the pod frame (wheel axis = X). No hubs, fasteners, nuts or bearings."""
import sys, json
from pathlib import Path
import numpy as np, build123d as b
from scipy.spatial.transform import Rotation as R
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'tools/human'))
PARTS=ROOT/'step_export/wheel_no_hubs/parts'
COLOUR={'roller_core':(.97,.95,.70),'roller_tread':(.84,.84,.90),'wheel_cartridge':(.40,.90,.85)}
KEEP=set(COLOUR)
manifest=json.loads((ROOT/'reference/assembly_manifest.json').read_text())
cache={}
def part(name):
    if name not in cache:cache[name]=b.import_step(PARTS/('V10H_'+name+'.step'))
    return cache[name]
def location(M):
    M=np.array(M);e=R.from_matrix(M[:3,:3]).as_euler('xyz',degrees=True);return b.Pos(*M[:3,3])*b.Rot(*e)
children=[]
for row in manifest['parts']:
    if row['part'] not in KEEP:continue
    s=location(row['matrix'])*part(row['part']);s=b.Compound(s.wrapped);s.label=row['key'];s.color=b.Color(*COLOUR[row['part']]);children.append(s)
wheel=b.Compound(children=children,label='V10H_wheel_no_hubs')
out=ROOT/'step_export/wheel_no_hubs/V10H_wheel_no_hubs_assembly.step'
b.export_step(wheel,str(out))
import collections;print(len(children),'components',dict(collections.Counter(r['part'] for r in manifest['parts'] if r['part'] in KEEP)))
back=b.import_step(str(out));print('re-imported solids',len(back.solids()),'volume mm3',round(sum(s.volume for s in back.solids()),1))

# the dummy motor (stator + rotor only), motor frame
PD=ROOT/'step_export/dummy_motor'
motor=b.Compound(children=[],label='V10H_dummy_motor')
kids=[]
for key,col in (('dummy_motor_stator',(1.0,.68,.30)),('dummy_motor_rotor',(.35,.85,.95))):
    s=b.Compound(b.import_step(PD/('V10H_'+key+'.step')).wrapped);s.label=key;s.color=b.Color(*col);kids.append(s)
b.export_step(b.Compound(children=kids,label='V10H_dummy_motor'),str(PD/'V10H_dummy_motor_assembly.step'));print('dummy motor assembly: stator + rotor')
