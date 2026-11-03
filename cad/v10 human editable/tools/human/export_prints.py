"""Mesh only the native SolidWorks check exports; no substitute CAD geometry."""
from pathlib import Path
import json,collections
import build123d as b
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'print/human';OUT.mkdir(parents=True,exist_ok=True)
manifest=json.loads((ROOT/'reference/assembly_manifest.json').read_text());counts=collections.Counter(r['part'] for r in manifest['parts'])
printed=lambda n:not n.startswith(('bolt_','nut_','washer_','bearing_')) and n not in ('pivot_shaft','fork_bar')
rows=['# Native v10 print list','','The STL files below are meshed from the editable SolidWorks parts in `parts/human/`. Dimensions are millimetres.','','| Part | Qty | Material | STL |','|---|---:|---|---|']
for name,qty in sorted(counts.items()):
    if not printed(name):continue
    shape=b.import_step(ROOT/'validation/human'/(name+'.step'))
    if not shape.is_valid or len(shape.solids())!=1:raise RuntimeError('Invalid native print '+name)
    path=OUT/(name+'.stl');b.export_stl(shape,path,tolerance=.01,angular_tolerance=.1)
    # parts touching the AK40 (warm inside the hub) are PETG, never PLA
    material='PETG (touches the motor)' if name in ('motor_cup','inner_fork_6804','wheel_hub_A') else 'Medium TPU' if name=='roller_tread' else 'PLA' if name.startswith(('wheel_','roller_')) else 'PETG / PLA for fit fixture'
    rows.append(f'| {name} | {qty} | {material} | [{name}.stl](print/human/{name}.stl) |')
    print(name,qty,flush=True)
(ROOT/'PRINT_LIST.md').write_text('\n'.join(rows)+'\n')
