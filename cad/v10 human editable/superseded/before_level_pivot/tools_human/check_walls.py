"""Repository ray-cast wall scan on the native SolidWorks exports."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT.parent/'v9'))
import build123d as b
import walls
from assembly import PrintedPart

if __name__=='__main__':
    names=sorted({r['part'] for r in json.loads((ROOT/'reference/assembly_manifest.json').read_text())['parts']
                  if not r['part'].startswith(('bolt_','nut_','washer_','bearing_')) and r['part']!='pivot_shaft'})
    names=sys.argv[1:] or names
    output=ROOT/'validation/human/wall_scan.json'
    rows=json.loads(output.read_text()) if output.exists() else {}
    for name in names:
        shape=b.import_step(ROOT/'validation/human'/(name+'.step'))
        material='TPU medium' if name=='roller_tread' else 'PLA' if name.startswith(('wheel_','roller_')) else 'PETG'
        row=walls.report({name:PrintedPart(name,shape,material,100,'native contract frame')})[name]
        rows[name]=row;output.write_text(json.dumps(rows,indent=2))
        print(name,'minimum',row['t_min'],'thin clusters',len(row['thin']),flush=True)

