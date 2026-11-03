"""Create a readable index of the single-source coordinate dimensions."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[1]

if __name__=='__main__':
    data=json.loads((ROOT/'validation/master_layout.json').read_text())
    lines=['# Editing shared hole locations','',
           'Open `parts/V10_master_layout.SLDPRT` and edit the appropriate `MASTER_` sketch. Each master circle marks one shared axis. Slave circles are concentric to the derived master; their own diameter and depth remain independently editable. `VIEW_` sketches are calculated from the primary sketches.','',
           'The construction datum is at (−100, −100) mm. Leave its two 100 mm dimensions unchanged. A point at coordinate 6.5 mm therefore has a displayed location dimension of 106.5 mm; a coordinate of 0 has a location dimension of 100 mm. This allows either coordinate to move off an axis without removing a constraint. The table gives actual coordinates and the matching dimensions.','',
           'For axes along the wheel axle, the sketch coordinates are u = −pod Z, v = pod Y. Vertical screw patterns use u = pod X, v = pod Y. The fork-bar pattern uses u = pod X, v = pod Z. Roller axes share one row coordinate and a rotated view at each roller.','',
           'Rebuild and save the master, affected parts, then containing assemblies. Open the pod assembly while editing to load all consumers. The provided `tools/save_chain.py` saves only this v10 tree, in dependency order. Keep all files together; copying just the assembly loses part and master references.','',
           '| Master sketch | Point | Actual u, v (mm) | u dimension | v dimension |','| --- | --- | --- | --- | --- |']
    for name,spec in data['patterns'].items():
        refs=data['references']['patterns'][name]
        for i,(pt,dims) in enumerate(zip(spec['points'],refs['coordinate_dimensions']),1):
            lines.append(f'| MASTER_{name} | {i} | {pt[0]:.6g}, {pt[1]:.6g} | `{dims[0]}` | `{dims[1]}` |')
    (ROOT/'MASTER_COORDINATES.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
