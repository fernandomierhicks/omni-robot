"""Wheel-only, single-material X2D print plates; no geometry changes."""
from pathlib import Path
import json, math, os, hashlib
import xml.etree.ElementTree as ET
from zipfile import ZipFile, ZIP_DEFLATED
import numpy as np
import trimesh
from shapely.geometry import Polygon, box
from shapely.ops import unary_union
from shapely.affinity import rotate, translate

OUT = Path(__file__).with_name('out')
KIT = OUT / 'wheel_test_6804'
DEST = OUT / 'x2d_plates'
NS = 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
ET.register_namespace('', NS)


# The kit's STLs have every hub flat (axle vertical): layers lie in the plane
# that carries the wheel's loads, which matters more than support. Hub A is
# turned over (180 deg about X) because that face down needs the least overhang.
FLIP_X = {'wheel_hub_A'}


def mesh(name):
    m = trimesh.load(KIT / (name + '.stl'))
    if name in FLIP_X:
        m.vertices = m.vertices * np.array([1, -1, -1])
        m.vertices[:, 2] -= m.vertices[:, 2].min()
    return m


def footprint(m):
    # Fill internal holes: reserve them for support/access, never nest parts there.
    tris = m.triangles[:, :, :2]
    polys = [Polygon(t) for t in tris if abs(np.linalg.det(np.array([t[1]-t[0], t[2]-t[0]]))) > 1e-8]
    p = unary_union(polys).simplify(.04)
    return Polygon(p.exterior)


def profiles(material):
    root = Path(os.environ['APPDATA']) / 'BambuStudio/system/BBL'
    files = {p.stem: p for p in root.rglob('*.json')}
    def resolve(name):
        d = json.loads(files[name].read_text(encoding='utf-8'))
        base = resolve(d['inherits']) if d.get('inherits') else {}
        for inc in d.get('include', []):
            base.update(resolve(inc))
        base.update(d)
        return base
    machine_name = 'Bambu Lab X2D 0.4 nozzle'
    process_name = '0.20mm Standard @BBL X2D'
    filament_name = f'Generic {material} @BBL X2D'
    if filament_name not in files:
        filament_name = next(n for n in files if n.startswith(f'Generic {material} @BBL X2D'))
    result = {}
    for name in (machine_name, process_name, filament_name):
        result.update({k: v for k, v in resolve(name).items()
                       if k not in ('type','name','inherits','include','from','setting_id','instantiation','description','compatible_printers','compatible_printers_condition','compatible_prints','compatible_prints_condition')})
    result.update(printer_settings_id=machine_name, print_settings_id=process_name,
                  filament_settings_id=[filament_name], filament_type=[material],
                  filament_colour=['#ED8A25' if material == 'PLA' else '#333333'],
                  curr_bed_type='Textured PEI Plate', print_sequence='by layer',
                  layer_height='0.2', initial_layer_print_height='0.2',
                  wall_loops='6' if material == 'PLA' else '4',
                  sparse_infill_density='60%' if material == 'PLA' else '100%',
                  sparse_infill_pattern='gyroid', top_shell_layers='5', bottom_shell_layers='5',
                  enable_support='1' if material == 'PLA' else '0',
                  support_type='tree(auto)', support_on_build_plate_only='1',
                  support_threshold_angle='30', support_top_z_distance='0.2',
                  support_interface_top_layers='3', brim_type='outer_only',
                  brim_width='3', brim_object_gap='0.15',
                  enable_prime_tower='0', timelapse_type='0')
    # Arbitrary medium-soft TPU uses the main direct-drive nozzle. These are
    # editable starting settings, not a validated filament preset.
    result['filament_extruder_variant'] = ['Direct Drive Standard']
    if material == 'TPU':
        result.update(outer_wall_speed=['25']*6, inner_wall_speed=['30']*6,
                      sparse_infill_speed=['30']*6, internal_solid_infill_speed=['30']*6,
                      top_surface_speed=['25']*6, initial_layer_speed=['20']*6,
                      filament_max_volumetric_speed=['2'])
    return result


def pack():
    names = ['wheel_hub_A', 'wheel_hub_B', 'wheel_hub_C', 'wheel_cartridge', 'roller_core']
    meshes = {n:mesh(n) for n in names}
    polys = {n:footprint(m) for n,m in meshes.items()}
    bed = box(6,6,250,250)
    # Actual silhouettes allow the hub lobes to interleave without nesting in
    # their centre holes. Six mm model separation reserves room for small brims.
    first=[('wheel_hub_A',70.,70.,0.),('wheel_hub_B',186.,186.,0.),('wheel_cartridge',195.,55.,0.)]
    second=[('wheel_hub_C',80.,128.,0.)]
    def fill(placed,count):
        shapes=[translate(rotate(polys[n],a,origin=(0,0)),x,y) for n,x,y,a in placed]
        for x in np.arange(23,234,3):
            for y in np.arange(23,234,3):
                q=translate(polys['roller_core'],x,y)
                if bed.covers(q) and all(q.distance(v)>=10 for v in shapes):
                    shapes.append(q);placed.append(('roller_core',float(x),float(y),0.))
                    count-=1
                    if not count:return
        raise RuntimeError('Insufficient core space')
    fill(first,6);fill(second,6)
    return meshes,[first,second]


def write_3mf(material, meshes, plates):
    root=ET.Element(f'{{{NS}}}model',unit='millimeter',attrib={'{http://www.w3.org/XML/1998/namespace}lang':'en-US'})
    ET.SubElement(root,f'{{{NS}}}metadata',name='Title').text=f'v9 wheel 6804 - {material} - X2D'
    ET.SubElement(root,f'{{{NS}}}metadata',name='Application').text='BambuStudio-02.03.00.00'
    resources=ET.SubElement(root,f'{{{NS}}}resources'); build=ET.SubElement(root,f'{{{NS}}}build')
    config=ET.Element('config')
    placements=[];plate_nodes=[]
    for pi,ps in enumerate(plates):
        plate=ET.SubElement(config,'plate')
        ET.SubElement(plate,'metadata',key='plater_id',value=str(pi+1))
        ET.SubElement(plate,'metadata',key='plater_name',value=f'{material} - plate {pi+1}')
        ET.SubElement(plate,'metadata',key='locked',value='false')
        plate_nodes.append(plate)
        placements.extend((pi,entry) for entry in ps)
    for i,(pi,(name,x,y,angle)) in enumerate(placements,1):
        m=meshes[name].copy()
        a=math.radians(angle); rot=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
        vertices=m.vertices@rot.T
        vertices += np.array([x+pi*307.2,y,-vertices[:,2].min()])
        obj=ET.SubElement(resources,f'{{{NS}}}object',id=str(i),type='model',name=f'{name}_{i:02d}')
        me=ET.SubElement(obj,f'{{{NS}}}mesh'); vs=ET.SubElement(me,f'{{{NS}}}vertices'); ts=ET.SubElement(me,f'{{{NS}}}triangles')
        for v in vertices:
            ET.SubElement(vs,f'{{{NS}}}vertex',**{k:f'{num:.6f}' for k,num in zip('xyz',v)})
        for f in m.faces:
            ET.SubElement(ts,f'{{{NS}}}triangle',**{f'v{k+1}':str(v) for k,v in enumerate(f)})
        ET.SubElement(build,f'{{{NS}}}item',objectid=str(i))
        o=ET.SubElement(config,'object',id=str(i))
        settings={'name':f'{name}_{i:02d}','extruder':'1'}
        if name=='roller_core': settings.update(wall_loops='4',sparse_infill_density='40%')
        if name=='wheel_cartridge': settings.update(sparse_infill_density='80%',enable_support='0')
        for k,v in settings.items(): ET.SubElement(o,'metadata',key=k,value=v)
        part=ET.SubElement(o,'part',id=str(i),subtype='normal_part')
        ET.SubElement(part,'metadata',key='name',value=name)
        ET.SubElement(part,'metadata',key='matrix',value='1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1')
        inst=ET.SubElement(plate_nodes[pi],'model_instance')
        for k,v in [('object_id',str(i)),('instance_id','0'),('identify_id',str(i))]: ET.SubElement(inst,'metadata',key=k,value=v)
    path=DEST/f'wheel_6804_{material}_X2D.3mf'
    with ZipFile(path,'w',ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml','<?xml version="1.0"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/><Default Extension="config" ContentType="application/octet-stream"/></Types>')
        z.writestr('_rels/.rels','<?xml version="1.0"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr('3D/3dmodel.model',ET.tostring(root,encoding='utf-8',xml_declaration=True))
        z.writestr('Metadata/model_settings.config',ET.tostring(config,encoding='utf-8',xml_declaration=True))
        z.writestr('Metadata/project_settings.config',json.dumps(profiles(material),indent=2))
    return path


def preview(material,meshes,placements,index):
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(8,8))
    ax.add_patch(plt.Rectangle((0,0),256,256,fill=False,color='black'))
    shapes=[]
    for i,(n,x,y,a) in enumerate(placements,1):
        p=translate(rotate(footprint(meshes[n]),a,origin=(0,0)),x,y)
        shapes.append(p)
        xx,yy=p.exterior.xy
        ax.fill(xx,yy,color='#ed8a25' if material=='PLA' else '#555555',alpha=.8)
        ax.text(x,y,n.replace('wheel_','').replace('roller_','')+f'\n{i}',ha='center',va='center',fontsize=7)
    gap=min(p.distance(q) for i,p in enumerate(shapes) for q in shapes[i+1:])
    bounds=unary_union(shapes).bounds
    assert min(bounds[:2])>=5.99 and max(bounds[2:])<=250.01
    assert gap>=6.99
    ax.set(xlim=(-5,261),ylim=(-5,261),aspect='equal',title=f'{material}: one wheel | {len(placements)} parts | min gap {gap:.1f} mm',xlabel='mm',ylabel='mm')
    fig.tight_layout(); fig.savefig(DEST/f'{material}_plate_{index}.png',dpi=140); plt.close(fig)
    return {'parts':len(placements),'min_model_gap_mm':gap,'bounds_mm':bounds,'placements':placements}


def main():
    DEST.mkdir(exist_ok=True)
    meshes,pla=pack()
    meshes['roller_tread']=mesh('roller_tread')
    tpu=[('roller_tread',59+46*i,82+46*j,0) for j in range(3) for i in range(4)]
    report={}
    for mat,plates in [('PLA',pla),('TPU',[tpu])]:
        path=write_3mf(mat,meshes,plates)
        report[mat]={'plates':[preview(mat,meshes,ps,i+1) for i,ps in enumerate(plates)]}
        report[mat]['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
        print(path,flush=True)
    (DEST/'layout_checks.json').write_text(json.dumps(report,indent=2),encoding='utf-8')


if __name__=='__main__': main()
