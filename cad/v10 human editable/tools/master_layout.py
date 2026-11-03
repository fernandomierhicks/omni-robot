"""One native, dimensioned coordinate sketch per shared fastener pattern.

Slave parts derive this part's sketches and use concentric relations. The
reference part contains no solids and no references back to its consumers.
"""
from sw_native import *

PATTERNS={'Cradle_to_fork':[(0,6.5),(-6.5,0),(0,-6.5),(6.5,0)]}

def sketch_pattern(p,name,points,prefix='MASTER_',offset=100):
    m=p.model;sm=m.SketchManager;m.ClearSelection2(True)
    m.Extension.SelectByID2('Front Plane','PLANE',0,0,0,False,0,None,0);sm.InsertSketch(True);sm.AddToDB=True
    datum_arc=sm.CreateCircleByRadius(-offset*MM,-offset*MM,0,.00025)
    datum=wrap(wrap(datum_arc,sld.ISketchArc).GetCenterPoint2(),sld.ISketchPoint)
    arcs=[sm.CreateCircleByRadius(x*MM,y*MM,0,.0005) for x,y in points]
    # Reference datum dimensions to another sketch entity, not the model
    # origin: InsertPart3 drops dimensions involving the model origin.
    origin_arc=sm.CreateCircleByRadius(0,0,0,.00025)
    origin=wrap(wrap(origin_arc,sld.ISketchArc).GetCenterPoint2(),sld.ISketchPoint)
    sm.AddToDB=False;sk=wrap(sm.ActiveSketch,sld.ISketch)
    m.ClearSelection2(True);origin.Select4(False,None)
    m.Extension.SelectByID2('Point1@Origin','EXTSKETCHPOINT',0,0,0,True,0,None,0)
    m.SketchAddConstraints('sgCOINCIDENT')
    coordinate_names=[];datum_names=[]
    for i,pt in enumerate([datum]+[wrap(wrap(arc,sld.ISketchArc).GetCenterPoint2(),sld.ISketchPoint) for arc in arcs]):
        is_datum=i==0;names=[]
        for axis in (0,1):
            m.ClearSelection2(True);pt.Select4(False,None)
            if is_datum:origin.Select4(True,None)
            else:datum.Select4(True,None)
            dd=(m.AddHorizontalDimension2 if axis==0 else m.AddVerticalDimension2)(pt.X+.004,pt.Y+.004,0)
            if dd is None:raise RuntimeError('Master coordinate dimension failed')
            dim=wrap(wrap(dd,sld.IDisplayDimension).GetDimension2(0),sld.IDimension)
            names.append(dim.FullName.split('@')[0])
        if not is_datum:coordinate_names.append(names)
        else:datum_names=names
    for obj in sk.GetSketchSegments():
        seg=wrap(obj,sld.ISketchSegment);m.ClearSelection2(True);seg.Select4(False,None)
        dd=m.AddDiameterDimension2(.001,.001,0)
        if dd is None:raise RuntimeError('Master marker diameter')
        seg.ConstructionGeometry=True
    status=sk.GetConstrainedStatus();sm.InsertSketch(True);f=last(m);f.Name=prefix+name
    if status!=3:raise RuntimeError('Master sketch under-constrained '+str(status))
    p.sketches.append({'name':f.Name,'status':status})
    return {'feature':f.Name,'coordinate_dimensions':[[d+'@'+f.Name for d in names] for names in coordinate_names],
            'datum_dimensions':[d+'@'+f.Name for d in datum_names],'segment_offset':1,'offset_mm':offset}

def build(sw):
    p=NativePart(sw,'master_layout')
    source=ROOT/'reference/shared_holes.json'
    plan=json.loads(source.read_text()) if source.exists() else {'patterns':{n:{'points':v} for n,v in PATTERNS.items()}}
    meta={'patterns':{},'views':{}}
    for name,spec in plan['patterns'].items():meta['patterns'][name]=sketch_pattern(p,name,spec['points'])
    em=wrap(p.model.GetEquationMgr(),sld.IEquationMgr);em.AutomaticRebuild=True;em.AutomaticSolveOrder=True
    # The pin's transverse coordinate belongs to the suspension pivot, not a
    # second independently editable copy of its Y coordinate.
    if 'Shaft_pin' in meta['patterns']:
        dst=meta['patterns']['Shaft_pin']['coordinate_dimensions'][0][1]
        src=meta['patterns']['Pivot']['coordinate_dimensions'][0][1]
        pd=meta['patterns']['Pivot']['datum_dimensions'][1];sd=meta['patterns']['Shaft_pin']['datum_dimensions'][1]
        if em.Add2(-1,f'"{dst}" = "{sd}" + "{src}" - "{pd}"',True)<0:raise RuntimeError('Pin/pivot master equation')
    views_path=ROOT/'reference/shared_hole_candidates.json'
    if views_path.exists():
        views_data=json.loads(views_path.read_text());candidates=views_data['candidates']
        done={}
        for candidate in candidates:
            if candidate['kind']!='circle':continue
            pattern=candidate['pattern'];C=candidate['projection'];k=candidate['projection_offset']
            if max(abs(C[i][j]-(1 if i==j else 0)) for i in range(2) for j in range(2))<1e-7 and max(abs(v) for v in k)<1e-7:
                candidate['master_feature']=meta['patterns'][pattern]['feature'];continue
            signature=json.dumps([pattern,C,k,candidate.get('dynamic_origin')],sort_keys=True)
            if signature not in done:
                name=f'{len(done)+1:03d}_{pattern}'
                pts=[[sum(C[i][j]*q[j] for j in range(2))+k[i] for i in range(2)] for q in plan['patterns'][pattern]['points']]
                view=sketch_pattern(p,name,pts,prefix='VIEW_',offset=1000)
                for j,coord_names in enumerate(view['coordinate_dimensions']):
                    for i,dst in enumerate(coord_names):
                        terms=[f'{k[i]:.12g}mm',f'"{view["datum_dimensions"][i]}"']
                        for axis in (0,1):
                            if abs(C[i][axis])>1e-12:
                                src=meta['patterns'][pattern]['coordinate_dimensions'][j][axis]
                                datum=meta['patterns'][pattern]['datum_dimensions'][axis]
                                terms.append(f'({C[i][axis]:.12g})*("{src}" - "{datum}")')
                        if candidate.get('dynamic_origin')=='pivot_shaft' and i==1:
                            src=meta['patterns']['Pivot']['coordinate_dimensions'][0][1]
                            datum=meta['patterns']['Pivot']['datum_dimensions'][1]
                            terms.append(f'-("{src}" - "{datum}" - 68.2mm)')
                        equation=f'"{dst}" = '+' + '.join(terms)
                        if em.Add2(-1,equation,True)<0:raise RuntimeError('View equation '+equation)
                done[signature]=view;meta['views'][view['feature']]=view
            candidate['master_feature']=done[signature]['feature']
        views_data['candidates']=candidates;views_path.write_text(json.dumps(views_data,indent=2))
    em.EvaluateAll()
    m=p.model;m.FeatureManager.EnableFeatureTree=True;m.FeatureManager.EnableFeatureTreeWindow=True
    m.ShowNamedView2('*Front',1);m.ViewZoomtofit2()
    path=ROOT/'parts/V10_master_layout.SLDPRT'
    ok,e,w=m.Extension.SaveAs3(str(path),0,1,None,None,0,0)
    if not ok or e:raise RuntimeError('Master save '+str((e,w)))
    (ROOT/'validation/master_layout.json').write_text(json.dumps({'path':str(path),'patterns':plan['patterns'],'sketches':p.sketches,'references':meta},indent=2))
    return p

if __name__=='__main__':build(connect())
