from common import *

def hexagon(m,E,plane,name,af,expr):
    s=Sketch(m,plane,name);s.polygon(0,0,af/math.sqrt(3),0,6)
    segs=s.segments();circ=next(wrap(x,sld.ISketchArc) for x in segs if x.GetType()==1)
    s.rel('sgCOINCIDENT',circ.GetCenterPoint2(),origin=True);s.ddim(circ,expr,(af+2,af+2))
    flat=next(x for x in segs if x.GetType()==0 and abs(wrap(x,sld.ISketchLine).GetStartPoint2().Y-wrap(x,sld.ISketchLine).GetEndPoint2().Y)<1e-8)
    s.rel('sgHORIZONTAL2D',flat);s.finish(E)
    if s.status()!=3:raise RuntimeError(name+' status '+str(s.status()))
    m.ClearSelection2(True);s.feat.Select2(False,0);return s

def bolt(sw,key):
    _,size,length,kind=key.split('_');length=float(length);small=size=='M2p5';sr=1 if small else 1.2
    hd,hh=(5,1.5) if small else ((5.5,1.65) if kind=='FLAT' else (6.5,2.5));af=1.5 if small else 2
    m,E,path=start(sw,key,'Screw Section')
    vals=[('nominal_length',length,'Screw length: under pan head or including flat head'),('model_shank_radius',sr,'Clearance envelope minor shank radius; thread is not modeled'),('head_diameter',hd,'Head outside diameter'),('head_height',hh,'Head axial height'),('drive_across_flats',af,'Hex drive across flats'),('tip_chamfer',.3,'Shank tip chamfer'),('head_round',hh*.25,'Pan head crown edge radius'),('drive_depth',hh*(.62 if kind=='FLAT' else .55),'Hex socket depth')]
    for n,v,t in vals:E.var(n,v,'mm',t)
    top=0 if kind=='FLAT' else -hh;seat=hh if kind=='FLAT' else 0
    vs=[(0,length,None,'nominal_length'),(sr,length,'model_shank_radius','nominal_length'),(sr,seat,'model_shank_radius','head_height' if seat else None),(hd/2,seat,'head_diameter / 2','head_height' if seat else None),(hd/2,top,'head_diameter / 2','head_height' if top else None),(0,top,None,'head_height' if top else None)]
    s=orthogonal_profile(m,E,'Top Plane','Screw Section',vs,axis='vertical');full_revolve(m,E,s,'Screw Blank Revolve')
    E.solve();m.ForceRebuild3(False)
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][2]+length)<1e-5 and abs(g[3]-sr)<1e-5)
    select_edges(m,es);f=chamfer_equal(m,.3,'Shank Tip Chamfer');link_feature(E,f,['tip_chamfer'])
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][2]-(-top))<1e-5 and abs(g[3]-hd/2)<1e-5)
    if kind=='PAN':
        select_edges(m,es);f=fillet_uniform(m,hh*.25,'Pan Head Round');link_feature(E,f,['head_round'])
    else:
        # The lower head rim becomes the conical countersunk seating face.
        es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][2]+hh)<1e-5 and abs(g[3]-hd/2)<1e-5)
        select_edges(m,es);f=chamfer_equal(m,.1,'Countersunk Head Chamfer')
        for a,b in [(hh,hd/2-sr),(hd/2-sr,hh)]:
            cd=wrap(f.GetDefinition(),sld.IChamferFeatureData2);cd.AccessSelections(m,None);cd.EqualDistance=False
            cd.SetEdgeChamferDistance(0,a*MM);cd.SetEdgeChamferDistance(1,b*MM)
            if f.ModifyDefinition(cd,m,None):break
        else:raise RuntimeError('Flat head cone failed')
        for dim in feature_own_dims(f):E.link(dim,'head_height' if abs(wrap(m.Parameter(dim),sld.IDimension).SystemValue*1000-hh)<1e-5 else 'head_diameter / 2 - model_shank_radius')
    plane='Front Plane'
    if kind=='PAN':ref_plane(m,E,plane,hh,'Head Crown Plane','head_height');plane='Head Crown Plane'
    hexagon(m,E,plane,'Hex Drive Sketch',af,'drive_across_flats')
    f=cut_extrude(m,hh*(.62 if kind=='FLAT' else .55),name='Hex Drive Cut');link_feature(E,f,['drive_depth'])
    m.MaterialPropertyValues=(.8,.05,.55,1.,.3,.5,0.,0.,0.)
    return finish(sw,m,E,key,path)

def nut(sw,nyloc=False):
    key='nut_M3_'+('nyloc' if nyloc else 'hex');m,E,path=start(sw,key,'Nut Hex Sketch');h=3 if nyloc else 2.4
    for n,v,t in [('nut_across_flats',5.5,'M3 nut hex across flats'),('nut_height',h,'Metal hex height'),('thread_bore_diameter',2.8,'Simplified thread bore'),('edge_break',.2,'Native edge break on hex perimeter'),('collar_diameter',4.95,'Nylon collar outside diameter'),('collar_height',1,'Nylon collar axial height')]:E.var(n,v,'mm',t)
    hexagon(m,E,'Front Plane','Nut Hex Sketch',5.5,'nut_across_flats')
    f=boss_extrude(m,h,name='Nut Hex Extrude');link_feature(E,f,['nut_height']);E.solve();m.ForceRebuild3(False)
    es=edges_where(bodies(m)[0],lambda g:g[0]=='line' and abs(g[1][2]-g[2][2])<1e-6)
    select_edges(m,es);f=chamfer_equal(m,.2,'Hex Edge Chamfers');link_feature(E,f,['edge_break'])
    if nyloc:
        ref_plane(m,E,'Front Plane',h,'Collar Seat Plane','nut_height')
        s=Sketch(m,'Collar Seat Plane','Nylon Collar Sketch');ci=s.circle(0,0,2.475);s.rel('sgCOINCIDENT',ci.GetCenterPoint2(),origin=True);s.ddim(ci,'collar_diameter',(5,5));s.finish(E)
        m.ClearSelection2(True);s.feat.Select2(False,0);f=boss_extrude(m,1,name='Nylon Collar');link_feature(E,f,['collar_height'])
    s=Sketch(m,'Front Plane','Thread Bore Sketch');ci=s.circle(0,0,1.4);s.rel('sgCOINCIDENT',ci.GetCenterPoint2(),origin=True);s.ddim(ci,'thread_bore_diameter',(5,5));s.finish(E)
    m.ClearSelection2(True);s.feat.Select2(False,0);cut_extrude(m,through=True,reverse=True,name='Thread Bore Cut')
    return finish(sw,m,E,key,path)

if __name__=='__main__':
    sw=connect()
    try:
        keys=sorted({p['part'] for p in json.loads((ROOT/'reference/assembly_manifest.json').read_text())['parts'] if p['part'].startswith('bolt_')})
        for key in keys:
            if not sys.argv[1:] or key in sys.argv[1:]:bolt(sw,key)
        if not sys.argv[1:] or 'nut_M3_hex' in sys.argv[1:]:nut(sw)
        if not sys.argv[1:] or 'nut_M3_nyloc' in sys.argv[1:]:nut(sw,True)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
