from common import *

def cap(sw):
    key='dummy_motor_rear_cap';m,E,path=start(sw,key,'Cover Section')
    vals=[('outside_radius',26.5,'Motor envelope radius'),('rear_face_distance',40.2,'Exterior rear face toward negative motor Z'),
          ('front_face_distance',36.7,'Case seating face'),('carrier_radius',10.5,'Rear bearing carrier outside radius'),
          ('carrier_end_distance',30.2,'Forward end of rear bearing carrier'),('bearing_seat_radius',7,'687 outside radius'),
          ('bearing_shoulder_distance',35.2,'Rear bearing outer-race shoulder'),('retainer_access_radius',5,'Tool and retaining-washer access radius'),
          ('rear_mount_pcd',47,'AK40 rear mounting pitch circle'),('rear_mount_boss_diameter',6,'Printed rear-mount boss diameter'),
          ('rear_mount_boss_length',5.5,'Boss length forward of the case seating face'),('rear_mount_pilot',2.1,'M2.5 thread-forming pilot'),
          ('cover_screw_pcd',46.6,'Case-cover screw pitch circle'),('cover_clearance_diameter',3.4,'M3 screw clearance'),
          ('countersink_mouth_diameter',5.6,'Countersink mouth clearance for a shallow M3 flat head'),
          ('countersink_depth',1.171,'Axial countersink depth down to the clearance bore')]
    for n,v,t in vals:E.var(n,v,'mm',t)
    E.var('rear_first_angle',90,'deg','First rear mounting hole from local +Y toward -X');E.var('rear_screw_count',4,'','Rear mounting hole count')
    E.var('cover_first_angle',45,'deg','First cover screw from local +Y toward -X');E.var('cover_screw_count',4,'','Case-cover screw count')
    vs=[(5,40.2,'retainer_access_radius','rear_face_distance'),(26.5,40.2,'outside_radius','rear_face_distance'),
        (26.5,36.7,'outside_radius','front_face_distance'),(10.5,36.7,'carrier_radius','front_face_distance'),
        (10.5,30.2,'carrier_radius','carrier_end_distance'),(7,30.2,'bearing_seat_radius','carrier_end_distance'),
        (7,35.2,'bearing_seat_radius','bearing_shoulder_distance'),(5,35.2,'retainer_access_radius','bearing_shoulder_distance')]
    s=orthogonal_profile(m,E,'Top Plane','Cover Section',vs,axis='vertical');full_revolve(m,E,s,'Cover Revolve')
    make_axis(m,'Motor Axis','Top Plane','Right Plane')
    ref_plane(m,E,'Front Plane',-36.7,'Case Seat Plane','front_face_distance')
    ref_plane(m,E,'Front Plane',-40.2,'Exterior Face Plane','rear_face_distance')
    polar_hole(m,E,'Case Seat Plane','Rear Mount Boss Sketch',23.5,90,6,'rear_mount_pcd / 2','rear_first_angle','rear_mount_boss_diameter')
    f=boss_extrude(m,5.5,name='Rear Mount Boss');link_feature(E,f,['rear_mount_boss_length'])
    patterned(m,E,'Motor Axis',['Rear Mount Boss'],4,'rear_screw_count','Rear Mount Boss Pattern')
    polar_hole(m,E,'Exterior Face Plane','Rear Mount Pilot Sketch',23.5,90,2.1,'rear_mount_pcd / 2','rear_first_angle','rear_mount_pilot')
    cut_extrude(m,through=True,reverse=True,name='Rear Mount Pilot Cut')
    patterned(m,E,'Motor Axis',['Rear Mount Pilot Cut'],4,'rear_screw_count','Rear Mount Pilot Pattern')
    polar_hole(m,E,'Exterior Face Plane','Cover Clearance Sketch',23.3,45,3.4,'cover_screw_pcd / 2','cover_first_angle','cover_clearance_diameter')
    cut_extrude(m,through=True,reverse=True,name='Cover Clearance Cut')
    patterned(m,E,'Motor Axis',['Cover Clearance Cut'],4,'cover_screw_count','Cover Clearance Pattern')
    E.solve();m.ForceRebuild3(False)
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[1][2]+40.2)<1e-5 and abs(g[3]-1.7)<1e-5)
    if len(es)!=4:raise RuntimeError('Expected four rear countersink edges, got '+str(len(es)))
    select_edges(m,es);f=chamfer_equal(m,1.1,'Cover Countersinks')
    cd=wrap(f.GetDefinition(),sld.IChamferFeatureData2)
    if not cd.AccessSelections(m,None):raise RuntimeError('Cannot define countersink distances')
    cd.Type=2;cd.EqualDistance=False;cd.SetEdgeChamferDistance(0,.001171);cd.SetEdgeChamferDistance(1,.0011)
    if not f.ModifyDefinition(cd,m,None):raise RuntimeError('Countersink definition failed')
    m.EditRebuild3()
    mouth=[g for e in (wrap(x,sld.IEdge) for x in bodies(m)[0].GetEdges()) if (g:=edge_geom(e))[0]=='circle' and abs(g[1][2]+40.2)<1e-5 and 2.7<g[3]<3]
    if len(mouth)!=4:raise RuntimeError('Countersink mouth geometry missing')
    # Determine which selected-face distance is radial before linking controls.
    if abs(mouth[0][3]-2.8)>1e-5:
        cd=wrap(f.GetDefinition(),sld.IChamferFeatureData2);cd.AccessSelections(m,None)
        cd.SetEdgeChamferDistance(0,.0011);cd.SetEdgeChamferDistance(1,.001171)
        if not f.ModifyDefinition(cd,m,None):raise RuntimeError('Countersink distance order')
        m.EditRebuild3()
    for dim in feature_own_dims(f):
        val=wrap(m.Parameter(dim),sld.IDimension).SystemValue*1000
        E.link(dim,'countersink_depth' if abs(val-1.171)<1e-5 else '(countersink_mouth_diameter - cover_clearance_diameter) / 2')
    return finish(sw,m,E,key,path)

if __name__=='__main__':
    sw=connect()
    try:cap(sw)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)
