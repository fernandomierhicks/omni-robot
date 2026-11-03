import sys,json,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros/tools'))
import rclpy
from arm_load import ArmDriver
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy
class Explorer(ArmDriver):
    def __init__(self):
        super().__init__();self.area=0.;self.maps=0
        self.create_subscription(OccupancyGrid,'/map',self.map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def map(self,m):
        self.area=sum(v>=0 for v in m.data)*m.info.resolution**2;self.maps+=1
rclpy.init();n=Explorer();until=time.monotonic()+120
while time.monotonic()<until and not(n.status and n.cli.service_is_ready() and (n.status.get('pose') or {}).get('frame')=='map' and n.status.get('caps',{}).get('nav')):n.spin_for(.5)
assert n.status,'not ready'
n.cmd('mission',{'name':'MANUAL'});n.spin_for(1)
result={'start':n.status,'truth_start':n.truth,'command':n.cmd('explore',{'new_map':True}),'rows':[]}
started=False;t0=time.monotonic();seen_mapping=False
while time.monotonic()-t0<300:
    n.spin_for(2)
    s=n.status or {};started|=s.get('mission')=='EXPLORE'
    mapping=s.get('slam',{}).get('mode')=='mapping'
    if mapping and not seen_mapping:
        n.area=0.;seen_mapping=True
    row={'wall_s':round(time.monotonic()-t0,1),'mission':s.get('mission'),'task':s.get('task'),'explore':s.get('explore'),'known_m2':round(n.area,1),'truth':n.truth}
    result['rows'].append(row)
    print(row,flush=True)
    if started and s.get('mission')!='EXPLORE':break
result.update(known_m2=round(n.area,1),wall_s=round(time.monotonic()-t0,1),ended=started and (n.status or {}).get('mission')!='EXPLORE',end=n.status)
n.cmd('stop');Path(__file__).with_suffix('.json').write_text(json.dumps(result,indent=1));n.destroy_node();rclpy.shutdown()
