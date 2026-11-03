import json,sys,socket,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros/tools'))
import rclpy
from arm_load import ArmDriver
from geometry_msgs.msg import Twist
def world(**req):
    with socket.create_connection(('127.0.0.1',9873),timeout=3) as s:
        s.sendall((json.dumps(req)+'\n').encode()); return json.loads(s.makefile().readline())
rclpy.init();n=ArmDriver();n.spin_for(3)
print('manual arm',flush=True)
out={}
for name in ('high','ready','stow'):
    r=n.cmd('arm',{'do':'pose','name':name});n.spin_for(7)
    out[name]={'command':r,'arm':n.arm}
    print(name,r[:2],(n.arm or {}).get('source'),flush=True)
print(world(person=False),flush=True); print(world(person=True),flush=True)
n.cmd('follow',{'mode':'follow'})
rows=[]
for k in range(60):
    n.spin_for(.5);rows.append({'follow':(n.status or {}).get('follow'),'truth':n.truth,'pose':(n.status or {}).get('pose')})
    if k%10==0:print(rows[-1],flush=True)
n.cmd('stop');n.spin_for(2);world(person=False)
out['follow']=rows
Path(__file__).with_suffix('.json').write_text(json.dumps(out,indent=1))
n.destroy_node();rclpy.shutdown()
