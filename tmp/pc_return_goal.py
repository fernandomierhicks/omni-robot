import sys,json,time,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros/tools'))
import rclpy
from arm_load import ArmDriver
from lifecycle_msgs.srv import GetState,ChangeState
from lifecycle_msgs.msg import Transition
rclpy.init();n=ArmDriver();n.spin_for(3);result={}
def wait(pred,seconds):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        n.spin_for(.2)
        if pred():return True
    return False
def service(c,r):
    if not c.wait_for_service(timeout_sec=2):return None
    f=c.call_async(r);wait(f.done,4);return f.result() if f.done() else None
def pids():
    out=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if b'/slam_toolbox/localization_slam_toolbox_node\0' in (p/'cmdline').read_bytes():out.append(int(p.name))
        except OSError:pass
    return out
result['load']=n.cmd('slam_load',{'name':'house'})
assert wait(lambda:(n.status or {}).get('slam',{}).get('mode')=='localization' and (n.status.get('pose') or {}).get('frame')=='map',60)
x,y,yaw=n.to_map(n.truth[0],n.truth[1],math.degrees(n.truth[2]))
result['initial_pose']=n.cmd('initial_pose',{'x':x,'y':y,'yaw':yaw});n.spin_for(2)
c=n.create_client(ChangeState,'/slam_toolbox/change_state');g=n.create_client(GetState,'/slam_toolbox/get_state')
r=ChangeState.Request();r.transition.id=Transition.TRANSITION_DEACTIVATE
before=pids();response=service(c,r);print('deactivated',response,flush=True)
states=[]
for k in range(24):
    n.spin_for(1);s=service(g,GetState.Request());states.append(s.current_state.id if s else None)
    if k>2 and states[-1]==3:break
result['inactive_recovery']={'states':states,'before':before,'after':pids(),'ok':states[-1]==3 and pids()==before}
print(result['inactive_recovery'],flush=True)
result['goal']=n.goto(-3.,3.3,90,'after arm Explore and SLAM recovery')
n.cmd('stop');Path(__file__).with_suffix('.json').write_text(json.dumps(result,indent=1));print(result['goal'],flush=True)
n.destroy_node();rclpy.shutdown()
