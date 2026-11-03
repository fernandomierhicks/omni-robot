import sys,json,time,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros/tools'))
import rclpy
from arm_load import ArmDriver
from lifecycle_msgs.srv import GetState,ChangeState
from lifecycle_msgs.msg import Transition
rclpy.init();n=ArmDriver();result={}
def wait(pred,seconds=60):
    until=time.monotonic()+seconds
    while time.monotonic()<until:
        n.spin_for(.2)
        if pred():return True
    return False
def service(cli,req,seconds=5):
    if not cli.wait_for_service(timeout_sec=2):return None
    f=cli.call_async(req);wait(f.done,seconds)
    return f.result() if f.done() else None
ready=lambda:n.status and n.cli.service_is_ready() and (n.status.get('pose') or {}).get('frame')=='map' and n.status.get('caps',{}).get('nav')
assert wait(ready,120),'stack not ready'
n.cmd('mission',{'name':'MANUAL'})
def seed_map():
    n.spin_for(.5)
    x,y,yaw=n.to_map(n.truth[0],n.truth[1],math.degrees(n.truth[2]))
    n.cmd('initial_pose',{'x':x,'y':y,'yaw':yaw})
    return wait(lambda:n.loc_error() is not None and n.loc_error()<.3,20)
result['initial_pose_seeded']=seed_map()
get=n.create_client(GetState,'/slam_toolbox/get_state');change=n.create_client(ChangeState,'/slam_toolbox/change_state')
req=ChangeState.Request();req.transition.id=Transition.TRANSITION_DEACTIVATE
response=service(change,req);print('deactivate',response,flush=True)
states=[]
for k in range(20):
    n.spin_for(1)
    r=service(get,GetState.Request())
    states.append(r.current_state.id if r else None)
    if k>2 and states[-1]==3:break
result['inactive_recovery']={'states':states,'ok':states[-1]==3,'status':n.status}
print('recovery',states,flush=True)
result['switches']=[]
for k in range(3):
    print('switch',k+1,flush=True)
    command=n.cmd('explore',{'new_map':True})
    ok=wait(lambda:n.status and n.status.get('mission')=='EXPLORE' and n.status.get('explore',{}).get('info',{}).get('known_m2',0)>0,60)
    first=n.status;n.spin_for(10);last=n.status
    n.cmd('cancel');n.cmd('slam_load',{'name':'house'})
    restored=wait(lambda:n.status and n.status.get('slam',{}).get('mode')=='localization' and (n.status.get('pose') or {}).get('frame')=='map',60)
    restored=restored and seed_map()
    result['switches'].append({'mapping':ok,'command':command,'first':first,'last':last,'restored':restored})
    print('mapping/restored',ok,restored,flush=True)
result['goal']=n.goto(-3.,3.3,90,'after SLAM switching')
n.cmd('stop');Path(__file__).with_suffix('.json').write_text(json.dumps(result,indent=1))
print(result['goal'],flush=True)
n.destroy_node();rclpy.shutdown()
