import sys, json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros/tools'))
import rclpy
from arm_load import ArmDriver
rclpy.init(); n=ArmDriver(); n.spin_for(3)
print(json.dumps({'status':n.status,'arm':n.arm,'objects':n.objects,'truth':n.truth,'origin':n.origin},indent=1))
n.destroy_node();rclpy.shutdown()
