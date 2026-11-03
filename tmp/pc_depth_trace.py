import json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from omni_arm.floor_vision import decode_depth
class Trace(Node):
    def __init__(self):
        super().__init__('pc_depth_trace')
        self.k=0;self.K=None;self.arm={};self.objs={}
        self.folder=Path.home()/'omni_pc_general_depth';self.folder.mkdir(exist_ok=True)
        self.create_subscription(CameraInfo,'/stereo/camera_info',self.info,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String,'/arm/state',self.state,5)
        self.create_subscription(String,'/sim/objects',self.objects,5)
        self.create_subscription(Image,'/stereo/depth',self.depth,2)
    def info(self,m):self.K=(m.k[0],m.k[4],m.k[2],m.k[5])
    def state(self,m):self.arm=json.loads(m.data)
    def objects(self,m):self.objs=json.loads(m.data)
    def depth(self,m):
        if self.K is None:return
        self.k+=1;p=self.folder/f'{self.k:04d}'
        np.savez_compressed(p.with_suffix('.npz'),depth=decode_depth(m.encoding,bytes(m.data),m.height,m.width),K=self.K)
        p.with_suffix('.json').write_text(json.dumps({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'arm':self.arm,'objects':self.objs}))
rclpy.init();n=Trace();until=time.monotonic()+600
while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.5)
n.destroy_node();rclpy.shutdown()
