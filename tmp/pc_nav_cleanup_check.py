import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
rclpy.init();n=rclpy.create_node('pc_nav_cleanup_check')
c=n.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
print('available',c.wait_for_service(timeout_sec=2),flush=True)
f=c.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN))
rclpy.spin_until_future_complete(n,f,timeout_sec=12)
print('done',f.done(),'result',f.result() if f.done() else None,flush=True)
n.destroy_node();rclpy.shutdown()
