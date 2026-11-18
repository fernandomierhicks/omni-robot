"""Bring a lifecycle node (slam_toolbox) to active, retrying lost requests; exits once it is active.

    ros2 run omni_bringup lifecycle_up slam_toolbox

slam_toolbox's own launch files send one configure request through launch_ros and never look again. That request
sometimes goes before the node's service has matched it (DDS discovery under load, as in the sim), so the node sits
unconfigured: no map frame, `save_map` waits forever (Nov 16-17: 2 of 3 tries in some runs). stack.py launches
slam_toolbox with autostart:=false and this alongside: it asks the node's actual state and sends the next transition,
each call with a timeout, until the node is active. A lost request or reply is then only a retry. A transition the
node refuses (a missing map file) is retried MAX_FAILS times, then this exits with an error. Wall-clock timeouts, so
it behaves the same on sim time.
"""

from __future__ import annotations

import sys
import time

# lifecycle_msgs State ids (fixed by the ROS 2 lifecycle design)
UNCONFIGURED, INACTIVE, ACTIVE, FINALIZED = 1, 2, 3, 4
CONFIGURE, ACTIVATE = 1, 3                          # lifecycle_msgs Transition ids
MAX_FAILS = 5                                       # refused transitions before giving up
CALL_S = 2.0                                        # s: a get_state reply
CHANGE_S = 5.0                                      # s: a transition's reply; a longer one (localization loads its
                                                    # pose graph) shows as a transitional state or no answer: waited


def next_transition(state: int | None) -> int | None:
    """The transition that moves `state` towards active: None to wait (a transition under way, or no answer)."""
    return {UNCONFIGURED: CONFIGURE, INACTIVE: ACTIVATE}.get(state)


def main(argv=None):
    import rclpy
    from lifecycle_msgs.srv import ChangeState, GetState

    args = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)[1:]
    target = (args[0] if args else "slam_toolbox").strip("/")
    rclpy.init()
    node = rclpy.create_node(f"lifecycle_up_{target}")
    log = node.get_logger()
    c_get = node.create_client(GetState, f"/{target}/get_state")
    c_set = node.create_client(ChangeState, f"/{target}/change_state")

    def call(client, req, timeout):
        """The reply, or None after `timeout` (the request dropped)."""
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        f = client.call_async(req)
        rclpy.spin_until_future_complete(node, f, timeout_sec=timeout)
        if not f.done():
            client.remove_pending_request(f)
            f.cancel()
            return None
        return f.result()

    fails, t0, t_log, code = 0, time.monotonic(), 0.0, 1
    try:
        while rclpy.ok():
            r = call(c_get, GetState.Request(), CALL_S)
            state = r.current_state.id if r is not None else None
            if state == ACTIVE:
                log.info(f"/{target} active ({time.monotonic() - t0:.1f} s)")
                code = 0
                break
            if state == FINALIZED:
                log.error(f"/{target} is finalized: it won't start")
                break
            tr = next_transition(state)
            if tr is None:
                if time.monotonic() - t_log > 10.0:
                    t_log = time.monotonic()
                    log.info(f"waiting for /{target}" + (f" (state {state})" if state else "'s lifecycle services"))
                time.sleep(0.5)
                continue
            req = ChangeState.Request()
            req.transition.id = tr
            r = call(c_set, req, CHANGE_S)
            if r is None:
                log.warning(f"/{target}: no reply to {'configure' if tr == CONFIGURE else 'activate'}: asking its state again")
            elif not r.success:
                fails += 1
                log.error(f"/{target} refused {'configure' if tr == CONFIGURE else 'activate'} ({fails}/{MAX_FAILS})")
                if fails >= MAX_FAILS:
                    break
                time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
