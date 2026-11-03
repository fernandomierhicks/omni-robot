// omni_interfaces/McuCommand <-> the mux's Request (omni_catch/mode_node.py's request_to_msg / msg_to_request), for
// mode_manager and catch_behavior.
#pragma once

#include "omni_cpp/host_link.hpp"
#include "omni_interfaces/msg/mcu_command.hpp"

namespace omni {

inline Request from_msg(const omni_interfaces::msg::McuCommand& m) {
    Request r;
    r.req_mode = m.req_mode, r.kid = m.kid, r.drive_mode = m.drive_mode, r.hold_yaw = m.hold_yaw;
    r.a = m.a, r.b = m.b, r.c = m.c, r.v_end = m.v_end, r.a_cap = m.a_cap, r.speed_cap = m.speed_cap;
    r.t_brake = m.t_brake, r.coast_s = m.coast_s, r.a_brake = m.a_brake, r.tilt = m.tilt, r.t_tilt = m.t_tilt;
    r.tilt_after = m.tilt_after, r.fly_rpm = m.fly_rpm, r.fire_seq = m.fire_seq;
    return r;
}

inline omni_interfaces::msg::McuCommand to_msg(const Request& r) {
    omni_interfaces::msg::McuCommand m;
    m.req_mode = r.req_mode, m.kid = r.kid, m.drive_mode = r.drive_mode, m.hold_yaw = r.hold_yaw;
    m.a = (float)r.a, m.b = (float)r.b, m.c = (float)r.c, m.v_end = (float)r.v_end, m.a_cap = (float)r.a_cap;
    m.speed_cap = (float)r.speed_cap, m.t_brake = r.t_brake, m.coast_s = (float)r.coast_s;
    m.a_brake = (float)r.a_brake, m.tilt = (float)r.tilt, m.t_tilt = r.t_tilt, m.tilt_after = (float)r.tilt_after;
    m.fly_rpm = (float)r.fly_rpm, m.fire_seq = r.fire_seq;
    return m;
}

}  // namespace omni
