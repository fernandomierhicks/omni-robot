#include "omni_cpp/mode_mux.hpp"

#include <algorithm>
#include <cctype>

namespace omni {

const char* const ROBOT_MODES[5] = {"IDLE", "TELEOP", "CATCH", "THROWER", "PICK"};

bool ModeMux::set_mode(std::string m, std::optional<bool> k, std::string& message) {
    std::transform(m.begin(), m.end(), m.begin(), [](unsigned char c) { return (char)std::toupper(c); });
    if (std::find_if(std::begin(ROBOT_MODES), std::end(ROBOT_MODES), [&](const char* x) { return m == x; }) ==
        std::end(ROBOT_MODES)) {
        message = "unknown mode '" + m + "' (one of IDLE, TELEOP, CATCH, THROWER, PICK)";
        return false;
    }
    mode = m;
    if (k) kid = *k;
    message = mode + ", " + (kid ? "kid" : "adult") + " profile";
    return true;
}

uint16_t ModeMux::fire(uint16_t seq) {
    if (!has_src_ || src_mode_ != mode) {      // a new source: adopt its counter, fire nothing
        has_src_ = true, src_mode_ = mode, src_seq_ = seq;
    } else if (seq != src_seq_) {
        src_seq_ = seq;
        fire_out = (uint16_t)(fire_out + 1);
    }
    return fire_out;
}

Request ModeMux::request(const CatchCommand& c) {
    if (mode != "CATCH") return request();
    Request r;
    r.req_mode = fw::REQ_AUTO, r.kid = kid, r.drive_mode = c.drive_mode;
    r.a = c.target[0], r.b = c.target[1], r.c = c.target[2], r.v_end = c.v_end;
    r.t_brake = c.t_brake, r.coast_s = c.coast_s, r.a_brake = c.a_brake, r.tilt = c.tilt, r.t_tilt = c.t_tilt;
    r.tilt_after = c.tilt_after, r.fly_rpm = c.fly_rpm;
    r.fire_seq = fire(c.fire_seq);
    return cap(r);
}

Request ModeMux::request(const Twist2* twist, const Request* thrower, const Twist2* pick) {
    return cap(request_(twist, thrower, pick));
}

Request ModeMux::cap(Request r) const {
    if (arm_out) {
        r.speed_cap = r.speed_cap > 0 ? std::min(r.speed_cap, ARM_OUT_SPEED) : ARM_OUT_SPEED;
        r.a_cap = r.a_cap > 0 ? std::min(r.a_cap, ARM_OUT_ACCEL) : ARM_OUT_ACCEL;
    }
    return r;
}

Request ModeMux::request_(const Twist2* twist, const Request* thrower, const Twist2* pick) {
    Request r;
    r.kid = kid;
    r.fire_seq = fire_out;
    if (mode == "IDLE") return r;              // REQ_DISARM
    r.req_mode = fw::REQ_AUTO;
    if (mode == "PICK") twist = pick;
    if (mode == "TELEOP" || mode == "PICK") {
        if (twist) {
            r.drive_mode = fw::DRIVE_VEL_BODY;
            r.hold_yaw = true;
            r.a = twist->vx, r.b = twist->vy, r.c = twist->wz;
        }
        return r;
    }
    if (mode == "THROWER" && thrower) {        // else the console went quiet: flywheels down, tilt home
        r.tilt = thrower->tilt;
        r.fly_rpm = thrower->fly_rpm;
        r.fire_seq = fire(thrower->fire_seq);
    }
    return r;                                  // CATCH without the behavior: a ramped stop
}

}  // namespace omni
