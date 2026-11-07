#include "omni_cpp/host_link.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace omni {

const char* const MODE_NAMES[5] = {"BOOT", "DISARMED", "AUTO", "MANUAL", "ESTOP"};
const char* const FEED_STATE_NAMES[4] = {"IDLE", "SEAT", "RUN", "DONE"};
static const char* const FAULT_NAMES[9] = {"LINK_LOST", "ESTOP", "BUMPER", "MOTOR_ERR", "MOTOR_SILENT", "LOW_BATT",
                                           "IMU_SILENT", "FLY_TELEM", "FAULT_STOP"};

std::string mode_name(uint8_t m) { return m < 5 ? MODE_NAMES[m] : std::to_string(m); }
std::string feed_state_name(uint8_t s) { return s < 4 ? FEED_STATE_NAMES[s] : std::to_string(s); }

std::vector<std::string> fault_names(uint16_t faults) {
    std::vector<std::string> out;
    for (int b = 0; b < 9; ++b)
        if (faults & (1u << b)) out.push_back(FAULT_NAMES[b]);
    return out;
}

fw::CmdBody to_command(const Request& r, const ClockSync& sync) {
    fw::CmdBody c{};
    c.req_mode = r.req_mode;
    c.profile = r.kid ? fw::PROFILE_KID : fw::PROFILE_ADULT;
    c.drive_mode = r.drive_mode;
    c.flags = r.hold_yaw ? fw::CMD_HOLD_YAW : 0;
    c.a = (float)r.a, c.b = (float)r.b, c.c = (float)r.c;
    c.v_end = (float)r.v_end, c.a_cap = (float)r.a_cap, c.speed_cap = (float)r.speed_cap;
    c.t_brake_us = r.t_brake ? (uint64_t)std::max<int64_t>(0, sync.to_mcu_us(r.t_brake)) : 0;
    c.coast_s = (float)r.coast_s, c.a_brake = (float)r.a_brake, c.tilt = (float)r.tilt;
    c.t_tilt_us = r.t_tilt ? (uint64_t)std::max<int64_t>(0, sync.to_mcu_us(r.t_tilt)) : 0;
    c.tilt_after = (float)r.tilt_after, c.fly_rpm = (float)r.fly_rpm;
    c.fire_seq = r.fire_seq;
    c.protocol = fw::PROTOCOL_VERSION;
    return c;
}

std::vector<uint8_t> HostLink::encode(fw::CmdBody cmd, int64_t t_host_ns) {
    seq_ = (uint8_t)(seq_ + 1);
    ++stats.cmds;
    cmd.t_host_ns = (uint64_t)t_host_ns;
    std::vector<uint8_t> out(sizeof(fw::CmdBody) + 8);
    out.resize(fw::frame_encode(fw::MSG_CMD, seq_, &cmd, sizeof cmd, out.data(), out.size()));
    return out;
}

std::vector<Message> HostLink::feed(const uint8_t* data, size_t n, int64_t t_host_ns) {
    std::vector<Message> out;
    uint8_t body[256];
    for (size_t i = 0; i < n; ++i) {
        uint8_t type;
        size_t len;
        uint32_t err0 = dec_.errors;
        bool done = dec_.push(data[i], type, body, sizeof body, len);
        stats.errors += dec_.errors - err0;
        if (!done) continue;
        ++stats.frames;
        Message m{};
        if (type == fw::MSG_STATE && len == sizeof(fw::StateBody)) {
            m.kind = Message::STATE;
            std::memcpy(&m.state, body, len);
            const fw::StateBody& s = m.state;
            sync.add((int64_t)s.echo_t_host_ns, (int64_t)s.echo_rx_us, (int64_t)s.t_mcu_us, t_host_ns);
            m.t_host = sync.to_host_s((int64_t)s.t_mcu_us);
            ++stats.states;
        } else if (type == fw::MSG_IMU && len == sizeof(fw::ImuBody)) {
            m.kind = Message::IMU;
            std::memcpy(&m.imu, body, len);
            m.t_host = sync.to_host_s((int64_t)m.imu.t_mcu_us);
            ++stats.imus;
        } else if (type == fw::MSG_INFO && len == sizeof(fw::InfoBody)) {
            m.kind = Message::INFO;
            std::memcpy(&m.info, body, len);
            m.t_host = sync.to_host_s((int64_t)m.info.t_mcu_us);
            info = m.info;
            has_info = true;
            ++stats.infos;
        } else if (type == fw::MSG_TIME && len == sizeof(fw::TimeBody)) {
            m.kind = Message::TIME;
            std::memcpy(&m.time, body, len);
            if (m.time.kind == fw::TE_SYNC) sync.add_mcu_pulse(m.time.seq, (int64_t)m.time.t_mcu_us);
            m.t_host = sync.to_host_s((int64_t)m.time.t_mcu_us);
            ++stats.times;
        } else {
            ++stats.errors;
            continue;
        }
        out.push_back(m);
    }
    return out;
}

}  // namespace omni
