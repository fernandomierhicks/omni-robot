// The host side of the host <-> MCU link (design.md 5, 30): bytes <-> messages. The frames and bodies are the
// firmware's own (firmware/lib/omni_fw/protocol.h, compiled in from there), so the two ends can't drift apart; the
// Python mirror is omni_mcu/protocol.py (test/test_cores.py checks this against it).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "omni_cpp/clock_sync.hpp"
#include "protocol.h"

namespace omni {

extern const char* const MODE_NAMES[5];       // BOOT DISARMED AUTO MANUAL ESTOP
extern const char* const FEED_STATE_NAMES[4]; // IDLE SEAT RUN DONE

std::string mode_name(uint8_t mode);
std::string feed_state_name(uint8_t s);
std::vector<std::string> fault_names(uint16_t faults);

// a decoded message, its MCU time mapped onto the host's clock (t_host, s; NaN before the clock map has a line)
struct Message {
    enum Kind { STATE, IMU, INFO, TIME } kind;
    double t_host;
    fw::StateBody state;
    fw::ImuBody imu;
    fw::InfoBody info;
    fw::TimeBody time;
};

struct LinkStats {
    int64_t frames = 0, errors = 0, states = 0, imus = 0, infos = 0, cmds = 0, times = 0;
};

// what the mode manager asks for: a CmdBody with its times on the host's clock (omni_interfaces/McuCommand)
struct Request {
    uint8_t req_mode = fw::REQ_DISARM;
    bool kid = false;
    uint8_t drive_mode = fw::DRIVE_IDLE;
    bool hold_yaw = false;
    double a = 0, b = 0, c = 0, v_end = 0, a_cap = 0, speed_cap = 0;
    double t_brake = 0;          // s, host clock; 0 = none
    double coast_s = 0, a_brake = 0, tilt = 0;
    double t_tilt = 0;           // s, host clock; 0 = none
    double tilt_after = 0, fly_rpm = 0;
    uint16_t fire_seq = 0;
};

fw::CmdBody to_command(const Request& r, const ClockSync& sync);

class HostLink {
public:
    explicit HostLink(const ClockSyncConfig& c = {}) : sync(c) {}

    // the next CMD frame, sent at t_host_ns (the host's clock)
    std::vector<uint8_t> encode(fw::CmdBody cmd, int64_t t_host_ns);
    // bytes that arrived at t_host_ns: the whole messages they complete
    std::vector<Message> feed(const uint8_t* data, size_t n, int64_t t_host_ns);
    void add_sync_edge(int64_t t_host_ns) { sync.add_host_edge(t_host_ns); }

    ClockSync sync;
    LinkStats stats;
    bool has_info = false;
    fw::InfoBody info{};         // the latest INFO

private:
    fw::FrameDecoder dec_;
    uint8_t seq_ = 0;
};

}  // namespace omni
