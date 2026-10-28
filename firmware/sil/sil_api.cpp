// The firmware core as a shared library for the MuJoCo sim (software in the loop, design.md 30). The sim fills
// FwHwInputs from its physics every 1 ms, calls fw_tick, and applies FwHwOutputs to its motor and servo models; the
// Jetson side's bytes go through fw_rx / fw_tx. Built by build_sil.py (g++ in WSL, zig c++ on Windows).
#include <string.h>

#include "board_teensy41.h"
#include "dm_motor.h"
#include "firmware.h"
#include "protocol.h"
#include "sh2.h"

#if defined(_WIN32)
#define FW_API extern "C" __declspec(dllexport)
#else
#define FW_API extern "C" __attribute__((visibility("default")))
#endif

FW_API void* fw_create(void) {
    fw::Firmware* f = new fw::Firmware();
    f->init();
    return f;
}
FW_API void fw_destroy(void* h) { delete static_cast<fw::Firmware*>(h); }
FW_API void fw_tick(void* h, const FwHwInputs* in, FwHwOutputs* out) { static_cast<fw::Firmware*>(h)->tick(*in, *out); }
FW_API void fw_rx(void* h, const uint8_t* d, uint32_t n, uint64_t t_rx_us) {
    static_cast<fw::Firmware*>(h)->rx(d, n, t_rx_us);
}
FW_API uint32_t fw_tx(void* h, uint8_t* out, uint32_t cap) { return (uint32_t)static_cast<fw::Firmware*>(h)->tx(out, cap); }
FW_API uint32_t fw_mode(void* h) { return static_cast<fw::Firmware*>(h)->mode(); }
FW_API uint32_t fw_faults(void* h) { return static_cast<fw::Firmware*>(h)->faults(); }

FW_API void fw_odom(void* h, float* out) {
    const fw::Odometry& o = static_cast<fw::Firmware*>(h)->odom();
    float v[9] = {o.pose.x, o.pose.y, o.pose.yaw, o.vx, o.vy, o.wz, o.slip, o.roll, o.pitch};
    memcpy(out, v, sizeof(v));
}

// layout checks for the ctypes mirror
FW_API uint32_t fw_sizeof(uint32_t which) {
    switch (which) {
        case 0: return sizeof(FwHwInputs);
        case 1: return sizeof(FwHwOutputs);
        case 2: return sizeof(fw::CmdBody);
        case 3: return sizeof(fw::StateBody);
        case 4: return sizeof(fw::ImuBody);
        case 5: return sizeof(FwCanFrame);
        case 6: return sizeof(fw::InfoBody);
        case 7: return sizeof(FwImuSample);
        case 8: return sizeof(fw::TimeBody);
        default: return 0;
    }
}

// the BNO085 report parser (sh2.h): one channel-3 payload and its interrupt time -> IMU samples (gyro, accel, quat,
// time), each as 11 doubles: t_us, w x y z, gx gy gz, ax ay az. Returns how many
FW_API uint32_t fw_sh2_samples(const uint8_t* p, uint32_t n, uint64_t t_int_us, double* out, uint32_t cap) {
    fw::Sh2Report r[32];
    size_t nr = fw::sh2_parse(p, n, t_int_us, r, 32);
    static FwImuSample last{};
    FwImuSample s[16];
    size_t m = fw::sh2_samples(r, nr, last, s, cap < 16 ? cap : 16);
    for (size_t i = 0; i < m; ++i) {
        double* o = out + 11 * i;
        o[0] = (double)s[i].t_us;
        for (int a = 0; a < 4; ++a) o[1 + a] = s[i].quat[a];
        for (int a = 0; a < 3; ++a) o[5 + a] = s[i].gyro[a];
        for (int a = 0; a < 3; ++a) o[8 + a] = s[i].accel[a];
    }
    return (uint32_t)m;
}

// codec entry points for cross-checking the Python implementations (tests)
FW_API uint32_t fw_frame_encode(uint8_t type, uint8_t seq, const uint8_t* body, uint32_t len, uint8_t* out,
                                uint32_t cap) {
    return (uint32_t)fw::frame_encode(type, seq, body, len, out, cap);
}
FW_API uint32_t fw_crc16(const uint8_t* d, uint32_t n) { return fw::crc16(d, n); }
FW_API int fw_dm_decode(const FwCanFrame* f, float* out5) {
    fw::DmFeedback fb;
    fw::DmRange r{fw::prm::DM_PMAX, fw::prm::DM_VMAX, fw::prm::DM_TMAX};
    if (!fw::dm_decode_feedback(*f, r, fb)) return 0;
    out5[0] = fb.id;
    out5[1] = fb.err;
    out5[2] = fb.pos;
    out5[3] = fb.vel;
    out5[4] = fb.torque;
    return 1;
}

FW_API uint32_t fw_board_report(char* out, uint32_t cap) { return (uint32_t)board::board_report(out, cap); }

// a few constants the sim's models must agree with (fw_params.h): name -> value
FW_API double fw_param(const char* name) {
    using namespace fw::prm;
    struct {
        const char* n;
        double v;
    } t[] = {{"DM_PMAX", DM_PMAX}, {"DM_VMAX", DM_VMAX}, {"DM_TMAX", DM_TMAX}, {"DM_MASTER_BASE", DM_MASTER_BASE},
             {"FLY_POLE_PAIRS", FLY_POLE_PAIRS}, {"MAX_WHEEL_SPEED", MAX_WHEEL_SPEED}, {"CMD_TIMEOUT", CMD_TIMEOUT},
             {"TILT_MIN", TILT_MIN}, {"TILT_MAX", TILT_MAX}, {"FEED_SPEED", FEED_SPEED},
             {"SYNC_PERIOD_US", (double)SYNC_PERIOD_US}, {"CAM_TRIGGER_HZ", CAM_TRIGGER_HZ},
             {"DM_SAMPLE_TO_RX_US", (double)DM_SAMPLE_TO_RX_US}};
    for (auto& e : t)
        if (strcmp(e.n, name) == 0) return e.v;
    return -1e300;
}
