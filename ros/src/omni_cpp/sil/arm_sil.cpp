// The arm driver core (arm_driver_core.hpp) as a C library for the v11 sim (design.md 11): the tidy harness
// (simulation/v11 sil/sil_pick.py) plays the arm with the code the robot's arm_driver node runs, on the sim's servo bus.
// The port is two callbacks into the caller (write bytes, read up to n bytes). Built and loaded by arm_sil.py (ctypes);
// not part of the colcon build.
#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "omni_cpp/arm_driver_core.hpp"

using omni::ArmDriverCore;

namespace {

typedef void (*write_fn)(const uint8_t* data, uint32_t n);
typedef uint32_t (*read_fn)(uint8_t* buf, uint32_t cap);

struct CbPort : omni::sts::Port {
    write_fn w = nullptr;
    read_fn r = nullptr;
    void write(const uint8_t* d, size_t n) override { w(d, (uint32_t)n); }
    size_t read(uint8_t* buf, size_t cap, int) override { return r(buf, (uint32_t)cap); }
};

struct Handle {
    ArmDriverCore core;
    CbPort port;
    Handle(const char* config, const char* cal) : core(config, cal ? cal : "") {}
};

thread_local std::string g_error;

int fail(const std::exception& e) {
    g_error = e.what();
    return -1;
}

}  // namespace

#ifdef _WIN32
#define ARM_API __declspec(dllexport)              // as firmware/sil/sil_api.cpp: a Windows DLL exports only what it marks
#else
#define ARM_API __attribute__((visibility("default")))
#endif

extern "C" {

ARM_API const char* arm_error() { return g_error.c_str(); }

ARM_API void* arm_create(const char* config, const char* calibration) {
    try {
        return new Handle(config, calibration);
    } catch (const std::exception& e) {
        g_error = e.what();
        return nullptr;
    }
}

ARM_API void arm_destroy(void* h) { delete (Handle*)h; }

// pings, torque on, holds where the arm is; returns the number of missing servos (their ids in missing_out[6]), -1 on
// an error. timeout_s: per reply (0 for the sim's bus, which answers at once)
ARM_API int arm_connect(void* hp, write_fn w, read_fn r, double timeout_s, int* missing_out) {
    auto* h = (Handle*)hp;
    h->port.w = w, h->port.r = r;
    try {
        auto miss = h->core.connect(h->port, timeout_s);
        for (size_t i = 0; i < miss.size() && i < 6; ++i) missing_out[i] = miss[i];
        return (int)miss.size();
    } catch (const std::exception& e) {
        return fail(e);
    }
}

// an ArmMotion: cmd MOVE 0 / HOLD 1 / RELAX 2; n_times times, n_values waypoint values (n_times x 6 row-major for a
// valid MOVE). 1 = taken, 0 = refused (arm_error says why), -1 = the port failed
ARM_API int arm_command(void* hp, int cmd, int linear, const double* times, uint32_t n_times, const double* waypoints,
                        uint32_t n_values) {
    auto* h = (Handle*)hp;
    try {
        std::string why;
        bool ok = h->core.command(cmd, linear != 0, std::vector<double>(times, times + n_times),
                                  std::vector<double>(waypoints, waypoints + n_values), why);
        if (!ok) g_error = why;
        return ok ? 1 : 0;
    } catch (const std::exception& e) {
        return fail(e);
    }
}

ARM_API int arm_read(void* hp) {
    try {
        ((Handle*)hp)->core.read();
        return 0;
    } catch (const std::exception& e) {
        return fail(e);
    }
}

ARM_API int arm_drive(void* hp, double t) {
    try {
        ((Handle*)hp)->core.drive(t);
        return 0;
    } catch (const std::exception& e) {
        return fail(e);
    }
}

// the state: q, dq, load, goal (6 each), temp_c (6), volts; flags bit 0 connected, 1 torque, 2 have_goal, 3 moving,
// 4 ok, 5 tool_out; the missing / error ids (up to 6 each) and their counts
ARM_API void arm_state(void* hp, double* q, double* dq, double* load, double* goal, int* temp_c, double* volts, int* flags,
                       int* missing, int* n_missing, int* errors, int* n_errors) {
    const ArmDriverCore& c = ((Handle*)hp)->core;
    for (int i = 0; i < 6; ++i) q[i] = c.q[i], dq[i] = c.dq[i], load[i] = c.load[i], goal[i] = c.goal[i], temp_c[i] = c.temp_c[i];
    *volts = c.volts;
    *flags = (c.connected() ? 1 : 0) | (c.torque ? 2 : 0) | (c.have_goal ? 4 : 0) | (c.moving ? 8 : 0) | (c.ok() ? 16 : 0) |
             (c.tool_out() ? 32 : 0);
    *n_missing = (int)c.missing.size();
    *n_errors = (int)c.errors.size();
    for (int i = 0; i < *n_missing && i < 6; ++i) missing[i] = c.missing[i];
    for (int i = 0; i < *n_errors && i < 6; ++i) errors[i] = c.errors[i];
}

// the bus's counters: timeouts, packets sent, replies received, parser errors
ARM_API void arm_bus_stats(void* hp, int* out4) {
    const auto* b = ((Handle*)hp)->core.bus();
    out4[0] = b ? b->n_timeouts : 0, out4[1] = b ? b->n_tx : 0, out4[2] = b ? b->n_rx : 0;
    out4[3] = b ? b->parser.errors : 0;
}

}  // extern "C"
