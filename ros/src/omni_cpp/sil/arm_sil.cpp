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
    Handle(const char* config, const char* cal, const char* load) : core(config, cal ? cal : "", load ? load : "") {}
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

ARM_API void* arm_create(const char* config, const char* calibration, const char* load) {
    try {
        return new Handle(config, calibration, load);
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
                        uint32_t n_values, int guard) {
    auto* h = (Handle*)hp;
    try {
        std::string why;
        bool ok = h->core.command(cmd, linear != 0, std::vector<double>(times, times + n_times),
                                  std::vector<double>(waypoints, waypoints + n_values), why, guard != 0);
        if (!ok) g_error = why;
        return ok ? 1 : 0;
    } catch (const std::exception& e) {
        return fail(e);
    }
}

ARM_API int arm_read(void* hp, double t) {
    try {
        ((Handle*)hp)->core.read(t);
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

// the load (design.md 4.5), each 6: q_enc, current, gravity, motor torque, outside torque, band, the detector's filtered
// outside torque, strained (0/1), status bits, volts, the goal's velocity and acceleration; tool force (3); reflexes,
// reflex_active, has_current
ARM_API void arm_load_state(void* hp, double* q_enc, double* current, double* grav, double* motor, double* ext,
                            double* band, double* filt, int* strained, int* status, double* volts, double* gvel,
                            double* gacc, double* force3, int* counts3) {
    const ArmDriverCore& c = ((Handle*)hp)->core;
    for (int i = 0; i < 6; ++i) {
        volts[i] = c.volts_j[i], gvel[i] = c.goal_vel[i], gacc[i] = c.goal_acc[i];
        q_enc[i] = c.q_enc[i], current[i] = c.current[i], grav[i] = c.est.grav[i], motor[i] = c.est.motor[i];
        ext[i] = c.est.ext[i], band[i] = c.est.band[i], filt[i] = c.detector.filt[i];
        strained[i] = c.detector.strained[i] ? 1 : 0, status[i] = c.status[i];
    }
    for (int i = 0; i < 3; ++i) force3[i] = c.tool_force_n[i];
    counts3[0] = c.reflexes, counts3[1] = c.reflex_active ? 1 : 0, counts3[2] = c.has_current ? 1 : 0;
}

ARM_API void arm_set_gravity(void* hp, double gx, double gy, double gz) { ((Handle*)hp)->core.set_gravity({gx, gy, gz}); }

// a load model json ("" = nominal); -1 when it can't be read
ARM_API int arm_set_load_model(void* hp, const char* path) {
    auto* h = (Handle*)hp;
    try {
        h->core.set_load_model(omni::LoadModel::load(path ? path : "", h->core.fk()));
        return 0;
    } catch (const std::exception& e) {
        return fail(e);
    }
}

// settings: torque limits, the reflex (on, backoff rad, soft torque), the detector (tau s, n), the servo settings'
// write flag
ARM_API void arm_settings(void* hp, int arm_torque, int grip_torque, int reflex_on, double backoff, int soft_torque,
                          double tau, int n, int write_servo_settings, int use_model_angles, int reflex_nominal,
                          int hold_trim, double deflect_angle) {
    ArmDriverCore& c = ((Handle*)hp)->core;
    c.arm_torque = arm_torque, c.grip_torque = grip_torque;
    c.reflex.on = reflex_on != 0, c.reflex.backoff = backoff, c.reflex.soft_torque = soft_torque;
    c.detector.tau = tau, c.detector.n = n;
    c.settings.write = write_servo_settings != 0;
    c.use_model_angles = use_model_angles != 0;
    c.reflex.nominal_ok = reflex_nominal != 0;
    c.trim_cfg.on = hold_trim != 0;
    c.deflect.on = deflect_angle > 0;
    if (deflect_angle > 0) c.deflect.angle = deflect_angle;
    if (!c.trim_cfg.on) c.trim = {};
}

// what connect changed in the servos' settings, one note per line
ARM_API const char* arm_config_notes(void* hp) {
    static thread_local std::string out;
    out.clear();
    for (auto& n : ((Handle*)hp)->core.config_notes) out += n + '\n';
    return out.c_str();
}

// the load model on one sample, without a bus (omni_cpp/test's parity with omni_calib.arm_load): q w duty volts temp
// current goal_vel goal_acc (6 each), gravity (3) -> grav motor ext band (6 each); also the true angles for q_enc and
// the encoder goals for q, and the tool force for ext
ARM_API int arm_estimate(void* hp, const double* in8x6, const double* g3, double* out4x6, double* true6, double* enc6,
                         double* force3) {
    const ArmDriverCore& c = ((Handle*)hp)->core;
    omni::ServoSample s;
    for (int i = 0; i < 6; ++i) {
        s.q[i] = in8x6[i], s.w[i] = in8x6[6 + i], s.duty[i] = in8x6[12 + i], s.volts[i] = in8x6[18 + i];
        s.temp[i] = in8x6[24 + i], s.current[i] = in8x6[30 + i], s.goal_vel[i] = in8x6[36 + i];
        s.goal_acc[i] = in8x6[42 + i];
    }
    omni::Vec3 g{g3[0], g3[1], g3[2]};
    auto e = omni::estimate(c.model, c.fk(), s, g);
    auto qt = omni::true_angles(c.model, c.fk(), s.q, g);
    auto qe = omni::encoder_goals(c.model, c.fk(), s.q, g);
    auto f = omni::tool_force(c.fk(), s.q, e.ext, c.fk().tool_point());
    for (int i = 0; i < 6; ++i) {
        out4x6[i] = e.grav[i], out4x6[6 + i] = e.motor[i], out4x6[12 + i] = e.ext[i], out4x6[18 + i] = e.band[i];
        true6[i] = qt[i], enc6[i] = qe[i];
    }
    for (int i = 0; i < 3; ++i) force3[i] = f[i];
    return 0;
}

// the bus's counters: timeouts, packets sent, replies received, parser errors
ARM_API void arm_bus_stats(void* hp, int* out4) {
    const auto* b = ((Handle*)hp)->core.bus();
    out4[0] = b ? b->n_timeouts : 0, out4[1] = b ? b->n_tx : 0, out4[2] = b ? b->n_rx : 0;
    out4[3] = b ? b->parser.errors : 0;
}

}  // extern "C"
