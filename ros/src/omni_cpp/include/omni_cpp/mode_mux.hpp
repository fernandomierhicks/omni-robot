// Robot modes on the host (design.md 14, 15, 30): who drives, and the profile the MCU enforces. C++ port of
// omni_catch/modes.py (its notes say why); test/test_cores.py checks the two against each other.
//
// Firing: the MCU fires one ball whenever the fire_seq it receives changes. Each source (the catch behavior, the
// console) counts its own requests, so the mux sends one counter of its own that moves by one per new request of the
// source in charge; a source that takes over is adopted without firing (a stale counter must never fire a ball).
#pragma once

#include <optional>
#include <string>

#include "omni_cpp/host_link.hpp"

namespace omni {

extern const char* const ROBOT_MODES[5];       // IDLE TELEOP CATCH THROWER PICK
constexpr double ARM_OUT_SPEED = 1.0;          // m/s: robot_arm.json arm_out (modes.ARM_OUT_CAPS)
constexpr double ARM_OUT_ACCEL = 2.131578947;  // m/s^2

struct Twist2 { double vx, vy, wz; };

// the catch behavior's request (omni_catch/game.CatchCommand); the mux turns it into an MCU request in CATCH
struct CatchCommand {
    uint8_t drive_mode = fw::DRIVE_GOTO;
    double target[3] = {0, 0, 0};
    double v_end = 0, t_brake = 0, coast_s = 0, a_brake = 0, tilt = 0, t_tilt = 0, tilt_after = 0, fly_rpm = 0;
    uint16_t fire_seq = 0;
};

class ModeMux {
public:
    explicit ModeMux(std::string mode = "IDLE", bool kid = false) : mode(std::move(mode)), kid(kid) {}

    // false and why for an unknown mode
    bool set_mode(std::string m, std::optional<bool> k, std::string& message);
    // the fire_seq to send for the source in charge's own seq
    uint16_t fire(uint16_t seq);
    // this cycle's request: twist (TELEOP), thrower (THROWER: tilt, fly_rpm, fire_seq), pick (PICK); each only if
    // fresh. With the arm out every request carries the arm-out caps.
    Request request(const Twist2* twist = nullptr, const Request* thrower = nullptr, const Twist2* pick = nullptr);
    // the same with the catch behavior's command (modes.ModeMux.request(catch=...)): used in CATCH
    Request request(const CatchCommand& c);

    std::string mode;
    bool kid;
    bool arm_out = false;
    uint16_t fire_out = 0;       // the fire_seq the MCU sees

private:
    Request request_(const Twist2* twist, const Request* thrower, const Twist2* pick);
    Request cap(Request r) const;              // the arm-out caps
    bool has_src_ = false;
    std::string src_mode_;
    uint16_t src_seq_ = 0;
};

}  // namespace omni
