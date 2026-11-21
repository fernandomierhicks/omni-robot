// The arm's load model (design.md 4.5): from each servo's telemetry the torque it puts on its joint, from the links'
// masses gravity's, and what is left: an outside force (a contact, a weight, a door that doesn't give). A detector wants
// that over a threshold for a few cycles in a row before it calls the joint strained. omni_calib/arm_load.py is its
// Python twin: the arm load calibration fits this model there (omni_cpp/test checks the two agree). ROS-free.
//
// Per joint, with the duty d (Present Load), the supply V, the speed v, the goal's acceleration a, the case temperature T:
//   motor  tm = A d V / (1 + alpha (T - 25)) - E v - J a - o        (or, with the current: Ki (i - io) - J a - o)
//   moving the link gets tm - sgn(v) (F + mu |g|): outside = -g - tm + sgn(v) (F + mu |g|) + M(q) a (the links'
//          inertia from their masses as point masses, kinematics' inertia_torque)
//   still  anything within +-(Fs + mus |g|) of tm is friction holding: outside = what lies past that band
// v is the goal's velocity while a motion plays, else the measured speed when it is past LOAD_W_EPS (a push: the speed
// register's step is ~0.15 rad/s, so its flicker at rest is no motion), else 0; g the gravity torque of the links. Joint
// angles here are the true ones: encoder + zero + compliance x gravity (the sag past the encoder).
#pragma once

#include <array>
#include <string>
#include <vector>

#include "omni_cpp/arm_kin.hpp"

namespace omni {

constexpr double LOAD_V_EPS = 0.02;      // rad/s: a goal slower than this is still
constexpr double LOAD_W_EPS = 0.30;      // rad/s: a measured speed past this moves the joint (no goal moving)

struct JointLoad {
    double A = 0;            // N m per volt of duty x supply
    double E = 0;            // N m s/rad: back-EMF and viscous friction
    double J = 0;            // kg m2: the rotor through the gearbox
    double o = 0;            // N m
    double F = 0.1;          // N m: the gearbox's friction moving ...
    double mu = 0.5;         // ... and its share of the gravity torque
    double Fs = 0.1;         // N m: the band's half width still ...
    double mus = 0.5;        // ... and its share
    double Ki = 0, io = 0;   // the current model: N m/A, A
    double alpha = 0.0039;   // 1/C: the winding's resistance (the case temperature stands in for it)
    double zero = 0;         // rad: the joint's true zero against the encoder's
    double compliance = 0;   // rad/N m past the encoder
    double thr = 0.35;       // N m: the detector's threshold on the outside torque, still ...
    double thr_move = 0.7;   // ... and moving (the dynamics the model misses)
};

struct LoadModel {
    std::array<JointLoad, N_ARM> j{};
    std::vector<LinkMass> links;
    bool use_current = false;
    std::string source = "nominal";
    // the datasheet's servo (stall torque at 12 V, no-load speed), the config's masses, wide friction bands
    static LoadModel nominal(const ArmFk& fk);
    // a calibration json (omni_calib arm_load: {"joints": {name: {A, E, ...}}, "links": [...], "use_current"}); "" =
    // nominal; a field it lacks keeps the nominal's. Throws std::runtime_error
    static LoadModel load(const std::string& path, const ArmFk& fk);
};

struct ServoSample {         // one cycle of the arm joints' telemetry (the jaw's ignored)
    Joints q{}, w{}, duty{}, volts{}, temp{}, current{}, goal_vel{}, goal_acc{};
};

struct LoadEstimate {
    Joints grav{}, motor{}, ext{}, band{};   // N m: gravity on the joint, the motor's, outside, the band's half width
    Joints inertia{};                        // N m: the links' M(q) a
    std::array<bool, 6> moving{};
};

LoadEstimate estimate(const LoadModel& m, const ArmFk& fk, const ServoSample& s, const Vec3& g);

// the outside torques as one force at p (the gripper's frame), N in base_link: damped least squares through J^T
Vec3 tool_force(const ArmFk& fk, const Joints& q, const Joints& ext, const Vec3& p, double damping = 1e-4);

// true joint angles from the encoders' (zero + sag) and the encoder goals for true ones (the inverse, by iteration)
Joints true_angles(const LoadModel& m, const ArmFk& fk, const Joints& q_enc, const Vec3& g);
Joints encoder_goals(const LoadModel& m, const ArmFk& fk, const Joints& q_true, const Vec3& g);

struct StrainDetector {
    double tau = 0.04;       // s: the outside torque's low-pass
    int n = 4;               // cycles over the threshold in a row
    Joints filt{};
    std::array<int, 6> count{};
    std::array<bool, 6> strained{};
    void reset();
    // dt: s since the last update. True when a joint has just become strained
    bool update(const LoadModel& m, const LoadEstimate& e, double dt);
};

}  // namespace omni
