// The SO-101's driver without ROS (design.md 11, 4.5): the servo bus, the calibration, the motions it plays, the arm's
// state, and what the servos' telemetry says about the load on each joint. arm_driver_node.cpp wraps it on the robot
// (the bus board's serial port, /arm/motion in, /arm/joint_states, /arm/hw, /arm/servo, /arm/out, TF out); the v11
// sim's harnesses run the same code on the sim's servo bus through sil/arm_sil.cpp, so the sim plays the arm exactly as
// the robot will.
//
// connect: ping every servo, write the servo settings that differ (P / D, the protections; EEPROM, unlocked for the
// write), torque on, hold where it is.
// Each cycle: read every servo (one SYNC_READ: position .. current), estimate the load (arm_load.hpp: gravity, the
// servo's torque, the outside torque, the strain detector), sample the motion being played, write the six goals (one
// SYNC_WRITE). Joint angles in and out are the joints' true angles as the load model knows them (the encoder's + the
// calibrated zero + the sag past the encoder): with the nominal model, the encoders' own.
// Commands are omni_interfaces/ArmMotion's: MOVE (waypoints at times, quintic or straight segments, played from the
// current goal so a new motion has no jump), HOLD (torque on, hold where the arm is), RELAX (torque off: the arm drops);
// each with a guard flag: guarded (the default), a strained joint stops the motion and backs it off (the reflex) and
// holding still the driver trims out the servos' droop; unguarded, the servos raw (the arm load calibration).
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "omni_cpp/arm_kin.hpp"
#include "omni_cpp/arm_load.hpp"
#include "omni_cpp/arm_motion.hpp"
#include "omni_cpp/feetech.hpp"

namespace omni {

constexpr int ARM_TORQUE = 1000;      // torque limit, 0.1% of stall: the arm's joints at full torque
constexpr int GRIP_TORQUE = 500;      // the gripper at half (est; LeRobot's SO-101 follower uses ~50%)

// written at connect where a servo differs (est; LeRobot's SO-101 follower: P 16, D 32, the gripper's limits)
struct ServoSettings {
    bool write = true;                // false: only read them back and report
    int p_gain = 16, d_gain = 32, i_gain = 0;
    int max_temp = 65;                // C: it unloads past this
    int min_volt = 95, max_volt = 140;    // 0.1 V
    int protect_current = 500, grip_protect_current = 250;   // 6.5 mA
    int overload_torque = 80, grip_overload_torque = 25;     // % of the output for protect_time
    int protect_torque = 20, protect_time = 200;             // % after an overload; 10 ms
};

// the hold trim: holding still, for its first `settle` s the driver nudges what it sends until the encoders read the
// goal (the servo's own droop under load, ~1 deg on the loaded joints at P 16); then it stays as it is, frozen too while
// an outside torque is felt, so it never pushes harder. The deflection cue: settled, a joint pushed off its goal by more
// than `angle` for `n` cycles is strained (a still servo's duty hardly answers a push: its gearbox's friction takes it,
// but the encoder sees the joint give)
struct DeflectSettings {
    bool on = true;
    double angle = 0.012;             // rad (~0.7 deg)
    double settle = 1.0;              // s after a motion ends: the trim works, the cue waits
    int n = 5;                        // cycles
};

struct TrimSettings {
    bool on = true;
    double rate = 3.0;                // 1/s: the integral's rate on the error
    double max = 0.07;                // rad: the most it adds
    double deadband = 0.0008;         // rad: errors under this (half a tick) are left
};

struct ReflexSettings {
    bool on = true;                   // a strained joint (guarded motion) stops the arm and backs off
    double backoff = 0.05;            // rad: the strained joints give this much, with the force
    int soft_torque = 350;            // their torque limit until the next command (0.1 %)
    bool nominal_ok = false;          // fire on the nominal model too (its thresholds are the datasheet's guess: off
                                      // until the arm load calibration has run; the detector still reports)
};

class ArmDriverCore {
public:
    enum Command { MOVE = 0, HOLD = 1, RELAX = 2 };

    // config: a robot_arm*.json path; calibration: a LeRobot calibration json ("" = the sim's nominal one); load: an
    // arm load calibration json ("" = the nominal model)
    explicit ArmDriverCore(const std::string& config_path, const std::string& calibration_path = "",
                           const std::string& load_path = "");

    // the bus once its port is open: pings every servo; none missing -> the settings, torque on, read, hold where it
    // is (returns the missing ids; empty = connected). Throws std::runtime_error when the port is lost.
    std::vector<int> connect(sts::Port& port, double timeout_s, sts::Bus::Clock clock = nullptr);
    void disconnect();
    bool connected() const { return bus_ != nullptr; }

    // an ArmMotion command; false (and why) if refused. MOVE: n times, n x 6 waypoints (row-major), the first row
    // replaced by the current goal. guard: false = no reflex for this motion (the calibration's loaded poses).
    // Throws when the port is lost.
    bool command(int cmd, bool linear, const std::vector<double>& times, const std::vector<double>& waypoints,
                 std::string& why, bool guard = true);
    void read(double t = -1);             // one SYNC_READ, the load estimate and the detector (throws when lost)
    void drive(double t);                 // sample the motion at t and write the goal (throws when lost)
    void cycle(double t) { read(t), drive(t); }
    void set_load_model(const LoadModel& m) { model = m; }
    void set_gravity(const Vec3& g) { gravity = g; }   // base_link, m/s2 (the IMU's tilt)

    // the state
    Joints q{}, dq{}, load{}, goal{};     // rad (true), rad/s, the duty -1..1, rad (commanded, true)
    Joints q_enc{};                       // rad: the encoders' angles
    Joints goal_vel{}, goal_acc{};        // the played goal's (rad/s, rad/s2; 0 holding)
    Joints current{}, volts_j{};          // A, V each
    std::array<int, 6> temp_c{}, status{};
    double volts = 0;
    bool has_current = false;
    std::vector<int> missing, errors;     // ids silent / answering with an error in the last read
    bool torque = false, have_goal = false, moving = false;
    bool ok() const { return bus_ && missing.empty() && errors.empty(); }
    bool tool_out() const;                // the tool (measured or commanded) is out: the base's arm-out caps

    // the load (design.md 4.5)
    LoadModel model;
    Vec3 gravity{0, 0, -9.81};
    LoadEstimate est;
    StrainDetector detector;
    Vec3 tool_force_n{};                  // N, base_link: the outside torques as a force at the tool point
    bool guard = true;
    bool use_model_angles = true;         // joint angles through the load model's zero and sag
    ReflexSettings reflex;
    TrimSettings trim_cfg;
    Joints trim{};                        // rad: what the hold trim adds to each joint's encoder goal
    DeflectSettings deflect;
    std::array<bool, 6> deflected{};      // the deflection cue's (also in detector.strained)
    bool reflex_active = false;           // the reflex stopped the arm; cleared by the next command
    int reflexes = 0;
    std::string reflex_why;

    ServoSettings settings;
    std::vector<std::string> config_notes;   // what connect changed or couldn't check

    const ArmFk& fk() const { return *fk_; }
    const sts::Bus* bus() const { return bus_.get(); }
    std::array<sts::JointCal, 6> cal;
    int arm_torque = ARM_TORQUE, grip_torque = GRIP_TORQUE;   // torque limits (0..1000): written when they change

private:
    void enable(bool on);
    void configure();
    void send(const Joints& g);
    void torque_limits(bool restore_soft);
    void fire_reflex();
    std::unique_ptr<ArmFk> fk_;
    std::unique_ptr<sts::Bus> bus_;
    std::unique_ptr<ArmPlay> play_;
    int applied_arm_ = -1, applied_grip_ = -1;
    std::array<bool, 6> softened_{};
    Joints goal_prev_{};
    double t_still_ = -1;
    std::array<int, 6> def_count_{};
    std::array<double, 6> push_sign_{};
    double t_drive_ = -1, t_read_ = -1;
};

}  // namespace omni
