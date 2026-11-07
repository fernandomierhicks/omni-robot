// The SO-101's driver without ROS (design.md 11): the servo bus, the calibration, the motions it plays, the arm's state.
// arm_driver_node.cpp wraps it on the robot (the bus board's serial port, /arm/motion in, /arm/joint_states, /arm/hw,
// /arm/out, TF out); the v11 sim's tidy harness runs the same code on the sim's servo bus through sil/arm_sil.cpp, so the
// sim plays the arm exactly as the robot will.
//
// Each cycle: read every servo (one SYNC_READ), sample the motion being played, write the six goals (one SYNC_WRITE).
// Commands are omni_interfaces/ArmMotion's: MOVE (waypoints at times, quintic or straight segments, played from the
// current goal so a new motion has no jump), HOLD (torque on, hold where the arm is), RELAX (torque off: the arm drops).
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "omni_cpp/arm_kin.hpp"
#include "omni_cpp/arm_motion.hpp"
#include "omni_cpp/feetech.hpp"

namespace omni {

constexpr int ARM_TORQUE = 1000;      // torque limit, 0.1% of stall: the arm's joints at full torque
constexpr int GRIP_TORQUE = 500;      // the gripper at half (est; LeRobot's SO-101 follower uses ~50%)

class ArmDriverCore {
public:
    enum Command { MOVE = 0, HOLD = 1, RELAX = 2 };

    // config: a robot_arm*.json path; calibration: a LeRobot calibration json ("" = the sim's nominal one)
    explicit ArmDriverCore(const std::string& config_path, const std::string& calibration_path = "");

    // the bus once its port is open: pings every servo; none missing -> torque on, read, hold where it is (returns
    // the missing ids; empty = connected). Throws std::runtime_error when the port is lost.
    std::vector<int> connect(sts::Port& port, double timeout_s, sts::Bus::Clock clock = nullptr);
    void disconnect();
    bool connected() const { return bus_ != nullptr; }

    // an ArmMotion command; false (and why) if refused. MOVE: n times, n x 6 waypoints (row-major), the first row
    // replaced by the current goal. Throws when the port is lost.
    bool command(int cmd, bool linear, const std::vector<double>& times, const std::vector<double>& waypoints,
                 std::string& why);
    void read();                          // one SYNC_READ into the state below (throws when lost)
    void drive(double t);                 // sample the motion at t and write the goal (throws when lost)
    void cycle(double t) { read(), drive(t); }

    // the state
    Joints q{}, dq{}, load{}, goal{};     // rad, rad/s, fraction of the torque limit, rad (commanded)
    std::array<int, 6> temp_c{};
    double volts = 0;
    std::vector<int> missing, errors;     // ids silent / answering with an error in the last read
    bool torque = false, have_goal = false, moving = false;
    bool ok() const { return bus_ && missing.empty() && errors.empty(); }
    bool tool_out() const;                // the tool (measured or commanded) is out: the base's arm-out caps

    const ArmFk& fk() const { return *fk_; }
    const sts::Bus* bus() const { return bus_.get(); }
    std::array<sts::JointCal, 6> cal;

private:
    void enable(bool on);
    void send(const Joints& g);
    std::unique_ptr<ArmFk> fk_;
    std::unique_ptr<sts::Bus> bus_;
    std::unique_ptr<ArmPlay> play_;
};

}  // namespace omni
