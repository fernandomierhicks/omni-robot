// The SO-101's forward kinematics in C++ (design.md 11): omni_arm/kinematics.py's fk, tool pose and tool_out, read from the
// same config/robot_arm*.json, for arm_driver's TF and /arm/out; and its statics (design.md 4.5): the gravity torque of
// the links on each joint and a point's Jacobian, as kinematics.py's gravity() and jacobian(). IK, collision checks and
// the tidy stay in Python.
#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include "omni_cpp/json_value.hpp"

namespace omni {

using Mat4 = std::array<double, 16>;             // row-major
using Vec3 = std::array<double, 3>;
using Joints = std::array<double, 6>;

Mat4 mat_identity();
Mat4 mat_mul(const Mat4& a, const Mat4& b);
std::array<double, 4> quat_of(const Mat4& T);    // x y z w of the rotation

struct LinkMass {                                // one moving body: its mass and centre in its own frame
    std::string body;
    double mass = 0;
    Vec3 com{};
};

constexpr int N_ARM = 5;                         // the joints that place the gripper (the jaw's is the sixth)

struct ArmFk {
    std::vector<double> lo, hi;                  // the joints' ranges (6)
    std::vector<std::string> joints;
    std::vector<int> servo_ids;
    double servo_speed = 0;                      // rad/s no load
    double out_radius = 0, out_z = 0;
    double stall_torque = 0;                     // N m (the servos' datasheet)
    bool current_feedback = false;               // the servos report their current (servos.current_feedback)
    std::vector<LinkMass> links;                 // the config's inertials (the official model's); empty: none given

    explicit ArmFk(const std::string& path);     // throws std::runtime_error

    // {body: base_link pose} for the chain's bodies, plus "tool", "arm_jaw", "wrist_cam" (a MuJoCo camera)
    std::map<std::string, Mat4> fk(const Joints& q) const;
    Mat4 tool(const Joints& q) const;
    bool tool_out(const Joints& q) const;
    // each of the five joints' axis point and direction in base_link, and every body's pose (bodies: may be null)
    void joint_frames(const Joints& q, std::array<Vec3, N_ARM>& origin, std::array<Vec3, N_ARM>& axis,
                      std::map<std::string, Mat4>* bodies = nullptr) const;
    // the torque (N m about each joint's axis; the jaw's 0) that gravity g (base_link, m/s2) puts on the joints
    // through these links (each body's mass at its centre)
    Joints gravity(const Joints& q, const Vec3& g, const std::vector<LinkMass>& links) const;
    // the torque the links need for joint accelerations qdd: M(q) qdd, each body a point mass at its centre (no
    // Coriolis: the arm's speeds are low)
    Joints inertia_torque(const Joints& q, const Joints& qdd, const std::vector<LinkMass>& links) const;
    // the torque a force f (base_link, N) at point p (the gripper's frame, m) puts on each joint
    Joints point_torque(const Joints& q, const Vec3& f, const Vec3& p) const;
    // d(point)/dq of p (the gripper's frame) in base_link: 3 x 5, row-major
    std::array<double, 3 * N_ARM> jacobian(const Joints& q, const Vec3& p) const;
    Vec3 tool_point() const { return {tool_[3], tool_[7], tool_[11]}; }   // the tool point in the gripper's frame

private:
    struct Body {
        std::string name;
        Mat4 fixed;
        bool has_joint = false;
        Vec3 axis{}, pos{};
    };
    std::vector<Body> bodies_;
    Mat4 tool_, jaw_, cam_;
    Vec3 jaw_axis_{};
};

}  // namespace omni
