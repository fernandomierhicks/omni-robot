// The SO-101's forward kinematics in C++ (design.md 11): omni_arm/kinematics.py's fk, tool pose and tool_out, read from the
// same config/robot_arm*.json, for arm_driver's TF and /arm/out. IK, collision checks and the tidy stay in Python.
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
std::array<double, 4> quat_of(const Mat4& T);    // x y z w of the rotation (arm_node._quat)

struct ArmFk {
    std::vector<double> lo, hi;                  // the joints' ranges (6)
    std::vector<std::string> joints;
    std::vector<int> servo_ids;
    double servo_speed = 0;                      // rad/s no load
    double out_radius = 0, out_z = 0;

    explicit ArmFk(const std::string& path);     // throws std::runtime_error

    // {body: base_link pose} for the chain's bodies, plus "tool", "arm_jaw", "wrist_cam" (a MuJoCo camera)
    std::map<std::string, Mat4> fk(const Joints& q) const;
    Mat4 tool(const Joints& q) const;
    bool tool_out(const Joints& q) const;

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
