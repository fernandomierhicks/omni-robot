// A motion played for the arm (design.md 11): omni_arm/motion.py's Motion (quintic segments between waypoints at given
// times) and the FollowJointTrajectory goal's straight-line interpolation. The brain plans; the driver only plays.
#pragma once

#include <array>
#include <vector>

#include "omni_cpp/arm_kin.hpp"

namespace omni {

double quintic(double s);                        // 0..1 with zero speed and acceleration at both ends

class ArmPlay {
public:
    enum Kind { QUINTIC, LINEAR };
    ArmPlay() = default;
    ArmPlay(Kind kind, std::vector<double> times, std::vector<Joints> waypoints)
        : kind_(kind), t_(std::move(times)), q_(std::move(waypoints)) {}
    bool empty() const { return q_.empty() || t_.size() != q_.size(); }
    double t_end() const { return t_.back(); }
    bool done(double t) const { return t >= t_.back(); }
    Joints sample(double t) const;
    Joints& first() { return q_.front(); }

private:
    Kind kind_ = QUINTIC;
    std::vector<double> t_;
    std::vector<Joints> q_;
};

}  // namespace omni
