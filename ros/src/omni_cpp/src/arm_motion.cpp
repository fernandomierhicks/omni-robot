#include "omni_cpp/arm_motion.hpp"

#include <algorithm>

namespace omni {

double quintic(double s) {
    s = std::min(1.0, std::max(0.0, s));
    return s * s * s * (10 + s * (-15 + 6 * s));
}

Joints ArmPlay::sample(double t) const {
    size_t n = t_.size();
    if (t >= t_[n - 1]) return q_[n - 1];
    Joints out;
    if (kind_ == QUINTIC) {
        // k = searchsorted(t, tt, side="right") - 1, at least 0
        long k = (long)(std::upper_bound(t_.begin(), t_.end(), t) - t_.begin()) - 1;
        if (k < 0) k = 0;
        double s = quintic((t - t_[k]) / (t_[k + 1] - t_[k]));
        for (int i = 0; i < 6; ++i) out[i] = q_[k][i] + s * (q_[k + 1][i] - q_[k][i]);
        return out;
    }
    // LINEAR: the last waypoint at or before t, then straight to the next
    size_t k = 0;
    for (size_t i = 0; i < n; ++i)
        if (t_[i] <= t) k = i;
    size_t j = std::min(k + 1, n - 1);
    double f = std::max(0.0, (t - t_[k]) / std::max(1e-6, t_[j] - t_[k]));   // a clock a hair behind the start: not before it
    for (int i = 0; i < 6; ++i) out[i] = q_[k][i] + f * (q_[j][i] - q_[k][i]);
    return out;
}

}  // namespace omni
