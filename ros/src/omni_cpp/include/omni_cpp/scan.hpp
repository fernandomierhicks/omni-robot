// LiDAR scan preparation (design.md 28): floor returns dropped, sample times, de-skew, turn-start stamps. The floor mask's
// Python original (scan_prep.py) was deleted Nov 7, 2026; test/test_cores.py checks the sample times against
// omni_perception/scan_time.py.
#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

namespace omni {

struct Pose2 { double x, y, yaw; };

// the A2M12's measured plane tilt (/lidar/tilt_cal: tan of the tilt, its azimuth, the plane's height)
struct LidarCal { double tan_tilt, azimuth, height; };

// true for returns at or beyond the range where the ray meets the floor, less margin (roll, pitch: the base's)
std::vector<bool> floor_mask(const std::vector<double>& a, const std::vector<double>& r, const LidarCal& cal,
                             double roll, double pitch, double margin);
// the same with each sample's own roll and pitch (the base's at the sample's time: a ramp swings the pitch ~20 deg in
// 0.3 s, about the time a turn takes to reach scan_prep; one pitch for the turn let the floor ahead through as a wall).
// laser_x, laser_y: the laser's offset from base_link. The base tilts about the floor point under base_link, so a laser
// ahead of it comes down nose-down (0.14 m ahead, 20 deg: 0.163 -> 0.105 m above the floor).
std::vector<bool> floor_mask(const std::vector<double>& a, const std::vector<double>& r, const LidarCal& cal,
                             const std::vector<double>& roll, const std::vector<double>& pitch, double margin,
                             double laser_x = 0.0, double laser_y = 0.0);

// the IMU's roll and pitch over the last `keep` seconds, read at any time: linear between the samples around it, the
// first / last one outside them, (0, 0) while empty. Samples out of order are dropped; one more than 1 s before the
// last (a clock jump) starts it over.
class TiltHistory {
public:
    explicit TiltHistory(double keep_s = 2.0) : keep_(keep_s) {}
    void add(double t, double roll, double pitch);
    std::pair<double, double> at(double t) const;
    bool empty() const { return h_.empty(); }
    double t_last() const { return h_.empty() ? -std::numeric_limits<double>::infinity() : h_.back()[0]; }

private:
    double keep_;
    std::deque<std::array<double, 3>> h_;       // (t, roll, pitch)
};

// each index's time; order -1: the array runs against time (the last index was measured first)
std::vector<double> sample_times(size_t n, double stamp, double time_increment, int order = 1);

// Ranges as seen from the laser at t_ref: each sample moved with the robot's motion (pose_at(t): base_link in odom,
// called n_key times across the scan and interpolated between), then re-binned by angle (nearest return kept).
std::vector<double> deskew(const std::vector<double>& a, const std::vector<double>& r, const std::vector<double>& ts,
                           const std::function<Pose2(double)>& pose_at, double laser_x, double laser_y, double t_ref,
                           double angle_min, double angle_increment, size_t n_out, int n_key = 11);

// turn-start times from the driver's stamps: the lower envelope of the stamps on a line, minus the fixed delay
class ScanClock {
public:
    explicit ScanClock(double latency_s = 0.0, size_t window = 100, double envelope_q = 0.03)
        : latency_(latency_s), window_(window), q_(envelope_q) {}
    double update(double stamp, double scan_time);
    double T = 0;                // the turn period (0: unknown)
    double jitter = std::numeric_limits<double>::quiet_NaN();

private:
    double latency_;
    size_t window_;
    double q_;
    std::deque<std::pair<long, double>> hist_;   // (k, stamp)
    long k_ = 0;
    bool has_last_ = false;
    double t_last_ = 0;
};

}  // namespace omni
