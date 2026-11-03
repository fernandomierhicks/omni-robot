// LiDAR scan preparation (design.md 28): floor returns dropped, sample times, de-skew, turn-start stamps. C++ port of
// omni_perception/scan_prep.py's floor mask and scan_time.py (their notes say why); test/test_cores.py checks the two
// against each other.
#pragma once

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
