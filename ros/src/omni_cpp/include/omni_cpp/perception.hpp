// The catch path's perception (design.md 5, 6, 30): C++ port of omni_catch/perception.py. Every stereo frame pair:
// the ball in each image (detector), the cameras placed where the robot was at the exposure (the MCU's odometry and
// the BNO085's roll and pitch, interpolated), then the filter (tracker) at the exposure time.
#pragma once

#include <deque>
#include <limits>
#include <optional>
#include <vector>

#include "omni_cpp/detector.hpp"
#include "omni_cpp/robot_config.hpp"
#include "omni_cpp/tracker.hpp"

namespace omni {

struct CameraModel {
    V3 pos_b;                   // in base_link
    M3 R_b;                     // columns: the camera's axes in base_link
    double f, cx, cy;
    int width, height;
    static CameraModel from_config(const CameraConfig& c);
};

M3 rot_zyx(double yaw, double pitch, double roll);

struct PoseSample {
    double t, x, y, yaw, roll, pitch;
};

class Perception {
public:
    Perception(const RobotConfig& robot, const TrackerConfig& tcfg = {}, const DetectorConfig& dcfg = {});

    void set_play_area(double cx, double cy, double hx, double hy, double z_max = 4.0, double margin = 0.5);
    void add_pose(double t, double x, double y, double yaw, double roll = 0.0, double pitch = 0.0);
    PoseSample pose_at(double t) const;
    std::vector<Cam> cams_at(double t) const;
    // one exposure: frames[k] is camera k's frame, or null
    std::vector<Blob> process(double t_cap, const std::vector<const Frame*>& frames);
    bool ready() const { return tracker.tracking() && tracker.n >= tcfg.min_frames; }
    std::optional<TrackView> view() const;
    void reset() { tracker.reset(); }

    RobotConfig robot;
    BallFlight flight;
    std::vector<CameraModel> cams;
    TrackerConfig tcfg;
    ImageTracker tracker;
    Detector det;
    double R_ball;
    std::deque<PoseSample> poses;                 // the last 600 (3 s at 200 Hz), odom frame
    double t_last_det = -std::numeric_limits<double>::infinity();
    long frames = 0, detections = 0, stereo = 0;
};

}  // namespace omni
