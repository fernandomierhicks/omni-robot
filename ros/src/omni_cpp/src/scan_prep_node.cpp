// scan_prep: /scan -> /scan_filtered for slam_toolbox and Nav2 (design.md 28), in C++ (its Python original,
// omni_perception/scan_prep.py, was deleted Nov 7). Drops the A2M12's floor returns (/lidar/tilt_cal, the BNO085's
// roll and pitch), then de-skews each scan with odom -> base_link at every sample's time and re-bins it by angle.
// Scans wait for the EKF's transform up to their last sample (0.3 s at most), never blocking the executor.
//
// Parameters: margin (m before the predicted floor hit), floor, deskew, deskew_keys (odom poses per turn),
// time_order (-1: the array runs against time), refine_stamps + latency (scan_time.ScanClock).
#include <chrono>
#include <cmath>
#include <deque>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "omni_cpp/params.hpp"
#include "omni_cpp/scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace omni {

using sensor_msgs::msg::LaserScan;

namespace {

double secs(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec * 1e-9; }

tf2::TimePoint tp(double t) { return tf2::TimePoint(std::chrono::nanoseconds((int64_t)(t * 1e9))); }

}  // namespace

class ScanPrep : public rclcpp::Node {
public:
    explicit ScanPrep(const rclcpp::NodeOptions& opt) : Node("scan_prep", opt) {
        namespace P = params;
        margin_ = P::number(*this, "margin", 0.3, 0.0, 2.0, "m: a return this far short of the predicted floor hit is kept");
        floor_ = P::flag(*this, "floor", true, "drop the lidar's floor returns (needs /lidar/tilt_cal)");
        deskew_ = P::flag(*this, "deskew", true, "de-skew each scan with odom at every sample's time");
        n_key_ = (int)P::integer(*this, "deskew_keys", 11, 2, 100, "odom poses looked up per scan for the de-skew");
        order_ = P::integer(*this, "time_order", 1, -1, 1, "1: the scan's array runs with time, -1: against it", true) >= 0 ? 1 : -1;
        double latency = P::number(*this, "latency", 0.0, 0.0, 0.5, "s: the lidar's stamp latency (refine_stamps)", true);
        if (P::flag(*this, "refine_stamps", false, "re-time scans from their spacing (scan_time ScanClock)", true))
            clock_ = std::make_unique<ScanClock>(latency);
        live_ = std::make_unique<P::Live>(*this);
        live_->bind("margin", &margin_);
        live_->bind("floor", &floor_);
        live_->bind("deskew", &deskew_);
        live_->bind("deskew_keys", &n_key_);
        buf_ = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(5.0));
        tfl_ = std::make_unique<tf2_ros::TransformListener>(*buf_, this);
        pub_ = create_publisher<LaserScan>("/scan_filtered", 10);
        s_cal_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/lidar/tilt_cal", rclcpp::QoS(1).transient_local(),
            [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr m) {
                cal_ = LidarCal{std::tan(m->vector.x), m->vector.y, m->vector.z};
                has_cal_ = true;
            });
        s_imu_ = create_subscription<sensor_msgs::msg::Imu>("/imu", 50, [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
            if (m->orientation_covariance[0] < 0) return;
            const auto& q = m->orientation;
            roll_ = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
            pitch_ = std::asin(std::max(-1.0, std::min(1.0, 2 * (q.w * q.y - q.z * q.x))));
        });
        s_scan_ = create_subscription<LaserScan>("/scan", 10, [this](LaserScan::SharedPtr m) { on_scan(m); });
        t_drain_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.005), [this] { drain(); });
        t_report_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(10.0), [this] { report(); });
    }

private:
    void on_scan(LaserScan::SharedPtr m) {
        if (clock_) {                          // the turn's start on the line under the driver's stamps
            int64_t ns = std::llround(clock_->update(secs(m->header.stamp), m->scan_time) * 1e9);
            m->header.stamp.sec = (int32_t)(ns / 1000000000), m->header.stamp.nanosec = (uint32_t)(ns % 1000000000);
        }
        if (!deskew_) return prepare(*m);
        queue_.push_back(m);
    }

    // de-skew needs odom -> base_link up to the scan's last sample; the EKF publishes it a little later
    void drain() {
        double now = this->now().seconds();
        while (!queue_.empty()) {
            const LaserScan& m = *queue_.front();
            double t_end = secs(m.header.stamp) + m.ranges.size() * m.time_increment;
            if (!buf_->canTransform("odom", "base_link", tp(t_end)) && now - t_end < 0.3) return;
            auto s = queue_.front();
            queue_.pop_front();
            prepare(*s);
        }
    }

    Pose2 odom_pose(double t) {
        auto tr = buf_->lookupTransform("odom", "base_link", tp(t));
        const auto& q = tr.transform.rotation;
        return {tr.transform.translation.x, tr.transform.translation.y,
                std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
    }

    std::vector<double> deskew_scan(const LaserScan& m, const std::vector<double>& a, const std::vector<double>& r,
                                    double t0) {
        if (m.time_increment <= 0) return r;
        try {
            if (!has_laser_off_) {
                auto tr = buf_->lookupTransform("base_link", m.header.frame_id, tf2::TimePointZero);
                laser_x_ = tr.transform.translation.x, laser_y_ = tr.transform.translation.y, has_laser_off_ = true;
            }
            auto ts = sample_times(r.size(), t0, m.time_increment, order_);
            return deskew(a, r, ts, [this](double t) { return odom_pose(t); }, laser_x_, laser_y_, t0, m.angle_min,
                          m.angle_increment, r.size(), n_key_);
        } catch (const std::exception& e) {
            if (!n_skew_fail_)
                RCLCPP_WARN(get_logger(), "de-skew: no odom -> base_link over the scan (%s)", e.what());
            ++n_skew_fail_;
            return r;
        }
    }

    void prepare(const LaserScan& m) {
        size_t n = m.ranges.size();
        std::vector<double> r(m.ranges.begin(), m.ranges.end()), a(n);
        for (size_t i = 0; i < n; ++i) a[i] = m.angle_min + m.angle_increment * (double)i;
        ++n_scans_;
        if (floor_ && has_cal_) {
            auto drop = floor_mask(a, r, cal_, roll_, pitch_, margin_);
            for (size_t i = 0; i < n; ++i) {
                n_in_ += std::isfinite(r[i]);
                if (drop[i]) ++n_drop_, r[i] = std::numeric_limits<double>::infinity();
            }
        }
        LaserScan out;
        out.header = m.header;
        out.angle_min = m.angle_min, out.angle_max = m.angle_max, out.angle_increment = m.angle_increment;
        out.time_increment = m.time_increment, out.scan_time = m.scan_time;
        out.range_min = m.range_min, out.range_max = m.range_max;
        if (deskew_) {
            r = deskew_scan(m, a, r, secs(m.header.stamp));
            out.time_increment = 0.0f;
        }
        out.ranges.resize(r.size());
        for (size_t i = 0; i < r.size(); ++i)
            out.ranges[i] = (r[i] < m.range_min || r[i] > m.range_max) ? std::numeric_limits<float>::infinity()
                                                                        : (float)r[i];
        pub_->publish(out);
    }

    void report() {
        std::string msg = std::to_string(n_scans_) + " scans";
        if (n_in_) {
            char b[64];
            std::snprintf(b, sizeof b, ", dropped %.1f%% of returns as floor", 100.0 * n_drop_ / n_in_);
            msg += b;
        }
        if (floor_ && !has_cal_) msg += " (no /lidar/tilt_cal yet: no floor filter)";
        if (deskew_) msg += ", de-skew failed on " + std::to_string(n_skew_fail_);
        RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    }

    double margin_;
    bool floor_, deskew_;
    int n_key_, order_;
    std::unique_ptr<ScanClock> clock_;
    std::unique_ptr<params::Live> live_;
    LidarCal cal_{};
    bool has_cal_ = false;
    double roll_ = 0, pitch_ = 0;
    int64_t n_in_ = 0, n_drop_ = 0, n_scans_ = 0, n_skew_fail_ = 0;
    bool has_laser_off_ = false;
    double laser_x_ = 0, laser_y_ = 0;
    std::deque<LaserScan::SharedPtr> queue_;
    std::unique_ptr<tf2_ros::Buffer> buf_;
    std::unique_ptr<tf2_ros::TransformListener> tfl_;
    rclcpp::Publisher<LaserScan>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr s_cal_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr s_imu_;
    rclcpp::Subscription<LaserScan>::SharedPtr s_scan_;
    rclcpp::TimerBase::SharedPtr t_drain_, t_report_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::ScanPrep)
