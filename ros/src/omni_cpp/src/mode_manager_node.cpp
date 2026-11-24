// mode_manager: who drives (design.md 14, 15, 30). The one writer to the MCU link: /mcu/cmd at 100 Hz.
//
// TELEOP takes its twist from several sources, each on its own topic with a priority: the freshest-enough source with
// the highest priority drives, the others are ignored (no two writers ever mix on one topic). Defaults:
//   RC               (rc_priority 200) the RC transmitter's sticks, from /mcu/state (protocol 10) while its drive
//                    switch (ch 5) is on: it drives whatever the mode (IDLE included), a person at the sticks wins
//   /cmd_vel/joy     100  the joystick (teleop_twist_joy): a person at the stick always wins
//   /cmd_vel/manual   90  a person through software: the console's stick, Foxglove's Teleop panel
//   /cmd_vel          50  Nav2 (its collision monitor's output)
// each fresh for 0.5 s (parameters twist_topics, twist_priorities, twist_timeouts; the RC's rc_timeout). /cmd_vel/lock
// (Bool) true stops TELEOP whatever the sources say, while it stays fresh (lock_timeout, 1 s). /cmd_vel/active
// (latched String) names the source in charge ("none" when nobody is, "rc" for the RC).
//
// The drive guard (Nov 23; drive_guard.hpp): every TELEOP and PICK twist, whoever sends it, is capped by the lidar
// (guard_scan_topic, the de-skewed /scan_filtered) so the base can still stop guard_buffer short of what the lidar
// sees, braking at guard_decel after guard_latency, creeping the last centimetres at up to guard_touch_speed; moving
// away is never limited; with guard_slide (the default) only the part of the motion into the obstacle is cut, so a
// diagonal push at a wall glides along it. It needs no SLAM, map or Nav2: the scan's points are kept in odom (odom -> base_link at the
// scan's stamp) and moved into base_link every tick (the latest transform, advanced by the MCU's measured twist over
// its age). Off: the guard parameter false, or /drive_guard/enable (latched Bool) false: the console's switch. Held
// off for a moment (bypass): /drive_guard/bypass (Bool) true and fresh (bypass_timeout), the RC's bypass channel
// (rc_bypass_channel) high, the flight stick's button (joy_bypass_button, on /joy) held. No scan yet (the lidar not
// running): it passes everything and says so; scans stopped after it had them (the lidar lost): the translation is
// capped at guard_no_scan_speed. /drive_guard/state (String JSON, 10 Hz): status (off | bypass | no_lidar |
// lidar_lost | clear | slowing), the limit and free distance, the scan's age.
//
// /catch/cmd (CATCH), /thrower/cmd (THROWER), /pick/cmd_vel (PICK) as before; /wheels/cmd (WHEELS, the console's
// motor test: McuCommand a, b, c, v_end = each wheel's rad/s; older than wheels_timeout -> DRIVE_IDLE, the MCU ramps
// the wheels to rest; unguarded: wheels up).
// /arm/out: every request carries the arm-out caps while the arm is out. With arm:=true (the arm robot, the default)
// a missing or stale /arm/out (older than arm_out_timeout, 0.5 s; the arm driver sends it at 10 Hz) counts as out: a
// dead arm node caps the base instead of uncapping it. arm:=false (the catch robot) ignores it.
// The arm-out caps come from the arm robot's generated config (config: "" = omni_arm's robot_arm.json, a variant's name
// such as d435, or a json path): its arm_out speed_cap and a_cap; the guard's circle its base robot_radius.
// Service /set_mode. Publishes /mcu/cmd and /robot_mode (latched, e.g. "CATCH adult"). Parameters: params.hpp
// (ranges, read-only, live); values in omni_bringup/config/robot.yaml.
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "omni_cpp/json_value.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "omni_cpp/drive_guard.hpp"
#include "omni_cpp/mcu_msgs.hpp"
#include "omni_cpp/mode_mux.hpp"
#include "omni_cpp/params.hpp"
#include "omni_interfaces/msg/mcu_command.hpp"
#include "omni_interfaces/msg/mcu_state.hpp"
#include "omni_interfaces/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace omni {

using omni_interfaces::msg::McuCommand;

namespace {

std::string upper(std::string s) {
    for (auto& c : s) c = (char)std::toupper((unsigned char)c);
    return s;
}

double secs(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec * 1e-9; }

struct Pose2d {
    double x = 0, y = 0, yaw = 0;
};

Pose2d pose_of(const geometry_msgs::msg::TransformStamped& tr) {
    const auto& q = tr.transform.rotation;
    return {tr.transform.translation.x, tr.transform.translation.y,
            std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
}

// a stick axis past its deadband, rescaled to -1..1, with expo (0 linear .. 1 cubic)
double stick(double u, double dead, double expo) {
    if (!std::isfinite(u)) return 0.0;
    u = std::clamp(u, -1.0, 1.0);
    double a = std::abs(u);
    if (a <= dead) return 0.0;
    a = (a - dead) / (1.0 - dead);
    a = (1.0 - expo) * a + expo * a * a * a;
    return std::copysign(a, u);
}

std::string num(double v, int prec = 3) {
    if (!std::isfinite(v) || v >= 1e8) return "null";
    std::ostringstream s;
    s.setf(std::ios::fixed);
    s.precision(prec);
    s << v;
    return s.str();
}

}  // namespace

class ModeManager : public rclcpp::Node {
public:
    explicit ModeManager(const rclcpp::NodeOptions& opt)
        : Node("mode_manager", opt),
          mux_(upper(params::text(*this, "mode", "IDLE", "the mode at start (IDLE, TELEOP, CATCH, THROWER, PICK); /set_mode changes it")),
               params::flag(*this, "kid", false, "the kid profile at start; /set_mode changes it", true)) {
        namespace P = params;
        live_ = std::make_unique<P::Live>(*this);
        p_cmd_ = create_publisher<McuCommand>("/mcu/cmd", 10);
        p_mode_ = create_publisher<std_msgs::msg::String>("/robot_mode", rclcpp::QoS(1).transient_local());
        p_active_ = create_publisher<std_msgs::msg::String>("/cmd_vel/active", rclcpp::QoS(1).transient_local());
        p_guard_ = create_publisher<std_msgs::msg::String>("/drive_guard/state", 5);
        has_arm_ = P::flag(*this, "arm", true, "this robot has an arm: a missing or stale /arm/out caps the base", true);
        double robot_radius = 0.0;
        if (has_arm_)
            robot_radius = load_arm_config(
                P::text(*this, "config", "", "the arm's robot_arm*.json (\"\" | d435 | a path): its arm_out caps, the robot's radius"));
        arm_out_timeout_ = P::number(*this, "arm_out_timeout", 0.5, 0.05, 5.0, "s: /arm/out older than this counts as arm out");
        lock_timeout_ = P::number(*this, "lock_timeout", 1.0, 0.1, 10.0, "s: a true /cmd_vel/lock holds this long after its last message");
        catch_timeout_ = P::number(*this, "catch_timeout", 0.3, 0.05, 2.0, "s: CATCH: a /catch/cmd older than this -> a ramped stop");
        thrower_timeout_ = P::number(*this, "thrower_timeout", 0.3, 0.05, 2.0, "s: THROWER: a /thrower/cmd older than this -> flywheels down");
        pick_timeout_ = P::number(*this, "pick_timeout", 0.3, 0.05, 2.0, "s: PICK: a /pick/cmd_vel older than this -> a ramped stop");
        wheels_timeout_ = P::number(*this, "wheels_timeout", 0.3, 0.05, 2.0,
                                    "s: WHEELS: a /wheels/cmd older than this -> the wheels ramp to rest");
        double rate = P::number(*this, "rate", 100.0, 20.0, 500.0, "Hz: /mcu/cmd (also the MCU's heartbeat)", true);
        live_->bind("arm_out_timeout", &arm_out_timeout_);
        live_->bind("lock_timeout", &lock_timeout_);
        live_->bind("catch_timeout", &catch_timeout_);
        live_->bind("thrower_timeout", &thrower_timeout_);
        live_->bind("pick_timeout", &pick_timeout_);
        live_->bind("wheels_timeout", &wheels_timeout_);

        // the RC (protocol 10): its sticks from /mcu/state while the drive switch is on
        rc_priority_ = P::integer(*this, "rc_priority", 200, 0, 1000, "the RC's priority among the twist sources", true);
        rc_timeout_ = P::number(*this, "rc_timeout", 0.2, 0.02, 2.0, "s: the RC's sticks stay fresh this long (/mcu/state at 200 Hz)");
        rc_max_speed_ = P::number(*this, "rc_max_speed", 2.0, 0.1, 5.0, "m/s at full stick (the firmware's profile caps it anyway)");
        rc_max_turn_ = P::number(*this, "rc_max_turn", 2.0, 0.1, 8.0, "rad/s at full yaw stick");
        rc_deadband_ = P::number(*this, "rc_deadband", 0.04, 0.0, 0.3, "each stick's deadband (of full travel)");
        rc_expo_ = P::number(*this, "rc_expo", 0.0, 0.0, 1.0, "stick expo: 0 linear, 1 cubic");
        rc_bypass_ch_ = P::integer(*this, "rc_bypass_channel", 6, 0, 8,
                                   "the RC channel (1-8) that holds the guard off while high; 0 = none");
        joy_bypass_button_ = P::integer(*this, "joy_bypass_button", 2, -1, 31,
                                        "the flight stick's button (on /joy) that holds the guard off while pressed; -1 = none");
        bypass_timeout_ = P::number(*this, "bypass_timeout", 0.3, 0.05, 2.0, "s: a true /drive_guard/bypass holds the guard off this long");
        for (auto [k, v] : {std::pair{"rc_timeout", &rc_timeout_}, {"rc_max_speed", &rc_max_speed_}, {"rc_max_turn", &rc_max_turn_},
                            {"rc_deadband", &rc_deadband_}, {"rc_expo", &rc_expo_}, {"bypass_timeout", &bypass_timeout_}})
            live_->bind(k, v);
        live_->bind("rc_bypass_channel", &rc_bypass_ch_);
        live_->bind("joy_bypass_button", &joy_bypass_button_);

        // the drive guard
        guard_on_ = P::flag(*this, "guard", true, "the lidar drive guard (the console's switch is /drive_guard/enable)");
        double gr = P::number(*this, "guard_radius", 0.0, 0.0, 1.0, "m: the robot as a circle; 0 = the arm config's robot_radius");
        guard_.cfg.radius = gr > 0.0 ? gr : (robot_radius > 0.0 ? robot_radius : 0.24);
        guard_.cfg.buffer = P::number(*this, "guard_buffer", 0.10, 0.0, 0.5, "m: a full-speed stop ends this far short of the obstacle");
        guard_.cfg.decel = P::number(*this, "guard_decel", 2.5, 0.5, 6.0, "m/s^2: the braking the speed law assumes");
        guard_.cfg.latency = P::number(*this, "guard_latency", 0.10, 0.0, 0.5, "s: from a lower limit to the wheels braking");
        guard_.cfg.touch_speed = P::number(*this, "guard_touch_speed", 0.05, 0.0, 0.3,
                                           "m/s: the creep allowed at the buffer, falling to 0 at contact");
        guard_.cfg.self_margin = P::number(*this, "guard_self_margin", 0.01, 0.0, 0.1,
                                           "m: returns this far inside the robot's rim are the robot itself");
        guard_.cfg.slide = P::flag(*this, "guard_slide", true,
                                   "slide along an obstacle: cut only the part of the motion into it (false: slow it all)");
        guard_max_gap_ = P::number(*this, "guard_max_gap", 30.0, 0.0, 90.0,
                                   "deg: empty runs of the scan narrower than this are filled from their flanks (blind wedges)");
        guard_no_scan_speed_ = P::number(*this, "guard_no_scan_speed", 0.3, 0.0, 2.0,
                                         "m/s: the cap once the scans have stopped coming (the lidar lost)");
        guard_scan_timeout_ = P::number(*this, "guard_scan_timeout", 0.5, 0.1, 5.0, "s: a scan older than this is lost");
        std::string scan_topic = P::text(*this, "guard_scan_topic", "/scan_filtered", "the guard's scan (de-skewed)", true);
        live_->bind("guard", &guard_on_);
        live_->bind("guard_buffer", &guard_.cfg.buffer);
        live_->bind("guard_decel", &guard_.cfg.decel);
        live_->bind("guard_latency", &guard_.cfg.latency);
        live_->bind("guard_touch_speed", &guard_.cfg.touch_speed);
        live_->bind("guard_self_margin", &guard_.cfg.self_margin);
        live_->bind("guard_slide", &guard_.cfg.slide);
        live_->bind("guard_max_gap", &guard_max_gap_);
        live_->bind("guard_no_scan_speed", &guard_no_scan_speed_);
        live_->bind("guard_scan_timeout", &guard_scan_timeout_);
        tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listen_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

        auto topics = declare_parameter<std::vector<std::string>>(
            "twist_topics", std::vector<std::string>{"/cmd_vel/joy", "/cmd_vel/manual", "/cmd_vel"},
            P::desc("TELEOP's twist sources, one topic each", true));
        auto prios = declare_parameter<std::vector<int64_t>>("twist_priorities", std::vector<int64_t>{100, 90, 50},
                                                             P::desc("their priorities: the highest fresh one drives", true));
        auto touts = declare_parameter<std::vector<double>>("twist_timeouts", std::vector<double>{0.5, 0.5, 0.5},
                                                            P::desc("s: how long each source stays fresh", true));
        if (prios.size() != topics.size() || touts.size() != topics.size())
            throw std::invalid_argument("twist_topics, twist_priorities and twist_timeouts must have the same length");
        sources_.resize(topics.size());
        for (size_t i = 0; i < topics.size(); ++i) {
            Source& src = sources_[i];
            src.topic = topics[i], src.priority = prios[i], src.timeout = touts[i];
            src.sub = create_subscription<geometry_msgs::msg::Twist>(
                src.topic, 10, [this, i](geometry_msgs::msg::Twist::ConstSharedPtr m) {
                    sources_[i].twist = {m->linear.x, m->linear.y, m->angular.z}, sources_[i].t = now_s();
                });
        }
        s_lock_ = create_subscription<std_msgs::msg::Bool>("/cmd_vel/lock", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
            lock_ = m->data, t_lock_ = now_s();
        });
        s_catch_ = create_subscription<McuCommand>("/catch/cmd", 10, [this](McuCommand::ConstSharedPtr m) {
            catch_ = from_msg(*m), has_catch_ = true, t_catch_ = now_s();
        });
        s_thrower_ = create_subscription<McuCommand>("/thrower/cmd", 10, [this](McuCommand::ConstSharedPtr m) {
            thrower_ = from_msg(*m), has_thrower_ = true, t_thrower_ = now_s();
        });
        s_wheels_ = create_subscription<McuCommand>("/wheels/cmd", 10, [this](McuCommand::ConstSharedPtr m) {
            wheels_ = from_msg(*m), has_wheels_ = true, t_wheels_ = now_s();
        });
        s_pick_ = create_subscription<geometry_msgs::msg::Twist>(
            "/pick/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
                pick_ = {m->linear.x, m->linear.y, m->angular.z}, t_pick_ = now_s();
            });
        s_arm_ = create_subscription<std_msgs::msg::Bool>("/arm/out", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
            arm_out_msg_ = m->data, t_arm_ = now_s();
        });
        s_state_ = create_subscription<omni_interfaces::msg::McuState>(
            "/mcu/state", 20, [this](omni_interfaces::msg::McuState::ConstSharedPtr m) { on_state(*m); });
        s_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { on_scan(*m); });
        s_joy_ = create_subscription<sensor_msgs::msg::Joy>("/joy", 10, [this](sensor_msgs::msg::Joy::ConstSharedPtr m) {
            int b = (int)joy_bypass_button_;
            joy_bypass_ = b >= 0 && b < (int)m->buttons.size() && m->buttons[b] != 0, t_joy_ = now_s();
        });
        s_enable_ = create_subscription<std_msgs::msg::Bool>(
            "/drive_guard/enable", rclcpp::QoS(1).transient_local(),
            [this](std_msgs::msg::Bool::ConstSharedPtr m) { guard_enable_msg_ = m->data; });
        s_bypass_ = create_subscription<std_msgs::msg::Bool>("/drive_guard/bypass", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
            bypass_msg_ = m->data, t_bypass_ = now_s();
        });
        srv_ = create_service<omni_interfaces::srv::SetMode>(
            "/set_mode", [this](const omni_interfaces::srv::SetMode::Request::SharedPtr q,
                                omni_interfaces::srv::SetMode::Response::SharedPtr a) {
                a->success = mux_.set_mode(q->mode, q->kid, a->message);
                if (a->success) announce();
            });
        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this] { tick(); });
        announce();
        announce_source("none");
        RCLCPP_INFO(get_logger(), "drive guard %s: circle r %.3f m, stop %.2f m short at %.1f m/s^2 after %.2f s (%s)",
                    guard_on_ ? "on" : "off", guard_.cfg.radius, guard_.cfg.buffer, guard_.cfg.decel,
                    guard_.cfg.latency, scan_topic.c_str());
    }

private:
    struct Source {
        std::string topic;
        int64_t priority = 0;
        double timeout = 0.5;
        Twist2 twist{};
        double t = -1e9;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
    };

    void announce_source(const std::string& name) {
        if (name == active_) return;
        active_ = name;
        std_msgs::msg::String m;
        m.data = name;
        p_active_->publish(m);
        RCLCPP_INFO(get_logger(), "drive source: %s", name.c_str());
    }

    // the RC's sticks as a body twist while its drive switch is on (the firmware's MANUAL mapping: ch 2 forward,
    // ch 1 right, ch 4 yaw right)
    void on_state(const omni_interfaces::msg::McuState& m) {
        double t = now_s();
        meas_ = {m.vx, m.vy, m.wz}, t_meas_ = t;
        rc_ok_ = m.rc_ok, rc_direct_ = m.rc_direct;
        for (int k = 0; k < 8; ++k) rc_[k] = m.rc[k];
        bool sw = m.rc_ok && !m.rc_direct && m.rc[4] > 0.5f;
        if (sw) {
            double s = rc_max_speed_, d = rc_deadband_, e = rc_expo_;
            rc_twist_ = {stick(m.rc[1], d, e) * s, -stick(m.rc[0], d, e) * s, -stick(m.rc[3], d, e) * rc_max_turn_};
            t_rc_ = t;
        }
    }

    // a scan's points (and its blind wedges' fills) into odom, at the scan's own time
    void on_scan(const sensor_msgs::msg::LaserScan& m) {
        double t = secs(m.header.stamp);
        geometry_msgs::msg::TransformStamped tr;
        try {
            tr = tf_buf_->lookupTransform("odom", m.header.frame_id, tf2_ros::fromMsg(m.header.stamp));
        } catch (const tf2::TransformException&) {
            try {
                tr = tf_buf_->lookupTransform("odom", m.header.frame_id, tf2::TimePointZero);
            } catch (const tf2::TransformException& e) {
                ++n_scan_tf_fail_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "drive guard: no odom -> %s (%s)",
                                     m.header.frame_id.c_str(), e.what());
                return;
            }
        }
        Pose2d L = pose_of(tr);
        int filled = 0;
        double span = std::abs(m.angle_increment) * m.ranges.size();
        auto pts = guard_scan_points(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max,
                                     guard_max_gap_ * M_PI / 180.0, span > 2.0 * M_PI - 2.0 * std::abs(m.angle_increment),
                                     &filled);
        double c = std::cos(L.yaw), s = std::sin(L.yaw);
        for (auto& p : pts) p = {L.x + c * p.x - s * p.y, L.y + s * p.x + c * p.y};
        scan_odom_ = std::move(pts);
        n_filled_ = filled;
        t_scan_ = t, t_scan_rx_ = now_s(), have_scan_ = true;
    }

    // the scan's points in base_link now: the latest odom -> base_link, advanced by the measured twist over its age
    bool update_guard_points(double t) {
        geometry_msgs::msg::TransformStamped tr;
        try {
            tr = tf_buf_->lookupTransform("odom", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException&) {
            return false;
        }
        Pose2d B = pose_of(tr);
        double age = std::clamp(t - secs(tr.header.stamp), 0.0, 0.2);
        if (t - t_meas_ < 0.2) {                           // where the base is now, not where the transform was
            double c = std::cos(B.yaw), s = std::sin(B.yaw);
            B.x += (c * meas_.vx - s * meas_.vy) * age, B.y += (s * meas_.vx + c * meas_.vy) * age;
            B.yaw += meas_.wz * age;
        }
        tf_age_ = age;
        double c = std::cos(B.yaw), s = std::sin(B.yaw);
        std::vector<GuardPoint> pts;
        pts.reserve(scan_odom_.size());
        for (const auto& p : scan_odom_) {
            double dx = p.x - B.x, dy = p.y - B.y;
            pts.push_back({c * dx + s * dy, -s * dx + c * dy});
        }
        guard_.set_points(std::move(pts));
        return true;
    }

    // the guard on a twist: the cap, or a pass with the reason
    Twist2 guarded(const Twist2& tw, double t) {
        GuardResult g;
        g.vx = tw.vx, g.vy = tw.vy, g.wz = tw.wz, g.asked = std::hypot(tw.vx, tw.vy);
        std::string why;
        bool rc_byp = rc_ok_ && rc_bypass_ch_ >= 1 && rc_[rc_bypass_ch_ - 1] > 0.5f && t - t_meas_ < rc_timeout_;
        bool joy_byp = joy_bypass_ && t - t_joy_ < 0.5;
        bool msg_byp = bypass_msg_ && t - t_bypass_ < bypass_timeout_;
        if (!guard_on_ || !guard_enable_msg_) {
            status_ = "off";
        } else if (rc_byp || joy_byp || msg_byp) {
            status_ = "bypass";
            why = rc_byp ? "rc" : joy_byp ? "joy" : "console";
        } else if (!have_scan_) {
            status_ = "no_lidar";
        } else if (t - t_scan_rx_ > guard_scan_timeout_) {
            status_ = "lidar_lost";
            g.limit = guard_no_scan_speed_;
            if (g.asked > g.limit) {
                double k = g.limit / g.asked;
                g.vx *= k, g.vy *= k, g.limited = true;
            }
        } else if (!update_guard_points(t)) {
            status_ = "no_odom";
            g.limit = guard_no_scan_speed_;
            if (g.asked > g.limit) {
                double k = g.limit / g.asked;
                g.vx *= k, g.vy *= k, g.limited = true;
            }
        } else {
            g = guard_.limit(tw.vx, tw.vy, tw.wz);
            status_ = g.limited ? "slowing" : "clear";
        }
        last_ = g, bypass_why_ = why, guarded_now_ = true;
        return {g.vx, g.vy, g.wz};
    }

    // TELEOP's twist: the RC while its switch is on, else the highest-priority fresh source, none while locked
    const Twist2* teleop_twist(double t) {
        if (lock_ && t - t_lock_ < lock_timeout_) {
            announce_source("locked");
            return nullptr;
        }
        const Source* best = nullptr;
        for (const Source& src : sources_)
            if (t - src.t < src.timeout && (!best || src.priority > best->priority)) best = &src;
        announce_source(best ? best->topic : "none");
        return best ? &best->twist : nullptr;
    }

    bool rc_driving(double t) const {
        if (t - t_rc_ >= rc_timeout_) return false;
        for (const Source& src : sources_)        // a source above the RC's priority (none by default) still wins
            if (src.priority > rc_priority_ && t - src.t < src.timeout) return false;
        return true;
    }

    // robot_arm.json's arm_out caps into the mux and the robot's radius (the one source: simulation/v11
    // sil/robot_export.py writes it)
    double load_arm_config(const std::string& name) {
        std::string dir = ament_index_cpp::get_package_share_directory("omni_arm") + "/config/";
        bool path = name.find('/') != std::string::npos || name.find('\\') != std::string::npos ||
                    (name.size() > 5 && name.compare(name.size() - 5, 5, ".json") == 0);
        std::string file = name.empty() ? dir + "robot_arm.json" : path ? name : dir + "robot_arm_" + name + ".json";
        std::ifstream f(file);
        if (!f) throw std::runtime_error("mode_manager: no arm config " + file);
        std::stringstream ss;
        ss << f.rdbuf();
        const Json cfg = Json::parse(ss.str());
        const Json& ao = cfg["arm_out"];
        mux_.arm_out_speed = ao["speed_cap"].num();
        mux_.arm_out_accel = ao["a_cap"].num();
        RCLCPP_INFO(get_logger(), "arm out caps %.2f m/s, %.2f m/s^2 (%s)", mux_.arm_out_speed, mux_.arm_out_accel,
                    file.c_str());
        const Json& rr = cfg.get("base").get("robot_radius");
        return rr.is_number() ? rr.num() : 0.0;
    }

    void update_arm_out(double t) {
        bool out = has_arm_ && (t - t_arm_ >= arm_out_timeout_ || arm_out_msg_);
        if (out != mux_.arm_out) {
            const char* why = !out ? "arm in: base limits back"
                              : (t - t_arm_ >= arm_out_timeout_ ? "no /arm/out from the arm: base capped as if out"
                                                                : "arm out: base capped");
            RCLCPP_INFO(get_logger(), "%s", why);
        }
        mux_.arm_out = out;
    }

    double now_s() { return now().seconds(); }

    void announce() {
        std_msgs::msg::String m;
        m.data = mux_.mode + (mux_.kid ? " kid" : " adult");
        p_mode_->publish(m);
        RCLCPP_INFO(get_logger(), "mode %s (%s profile)", mux_.mode.c_str(), mux_.kid ? "kid" : "adult");
    }

    void publish_guard(double t) {
        if (!guarded_now_) {                               // nobody driving through the guard: its standing state
            bool on = guard_on_ && guard_enable_msg_;
            status_ = !on ? "off" : !have_scan_ ? "no_lidar" : t - t_scan_rx_ > guard_scan_timeout_ ? "lidar_lost" : "clear";
            last_ = GuardResult{};
            bypass_why_.clear();
        }
        std::ostringstream o;
        o << "{\"status\": \"" << status_ << "\", \"on\": " << (guard_on_ && guard_enable_msg_ ? "true" : "false")
          << ", \"bypass\": " << (bypass_why_.empty() ? "null" : "\"" + bypass_why_ + "\"")
          << ", \"asked\": " << num(last_.asked) << ", \"limit\": " << num(last_.limit)
          << ", \"free\": " << num(last_.free) << ", \"scan_age\": " << (have_scan_ ? num(t - t_scan_rx_) : "null")
          << ", \"points\": " << scan_odom_.size() << ", \"filled\": " << n_filled_
          << ", \"tf_age\": " << num(tf_age_) << ", \"rc\": " << (rc_driving(t) ? "true" : "false")
          << ", \"radius\": " << num(guard_.cfg.radius) << "}";
        std_msgs::msg::String m;
        m.data = o.str();
        p_guard_->publish(m);
    }

    void tick() {
        double t = now_s();
        update_arm_out(t);
        guarded_now_ = false;
        Request r;
        if (rc_driving(t)) {
            // the RC drives whatever the mode: TELEOP's request from its sticks, guarded
            announce_source("rc");
            Twist2 tw = guarded(rc_twist_, t);
            ModeMux m = mux_;
            m.mode = "TELEOP";
            r = m.request(&tw);
        } else if (mux_.mode == "CATCH") {
            if (has_catch_ && t - t_catch_ < catch_timeout_) {
                r = catch_;                        // a copy: the stored request keeps the behavior's own fire_seq
                r.kid = mux_.kid;
                r.fire_seq = mux_.fire(catch_.fire_seq);
            } else {
                r = mux_.request();                // the behavior is silent: a ramped stop
            }
        } else if (mux_.mode == "THROWER") {
            bool fresh = has_thrower_ && t - t_thrower_ < thrower_timeout_;
            r = mux_.request(nullptr, fresh ? &thrower_ : nullptr);
        } else if (mux_.mode == "PICK") {
            Twist2 tw{};
            bool fresh = t - t_pick_ < pick_timeout_;
            if (fresh) tw = guarded(pick_, t);
            r = mux_.request(nullptr, nullptr, fresh ? &tw : nullptr);
        } else if (mux_.mode == "WHEELS") {
            bool fresh = has_wheels_ && t - t_wheels_ < wheels_timeout_;
            r = mux_.request(nullptr, nullptr, nullptr, fresh ? &wheels_ : nullptr);
        } else {
            const Twist2* src = teleop_twist(t);           // (IDLE: named, but the mux disarms)
            Twist2 tw{};
            if (src) tw = mux_.mode == "TELEOP" ? guarded(*src, t) : *src;
            r = mux_.request(src ? &tw : nullptr);
        }
        McuCommand m = to_msg(r);
        m.header.stamp = now();
        p_cmd_->publish(m);
        if (++n_tick_ % 10 == 0) publish_guard(t);
    }

    ModeMux mux_;
    std::vector<Source> sources_;
    std::string active_;
    Twist2 pick_{};
    Request catch_, thrower_, wheels_;
    bool has_catch_ = false, has_thrower_ = false, has_wheels_ = false;
    double t_catch_ = -1e9, t_thrower_ = -1e9, t_pick_ = -1e9, t_wheels_ = -1e9;
    bool has_arm_ = true, arm_out_msg_ = false, lock_ = false;
    double arm_out_timeout_ = 0.5, lock_timeout_ = 1.0, t_arm_ = -1e9, t_lock_ = -1e9;
    double catch_timeout_ = 0.3, thrower_timeout_ = 0.3, pick_timeout_ = 0.3, wheels_timeout_ = 0.3;
    // the RC
    int64_t rc_priority_ = 200, rc_bypass_ch_ = 6, joy_bypass_button_ = 2;
    double rc_timeout_ = 0.2, rc_max_speed_ = 2.0, rc_max_turn_ = 2.0, rc_deadband_ = 0.04, rc_expo_ = 0.0;
    bool rc_ok_ = false, rc_direct_ = false;
    float rc_[8]{};
    Twist2 rc_twist_{}, meas_{};
    double t_rc_ = -1e9, t_meas_ = -1e9;
    // the guard
    DriveGuard guard_;
    bool guard_on_ = true, guard_enable_msg_ = true, bypass_msg_ = false, joy_bypass_ = false, have_scan_ = false;
    bool guarded_now_ = false;
    double guard_max_gap_ = 30.0, guard_no_scan_speed_ = 0.3, guard_scan_timeout_ = 0.5, bypass_timeout_ = 0.3;
    double t_scan_ = -1e9, t_scan_rx_ = -1e9, t_bypass_ = -1e9, t_joy_ = -1e9, tf_age_ = 0.0;
    std::vector<GuardPoint> scan_odom_;
    int n_filled_ = 0;
    uint64_t n_scan_tf_fail_ = 0, n_tick_ = 0;
    std::string status_ = "no_lidar", bypass_why_;
    GuardResult last_;
    std::shared_ptr<tf2_ros::Buffer> tf_buf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listen_;
    std::unique_ptr<params::Live> live_;
    rclcpp::Publisher<McuCommand>::SharedPtr p_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_mode_, p_active_, p_guard_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr s_pick_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_lock_, s_enable_, s_bypass_;
    rclcpp::Subscription<McuCommand>::SharedPtr s_catch_, s_thrower_, s_wheels_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_arm_;
    rclcpp::Subscription<omni_interfaces::msg::McuState>::SharedPtr s_state_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr s_scan_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr s_joy_;
    rclcpp::Service<omni_interfaces::srv::SetMode>::SharedPtr srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::ModeManager)
