// heading_node: which way the robot faces while Nav2 drives (the core is heading.cpp), in C++.
// Sits between Nav2's velocity smoother and its collision monitor.
//
// Subscribes cmd_vel_shape (the smoother's output), /odometry/filtered (yaw and turn rate), /plan (the path left).
// Publishes cmd_vel_smoothed: when agnostic each input unchanged, else the shaped command at `rate` Hz while Nav2's
// command is fresh. Parameters heading (agnostic | forward | spinning), spin_rate (rad/s), turn_max (rad/s), lead (s),
// rate (Hz); the console sets heading, spin_rate and turn_max at runtime. Wall time (steady clock): it needs no stamps.
#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "omni_cpp/heading.hpp"
#include "omni_cpp/params.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace omni {

class HeadingNode : public rclcpp::Node {
public:
    explicit HeadingNode(const rclcpp::NodeOptions& opt) : Node("heading_node", opt) {
        HeadingConfig c;
        namespace P = params;
        c.mode = P::text(*this, "heading", c.mode, "which way the robot faces on Nav2's path: agnostic | forward | spinning",
                         false);
        c.spin_rate = P::number(*this, "spin_rate", c.spin_rate, -12.6, 12.6, "rad/s: spinning's turn rate (negative: clockwise)");
        c.turn_max = P::number(*this, "turn_max", c.turn_max, 0.1, 12.6, "rad/s: forward's top turn rate");
        c.lead = P::number(*this, "lead", c.lead, 0.0, 0.5, "s from here to the firmware acting on the command");
        double rate = P::number(*this, "rate", 50.0, 5.0, 500.0, "Hz: the shaper's cycle", true);
        c.period = 1.0 / rate;
        if (!heading_mode_ok(c.mode)) throw std::invalid_argument("heading: one of agnostic, forward, spinning");
        core_ = HeadingShaper(c);
        params_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& ps) {
            rcl_interfaces::msg::SetParametersResult res;
            res.successful = true;
            for (auto& p : ps) {
                if (p.get_name() == "heading") {
                    std::string v = p.as_string();
                    if (!heading_mode_ok(v)) {
                        res.successful = false;
                        res.reason = "heading: one of agnostic, forward, spinning";
                        return res;
                    }
                    if (v != core_.cfg.mode) RCLCPP_INFO(get_logger(), "heading %s", v.c_str());
                    core_.cfg.mode = v;
                } else if (p.get_name() == "spin_rate" || p.get_name() == "turn_max" || p.get_name() == "lead") {
                    double v = p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ? (double)p.as_int() : p.as_double();
                    (p.get_name() == "spin_rate" ? core_.cfg.spin_rate : p.get_name() == "turn_max" ? core_.cfg.turn_max
                                                                                                   : core_.cfg.lead) = v;
                }
            }
            return res;
        });
        pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_smoothed", 10);
        s_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_shape", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
                publish(core_.on_cmd(t(), m->linear.x, m->linear.y, m->angular.z));
            });
        s_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                const auto& q = m->pose.pose.orientation;
                double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
                core_.on_odom(t(), m->pose.pose.position.x, m->pose.pose.position.y, yaw, m->twist.twist.angular.z);
            });
        s_plan_ = create_subscription<nav_msgs::msg::Path>("/plan", 1, [this](nav_msgs::msg::Path::ConstSharedPtr m) {
            double len = 0;
            for (size_t i = 1; i < m->poses.size(); ++i)
                len += std::hypot(m->poses[i].pose.position.x - m->poses[i - 1].pose.position.x,
                                  m->poses[i].pose.position.y - m->poses[i - 1].pose.position.y);
            core_.on_plan(t(), len);
        });
        timer_ = create_wall_timer(std::chrono::duration<double>(c.period), [this] {
            double now = t();
            if (core_.active(now) && now - core_.t_out > 0.015) {      // not right after an input's own publish
                auto c = core_.step(now);
                if (c) publish(*c);
            }
        });
        RCLCPP_INFO(get_logger(), "heading %s", c.mode.c_str());
    }

private:
    static double t() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void publish(const Cmd3& c) {
        geometry_msgs::msg::Twist m;
        m.linear.x = c.vx, m.linear.y = c.vy, m.angular.z = c.wz;
        pub_->publish(m);
    }

    HeadingShaper core_;
    OnSetParametersCallbackHandle::SharedPtr params_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr s_cmd_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr s_plan_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::HeadingNode)
