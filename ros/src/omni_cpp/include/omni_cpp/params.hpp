// The robot's tunable values as ROS 2 parameters (design.md 2.1): every one declared with a description, a range for
// numbers, and read-only when it can't change while the node runs (a port, a rate, a config file). ROS itself refuses
// an out-of-range or read-only `ros2 param set`; a live one updates the node's variable at once (Live::bind).
// The values for the robot live in omni_bringup/config/robot.yaml (one section per node); the launch loads it.
//   ros2 param list /mode_manager · ros2 param describe /mode_manager pick_timeout · ros2 param set ... · ros2 param dump
#pragma once

#include <functional>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace omni::params {

inline rcl_interfaces::msg::ParameterDescriptor desc(const std::string& text, bool read_only) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = text;
    d.read_only = read_only;
    return d;
}

// a number in [lo, hi]
inline double number(rclcpp::Node& n, const std::string& name, double def, double lo, double hi, const std::string& text,
                     bool read_only = false) {
    auto d = desc(text, read_only);
    rcl_interfaces::msg::FloatingPointRange r;
    r.from_value = lo, r.to_value = hi, r.step = 0.0;
    d.floating_point_range.push_back(r);
    return n.declare_parameter<double>(name, def, d);
}

// an integer in [lo, hi]
inline int64_t integer(rclcpp::Node& n, const std::string& name, int64_t def, int64_t lo, int64_t hi,
                       const std::string& text, bool read_only = false) {
    auto d = desc(text, read_only);
    rcl_interfaces::msg::IntegerRange r;
    r.from_value = lo, r.to_value = hi, r.step = 1;
    d.integer_range.push_back(r);
    return n.declare_parameter<int64_t>(name, def, d);
}

inline bool flag(rclcpp::Node& n, const std::string& name, bool def, const std::string& text, bool read_only = false) {
    return n.declare_parameter<bool>(name, def, desc(text, read_only));
}

inline std::string text(rclcpp::Node& n, const std::string& name, const std::string& def, const std::string& text_,
                        bool read_only = true) {
    return n.declare_parameter<std::string>(name, def, desc(text_, read_only));
}

// Live parameters: bind(name, &variable) and a `ros2 param set` (already range-checked by ROS) writes the variable.
class Live {
public:
    explicit Live(rclcpp::Node& n) {
        handle_ = n.add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& ps) {
            for (const auto& p : ps) {
                auto it = setters_.find(p.get_name());
                if (it != setters_.end()) it->second(p);
            }
        });
    }
    void bind(const std::string& name, double* v) { setters_[name] = [v](const rclcpp::Parameter& p) { *v = p.as_double(); }; }
    void bind(const std::string& name, int64_t* v) { setters_[name] = [v](const rclcpp::Parameter& p) { *v = p.as_int(); }; }
    void bind(const std::string& name, int* v) { setters_[name] = [v](const rclcpp::Parameter& p) { *v = (int)p.as_int(); }; }
    void bind(const std::string& name, bool* v) { setters_[name] = [v](const rclcpp::Parameter& p) { *v = p.as_bool(); }; }
    void on(const std::string& name, std::function<void(const rclcpp::Parameter&)> fn) { setters_[name] = std::move(fn); }

private:
    std::map<std::string, std::function<void(const rclcpp::Parameter&)>> setters_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr handle_;
};

}  // namespace omni::params
