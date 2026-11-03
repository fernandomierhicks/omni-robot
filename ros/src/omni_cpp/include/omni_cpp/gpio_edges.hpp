// The host's stamps of the MCU's SYNC_OUT edges (design.md 10), through the kernel's GPIO character device (uAPI v2,
// no library). C++ port of omni_mcu/gpio_edges.py. The kernel stamps each edge in its interrupt handler or, with the
// hardware timestamp engine (HTE: the Orin's AON GPIO lines; VERIFY which header pins), in hardware:
//   realtime   CLOCK_REALTIME, the ROS clock on the robot: used as is
//   monotonic  CLOCK_MONOTONIC: moved onto CLOCK_REALTIME with the two clocks' offset, measured here
//   hte        the HTE provider's stamps, on the monotonic base (VERIFY on the robot): as monotonic
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace omni {

class EdgeReader {
public:
    using OnEdge = std::function<void(int64_t t_ros_ns)>;
    using Log = std::function<void(const std::string&)>;
    EdgeReader(std::string chip, unsigned line, std::string clock, OnEdge on_edge, Log log);
    ~EdgeReader();
    void start();
    void stop();
    std::atomic<int64_t> n{0};

private:
    void run();
    int64_t to_ros_ns(int64_t t_ns);
    std::string chip_, clock_;
    unsigned line_;
    OnEdge on_edge_;
    Log log_;
    std::atomic<bool> running_{false};
    std::thread th_;
    int64_t off_ = 0;
    int64_t t_off_ = -1;
};

}  // namespace omni
