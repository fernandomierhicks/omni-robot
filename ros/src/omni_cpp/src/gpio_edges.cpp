#include "omni_cpp/gpio_edges.hpp"

#include <fcntl.h>
#include <linux/gpio.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>

namespace omni {

namespace {

int64_t clock_ns(clockid_t c) {
    timespec ts;
    clock_gettime(c, &ts);
    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}

int64_t realtime_minus_monotonic_ns() {        // from the tightest of a few bracketed reads
    int64_t best_w = -1, best = 0;
    for (int i = 0; i < 5; ++i) {
        int64_t a = clock_ns(CLOCK_MONOTONIC), r = clock_ns(CLOCK_REALTIME), b = clock_ns(CLOCK_MONOTONIC);
        if (best_w < 0 || b - a < best_w) best_w = b - a, best = r - (a + b) / 2;
    }
    return best;
}

}  // namespace

EdgeReader::EdgeReader(std::string chip, unsigned line, std::string clock, OnEdge on_edge, Log log)
    : chip_(std::move(chip)), clock_(std::move(clock)), line_(line), on_edge_(std::move(on_edge)),
      log_(std::move(log)) {}

EdgeReader::~EdgeReader() { stop(); }

void EdgeReader::start() {
    running_ = true;
    th_ = std::thread(&EdgeReader::run, this);
}

void EdgeReader::stop() {
    running_ = false;
    if (th_.joinable()) th_.join();
}

int64_t EdgeReader::to_ros_ns(int64_t t_ns) {
    if (clock_ == "realtime") return t_ns;
    int64_t now = clock_ns(CLOCK_MONOTONIC);
    if (t_off_ < 0 || now - t_off_ > 1000000000) {   // the offset moves as NTP / chrony slews the wall clock
        off_ = realtime_minus_monotonic_ns();
        t_off_ = now;
    }
    return t_ns + off_;
}

void EdgeReader::run() {
    if (clock_ != "realtime" && clock_ != "monotonic" && clock_ != "hte") {
        log_("sync edges: clock must be realtime, monotonic or hte, not " + clock_);
        return;
    }
    int chip = ::open(chip_.c_str(), O_RDWR | O_CLOEXEC);
    if (chip < 0) {
        log_("sync edges on " + chip_ + ": " + std::strerror(errno));
        return;
    }
    gpio_v2_line_request req{};
    req.offsets[0] = line_;
    req.num_lines = 1;
    std::strncpy(req.consumer, "omni_sync", sizeof req.consumer - 1);
    req.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING;
    if (clock_ == "realtime") req.config.flags |= GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME;
#ifdef GPIO_V2_LINE_FLAG_EVENT_CLOCK_HTE
    if (clock_ == "hte") req.config.flags |= GPIO_V2_LINE_FLAG_EVENT_CLOCK_HTE;
#endif
    int rc = ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &req);
    ::close(chip);
    if (rc < 0) {
        log_("sync edges on " + chip_ + ":" + std::to_string(line_) + ": " + std::strerror(errno));
        return;
    }
    pollfd p{req.fd, POLLIN, 0};
    while (running_) {
        if (poll(&p, 1, 500) <= 0) continue;
        gpio_v2_line_event ev;
        if (read(req.fd, &ev, sizeof ev) != (ssize_t)sizeof ev) continue;
        ++n;
        on_edge_(to_ros_ns((int64_t)ev.timestamp_ns));
    }
    ::close(req.fd);
}

}  // namespace omni
