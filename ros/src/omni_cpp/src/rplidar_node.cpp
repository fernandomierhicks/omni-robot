// rplidar_node: the RPLIDAR A2M8 on its USB serial adapter (design.md 2, 4.6), in place of sllidar_ros2: /scan stamped
// from the device's own sample clock (rplidar.hpp says how and why), /lidar/status (JSON, 1 Hz).
//
//   /scan          sensor_msgs/LaserScan in frame_id (REP-103: x at the lidar's 0 deg mark, counter-clockwise), bins
//                  at i x 2 pi / bins from 0; the head turns clockwise, so the array runs against time: bin bins - 1
//                  at the stamp, bin i at stamp + (bins - 1 - i) x time_increment (scan_prep: time_order -1).
//                  +inf: no return; NaN: no sample fell in the bin.
//   /lidar/status  model, firmware, health, mode, turn rate, samples a turn, bytes skipped, packets lost, clock lines,
//                  the clock's ppm and floor spread, the error if any.
//
// The port: opened exclusively (TIOCEXCL; the console's Test > Lidar names this node when it finds the port taken),
// DTR cleared (the A2M8's adapter runs the motor flat out while DTR is raised), 115200 baud. At start: STOP, the
// device's info, health and scan modes (GET_LIDAR_CONF), the motor ramped to pwm over spin_up s, then the scan. A
// failure (unplugged, no data for 1 s, a USB reset) stops the motor, closes the port and tries again every 2 s;
// the node itself stays up.
//
// Parameters: port (auto: a CP210x by id, else /dev/ttyUSB0), mode (Stability; "": the device's typical), pwm
// (live), bins, frame_id, range_min / range_max, latency (live: the packet delay beyond its transmission, omni_calib
// measures it), clock_window, spin_up.
#include <dirent.h>
#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "omni_cpp/json.hpp"
#include "omni_cpp/params.hpp"
#include "omni_cpp/rplidar.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace omni {

namespace R = rplidar;

namespace {

double steady_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string realpath_of(const std::string& p) {
    char buf[PATH_MAX];
    return ::realpath(p.c_str(), buf) ? std::string(buf) : p;
}

std::string find_port() {
    for (const char* pat : {"/dev/serial/by-id/*CP210*", "/dev/serial/by-id/*Silicon_Labs*", "/dev/ttyUSB*"}) {
        glob_t g{};
        if (::glob(pat, 0, nullptr, &g) == 0 && g.gl_pathc > 0) {
            std::string hit = g.gl_pathv[0];
            ::globfree(&g);
            return hit;
        }
        ::globfree(&g);
    }
    return "";
}

// other processes with the port open (Linux /proc): two readers garble a serial port
std::string holders(const std::string& port) {
    std::string real = realpath_of(port), out;
    DIR* proc = ::opendir("/proc");
    if (!proc) return out;
    std::string me = std::to_string(::getpid());
    while (dirent* e = ::readdir(proc)) {
        std::string pid = e->d_name;
        if (pid.empty() || pid.find_first_not_of("0123456789") != std::string::npos || pid == me) continue;
        std::string fd_dir = "/proc/" + pid + "/fd";
        DIR* fds = ::opendir(fd_dir.c_str());
        if (!fds) continue;
        while (dirent* f = ::readdir(fds)) {
            if (f->d_name[0] == '.') continue;
            if (realpath_of(fd_dir + "/" + f->d_name) == real) {
                char comm[64] = {0};
                if (FILE* c = std::fopen(("/proc/" + pid + "/comm").c_str(), "r")) {
                    if (std::fgets(comm, sizeof comm, c)) comm[std::strcspn(comm, "\n")] = 0;
                    std::fclose(c);
                }
                out += (out.empty() ? "" : ", ") + std::string(comm) + " (pid " + pid + ")";
                break;
            }
        }
        ::closedir(fds);
    }
    ::closedir(proc);
    return out;
}

struct Mode {
    int id;
    std::string name;
    double us;           // per sample
    uint8_t answer;
    bool typical;
};

}  // namespace

class RplidarNode : public rclcpp::Node {
public:
    explicit RplidarNode(const rclcpp::NodeOptions& opt) : Node("rplidar", opt) {
        namespace P = params;
        port_ = P::text(*this, "port", "auto", "the lidar's serial device; auto: a CP210x adapter by id, else /dev/ttyUSB0");
        mode_ = P::text(*this, "mode", "Stability", "the scan mode by name (Standard | Express | Boost | Stability); \"\": the device's typical");
        pwm_ = P::integer(*this, "pwm", 660, 0, 1023, "the motor's PWM: the turn rate, open loop (660: 12.5 Hz warm)");
        bins_ = (int)P::integer(*this, "bins", 320, 90, 4000,
                                   "angle bins a turn on /scan: about the turn's samples (empty bins lower slam_toolbox's match scores)", true);
        frame_ = P::text(*this, "frame_id", "laser", "/scan's frame (x at the lidar's 0 deg mark)");
        range_min_ = P::number(*this, "range_min", 0.15, 0.0, 1.0, "m: /scan's range_min (the A2M8's datasheet)", true);
        range_max_ = P::number(*this, "range_max", 12.0, 1.0, 40.0, "m: /scan's range_max", true);
        latency_ = P::number(*this, "latency", 0.0005, 0.0, 0.1,
                             "s: a packet's last byte reaches the host this long after its transmission at the fastest "
                             "(the device's processing, USB): taken off every stamp; omni_calib measures it");
        window_ = P::number(*this, "clock_window", 10.0, 2.0, 60.0, "s: the sample clock's envelope window", true);
        spin_up_ = P::number(*this, "spin_up", 1.0, 0.0, 5.0, "s: the motor's ramp before the scan starts", true);
        live_ = std::make_unique<P::Live>(*this);
        live_->on("pwm", [this](const rclcpp::Parameter& p) {
            pwm_ = p.as_int();
            pwm_dirty_ = true;
        });
        live_->on("latency", [this](const rclcpp::Parameter& p) {
            latency_ = p.as_double();
            latency_dirty_ = true;
        });
        p_scan_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        p_status_ = create_publisher<std_msgs::msg::String>("/lidar/status", rclcpp::QoS(1).transient_local());
        t_status_ = create_wall_timer(std::chrono::seconds(1), [this] { status(); });
        thread_ = std::thread([this] { run(); });
    }

    ~RplidarNode() override {
        stop_ = true;
        if (thread_.joinable()) thread_.join();
    }

private:
    // -- the port -------------------------------------------------------------------------------------------------
    bool open_port(const std::string& dev, std::string& err) {
        int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            err = dev + ": " + std::strerror(errno);
            return false;
        }
        if (::ioctl(fd, TIOCEXCL) < 0) {
            err = dev + ": can't take it exclusively (" + std::strerror(errno) + ")";
            ::close(fd);
            return false;
        }
        termios t{};
        if (::tcgetattr(fd, &t) != 0) {
            err = dev + ": not a serial port (" + std::strerror(errno) + ")";
            ::close(fd);
            return false;
        }
        ::cfmakeraw(&t);
        ::cfsetspeed(&t, B115200);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= ~(CRTSCTS | HUPCL);
        t.c_cc[VMIN] = 0, t.c_cc[VTIME] = 0;
        ::tcsetattr(fd, TCSANOW, &t);
        int dtr = TIOCM_DTR;
        ::ioctl(fd, TIOCMBIC, &dtr);           // DTR cleared: the PWM runs the motor
        fd_ = fd;
        return true;
    }

    void close_port() {
        if (fd_ >= 0) {
            ::ioctl(fd_, TIOCNXCL);             // a pty keeps the exclusive flag past the close otherwise
            ::close(fd_);
        }
        fd_ = -1;
    }

    void send(uint8_t cmd, const std::vector<uint8_t>& payload = {}) {
        auto p = R::request(cmd, payload);
        std::lock_guard<std::mutex> lk(write_mu_);
        size_t off = 0;
        auto t_end = steady_s() + 0.2;
        while (off < p.size() && steady_s() < t_end) {
            ssize_t w = ::write(fd_, p.data() + off, p.size() - off);
            if (w > 0) {
                off += (size_t)w;
            } else if (w < 0 && errno != EAGAIN && errno != EINTR) {
                throw std::runtime_error(std::string("write: ") + std::strerror(errno));
            } else {
                pollfd q{fd_, POLLOUT, 0};
                ::poll(&q, 1, 20);
            }
        }
        if (off < p.size()) throw std::runtime_error("write timed out");
    }

    void set_pwm(int pwm) { send(R::SET_PWM, {(uint8_t)(pwm & 0xFF), (uint8_t)(pwm >> 8)}); }

    // > 0 bytes, 0 none within ms, throws when the port is gone
    ssize_t read_some(uint8_t* buf, size_t cap, int ms) {
        pollfd q{fd_, POLLIN, 0};
        int r = ::poll(&q, 1, ms);
        if (r == 0 || (r < 0 && errno == EINTR)) return 0;
        if (r < 0 || (q.revents & (POLLERR | POLLNVAL | POLLHUP)))
            throw std::runtime_error("the port went (unplugged? a USB reset?)");
        ssize_t n = ::read(fd_, buf, cap);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) return 0;
            throw std::runtime_error(std::string("read: ") + std::strerror(errno));
        }
        if (n == 0) throw std::runtime_error("the port went (unplugged? a USB reset?)");
        return n;
    }

    std::vector<uint8_t> read_exact(size_t n, double timeout) {
        std::vector<uint8_t> out(n);
        size_t got = 0;
        double t_end = steady_s() + timeout;
        while (got < n && !stop_) {
            double left = t_end - steady_s();
            if (left <= 0) break;
            ssize_t r = read_some(out.data() + got, n - got, (int)std::ceil(std::min(left, 0.05) * 1000));
            got += (size_t)std::max<ssize_t>(r, 0);
        }
        out.resize(got);
        return out;
    }

    void flush() { ::tcflush(fd_, TCIFLUSH); }

    std::vector<uint8_t> answer(uint8_t cmd, const std::vector<uint8_t>& payload, double timeout, uint8_t* type = nullptr) {
        send(cmd, payload);
        auto d = read_exact(7, timeout);
        R::Descriptor desc;
        if (!R::descriptor(d.data(), d.size(), desc)) {
            char b[64];
            std::snprintf(b, sizeof b, "no answer to 0x%02X (%zu bytes)", cmd, d.size());
            throw std::runtime_error(b);
        }
        if (type) *type = desc.type;
        auto body = read_exact(desc.size, timeout);
        if (body.size() < desc.size) throw std::runtime_error("short answer");
        return body;
    }

    std::vector<uint8_t> conf(uint32_t key, int mode = -1) {
        std::vector<uint8_t> p{(uint8_t)key, (uint8_t)(key >> 8), (uint8_t)(key >> 16), (uint8_t)(key >> 24)};
        if (mode >= 0) p.push_back((uint8_t)mode), p.push_back((uint8_t)(mode >> 8));
        auto b = answer(R::GET_CONF, p, 0.5);
        if (b.size() < 4 || (b[0] | b[1] << 8 | b[2] << 16 | (uint32_t)b[3] << 24) != key)
            throw std::runtime_error("bad GET_LIDAR_CONF answer");
        return {b.begin() + 4, b.end()};
    }

    std::vector<Mode> modes() {
        std::vector<Mode> out;
        try {
            auto c = conf(R::CONF_COUNT), ty = conf(R::CONF_TYPICAL);
            int n = c.at(0) | c.at(1) << 8, typical = ty.at(0) | ty.at(1) << 8;
            for (int m = 0; m < n; ++m) {
                auto us = conf(R::CONF_US_PER_SAMPLE, m), ans = conf(R::CONF_ANS_TYPE, m), nm = conf(R::CONF_NAME, m);
                uint32_t q8 = us.at(0) | us.at(1) << 8 | us.at(2) << 16 | (uint32_t)us.at(3) << 24;
                std::string name(nm.begin(), std::find(nm.begin(), nm.end(), (uint8_t)0));
                out.push_back({m, name, q8 / 256.0, ans.at(0), m == typical});
            }
        } catch (const std::exception&) {          // firmware without GET_LIDAR_CONF: Standard alone
            send(R::STOP);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            flush();
            out = {{0, "Standard", 500.0, R::ANS_STANDARD, true}};
            has_conf_ = false;
            return out;
        }
        has_conf_ = true;
        return out;
    }

    // -- a session: open, start, stream until it fails ------------------------------------------------------------
    void run() {
        std::string said;
        while (!stop_) {
            std::string err;
            try {
                session();
            } catch (const std::exception& e) {
                err = e.what();
            }
            stop_device();
            close_port();
            set_state(false, err);
            if (stop_) break;
            if (err != said) RCLCPP_WARN(get_logger(), "lidar: %s (trying again every 2 s)", err.c_str());
            said = err;
            for (int i = 0; i < 40 && !stop_; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void stop_device() {
        if (fd_ < 0) return;
        try {
            send(R::STOP);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            set_pwm(0);
        } catch (const std::exception&) {
        }
    }

    void session() {
        std::string dev = port_ == "auto" ? find_port() : port_;
        if (dev.empty()) throw std::runtime_error("no lidar serial port (no CP210x adapter, no /dev/ttyUSB*): is it plugged in?");
        std::string who = holders(dev);
        if (!who.empty()) throw std::runtime_error(dev + " is in use by " + who + " (one reader at a time)");
        std::string err;
        if (!open_port(dev, err)) throw std::runtime_error(err);
        send(R::STOP);
        set_pwm(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        flush();
        auto info = answer(R::GET_INFO, {}, 0.5);
        if (info.size() != 20) throw std::runtime_error("bad GET_INFO answer");
        auto health = answer(R::GET_HEALTH, {}, 0.5);
        static const char* const HEALTH[] = {"good", "warning", "error"};
        std::string h = health.at(0) < 3 ? HEALTH[health[0]] : std::to_string(health[0]);
        int code = health.at(1) | health.at(2) << 8;
        if (health[0] == 2) {                     // the SDK resets a device in error
            send(R::RESET);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            flush();
            throw std::runtime_error("the device reported health error (code " + std::to_string(code) + "): reset it");
        }
        auto ms = modes();
        const Mode* m = nullptr;
        for (const auto& x : ms)
            if (mode_.empty() ? x.typical : strcasecmp(x.name.c_str(), mode_.c_str()) == 0) m = &x;
        if (!m) {
            std::string names;
            for (const auto& x : ms) names += (names.empty() ? "" : ", ") + x.name;
            throw std::runtime_error("no scan mode \"" + mode_ + "\" (this one has " + names + ")");
        }
        char fw[16];
        std::snprintf(fw, sizeof fw, "%d.%02d", info[2], info[1]);
        std::string serial;
        for (size_t i = 4; i < 20; ++i) {
            char b[3];
            std::snprintf(b, sizeof b, "%02X", info[i]);
            serial += b;
        }
        {
            std::lock_guard<std::mutex> lk(mu_);
            st_ = Status{};
            st_.port = dev, st_.model = R::model_name(info[0]), st_.firmware = fw, st_.hardware = info[3];
            st_.serial = serial, st_.health = h, st_.health_code = code, st_.mode = m->name, st_.rate = 1e6 / m->us;
        }
        RCLCPP_INFO(get_logger(), "lidar: %s firmware %s hardware %d on %s, health %s; scanning %s (%.0f samples/s) at PWM %ld",
                    R::model_name(info[0]).c_str(), fw, info[3], dev.c_str(), h.c_str(), m->name.c_str(), 1e6 / m->us,
                    (long)pwm_);
        // the motor, gently up to its PWM (rplidar.py's soft start), then the scan
        int pwm = (int)pwm_, lo = std::min(pwm, 200), steps = std::max(1, (int)std::lround(spin_up_ / 0.1));
        set_pwm(lo);
        for (int i = 1; i <= steps && !stop_; ++i) {
            std::this_thread::sleep_for(std::chrono::duration<double>(spin_up_ / steps));
            set_pwm((int)std::lround(lo + (pwm - lo) * (double)i / steps));
        }
        pwm_dirty_ = false;
        flush();
        if (m->answer == R::ANS_STANDARD && m->id == 0) send(R::SCAN);
        else send(R::EXPRESS_SCAN, {(uint8_t)(has_conf_ ? m->id : 0), 0, 0, 0, 0});
        auto d = read_exact(7, 2.0);
        R::Descriptor desc;
        if (!R::descriptor(d.data(), d.size(), desc)) throw std::runtime_error("the " + m->name + " scan didn't start");
        R::StreamConfig sc;
        sc.ans_type = desc.type, sc.period_s = m->us * 1e-6, sc.latency_s = latency_, sc.bins = bins_, sc.window_s = window_;
        R::LidarStream stream(sc);              // throws for an answer type it can't read
        latency_dirty_ = false;
        set_state(true, "");
        std::vector<uint8_t> buf(4096);
        double last = steady_s();
        while (!stop_) {
            if (pwm_dirty_.exchange(false)) set_pwm((int)pwm_);
            if (latency_dirty_.exchange(false)) stream.set_latency(latency_);
            ssize_t n = read_some(buf.data(), buf.size(), 50);
            double t = steady_s();                // the bytes' arrival: the sample clock's evidence
            if (n <= 0) {
                if (t - last > 1.0) throw std::runtime_error("no data from the lidar for 1 s (motor stopped? cable?)");
                continue;
            }
            last = t;
            for (const auto& turn : stream.feed(buf.data(), (size_t)n, t)) publish(turn);
            std::lock_guard<std::mutex> lk(mu_);
            st_.bytes += n, st_.bad = stream.dec.bad, st_.lost = stream.dec.lost, st_.breaks = stream.dec.breaks;
            st_.lines = stream.clock.lines, st_.ppm = stream.clock.ppm(), st_.spread = stream.clock.spread;
            st_.dropped = stream.turns.dropped;
        }
    }

    void publish(const R::Turn& turn) {
        // the steady clock onto ROS time (the system clock on the robot), at the moment of publishing
        double off = now().seconds() - steady_s();
        sensor_msgs::msg::LaserScan s;
        int64_t ns = std::llround((turn.stamp + off) * 1e9);
        s.header.stamp.sec = (int32_t)(ns / 1000000000), s.header.stamp.nanosec = (uint32_t)(ns % 1000000000);
        s.header.frame_id = frame_;
        double inc = 2 * M_PI / bins_;
        s.angle_min = 0.0f, s.angle_increment = (float)inc, s.angle_max = (float)(inc * (bins_ - 1));
        s.time_increment = (float)turn.time_increment, s.scan_time = (float)turn.scan_time;
        s.range_min = (float)range_min_, s.range_max = (float)range_max_;
        s.ranges = turn.ranges;
        p_scan_->publish(s);
        std::lock_guard<std::mutex> lk(mu_);
        ++st_.turns;
        st_.hist.push_back({turn.stamp, turn.samples});
        while (st_.hist.size() > 1 && turn.stamp - st_.hist.front().first > 5.0) st_.hist.pop_front();
    }

    // -- /lidar/status --------------------------------------------------------------------------------------------
    struct Status {
        bool scanning = false;
        std::string port, model, firmware, serial, health, mode, error;
        int hardware = 0, health_code = 0;
        double rate = 0, ppm = 0, spread = 0;
        int64_t bytes = 0, bad = 0, lost = 0, breaks = 0, lines = 0, turns = 0, dropped = 0;
        std::deque<std::pair<double, int>> hist;   // (stamp, samples) of the last 5 s of turns
    };

    void set_state(bool scanning, const std::string& err) {
        std::lock_guard<std::mutex> lk(mu_);
        st_.scanning = scanning, st_.error = err;
        if (!scanning) st_.hist.clear();
    }

    void status() {
        JsonObject j;
        std::lock_guard<std::mutex> lk(mu_);
        double hz = NAN, spt = NAN;
        if (st_.hist.size() >= 2) {
            double dt = st_.hist.back().first - st_.hist.front().first;
            int64_t n = 0;
            for (size_t i = 1; i < st_.hist.size(); ++i) n += st_.hist[i].second;
            hz = (double)(st_.hist.size() - 1) / dt, spt = (double)n / (double)(st_.hist.size() - 1);
        }
        j.boolean("scanning", st_.scanning).string("port", st_.port).string("model", st_.model)
            .string("firmware", st_.firmware).integer("hardware", st_.hardware).string("serial", st_.serial)
            .string("health", st_.health).integer("health_code", st_.health_code).string("mode", st_.mode)
            .number("samples_per_s", st_.rate, 0).integer("pwm", pwm_).number("hz", hz, 2)
            .number("samples_per_turn", spt, 1).integer("turns", st_.turns).integer("bytes", st_.bytes)
            .integer("bad_bytes", st_.bad).integer("lost_packets", st_.lost).integer("clock_breaks", st_.breaks)
            .integer("clock_lines", st_.lines).number("clock_ppm", st_.ppm, 1).number("clock_spread_ms", st_.spread * 1e3, 3)
            .number("latency_ms", latency_ * 1e3, 3).integer("seam_dropped", st_.dropped);
        if (st_.error.empty()) j.null("error");
        else j.string("error", st_.error);
        std_msgs::msg::String m;
        m.data = j.str();
        p_status_->publish(m);
    }

    std::string port_, mode_, frame_;
    std::atomic<int64_t> pwm_;
    int bins_;
    double range_min_, range_max_, window_, spin_up_;
    std::atomic<double> latency_;
    std::atomic<bool> pwm_dirty_{false}, latency_dirty_{false}, stop_{false};
    bool has_conf_ = false;
    int fd_ = -1;
    std::mutex write_mu_, mu_;
    Status st_;
    std::unique_ptr<params::Live> live_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr p_scan_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_status_;
    rclcpp::TimerBase::SharedPtr t_status_;
    std::thread thread_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::RplidarNode)
