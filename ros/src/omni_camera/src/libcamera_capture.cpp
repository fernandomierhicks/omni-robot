// libcamera_capture: one CSI camera on the camera head (design.md 4.9) as /cam/<side>/image_raw, the topic
// catch_perception reads with source:=image. Built only against Raspberry Pi's libcamera fork (>= 0.5; Ubuntu's 0.2
// can't drive Pi cameras): characterization/pi_camera/setup/build_libcamera.sh.
//
// The ISP scales the chosen sensor mode to width x height in BGR888 (libcamera's naming, after DRM: R, G, B in
// memory), published as rgb8 with the rows packed. The stamp is the sensor's own SensorTimestamp (start of readout,
// CLOCK_MONOTONIC) moved onto the node's clock, so a frame carries when it was seen, not when it arrived. A
// unique_ptr publish: in a container with use_intra_process_comms the frame reaches catch_perception uncopied.
// /cam/<side>/stats (JSON, 1 Hz): fps from the sensor's stamps, drops, sensor -> published lag, exposure.
// Parameters: side, camera (index), width, height, fps, sensor_mode [w, h, bit depth] ([] lets libcamera pick),
// exposure_us (0: auto exposure, capped by the frame time), gain (with a fixed exposure), buffers, frame_id.
#include <sys/mman.h>
#include <time.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <libcamera/libcamera.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace omni {

namespace lc = libcamera;

namespace {

int64_t mono_ns() {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000 + t.tv_nsec;
}

double pct(std::vector<double> v, double q) {                      // np.percentile's linear interpolation
    if (v.empty()) return NAN;
    std::sort(v.begin(), v.end());
    double pos = q / 100.0 * (v.size() - 1), fl = std::floor(pos);
    size_t i = (size_t)fl, j = std::min(v.size() - 1, i + 1);
    return v[i] + (pos - fl) * (v[j] - v[i]);
}

}  // namespace

class LibcameraCapture : public rclcpp::Node {
public:
    explicit LibcameraCapture(const rclcpp::NodeOptions& opt) : Node("libcamera_capture", opt) {
        side_ = declare_parameter<std::string>("side", "left");
        int index = (int)declare_parameter<int64_t>("camera", 0);
        width_ = (unsigned)declare_parameter<int64_t>("width", 768);
        height_ = (unsigned)declare_parameter<int64_t>("height", 432);
        fps_ = declare_parameter<double>("fps", 120.0);
        auto mode = declare_parameter<std::vector<int64_t>>("sensor_mode", {1536, 864, 10});   // the IMX708's 120 fps
        int64_t exposure_us = declare_parameter<int64_t>("exposure_us", 0);
        double gain = declare_parameter<double>("gain", 4.0);
        unsigned buffers = (unsigned)declare_parameter<int64_t>("buffers", 6);
        frame_id_ = declare_parameter<std::string>("frame_id", "cam_" + side_ + "_optical");

        pub_ = create_publisher<sensor_msgs::msg::Image>("/cam/" + side_ + "/image_raw", 5);
        stats_pub_ = create_publisher<std_msgs::msg::String>("/cam/" + side_ + "/stats", 5);

        cm_ = std::make_unique<lc::CameraManager>();
        if (cm_->start()) throw std::runtime_error("libcamera: the camera manager didn't start");
        auto cams = cm_->cameras();
        if (index < 0 || (size_t)index >= cams.size())
            throw std::runtime_error("libcamera: no camera " + std::to_string(index) + " (" +
                                     std::to_string(cams.size()) + " found)");
        cam_ = cams[index];
        if (cam_->acquire()) throw std::runtime_error("libcamera: " + cam_->id() + " is in use");

        cfg_ = cam_->generateConfiguration({lc::StreamRole::VideoRecording});
        lc::StreamConfiguration& sc = cfg_->at(0);
        sc.pixelFormat = lc::formats::BGR888;
        sc.size = lc::Size(width_, height_);
        sc.bufferCount = buffers;
        if (mode.size() == 3) {
            cfg_->sensorConfig = lc::SensorConfiguration();
            cfg_->sensorConfig->outputSize = lc::Size((unsigned)mode[0], (unsigned)mode[1]);
            cfg_->sensorConfig->bitDepth = (unsigned)mode[2];
        }
        if (cfg_->validate() == lc::CameraConfiguration::Invalid)
            throw std::runtime_error("libcamera: configuration invalid");
        if (sc.pixelFormat != lc::formats::BGR888 || sc.size != lc::Size(width_, height_))
            throw std::runtime_error("libcamera: adjusted to " + sc.toString());
        if (cam_->configure(cfg_.get())) throw std::runtime_error("libcamera: configure failed");
        stream_ = sc.stream();
        stride_ = sc.stride;

        alloc_ = std::make_unique<lc::FrameBufferAllocator>(cam_);
        if (alloc_->allocate(stream_) < 0) throw std::runtime_error("libcamera: buffer allocation failed");
        for (const auto& b : alloc_->buffers(stream_)) {
            const lc::FrameBuffer::Plane& pl = b->planes()[0];
            size_t len = pl.offset + pl.length;
            void* m = mmap(nullptr, len, PROT_READ, MAP_SHARED, pl.fd.get(), 0);
            if (m == MAP_FAILED) throw std::runtime_error("libcamera: mmap failed");
            maps_[b.get()] = {m, len, (const uint8_t*)m + pl.offset};
            auto req = cam_->createRequest();
            if (!req || req->addBuffer(stream_, b.get())) throw std::runtime_error("libcamera: request failed");
            requests_.push_back(std::move(req));
        }

        lc::ControlList ctl(lc::controls::controls);
        frame_us_ = std::llround(1e6 / fps_);
        std::array<int64_t, 2> lim{frame_us_, frame_us_};
        ctl.set(lc::controls::FrameDurationLimits, std::span<const int64_t, 2>(lim));
        if (exposure_us > 0) {
            ctl.set(lc::controls::AeEnable, false);
            ctl.set(lc::controls::ExposureTime, (int32_t)exposure_us);
            ctl.set(lc::controls::AnalogueGain, (float)gain);
        }
        cam_->requestCompleted.connect(this, &LibcameraCapture::on_request);
        if (cam_->start(&ctl)) throw std::runtime_error("libcamera: start failed");
        for (auto& r : requests_) cam_->queueRequest(r.get());

        report_ = create_wall_timer(std::chrono::seconds(1), [this] { report(); });
        RCLCPP_INFO(get_logger(), "%s -> /cam/%s/image_raw: %s at %.0f fps (sensor mode %s)", cam_->id().c_str(),
                    side_.c_str(), sc.toString().c_str(), fps_,
                    cfg_->sensorConfig ? cfg_->sensorConfig->outputSize.toString().c_str() : "auto");
    }

    ~LibcameraCapture() override {
        if (cam_) {
            cam_->stop();
            cam_->requestCompleted.disconnect(this);
        }
        requests_.clear();
        for (auto& [b, m] : maps_) munmap(m.base, m.len);
        if (alloc_) alloc_->free(stream_);
        alloc_.reset();
        if (cam_) cam_->release();
        cam_.reset();
        if (cm_) cm_->stop();
    }

private:
    struct Map {
        void* base;
        size_t len;
        const uint8_t* data;
    };

    // libcamera's thread: copy the frame out, give the buffer back, publish
    void on_request(lc::Request* r) {
        if (r->status() == lc::Request::RequestCancelled) return;
        int64_t t_sensor = r->metadata().get(lc::controls::SensorTimestamp).value_or(0);
        int32_t exposure = r->metadata().get(lc::controls::ExposureTime).value_or(0);
        const uint8_t* px = maps_.at(r->findBuffer(stream_)).data;

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        int64_t offset = now().nanoseconds() - mono_ns();               // CLOCK_MONOTONIC -> the node's clock
        int64_t ns = t_sensor + offset;
        msg->header.stamp.sec = (int32_t)(ns / 1000000000);
        msg->header.stamp.nanosec = (uint32_t)(ns % 1000000000);
        msg->header.frame_id = frame_id_;
        msg->width = width_, msg->height = height_;
        msg->encoding = "rgb8";
        msg->step = width_ * 3;
        if (stride_ == msg->step) {
            msg->data.assign(px, px + (size_t)msg->step * height_);
        } else {
            msg->data.reserve((size_t)msg->step * height_);
            for (unsigned y = 0; y < height_; ++y)
                msg->data.insert(msg->data.end(), px + (size_t)y * stride_, px + (size_t)y * stride_ + msg->step);
        }
        r->reuse(lc::Request::ReuseBuffers);
        cam_->queueRequest(r);
        pub_->publish(std::move(msg));

        std::lock_guard<std::mutex> lk(mu_);
        lag_ms_.push_back((mono_ns() - t_sensor) * 1e-6);
        sensor_ts_.push_back(t_sensor);
        exposure_us_ = exposure;
    }

    void report() {
        std::vector<double> lag;
        std::vector<int64_t> ts;
        int32_t exposure;
        {
            std::lock_guard<std::mutex> lk(mu_);
            lag.assign(lag_ms_.begin(), lag_ms_.end());
            ts.assign(sensor_ts_.begin(), sensor_ts_.end());
            lag_ms_.clear();
            if (sensor_ts_.size() > 1) sensor_ts_.erase(sensor_ts_.begin(), sensor_ts_.end() - 1);   // keep the last
            exposure = exposure_us_;
        }
        int64_t dropped = 0;
        for (size_t i = 1; i < ts.size(); ++i)
            dropped += std::max<int64_t>(0, std::llround((ts[i] - ts[i - 1]) / (frame_us_ * 1e3)) - 1);
        frames_ += (int64_t)lag.size();
        dropped_ += dropped;
        double fps = ts.size() > 1 ? (ts.size() - 1) / ((ts.back() - ts.front()) * 1e-9) : 0.0;
        char buf[400];
        snprintf(buf, sizeof buf,
                 "{\"frames\": %ld, \"dropped\": %ld, \"fps\": %.2f, \"dropped_1s\": %ld, \"lag_ms_median\": %.2f, "
                 "\"lag_ms_95\": %.2f, \"lag_ms_max\": %.2f, \"exposure_us\": %d}",
                 (long)frames_, (long)dropped_, fps, (long)dropped, pct(lag, 50), pct(lag, 95),
                 lag.empty() ? NAN : *std::max_element(lag.begin(), lag.end()), exposure);
        std_msgs::msg::String m;
        m.data = buf;
        stats_pub_->publish(m);
    }

    std::string side_, frame_id_;
    unsigned width_ = 0, height_ = 0, stride_ = 0;
    double fps_ = 120.0;
    int64_t frame_us_ = 8333;
    std::unique_ptr<lc::CameraManager> cm_;
    std::shared_ptr<lc::Camera> cam_;
    std::unique_ptr<lc::CameraConfiguration> cfg_;
    std::unique_ptr<lc::FrameBufferAllocator> alloc_;
    lc::Stream* stream_ = nullptr;
    std::map<const lc::FrameBuffer*, Map> maps_;
    std::vector<std::unique_ptr<lc::Request>> requests_;

    std::mutex mu_;
    std::deque<double> lag_ms_;
    std::vector<int64_t> sensor_ts_;
    int32_t exposure_us_ = 0;
    int64_t frames_ = 0, dropped_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
    rclcpp::TimerBase::SharedPtr report_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::LibcameraCapture)
