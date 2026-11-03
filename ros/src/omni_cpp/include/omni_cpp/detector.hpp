// Colour-blob ball detector (design.md 6, 30, 31): C++ port of omni_catch/detector.py with fast_detector.py's pixel
// loops (native/blob.cpp: the same labels, in raster order of their first pixel, and the same moment sums), so it
// reports the blobs the robot's Python FastDetector does. test/test_cores.py checks both on sparse and RGB frames.
//
// A frame only answers two questions, as a real-time detector at 120 fps asks them: which pixels of a window pass the
// colour test, and which pixels of a decimated view of the whole frame do.
//   RgbFrame     a real image (rgb8 / bgr8): the colour test per pixel (detector.ball_mask's arithmetic)
//   SparseFrame  the sim's frame (omni_interfaces/SparseImage): the ball's and the clutter's pixel indices on a plain
//                background. The Python paints them (ball colour x 0.8-1.0 on a background +-6) and tests the colour;
//                every painted pixel passes and no background pixel can (its chroma is at most 18 of a value >= 92),
//                so the mask is the indices themselves.
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace omni {

struct DetectorConfig {
    double hue_lo = 45.0, hue_hi = 100.0;      // the optic-yellow ball is ~70 deg
    double min_sat = 0.45, min_val = 0.35;
    int min_blob_px = 8, coarse_stride = 4;
    double window_px = 25.0, window_scale = 2.5;
    int max_candidates = 3, acquire_candidates = 40;
    double size_gain = 1.0;
};

class Frame {
public:
    virtual ~Frame() = default;
    // the colour test over the window [u0, u1) x [v0, v1) (inside the frame), row-major into out
    virtual void mask(const DetectorConfig& c, int u0, int v0, int u1, int v1, std::vector<uint8_t>& out) const = 0;
    // the colour test over the frame decimated by s (img[::s, ::s]): rows x cols into out
    virtual void coarse_mask(const DetectorConfig& c, int s, std::vector<uint8_t>& out, int& rows, int& cols) const = 0;
    int width = 0, height = 0;
};

class RgbFrame : public Frame {
public:
    // data: the first pixel; row and px strides in bytes; bgr: the channels' order
    RgbFrame(const uint8_t* data, int width, int height, long row, long px, bool bgr = false);
    void mask(const DetectorConfig& c, int u0, int v0, int u1, int v1, std::vector<uint8_t>& out) const override;
    void coarse_mask(const DetectorConfig& c, int s, std::vector<uint8_t>& out, int& rows, int& cols) const override;

private:
    const uint8_t* p_;
    long row_, px_;
    int ir_, ig_, ib_;
};

class SparseFrame : public Frame {
public:
    SparseFrame(int width, int height, std::vector<int64_t> ball, std::vector<int64_t> clutter);
    void mask(const DetectorConfig& c, int u0, int v0, int u1, int v1, std::vector<uint8_t>& out) const override;
    void coarse_mask(const DetectorConfig& c, int s, std::vector<uint8_t>& out, int& rows, int& cols) const override;
    std::vector<int64_t> ball, clutter;
};

// detector.ball_mask for one pixel, in the same float32 arithmetic
bool is_ball_px(int r, int g, int b, const DetectorConfig& c);

struct DetBlob {
    double u, v, b;
    int n;
};

struct CoarseComp {
    int k, size, x0, x1, y0, y1;      // label (1-based), pixels, extent (coarse pixels)
};

class Detector {
public:
    explicit Detector(const DetectorConfig& c = {}) : cfg(c) {}
    // the ball, or none; pred: (u, v, radius px) where the tracker expects it
    std::optional<DetBlob> detect(const Frame& fr, const double* pred = nullptr);
    // up to n blobs, largest first, from the whole frame (acquisition)
    std::vector<DetBlob> candidates(const Frame& fr, int n);

    DetectorConfig cfg;
    long n_coarse = 0;

private:
    std::vector<DetBlob> components(const Frame& fr, double u0, double v0, double u1, double v1);
    std::vector<CoarseComp> coarse(const Frame& fr);
    std::vector<DetBlob> window(const Frame& fr, const CoarseComp& c);
    std::vector<uint8_t> mask_;
    std::vector<int32_t> lab_, stack_;
};

}  // namespace omni
