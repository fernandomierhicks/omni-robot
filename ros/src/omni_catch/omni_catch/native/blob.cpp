// The ball detector's pixel work in C++ (design.md 7: "a C++ or GPU detector"), a drop-in for omni_catch.detector's
// numpy + scipy path: the colour test (ball_mask), 4-connected components labelled in raster order of their first
// pixel (as scipy.ndimage.label numbers them), and each component's moments (Detector._measure). The tracking logic
// stays in Python (fast_detector.py); only the per-pixel loops move here. Built on demand by native/build.py (g++ on
// Linux and the Jetson, zig c++ on Windows), loaded with ctypes. No allocation per call beyond a small scratch buffer.
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#define BD_API extern "C" __declspec(dllexport)
#else
#define BD_API extern "C" __attribute__((visibility("default")))
#endif

struct Params {
    float hue_lo, hue_hi, min_sat, min_val;
};

// detector.ball_mask, pixel by pixel, in the same float32 arithmetic
static inline bool is_ball(const uint8_t* px, const Params& p) {
    int r = px[0], g = px[1], b = px[2];
    int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
    int chroma = mx - mn;
    if (!((float)mx >= p.min_val * 255.0f)) return false;
    if (!((float)chroma >= p.min_sat * (float)(mx > 1 ? mx : 1))) return false;
    float cz = (float)(chroma > 1 ? chroma : 1);
    float h;
    if (mx == r) {
        h = 60.0f * ((float)(g - b) / cz);
        h = fmodf(h, 360.0f);
        if (h < 0.0f) h += 360.0f;
    } else if (mx == g) {
        h = 60.0f * ((float)(b - r) / cz + 2.0f);
    } else {
        h = 60.0f * ((float)(r - g) / cz + 4.0f);
    }
    return h >= p.hue_lo && h <= p.hue_hi;
}

static uint8_t* g_mask = nullptr;
static int32_t* g_lab = nullptr;
static int32_t* g_stack = nullptr;
static size_t g_cap = 0;

static bool reserve(size_t n) {
    if (n <= g_cap) return true;
    free(g_mask);
    free(g_lab);
    free(g_stack);
    g_mask = (uint8_t*)malloc(n);
    g_lab = (int32_t*)malloc(n * sizeof(int32_t));
    g_stack = (int32_t*)malloc(n * sizeof(int32_t));
    g_cap = (g_mask && g_lab && g_stack) ? n : 0;
    return g_cap != 0;
}

// Label the mask of an h x w window (pixel (x, y) at p + y * row + x * px). For each component in label order, call
// on_pixel for its pixels via the accumulator arrays. Returns the number of components, or -1 (no memory).
template <typename F>
static int label(const uint8_t* p, int h, int w, long row, long px, const Params& prm, F on_component) {
    size_t n = (size_t)h * (size_t)w;
    if (n == 0) return 0;
    if (!reserve(n)) return -1;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) g_mask[(size_t)y * w + x] = is_ball(p + y * row + x * px, prm) ? 1 : 0;
    memset(g_lab, 0, n * sizeof(int32_t));
    int k = 0;
    for (size_t i0 = 0; i0 < n; ++i0) {
        if (!g_mask[i0] || g_lab[i0]) continue;
        ++k;
        size_t top = 0;
        g_stack[top++] = (int32_t)i0;
        g_lab[i0] = k;
        on_component(-1, 0, 0);                               // a new component starts
        while (top) {
            int32_t i = g_stack[--top];
            int y = i / w, x = i % w;
            on_component(k, x, y);
            const int nb[4][2] = {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
            for (auto& q : nb) {
                if (q[0] < 0 || q[1] < 0 || q[0] >= w || q[1] >= h) continue;
                int32_t j = q[1] * w + q[0];
                if (g_mask[j] && !g_lab[j]) {
                    g_lab[j] = k;
                    g_stack[top++] = j;
                }
            }
        }
    }
    return k;
}

// Components of a window measured as Detector._measure does: out[6 * i] = (uc, vc, b, n, ok, 0), image coordinates
// (u0, v0: the window's corner in the image; W, H: the image's size for the border test). Returns how many (<= cap).
BD_API int bd_components(const uint8_t* p, int h, int w, long row, long px, int u0, int v0, int W, int H,
                         float hue_lo, float hue_hi, float min_sat, float min_val, int min_px, double* out, int cap) {
    Params prm{hue_lo, hue_hi, min_sat, min_val};
    double su = 0, sv = 0, suu = 0, suv = 0, svv = 0;
    long cnt = 0;
    bool border = false;
    int nout = 0;
    auto flush = [&]() {
        if (cnt == 0) return;
        if (nout < cap) {
            double uc = su / cnt, vc = sv / cnt;
            // central second moments: sum((u - uc)^2) / n, in the order numpy computes du @ du
            double cuu = suu / cnt - uc * uc, cuv = suv / cnt - uc * vc, cvv = svv / cnt - vc * vc;
            double tr = 0.5 * (cuu + cvv), det = cuu * cvv - cuv * cuv;
            double disc = tr * tr - det;
            double lam0 = tr - sqrt(disc > 0 ? disc : 0);
            double b = 2.0 * sqrt(lam0 - 1.0 / 12.0 > 1e-6 ? lam0 - 1.0 / 12.0 : 1e-6);
            double* o = out + 6 * nout;
            o[0] = uc; o[1] = vc; o[2] = b; o[3] = (double)cnt;
            o[4] = (cnt >= min_px && !border) ? 1.0 : 0.0; o[5] = 0.0;
            ++nout;
        }
        su = sv = suu = suv = svv = 0;
        cnt = 0;
        border = false;
    };
    int k = label(p, h, w, row, px, prm, [&](int kk, int x, int y) {
        if (kk < 0) { flush(); return; }
        double u = x + u0 + 0.5, v = y + v0 + 0.5;
        su += u; sv += v; suu += u * u; suv += u * v; svv += v * v;
        ++cnt;
        if (u < 1 || v < 1 || u > W - 1 || v > H - 1) border = true;
    });
    flush();
    return k < 0 ? -1 : nout;
}

// The coarse view's components: out[5 * i] = (size, x_min, x_max, y_min, y_max) in coarse pixels, label order.
BD_API int bd_coarse(const uint8_t* p, int h, int w, long row, long px, float hue_lo, float hue_hi, float min_sat,
                     float min_val, int32_t* out, int cap) {
    Params prm{hue_lo, hue_hi, min_sat, min_val};
    int nout = -1;
    int k = label(p, h, w, row, px, prm, [&](int kk, int x, int y) {
        if (kk < 0) {
            ++nout;
            if (nout < cap) {
                int32_t* o = out + 5 * nout;
                o[0] = 0; o[1] = 1 << 30; o[2] = -1; o[3] = 1 << 30; o[4] = -1;
            }
            return;
        }
        if (nout >= cap) return;
        int32_t* o = out + 5 * nout;
        ++o[0];
        if (x < o[1]) o[1] = x;
        if (x > o[2]) o[2] = x;
        if (y < o[3]) o[3] = y;
        if (y > o[4]) o[4] = y;
    });
    if (k < 0) return -1;
    return k < cap ? k : cap;
}
