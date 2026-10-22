// Small math helpers shared by the firmware core. No Arduino, no heap: the same code builds for the Teensy 4.1 and
// for the PC (the SIL library the MuJoCo sim calls, design.md 30).
#pragma once

#include <math.h>
#include <stdint.h>

namespace fw {

constexpr float PI_F = 3.14159265358979f;

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

inline float wrap_angle(float a) {
    a = fmodf(a + PI_F, 2.0f * PI_F);
    if (a < 0.0f) a += 2.0f * PI_F;
    return a - PI_F;
}

struct Vec2 {
    float x = 0.0f, y = 0.0f;
    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float k) const { return {x * k, y * k}; }
    float dot(const Vec2& o) const { return x * o.x + y * o.y; }
    float norm() const { return sqrtf(x * x + y * y); }
};

// world (odom) <-> body frame for a heading yaw
inline Vec2 world_to_body(const Vec2& v, float yaw) {
    float c = cosf(yaw), s = sinf(yaw);
    return {c * v.x + s * v.y, -s * v.x + c * v.y};
}
inline Vec2 body_to_world(const Vec2& v, float yaw) {
    float c = cosf(yaw), s = sinf(yaw);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}

}  // namespace fw
