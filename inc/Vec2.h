#pragma once
#include <cmath>
#include <immintrin.h>

class Vec2 {
public:
    float x;
    float y;

    Vec2() : x(0.0f), y(0.0f) {}
    Vec2(float xVal, float yVal) : x(xVal), y(yVal) {}

    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }

    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    Vec2 operator/(float scalar) const {
        return Vec2(x / scalar, y / scalar);
    }

    Vec2& operator+=(const Vec2& other) {
        x += other.x; y += other.y; return *this;
    }

    Vec2& operator-=(const Vec2& other) {
        x -= other.x; y -= other.y; return *this;
    }

    Vec2& operator*=(float scalar) {
        x *= scalar; y *= scalar; return *this;
    }

    Vec2& operator/=(float scalar) {
        x /= scalar; y /= scalar; return *this;
    }

    // Vector Operations
    float Magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    Vec2 Normalize() const {
        float magSquared = x*x + y*y;
        if (magSquared > 0.00001f) {
            // alignas(16) float temp[4] = {magSquared, 0.0f, 0.0f, 0.0f};
            // __m128 ms = _mm_load_ps(temp);
            // __m128 m = _mm_sqrt_ps(vec);
            // __m128 norm = _mm_mul_ps(vec, invMag);
            __m128 reg = _mm_set_ss(magSquared);
            __m128 res = _mm_rsqrt_ss(reg);
            float invMag = _mm_cvtss_f32(res);

            return Vec2(x*invMag, y*invMag);
        }
        return Vec2(0.0f, 0.0f);
    }

    float Dot(const Vec2& other) const {
        return (x * other.x) + (y * other.y);
    }
};