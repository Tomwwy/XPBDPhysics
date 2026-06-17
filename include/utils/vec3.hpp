#ifndef UTILS_VEC3_HPP
#define UTILS_VEC3_HPP

#include <cmath>

namespace utils {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3() = default;
    explicit constexpr Vec3(float scalar) : x(scalar), y(scalar), z(scalar) {}
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    constexpr Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    constexpr Vec3 operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    constexpr Vec3 operator/(float scalar) const { return {x / scalar, y / scalar, z / scalar}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }

    constexpr Vec3& operator+=(const Vec3& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    constexpr Vec3& operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    constexpr Vec3& operator/=(float scalar)
    {
        x /= scalar;
        y /= scalar;
        z /= scalar;
        return *this;
    }

    constexpr bool operator==(const Vec3& rhs) const { return x == rhs.x && y == rhs.y && z == rhs.z; }
    constexpr bool operator!=(const Vec3& rhs) const { return !(*this == rhs); }
};

inline constexpr Vec3 operator*(float scalar, const Vec3& value)
{
    return value * scalar;
}

inline float dot(const Vec3& lhs, const Vec3& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline Vec3 cross(const Vec3& lhs, const Vec3& rhs)
{
    return {lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
}

inline float lengthSquared(const Vec3& value)
{
    return dot(value, value);
}

inline float lengthSq(const Vec3& value)
{
    return lengthSquared(value);
}

inline float length(const Vec3& value)
{
    return std::sqrt(lengthSquared(value));
}

inline Vec3 normalized(const Vec3& value)
{
    const float len = length(value);
    return len > 0.0f ? value / len : Vec3{};
}

inline float distanceSquared(const Vec3& lhs, const Vec3& rhs)
{
    return lengthSquared(lhs - rhs);
}

inline float distanceSq(const Vec3& lhs, const Vec3& rhs)
{
    return distanceSquared(lhs, rhs);
}

inline float distance(const Vec3& lhs, const Vec3& rhs)
{
    return length(lhs - rhs);
}

inline Vec3 vmin(const Vec3& lhs, const Vec3& rhs)
{
    return {std::fminf(lhs.x, rhs.x), std::fminf(lhs.y, rhs.y), std::fminf(lhs.z, rhs.z)};
}

inline Vec3 vmax(const Vec3& lhs, const Vec3& rhs)
{
    return {std::fmaxf(lhs.x, rhs.x), std::fmaxf(lhs.y, rhs.y), std::fmaxf(lhs.z, rhs.z)};
}

inline Vec3 vclamp(const Vec3& value, const Vec3& low, const Vec3& high)
{
    return {std::fmaxf(low.x, std::fminf(high.x, value.x)),
            std::fmaxf(low.y, std::fminf(high.y, value.y)),
            std::fmaxf(low.z, std::fminf(high.z, value.z))};
}

inline float maxComponent(const Vec3& value)
{
    return std::fmaxf(std::fmaxf(value.x, value.y), value.z);
}

}  // namespace utils

#endif  // UTILS_VEC3_HPP
