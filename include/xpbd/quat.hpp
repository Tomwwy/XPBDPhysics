#ifndef XPBD_QUAT_HPP
#define XPBD_QUAT_HPP

#include "xpbd/math.hpp"

#include <cmath>

namespace xpbd {

// Unit quaternion (x, y, z, w) for rigid-body orientation. Stored xyzw so the
// vector part lines up with a Vec3; the identity rotation is (0, 0, 0, 1).
struct Quat {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;

    constexpr Quat() = default;
    constexpr Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    static constexpr Quat identity() { return Quat{0.0f, 0.0f, 0.0f, 1.0f}; }

    // Rotation of `angle` radians about a (not necessarily unit) axis.
    static Quat fromAxisAngle(const Vec3& axis, float angle) {
        const float len = length(axis);
        if (!(len > 0.0f)) {
            return identity();
        }
        const float half = angle * 0.5f;
        const float s = std::sin(half) / len;
        return Quat{axis.x * s, axis.y * s, axis.z * s, std::cos(half)};
    }
};

// Hamilton product: applying `lhs` after `rhs` (lhs * rhs rotates by rhs first).
inline Quat operator*(const Quat& a, const Quat& b) {
    return Quat{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

inline Quat operator*(const Quat& q, float s) {
    return Quat{q.x * s, q.y * s, q.z * s, q.w * s};
}

inline Quat operator+(const Quat& a, const Quat& b) {
    return Quat{a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}

inline Quat normalized(const Quat& q) {
    const float lenSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!(lenSq > 0.0f)) {
        return Quat::identity();
    }
    const float inv = 1.0f / std::sqrt(lenSq);
    return Quat{q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

inline Quat conjugate(const Quat& q) {
    return Quat{-q.x, -q.y, -q.z, q.w};
}

// Rotate a vector by a unit quaternion: v' = q * (0,v) * q^-1, expanded.
inline Vec3 rotate(const Quat& q, const Vec3& v) {
    const Vec3 u{q.x, q.y, q.z};
    const float s = q.w;
    return u * (2.0f * dot(u, v)) + v * (s * s - dot(u, u)) + cross(u, v) * (2.0f * s);
}

// Integrate orientation by an angular velocity (rad/s) over dt, then
// renormalize. q_dot = 0.5 * (0, omega) * q  (standard quaternion kinematics).
inline Quat integrateOrientation(const Quat& q, const Vec3& angularVelocity, float dt) {
    const Quat omega{angularVelocity.x, angularVelocity.y, angularVelocity.z, 0.0f};
    const Quat dq = omega * q * (0.5f * dt);
    return normalized(q + dq);
}

}  // namespace xpbd

#endif  // XPBD_QUAT_HPP
