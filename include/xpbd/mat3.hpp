#ifndef XPBD_MAT3_HPP
#define XPBD_MAT3_HPP

#include "xpbd/math.hpp"
#include "xpbd/quat.hpp"

namespace xpbd {

// Row-major 3x3 matrix. Used only for rigid-body inertia: the body-space inverse
// inertia is a constant, and the world-space inverse inertia is R * I^-1 * R^T,
// recomputed from the orientation each time a contact needs a lever arm.
struct Mat3 {
    // rows[i] is the i-th row as a Vec3.
    Vec3 rows[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    constexpr Mat3() = default;
    constexpr Mat3(const Vec3& r0, const Vec3& r1, const Vec3& r2) : rows{r0, r1, r2} {}

    static constexpr Mat3 identity() { return Mat3{}; }

    static constexpr Mat3 diagonal(float a, float b, float c) {
        return Mat3{{a, 0.0f, 0.0f}, {0.0f, b, 0.0f}, {0.0f, 0.0f, c}};
    }

    static constexpr Mat3 zero() {
        return Mat3{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    }
};

inline Vec3 operator*(const Mat3& m, const Vec3& v) {
    return {dot(m.rows[0], v), dot(m.rows[1], v), dot(m.rows[2], v)};
}

inline Mat3 transpose(const Mat3& m) {
    return Mat3{{m.rows[0].x, m.rows[1].x, m.rows[2].x},
                {m.rows[0].y, m.rows[1].y, m.rows[2].y},
                {m.rows[0].z, m.rows[1].z, m.rows[2].z}};
}

inline Mat3 operator*(const Mat3& a, const Mat3& b) {
    const Mat3 bt = transpose(b);  // bt.rows[c] is column c of b
    return Mat3{{dot(a.rows[0], bt.rows[0]), dot(a.rows[0], bt.rows[1]), dot(a.rows[0], bt.rows[2])},
                {dot(a.rows[1], bt.rows[0]), dot(a.rows[1], bt.rows[1]), dot(a.rows[1], bt.rows[2])},
                {dot(a.rows[2], bt.rows[0]), dot(a.rows[2], bt.rows[1]), dot(a.rows[2], bt.rows[2])}};
}

// Rotation matrix from a unit quaternion (orthonormal, so its transpose is its
// inverse).
inline Mat3 toMat3(const Quat& q) {
    const float x = q.x, y = q.y, z = q.z, w = q.w;
    const float xx = x * x, yy = y * y, zz = z * z;
    const float xy = x * y, xz = x * z, yz = y * z;
    const float wx = w * x, wy = w * y, wz = w * z;
    return Mat3{{1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz), 2.0f * (xz + wy)},
                {2.0f * (xy + wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx)},
                {2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (xx + yy)}};
}

// World-space inverse inertia: R * Iinv_local * R^T.
inline Mat3 worldInverseInertia(const Quat& orientation, const Mat3& inverseInertiaLocal) {
    const Mat3 r = toMat3(orientation);
    return r * inverseInertiaLocal * transpose(r);
}

// General inverse via cofactors. Returns Mat3::zero() if the matrix is singular,
// which the caller can treat as "no rotational response" (infinite inertia).
inline Mat3 inverse(const Mat3& m) {
    const Vec3& a = m.rows[0];
    const Vec3& b = m.rows[1];
    const Vec3& c = m.rows[2];
    const float det = a.x * (b.y * c.z - b.z * c.y) -
                      a.y * (b.x * c.z - b.z * c.x) +
                      a.z * (b.x * c.y - b.y * c.x);
    if (det == 0.0f) {
        return Mat3::zero();
    }
    const float inv = 1.0f / det;
    return Mat3{{(b.y * c.z - b.z * c.y) * inv, (a.z * c.y - a.y * c.z) * inv, (a.y * b.z - a.z * b.y) * inv},
                {(b.z * c.x - b.x * c.z) * inv, (a.x * c.z - a.z * c.x) * inv, (a.z * b.x - a.x * b.z) * inv},
                {(b.x * c.y - b.y * c.x) * inv, (a.y * c.x - a.x * c.y) * inv, (a.x * b.y - a.y * b.x) * inv}};
}

}  // namespace xpbd

#endif  // XPBD_MAT3_HPP
