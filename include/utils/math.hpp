// utils/math.hpp -- standalone 3D math for LinearBVH broadphase.
//
// Slimmed from the full MLSH math.hpp: no SIMD dispatch, no Int3 / voxel
// helpers, just the Vec3 / AABB types and ray-geometry functions that
// LinearBVH needs for broadphase queries and narrowphase raycast fallback.
#ifndef UTILS_MATH_HPP
#define UTILS_MATH_HPP

#include <algorithm>
#include <cmath>
#include <utility>

namespace utils {

// --- Scalar float vector ----------------------------------------------------
struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }

    constexpr bool operator==(const Vec3& o) const { return x == o.x && y == o.y && z == o.z; }
    constexpr bool operator!=(const Vec3& o) const { return !(*this == o); }
};

inline constexpr Vec3 operator*(float s, const Vec3& v) { return v * s; }

// --- Basic vector math (scalar, compiler auto-vectorizes) --------------------

inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline float lengthSq(const Vec3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

inline float length(const Vec3& v) {
    return std::sqrt(lengthSq(v));
}

inline Vec3 normalized(const Vec3& v) {
    float len = length(v);
    return len > 0.0f ? v / len : Vec3{};
}

inline float distanceSq(const Vec3& a, const Vec3& b) {
    return lengthSq(a - b);
}

inline Vec3 vmin(const Vec3& a, const Vec3& b) {
    return {std::fminf(a.x, b.x), std::fminf(a.y, b.y), std::fminf(a.z, b.z)};
}

inline Vec3 vmax(const Vec3& a, const Vec3& b) {
    return {std::fmaxf(a.x, b.x), std::fmaxf(a.y, b.y), std::fmaxf(a.z, b.z)};
}

inline Vec3 vclamp(const Vec3& v, const Vec3& lo, const Vec3& hi) {
    return {std::fmaxf(lo.x, std::fminf(hi.x, v.x)),
            std::fmaxf(lo.y, std::fminf(hi.y, v.y)),
            std::fmaxf(lo.z, std::fminf(hi.z, v.z))};
}

inline float maxComponent(const Vec3& v) {
    return std::fmaxf(std::fmaxf(v.x, v.y), v.z);
}

// --- Axis-aligned bounding box ----------------------------------------------
struct AABB {
    Vec3 min;
    Vec3 max;

    constexpr AABB() = default;
    constexpr AABB(const Vec3& mn, const Vec3& mx) : min(mn), max(mx) {}

    static AABB fromCenterRadius(const Vec3& c, float r) {
        return {c - Vec3(r), c + Vec3(r)};
    }
    Vec3 center() const { return (min + max) * 0.5f; }
    Vec3 extent() const { return max - min; }

    bool overlaps(const AABB& o) const {
        return max.x >= o.min.x && min.x <= o.max.x &&
               max.y >= o.min.y && min.y <= o.max.y &&
               max.z >= o.min.z && min.z <= o.max.z;
    }
    bool contains(const Vec3& p) const {
        return p.x >= min.x && p.x <= max.x &&
               p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }
};

// Voxel tolerance -- used as a fudge factor in ray-box tests.
constexpr float kVoxelEps = 1e-6f;

// --- Ray vs box slab test ---------------------------------------------------
// Returns true if the ray hits within [0, maxDist]; tEntry is the entry
// parameter (0 if the origin is already inside).
inline bool rayBoxEntry(const Vec3& origin, const Vec3& dir,
                        const Vec3& bmin, const Vec3& bmax,
                        float maxDist, float eps, float& tEntry) {
    float tmin = 0.0f;
    float tmax = maxDist;

    // X slab
    {
        float invD = 1.0f / dir.x;
        float t0 = (bmin.x - origin.x) * invD;
        float t1 = (bmax.x - origin.x) * invD;
        if (invD < 0.0f) std::swap(t0, t1);
        tmin = std::fmaxf(t0, tmin);
        tmax = std::fminf(t1, tmax);
        if (tmin > tmax + eps) return false;
    }
    // Y slab
    {
        float invD = 1.0f / dir.y;
        float t0 = (bmin.y - origin.y) * invD;
        float t1 = (bmax.y - origin.y) * invD;
        if (invD < 0.0f) std::swap(t0, t1);
        tmin = std::fmaxf(t0, tmin);
        tmax = std::fminf(t1, tmax);
        if (tmin > tmax + eps) return false;
    }
    // Z slab
    {
        float invD = 1.0f / dir.z;
        float t0 = (bmin.z - origin.z) * invD;
        float t1 = (bmax.z - origin.z) * invD;
        if (invD < 0.0f) std::swap(t0, t1);
        tmin = std::fmaxf(t0, tmin);
        tmax = std::fminf(t1, tmax);
        if (tmin > tmax + eps) return false;
    }

    tEntry = tmin >= 0.0f ? tmin : (tmax >= 0.0f ? 0.0f : tmax);
    return tEntry <= maxDist;
}

// --- Exact ray-geometry narrowphase tests -----------------------------------
// `dir` is assumed normalized. Each returns the nearest entry parameter in
// [0, maxDist], or 0 if the origin already lies inside the solid. These back
// the Collider narrowphase so a raycast reports a true surface hit.

inline bool raySphere(const Vec3& o, const Vec3& dir, const Vec3& center,
                      float radius, float maxDist, float& tEntry) {
    Vec3 oc = o - center;
    float b = dot(oc, dir);
    float c = dot(oc, oc) - radius * radius;
    if (c > 0.0f && b > 0.0f) return false;
    float disc = b * b - c;
    if (disc < 0.0f) return false;
    float t0 = -b - std::sqrt(disc);
    float t = t0 >= 0.0f ? t0 : 0.0f;
    if (t > maxDist) return false;
    tEntry = t;
    return true;
}

// Möller–Trumbore, double-sided.
inline bool rayTriangle(const Vec3& o, const Vec3& dir, const Vec3& a,
                        const Vec3& b, const Vec3& c, float maxDist, float& tEntry) {
    constexpr float kEps = 1e-6f;
    Vec3 e1 = b - a, e2 = c - a;
    Vec3 p = cross(dir, e2);
    float det = dot(e1, p);
    if (std::abs(det) < 1e-12f) return false;
    float inv = 1.0f / det;
    Vec3 tv = o - a;
    float u = dot(tv, p) * inv;
    if (u < -kEps || u > 1.0f + kEps) return false;
    Vec3 q = cross(tv, e1);
    float v = dot(dir, q) * inv;
    if (v < -kEps || u + v > 1.0f + kEps) return false;
    float t = dot(e2, q) * inv;
    if (t < 0.0f || t > maxDist) return false;
    tEntry = t;
    return true;
}

// Ray vs convex solid given as inward half-spaces. Each plane is {n, off}
// with a point p inside the solid iff dot(n, p) <= off.
inline bool rayConvex(const Vec3& o, const Vec3& dir,
                      const std::pair<Vec3, float>* planes, int count,
                      float maxDist, float& tEntry) {
    constexpr float kEps = 1e-6f;
    float tEnter = 0.0f;
    float tExit = maxDist;
    for (int i = 0; i < count; ++i) {
        float denom = dot(planes[i].first, dir);
        float num = planes[i].second - dot(planes[i].first, o);
        if (std::abs(denom) < 1e-12f) {
            if (num < -kEps) return false;
        } else if (denom > 0.0f) {
            tExit = std::fminf(tExit, num / denom);
        } else {
            tEnter = std::fmaxf(tEnter, num / denom);
        }
        if (tEnter > tExit) return false;
    }
    tEntry = tEnter;
    return true;
}

// --- Feature size helpers (from voxelize.hpp) --------------------------------
// Largest extent of each shape; used to validate collider registration.

inline float sphereFeatureSize(float radius) { return radius * 2.0f; }
inline float boxFeatureSize(const AABB& box) { return maxComponent(box.extent()); }
inline float triangleFeatureSize(const Vec3& a, const Vec3& b, const Vec3& c) {
    return maxComponent(vmax(vmax(a, b), c) - vmin(vmin(a, b), c));
}
inline float tetrahedronFeatureSize(const Vec3& a, const Vec3& b,
                                     const Vec3& c, const Vec3& d) {
    return maxComponent(vmax(vmax(a, b), vmax(c, d)) - vmin(vmin(a, b), vmin(c, d)));
}

namespace detail {

// Inward-facing plane for tetrahedron construction.
// Given three triangle vertices (ccw from outside) and an interior point,
// returns {normal, offset} such that a point p is inside when dot(n, p) <= off.
inline std::pair<Vec3, float> inwardPlane(const Vec3& p0, const Vec3& p1,
                                          const Vec3& p2, const Vec3& interior) {
    Vec3 n = normalized(cross(p1 - p0, p2 - p0));
    if (dot(n, interior - p0) > 0.0f) n = -n;
    return {n, dot(n, p0)};
}

}  // namespace detail

}  // namespace utils

#endif  // UTILS_MATH_HPP
