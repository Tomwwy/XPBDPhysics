#include "math_internal.hpp"

#include <algorithm>
#include <cmath>

namespace utils::detail {
namespace {

float dotScalar(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 crossScalar(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

float lengthSqScalar(const Vec3& v) {
    return dotScalar(v, v);
}

float distanceSqScalar(const Vec3& a, const Vec3& b) {
    return lengthSqScalar(a - b);
}

Vec3 vminScalar(const Vec3& a, const Vec3& b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}

Vec3 vmaxScalar(const Vec3& a, const Vec3& b) {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

Vec3 vclampScalar(const Vec3& v, const Vec3& lo, const Vec3& hi) {
    return {std::clamp(v.x, lo.x, hi.x),
            std::clamp(v.y, lo.y, hi.y),
            std::clamp(v.z, lo.z, hi.z)};
}

float maxComponentScalar(const Vec3& v) {
    return std::max(std::max(v.x, v.y), v.z);
}

bool aabbOverlapsScalar(const Vec3& amin, const Vec3& amax,
                        const Vec3& bmin, const Vec3& bmax) {
    return amin.x <= bmax.x && amax.x >= bmin.x &&
           amin.y <= bmax.y && amax.y >= bmin.y &&
           amin.z <= bmax.z && amax.z >= bmin.z;
}

bool aabbContainsScalar(const Vec3& amin, const Vec3& amax, const Vec3& p) {
    return p.x >= amin.x && p.x <= amax.x &&
           p.y >= amin.y && p.y <= amax.y &&
           p.z >= amin.z && p.z <= amax.z;
}

bool rayBoxEntryScalar(const Vec3& origin, const Vec3& dir,
                       const Vec3& bmin, const Vec3& bmax,
                       float maxDist, float eps, float& tEntry) {
    constexpr float kParallelDirEps = 1e-12f;
    float tmin = 0.0f;
    float tmax = maxDist;

    const auto updateSlab = [&](float o, float d, float mn, float mx) {
        if (std::abs(d) < kParallelDirEps) {
            return o >= mn && o <= mx;
        }

        float invD = 1.0f / d;
        float t0 = (mn - o) * invD;
        float t1 = (mx - o) * invD;
        if (invD < 0.0f) std::swap(t0, t1);
        tmin = std::fmaxf(t0, tmin);
        tmax = std::fminf(t1, tmax);
        return tmin <= tmax + eps;
    };

    if (!updateSlab(origin.x, dir.x, bmin.x, bmax.x)) return false;
    if (!updateSlab(origin.y, dir.y, bmin.y, bmax.y)) return false;
    if (!updateSlab(origin.z, dir.z, bmin.z, bmax.z)) return false;

    tEntry = tmin >= 0.0f ? tmin : (tmax >= 0.0f ? 0.0f : tmax);
    return tEntry <= maxDist;
}

}  // namespace

const MathOps& scalarMathOps() {
    static const MathOps ops{
        dotScalar,
        crossScalar,
        lengthSqScalar,
        distanceSqScalar,
        vminScalar,
        vmaxScalar,
        vclampScalar,
        maxComponentScalar,
        aabbOverlapsScalar,
        aabbContainsScalar,
        rayBoxEntryScalar,
    };
    return ops;
}

}  // namespace utils::detail
