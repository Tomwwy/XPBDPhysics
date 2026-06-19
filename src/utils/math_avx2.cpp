#include "math_internal.hpp"

#include <algorithm>
#include <immintrin.h>

namespace utils::detail {
namespace {

__m128 load3(const Vec3& v) {
    return _mm_set_ps(0.0f, v.z, v.y, v.x);
}

__m128 load3w(const Vec3& v, float w) {
    return _mm_set_ps(w, v.z, v.y, v.x);
}

Vec3 store3(__m128 v) {
    float out[4];
    _mm_storeu_ps(out, v);
    return {out[0], out[1], out[2]};
}

float sum3(__m128 v) {
    float out[4];
    _mm_storeu_ps(out, v);
    return out[0] + out[1] + out[2];
}

float min3(__m128 v) {
    float out[4];
    _mm_storeu_ps(out, v);
    return std::min(std::min(out[0], out[1]), out[2]);
}

float max3(__m128 v) {
    float out[4];
    _mm_storeu_ps(out, v);
    return std::max(std::max(out[0], out[1]), out[2]);
}

float dotAvx2(const Vec3& a, const Vec3& b) {
    return sum3(_mm_mul_ps(load3(a), load3(b)));
}

Vec3 crossAvx2(const Vec3& a, const Vec3& b) {
    const __m128 va = load3(a);
    const __m128 vb = load3(b);
    const __m128 aYzx = _mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 0, 2, 1));
    const __m128 bYzx = _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 0, 2, 1));
    const __m128 aZxy = _mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 1, 0, 2));
    const __m128 bZxy = _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 1, 0, 2));
    return store3(_mm_sub_ps(_mm_mul_ps(aYzx, bZxy), _mm_mul_ps(aZxy, bYzx)));
}

float lengthSqAvx2(const Vec3& v) {
    return dotAvx2(v, v);
}

float distanceSqAvx2(const Vec3& a, const Vec3& b) {
    const __m128 d = _mm_sub_ps(load3(a), load3(b));
    return sum3(_mm_mul_ps(d, d));
}

Vec3 vminAvx2(const Vec3& a, const Vec3& b) {
    return store3(_mm_min_ps(load3(a), load3(b)));
}

Vec3 vmaxAvx2(const Vec3& a, const Vec3& b) {
    return store3(_mm_max_ps(load3(a), load3(b)));
}

Vec3 vclampAvx2(const Vec3& v, const Vec3& lo, const Vec3& hi) {
    return store3(_mm_min_ps(_mm_max_ps(load3(v), load3(lo)), load3(hi)));
}

float maxComponentAvx2(const Vec3& v) {
    return max3(load3(v));
}

bool aabbOverlapsAvx2(const Vec3& amin, const Vec3& amax,
                      const Vec3& bmin, const Vec3& bmax) {
    const __m256 lhs = _mm256_set_ps(0.0f, amax.z, amax.y, amax.x,
                                     0.0f, amin.z, amin.y, amin.x);
    const __m256 rhs = _mm256_set_ps(0.0f, bmin.z, bmin.y, bmin.x,
                                     0.0f, bmax.z, bmax.y, bmax.x);
    const int leMask = _mm256_movemask_ps(_mm256_cmp_ps(lhs, rhs, _CMP_LE_OQ));
    const int geMask = _mm256_movemask_ps(_mm256_cmp_ps(lhs, rhs, _CMP_GE_OQ));
    return (leMask & 0x07) == 0x07 && (geMask & 0x70) == 0x70;
}

bool aabbContainsAvx2(const Vec3& amin, const Vec3& amax, const Vec3& p) {
    const __m256 vp = _mm256_set_ps(0.0f, p.z, p.y, p.x,
                                    0.0f, p.z, p.y, p.x);
    const __m256 bounds = _mm256_set_ps(0.0f, amax.z, amax.y, amax.x,
                                        0.0f, amin.z, amin.y, amin.x);
    const int geMask = _mm256_movemask_ps(_mm256_cmp_ps(vp, bounds, _CMP_GE_OQ));
    const int leMask = _mm256_movemask_ps(_mm256_cmp_ps(vp, bounds, _CMP_LE_OQ));
    return (geMask & 0x07) == 0x07 && (leMask & 0x70) == 0x70;
}

bool rayBoxEntryAvx2(const Vec3& o, const Vec3& d, const Vec3& bmin,
                     const Vec3& bmax, float maxDist, float eps, float& tEntry) {
    const __m128 vd = load3w(d, 1.0f);
    const __m128 absD = _mm_andnot_ps(_mm_set1_ps(-0.0f), vd);
    if ((_mm_movemask_ps(_mm_cmp_ps(absD, _mm_set1_ps(1e-12f), _CMP_LT_OQ)) & 0x7) != 0) {
        return scalarMathOps().rayBoxEntry(o, d, bmin, bmax, maxDist, eps, tEntry);
    }

    const __m128 inv = _mm_div_ps(_mm_set1_ps(1.0f), vd);
    const __m128 vo = load3(o);
    const __m128 t1 = _mm_mul_ps(_mm_sub_ps(load3(bmin), vo), inv);
    const __m128 t2 = _mm_mul_ps(_mm_sub_ps(load3(bmax), vo), inv);
    const __m128 tNear = _mm_min_ps(t1, t2);
    const __m128 tFar = _mm_max_ps(t1, t2);
    const float tmin = std::max(0.0f, max3(tNear));
    const float tmax = std::min(maxDist, min3(tFar));
    if (tmin > tmax + eps) return false;
    tEntry = tmin;
    return tEntry <= maxDist;
}

}  // namespace

const MathOps& avx2MathOps() {
    static const MathOps ops{
        dotAvx2,
        crossAvx2,
        lengthSqAvx2,
        distanceSqAvx2,
        vminAvx2,
        vmaxAvx2,
        vclampAvx2,
        maxComponentAvx2,
        aabbOverlapsAvx2,
        aabbContainsAvx2,
        rayBoxEntryAvx2,
    };
    return ops;
}

}  // namespace utils::detail
