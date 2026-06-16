// utils/collider.hpp -- geometry interface for LinearBVH broadphase.
//
// Slimmed from the full MLSH collider.hpp: no voxelize() method or
// voxelize.hpp dependency. LinearBVH only uses featureSize(), bounds(),
// and raycast() from this interface.
#ifndef UTILS_COLLIDER_HPP
#define UTILS_COLLIDER_HPP

#include "math.hpp"

#include <array>
#include <vector>

namespace utils {

// Abstract geometry. Implementations own their parameters; the BVH only
// borrows a pointer, so the collider must outlive its BVH registration.
class Collider {
public:
    virtual ~Collider() = default;

    // Largest extent of the shape; used to validate the collider.
    virtual float featureSize() const = 0;

    // Analytic bounding box.
    virtual AABB bounds() const = 0;

    // Exact ray vs surface. `dir` is normalized; `t` is returned in world
    // distance. Returns true and the nearest entry parameter in [0, maxDist]
    // (0 when the origin is already inside a solid), false on a clean miss.
    virtual bool raycast(const Vec3& o, const Vec3& dir, float maxDist,
                         float& t) const = 0;
};

// --- Built-in colliders ----------------------------------------------------

class SphereCollider final : public Collider {
public:
    SphereCollider(const Vec3& center, float radius)
        : center_(center), radius_(radius) {}

    float featureSize() const override { return sphereFeatureSize(radius_); }
    AABB bounds() const override { return AABB::fromCenterRadius(center_, radius_); }
    bool raycast(const Vec3& o, const Vec3& dir, float maxDist,
                 float& t) const override {
        return raySphere(o, dir, center_, radius_, maxDist, t);
    }

    const Vec3& center() const { return center_; }
    float radius() const { return radius_; }

private:
    Vec3 center_;
    float radius_;
};

class BoxCollider final : public Collider {
public:
    explicit BoxCollider(const AABB& box) : box_(box) {}

    float featureSize() const override { return boxFeatureSize(box_); }
    AABB bounds() const override { return box_; }
    bool raycast(const Vec3& o, const Vec3& dir, float maxDist,
                 float& t) const override {
        return rayBoxEntry(o, dir, box_.min, box_.max, maxDist, kVoxelEps, t);
    }

    const AABB& box() const { return box_; }

private:
    AABB box_;
};

class TriangleCollider final : public Collider {
public:
    TriangleCollider(const Vec3& a, const Vec3& b, const Vec3& c)
        : a_(a), b_(b), c_(c) {}

    float featureSize() const override {
        return triangleFeatureSize(a_, b_, c_);
    }
    AABB bounds() const override {
        return {vmin(vmin(a_, b_), c_), vmax(vmax(a_, b_), c_)};
    }
    bool raycast(const Vec3& o, const Vec3& dir, float maxDist,
                 float& t) const override {
        return rayTriangle(o, dir, a_, b_, c_, maxDist, t);
    }

private:
    Vec3 a_, b_, c_;
};

class TetrahedronCollider final : public Collider {
public:
    TetrahedronCollider(const Vec3& a, const Vec3& b, const Vec3& c,
                        const Vec3& d)
        : a_(a), b_(b), c_(c), d_(d) {
        planes_ = {detail::inwardPlane(a, b, c, d),
                   detail::inwardPlane(a, b, d, c),
                   detail::inwardPlane(a, c, d, b),
                   detail::inwardPlane(b, c, d, a)};
    }

    float featureSize() const override {
        return tetrahedronFeatureSize(a_, b_, c_, d_);
    }
    AABB bounds() const override {
        return {vmin(vmin(a_, b_), vmin(c_, d_)),
                vmax(vmax(a_, b_), vmax(c_, d_))};
    }
    bool raycast(const Vec3& o, const Vec3& dir, float maxDist,
                 float& t) const override {
        return rayConvex(o, dir, planes_.data(),
                         static_cast<int>(planes_.size()), maxDist, t);
    }

private:
    Vec3 a_, b_, c_, d_;
    std::array<std::pair<Vec3, float>, 4> planes_;
};

}  // namespace utils

#endif  // UTILS_COLLIDER_HPP
