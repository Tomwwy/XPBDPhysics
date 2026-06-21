#ifndef XPBD_NARROWPHASE_SPHERE_SPHERE_HPP
#define XPBD_NARROWPHASE_SPHERE_SPHERE_HPP

#include "xpbd/colliders/world_shape.hpp"
#include "xpbd/math.hpp"
#include "xpbd/narrowphase/contact.hpp"

#include <cmath>

namespace xpbd::narrowphase {

// Sphere vs sphere. `fallbackNormal` is used only when the centers are
// coincident (no defined separating direction); callers pass a deterministic
// axis there so the result is stable frame to frame. Returns a non-touching
// Contact when the spheres are disjoint.
inline Contact sphereSphere(const WorldSphere& a, const WorldSphere& b,
                            const Vec3& fallbackNormal = Vec3{0.0f, 1.0f, 0.0f}) {
    const float minDistance = a.radius + b.radius;
    Contact contact;
    if (!(minDistance > 0.0f)) {
        return contact;  // degenerate radii
    }

    const Vec3 delta = b.center - a.center;
    const float distSq = lengthSq(delta);
    if (distSq >= minDistance * minDistance) {
        return contact;  // not touching
    }

    if (distSq > 1e-12f) {
        const float dist = std::sqrt(distSq);
        contact.normal = delta / dist;
        contact.penetration = minDistance - dist;
    } else {
        contact.normal = fallbackNormal;
        contact.penetration = minDistance;
    }
    contact.touching = true;
    return contact;
}

}  // namespace xpbd::narrowphase

#endif  // XPBD_NARROWPHASE_SPHERE_SPHERE_HPP
