#ifndef XPBD_COLLISION_SPHERE_HPP
#define XPBD_COLLISION_SPHERE_HPP

#include "xpbd/collision_filter.hpp"
#include "xpbd/math.hpp"

namespace xpbd {

struct CollisionSphere {
    Vec3 center;
    float radius = 0.5f;
    bool enabled = true;
    CollisionLayerMask collisionLayer = kCollisionLayerDefault;
    CollisionLayerMask collisionMask = kCollisionLayerAll;
};

}  // namespace xpbd

#endif  // XPBD_COLLISION_SPHERE_HPP
