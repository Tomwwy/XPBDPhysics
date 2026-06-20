#ifndef XPBD_COLLIDER_HPP
#define XPBD_COLLIDER_HPP

#include "linear_bvh.hpp"
#include "utils/shapes.hpp"
#include "xpbd/broadphase_partition.hpp"
#include "xpbd/collision_filter.hpp"
#include "xpbd/entity.hpp"
#include "xpbd/math.hpp"

namespace xpbd {

using utils::Shape;
using utils::ShapeType;

// A collider attaches collision geometry and filtering to a body (a Particle
// today; a RigidBody later). It is orthogonal to the body's DOFs: the body
// supplies the world transform, the collider supplies the shape, the layer/mask
// decide who it collides with, and the partition decides which broadphase tree
// holds its proxy.
//
// The collider owns its broadphase proxy handle directly. That single field
// replaces the old parallel ref-array + entity->object map: the proxy's user
// data packs this collider's Entity, so a broadphase pair resolves straight
// back to the collider, and the handle here drives update/remove.
struct Collider {
    Entity body;                 // the Particle/RigidBody this collider follows
    Shape shape{};               // local-space geometry
    Vec3 offset{};               // added to the body's world origin; for a
                                 // body-less (static) collider this *is* the
                                 // world placement (body origin treated as 0).
    CollisionLayerMask layer = kCollisionLayerDefault;
    CollisionLayerMask mask = kCollisionLayerAll;
    BroadphasePartition partition = BroadphasePartition::Dynamic;
    bool enabled = true;

    // Broadphase bookkeeping, owned by the collision system.
    utils::LinearBVH::ObjectId proxy = utils::LinearBVH::kInvalid;
};

}  // namespace xpbd

#endif  // XPBD_COLLIDER_HPP
