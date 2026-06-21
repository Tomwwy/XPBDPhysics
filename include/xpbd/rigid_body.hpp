#ifndef XPBD_RIGID_BODY_HPP
#define XPBD_RIGID_BODY_HPP

#include "xpbd/mat3.hpp"
#include "xpbd/math.hpp"
#include "xpbd/quat.hpp"

namespace xpbd {

// A rigid body: six DOFs (position + orientation) integrated by XPBD. Like a
// Particle it stores a previous state so velocities are recovered from the
// position/orientation delta after the solve, but it adds an orientation and a
// rotational inertia so contacts apply torque through a lever arm.
//
// Collision geometry rides on separate Colliders that reference this body; the
// body supplies the world transform (position + orientation) and the collider
// supplies the local-space shape and offset. A tetrahedron rigid body, for
// instance, is one RigidBody with four sphere colliders at its vertex offsets.
struct RigidBody {
    Vec3 position{};
    Vec3 previousPosition{};
    Vec3 velocity{};

    Quat orientation = Quat::identity();
    Quat previousOrientation = Quat::identity();
    Vec3 angularVelocity{};

    Vec3 externalForce{};   // accumulated force  (cleared after integration)
    Vec3 externalTorque{};  // accumulated torque (cleared after integration)

    float inverseMass = 1.0f;
    Mat3 inverseInertiaLocal = Mat3::identity();  // body space; world = R I^-1 R^T
};

// Generalized inverse mass of a body along a unit direction `n`, applied at the
// world-space lever arm `r` (contact point relative to the center of mass):
//   w = invMass + (r x n) . (Iinv_world * (r x n))
// Reduces to plain invMass when r is zero or the body has no rotational inertia.
inline float generalizedInverseMass(float inverseMass,
                                    const Mat3& inverseInertiaWorld,
                                    const Vec3& r,
                                    const Vec3& n) {
    const Vec3 rn = cross(r, n);
    return inverseMass + dot(rn, inverseInertiaWorld * rn);
}

}  // namespace xpbd

#endif  // XPBD_RIGID_BODY_HPP
