#ifndef XPBD_CONTACT_CONSTRAINT_HPP
#define XPBD_CONTACT_CONSTRAINT_HPP

#include "xpbd/entity.hpp"
#include "xpbd/math.hpp"

namespace xpbd {

// A transient collision constraint produced by narrowphase each substep and
// solved in the constraint-iteration loop with lambda accumulation, exactly
// like a DistanceConstraint. Unlike persistent constraints these live in a
// per-substep buffer, not a TypedStore.
//
// The bodies are resolved generically: the contact solver dispatches the
// positional correction onto whatever body type each entity is (a Particle
// today; a RigidBody with a lever arm later). `normal` points from A to B.
struct ContactConstraint {
    Entity colliderA;     // source colliders (for re-evaluating geometry)
    Entity colliderB;
    Entity bodyA;         // bodies to correct (may be invalid => static)
    Entity bodyB;
    Vec3 normal{};        // unit, A -> B; fallback direction if coincident
    Vec3 point{};         // world-space contact point; lever arm for rigid bodies
    float compliance = 0.0f;
    float lambda = 0.0f;  // accumulated over solver iterations within a substep
};

}  // namespace xpbd

#endif  // XPBD_CONTACT_CONSTRAINT_HPP
