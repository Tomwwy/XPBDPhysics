#include "xpbd/xpbd_world.hpp"

namespace xpbd {

// Advance every rigid body one substep with the same Verlet-style scheme the
// particles use: position integrates from the previous-position delta plus
// external acceleration, and orientation integrates from angular velocity.
// Velocities are recovered after the constraint solve (updateRigidBodyVelocities),
// mirroring how particle velocities are recovered from the position delta.
void XPBDWorld::rigidBodyIntegrationSystem(XPBDWorld& world, float dt)
{
    if (!(dt > 0.0f)) {
        return;
    }

    const float dtSq = dt * dt;
    const Vec3 gravity = world.gravity_;
    const float damping = world.damping_;

    world.forEachRigidBody([&](Entity, RigidBody& body) {
        if (body.inverseMass <= 0.0f) {
            body.previousPosition = body.position;
            body.previousOrientation = body.orientation;
            body.velocity = {};
            body.angularVelocity = {};
            body.externalForce = {};
            body.externalTorque = {};
            return;
        }

        // --- Linear: Verlet position step (matches the particle kernel) -------
        const Vec3 currentPosition = body.position;
        const Vec3 velocityStep = (body.position - body.previousPosition) * damping;
        const Vec3 acceleration = gravity + body.externalForce * body.inverseMass;
        body.position += velocityStep + acceleration * dtSq;
        body.previousPosition = currentPosition;

        // --- Angular: integrate orientation from angular velocity -------------
        // Damp the carried angular velocity, fold in the external torque through
        // the world inverse inertia, then advance and renormalize the quaternion.
        const Mat3 inverseInertiaWorld =
            worldInverseInertia(body.orientation, body.inverseInertiaLocal);
        Vec3 omega = body.angularVelocity * damping + (inverseInertiaWorld * body.externalTorque) * dt;

        body.previousOrientation = body.orientation;
        body.orientation = integrateOrientation(body.orientation, omega, dt);
        body.angularVelocity = omega;

        body.externalForce = {};
        body.externalTorque = {};
    });
}

// Recover linear and angular velocity from the pose delta over the substep,
// mirroring updateParticleVelocities. Angular velocity is extracted from the
// relative rotation previousOrientation -> orientation.
void XPBDWorld::updateRigidBodyVelocities(float dt)
{
    if (!(dt > 0.0f)) {
        return;
    }

    const float invDt = 1.0f / dt;
    forEachRigidBody([invDt](Entity, RigidBody& body) {
        body.velocity = (body.position - body.previousPosition) * invDt;

        // delta = orientation * previousOrientation^-1; its vector part scaled by
        // 2/dt is the angular velocity (small-angle quaternion derivative).
        Quat delta = body.orientation * conjugate(body.previousOrientation);
        if (delta.w < 0.0f) {
            delta = delta * -1.0f;  // shortest arc
        }
        body.angularVelocity = Vec3{delta.x, delta.y, delta.z} * (2.0f * invDt);
    });
}

}  // namespace xpbd
