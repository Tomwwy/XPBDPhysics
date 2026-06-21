#include "xpbd/xpbd_world.hpp"

#include <vector>

namespace xpbd {

void XPBDWorld::distanceConstraintSystem(XPBDWorld& world, float dt)
{
    const float dtSq = dt * dt;
    if (!(dtSq > 0.0f)) {
        return;
    }

    std::vector<Entity> constraintsToDestroy;

    world.forEachDistanceConstraint([&world, dtSq, &constraintsToDestroy](Entity entity,
                                                                          DistanceConstraint& constraint) {
        Particle* particleA = world.particle(constraint.particleA);
        Particle* particleB = world.particle(constraint.particleB);
        if (particleA == nullptr || particleB == nullptr) {
            // one or both particles are gone, constraint not useful anymore, mark destroy
            constraintsToDestroy.push_back(entity);
            return;
        }
        if (!constraint.enabled) {
            return;
        }

        const float wA = particleA->inverseMass;
        const float wB = particleB->inverseMass;
        const float wSum = wA + wB;
        if (wSum <= 0.0f) {
            return;
        }

        const Vec3 delta = particleB->position - particleA->position;
        const float dist = length(delta);
        if (dist <= 1e-7f) {
            return;
        }

        const Vec3 direction = delta / dist;
        const float c = dist - constraint.restLength;
        const float alpha = constraint.compliance / dtSq;
        const float deltaLambda = (-c - alpha * constraint.lambda) / (wSum + alpha);
        constraint.lambda += deltaLambda;

        const Vec3 correction = direction * deltaLambda;
        particleA->position -= correction * wA;
        particleB->position += correction * wB;
    });

    for (Entity entity : constraintsToDestroy) {
        world.destroy(entity);
    }
}

}  // namespace xpbd
