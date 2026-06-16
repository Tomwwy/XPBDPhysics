#include "xpbd/xpbd_world.hpp"

#include <algorithm>

namespace xpbd {

XPBDWorld::XPBDWorld()
    : simd_(&detail::simdDispatch())
{
}

Entity XPBDWorld::createParticle(const Vec3& position, float mass, float radius)
{
    Particle particle;
    particle.position = position;
    particle.previousPosition = position;
    particle.velocity = {};
    particle.externalAcceleration = {};
    particle.inverseMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    particle.radius = radius;
    return particles_.create(EntityType::Particle, particle);
}

Entity XPBDWorld::createDistanceConstraint(Entity particleA,
                                           Entity particleB,
                                           float restLength,
                                           float compliance)
{
    DistanceConstraint constraint;
    constraint.particleA = particleA;
    constraint.particleB = particleB;
    constraint.restLength = std::max(0.0f, restLength);
    constraint.compliance = std::max(0.0f, compliance);
    return distanceConstraints_.create(EntityType::DistanceConstraint, constraint);
}

bool XPBDWorld::destroy(Entity entity)
{
    switch (entity.type()) {
    case EntityType::Particle:
        return particles_.destroy(entity);
    case EntityType::DistanceConstraint:
        return distanceConstraints_.destroy(entity);
    default:
        return false;
    }
}

bool XPBDWorld::alive(Entity entity) const
{
    switch (entity.type()) {
    case EntityType::Particle:
        return particles_.alive(entity);
    case EntityType::DistanceConstraint:
        return distanceConstraints_.alive(entity);
    default:
        return false;
    }
}

Particle* XPBDWorld::particle(Entity entity)
{
    return entity.type() == EntityType::Particle ? particles_.get(entity) : nullptr;
}

const Particle* XPBDWorld::particle(Entity entity) const
{
    return entity.type() == EntityType::Particle ? particles_.get(entity) : nullptr;
}

DistanceConstraint* XPBDWorld::distanceConstraint(Entity entity)
{
    return entity.type() == EntityType::DistanceConstraint ? distanceConstraints_.get(entity) : nullptr;
}

const DistanceConstraint* XPBDWorld::distanceConstraint(Entity entity) const
{
    return entity.type() == EntityType::DistanceConstraint ? distanceConstraints_.get(entity) : nullptr;
}

void XPBDWorld::clearEntities()
{
    particles_.destroyAll();
    distanceConstraints_.destroyAll();
}

void XPBDWorld::setGravity(const Vec3& gravity)
{
    gravity_ = gravity;
}

const Vec3& XPBDWorld::gravity() const
{
    return gravity_;
}

void XPBDWorld::setDamping(float damping)
{
    damping_ = std::max(0.0f, std::min(damping, 1.0f));
}

float XPBDWorld::damping() const
{
    return damping_;
}

void XPBDWorld::setSolverIterations(int iterations)
{
    solverIterations_ = std::max(1, iterations);
}

int XPBDWorld::solverIterations() const
{
    return solverIterations_;
}

void XPBDWorld::setSubsteps(int substeps)
{
    substeps_ = std::max(1, substeps);
}

int XPBDWorld::substeps() const
{
    return substeps_;
}

void XPBDWorld::addIntegrationSystem(System system)
{
    integrationSystems_.push_back(std::move(system));
}

void XPBDWorld::addConstraintSystem(System system)
{
    constraintSystems_.push_back(std::move(system));
}

void XPBDWorld::clearSystems()
{
    integrationSystems_.clear();
    constraintSystems_.clear();
}

void XPBDWorld::step(float dt)
{
    if (!(dt > 0.0f)) {
        return;
    }

    const float subDt = dt / static_cast<float>(substeps_);
    for (int substep = 0; substep < substeps_; ++substep) {
        for (System& system : integrationSystems_) {
            system(*this, subDt);
        }

        resetConstraintLambdas();
        for (int iteration = 0; iteration < solverIterations_; ++iteration) {
            for (System& system : constraintSystems_) {
                system(*this, subDt);
            }
        }

        updateParticleVelocities(subDt);
    }
}

std::size_t XPBDWorld::particleSlots() const
{
    return particles_.slots();
}

std::size_t XPBDWorld::particleCount() const
{
    return particles_.aliveCount();
}

std::size_t XPBDWorld::distanceConstraintSlots() const
{
    return distanceConstraints_.slots();
}

std::size_t XPBDWorld::distanceConstraintCount() const
{
    return distanceConstraints_.aliveCount();
}

const char* XPBDWorld::simdBackendName() const
{
    return simd_ != nullptr ? simd_->name : "scalar";
}

void XPBDWorld::verletIntegrationSystem(XPBDWorld& world, float dt)
{
    if (world.simd_ == nullptr || world.simd_->integrateParticles == nullptr) {
        return;
    }

    world.simd_->integrateParticles(world.particles_.data(),
                                    world.particles_.aliveData(),
                                    world.particles_.slots(),
                                    world.gravity_,
                                    world.damping_,
                                    dt);
}

void XPBDWorld::distanceConstraintSystem(XPBDWorld& world, float dt)
{
    const float dtSq = dt * dt;
    if (!(dtSq > 0.0f)) {
        return;
    }

    world.forEachDistanceConstraint([&world, dtSq](Entity, DistanceConstraint& constraint) {
        if (!constraint.enabled) {
            return;
        }

        Particle* particleA = world.particle(constraint.particleA);
        Particle* particleB = world.particle(constraint.particleB);
        if (particleA == nullptr || particleB == nullptr) {
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
}

void XPBDWorld::resetConstraintLambdas()
{
    forEachDistanceConstraint([](Entity, DistanceConstraint& constraint) {
        constraint.lambda = 0.0f;
    });
}

void XPBDWorld::updateParticleVelocities(float dt)
{
    if (!(dt > 0.0f) || simd_ == nullptr || simd_->updateParticleVelocities == nullptr) {
        return;
    }

    simd_->updateParticleVelocities(particles_.data(), particles_.aliveData(), particles_.slots(), dt);
}

}  // namespace xpbd
