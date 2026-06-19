#include "xpbd/xpbd_world.hpp"

#include <algorithm>
#include <utility>

namespace xpbd {

XPBDWorld::XPBDWorld()
    : simd_(&detail::simdDispatch())
{
}

Entity XPBDWorld::createParticle(const Vec3& position,
                                 float mass,
                                 float radius,
                                 CollisionLayerMask collisionLayer,
                                 CollisionLayerMask collisionMask)
{
    Particle particle;
    particle.position = position;
    particle.previousPosition = position;
    particle.velocity = {};
    particle.externalAcceleration = {};
    particle.inverseMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    particle.radius = radius;
    particle.collisionLayer = collisionLayer;
    particle.collisionMask = collisionMask;
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

Entity XPBDWorld::createCollisionSphere(const Vec3& center,
                                        float radius,
                                        CollisionLayerMask collisionLayer,
                                        CollisionLayerMask collisionMask)
{
    CollisionSphere sphere;
    sphere.center = center;
    sphere.radius = std::max(0.0f, radius);
    sphere.collisionLayer = collisionLayer;
    sphere.collisionMask = collisionMask;
    return collisionSpheres_.create(EntityType::SphereRigidBody, sphere);
}

bool XPBDWorld::destroy(Entity entity)
{
    switch (entity.type()) {
    case EntityType::Particle:
        return particles_.destroy(entity);
    case EntityType::DistanceConstraint:
        return distanceConstraints_.destroy(entity);
    case EntityType::SphereRigidBody:
        return collisionSpheres_.destroy(entity);
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
    case EntityType::SphereRigidBody:
        return collisionSpheres_.alive(entity);
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

CollisionSphere* XPBDWorld::collisionSphere(Entity entity)
{
    return entity.type() == EntityType::SphereRigidBody ? collisionSpheres_.get(entity) : nullptr;
}

const CollisionSphere* XPBDWorld::collisionSphere(Entity entity) const
{
    return entity.type() == EntityType::SphereRigidBody ? collisionSpheres_.get(entity) : nullptr;
}

bool XPBDWorld::setParticleCollisionFilter(Entity entity,
                                           CollisionLayerMask collisionLayer,
                                           CollisionLayerMask collisionMask)
{
    Particle* particle = this->particle(entity);
    if (particle == nullptr) {
        return false;
    }

    particle->collisionLayer = collisionLayer;
    particle->collisionMask = collisionMask;
    return true;
}

bool XPBDWorld::setCollisionSphereFilter(Entity entity,
                                         CollisionLayerMask collisionLayer,
                                         CollisionLayerMask collisionMask)
{
    CollisionSphere* sphere = collisionSphere(entity);
    if (sphere == nullptr) {
        return false;
    }

    sphere->collisionLayer = collisionLayer;
    sphere->collisionMask = collisionMask;
    return true;
}

void XPBDWorld::clearEntities()
{
    particles_.destroyAll();
    distanceConstraints_.destroyAll();
    collisionSpheres_.destroyAll();
    for (detail::SphereCollisionLayerTree& layer : sphereCollisionLayers_) {
        layer.broadphase.clear();
        layer.refsByObject.clear();
        layer.objectByEntity.clear();
        layer.aggregateCollisionMask = 0u;
        layer.active = false;
    }
    sphereCollisionBroadphaseStamp_ = 0u;
    sphereCollisionBroadphaseSolveCount_ = 0u;
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

void XPBDWorld::setSphereCollisionsEnabled(bool enabled)
{
    sphereCollisionsEnabled_ = enabled;
}

bool XPBDWorld::sphereCollisionsEnabled() const
{
    return sphereCollisionsEnabled_;
}

void XPBDWorld::setSphereCollisionCompliance(float compliance)
{
    sphereCollisionCompliance_ = std::max(0.0f, compliance);
}

float XPBDWorld::sphereCollisionCompliance() const
{
    return sphereCollisionCompliance_;
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
    sphereCollisionBroadphasePairs_ = 0;
    sphereCollisionContacts_ = 0;
    sphereCollisionBvhNodes_ = 0;

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

std::size_t XPBDWorld::collisionSphereSlots() const
{
    return collisionSpheres_.slots();
}

std::size_t XPBDWorld::collisionSphereCount() const
{
    return collisionSpheres_.aliveCount();
}

std::size_t XPBDWorld::sphereCollisionBroadphasePairCount() const
{
    return sphereCollisionBroadphasePairs_;
}

std::size_t XPBDWorld::sphereCollisionContactCount() const
{
    return sphereCollisionContacts_;
}

std::size_t XPBDWorld::sphereCollisionBvhNodeCount() const
{
    return sphereCollisionBvhNodes_;
}

std::size_t XPBDWorld::sphereCollisionBvhRebuildInterval() const
{
    return sphereCollisionBvhRebuildInterval_;
}

void XPBDWorld::setSphereCollisionBvhRebuildInterval(std::size_t interval)
{
    sphereCollisionBvhRebuildInterval_ = interval;
}

const char* XPBDWorld::simdBackendName() const
{
    return simd_ != nullptr ? simd_->name : "scalar";
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
