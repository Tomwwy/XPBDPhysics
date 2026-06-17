#include "xpbd/xpbd_world.hpp"

#include "linear_bvh.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace xpbd {
namespace {

struct SphereCollisionRef {
    Entity entity;
    bool particle = true;
};

struct SphereCollisionState {
    Vec3 position;
    float radius = 0.0f;
    float inverseMass = 0.0f;
    Particle* particle = nullptr;
    bool valid = false;
};

Vec3 fallbackCollisionNormal(Entity a, Entity b)
{
    std::uint32_t hash = a.index() * 73856093u ^ b.index() * 19349663u;
    switch (hash % 6u) {
    case 0u:
        return {1.0f, 0.0f, 0.0f};
    case 1u:
        return {-1.0f, 0.0f, 0.0f};
    case 2u:
        return {0.0f, 1.0f, 0.0f};
    case 3u:
        return {0.0f, -1.0f, 0.0f};
    case 4u:
        return {0.0f, 0.0f, 1.0f};
    default:
        return {0.0f, 0.0f, -1.0f};
    }
}

SphereCollisionState sphereCollisionState(XPBDWorld& world, const SphereCollisionRef& ref)
{
    if (ref.particle) {
        Particle* particle = world.particle(ref.entity);
        if (particle == nullptr || !(particle->radius > 0.0f)) {
            return {};
        }

        SphereCollisionState state;
        state.position = particle->position;
        state.radius = particle->radius;
        state.inverseMass = particle->inverseMass;
        state.particle = particle;
        state.valid = true;
        return state;
    }

    const CollisionSphere* sphere = world.collisionSphere(ref.entity);
    if (sphere == nullptr || !sphere->enabled || !(sphere->radius > 0.0f)) {
        return {};
    }

    SphereCollisionState state;
    state.position = sphere->center;
    state.radius = sphere->radius;
    state.valid = true;
    return state;
}

}  // namespace

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

Entity XPBDWorld::createCollisionSphere(const Vec3& center, float radius)
{
    CollisionSphere sphere;
    sphere.center = center;
    sphere.radius = std::max(0.0f, radius);
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

void XPBDWorld::clearEntities()
{
    particles_.destroyAll();
    distanceConstraints_.destroyAll();
    collisionSpheres_.destroyAll();
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

void XPBDWorld::sphereCollisionSystem(XPBDWorld& world, float dt)
{
    if (!world.sphereCollisionsEnabled_ || !(dt > 0.0f)) {
        return;
    }

    utils::LinearBVH broadphase;
    std::vector<SphereCollisionRef> refsByObject;
    refsByObject.push_back({});

    const auto addRef = [&broadphase, &refsByObject](Entity entity,
                                                     bool particle,
                                                     const Vec3& center,
                                                     float radius) {
        const utils::LinearBVH::ObjectId id = broadphase.insertSphere(center, radius);
        if (id == utils::LinearBVH::kInvalid) {
            return;
        }

        if (refsByObject.size() <= id) {
            refsByObject.resize(static_cast<std::size_t>(id) + 1u);
        }
        refsByObject[static_cast<std::size_t>(id)] = {entity, particle};
    };

    world.forEachParticle([&addRef](Entity entity, const Particle& particle) {
        if (particle.radius > 0.0f) {
            addRef(entity, true, particle.position, particle.radius);
        }
    });

    world.forEachCollisionSphere([&addRef](Entity entity, const CollisionSphere& sphere) {
        if (sphere.enabled && sphere.radius > 0.0f) {
            addRef(entity, false, sphere.center, sphere.radius);
        }
    });

    if (broadphase.objectCount() < 2) {
        return;
    }

    std::vector<std::pair<utils::LinearBVH::ObjectId, utils::LinearBVH::ObjectId>> pairs;
    broadphase.overlapPairs(pairs);
    world.sphereCollisionBroadphasePairs_ += pairs.size();
    world.sphereCollisionBvhNodes_ = broadphase.nodeCount();

    const float alpha = world.sphereCollisionCompliance_ / (dt * dt);
    std::size_t contactCount = 0;

    for (const auto& pair : pairs) {
        if (pair.first >= refsByObject.size() || pair.second >= refsByObject.size()) {
            continue;
        }

        const SphereCollisionRef& refA = refsByObject[static_cast<std::size_t>(pair.first)];
        const SphereCollisionRef& refB = refsByObject[static_cast<std::size_t>(pair.second)];
        if (refA.entity == refB.entity) {
            continue;
        }

        SphereCollisionState stateA = sphereCollisionState(world, refA);
        SphereCollisionState stateB = sphereCollisionState(world, refB);
        if (!stateA.valid || !stateB.valid) {
            continue;
        }

        const float minDistance = stateA.radius + stateB.radius;
        if (!(minDistance > 0.0f)) {
            continue;
        }

        const Vec3 delta = stateB.position - stateA.position;
        const float distSq = lengthSq(delta);
        if (distSq >= minDistance * minDistance) {
            continue;
        }

        const float wA = stateA.inverseMass;
        const float wB = stateB.inverseMass;
        const float wSum = wA + wB;
        if (wSum <= 0.0f) {
            continue;
        }

        float dist = 0.0f;
        Vec3 normal = fallbackCollisionNormal(refA.entity, refB.entity);
        if (distSq > 1e-12f) {
            dist = std::sqrt(distSq);
            normal = delta / dist;
        }

        const float penetration = minDistance - dist;
        const float deltaLambda = penetration / (wSum + alpha);
        const Vec3 correction = normal * deltaLambda;

        if (stateA.particle != nullptr) {
            stateA.particle->position -= correction * wA;
        }
        if (stateB.particle != nullptr) {
            stateB.particle->position += correction * wB;
        }
        ++contactCount;
    }

    world.sphereCollisionContacts_ += contactCount;
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
