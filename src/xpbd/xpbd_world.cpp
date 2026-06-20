#include "xpbd/xpbd_world.hpp"

#include "utils/shapes.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace xpbd {

XPBDWorld::XPBDWorld()
    : simd_(&detail::simdDispatch())
{
    broadphases_[partitionIndex(BroadphasePartition::Static)] = utils::LinearBVH{};
    broadphases_[partitionIndex(BroadphasePartition::Dynamic)] = utils::LinearBVH{};
}

// --- Entity creation -------------------------------------------------------

Entity XPBDWorld::createParticle(const Vec3& position, float mass)
{
    Particle particle;
    particle.position = position;
    particle.previousPosition = position;
    particle.velocity = {};
    particle.externalAcceleration = {};
    particle.inverseMass = mass > 0.0f ? 1.0f / mass : 0.0f;
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

Entity XPBDWorld::createCollider(Entity body,
                                 const Shape& shape,
                                 CollisionLayerMask layer,
                                 CollisionLayerMask mask,
                                 BroadphasePartition partition,
                                 const Vec3& offset)
{
    Collider collider;
    collider.body = body;
    collider.shape = shape;
    collider.offset = offset;
    collider.layer = layer;
    collider.mask = mask;
    collider.partition = partition;
    collider.enabled = true;
    collider.proxy = utils::LinearBVH::kInvalid;
    return colliders_.create(EntityType::Collider, collider);
}

Entity XPBDWorld::createStaticCollider(const Shape& shape,
                                       const Vec3& worldPosition,
                                       CollisionLayerMask layer,
                                       CollisionLayerMask mask)
{
    return createCollider(Entity{}, shape, layer, mask, BroadphasePartition::Static, worldPosition);
}

bool XPBDWorld::destroy(Entity entity)
{
    switch (entity.type()) {
    case EntityType::Particle:
        return particles_.destroy(entity);
    case EntityType::DistanceConstraint:
        return distanceConstraints_.destroy(entity);
    case EntityType::Collider: {
        Collider* c = colliders_.get(entity);
        if (c == nullptr) {
            return false;
        }
        removeColliderProxy(*c);
        return colliders_.destroy(entity);
    }
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
    case EntityType::Collider:
        return colliders_.alive(entity);
    default:
        return false;
    }
}

// --- Accessors -------------------------------------------------------------

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

Collider* XPBDWorld::collider(Entity entity)
{
    return entity.type() == EntityType::Collider ? colliders_.get(entity) : nullptr;
}

const Collider* XPBDWorld::collider(Entity entity) const
{
    return entity.type() == EntityType::Collider ? colliders_.get(entity) : nullptr;
}

bool XPBDWorld::colliderWorldSphere(Entity entity, Vec3& center, float& radius) const
{
    const Collider* c = collider(entity);
    if (c == nullptr || !c->enabled || c->shape.type != ShapeType::Sphere) {
        return false;
    }

    Vec3 origin = c->offset;
    if (c->body.valid()) {
        const Particle* body = particle(c->body);
        if (body == nullptr) {
            return false;  // body died; proxy will be reaped next refresh
        }
        origin = body->position + c->offset;
    }

    center = origin + c->shape.sphere.center;
    radius = c->shape.sphere.radius;
    return radius > 0.0f;
}

bool XPBDWorld::setColliderFilter(Entity entity,
                                  CollisionLayerMask layer,
                                  CollisionLayerMask mask)
{
    Collider* c = collider(entity);
    if (c == nullptr) {
        return false;
    }
    c->layer = layer;
    c->mask = mask;
    return true;
}

void XPBDWorld::clearEntities()
{
    particles_.destroyAll();
    distanceConstraints_.destroyAll();
    colliders_.destroyAll();
    for (utils::LinearBVH& bvh : broadphases_) {
        bvh.clear();
    }
    contacts_.clear();
    broadphaseSolveCount_ = 0u;
}

// --- Configuration ---------------------------------------------------------

void XPBDWorld::setGravity(const Vec3& gravity) { gravity_ = gravity; }
const Vec3& XPBDWorld::gravity() const { return gravity_; }

void XPBDWorld::setDamping(float damping)
{
    damping_ = std::max(0.0f, std::min(damping, 1.0f));
}
float XPBDWorld::damping() const { return damping_; }

void XPBDWorld::setSolverIterations(int iterations)
{
    solverIterations_ = std::max(1, iterations);
}
int XPBDWorld::solverIterations() const { return solverIterations_; }

void XPBDWorld::setSubsteps(int substeps)
{
    substeps_ = std::max(1, substeps);
}
int XPBDWorld::substeps() const { return substeps_; }

void XPBDWorld::setCollisionsEnabled(bool enabled) { collisionsEnabled_ = enabled; }
bool XPBDWorld::collisionsEnabled() const { return collisionsEnabled_; }

void XPBDWorld::setContactCompliance(float compliance)
{
    contactCompliance_ = std::max(0.0f, compliance);
}
float XPBDWorld::contactCompliance() const { return contactCompliance_; }

void XPBDWorld::setBroadphaseRebuildInterval(std::size_t interval)
{
    broadphaseRebuildInterval_ = interval;
}
std::size_t XPBDWorld::broadphaseRebuildInterval() const { return broadphaseRebuildInterval_; }

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

// --- Collision pipeline ----------------------------------------------------

utils::LinearBVH& XPBDWorld::broadphase(BroadphasePartition partition)
{
    return broadphases_[partitionIndex(partition)];
}

void XPBDWorld::removeColliderProxy(Collider& collider)
{
    if (collider.proxy != utils::LinearBVH::kInvalid) {
        broadphase(collider.partition).remove(collider.proxy);
        collider.proxy = utils::LinearBVH::kInvalid;
    }
}

// Sync every collider's proxy AABB into its partition's BVH. Each collider owns
// its proxy ObjectId, and the proxy's user data packs the collider Entity, so a
// broadphase pair resolves straight back to the collider with no side maps.
void XPBDWorld::refreshBroadphase()
{
    colliders_.forEachAlive(EntityType::Collider, [this](Entity entity, Collider& collider) {
        Vec3 center;
        float radius = 0.0f;
        const bool active = collider.enabled && colliderWorldSphere(entity, center, radius);
        if (!active) {
            removeColliderProxy(collider);
            return;
        }

        const utils::AABB bounds = utils::AABB::fromCenterRadius(center, radius);
        utils::LinearBVH& bvh = broadphase(collider.partition);
        if (collider.proxy == utils::LinearBVH::kInvalid) {
            collider.proxy = bvh.insertProxy(bounds, entity.value);
        } else if (!bvh.updateProxy(collider.proxy, bounds)) {
            bvh.remove(collider.proxy);
            collider.proxy = bvh.insertProxy(bounds, entity.value);
        }
    });

    ++broadphaseSolveCount_;
    const bool rebuildByCadence =
        broadphaseRebuildInterval_ > 0u &&
        (broadphaseSolveCount_ % broadphaseRebuildInterval_) == 0u;

    broadphaseNodes_ = 0;
    for (utils::LinearBVH& bvh : broadphases_) {
        if (rebuildByCadence && bvh.objectCount() > 0) {
            bvh.rebuild();
        }
        broadphaseNodes_ += bvh.nodeCount();
    }
}

// Narrowphase: turn overlapping proxy pairs into ContactConstraints. Runs once
// per substep; the constraints are then solved (and re-solved) in the iteration
// loop. Static x Static is skipped (neither body moves).
void XPBDWorld::generateContacts(float dt)
{
    contacts_.clear();
    if (!(dt > 0.0f)) {
        return;
    }

    utils::LinearBVH& dynamicBvh = broadphase(BroadphasePartition::Dynamic);
    utils::LinearBVH& staticBvh = broadphase(BroadphasePartition::Static);

    static thread_local std::vector<std::pair<utils::LinearBVH::ObjectId,
                                              utils::LinearBVH::ObjectId>> pairs;

    const auto emitContact = [this](utils::LinearBVH::ObjectId idA,
                                    utils::LinearBVH::ObjectId idB,
                                    const utils::LinearBVH& bvhA,
                                    const utils::LinearBVH& bvhB) {
        const Entity entityA{bvhA.userDataForId(idA)};
        const Entity entityB{bvhB.userDataForId(idB)};
        if (entityA == entityB || !entityA.valid() || !entityB.valid()) {
            return;
        }

        const Collider* colliderA = collider(entityA);
        const Collider* colliderB = collider(entityB);
        if (colliderA == nullptr || colliderB == nullptr) {
            return;
        }
        if (!collisionFiltersMatch(colliderA->layer, colliderA->mask,
                                   colliderB->layer, colliderB->mask)) {
            return;
        }

        Vec3 centerA;
        Vec3 centerB;
        float radiusA = 0.0f;
        float radiusB = 0.0f;
        if (!colliderWorldSphere(entityA, centerA, radiusA) ||
            !colliderWorldSphere(entityB, centerB, radiusB)) {
            return;
        }

        const float minDistance = radiusA + radiusB;
        if (!(minDistance > 0.0f)) {
            return;
        }

        const Vec3 delta = centerB - centerA;
        const float distSq = lengthSq(delta);
        if (distSq >= minDistance * minDistance) {
            return;  // broadphase candidate, but not actually touching
        }

        ContactConstraint contact;
        contact.colliderA = entityA;
        contact.colliderB = entityB;
        contact.bodyA = colliderA->body;
        contact.bodyB = colliderB->body;
        contact.compliance = contactCompliance_;
        contact.lambda = 0.0f;
        if (distSq > 1e-12f) {
            contact.normal = delta / std::sqrt(distSq);
        } else {
            // Coincident centers: deterministic fallback axis from entity ids.
            const std::uint32_t hash = entityA.index() * 73856093u ^ entityB.index() * 19349663u;
            static const Vec3 axes[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                                         {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
            contact.normal = axes[hash % 6u];
        }
        contacts_.push_back(contact);
    };

    // Dynamic x Dynamic (self pairs).
    if (dynamicBvh.objectCount() >= 2) {
        pairs.clear();
        dynamicBvh.overlapPairs(pairs);
        broadphasePairs_ += pairs.size();
        for (const auto& pair : pairs) {
            emitContact(pair.first, pair.second, dynamicBvh, dynamicBvh);
        }
    }

    // Dynamic x Static (cross pairs); Static x Static intentionally skipped.
    if (dynamicBvh.objectCount() > 0 && staticBvh.objectCount() > 0) {
        pairs.clear();
        dynamicBvh.overlapPairs(staticBvh, pairs);
        broadphasePairs_ += pairs.size();
        for (const auto& pair : pairs) {
            emitContact(pair.first, pair.second, dynamicBvh, staticBvh);
        }
    }
}

// XPBD contact solve with per-substep lambda accumulation. Dispatches the
// positional correction onto each referenced body. Today the only body type is
// Particle; a RigidBody branch slots in here (apply at lever arm).
void XPBDWorld::solveContacts(float dt)
{
    const float dtSq = dt * dt;
    if (!(dtSq > 0.0f)) {
        return;
    }

    for (ContactConstraint& contact : contacts_) {
        Particle* a = particle(contact.bodyA);  // null => static/body-less
        Particle* b = particle(contact.bodyB);

        const float wA = a != nullptr ? a->inverseMass : 0.0f;
        const float wB = b != nullptr ? b->inverseMass : 0.0f;
        const float wSum = wA + wB;
        if (wSum <= 0.0f) {
            continue;
        }

        // Re-evaluate penetration along the cached contact normal each iteration.
        Vec3 centerA;
        Vec3 centerB;
        float radiusA = 0.0f;
        float radiusB = 0.0f;
        if (!colliderWorldSphere(contact.colliderA, centerA, radiusA) ||
            !colliderWorldSphere(contact.colliderB, centerB, radiusB)) {
            continue;
        }

        const float minDistance = radiusA + radiusB;
        const float separation = dot(centerB - centerA, contact.normal);
        const float c = separation - minDistance;  // <0 when penetrating
        if (c >= 0.0f) {
            continue;
        }

        const float alpha = contact.compliance / dtSq;
        const float deltaLambda = (-c - alpha * contact.lambda) / (wSum + alpha);
        contact.lambda += deltaLambda;

        const Vec3 correction = contact.normal * deltaLambda;
        if (a != nullptr) {
            a->position -= correction * wA;
        }
        if (b != nullptr) {
            b->position += correction * wB;
        }
    }
}

// --- Step loop -------------------------------------------------------------

void XPBDWorld::step(float dt)
{
    if (!(dt > 0.0f)) {
        return;
    }

    const float subDt = dt / static_cast<float>(substeps_);
    broadphasePairs_ = 0;
    contacts_count_ = 0;
    broadphaseNodes_ = 0;

    for (int substep = 0; substep < substeps_; ++substep) {
        for (System& system : integrationSystems_) {
            system(*this, subDt);
        }

        // Build collision constraints once per substep against integrated
        // positions, then solve them alongside user constraints each iteration.
        if (collisionsEnabled_) {
            refreshBroadphase();
            generateContacts(subDt);
        } else {
            contacts_.clear();
        }

        resetConstraintLambdas();
        for (int iteration = 0; iteration < solverIterations_; ++iteration) {
            for (System& system : constraintSystems_) {
                system(*this, subDt);
            }
            if (collisionsEnabled_) {
                solveContacts(subDt);
            }
        }
        contacts_count_ += contacts_.size();

        updateParticleVelocities(subDt);
    }
}

// --- Statistics ------------------------------------------------------------

std::size_t XPBDWorld::particleSlots() const { return particles_.slots(); }
std::size_t XPBDWorld::particleCount() const { return particles_.aliveCount(); }
std::size_t XPBDWorld::distanceConstraintSlots() const { return distanceConstraints_.slots(); }
std::size_t XPBDWorld::distanceConstraintCount() const { return distanceConstraints_.aliveCount(); }
std::size_t XPBDWorld::colliderSlots() const { return colliders_.slots(); }
std::size_t XPBDWorld::colliderCount() const { return colliders_.aliveCount(); }
std::size_t XPBDWorld::broadphasePairCount() const { return broadphasePairs_; }
std::size_t XPBDWorld::contactCount() const { return contacts_count_; }
std::size_t XPBDWorld::broadphaseNodeCount() const { return broadphaseNodes_; }

const char* XPBDWorld::simdBackendName() const
{
    return simd_ != nullptr ? simd_->name : "scalar";
}

void XPBDWorld::resetConstraintLambdas()
{
    forEachDistanceConstraint([](Entity, DistanceConstraint& constraint) {
        constraint.lambda = 0.0f;
    });
    for (ContactConstraint& contact : contacts_) {
        contact.lambda = 0.0f;
    }
}

void XPBDWorld::updateParticleVelocities(float dt)
{
    if (!(dt > 0.0f) || simd_ == nullptr || simd_->updateParticleVelocities == nullptr) {
        return;
    }

    simd_->updateParticleVelocities(particles_.data(), particles_.aliveData(), particles_.slots(), dt);
}

}  // namespace xpbd


