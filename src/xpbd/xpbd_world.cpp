#include "xpbd/xpbd_world.hpp"

#include "utils/shapes.hpp"
#include "xpbd/narrowphase/narrowphase.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

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

Entity XPBDWorld::createRigidBody(const Vec3& position,
                                  const Quat& orientation,
                                  float mass,
                                  const Mat3& inverseInertiaLocal,
                                  EntityType type)
{
    RigidBody body;
    body.position = position;
    body.previousPosition = position;
    body.velocity = {};
    body.orientation = normalized(orientation);
    body.previousOrientation = body.orientation;
    body.angularVelocity = {};
    body.externalForce = {};
    body.externalTorque = {};
    body.inverseMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    body.inverseInertiaLocal = mass > 0.0f ? inverseInertiaLocal : Mat3::zero();
    return rigidBodies_.create(type, body);
}

Entity XPBDWorld::createTetrahedronRigidBody(const Vec3 vertices[4],
                                             float mass,
                                             float vertexRadius,
                                             CollisionLayerMask layer,
                                             CollisionLayerMask mask,
                                             std::vector<Entity>* outColliders)
{
    const Vec3 centroid = (vertices[0] + vertices[1] + vertices[2] + vertices[3]) * 0.25f;

    // Solid-tetrahedron inertia about the centroid, approximated as the inertia
    // of four point masses (mass/4) at the vertices. This is a reasonable proxy
    // for a tet built from vertex spheres and stays positive-definite for any
    // non-degenerate tet. I = sum m_i (|r|^2 I3 - r r^T).
    Mat3 inertia = Mat3::zero();
    const float pointMass = mass > 0.0f ? mass * 0.25f : 0.0f;
    for (int i = 0; i < 4; ++i) {
        const Vec3 r = vertices[i] - centroid;
        const float rSq = dot(r, r);
        inertia.rows[0] += Vec3{rSq - r.x * r.x, -r.x * r.y, -r.x * r.z} * pointMass;
        inertia.rows[1] += Vec3{-r.y * r.x, rSq - r.y * r.y, -r.y * r.z} * pointMass;
        inertia.rows[2] += Vec3{-r.z * r.x, -r.z * r.y, rSq - r.z * r.z} * pointMass;
    }

    const Mat3 inverseInertia = mass > 0.0f ? inverse(inertia) : Mat3::zero();
    const Entity body = createRigidBody(centroid, Quat::identity(), mass, inverseInertia,
                                        EntityType::TetrahedronRigidBody);

    for (int i = 0; i < 4; ++i) {
        const Entity collider = createCollider(
            body,
            Shape::makeSphere(vertexRadius),
            layer,
            mask,
            BroadphasePartition::Dynamic,
            vertices[i] - centroid);  // local offset; rotates with the body
        if (outColliders != nullptr) {
            outColliders->push_back(collider);
        }
    }
    return body;
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
    case EntityType::SphereRigidBody:
    case EntityType::BoxRigidBody:
    case EntityType::TetrahedronRigidBody:
    case EntityType::CapsuleRigidBody:
        return rigidBodies_.destroy(entity);
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
    case EntityType::SphereRigidBody:
    case EntityType::BoxRigidBody:
    case EntityType::TetrahedronRigidBody:
    case EntityType::CapsuleRigidBody:
        return rigidBodies_.alive(entity);
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

bool XPBDWorld::isRigidBody(Entity entity)
{
    switch (entity.type()) {
    case EntityType::SphereRigidBody:
    case EntityType::BoxRigidBody:
    case EntityType::TetrahedronRigidBody:
    case EntityType::CapsuleRigidBody:
        return true;
    default:
        return false;
    }
}

RigidBody* XPBDWorld::rigidBody(Entity entity)
{
    return isRigidBody(entity) ? rigidBodies_.get(entity) : nullptr;
}

const RigidBody* XPBDWorld::rigidBody(Entity entity) const
{
    return isRigidBody(entity) ? rigidBodies_.get(entity) : nullptr;
}

bool XPBDWorld::resolveBodyTransform(Entity body, BodyTransform& out) const
{
    if (const Particle* p = particle(body)) {
        out.position = p->position;
        out.orientation = Quat::identity();
        return true;
    }
    if (const RigidBody* rb = rigidBody(body)) {
        out.position = rb->position;
        out.orientation = rb->orientation;
        return true;
    }
    return false;
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
    rigidBodies_.destroyAll();
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
    std::vector<Entity> collidersToDestroy;

    colliders_.forEachAlive(EntityType::Collider,
                            [this, &collidersToDestroy](Entity entity, Collider& collider) {
        const std::optional<WorldSphere> sphere = worldShape<ShapeType::Sphere>(collider);
        if (!sphere) {
            removeColliderProxy(collider);
            // mark destroy when the collider's body is gone (resolveBodyTransform
            // fails for a dead Particle or RigidBody handle).
            BodyTransform xf;
            if (collider.body.valid() && !resolveBodyTransform(collider.body, xf)) {
                collidersToDestroy.push_back(entity);
            }
            return;
        }

        const utils::AABB bounds = utils::AABB::fromCenterRadius(sphere->center, sphere->radius);
        utils::LinearBVH& bvh = broadphase(collider.partition);
        if (collider.proxy == utils::LinearBVH::kInvalid) {
            collider.proxy = bvh.insertProxy(bounds, entity.value);
        } else if (!bvh.updateProxy(collider.proxy, bounds)) {
            bvh.remove(collider.proxy);
            collider.proxy = bvh.insertProxy(bounds, entity.value);
        }
    });

    for (Entity entity : collidersToDestroy) {
        destroy(entity);
    }

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
        // same body multiple colliders shouldn't self collide
        if (colliderA->body.valid() && colliderA->body == colliderB->body) {
            return;
        }
        if (!collisionFiltersMatch(colliderA->layer, colliderA->mask,
                                   colliderB->layer, colliderB->mask)) {
            return;
        }

        const std::optional<WorldSphere> sphereA = worldShape<ShapeType::Sphere>(*colliderA);
        const std::optional<WorldSphere> sphereB = worldShape<ShapeType::Sphere>(*colliderB);
        if (!sphereA || !sphereB) {
            return;
        }

        // Coincident centers have no defined separating axis; pick a
        // deterministic one from the entity ids so the contact is stable.
        const std::uint32_t hash = entityA.index() * 73856093u ^ entityB.index() * 19349663u;
        static const Vec3 kFallbackAxes[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                                              {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
        const Contact hit =
            narrowphase::sphereSphere(*sphereA, *sphereB, kFallbackAxes[hash % 6u]);
        if (!hit.touching) {
            return;  // broadphase candidate, but not actually touching
        }

        ContactConstraint contact;
        contact.colliderA = entityA;
        contact.colliderB = entityB;
        contact.bodyA = colliderA->body;
        contact.bodyB = colliderB->body;
        contact.compliance = contactCompliance_;
        contact.lambda = 0.0f;
        contact.normal = hit.normal;
        contact.point = hit.point;
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
// positional correction onto each referenced body: a Particle is a point mass
// (correction = normal * dLambda * w); a RigidBody also rotates, using the lever
// arm from its center of mass to the contact point (DESIGN.md 4.2).
void XPBDWorld::solveContacts(float dt)
{
    const float dtSq = dt * dt;
    if (!(dtSq > 0.0f)) {
        return;
    }

    // Per-body state needed to apply a correction: inverse mass, world inverse
    // inertia, and the lever arm r (contact point - center of mass). A particle
    // (or static body) leaves inertia zero and r unused.
    struct BodyResponse {
        Particle* particle = nullptr;
        RigidBody* rigid = nullptr;
        float inverseMass = 0.0f;
        Mat3 inverseInertiaWorld = Mat3::zero();
        Vec3 r{};  // lever arm from COM to contact point
    };

    const auto resolveResponse = [this](Entity body, const Vec3& contactPoint) {
        BodyResponse response;
        if (Particle* p = particle(body)) {
            response.particle = p;
            response.inverseMass = p->inverseMass;
        } else if (RigidBody* rb = rigidBody(body)) {
            response.rigid = rb;
            response.inverseMass = rb->inverseMass;
            response.inverseInertiaWorld =
                worldInverseInertia(rb->orientation, rb->inverseInertiaLocal);
            response.r = contactPoint - rb->position;
        }
        return response;
    };

    // Generalized inverse mass of a body along the contact normal.
    const auto effectiveMass = [](const BodyResponse& body, const Vec3& normal) {
        if (body.rigid != nullptr) {
            return generalizedInverseMass(body.inverseMass, body.inverseInertiaWorld,
                                          body.r, normal);
        }
        return body.inverseMass;  // particle / static
    };

    // Apply a positional impulse to a body: translate by impulse * invMass and,
    // for a rigid body, rotate by the angular response about the lever arm.
    const auto applyImpulse = [](BodyResponse& body, const Vec3& impulse) {
        if (body.particle != nullptr) {
            body.particle->position += impulse * body.inverseMass;
        } else if (body.rigid != nullptr) {
            body.rigid->position += impulse * body.inverseMass;
            const Vec3 angular = body.inverseInertiaWorld * cross(body.r, impulse);
            body.rigid->orientation =
                normalized(Quat{angular.x, angular.y, angular.z, 0.0f} *
                               body.rigid->orientation * 0.5f +
                           body.rigid->orientation);
        }
    };

    for (ContactConstraint& contact : contacts_) {
        // Re-evaluate the geometry along the cached normal each iteration.
        const Collider* colA = collider(contact.colliderA);
        const Collider* colB = collider(contact.colliderB);
        if (colA == nullptr || colB == nullptr) {
            continue;
        }

        const std::optional<WorldSphere> sphereA = worldShape<ShapeType::Sphere>(*colA);
        const std::optional<WorldSphere> sphereB = worldShape<ShapeType::Sphere>(*colB);
        if (!sphereA || !sphereB) {
            continue;
        }

        const float minDistance = sphereA->radius + sphereB->radius;
        const float separation = dot(sphereB->center - sphereA->center, contact.normal);
        const float c = separation - minDistance;  // <0 when penetrating
        if (c >= 0.0f) {
            continue;
        }

        // Contact point on the overlap midline, updated for the current pose so
        // the rigid-body lever arm tracks the bodies as they move.
        const Vec3 contactPoint =
            sphereA->center + contact.normal * (sphereA->radius + c * 0.5f);

        BodyResponse a = resolveResponse(contact.bodyA, contactPoint);
        BodyResponse b = resolveResponse(contact.bodyB, contactPoint);

        const float wA = effectiveMass(a, contact.normal);
        const float wB = effectiveMass(b, contact.normal);
        const float wSum = wA + wB;
        if (wSum <= 0.0f) {
            continue;
        }

        const float alpha = contact.compliance / dtSq;
        const float deltaLambda = (-c - alpha * contact.lambda) / (wSum + alpha);
        contact.lambda += deltaLambda;

        const Vec3 impulse = contact.normal * deltaLambda;
        applyImpulse(a, -impulse);
        applyImpulse(b, impulse);
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
        updateRigidBodyVelocities(subDt);
    }
}

// --- Statistics ------------------------------------------------------------

std::size_t XPBDWorld::particleSlots() const { return particles_.slots(); }
std::size_t XPBDWorld::particleCount() const { return particles_.aliveCount(); }
std::size_t XPBDWorld::rigidBodySlots() const { return rigidBodies_.slots(); }
std::size_t XPBDWorld::rigidBodyCount() const { return rigidBodies_.aliveCount(); }
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
