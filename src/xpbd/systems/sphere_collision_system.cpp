#include "xpbd/xpbd_world.hpp"

#include "linear_bvh.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
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

}  // namespace xpbd
