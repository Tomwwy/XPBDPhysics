#include "xpbd/xpbd_world.hpp"

#include "linear_bvh.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>

namespace xpbd {
namespace {

constexpr std::size_t kSphereCollisionLayerCount = static_cast<std::size_t>(kCollisionLayerBitCount);

struct SphereCollisionState {
    Vec3 position;
    float radius = 0.0f;
    float inverseMass = 0.0f;
    Particle* particle = nullptr;
    bool valid = false;
};

struct EntityPairKey {
    std::uint64_t first = 0u;
    std::uint64_t second = 0u;

    bool operator==(const EntityPairKey& rhs) const
    {
        return first == rhs.first && second == rhs.second;
    }
};

struct EntityPairKeyHash {
    std::size_t operator()(const EntityPairKey& key) const
    {
        const std::uint64_t mixed = key.first ^ (key.second + UINT64_C(0x9e3779b97f4a7c15) +
                                                 (key.first << 6u) + (key.first >> 2u));
        return static_cast<std::size_t>(mixed ^ (mixed >> 32u));
    }
};

template <typename Func>
void forEachCollisionLayerBit(CollisionLayerMask layers, Func&& func)
{
    for (std::size_t bitIndex = 0; bitIndex < kSphereCollisionLayerCount; ++bitIndex) {
        const CollisionLayerMask bit = CollisionLayerMask{1u} << bitIndex;
        if ((layers & bit) != 0u) {
            func(bitIndex, bit);
        }
    }
}

bool makeEntityPairKey(Entity a, Entity b, EntityPairKey& key)
{
    if (a == b) {
        return false;
    }

    key.first = a.value;
    key.second = b.value;
    if (key.first > key.second) {
        std::swap(key.first, key.second);
    }
    return true;
}

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

SphereCollisionState sphereCollisionState(XPBDWorld& world,
                                          const detail::SphereCollisionBroadphaseRef& ref)
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

void resetLayerSyncState(detail::SphereCollisionLayerTree& layer, CollisionLayerMask bit)
{
    layer.bit = bit;
    layer.aggregateCollisionMask = 0u;
    layer.active = false;
}

void removeStaleRefs(detail::SphereCollisionLayerTree& layer, std::uint64_t stamp)
{
    for (std::size_t objectIndex = 1u; objectIndex < layer.refsByObject.size(); ++objectIndex) {
        detail::SphereCollisionBroadphaseRef& ref = layer.refsByObject[objectIndex];
        if (!ref.registered || ref.lastSeenStamp == stamp) {
            continue;
        }

        layer.objectByEntity.erase(ref.entity.value);
        layer.broadphase.remove(static_cast<utils::LinearBVH::ObjectId>(objectIndex));
        ref = {};
    }

    if (layer.broadphase.objectCount() == 0u) {
        layer.broadphase.clear();
        layer.refsByObject = {{}};
        layer.objectByEntity.clear();
        layer.aggregateCollisionMask = 0u;
        layer.active = false;
    }
}

}  // namespace

void XPBDWorld::sphereCollisionSystem(XPBDWorld& world, float dt)
{
    if (!world.sphereCollisionsEnabled_ || !(dt > 0.0f)) {
        return;
    }

    std::array<detail::SphereCollisionLayerTree, kSphereCollisionLayerCount>& layers =
        world.sphereCollisionLayers_;

    ++world.sphereCollisionBroadphaseStamp_;
    if (world.sphereCollisionBroadphaseStamp_ == 0u) {
        for (detail::SphereCollisionLayerTree& layer : layers) {
            for (detail::SphereCollisionBroadphaseRef& ref : layer.refsByObject) {
                ref.lastSeenStamp = 0u;
            }
        }
        world.sphereCollisionBroadphaseStamp_ = 1u;
    }
    const std::uint64_t stamp = world.sphereCollisionBroadphaseStamp_;

    for (std::size_t bitIndex = 0; bitIndex < layers.size(); ++bitIndex) {
        const CollisionLayerMask bit = CollisionLayerMask{1u} << bitIndex;
        resetLayerSyncState(layers[bitIndex], bit);
    }

    const auto addOrUpdateRef = [&layers, stamp](Entity entity,
                                                 bool particle,
                                                 const Vec3& center,
                                                 float radius,
                                                 CollisionLayerMask collisionLayer,
                                                 CollisionLayerMask collisionMask) {
        if (collisionLayer == 0u || collisionMask == 0u) {
            return;
        }

        forEachCollisionLayerBit(collisionLayer, [&](std::size_t bitIndex, CollisionLayerMask bit) {
            detail::SphereCollisionLayerTree& layer = layers[bitIndex];
            utils::LinearBVH::ObjectId id = utils::LinearBVH::kInvalid;
            auto objectIt = layer.objectByEntity.find(entity.value);
            if (objectIt != layer.objectByEntity.end()) {
                id = objectIt->second;
                if (!layer.broadphase.updateSphere(id, center, radius)) {
                    layer.broadphase.remove(id);
                    layer.objectByEntity.erase(objectIt);
                    if (static_cast<std::size_t>(id) < layer.refsByObject.size()) {
                        layer.refsByObject[static_cast<std::size_t>(id)] = {};
                    }
                    id = utils::LinearBVH::kInvalid;
                }
            }

            if (id == utils::LinearBVH::kInvalid) {
                id = layer.broadphase.insertSphere(center, radius);
                if (id != utils::LinearBVH::kInvalid) {
                    layer.objectByEntity.emplace(entity.value, id);
                }
            }

            if (id == utils::LinearBVH::kInvalid) {
                return;
            }

            if (layer.refsByObject.size() <= id) {
                layer.refsByObject.resize(static_cast<std::size_t>(id) + 1u);
            }

            detail::SphereCollisionBroadphaseRef& ref = layer.refsByObject[static_cast<std::size_t>(id)];
            ref.entity = entity;
            ref.particle = particle;
            ref.collisionLayer = collisionLayer;
            ref.collisionMask = collisionMask;
            ref.lastSeenStamp = stamp;
            ref.registered = true;
            layer.bit = bit;
            layer.aggregateCollisionMask |= collisionMask;
            layer.active = true;
        });
    };

    world.forEachParticle([&addOrUpdateRef](Entity entity, const Particle& particle) {
        if (particle.radius > 0.0f) {
            addOrUpdateRef(entity,
                           true,
                           particle.position,
                           particle.radius,
                           particle.collisionLayer,
                           particle.collisionMask);
        }
    });

    world.forEachCollisionSphere([&addOrUpdateRef](Entity entity, const CollisionSphere& sphere) {
        if (sphere.enabled && sphere.radius > 0.0f) {
            addOrUpdateRef(entity,
                           false,
                           sphere.center,
                           sphere.radius,
                           sphere.collisionLayer,
                           sphere.collisionMask);
        }
    });

    for (detail::SphereCollisionLayerTree& layer : layers) {
        removeStaleRefs(layer, stamp);
    }

    ++world.sphereCollisionBroadphaseSolveCount_;
    const bool rebuildByCadence =
        world.sphereCollisionBvhRebuildInterval_ > 0u &&
        (world.sphereCollisionBroadphaseSolveCount_ % world.sphereCollisionBvhRebuildInterval_) == 0u;

    std::size_t activeObjectCount = 0u;
    std::size_t bvhNodeCount = 0u;
    for (detail::SphereCollisionLayerTree& layer : layers) {
        if (!layer.active) {
            continue;
        }

        if (rebuildByCadence) {
            layer.broadphase.rebuild();
        }
        activeObjectCount += layer.broadphase.objectCount();
        bvhNodeCount += layer.broadphase.nodeCount();
    }
    world.sphereCollisionBvhNodes_ = bvhNodeCount;

    if (activeObjectCount < 2) {
        return;
    }

    std::vector<std::pair<utils::LinearBVH::ObjectId, utils::LinearBVH::ObjectId>> pairs;
    std::unordered_set<EntityPairKey, EntityPairKeyHash> solvedPairs;

    const float alpha = world.sphereCollisionCompliance_ / (dt * dt);
    std::size_t contactCount = 0;

    const auto solvePair = [&](const detail::SphereCollisionBroadphaseRef& refA,
                               const detail::SphereCollisionBroadphaseRef& refB) {
        if (!collisionFiltersMatch(refA.collisionLayer,
                                   refA.collisionMask,
                                   refB.collisionLayer,
                                   refB.collisionMask)) {
            return;
        }

        EntityPairKey key;
        if (!makeEntityPairKey(refA.entity, refB.entity, key)) {
            return;
        }
        if (!solvedPairs.insert(key).second) {
            return;
        }

        SphereCollisionState stateA = sphereCollisionState(world, refA);
        SphereCollisionState stateB = sphereCollisionState(world, refB);
        if (!stateA.valid || !stateB.valid) {
            return;
        }

        const float minDistance = stateA.radius + stateB.radius;
        if (!(minDistance > 0.0f)) {
            return;
        }

        const Vec3 delta = stateB.position - stateA.position;
        const float distSq = lengthSq(delta);
        if (distSq >= minDistance * minDistance) {
            return;
        }

        const float wA = stateA.inverseMass;
        const float wB = stateB.inverseMass;
        const float wSum = wA + wB;
        if (wSum <= 0.0f) {
            return;
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
    };

    const auto processPairs = [&](const detail::SphereCollisionLayerTree& layerA,
                                  const detail::SphereCollisionLayerTree& layerB,
                                  bool sameLayer) {
        pairs.clear();
        if (sameLayer) {
            layerA.broadphase.overlapPairs(pairs);
        } else {
            layerA.broadphase.overlapPairs(layerB.broadphase, pairs);
        }
        world.sphereCollisionBroadphasePairs_ += pairs.size();
        solvedPairs.reserve(solvedPairs.size() + pairs.size());

        for (const auto& pair : pairs) {
            if (pair.first >= layerA.refsByObject.size() || pair.second >= layerB.refsByObject.size()) {
                continue;
            }

            const detail::SphereCollisionBroadphaseRef& refA =
                layerA.refsByObject[static_cast<std::size_t>(pair.first)];
            const detail::SphereCollisionBroadphaseRef& refB =
                layerB.refsByObject[static_cast<std::size_t>(pair.second)];
            solvePair(refA, refB);
        }
    };

    for (std::size_t layerIndexA = 0; layerIndexA < layers.size(); ++layerIndexA) {
        const detail::SphereCollisionLayerTree& layerA = layers[layerIndexA];
        if (!layerA.active) {
            continue;
        }

        if (layerA.broadphase.objectCount() >= 2 && (layerA.aggregateCollisionMask & layerA.bit) != 0u) {
            processPairs(layerA, layerA, true);
        }

        for (std::size_t layerIndexB = layerIndexA + 1u; layerIndexB < layers.size(); ++layerIndexB) {
            const detail::SphereCollisionLayerTree& layerB = layers[layerIndexB];
            if (!layerB.active) {
                continue;
            }
            if ((layerA.aggregateCollisionMask & layerB.bit) == 0u ||
                (layerB.aggregateCollisionMask & layerA.bit) == 0u) {
                continue;
            }

            processPairs(layerA, layerB, false);
        }
    }

    world.sphereCollisionContacts_ += contactCount;
}

}  // namespace xpbd
