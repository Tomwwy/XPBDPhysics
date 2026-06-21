#ifndef XPBD_WORLD_HPP
#define XPBD_WORLD_HPP

#include "linear_bvh.hpp"
#include "utils/shapes.hpp"
#include "xpbd/broadphase_partition.hpp"
#include "xpbd/colliders/collider.hpp"
#include "xpbd/colliders/world_shape.hpp"
#include "xpbd/constraints/contact_constraint.hpp"
#include "xpbd/constraints/distance_constraint.hpp"
#include "xpbd/entity.hpp"
#include "xpbd/particle.hpp"
#include "xpbd/typed_store.hpp"
#include "xpbd/xpbd_simd.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace xpbd {

class XPBDWorld {
public:
    using System = std::function<void(XPBDWorld&, float)>;

    XPBDWorld();

    // --- Entity creation ---------------------------------------------------
    Entity createParticle(const Vec3& position, float mass);
    Entity createDistanceConstraint(Entity particleA,
                                    Entity particleB,
                                    float restLength,
                                    float compliance = 0.0f);

    // Attach collision geometry + filtering to a body (a particle today). The
    // proxy lives in `partition`'s broadphase; `offset` is added to the body's
    // world origin.
    Entity createCollider(Entity body,
                          const Shape& shape,
                          CollisionLayerMask layer = kCollisionLayerDefault,
                          CollisionLayerMask mask = kCollisionLayerAll,
                          BroadphasePartition partition = BroadphasePartition::Dynamic,
                          const Vec3& offset = {});

    // Body-less static collider placed directly in the world (Static partition).
    Entity createStaticCollider(const Shape& shape,
                                const Vec3& worldPosition,
                                CollisionLayerMask layer = kCollisionLayerDefault,
                                CollisionLayerMask mask = kCollisionLayerAll);

    bool destroy(Entity entity);
    bool alive(Entity entity) const;

    // --- Accessors ---------------------------------------------------------
    Particle* particle(Entity entity);
    const Particle* particle(Entity entity) const;

    DistanceConstraint* distanceConstraint(Entity entity);
    const DistanceConstraint* distanceConstraint(Entity entity) const;

    Collider* collider(Entity entity);
    const Collider* collider(Entity entity) const;

    // Resolve a collider's local shape into its world-space form (e.g.
    // WorldSphere), or std::nullopt if the collider can't currently collide:
    // missing/disabled, wrong shape type, dead body, or degenerate geometry.
    // This is the generic replacement for the old
    // colliderIsActiveAndIsSphere + computeColliderWorldSphere pair; adding a
    // shape means specializing WorldShapeTraits, not touching this method.
    template <ShapeType T>
    std::optional<typename WorldShapeTraits<T>::WorldType>
    worldShape(const Collider& collider) const
    {
        using Traits = WorldShapeTraits<T>;
        if (!collider.enabled || collider.shape.type != T) {
            return std::nullopt;
        }
        BodyTransform xf;
        if (collider.body.valid()) {
            const Particle* body = particle(collider.body);
            if (body == nullptr) {
                return std::nullopt;  // body died; proxy reaped next refresh
            }
            xf.position = body->position;
        }
        if (!Traits::localValid(collider.shape)) {
            return std::nullopt;
        }
        return Traits::toWorld(collider.shape, xf, collider.offset);
    }

    // Same as above but resolving the collider entity first; std::nullopt if the
    // entity isn't a live collider.
    template <ShapeType T>
    std::optional<typename WorldShapeTraits<T>::WorldType>
    worldShape(Entity entity) const
    {
        const Collider* c = collider(entity);
        return c != nullptr ? worldShape<T>(*c)
                            : std::optional<typename WorldShapeTraits<T>::WorldType>{};
    }

    bool setColliderFilter(Entity entity,
                           CollisionLayerMask layer,
                           CollisionLayerMask mask);

    void clearEntities();

    // --- Configuration -----------------------------------------------------
    void setGravity(const Vec3& gravity);
    const Vec3& gravity() const;

    void setDamping(float damping);
    float damping() const;

    void setSolverIterations(int iterations);
    int solverIterations() const;

    void setSubsteps(int substeps);
    int substeps() const;

    void setCollisionsEnabled(bool enabled);
    bool collisionsEnabled() const;

    void setContactCompliance(float compliance);
    float contactCompliance() const;

    // 0 disables cadence rebuilds; structural changes still rebuild lazily.
    void setBroadphaseRebuildInterval(std::size_t interval);
    std::size_t broadphaseRebuildInterval() const;

    void addIntegrationSystem(System system);
    void addConstraintSystem(System system);
    void clearSystems();

    void step(float dt);

    // --- Statistics --------------------------------------------------------
    std::size_t particleSlots() const;
    std::size_t particleCount() const;
    std::size_t distanceConstraintSlots() const;
    std::size_t distanceConstraintCount() const;
    std::size_t colliderSlots() const;
    std::size_t colliderCount() const;
    std::size_t broadphasePairCount() const;
    std::size_t contactCount() const;
    std::size_t broadphaseNodeCount() const;

    const char* simdBackendName() const;

    // --- Iteration ---------------------------------------------------------
    template <typename Func>
    void forEachParticle(Func&& func)
    {
        particles_.forEachAlive(EntityType::Particle, std::forward<Func>(func));
    }

    template <typename Func>
    void forEachParticle(Func&& func) const
    {
        particles_.forEachAlive(EntityType::Particle, std::forward<Func>(func));
    }

    template <typename Func>
    void forEachDistanceConstraint(Func&& func)
    {
        distanceConstraints_.forEachAlive(EntityType::DistanceConstraint, std::forward<Func>(func));
    }

    template <typename Func>
    void forEachDistanceConstraint(Func&& func) const
    {
        distanceConstraints_.forEachAlive(EntityType::DistanceConstraint, std::forward<Func>(func));
    }

    template <typename Func>
    void forEachCollider(Func&& func)
    {
        colliders_.forEachAlive(EntityType::Collider, std::forward<Func>(func));
    }

    template <typename Func>
    void forEachCollider(Func&& func) const
    {
        colliders_.forEachAlive(EntityType::Collider, std::forward<Func>(func));
    }

    static void verletIntegrationSystem(XPBDWorld& world, float dt);
    static void distanceConstraintSystem(XPBDWorld& world, float dt);

private:
    void resetConstraintLambdas();
    void updateParticleVelocities(float dt);

    // Collision pipeline (built-in, gated by collisionsEnabled_).
    void refreshBroadphase();
    void removeColliderProxy(Collider& collider);
    void generateContacts(float dt);
    void solveContacts(float dt);

    utils::LinearBVH& broadphase(BroadphasePartition partition);

    TypedStore<Particle> particles_;
    TypedStore<DistanceConstraint> distanceConstraints_;
    TypedStore<Collider> colliders_;
    std::vector<System> integrationSystems_;
    std::vector<System> constraintSystems_;
    std::array<utils::LinearBVH, kBroadphasePartitionCount> broadphases_;
    std::vector<ContactConstraint> contacts_;
    const detail::SimdDispatch* simd_ = nullptr;
    Vec3 gravity_ = {0.0f, -9.81f, 0.0f};
    float damping_ = 0.995f;
    int solverIterations_ = 8;
    int substeps_ = 4;
    bool collisionsEnabled_ = true;
    float contactCompliance_ = 0.0f;
    std::size_t broadphaseRebuildInterval_ = 32u;
    std::size_t broadphaseSolveCount_ = 0u;
    std::size_t broadphasePairs_ = 0;
    std::size_t contacts_count_ = 0;
    std::size_t broadphaseNodes_ = 0;
};

}  // namespace xpbd

#endif  // XPBD_WORLD_HPP
