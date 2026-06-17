#ifndef XPBD_WORLD_HPP
#define XPBD_WORLD_HPP

#include "xpbd/colliders/collision_sphere.hpp"
#include "xpbd/constraints/distance_constraint.hpp"
#include "xpbd/entity.hpp"
#include "xpbd/particle.hpp"
#include "xpbd/typed_store.hpp"
#include "xpbd/xpbd_simd.hpp"

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

namespace xpbd {

class XPBDWorld {
public:
    using System = std::function<void(XPBDWorld&, float)>;

    XPBDWorld();

    Entity createParticle(const Vec3& position,
                          float mass,
                          float radius = 0.025f,
                          CollisionLayerMask collisionLayer = kCollisionLayerDefault,
                          CollisionLayerMask collisionMask = kCollisionLayerAll);
    Entity createDistanceConstraint(Entity particleA,
                                    Entity particleB,
                                    float restLength,
                                    float compliance = 0.0f);
    Entity createCollisionSphere(const Vec3& center,
                                 float radius,
                                 CollisionLayerMask collisionLayer = kCollisionLayerDefault,
                                 CollisionLayerMask collisionMask = kCollisionLayerAll);

    bool destroy(Entity entity);
    bool alive(Entity entity) const;

    Particle* particle(Entity entity);
    const Particle* particle(Entity entity) const;

    DistanceConstraint* distanceConstraint(Entity entity);
    const DistanceConstraint* distanceConstraint(Entity entity) const;

    CollisionSphere* collisionSphere(Entity entity);
    const CollisionSphere* collisionSphere(Entity entity) const;

    bool setParticleCollisionFilter(Entity entity,
                                    CollisionLayerMask collisionLayer,
                                    CollisionLayerMask collisionMask);
    bool setCollisionSphereFilter(Entity entity,
                                  CollisionLayerMask collisionLayer,
                                  CollisionLayerMask collisionMask);

    void clearEntities();

    void setGravity(const Vec3& gravity);
    const Vec3& gravity() const;

    void setDamping(float damping);
    float damping() const;

    void setSolverIterations(int iterations);
    int solverIterations() const;

    void setSubsteps(int substeps);
    int substeps() const;

    void setSphereCollisionsEnabled(bool enabled);
    bool sphereCollisionsEnabled() const;

    void setSphereCollisionCompliance(float compliance);
    float sphereCollisionCompliance() const;

    void addIntegrationSystem(System system);
    void addConstraintSystem(System system);
    void clearSystems();

    void step(float dt);

    std::size_t particleSlots() const;
    std::size_t particleCount() const;
    std::size_t distanceConstraintSlots() const;
    std::size_t distanceConstraintCount() const;
    std::size_t collisionSphereSlots() const;
    std::size_t collisionSphereCount() const;
    std::size_t sphereCollisionBroadphasePairCount() const;
    std::size_t sphereCollisionContactCount() const;
    std::size_t sphereCollisionBvhNodeCount() const;

    const char* simdBackendName() const;

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
    void forEachCollisionSphere(Func&& func)
    {
        collisionSpheres_.forEachAlive(EntityType::SphereRigidBody, std::forward<Func>(func));
    }

    template <typename Func>
    void forEachCollisionSphere(Func&& func) const
    {
        collisionSpheres_.forEachAlive(EntityType::SphereRigidBody, std::forward<Func>(func));
    }

    static void verletIntegrationSystem(XPBDWorld& world, float dt);
    static void distanceConstraintSystem(XPBDWorld& world, float dt);
    static void sphereCollisionSystem(XPBDWorld& world, float dt);

private:
    void resetConstraintLambdas();
    void updateParticleVelocities(float dt);

    TypedStore<Particle> particles_;
    TypedStore<DistanceConstraint> distanceConstraints_;
    TypedStore<CollisionSphere> collisionSpheres_;
    std::vector<System> integrationSystems_;
    std::vector<System> constraintSystems_;
    const detail::SimdDispatch* simd_ = nullptr;
    Vec3 gravity_ = {0.0f, -9.81f, 0.0f};
    float damping_ = 0.995f;
    int solverIterations_ = 8;
    int substeps_ = 4;
    bool sphereCollisionsEnabled_ = true;
    float sphereCollisionCompliance_ = 0.0f;
    std::size_t sphereCollisionBroadphasePairs_ = 0;
    std::size_t sphereCollisionContacts_ = 0;
    std::size_t sphereCollisionBvhNodes_ = 0;
};

}  // namespace xpbd

#endif  // XPBD_WORLD_HPP
