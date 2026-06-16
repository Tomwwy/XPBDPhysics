#ifndef XPBD_WORLD_HPP
#define XPBD_WORLD_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace xpbd {

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    constexpr Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    constexpr Vec3 operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    constexpr Vec3 operator/(float scalar) const { return {x / scalar, y / scalar, z / scalar}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3& operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
};

inline constexpr Vec3 operator*(float scalar, const Vec3& value)
{
    return value * scalar;
}

inline float dot(const Vec3& lhs, const Vec3& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline float lengthSquared(const Vec3& value)
{
    return dot(value, value);
}

inline float length(const Vec3& value)
{
    return std::sqrt(lengthSquared(value));
}

inline Vec3 normalized(const Vec3& value)
{
    const float len = length(value);
    return len > 0.0f ? value / len : Vec3{};
}

enum class EntityType : std::uint16_t {
    Invalid = 0,
    Particle = 1,
    DistanceConstraint = 2,
    TetrahedronVolumeConstraint = 3,
    VolumeConstraint = 4,
    BendingConstraint = 5,
    Triangle = 6,
    Tetrahedron = 7,
    Box = 8,
    Capsule = 9,
    SphereRigidBody = 10,
    BoxRigidBody = 11,
    TetrahedronRigidBody = 12,
    CapsuleRigidBody = 13,
};

struct Entity {
    std::uint64_t value = 0;

    static constexpr std::uint64_t kIndexMask = 0xFFFF'FFFFull;
    static constexpr std::uint64_t kGenerationMask = 0xFFFFull;
    static constexpr std::uint32_t kInvalidIndex = std::numeric_limits<std::uint32_t>::max();

    constexpr Entity() = default;
    constexpr explicit Entity(std::uint64_t packed) : value(packed) {}

    static constexpr Entity make(EntityType type, std::uint32_t index, std::uint16_t generation)
    {
        return Entity{(static_cast<std::uint64_t>(type) << 48u) |
                      (static_cast<std::uint64_t>(generation) << 32u) |
                      static_cast<std::uint64_t>(index)};
    }

    constexpr std::uint32_t index() const
    {
        return static_cast<std::uint32_t>(value & kIndexMask);
    }

    constexpr std::uint16_t generation() const
    {
        return static_cast<std::uint16_t>((value >> 32u) & kGenerationMask);
    }

    constexpr EntityType type() const
    {
        return static_cast<EntityType>(static_cast<std::uint16_t>((value >> 48u) & kGenerationMask));
    }

    constexpr bool valid() const
    {
        return type() != EntityType::Invalid;
    }

    constexpr explicit operator bool() const
    {
        return valid();
    }

    friend constexpr bool operator==(Entity lhs, Entity rhs)
    {
        return lhs.value == rhs.value;
    }

    friend constexpr bool operator!=(Entity lhs, Entity rhs)
    {
        return lhs.value != rhs.value;
    }
};

struct Particle {
    Vec3 position;
    Vec3 previousPosition;
    Vec3 velocity;
    Vec3 externalAcceleration;
    float inverseMass = 1.0f;
    float radius = 0.025f;
};

struct DistanceConstraint {
    Entity particleA;
    Entity particleB;
    float restLength = 0.0f;
    float compliance = 0.0f;
    float lambda = 0.0f;
    bool enabled = true;
};

template <typename T>
class TypedStore {
public:
    Entity create(EntityType type, const T& value)
    {
        std::uint32_t index = 0;
        if (!freeList_.empty()) {
            index = freeList_.back();
            freeList_.pop_back();
        } else {
            assert(data_.size() <= static_cast<std::size_t>(Entity::kInvalidIndex));
            index = static_cast<std::uint32_t>(data_.size());
            data_.push_back(T{});
            generations_.push_back(1);
            alive_.push_back(false);
        }

        data_[index] = value;
        alive_[index] = true;
        return Entity::make(type, index, generations_[index]);
    }

    bool destroy(Entity entity)
    {
        const std::uint32_t index = entity.index();
        if (!alive(entity)) {
            return false;
        }

        alive_[index] = false;
        generations_[index] = nextGeneration(generations_[index]);
        freeList_.push_back(index);
        return true;
    }

    bool alive(Entity entity) const
    {
        const std::uint32_t index = entity.index();
        return index < alive_.size() &&
               alive_[index] &&
               generations_[index] == entity.generation();
    }

    T* get(Entity entity)
    {
        return alive(entity) ? &data_[entity.index()] : nullptr;
    }

    const T* get(Entity entity) const
    {
        return alive(entity) ? &data_[entity.index()] : nullptr;
    }

    void destroyAll()
    {
        freeList_.clear();
        for (std::uint32_t index = 0; index < data_.size(); ++index) {
            if (alive_[index]) {
                generations_[index] = nextGeneration(generations_[index]);
            }
            alive_[index] = false;
            freeList_.push_back(index);
        }
    }

    void release()
    {
        data_.clear();
        generations_.clear();
        alive_.clear();
        freeList_.clear();
    }

    std::size_t slots() const
    {
        return data_.size();
    }

    std::size_t aliveCount() const
    {
        return alive_.size() - freeList_.size();
    }

    template <typename Func>
    void forEachAlive(EntityType type, Func&& func)
    {
        for (std::uint32_t index = 0; index < data_.size(); ++index) {
            if (!alive_[index]) {
                continue;
            }
            func(Entity::make(type, index, generations_[index]), data_[index]);
        }
    }

    template <typename Func>
    void forEachAlive(EntityType type, Func&& func) const
    {
        for (std::uint32_t index = 0; index < data_.size(); ++index) {
            if (!alive_[index]) {
                continue;
            }
            func(Entity::make(type, index, generations_[index]), data_[index]);
        }
    }

private:
    static std::uint16_t nextGeneration(std::uint16_t generation)
    {
        ++generation;
        return generation == 0 ? 1 : generation;
    }

    std::vector<T> data_;
    std::vector<std::uint16_t> generations_;
    std::vector<bool> alive_;
    std::vector<std::uint32_t> freeList_;
};

class XPBDWorld {
public:
    using System = std::function<void(XPBDWorld&, float)>;

    Entity createParticle(const Vec3& position, float mass, float radius = 0.025f)
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

    Entity createDistanceConstraint(Entity particleA,
                                    Entity particleB,
                                    float restLength,
                                    float compliance = 0.0f)
    {
        DistanceConstraint constraint;
        constraint.particleA = particleA;
        constraint.particleB = particleB;
        constraint.restLength = std::max(0.0f, restLength);
        constraint.compliance = std::max(0.0f, compliance);
        return distanceConstraints_.create(EntityType::DistanceConstraint, constraint);
    }

    bool destroy(Entity entity)
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

    bool alive(Entity entity) const
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

    Particle* particle(Entity entity)
    {
        return entity.type() == EntityType::Particle ? particles_.get(entity) : nullptr;
    }

    const Particle* particle(Entity entity) const
    {
        return entity.type() == EntityType::Particle ? particles_.get(entity) : nullptr;
    }

    DistanceConstraint* distanceConstraint(Entity entity)
    {
        return entity.type() == EntityType::DistanceConstraint ? distanceConstraints_.get(entity) : nullptr;
    }

    const DistanceConstraint* distanceConstraint(Entity entity) const
    {
        return entity.type() == EntityType::DistanceConstraint ? distanceConstraints_.get(entity) : nullptr;
    }

    void clearEntities()
    {
        particles_.destroyAll();
        distanceConstraints_.destroyAll();
    }

    void setGravity(const Vec3& gravity)
    {
        gravity_ = gravity;
    }

    const Vec3& gravity() const
    {
        return gravity_;
    }

    void setDamping(float damping)
    {
        damping_ = std::max(0.0f, std::min(damping, 1.0f));
    }

    float damping() const
    {
        return damping_;
    }

    void setSolverIterations(int iterations)
    {
        solverIterations_ = std::max(1, iterations);
    }

    int solverIterations() const
    {
        return solverIterations_;
    }

    void setSubsteps(int substeps)
    {
        substeps_ = std::max(1, substeps);
    }

    int substeps() const
    {
        return substeps_;
    }

    void addIntegrationSystem(System system)
    {
        integrationSystems_.push_back(std::move(system));
    }

    void addConstraintSystem(System system)
    {
        constraintSystems_.push_back(std::move(system));
    }

    void clearSystems()
    {
        integrationSystems_.clear();
        constraintSystems_.clear();
    }

    void step(float dt)
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

    std::size_t particleSlots() const
    {
        return particles_.slots();
    }

    std::size_t particleCount() const
    {
        return particles_.aliveCount();
    }

    std::size_t distanceConstraintSlots() const
    {
        return distanceConstraints_.slots();
    }

    std::size_t distanceConstraintCount() const
    {
        return distanceConstraints_.aliveCount();
    }

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

    static void verletIntegrationSystem(XPBDWorld& world, float dt)
    {
        const float dtSq = dt * dt;
        world.forEachParticle([&world, dt, dtSq](Entity, Particle& particle) {
            if (particle.inverseMass <= 0.0f) {
                particle.previousPosition = particle.position;
                particle.velocity = {};
                particle.externalAcceleration = {};
                return;
            }

            const Vec3 current = particle.position;
            const Vec3 velocityStep = (particle.position - particle.previousPosition) * world.damping_;
            const Vec3 acceleration = world.gravity_ + particle.externalAcceleration;
            particle.position += velocityStep + acceleration * dtSq;
            particle.previousPosition = current;
            particle.velocity = (particle.position - particle.previousPosition) / dt;
            particle.externalAcceleration = {};
        });
    }

    static void distanceConstraintSystem(XPBDWorld& world, float dt)
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

private:
    void resetConstraintLambdas()
    {
        forEachDistanceConstraint([](Entity, DistanceConstraint& constraint) {
            constraint.lambda = 0.0f;
        });
    }

    void updateParticleVelocities(float dt)
    {
        if (!(dt > 0.0f)) {
            return;
        }

        forEachParticle([dt](Entity, Particle& particle) {
            particle.velocity = (particle.position - particle.previousPosition) / dt;
        });
    }

    TypedStore<Particle> particles_;
    TypedStore<DistanceConstraint> distanceConstraints_;
    std::vector<System> integrationSystems_;
    std::vector<System> constraintSystems_;
    Vec3 gravity_ = {0.0f, -9.81f, 0.0f};
    float damping_ = 0.995f;
    int solverIterations_ = 8;
    int substeps_ = 4;
};

inline float distance(const Vec3& lhs, const Vec3& rhs)
{
    return length(lhs - rhs);
}

}  // namespace xpbd

#endif  // XPBD_WORLD_HPP
