#ifndef XPBD_WORLD_HPP
#define XPBD_WORLD_HPP

#include "xpbd/xpbd_simd.hpp"

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
            alive_.push_back(0);
        }

        data_[index] = value;
        alive_[index] = 1;
        return Entity::make(type, index, generations_[index]);
    }

    bool destroy(Entity entity)
    {
        const std::uint32_t index = entity.index();
        if (!alive(entity)) {
            return false;
        }

        alive_[index] = 0;
        generations_[index] = nextGeneration(generations_[index]);
        freeList_.push_back(index);
        return true;
    }

    bool alive(Entity entity) const
    {
        const std::uint32_t index = entity.index();
        return index < generations_.size() &&
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
            if (alive_[index] != 0) {
                generations_[index] = nextGeneration(generations_[index]);
            }
            alive_[index] = 0;
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

    T* data()
    {
        return data_.empty() ? nullptr : data_.data();
    }

    const T* data() const
    {
        return data_.empty() ? nullptr : data_.data();
    }

    const std::uint8_t* aliveData() const
    {
        return alive_.empty() ? nullptr : alive_.data();
    }

    template <typename Func>
    void forEachAlive(EntityType type, Func&& func)
    {
        for (std::uint32_t index = 0; index < data_.size(); ++index) {
            if (alive_[index] == 0) {
                continue;
            }
            func(Entity::make(type, index, generations_[index]), data_[index]);
        }
    }

    template <typename Func>
    void forEachAlive(EntityType type, Func&& func) const
    {
        for (std::uint32_t index = 0; index < data_.size(); ++index) {
            if (alive_[index] == 0) {
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
    std::vector<std::uint8_t> alive_;
    std::vector<std::uint32_t> freeList_;
};

class XPBDWorld {
public:
    using System = std::function<void(XPBDWorld&, float)>;

    XPBDWorld();

    Entity createParticle(const Vec3& position, float mass, float radius = 0.025f);
    Entity createDistanceConstraint(Entity particleA,
                                    Entity particleB,
                                    float restLength,
                                    float compliance = 0.0f);

    bool destroy(Entity entity);
    bool alive(Entity entity) const;

    Particle* particle(Entity entity);
    const Particle* particle(Entity entity) const;

    DistanceConstraint* distanceConstraint(Entity entity);
    const DistanceConstraint* distanceConstraint(Entity entity) const;

    void clearEntities();

    void setGravity(const Vec3& gravity);
    const Vec3& gravity() const;

    void setDamping(float damping);
    float damping() const;

    void setSolverIterations(int iterations);
    int solverIterations() const;

    void setSubsteps(int substeps);
    int substeps() const;

    void addIntegrationSystem(System system);
    void addConstraintSystem(System system);
    void clearSystems();

    void step(float dt);

    std::size_t particleSlots() const;
    std::size_t particleCount() const;
    std::size_t distanceConstraintSlots() const;
    std::size_t distanceConstraintCount() const;

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

    static void verletIntegrationSystem(XPBDWorld& world, float dt);
    static void distanceConstraintSystem(XPBDWorld& world, float dt);

private:
    void resetConstraintLambdas();
    void updateParticleVelocities(float dt);

    TypedStore<Particle> particles_;
    TypedStore<DistanceConstraint> distanceConstraints_;
    std::vector<System> integrationSystems_;
    std::vector<System> constraintSystems_;
    const detail::SimdDispatch* simd_ = nullptr;
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
