#ifndef XPBD_ENTITY_HPP
#define XPBD_ENTITY_HPP

#include <cstdint>
#include <limits>

namespace xpbd {

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

    static constexpr std::uint64_t kIndexMask = 0x0000'0000'FFFF'FFFFull;
    static constexpr std::uint64_t kGenerationMask = 0x0000'FFFF'0000'0000ull;
    static constexpr std::uint64_t kTypeMask = 0xFFFF'0000'0000'0000ull;
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
        return static_cast<std::uint16_t>((value & kGenerationMask) >> 32u);
    }

    constexpr EntityType type() const
    {
        return static_cast<EntityType>(static_cast<std::uint16_t>((value & kTypeMask) >> 48u));
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

}  // namespace xpbd

#endif  // XPBD_ENTITY_HPP
