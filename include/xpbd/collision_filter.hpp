#ifndef XPBD_COLLISION_FILTER_HPP
#define XPBD_COLLISION_FILTER_HPP

#include <cstdint>

namespace xpbd {

using CollisionLayerMask = std::uint32_t;

constexpr CollisionLayerMask kCollisionLayerDefault = CollisionLayerMask{1u};
constexpr CollisionLayerMask kCollisionLayerAll = ~CollisionLayerMask{0u};
constexpr int kCollisionLayerBitCount = 32;

inline bool collisionFiltersMatch(CollisionLayerMask layerA,
                                  CollisionLayerMask maskA,
                                  CollisionLayerMask layerB,
                                  CollisionLayerMask maskB)
{
    return (layerA & maskB) != 0u && (layerB & maskA) != 0u;
}

}  // namespace xpbd

#endif  // XPBD_COLLISION_FILTER_HPP
