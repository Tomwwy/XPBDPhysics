#ifndef XPBD_BROADPHASE_PARTITION_HPP
#define XPBD_BROADPHASE_PARTITION_HPP

#include <cstddef>
#include <cstdint>

namespace xpbd {

// Which broadphase BVH a collider's proxy lives in. This is the *motion* axis,
// orthogonal to collision layers (the *filtering* axis). Static geometry never
// refits, so keeping it in its own tree avoids touching it every step. Static
// colliders still collide with Dynamic ones (Dynamic x Static pass); only the
// Static x Static pass is skipped.
enum class BroadphasePartition : std::uint8_t {
    Static = 0,
    Dynamic = 1,
};

constexpr std::size_t kBroadphasePartitionCount = 2;

constexpr std::size_t partitionIndex(BroadphasePartition partition)
{
    return static_cast<std::size_t>(partition);
}

}  // namespace xpbd

#endif  // XPBD_BROADPHASE_PARTITION_HPP
