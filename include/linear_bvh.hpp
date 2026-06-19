// linear_bvh.hpp -- a Morton-code linear BVH for broadphase comparisons.
//
// Self-contained broadphase BVH. Drop this header + utils/ + unordered_dense/
// into your project and include "linear_bvh.hpp".
//
// Provides:
//   - AABB / sphere overlap queries
//   - self-overlap pair extraction
//   - raycast with narrowphase fallback (Collider::raycast)
//   - incremental insert / update / remove with lazy rebuild
#ifndef UTILS_LINEAR_BVH_HPP
#define UTILS_LINEAR_BVH_HPP

#include "utils/collider.hpp"
#include "utils/math.hpp"
#include "unordered_dense/unordered_dense.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <type_traits>
#include <stdexcept>
#include <utility>
#include <vector>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace utils {

class LinearBVH {
    template <typename Key>
    using DenseSet = ankerl::unordered_dense::set<Key>;

public:
    using ObjectId = uint64_t;
    using ObjectIndex = uint32_t;
    static constexpr ObjectId kInvalid = 0;
    static constexpr ObjectIndex kInvalidIndex = std::numeric_limits<ObjectIndex>::max();

    struct Node {
        AABB bounds;
        int left = -1;
        int right = -1;
        size_t start = 0;  // half-open range in sortedIds_ for leaves
        size_t end = 0;

        [[nodiscard]] bool leaf() const { return left < 0 && right < 0; }
    };

    struct RayHit {
        bool hit = false;
        ObjectId id = kInvalid;
        float t = 0.0f;
    };

    LinearBVH() = default;

    // Borrowed insertion: `collider` must outlive the BVH registration.
    ObjectId insert(const Collider& collider) {
        AABB b;
        if (!prepareCollider(collider, b)) return kInvalid;

        Object obj;
        obj.collider = &collider;
        obj.bounds = b;
        return commitObject(std::move(obj));
    }

    ObjectId insert(Collider&&) = delete;
    ObjectId insert(const Collider&&) = delete;

    ObjectId insertSphere(const Vec3& center, float radius) {
        if (!(radius > 0.0f)) return kInvalid;
        return insertOwned(std::make_unique<SphereCollider>(center, radius));
    }

    ObjectId insertBox(const AABB& box) {
        return insertOwned(std::make_unique<BoxCollider>(box));
    }

    ObjectId insertTriangle(const Vec3& a, const Vec3& b, const Vec3& c) {
        return insertOwned(std::make_unique<TriangleCollider>(a, b, c));
    }

    ObjectId insertTetrahedron(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
        if (std::abs(dot(b - a, cross(c - a, d - a))) <= kVoxelEps) return kInvalid;
        return insertOwned(std::make_unique<TetrahedronCollider>(a, b, c, d));
    }

    void remove(ObjectId id) {
        ObjectIndex index = kInvalidIndex;
        ObjectSlot* slot = slotFor(id, &index);
        if (slot == nullptr) return;

        slot->object = Object{};
        slot->alive = false;
        slot->generation = nextGeneration(slot->generation);
        if (index < leafForObject_.size()) leafForObject_[index] = -1;
        freeList_.push_back(index);
        --objectCount_;
        dirty_ = true;
        refitDirty_ = false;
        pendingRefitLeaves_.clear();
        pendingRefitLeafSet_.clear();
    }

    // Borrowed update: `collider` must outlive the BVH registration.
    bool update(ObjectId id, const Collider& collider) {
        ObjectIndex index = kInvalidIndex;
        ObjectSlot* slot = slotFor(id, &index);
        if (slot == nullptr) return false;

        AABB b;
        if (!prepareCollider(collider, b)) return false;

        slot->object.owned.reset();
        slot->object.collider = &collider;
        slot->object.bounds = b;
        markUpdated(index, b);
        return true;
    }

    bool update(ObjectId, Collider&&) = delete;
    bool update(ObjectId, const Collider&&) = delete;

    bool updateSphere(ObjectId id, const Vec3& center, float radius) {
        if (!(radius > 0.0f)) return false;
        return updateOwned(id, std::make_unique<SphereCollider>(center, radius));
    }

    bool updateBox(ObjectId id, const AABB& box) {
        return updateOwned(id, std::make_unique<BoxCollider>(box));
    }

    bool updateTriangle(ObjectId id, const Vec3& a, const Vec3& b, const Vec3& c) {
        return updateOwned(id, std::make_unique<TriangleCollider>(a, b, c));
    }

    bool updateTetrahedron(ObjectId id, const Vec3& a, const Vec3& b,
                           const Vec3& c, const Vec3& d) {
        if (std::abs(dot(b - a, cross(c - a, d - a))) <= kVoxelEps) return false;
        return updateOwned(id, std::make_unique<TetrahedronCollider>(a, b, c, d));
    }

    void clear() {
        objects_.clear();
        freeList_.clear();
        objectCount_ = 0;
        nodes_.clear();
        sortedIds_.clear();
        parents_.clear();
        leafForObject_.clear();
        pendingRefitLeaves_.clear();
        pendingRefitLeafSet_.clear();
        dirty_ = false;
        refitDirty_ = false;
        incrementalEditCount_ = 0;
        unqueriedInsertCount_ = 0;
        lastRebuildRootArea_ = 0.0f;
    }

    size_t objectCount() const { return objectCount_; }
    bool contains(ObjectId id) const { return objectFor(id) != nullptr; }
    const Collider& collider(ObjectId id) const { return *checkedObject(id).collider; }
    AABB bounds(ObjectId id) const { return checkedObject(id).bounds; }
    bool dirty() const { return dirty_ || refitDirty_; }

    static constexpr ObjectIndex indexOf(ObjectId id) {
        return id == kInvalid ? kInvalidIndex
                              : static_cast<ObjectIndex>(id & UINT64_C(0xffffffff));
    }

    static constexpr uint32_t generationOf(ObjectId id) {
        return static_cast<uint32_t>(id >> 32u);
    }

    ObjectId idForIndex(ObjectIndex index) const {
        return objectIdForIndex(index);
    }

    size_t nodeCount() const {
        ensureBuilt();
        return nodes_.size();
    }

    const std::vector<Node>& nodes() const {
        ensureBuilt();
        return nodes_;
    }

    void rebuild() const {
        nodes_.clear();
        sortedIds_.clear();
        parents_.clear();
        leafForObject_.assign(objects_.size(), -1);
        pendingRefitLeaves_.clear();
        pendingRefitLeafSet_.clear();
        refitDirty_ = false;
        incrementalEditCount_ = 0;
        unqueriedInsertCount_ = 0;
        lastRebuildRootArea_ = 0.0f;

        if (objectCount_ == 0) {
            dirty_ = false;
            return;
        }
        if (objectCount_ > static_cast<size_t>(std::numeric_limits<int>::max() / 2)) {
            throw std::length_error("LinearBVH: too many objects for int node indices");
        }

        std::vector<BuildRef> refs;
        refs.reserve(objectCount_);

        AABB centroidBounds;
        bool first = true;
        for (ObjectIndex index = 0; index < objects_.size(); ++index) {
            const ObjectSlot& slot = objects_[index];
            if (!slot.alive) continue;

            const Object& obj = slot.object;
            const Vec3 c = boundsCenter(obj.bounds);
            if (first) {
                centroidBounds = {c, c};
                first = false;
            } else {
                centroidBounds.min = vmin(centroidBounds.min, c);
                centroidBounds.max = vmax(centroidBounds.max, c);
            }
            refs.push_back({index, obj.bounds, c, 0});
        }

        for (BuildRef& ref : refs) ref.code = mortonCode(ref.centroid, centroidBounds);
        std::sort(refs.begin(), refs.end(), [](const BuildRef& a, const BuildRef& b) {
            if (a.code != b.code) return a.code < b.code;
            return a.id < b.id;
        });

        sortedIds_.resize(refs.size());
        for (size_t i = 0; i < refs.size(); ++i) sortedIds_[i] = refs[i].id;

        nodes_.resize(refs.size() * 2 - 1);
        parents_.assign(nodes_.size(), -1);
        int nextNode = 1;
        buildRecursive(0, 0, refs.size(), refs, nextNode, -1);
        nodes_.resize(static_cast<size_t>(nextNode));
        parents_.resize(static_cast<size_t>(nextNode));
        lastRebuildRootArea_ = surfaceArea(nodes_[0].bounds);
        dirty_ = false;
    }

    std::vector<ObjectId> queryAABB(const AABB& region) const {
        std::vector<ObjectId> out;
        queryAABB(region, out);
        return out;
    }

    void queryAABB(const AABB& region, std::vector<ObjectId>& out) const {
        out.clear();
        if (objectCount_ == 0 || !validQueryBounds(region)) return;

        ensureBuilt();
        reserveQueryOutput(out);

        static thread_local std::vector<int> stack;
        prepareScratchStack(stack);
        stack.push_back(0);
        while (!stack.empty()) {
            int nodeIdx = stack.back();
            stack.pop_back();
            const Node& node = nodes_[static_cast<size_t>(nodeIdx)];
            if (!node.bounds.overlaps(region)) continue;

            if (node.leaf()) {
                out.push_back(objectIdForIndex(sortedIds_[node.start]));
            } else {
                stack.push_back(node.right);
                stack.push_back(node.left);
            }
        }
    }

    std::vector<ObjectId> querySphere(const Vec3& center, float radius) const {
        std::vector<ObjectId> out;
        querySphere(center, radius, out);
        return out;
    }

    void querySphere(const Vec3& center, float radius, std::vector<ObjectId>& out) const {
        out.clear();
        if (objectCount_ == 0 || radius < 0.0f) return;

        ensureBuilt();
        reserveQueryOutput(out);

        const float radiusSq = radius * radius;
        static thread_local std::vector<int> stack;
        prepareScratchStack(stack);
        stack.push_back(0);
        while (!stack.empty()) {
            int nodeIdx = stack.back();
            stack.pop_back();
            const Node& node = nodes_[static_cast<size_t>(nodeIdx)];
            if (!aabbOverlapsSphere(node.bounds, center, radiusSq)) continue;

            if (node.leaf()) {
                out.push_back(objectIdForIndex(sortedIds_[node.start]));
            } else {
                stack.push_back(node.right);
                stack.push_back(node.left);
            }
        }
    }

    std::vector<std::pair<ObjectIndex, ObjectIndex>> overlapPairs() const {
        std::vector<std::pair<ObjectIndex, ObjectIndex>> out;
        overlapPairs(out);
        return out;
    }

    void overlapPairs(std::vector<std::pair<ObjectIndex, ObjectIndex>>& out) const {
        out.clear();
        if (objectCount_ < 2) return;

        ensureBuilt();
        reservePairOutput(out);
        collectPairs(0, 0, out);
    }

    void overlapPairs(std::vector<std::pair<ObjectId, ObjectId>>& out) const {
        out.clear();
        if (objectCount_ < 2) return;

        ensureBuilt();
        reservePairOutput(out);
        collectPairs(0, 0, out);
    }

    std::vector<std::pair<ObjectIndex, ObjectIndex>> overlapPairs(const LinearBVH& other) const {
        std::vector<std::pair<ObjectIndex, ObjectIndex>> out;
        overlapPairs(other, out);
        return out;
    }

    void overlapPairs(const LinearBVH& other,
                      std::vector<std::pair<ObjectIndex, ObjectIndex>>& out) const {
        out.clear();
        if (&other == this) {
            overlapPairs(out);
            return;
        }
        if (objectCount_ == 0 || other.objectCount_ == 0) return;

        ensureBuilt();
        other.ensureBuilt();
        reservePairOutput(out, other.objectCount_);
        collectPairsWith(0, other, 0, out);
    }

    void overlapPairs(const LinearBVH& other,
                      std::vector<std::pair<ObjectId, ObjectId>>& out) const {
        out.clear();
        if (&other == this) {
            overlapPairs(out);
            return;
        }
        if (objectCount_ == 0 || other.objectCount_ == 0) return;

        ensureBuilt();
        other.ensureBuilt();
        reservePairOutput(out, other.objectCount_);
        collectPairsWith(0, other, 0, out);
    }

    RayHit raycast(const Vec3& origin, const Vec3& dir, float maxDist = 1e6f) const {
        RayHit best;
        best.t = maxDist;
        if (objectCount_ == 0 || maxDist < 0.0f) return RayHit{};

        Vec3 d = dir;
        const float dlen = length(d);
        if (dlen <= 0.0f) return RayHit{};
        d = d / dlen;

        ensureBuilt();
        float rootT = 0.0f;
        if (!intersectNode(0, origin, d, best.t, rootT)) return RayHit{};

        static thread_local std::vector<StackEntry> stack;
        prepareScratchStack(stack);
        stack.push_back({0, rootT});
        while (!stack.empty()) {
            StackEntry entry = stack.back();
            stack.pop_back();
            if (entry.t > best.t) continue;

            const Node& node = nodes_[static_cast<size_t>(entry.node)];
            if (node.leaf()) {
                const ObjectIndex index = sortedIds_[node.start];
                const Object& obj = objects_[index].object;
                float t = 0.0f;
                if (obj.collider->raycast(origin, d, best.t, t) &&
                    (!best.hit || t < best.t)) {
                    best.hit = true;
                    best.id = objectIdForIndex(index);
                    best.t = t;
                }
                continue;
            }

            float tLeft = 0.0f;
            float tRight = 0.0f;
            const bool hitLeft = intersectNode(node.left, origin, d, best.t, tLeft);
            const bool hitRight = intersectNode(node.right, origin, d, best.t, tRight);
            if (hitLeft && hitRight) {
                if (tLeft <= tRight) {
                    stack.push_back({node.right, tRight});
                    stack.push_back({node.left, tLeft});
                } else {
                    stack.push_back({node.left, tLeft});
                    stack.push_back({node.right, tRight});
                }
            } else if (hitLeft) {
                stack.push_back({node.left, tLeft});
            } else if (hitRight) {
                stack.push_back({node.right, tRight});
            }
        }
        return best.hit ? best : RayHit{};
    }

    static constexpr uint64_t morton3D64(uint32_t x, uint32_t y, uint32_t z) {
        return expandBits21(x) | (expandBits21(y) << 1) | (expandBits21(z) << 2);
    }

private:
    struct Object {
        const Collider* collider = nullptr;
        std::unique_ptr<Collider> owned;
        AABB bounds;
    };

    struct ObjectSlot {
        Object object;
        uint32_t generation = 1;
        bool alive = false;
    };

    struct BuildRef {
        ObjectIndex id = kInvalidIndex;
        AABB bounds;
        Vec3 centroid;
        uint64_t code = 0;
    };

    struct StackEntry {
        int node = -1;
        float t = 0.0f;
    };

    ObjectId insertOwned(std::unique_ptr<Collider> c) {
        AABB b;
        if (!prepareCollider(*c, b)) return kInvalid;

        Object obj;
        obj.owned = std::move(c);
        obj.collider = obj.owned.get();
        obj.bounds = b;
        return commitObject(std::move(obj));
    }

    bool updateOwned(ObjectId id, std::unique_ptr<Collider> c) {
        ObjectIndex index = kInvalidIndex;
        ObjectSlot* slot = slotFor(id, &index);
        if (slot == nullptr) return false;

        AABB b;
        if (!prepareCollider(*c, b)) return false;

        slot->object.owned = std::move(c);
        slot->object.collider = slot->object.owned.get();
        slot->object.bounds = b;
        markUpdated(index, b);
        return true;
    }

    ObjectId commitObject(Object obj) {
        ObjectIndex index = kInvalidIndex;
        if (!freeList_.empty()) {
            index = freeList_.back();
            freeList_.pop_back();
        } else {
            if (objects_.size() >= static_cast<size_t>(kInvalidIndex)) {
                throw std::length_error("LinearBVH: ObjectId index space exhausted");
            }
            index = static_cast<ObjectIndex>(objects_.size());
            objects_.push_back(ObjectSlot{});
            leafForObject_.push_back(-1);
        }

        ObjectSlot& slot = objects_[index];
        if (slot.generation == 0u) {
            slot.generation = 1u;
        }
        slot.object = std::move(obj);
        slot.alive = true;
        leafForObject_[index] = -1;
        ++objectCount_;

        markInserted(index, slot.object.bounds);
        return makeObjectId(index, slot.generation);
    }

    static constexpr ObjectId makeObjectId(ObjectIndex index, uint32_t generation) {
        return (static_cast<ObjectId>(generation) << 32u) | static_cast<ObjectId>(index);
    }

    static uint32_t nextGeneration(uint32_t generation) {
        ++generation;
        return generation == 0u ? 1u : generation;
    }

    ObjectSlot* slotFor(ObjectId id, ObjectIndex* outIndex = nullptr) {
        return const_cast<ObjectSlot*>(
            static_cast<const LinearBVH*>(this)->slotFor(id, outIndex));
    }

    const ObjectSlot* slotFor(ObjectId id, ObjectIndex* outIndex = nullptr) const {
        const ObjectIndex index = indexOf(id);
        if (outIndex != nullptr) *outIndex = index;
        if (index == kInvalidIndex || index >= objects_.size()) return nullptr;

        const ObjectSlot& slot = objects_[index];
        return slot.alive && slot.generation == generationOf(id) ? &slot : nullptr;
    }

    Object* objectFor(ObjectId id, ObjectIndex* outIndex = nullptr) {
        ObjectSlot* slot = slotFor(id, outIndex);
        return slot != nullptr ? &slot->object : nullptr;
    }

    const Object* objectFor(ObjectId id, ObjectIndex* outIndex = nullptr) const {
        const ObjectSlot* slot = slotFor(id, outIndex);
        return slot != nullptr ? &slot->object : nullptr;
    }

    const Object& checkedObject(ObjectId id) const {
        const Object* object = objectFor(id);
        if (object == nullptr) {
            throw std::out_of_range("LinearBVH: invalid ObjectId");
        }
        return *object;
    }

    ObjectId objectIdForIndex(ObjectIndex index) const {
        if (index == kInvalidIndex || index >= objects_.size()) return kInvalid;
        const ObjectSlot& slot = objects_[index];
        return slot.alive ? makeObjectId(index, slot.generation) : kInvalid;
    }

    static bool prepareCollider(const Collider& collider, AABB& bounds) {
        const float feature = collider.featureSize();
        if (!(feature > 0.0f) || !std::isfinite(feature)) return false;
        bounds = collider.bounds();
        return validObjectBounds(bounds);
    }

    static bool finiteVec(const Vec3& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    static Vec3 boundsCenter(const AABB& b) {
        return {(b.min.x * 0.5f) + (b.max.x * 0.5f),
                (b.min.y * 0.5f) + (b.max.y * 0.5f),
                (b.min.z * 0.5f) + (b.max.z * 0.5f)};
    }

    static bool validObjectBounds(const AABB& b) {
        return finiteVec(b.min) && finiteVec(b.max) &&
               b.max.x >= b.min.x && b.max.y >= b.min.y && b.max.z >= b.min.z;
    }

    static bool validQueryBounds(const AABB& b) {
        return finiteVec(b.min) && finiteVec(b.max) &&
               b.max.x >= b.min.x && b.max.y >= b.min.y && b.max.z >= b.min.z;
    }

    static AABB unite(const AABB& a, const AABB& b) {
        return {vmin(a.min, b.min), vmax(a.max, b.max)};
    }

    static bool sameBounds(const AABB& a, const AABB& b) {
        return a.min == b.min && a.max == b.max;
    }

    static float surfaceArea(const AABB& b) {
        const Vec3 e = b.max - b.min;
        const float x = std::max(0.0f, e.x);
        const float y = std::max(0.0f, e.y);
        const float z = std::max(0.0f, e.z);
        return 2.0f * ((x * y) + (x * z) + (y * z));
    }

    static bool aabbOverlapsSphere(const AABB& box, const Vec3& center, float radiusSq) {
        Vec3 nearest = vclamp(center, box.min, box.max);
        return distanceSq(nearest, center) <= radiusSq + kVoxelEps;
    }

    size_t queryReserveHint() const {
        return std::min(objectCount_, std::max<size_t>(size_t{64}, objectCount_ / 8));
    }

    void reserveQueryOutput(std::vector<ObjectId>& out) const {
        const size_t hint = queryReserveHint();
        if (out.capacity() < hint) out.reserve(hint);
    }

    template <typename Id>
    void reservePairOutput(std::vector<std::pair<Id, Id>>& out) const {
        const size_t hint = std::min(scaledReserveLimit(objectCount_, 4),
                                     std::max<size_t>(size_t{256}, objectCount_));
        if (out.capacity() < hint) out.reserve(hint);
    }

    template <typename Id>
    void reservePairOutput(std::vector<std::pair<Id, Id>>& out,
                           size_t otherObjectCount) const {
        const size_t smaller = std::min(objectCount_, otherObjectCount);
        const size_t hint = std::min(scaledReserveLimit(smaller, 4),
                                     std::max<size_t>(size_t{256}, smaller));
        if (out.capacity() < hint) out.reserve(hint);
    }

    static size_t scaledReserveLimit(size_t count, size_t factor) {
        if (factor == 0) return 0;
        const size_t maxValue = std::numeric_limits<size_t>::max();
        return count > maxValue / factor ? maxValue : count * factor;
    }

    template <typename Entry>
    static void prepareScratchStack(std::vector<Entry>& stack, size_t minCapacity = 64) {
        stack.clear();
        if (stack.capacity() < minCapacity) stack.reserve(minCapacity);
    }

    void ensureBuilt() const {
        if (dirty_) {
            rebuild();
            return;
        }
        if (refitDirty_) {
            if (shouldRebuildForEditCount()) {
                rebuild();
                return;
            }
            refitPending();
            if (shouldRebuildAfterIncrementalEdits()) rebuild();
        }
        unqueriedInsertCount_ = 0;
    }

    int buildRecursive(int nodeIdx, size_t start, size_t end,
                       const std::vector<BuildRef>& refs, int& nextNode,
                       int parent) const {
        Node& node = nodes_[static_cast<size_t>(nodeIdx)];
        parents_[static_cast<size_t>(nodeIdx)] = parent;
        node.start = start;
        node.end = end;

        if (end - start == 1) {
            node.bounds = refs[start].bounds;
            node.left = -1;
            node.right = -1;
            leafForObject_[refs[start].id] = nodeIdx;
            return nodeIdx;
        }

        const size_t split = findSplit(refs, start, end);
        const int leftIdx = nextNode++;
        const int rightIdx = nextNode++;
        node.left = leftIdx;
        node.right = rightIdx;

        buildRecursive(leftIdx, start, split + 1, refs, nextNode, nodeIdx);
        buildRecursive(rightIdx, split + 1, end, refs, nextNode, nodeIdx);

        const Node& left = nodes_[static_cast<size_t>(leftIdx)];
        const Node& right = nodes_[static_cast<size_t>(rightIdx)];
        node.bounds = unite(left.bounds, right.bounds);
        return nodeIdx;
    }

    void markInserted(ObjectIndex id, const AABB& bounds) {
        ++unqueriedInsertCount_;
        if (!dirty_ && shouldDeferUnqueriedInserts()) {
            markDirtyForRebuild();
            return;
        }

        if (!dirty_ && refitDirty_) {
            refitPending();
            if (shouldRebuildAfterIncrementalEdits()) markDirtyForRebuild();
        }

        if (canInsertIncrementally()) {
            insertLeafIncremental(id, bounds);
            ++incrementalEditCount_;
            if (shouldRebuildAfterIncrementalEdits()) markDirtyForRebuild();
        } else {
            markDirtyForRebuild();
        }
    }

    void markUpdated(ObjectIndex id, const AABB& bounds) {
        if (!dirty_ && parents_.size() == nodes_.size() &&
            sortedIds_.size() == objectCount_ &&
            leafForObject_.size() == objects_.size() &&
            id < leafForObject_.size()) {
            const int leafIdx = leafForObject_[id];
            if (leafIdx >= 0 && static_cast<size_t>(leafIdx) < nodes_.size()) {
                nodes_[static_cast<size_t>(leafIdx)].bounds = bounds;
                if (pendingRefitLeafSet_.insert(leafIdx).second) {
                    pendingRefitLeaves_.push_back(leafIdx);
                }
                refitDirty_ = true;
                ++incrementalEditCount_;
                return;
            }
        }
        markDirtyForRebuild();
    }

    void markDirtyForRebuild() {
        dirty_ = true;
        refitDirty_ = false;
        pendingRefitLeaves_.clear();
        pendingRefitLeafSet_.clear();
    }

    bool canInsertIncrementally() const {
        return !dirty_ && !refitDirty_ && !nodes_.empty() &&
               parents_.size() == nodes_.size() &&
               leafForObject_.size() == objects_.size() &&
               sortedIds_.size() + 1 == objectCount_;
    }

    void insertLeafIncremental(ObjectIndex id, const AABB& bounds) {
        if (nodes_.size() > static_cast<size_t>(std::numeric_limits<int>::max() - 2)) {
            markDirtyForRebuild();
            return;
        }

        const int siblingIdx = chooseInsertionSibling(bounds);
        const size_t idIndex = sortedIds_.size();
        sortedIds_.push_back(id);

        Node leaf;
        leaf.bounds = bounds;
        leaf.start = idIndex;
        leaf.end = idIndex + 1;

        if (siblingIdx == 0) {
            const int oldRootIdx = static_cast<int>(nodes_.size());
            nodes_.push_back(nodes_[0]);
            parents_.push_back(0);
            rebindMovedNode(oldRootIdx);

            const int leafIdx = static_cast<int>(nodes_.size());
            nodes_.push_back(leaf);
            parents_.push_back(0);
            leafForObject_[id] = leafIdx;

            Node root;
            root.left = oldRootIdx;
            root.right = leafIdx;
            root.start = std::min(nodes_[static_cast<size_t>(oldRootIdx)].start, leaf.start);
            root.end = std::max(nodes_[static_cast<size_t>(oldRootIdx)].end, leaf.end);
            root.bounds = unite(nodes_[static_cast<size_t>(oldRootIdx)].bounds, bounds);
            nodes_[0] = root;
            parents_[0] = -1;
            return;
        }

        const int oldParentIdx = parents_[static_cast<size_t>(siblingIdx)];
        assert(oldParentIdx >= 0);

        const int leafIdx = static_cast<int>(nodes_.size());
        nodes_.push_back(leaf);
        parents_.push_back(-1);
        leafForObject_[id] = leafIdx;

        const int parentIdx = static_cast<int>(nodes_.size());
        Node parent;
        parent.left = siblingIdx;
        parent.right = leafIdx;
        parent.start = std::min(nodes_[static_cast<size_t>(siblingIdx)].start, leaf.start);
        parent.end = std::max(nodes_[static_cast<size_t>(siblingIdx)].end, leaf.end);
        parent.bounds = unite(nodes_[static_cast<size_t>(siblingIdx)].bounds, bounds);
        nodes_.push_back(parent);
        parents_.push_back(oldParentIdx);

        Node& oldParent = nodes_[static_cast<size_t>(oldParentIdx)];
        if (oldParent.left == siblingIdx) {
            oldParent.left = parentIdx;
        } else {
            assert(oldParent.right == siblingIdx);
            oldParent.right = parentIdx;
        }

        parents_[static_cast<size_t>(siblingIdx)] = parentIdx;
        parents_[static_cast<size_t>(leafIdx)] = parentIdx;
        refitAncestors(parentIdx);
    }

    void rebindMovedNode(int nodeIdx) {
        Node& node = nodes_[static_cast<size_t>(nodeIdx)];
        if (node.leaf()) {
            leafForObject_[sortedIds_[node.start]] = nodeIdx;
            return;
        }
        parents_[static_cast<size_t>(node.left)] = nodeIdx;
        parents_[static_cast<size_t>(node.right)] = nodeIdx;
    }

    int chooseInsertionSibling(const AABB& bounds) const {
        int idx = 0;
        while (!nodes_[static_cast<size_t>(idx)].leaf()) {
            const Node& node = nodes_[static_cast<size_t>(idx)];
            const float nodeArea = surfaceArea(node.bounds);
            const float combinedArea = surfaceArea(unite(node.bounds, bounds));
            // Dynamic AABB-tree insertion heuristic: direct insertion creates a
            // new parent over two children, so this variant intentionally uses
            // the 2x parent/inheritance costs.
            const float parentCost = 2.0f * combinedArea;
            const float inheritanceCost = 2.0f * (combinedArea - nodeArea);

            const float leftCost = insertionChildCost(node.left, bounds, inheritanceCost);
            const float rightCost = insertionChildCost(node.right, bounds, inheritanceCost);
            if (parentCost <= leftCost && parentCost <= rightCost) break;
            idx = leftCost <= rightCost ? node.left : node.right;
        }
        return idx;
    }

    float insertionChildCost(int childIdx, const AABB& bounds, float inheritanceCost) const {
        const Node& child = nodes_[static_cast<size_t>(childIdx)];
        const float combinedArea = surfaceArea(unite(child.bounds, bounds));
        if (child.leaf()) return combinedArea + inheritanceCost;
        return (combinedArea - surfaceArea(child.bounds)) + inheritanceCost;
    }

    void refitPending() const {
        for (int leafIdx : pendingRefitLeaves_) {
            if (leafIdx >= 0 && static_cast<size_t>(leafIdx) < nodes_.size()) {
                refitAncestors(leafIdx);
            }
        }
        pendingRefitLeaves_.clear();
        pendingRefitLeafSet_.clear();
        refitDirty_ = false;
    }

    void refitAncestors(int nodeIdx) const {
        bool canStopOnUnchanged = nodes_[static_cast<size_t>(nodeIdx)].leaf();
        for (int idx = nodeIdx; idx >= 0; idx = parents_[static_cast<size_t>(idx)]) {
            Node& node = nodes_[static_cast<size_t>(idx)];
            if (node.leaf()) continue;

            const Node& left = nodes_[static_cast<size_t>(node.left)];
            const Node& right = nodes_[static_cast<size_t>(node.right)];
            const AABB bounds = unite(left.bounds, right.bounds);
            const size_t start = std::min(left.start, right.start);
            const size_t end = std::max(left.end, right.end);
            if (sameBounds(node.bounds, bounds) && node.start == start && node.end == end) {
                if (canStopOnUnchanged) break;
            } else {
                node.bounds = bounds;
                node.start = start;
                node.end = end;
            }
            canStopOnUnchanged = true;
        }
    }

    bool shouldRebuildAfterIncrementalEdits() const {
        if (objectCount_ == 0 || nodes_.empty()) return false;

        if (shouldRebuildForEditCount()) return true;

        const float rootArea = surfaceArea(nodes_[0].bounds);
        if (lastRebuildRootArea_ > 0.0f && rootArea > lastRebuildRootArea_ * 3.0f) {
            return true;
        }

        if ((incrementalEditCount_ & size_t{15}) != 0) return false;
        const size_t depthLimit = maxAllowedDepth(objectCount_);
        return maxTreeDepth(0) > depthLimit;
    }

    bool shouldRebuildForEditCount() const {
        if (objectCount_ == 0 || nodes_.empty()) return false;
        const size_t editLimit = std::max<size_t>(size_t{64}, objectCount_ / 2);
        return incrementalEditCount_ >= editLimit;
    }

    bool shouldDeferUnqueriedInserts() const {
        // After a sustained append burst, let the next read rebuild once
        // instead of paying dynamic-tree insertion cost for every object.
        return !nodes_.empty() && unqueriedInsertCount_ >= kDeferredInsertBatchThreshold;
    }

    static size_t maxAllowedDepth(size_t objectCount) {
        size_t depth = 0;
        size_t n = std::max<size_t>(objectCount, 1);
        while (n > 1) {
            n = (n + 1) >> 1;
            ++depth;
        }
        return depth * 4 + 16;
    }

    size_t maxTreeDepth(int nodeIdx) const {
        size_t best = 0;
        static thread_local std::vector<std::pair<int, size_t>> stack;
        prepareScratchStack(stack);
        stack.push_back({nodeIdx, 1});
        while (!stack.empty()) {
            const auto entry = stack.back();
            stack.pop_back();

            const Node& node = nodes_[static_cast<size_t>(entry.first)];
            if (node.leaf()) {
                best = std::max(best, entry.second);
                continue;
            }
            stack.push_back({node.left, entry.second + 1});
            stack.push_back({node.right, entry.second + 1});
        }
        return best;
    }

    static size_t findSplit(const std::vector<BuildRef>& refs, size_t start, size_t end) {
        assert(end - start >= 2);
        const uint64_t firstCode = refs[start].code;
        const uint64_t lastCode = refs[end - 1].code;

        if (firstCode == lastCode) return (start + end - 1) >> 1;

        const int commonPrefix = countLeadingZeros(firstCode ^ lastCode);
        size_t left = start;
        size_t right = end - 1;
        while (left < right) {
            const size_t mid = (left + right + 1) >> 1;
            const int midPrefix = countLeadingZeros(firstCode ^ refs[mid].code);
            if (midPrefix > commonPrefix) {
                left = mid;
            } else {
                right = mid - 1;
            }
        }
        return left;
    }

    static int countLeadingZeros(uint64_t x) {
        if (x == 0) return 64;
#if defined(_MSC_VER)
        unsigned long idx = 0;
#if defined(_M_X64) || defined(_M_ARM64)
        _BitScanReverse64(&idx, x);
        return 63 - static_cast<int>(idx);
#else
        const uint32_t hi = static_cast<uint32_t>(x >> 32);
        if (hi != 0) {
            _BitScanReverse(&idx, hi);
            return 31 - static_cast<int>(idx);
        }
        _BitScanReverse(&idx, static_cast<uint32_t>(x));
        return 63 - static_cast<int>(idx);
#endif
#elif defined(__GNUC__) || defined(__clang__)
        return __builtin_clzll(x);
#else
        int count = 0;
        uint64_t bit = uint64_t{1} << 63;
        while ((x & bit) == 0) {
            ++count;
            bit >>= 1;
        }
        return count;
#endif
    }

    static constexpr uint64_t expandBits21(uint32_t v) {
        uint64_t x = static_cast<uint64_t>(v) & UINT64_C(0x1fffff);
        x = (x | (x << 32)) & UINT64_C(0x1f00000000ffff);
        x = (x | (x << 16)) & UINT64_C(0x1f0000ff0000ff);
        x = (x | (x << 8)) & UINT64_C(0x100f00f00f00f00f);
        x = (x | (x << 4)) & UINT64_C(0x10c30c30c30c30c3);
        x = (x | (x << 2)) & UINT64_C(0x1249249249249249);
        return x;
    }

    static uint32_t quantizeAxis(float value, float lo, float extent) {
        constexpr uint32_t kMaxCoord = 0x1fffff;
        if (!(extent > 0.0f) || !std::isfinite(extent)) return 0;

        const double scaled = (static_cast<double>(value) - static_cast<double>(lo)) *
                              (2097151.999999 / static_cast<double>(extent));
        if (!(scaled > 0.0)) return 0;
        if (scaled >= static_cast<double>(kMaxCoord)) return kMaxCoord;
        return static_cast<uint32_t>(scaled);
    }

    static uint64_t mortonCode(const Vec3& p, const AABB& centroidBounds) {
        const Vec3 extent = centroidBounds.max - centroidBounds.min;
        return morton3D64(quantizeAxis(p.x, centroidBounds.min.x, extent.x),
                          quantizeAxis(p.y, centroidBounds.min.y, extent.y),
                          quantizeAxis(p.z, centroidBounds.min.z, extent.z));
    }

    bool intersectNode(int nodeIdx, const Vec3& origin, const Vec3& dir,
                       float maxDist, float& t) const {
        const Node& node = nodes_[static_cast<size_t>(nodeIdx)];
        return rayBoxEntry(origin, dir, node.bounds.min, node.bounds.max, maxDist, kVoxelEps, t);
    }

    template <typename Id>
    void appendSelfPair(std::vector<std::pair<Id, Id>>& out,
                        ObjectIndex idA,
                        ObjectIndex idB) const {
        static_assert(std::is_same_v<Id, ObjectIndex> || std::is_same_v<Id, ObjectId>,
                      "LinearBVH pair output ids must be ObjectIndex or ObjectId");
        if (idA == idB) return;
        if (idA > idB) std::swap(idA, idB);

        if constexpr (std::is_same_v<Id, ObjectId>) {
            out.emplace_back(objectIdForIndex(idA), objectIdForIndex(idB));
        } else {
            out.emplace_back(idA, idB);
        }
    }

    template <typename Id>
    void appendCrossPair(std::vector<std::pair<Id, Id>>& out,
                         const LinearBVH& other,
                         ObjectIndex idA,
                         ObjectIndex idB) const {
        static_assert(std::is_same_v<Id, ObjectIndex> || std::is_same_v<Id, ObjectId>,
                      "LinearBVH pair output ids must be ObjectIndex or ObjectId");
        if constexpr (std::is_same_v<Id, ObjectId>) {
            out.emplace_back(objectIdForIndex(idA), other.objectIdForIndex(idB));
        } else {
            out.emplace_back(idA, idB);
        }
    }

    template <typename Id>
    void collectPairs(int aIdx, int bIdx, std::vector<std::pair<Id, Id>>& out) const {
        static thread_local std::vector<std::pair<int, int>> stack;
        prepareScratchStack(stack);
        stack.push_back({aIdx, bIdx});

        while (!stack.empty()) {
            const auto entry = stack.back();
            stack.pop_back();

            const Node& a = nodes_[static_cast<size_t>(entry.first)];
            const Node& b = nodes_[static_cast<size_t>(entry.second)];
            if (!a.bounds.overlaps(b.bounds)) continue;

            if (entry.first == entry.second) {
                if (a.leaf()) continue;
                stack.push_back({a.right, a.right});
                stack.push_back({a.left, a.right});
                stack.push_back({a.left, a.left});
                continue;
            }

            if (a.leaf() && b.leaf()) {
                appendSelfPair(out, sortedIds_[a.start], sortedIds_[b.start]);
                continue;
            }

            if (a.leaf()) {
                stack.push_back({entry.first, b.right});
                stack.push_back({entry.first, b.left});
            } else if (b.leaf()) {
                stack.push_back({a.right, entry.second});
                stack.push_back({a.left, entry.second});
            } else {
                stack.push_back({a.right, b.right});
                stack.push_back({a.right, b.left});
                stack.push_back({a.left, b.right});
                stack.push_back({a.left, b.left});
            }
        }
    }

    template <typename Id>
    void collectPairsWith(int aIdx,
                          const LinearBVH& other,
                          int bIdx,
                          std::vector<std::pair<Id, Id>>& out) const {
        static thread_local std::vector<std::pair<int, int>> stack;
        prepareScratchStack(stack);
        stack.push_back({aIdx, bIdx});

        while (!stack.empty()) {
            const auto entry = stack.back();
            stack.pop_back();

            const Node& a = nodes_[static_cast<size_t>(entry.first)];
            const Node& b = other.nodes_[static_cast<size_t>(entry.second)];
            if (!a.bounds.overlaps(b.bounds)) continue;

            if (a.leaf() && b.leaf()) {
                appendCrossPair(out, other, sortedIds_[a.start], other.sortedIds_[b.start]);
                continue;
            }

            if (a.leaf()) {
                stack.push_back({entry.first, b.right});
                stack.push_back({entry.first, b.left});
            } else if (b.leaf()) {
                stack.push_back({a.right, entry.second});
                stack.push_back({a.left, entry.second});
            } else {
                stack.push_back({a.right, b.right});
                stack.push_back({a.right, b.left});
                stack.push_back({a.left, b.right});
                stack.push_back({a.left, b.left});
            }
        }
    }

    std::vector<ObjectSlot> objects_;
    std::vector<ObjectIndex> freeList_;
    size_t objectCount_ = 0;
    mutable std::vector<Node> nodes_;
    mutable std::vector<ObjectIndex> sortedIds_;
    mutable std::vector<int> parents_;
    mutable std::vector<int> leafForObject_;
    mutable std::vector<int> pendingRefitLeaves_;
    mutable DenseSet<int> pendingRefitLeafSet_;
    mutable bool dirty_ = false;
    mutable bool refitDirty_ = false;
    mutable size_t incrementalEditCount_ = 0;
    mutable size_t unqueriedInsertCount_ = 0;
    mutable float lastRebuildRootArea_ = 0.0f;

    static constexpr size_t kDeferredInsertBatchThreshold = 64;
};

}  // namespace utils

#endif  // UTILS_LINEAR_BVH_HPP
