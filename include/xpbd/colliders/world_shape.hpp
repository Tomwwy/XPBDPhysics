#ifndef XPBD_WORLD_SHAPE_HPP
#define XPBD_WORLD_SHAPE_HPP

#include "utils/shapes.hpp"
#include "xpbd/math.hpp"

namespace xpbd {

using utils::Shape;
using utils::ShapeType;

// The world-space frame of the body a collider rides. Today a Particle supplies
// only a position; a RigidBody will add an orientation, at which point `apply()`
// becomes a full rigid transform and every caller below picks that up for free.
struct BodyTransform {
    Vec3 position{};  // body world origin (zero for a static / body-less collider)
    // Quat orientation;  // future: rotate local points before translating

    // Map a point expressed in the collider's local frame into world space.
    // The collider offset and the shape's local center are both local points,
    // so folding them through here keeps them correct once orientation lands.
    Vec3 apply(const Vec3& localPoint) const { return position + localPoint; }
};

// --- World-space shape results ---------------------------------------------
// One POD per ShapeType, produced by resolving a local Shape through a
// BodyTransform. These are what the narrowphase consumes.

struct WorldSphere {
    Vec3 center{};
    float radius = 0.0f;
};

// --- Shape traits -----------------------------------------------------------
// Maps a ShapeType to its world result and the geometry to get there. Adding a
// shape = a POD WorldX above + a WorldShapeTraits specialization here; callers
// that are written against the trait (worldShape<T>) need no changes.
template <ShapeType T>
struct WorldShapeTraits;  // primary left undefined: only valid for real shapes

template <>
struct WorldShapeTraits<ShapeType::Sphere> {
    using WorldType = WorldSphere;
    static constexpr ShapeType kType = ShapeType::Sphere;

    // Local-space validity, independent of any body (radius must be positive).
    static bool localValid(const Shape& shape) { return shape.sphere.radius > 0.0f; }

    static WorldType toWorld(const Shape& shape, const BodyTransform& xf, const Vec3& offset) {
        return WorldType{xf.apply(offset + shape.sphere.center), shape.sphere.radius};
    }
};

}  // namespace xpbd

#endif  // XPBD_WORLD_SHAPE_HPP
