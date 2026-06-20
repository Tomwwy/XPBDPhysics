// utils/shapes.hpp -- POD collision shape catalog.
//
// Plain-old-data shapes used by physics collider components and to build
// broadphase proxy AABBs. Unlike utils::Collider (a polymorphic interface used
// for the BVH's standalone exact-raycast API), these carry no vtable, are
// trivially copyable, and are friendly to tight narrowphase loops and SIMD.
//
// To add a shape: add a POD struct, a ShapeType tag, a Shape::make* factory,
// and handle it in shapeAabb() (and your narrowphase dispatch).
#ifndef UTILS_SHAPES_HPP
#define UTILS_SHAPES_HPP

#include "math.hpp"

#include <cstdint>

namespace utils {

enum class ShapeType : std::uint8_t {
    None = 0,
    Sphere = 1,
    // Box, Capsule, Tetrahedron, ... (see design doc)
};

// Shape data is expressed in the owning collider's local frame; the world
// transform is applied by the collider when it computes a proxy AABB.
struct SphereShape {
    Vec3 center{};   // local-space center (usually origin for a point collider)
    float radius = 0.0f;
};

// Tagged union of POD shapes. Trivially copyable; default-constructs to None.
struct Shape {
    ShapeType type = ShapeType::None;
    union {
        SphereShape sphere;
    };

    constexpr Shape() : type(ShapeType::None), sphere{} {}

    static constexpr Shape makeSphere(const Vec3& center, float radius) {
        Shape s;
        s.type = ShapeType::Sphere;
        s.sphere = SphereShape{center, radius};
        return s;
    }

    static constexpr Shape makeSphere(float radius) {
        return makeSphere(Vec3{}, radius);
    }

    constexpr bool valid() const { return type != ShapeType::None; }
};

// World-space AABB for a shape whose local frame is translated by `origin`.
// (Rotation is a no-op for spheres; richer shapes will take a full transform.)
inline AABB shapeAabb(const Shape& shape, const Vec3& origin) {
    switch (shape.type) {
    case ShapeType::Sphere:
        return AABB::fromCenterRadius(origin + shape.sphere.center, shape.sphere.radius);
    case ShapeType::None:
    default:
        return AABB{origin, origin};
    }
}

// Largest extent of a shape; mirrors Collider::featureSize() for validation.
inline float shapeFeatureSize(const Shape& shape) {
    switch (shape.type) {
    case ShapeType::Sphere:
        return shape.sphere.radius * 2.0f;
    case ShapeType::None:
    default:
        return 0.0f;
    }
}

}  // namespace utils

#endif  // UTILS_SHAPES_HPP
