# XPBD Engine Design

This document describes the architecture of the XPBD world after the foundation
refactor, and the plan for the extensions it was built to support: rigid bodies,
additional collision shapes, and tetrahedral volume constraints + collision.

## 1. Design principles

The engine separates four concerns that used to be fused into `Particle` /
`CollisionSphere`. Keeping them orthogonal is what makes the system extensible:

| Axis | Question it answers | Type(s) |
|------|--------------------|---------|
| **Body** (DOFs) | What does the solver integrate and correct? | `Particle`, `RigidBody` (planned) |
| **Shape** (geometry) | What is the collision surface? | `utils::Shape` (POD: `Sphere`, …) |
| **Collider** (binding) | Which shape rides which body, with what filter, in which tree? | `Collider` |
| **Broadphase** (partition) | Which spatial structure holds the proxy? | `BroadphasePartition` → one BVH each |
| **Constraint** | What corrections are applied? | `DistanceConstraint`, `ContactConstraint`, … |

The rule that keeps it decoupled: **a collider points at a body; the collision
system resolves contacts and writes corrections back to the referenced body,
dispatching on body type.** Adding a shape = a POD struct + a narrowphase
function. Adding a body type = a store + integration + a contact-response
branch. Neither touches the others.

## 2. Layers vs partitions — two independent axes

A recurring temptation is to use one BVH per collision-layer bit. That conflates
two genuinely different axes:

- **Layer/mask (filtering)** — *who may I collide with?* Multi-bit set
  membership, bidirectional via `collisionFiltersMatch`. Lives on the collider
  as `layer` + `mask`, full 32 bits, entirely user-defined.
- **Partition (motion role)** — *which tree holds my proxy?* Single-valued,
  purely a performance concern (static geometry never refits). Lives on the
  collider as `BroadphasePartition` (`Static` | `Dynamic`).

Mapping a layer bit to a tree forces multi-bit objects to be duplicated across
trees, makes broadphase O(L²) in the number of layers, and needs a dedup set to
undo the duplication. Instead: **filtering is mask bits checked in narrowphase;
partitioning is a tiny enum selecting one of two BVHs.** Static still collides
with Dynamic; only Static×Static is skipped.

## 3. Current foundation (implemented)

### 3.1 Bodies

```cpp
struct Particle {            // pure point DOF
    Vec3 position, previousPosition, velocity, externalAcceleration;
    float inverseMass = 1.0f;
};
```

No radius, no layer, no mask — those moved to the collider.

### 3.2 Shapes

POD, no vtable, trivially copyable (`utils/shapes.hpp`):

```cpp
enum class ShapeType : uint8_t { None, Sphere /*, Box, Capsule, Tetra */ };
struct SphereShape { Vec3 center; float radius; };
struct Shape {                       // tagged union
    ShapeType type; union { SphereShape sphere; };
    static constexpr Shape makeSphere(const Vec3& c, float r);
};
AABB  shapeAabb(const Shape&, const Vec3& origin);   // proxy bounds
float shapeFeatureSize(const Shape&);
```

The polymorphic `utils::Collider` interface still exists, but only for the BVH's
standalone exact-raycast API. The physics broadphase never allocates one.

### 3.3 Collider

```cpp
struct Collider {
    Entity body;                 // Particle/RigidBody this follows (invalid => static)
    Shape  shape;                // local-space geometry
    Vec3   offset;               // added to body origin; the world placement when body-less
    CollisionLayerMask layer, mask;
    BroadphasePartition partition;
    bool enabled;
    utils::LinearBVH::ObjectId proxy;   // owns its broadphase handle directly
};
```

The single `proxy` field replaces the old parallel `refsByObject` array **and**
the `objectByEntity` map. The proxy's user-data payload packs the collider's
`Entity`, so a broadphase pair resolves straight back to the collider via
`bvh.userDataForId(id)`; the handle here drives `updateProxy` / `remove`.

### 3.4 Broadphase

`std::array<LinearBVH, 2>` indexed by `BroadphasePartition`. The BVH gained a
lightweight proxy path so dynamic colliders never allocate a `Collider` per
update:

```cpp
ObjectId insertProxy(const AABB& bounds, uint64_t userData = 0);
bool     updateProxy(ObjectId id, const AABB& bounds);
uint64_t userData(ObjectIndex index) const;     // from overlapPairs
uint64_t userDataForId(ObjectId id) const;
```

Proxies carry `collider == nullptr`, so `raycast()` skips them.

### 3.5 Contacts as constraints

Narrowphase emits transient `ContactConstraint`s into a per-substep buffer:

```cpp
struct ContactConstraint {
    Entity colliderA, colliderB;  // to re-evaluate geometry each iteration
    Entity bodyA, bodyB;          // to dispatch the correction (invalid => static)
    Vec3 normal;                  // unit, A -> B
    float compliance, lambda;     // XPBD accumulation within the substep
};
```

They are solved in the iteration loop with lambda accumulation, exactly like
`DistanceConstraint`. The solver dispatches the positional correction onto each
referenced body (today: `Particle`; the `RigidBody` branch slots in here).

### 3.6 Step loop

```
for each substep:
    integration systems (verlet, user forces)
    if collisions: refreshBroadphase(); generateContacts()   # once per substep
    reset lambdas
    for each iteration:
        constraint systems (distance, …)
        if collisions: solveContacts()                        # re-solved each iteration
    update velocities
```

Broadphase sync + narrowphase run once per substep against integrated positions;
the resulting contacts are re-solved every iteration alongside user constraints.

## 4. Extension: Rigid bodies (implemented)

A rigid body is **not** a collider and **not** a particle. It is a body type
with orientation and rotational inertia:

```cpp
struct RigidBody {
    Vec3 position, previousPosition, velocity;
    Quat orientation, previousOrientation;
    Vec3 angularVelocity;
    Vec3 externalForce, externalTorque;   // cleared after integration
    float inverseMass;
    Mat3 inverseInertiaLocal;   // body-space; world = R * I^-1 * R^T
};
```

Stored in its own `TypedStore<RigidBody>` (`include/xpbd/rigid_body.hpp`). All
four `*RigidBody` `EntityType`s share that store and reconstruct handles under a
canonical type for iteration; the per-instance creation type is preserved on the
handle the caller holds. The supporting math lives in `include/xpbd/quat.hpp`
(unit quaternion: normalize, Hamilton product, rotate, integrate orientation)
and `include/xpbd/mat3.hpp` (`toMat3`, `inverse`, `worldInverseInertia`).

A collider attached to a rigid body sets `body` to the rigid-body entity and
uses `offset` (a local-space point) for the attachment frame. Because
`BodyTransform::apply` now rotates before translating, vertex colliders rotate
with the body and the entire sphere narrowphase is reused unchanged.

### 4.1 Integration

`rigidBodyIntegrationSystem` advances rigid bodies: position via the same Verlet
scheme as particles (external force folded in through `inverseMass`), orientation
via quaternion integration of `angularVelocity` (external torque folded in
through the world inverse inertia), then re-normalize. Velocities are recovered
from the position/orientation deltas after the solve in
`updateRigidBodyVelocities`, mirroring `updateParticleVelocities` — angular
velocity comes from the relative rotation `previousOrientation -> orientation`.

### 4.2 Contact response dispatch

`solveContacts` dispatches the positional correction onto each referenced body.
A particle is a point mass (correction = `normal * dLambda * w`). For a rigid
body the generalized inverse mass along the contact normal `n` at lever arm `r`
(contact point relative to COM) is:

```
w = invMass + (r x n) . (Iinv_world * (r x n))     // generalizedInverseMass()
```

and the impulse rotates as well as translates the body:

```cpp
applyImpulse(body, impulse):
    Particle?  -> p.position    += impulse * invMass
    RigidBody? -> rb.position    += impulse * invMass
                  rb.orientation  = renormalize(rb.orientation + angularDelta)
                  // angularDelta from Iinv_world * (r x impulse)
```

`generateContacts` records the world contact point on each `ContactConstraint`
(`Contact::point` from the narrowphase), and `solveContacts` re-derives it from
the current pose each iteration so the lever arm tracks the bodies as they move.

### 4.3 Friction

Add a tangential constraint per contact, clamped by the Coulomb cone
(`|lambda_t| <= mu * lambda_n`), solved after the normal component each
iteration. Static vs dynamic friction follows the standard XPBD contact paper.

## 5. Extension: More shapes + narrowphase matrix

Each new `ShapeType` adds: (a) a POD struct, (b) `shapeAabb` / `shapeFeatureSize`
cases, (c) narrowphase functions against existing shapes. Narrowphase is a
dispatch on the ordered `(typeA, typeB)` pair:

| | Sphere | Box | Capsule | Tetra |
|--|--|--|--|--|
| **Sphere** | done | sphere-box | sphere-capsule | sphere-tetra |
| **Box** | | box-box (SAT) | ... | ... |
| **Capsule** | | | capsule-capsule | ... |
| **Tetra** | | | | tetra-tetra |

A contact is the canonical output of every cell: `(normal, points, penetration)`.
Keep the matrix lower-triangular by ordering the pair so `typeA <= typeB` and
swapping the resulting normal if needed — this halves the number of functions.
Unimplemented cells return "no contact" so partial support is safe.

The dispatcher lives next to `generateContacts`; broadphase is shape-agnostic
(it only sees AABBs), so adding shapes never touches the broadphase.

## 6. Extension: Tetrahedral volume constraints + collision

### 6.1 Volume constraint

A soft body is a set of particles plus tetrahedra over them. Each tetra is a
constraint entity (`EntityType::TetrahedronVolumeConstraint`) referencing four
particles:

```cpp
struct TetVolumeConstraint {
    Entity p0, p1, p2, p3;
    float restVolume;       // signed rest volume
    float compliance, lambda;
};
```

Solved as a standard XPBD volume constraint: `C = V(x) - restVolume`, with
gradients being the cross-product area-normals of the opposing faces, scaled by
1/6. Add a `tetVolumeConstraintSystem` registered like
`distanceConstraintSystem`. Edge length constraints (reuse `DistanceConstraint`)
provide shear/stretch stiffness alongside volume preservation.

### 6.2 Tetrahedral collision

Two routes, both fitting the existing foundation:

1. **Surface-particle colliders (cheap).** Attach sphere colliders to the
   surface particles of the tet mesh. Collision then reuses the existing
   sphere pipeline unchanged — no new narrowphase. Good first step.
2. **Tetra shape (accurate).** Add `ShapeType::Tetrahedron`; its proxy AABB is
   the vertex bound. Narrowphase does tetra-vs-X (SAT / closest-feature) and the
   contact correction is distributed to the four particles by barycentric
   weights of the contact point. This is volume-aware and avoids the "bag of
   spheres" artifacts.

Both register colliders in the broadphase exactly like spheres; only the
narrowphase cell differs.

## 7. Where to plug in — checklist

- **New shape:** `utils/shapes.hpp` (struct + `ShapeType` + factories +
  `shapeAabb`/`shapeFeatureSize`), then narrowphase cells in the dispatcher.
- **New body type:** `TypedStore` + `EntityType`, integration step,
  `applyCorrection` branch in the contact solver, accessor on the world.
- **New persistent constraint:** struct with `lambda`, a `*System` static
  method, register via `addConstraintSystem`, reset in `resetConstraintLambdas`.
- **New broadphase role:** extend `BroadphasePartition` (e.g. `Sensor`) and the
  pass matrix in `generateContacts`.

## 8. Performance notes

- Dynamic proxies update via `updateProxy` (AABB only) — no per-step heap
  allocation. The BVH refits incrementally and rebuilds on a cadence
  (`broadphaseRebuildInterval`) or when structurally degraded.
- One BVH per motion role (2 total) instead of 32-per-layer-bit: no object
  duplication, no O(L²) cross-pairing, no per-frame dedup set.
- Shapes are POD and trivially copyable, so narrowphase loops stay
  vectorizer-friendly and cache-dense.
- `Particle` SIMD integration is unaffected by the refactor: the removed
  fields (radius/layer/mask) were never read by the SIMD kernels.
