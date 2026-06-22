# TODO

Foundation refactor (bodies/colliders/broadphase + contacts-as-constraints)
is done — see DESIGN.md for the target architecture and the remaining
extension work (rigidbodies, tetra volume constraints + collision, more shapes).

Next up (tracked in DESIGN.md):
- RigidBody body type (orientation, inverse inertia) + integration — DONE
  (`include/xpbd/rigid_body.hpp` + `quat.hpp` + `mat3.hpp`;
  `rigidBodyIntegrationSystem` / `updateRigidBodyVelocities` in
  `src/xpbd/systems/rigid_body_integration_system.cpp`.)
- Contact response dispatch onto rigidbodies (lever arm, angular correction) — DONE
  (`solveContacts` dispatches per body type; `Contact`/`ContactConstraint` carry
  the world contact point. Manual demo: `examples/tetra_rigidbody.cpp`,
  built as `xpbd_tetra_rigidbody`.)
- Friction in the contact solver
- Box / capsule / tetrahedron shapes + narrowphase dispatch matrix
  (tetra rigid bodies currently use route-1 vertex spheres, DESIGN.md 6.2)
- Tetrahedron volume constraint and volume-conserving collision


- damping depends on substep? pow(damping, 1.0/substeps) ?
- refreshBroadphase() has unnecessary static bvh update? Maybe only update for entity which actually changed (Collider entity has something like "changed" bool)? 
- Contact warm-start, somehow record last substep lambda and re-use? Needs to also be fast (e.g. using a unordered_dense map?) 


- split narrow phase solver to different files, e.g. GJK, sphere vs sphere, sphere vs tetrahedron etc. — DONE
  (`include/xpbd/narrowphase/`: `contact.hpp` canonical result, `sphere_sphere.hpp`
  cell, `narrowphase.hpp` dispatch-matrix umbrella; new cells/GJK slot in here.)


- Maybe turn functions like colliderWorldSphere/colliderIsActiveAndIsSphere/computeColliderWorldSphere to function template to check if valid and return world space shapes? — DONE
  (`include/xpbd/colliders/world_shape.hpp`: `BodyTransform` + `WorldShapeTraits<ShapeType>`;
  `XPBDWorld::worldShape<T>()` returns `std::optional<WorldType>`. `BodyTransform::apply`
  is the single hook that becomes a full rigid transform when orientation lands.)


#### 6. `colliderWorldSphere` is sphere-specific but named generically — RESOLVED

Replaced by the templated `worldShape<T>()` + `WorldShapeTraits` above; adding a
shape is a trait specialization, not an edit to the world.

#### 7. `alive()` in TypedStore doesn't check the `alive_[]` flag

**`include/xpbd/typed_store.hpp:49-54`** — `alive()` validates only `index < size` and generation match. It does NOT verify `alive_[index] != 0`. The `alive_[]` array is used only for iteration (`forEachAlive`, SIMD dispatch). In practice this is safe because generation always bumps on destroy, so a stale handle can't match. But the dual bookkeeping is fragile — if a future change to `destroyAll()` or `release()` ever skips a generation bump, stale handles silently become valid. Consider either removing `alive_[]` entirely and computing aliveness from generation alone (slower iteration), or adding an `assert(alive_[index] != 0)` inside `alive()` for debug builds.

#### 8. No per-collider contact compliance

All contacts use the global `contactCompliance_`. Different material pairs (cloth vs metal, rubber vs rubber) ideally use the maximum or product of per-collider compliance values. Currently there's no per-collider compliance field, so all contacts are equally stiff.


#### 10. `static thread_local` pairs vector is a hidden shared state

**`src/xpbd/xpbd_world.cpp:303-304`** — The `pairs` vector is `static thread_local`. If someone calls `generateContacts` concurrently via different `XPBDWorld` instances on the same thread, they share this vector. Currently safe because `step()` is synchronous, but it's a trap for future parallelization.

#### 11. Tests aren't wired into CMake/CTest

**`tests/foundation_tests.cpp`** — The test binary is built but there's no `add_test()` or `enable_testing()` in CMakeLists.txt. Running tests requires manually executing the binary.


- simd math for quat and mat3 etc.
- similar simd integration for rigidbody (dedup with particle). Also distance constraints etc. can also support rigidbody, like particle. Basically if particle and rigidbody has similar parts, should de-dup, e.g. externalAcceleration and externalForce are basically the same thing. Maybe rigidbody stores a particle? (Probably shouldn't store ref, cause that's extra entity lookup in hotpath)
- remove different rigidbody types e.g. EntityType::SphereRigidBody, like particle there is only one EntityType::RigidBody