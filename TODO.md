# TODO

Foundation refactor (bodies/colliders/broadphase + contacts-as-constraints)
is done — see DESIGN.md for the target architecture and the remaining
extension work (rigidbodies, tetra volume constraints + collision, more shapes).

Next up (tracked in DESIGN.md):
- RigidBody body type (orientation, inverse inertia) + integration
- Contact response dispatch onto rigidbodies (lever arm, angular correction)
- Friction in the contact solver
- Box / capsule / tetrahedron shapes + narrowphase dispatch matrix
- Tetrahedron volume constraint and volume-conserving collision



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
