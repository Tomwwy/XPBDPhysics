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



- split narrow phase solver to different files, e.g. GJK, sphere vs sphere, sphere vs tetrahedron etc. 





---

### 🟡 Design Issues / Improvements

#### 4. Contact generation recomputes world-sphere positions wastefully

`colliderWorldSphere()` is called for every collider in `refreshBroadphase()`, then AGAIN in `generateContacts()` for each candidate pair, then AGAIN per iteration in `solveContacts()`. The world-space sphere center and radius could be cached per-collider during `refreshBroadphase()` and reused. For ~750 particles with 4 substeps and 10 iterations, that's ~30,000 extra lookups per frame just for `solveContacts`. (No, maybe they can't be cached, because constraints change the data like world position constantly? )

Maybe turn functions like colliderWorldSphere/colliderIsActiveAndIsSphere/computeColliderWorldSphere to function template to return world space shapes? 


#### 5. XPBD contacts lack warm starting

**`src/xpbd/xpbd_world.cpp:352`** — `contact.lambda = 0.0f` on every contact, every substep. Standard XPBD accumulates lambda across iterations within a substep (which this does), but warm starting persists lambda across substeps/frames using contact tracking. Without it, convergence is slower — you need more solver iterations to achieve the same stiffness.

#### 6. `colliderWorldSphere` is sphere-specific but named generically

**`include/xpbd/xpbd_world.hpp:68`** — The function name and signature suggest it computes a world-space collider bounding volume, but it hardcodes `shape.sphere` access. When Box/Capsule are added, this API will need to become either a tagged-union dispatch or a virtual method. Consider renaming it or making it dispatch on shape type now.

#### 7. `alive()` in TypedStore doesn't check the `alive_[]` flag

**`include/xpbd/typed_store.hpp:49-54`** — `alive()` validates only `index < size` and generation match. It does NOT verify `alive_[index] != 0`. The `alive_[]` array is used only for iteration (`forEachAlive`, SIMD dispatch). In practice this is safe because generation always bumps on destroy, so a stale handle can't match. But the dual bookkeeping is fragile — if a future change to `destroyAll()` or `release()` ever skips a generation bump, stale handles silently become valid. Consider either removing `alive_[]` entirely and computing aliveness from generation alone (slower iteration), or adding an `assert(alive_[index] != 0)` inside `alive()` for debug builds.

#### 8. No per-collider contact compliance

All contacts use the global `contactCompliance_`. Different material pairs (cloth vs metal, rubber vs rubber) ideally use the maximum or product of per-collider compliance values. Currently there's no per-collider compliance field, so all contacts are equally stiff.

#### 9. No collision response for edge case: sphere exactly touching

In `generateContacts()` (line 342):
```cpp
if (distSq >= minDistance * minDistance) {
    return;  // broadphase candidate, but not actually touching
}
```
This skips exactly-touching spheres (`distSq == minDistance * minDistance`). The contact constraint is only generated for penetrating spheres (`c < 0`). This is correct for XPBD (contacts only push apart, no resting contact force), but means there's zero normal force at the exact contact point, which can cause visible jitter. Once friction is added, this will matter more.

#### 10. `static thread_local` pairs vector is a hidden shared state

**`src/xpbd/xpbd_world.cpp:303-304`** — The `pairs` vector is `static thread_local`. If someone calls `generateContacts` concurrently via different `XPBDWorld` instances on the same thread, they share this vector. Currently safe because `step()` is synchronous, but it's a trap for future parallelization.

#### 11. Tests aren't wired into CMake/CTest

**`tests/foundation_tests.cpp`** — The test binary is built but there's no `add_test()` or `enable_testing()` in CMakeLists.txt. Running tests requires manually executing the binary.

---

### 🟢 Nitpicks

| File              | Line | Note                                                                                                                                                                                 |
| ----------------- | ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `typed_store.hpp` | 92   | `aliveCount()` recomputes from scratch every call; could maintain a counter                                                                                                          |
| `xpbd_world.cpp`  | 498  | `resetConstraintLambdas` iterates over ALL distance constraints even those with `lambda == 0`                                                                                        |
| `xpbd_world.cpp`  | 288  | `broadphaseNodes_ = 0; for (...) { broadphaseNodes_ += bvh.nodeCount(); }` — resets inside `refreshBroadphase` but overwritten at `step()` line 449 anyway                           |
| `xpbd_simd.cpp`   | 266  | `integrateFourSse2` stores to aligned stack arrays then scalar-writes back — defeats some of the SIMD benefit. AoS→SoA layout for particles would help, but that's a larger refactor |
| `shapes.hpp`      | 35   | Anonymous union + tagged type — technically UB when accessing inactive member, though universally supported                                                                          |
| `main.cpp`        | 106  | `meshNormal` calls `length()` which includes `sqrt`, then divides by it — could just use `normalized()` directly                                                                     |

---

### Top Priority Fixes

If I were to fix the most impactful issues, they'd be:

1. **Contact warm starting** (better convergence for the same CPU budget)

Want me to implement any of these fixes?
