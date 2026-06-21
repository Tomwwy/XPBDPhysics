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
- No same-body collision filter (contact generation) emitContact lambda does NOT check whether two different colliders belong to the **same body**. If attach two sphere colliders to one particle (a compound shape), they will generate spurious self-contacts with each other.
- Destroying a particle orphans its colliders and constraints, `destroy()` handles `EntityType::Particle` by simply calling `particles_.destroy(entity)`. It does NOT cascade to destroy the colliders and distance constraints that reference that particle. Consequences: **Colliders**: Their proxies are lazily removed in the next `refreshBroadphase()` (because `colliderWorldSphere` returns false when the body particle is dead), but the `Collider` entity itself lives forever in `colliders_`, leaking a slot. **Distance constraints**: They persist forever in `distanceConstraints_`. Each substep, the solver checks `world.particle(constraint.particleA)` and skips them, wasting iteration time.



#### 3. `renderMs` measurement is incorrect in demo

**`main.cpp:632` and `main.cpp:689`** — `renderMs` is measured as `(GetTime() - renderStart) * 1000.0`, but `renderStart` is obtained AFTER the simulation step, and `GetTime()` at line 689 is obtained AFTER `EndDrawing()`. However, `renderMs` from the **previous** frame is displayed (line 661), and it's only used for display. Minor, but the value shown is always one frame stale.

---

### 🟡 Design Issues / Improvements

#### 4. Contact generation recomputes world-sphere positions wastefully

`colliderWorldSphere()` is called for every collider in `refreshBroadphase()`, then AGAIN in `generateContacts()` for each candidate pair, then AGAIN per iteration in `solveContacts()`. The world-space sphere center and radius could be cached per-collider during `refreshBroadphase()` and reused. For ~750 particles with 4 substeps and 10 iterations, that's ~30,000 extra lookups per frame just for `solveContacts`.

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

1. **Same-body collision filter** (1-line fix, prevents real bugs with compound shapes)
2. **Cascading entity destruction** (proper cleanup on particle destroy)
3. **Contact warm starting** (better convergence for the same CPU budget)

Want me to implement any of these fixes?