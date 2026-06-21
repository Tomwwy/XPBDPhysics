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