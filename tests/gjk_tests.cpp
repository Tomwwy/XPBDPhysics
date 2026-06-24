// Standalone numeric tests for the GJK/EPA convex-convex narrowphase cell.
// No external test framework: returns non-zero on first failure, mirroring
// tests/foundation_tests.cpp.
//
// GJK/EPA is exercised through gjkEpaContact(), which folds each shape's
// rounding radius into the core query (see gjk.hpp). The cases cover all three
// branches: cleanly separated (no contact), separated cores with overlapping
// rounding shells (sphere/capsule fast path), and overlapping cores (EPA).
#include "xpbd/narrowphase/gjk.hpp"
#include "xpbd/quat.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <random>

namespace {

using namespace xpbd;
using namespace xpbd::narrowphase;

int gFailures = 0;

void check(bool condition, const char* message)
{
    if (!condition) {
        std::printf("FAIL: %s\n", message);
        ++gFailures;
    }
}

bool approx(float a, float b, float eps = 1e-3f)
{
    return std::fabs(a - b) <= eps;
}

bool approxVec(const Vec3& a, const Vec3& b, float eps = 2e-3f)
{
    return approx(a.x, b.x, eps) && approx(a.y, b.y, eps) && approx(a.z, b.z, eps);
}

// Eight corners of an axis-aligned box centered at `c` with half-extents `h`.
std::array<Vec3, 8> boxVerts(const Vec3& c, const Vec3& h)
{
    return {Vec3{c.x - h.x, c.y - h.y, c.z - h.z}, Vec3{c.x + h.x, c.y - h.y, c.z - h.z},
            Vec3{c.x + h.x, c.y + h.y, c.z - h.z}, Vec3{c.x - h.x, c.y + h.y, c.z - h.z},
            Vec3{c.x - h.x, c.y - h.y, c.z + h.z}, Vec3{c.x + h.x, c.y - h.y, c.z + h.z},
            Vec3{c.x + h.x, c.y + h.y, c.z + h.z}, Vec3{c.x - h.x, c.y + h.y, c.z + h.z}};
}

// --- Sphere vs sphere: validate GJK/EPA against the closed-form answer -------

// Two separated spheres produce no contact.
void testSphereSphereSeparated()
{
    GjkSphere a{{0.0f, 0.0f, 0.0f}, 1.0f};
    GjkSphere b{{3.0f, 0.0f, 0.0f}, 1.0f};  // gap of 1.0 between surfaces
    const Contact c = gjkEpaContact(a, b);
    check(!c.touching, "separated spheres do not touch");
}

// Overlapping spheres: depth and normal match the analytic sphere-sphere result.
void testSphereSpherePenetrating()
{
    GjkSphere a{{0.0f, 0.0f, 0.0f}, 1.0f};
    GjkSphere b{{1.5f, 0.0f, 0.0f}, 1.0f};  // centers 1.5 apart, radii sum 2.0
    const Contact c = gjkEpaContact(a, b);
    check(c.touching, "overlapping spheres touch");
    check(approx(c.penetration, 0.5f), "sphere-sphere penetration is rSum - dist");
    check(approxVec(c.normal, {1.0f, 0.0f, 0.0f}), "sphere-sphere normal points A -> B");
    // Contact point sits on the midline: A surface at x=1, B surface at x=0.5.
    check(approxVec(c.point, {0.75f, 0.0f, 0.0f}), "sphere-sphere contact point on overlap midline");
}

// Concentric spheres (cores coincident) still yield a stable unit normal + depth.
void testSphereSphereConcentric()
{
    GjkSphere a{{0.0f, 0.0f, 0.0f}, 1.0f};
    GjkSphere b{{0.0f, 0.0f, 0.0f}, 0.5f};
    const Contact c = gjkEpaContact(a, b);
    check(c.touching, "concentric spheres touch");
    check(approx(c.penetration, 1.5f, 1e-2f), "concentric depth is sum of radii");
    check(approx(length(c.normal), 1.0f), "concentric normal is unit length");
}

// --- Box vs box (pure polytopes, radius 0): exercises EPA on flat faces ------

void testBoxBoxFaceContact()
{
    const std::array<Vec3, 8> va = boxVerts({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    // Overlap of 0.2 along +x: B spans x in [0.8, 2.8].
    const std::array<Vec3, 8> vb = boxVerts({1.8f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    GjkHull a{va.data(), 8};
    GjkHull b{vb.data(), 8};
    const Contact c = gjkEpaContact(a, b);
    check(c.touching, "overlapping boxes touch");
    check(approx(c.penetration, 0.2f, 5e-3f), "box-box face penetration equals overlap");
    // Minimum-translation axis is +x (the shallow overlap direction).
    check(approx(std::fabs(c.normal.x), 1.0f, 5e-3f), "box-box normal is the min-overlap axis (x)");
    check(approx(c.normal.y, 0.0f, 5e-3f) && approx(c.normal.z, 0.0f, 5e-3f),
          "box-box normal has no y/z component");
}

void testBoxBoxSeparated()
{
    const std::array<Vec3, 8> va = boxVerts({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    const std::array<Vec3, 8> vb = boxVerts({2.5f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});  // gap 0.5
    GjkHull a{va.data(), 8};
    GjkHull b{vb.data(), 8};
    const Contact c = gjkEpaContact(a, b);
    check(!c.touching, "separated boxes do not touch");
}

// Diagonal corner-to-corner overlap: the min-translation axis is still a face
// axis, not the diagonal (boxes separate along the shallowest axis).
void testBoxBoxCornerOverlap()
{
    const std::array<Vec3, 8> va = boxVerts({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    const std::array<Vec3, 8> vb = boxVerts({1.9f, 1.9f, 0.0f}, {1.0f, 1.0f, 1.0f});
    GjkHull a{va.data(), 8};
    GjkHull b{vb.data(), 8};
    const Contact c = gjkEpaContact(a, b);
    check(c.touching, "corner-overlapping boxes touch");
    check(approx(c.penetration, 0.1f, 1e-2f), "corner overlap depth is the shallow-axis overlap");
}

// --- Box vs sphere: mixed polytope + rounded core ---------------------------

// Sphere pressing on a box face: depth and normal come from the face it hits.
void testBoxSphereFace()
{
    const std::array<Vec3, 8> va = boxVerts({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    GjkHull box{va.data(), 8};
    // Sphere center 1.4 to the right of origin: box wall at x=1, sphere radius
    // 0.5 reaches to x=0.9, overlapping the wall by 0.1.
    GjkSphere sphere{{1.4f, 0.0f, 0.0f}, 0.5f};
    const Contact c = gjkEpaContact(box, sphere);
    check(c.touching, "box and sphere overlap");
    check(approx(c.penetration, 0.1f, 5e-3f), "box-sphere penetration matches gap");
    check(approxVec(c.normal, {1.0f, 0.0f, 0.0f}, 5e-3f), "box-sphere normal points box -> sphere");
}

// Sphere just outside a box corner: separated cores, no rounding overlap.
void testBoxSphereSeparated()
{
    const std::array<Vec3, 8> va = boxVerts({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    GjkHull box{va.data(), 8};
    GjkSphere sphere{{2.0f, 2.0f, 2.0f}, 0.5f};  // corner at (1,1,1), dist ~1.73 > 0.5
    const Contact c = gjkEpaContact(box, sphere);
    check(!c.touching, "sphere clear of box corner does not touch");
}

// --- Capsule: segment core + radius -----------------------------------------

// Two parallel capsules side by side: closest features are the core segments.
void testCapsuleCapsule()
{
    GjkCapsule a{{0.0f, -1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.5f};
    GjkCapsule b{{0.8f, -1.0f, 0.0f}, {0.8f, 1.0f, 0.0f}, 0.5f};  // axes 0.8 apart
    const Contact c = gjkEpaContact(a, b);
    check(c.touching, "parallel capsules overlap");
    check(approx(c.penetration, 0.2f, 5e-3f), "capsule-capsule depth is rSum - axis gap");
    check(approxVec(c.normal, {1.0f, 0.0f, 0.0f}, 5e-3f), "capsule-capsule normal is perpendicular to axes");
}

// A capsule resting against a box face along its length.
void testCapsuleBox()
{
    const std::array<Vec3, 8> va = boxVerts({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    GjkHull box{va.data(), 8};
    // Vertical capsule whose axis is at x = 1.4, radius 0.5 -> overlaps the wall by 0.1.
    GjkCapsule cap{{1.4f, -0.5f, 0.0f}, {1.4f, 0.5f, 0.0f}, 0.5f};
    const Contact c = gjkEpaContact(box, cap);
    check(c.touching, "capsule and box overlap");
    check(approx(c.penetration, 0.1f, 5e-3f), "capsule-box penetration matches gap");
    check(approxVec(c.normal, {1.0f, 0.0f, 0.0f}, 5e-3f), "capsule-box normal points box -> capsule");
}

// --- Tetrahedron (4-vertex hull): EPA on a non-box polytope ------------------

void testTetraTetra()
{
    const Vec3 ta[4] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    // Second tetra shifted so it overlaps the first near the origin corner.
    const Vec3 tb[4] = {{0.2f, 0.2f, 0.2f}, {1.2f, 0.2f, 0.2f}, {0.2f, 1.2f, 0.2f}, {0.2f, 0.2f, 1.2f}};
    GjkHull a{ta, 4};
    GjkHull b{tb, 4};
    const Contact c = gjkEpaContact(a, b);
    check(c.touching, "overlapping tetrahedra touch");
    check(c.penetration > 0.0f && std::isfinite(c.penetration), "tetra-tetra depth is positive and finite");
    check(approx(length(c.normal), 1.0f), "tetra-tetra normal is unit length");
}

void testTetraSeparated()
{
    const Vec3 ta[4] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const Vec3 tb[4] = {{5, 5, 5}, {6, 5, 5}, {5, 6, 5}, {5, 5, 6}};
    GjkHull a{ta, 4};
    GjkHull b{tb, 4};
    const Contact c = gjkEpaContact(a, b);
    check(!c.touching, "distant tetrahedra do not touch");
}

// --- Invariants -------------------------------------------------------------

// Swapping the operands flips the normal but preserves the depth (A -> B vs B -> A).
void testNormalAntisymmetry()
{
    const std::array<Vec3, 8> va = boxVerts({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    const std::array<Vec3, 8> vb = boxVerts({1.8f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    GjkHull a{va.data(), 8};
    GjkHull b{vb.data(), 8};
    const Contact ab = gjkEpaContact(a, b);
    const Contact ba = gjkEpaContact(b, a);
    check(ab.touching && ba.touching, "both orderings report contact");
    check(approx(ab.penetration, ba.penetration, 5e-3f), "penetration is order-independent");
    check(approxVec(ab.normal, ba.normal * -1.0f, 5e-3f), "swapping operands flips the normal");
}

// The contact point lies between the two surfaces along the normal.
void testContactPointBetweenSurfaces()
{
    GjkSphere a{{0.0f, 0.0f, 0.0f}, 1.0f};
    GjkSphere b{{1.5f, 0.0f, 0.0f}, 1.0f};
    const Contact c = gjkEpaContact(a, b);
    check(c.point.x > 0.5f && c.point.x < 1.0f, "contact point lies inside the overlap region");
}

// --- Rotated-hull robustness (regression for the reduceSimplex tetra bug) ----
//
// A box vs a point/segment generates flat (near-coplanar) Minkowski simplices,
// and the GJK tetrahedron test used to (a) only check the 3 faces touching the
// newest vertex and (b) ignore degeneracy. Both made it report the origin as
// enclosed when the cores were actually separated, so GJK falsely flagged
// intersection and EPA returned a garbage normal/penetration on shapes that
// were not touching. These randomized cases drive a rotated box against a
// sphere and check the analytic answer in the box's local frame.

std::array<Vec3, 8> rotatedBox(const Vec3& c, float h, const Quat& q)
{
    const std::array<Vec3, 8> local = {
        Vec3{-h, -h, -h}, Vec3{h, -h, -h}, Vec3{h, h, -h}, Vec3{-h, h, -h},
        Vec3{-h, -h, h},  Vec3{h, -h, h},  Vec3{h, h, h},  Vec3{-h, h, h}};
    std::array<Vec3, 8> world{};
    for (std::size_t i = 0; i < 8; ++i) world[i] = c + rotate(q, local[i]);
    return world;
}

// Sphere centered OUTSIDE a rotated box face (nearest feature is the face):
// separated cores. With a small enough rounding shell the pair must not touch,
// and when it does touch the normal is the rotated face axis.
void testRotatedBoxSphereSeparated()
{
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> U(-1.0f, 1.0f), Pos(0.0f, 1.0f);
    const float h = 0.8f, rb = 0.3f;
    int falsePositives = 0, wrongNormals = 0;

    for (int t = 0; t < 5000; ++t) {
        const Vec3 axis = normalized(Vec3{U(rng), U(rng), U(rng)} + Vec3{0, 0, 1e-3f});
        const Quat q = Quat::fromAxisAngle(axis, U(rng) * 3.14159f);
        const Vec3 ca{U(rng) * 0.5f, U(rng) * 0.5f, U(rng) * 0.5f};
        const auto verts = rotatedBox(ca, h, q);
        GjkHull box{verts.data(), 8};

        // Center beyond the +local-x face by `gap`, with y/z inside the face.
        const float gap = 0.05f + Pos(rng) * 0.6f;
        const Vec3 center = ca + rotate(q, Vec3{h + gap, U(rng) * 0.5f, U(rng) * 0.5f});
        GjkSphere sphere{center, rb};
        const Contact c = gjkEpaContact(box, sphere);

        const bool shouldTouch = gap < rb;  // box rounding radius is 0
        if (c.touching != shouldTouch) ++falsePositives;
        if (c.touching) {
            const Vec3 trueN = rotate(q, Vec3{1.0f, 0.0f, 0.0f});
            if (dot(normalized(c.normal), trueN) < 0.95f) ++wrongNormals;
        }
    }
    check(falsePositives == 0, "rotated box vs external sphere: no false contacts");
    check(wrongNormals == 0, "rotated box vs external sphere: normal is the face axis");
}

// Sphere centered INSIDE a rotated box: overlapping cores. The reported normal
// must actually separate the pair (push the sphere out by depth*normal and the
// shapes must come apart).
void testRotatedBoxSphereOverlap()
{
    std::mt19937 rng(67890);
    std::uniform_real_distribution<float> U(-1.0f, 1.0f);
    const float h = 0.8f, rb = 0.3f;
    int badSeparations = 0;

    for (int t = 0; t < 5000; ++t) {
        const Vec3 axis = normalized(Vec3{U(rng), U(rng), U(rng)} + Vec3{0, 0, 1e-3f});
        const Quat q = Quat::fromAxisAngle(axis, U(rng) * 3.14159f);
        const Vec3 ca{U(rng) * 0.5f, U(rng) * 0.5f, U(rng) * 0.5f};
        const auto verts = rotatedBox(ca, h, q);
        GjkHull box{verts.data(), 8};

        const Vec3 center = ca + rotate(q, Vec3{U(rng) * 0.6f, U(rng) * 0.6f, U(rng) * 0.6f});
        GjkSphere sphere{center, rb};
        const Contact c = gjkEpaContact(box, sphere);
        if (!c.touching) continue;

        // Push the sphere out along the contact normal by the full depth + slack.
        GjkSphere pushed{center + c.normal * (c.penetration + 0.05f), rb};
        const Contact after = gjkEpaContact(box, pushed);
        const float residual = after.touching ? after.penetration : 0.0f;
        if (residual > 0.1f) ++badSeparations;
    }
    check(badSeparations == 0, "rotated box vs inside sphere: normal separates the pair");
}

}  // namespace

int main()
{
    testSphereSphereSeparated();
    testSphereSpherePenetrating();
    testSphereSphereConcentric();
    testBoxBoxFaceContact();
    testBoxBoxSeparated();
    testBoxBoxCornerOverlap();
    testBoxSphereFace();
    testBoxSphereSeparated();
    testCapsuleCapsule();
    testCapsuleBox();
    testTetraTetra();
    testTetraSeparated();
    testNormalAntisymmetry();
    testContactPointBetweenSurfaces();
    testRotatedBoxSphereSeparated();
    testRotatedBoxSphereOverlap();

    if (gFailures == 0) {
        std::printf("All GJK/EPA tests passed.\n");
        return 0;
    }
    std::printf("%d GJK/EPA test(s) failed.\n", gFailures);
    return 1;
}
