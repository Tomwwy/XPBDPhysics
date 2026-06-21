// Standalone smoke test for the refactored XPBD collision foundation.
// No external test framework: returns non-zero on first failure.
#include "xpbd/xpbd_world.hpp"

#include <cmath>
#include <cstdio>

namespace {

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

// A dynamic particle resting above a static sphere should be pushed out so the
// two spheres are no longer interpenetrating.
void testDynamicVsStaticContact()
{
    xpbd::XPBDWorld world;
    world.setGravity({0.0f, -9.81f, 0.0f});
    world.setSubsteps(2);
    world.setSolverIterations(8);
    world.addIntegrationSystem(xpbd::XPBDWorld::verletIntegrationSystem);

    const xpbd::Entity ground = world.createStaticCollider(
        xpbd::Shape::makeSphere(1.0f), {0.0f, 0.0f, 0.0f});
    (void)ground;

    const xpbd::Entity p = world.createParticle({0.0f, 1.2f, 0.0f}, 1.0f);
    world.createCollider(p, xpbd::Shape::makeSphere(0.25f));

    for (int i = 0; i < 240; ++i) {
        world.step(1.0f / 120.0f);
    }

    const xpbd::Particle* particle = world.particle(p);
    check(particle != nullptr, "particle survived simulation");
    if (particle == nullptr) {
        return;
    }

    const float dist = std::sqrt(particle->position.x * particle->position.x +
                                 particle->position.y * particle->position.y +
                                 particle->position.z * particle->position.z);
    // Sum of radii is 1.25; particle must not sink appreciably below that.
    check(dist > 1.25f - 0.05f, "particle does not tunnel into static sphere");
    check(particle->position.y > 0.0f, "particle stays above origin");
}

// Two dynamic particles overlapping should separate; layer mismatch disables it.
void testDynamicPairAndFiltering()
{
    constexpr xpbd::CollisionLayerMask layerA = 1u << 0;
    constexpr xpbd::CollisionLayerMask layerB = 1u << 1;

    {
        xpbd::XPBDWorld world;
        world.setGravity({0.0f, 0.0f, 0.0f});  // isolate contact response
        world.addIntegrationSystem(xpbd::XPBDWorld::verletIntegrationSystem);

        const xpbd::Entity a = world.createParticle({-0.1f, 0.0f, 0.0f}, 1.0f);
        const xpbd::Entity b = world.createParticle({0.1f, 0.0f, 0.0f}, 1.0f);
        world.createCollider(a, xpbd::Shape::makeSphere(0.25f));
        world.createCollider(b, xpbd::Shape::makeSphere(0.25f));

        for (int i = 0; i < 60; ++i) {
            world.step(1.0f / 120.0f);
        }
        const float sep = world.particle(b)->position.x - world.particle(a)->position.x;
        check(sep > 0.45f, "matching-layer dynamic pair separates toward 2*radius");
    }

    {
        xpbd::XPBDWorld world;
        world.setGravity({0.0f, 0.0f, 0.0f});
        world.addIntegrationSystem(xpbd::XPBDWorld::verletIntegrationSystem);

        const xpbd::Entity a = world.createParticle({-0.1f, 0.0f, 0.0f}, 1.0f);
        const xpbd::Entity b = world.createParticle({0.1f, 0.0f, 0.0f}, 1.0f);
        // Disjoint layer/mask: A only sees A, B only sees B -> no contact.
        world.createCollider(a, xpbd::Shape::makeSphere(0.25f), layerA, layerA);
        world.createCollider(b, xpbd::Shape::makeSphere(0.25f), layerB, layerB);

        for (int i = 0; i < 60; ++i) {
            world.step(1.0f / 120.0f);
        }
        const float sep = world.particle(b)->position.x - world.particle(a)->position.x;
        check(approx(sep, 0.2f, 1e-3f), "filtered pair does not interact");
    }
}

// Multiple colliders on one particle are a compound shape, not independent
// bodies, so they must not generate self-contacts with each other.
void testSameBodyCollidersDoNotSelfCollide()
{
    xpbd::XPBDWorld world;
    world.setGravity({0.0f, 0.0f, 0.0f});
    world.setSubsteps(1);

    const xpbd::Entity p = world.createParticle({0.0f, 0.0f, 0.0f}, 1.0f);
    world.createCollider(p,
                         xpbd::Shape::makeSphere(0.25f),
                         xpbd::kCollisionLayerDefault,
                         xpbd::kCollisionLayerAll,
                         xpbd::BroadphasePartition::Dynamic,
                         {-0.1f, 0.0f, 0.0f});
    world.createCollider(p,
                         xpbd::Shape::makeSphere(0.25f),
                         xpbd::kCollisionLayerDefault,
                         xpbd::kCollisionLayerAll,
                         xpbd::BroadphasePartition::Dynamic,
                         {0.1f, 0.0f, 0.0f});

    world.step(1.0f / 60.0f);
    check(world.contactCount() == 0, "same-body colliders do not self-collide");
}

// Destroying a collider must remove its proxy without disturbing others.
void testColliderLifecycle()
{
    xpbd::XPBDWorld world;
    world.setGravity({0.0f, 0.0f, 0.0f});
    world.addIntegrationSystem(xpbd::XPBDWorld::verletIntegrationSystem);

    const xpbd::Entity p = world.createParticle({0.0f, 0.0f, 0.0f}, 1.0f);
    const xpbd::Entity c = world.createCollider(p, xpbd::Shape::makeSphere(0.25f));
    world.step(1.0f / 120.0f);
    check(world.colliderCount() == 1, "one collider registered");

    check(world.destroy(c), "collider destroyed");
    check(world.colliderCount() == 0, "collider count drops to zero");
    world.step(1.0f / 120.0f);  // must not crash / touch a dead proxy
    check(world.contactCount() == 0, "no contacts after collider removed");
}

// Destroying a particle invalidates its handle; referencing colliders and
// constraints are cleaned up lazily when their systems next resolve that handle.
void testParticleDestroyLazilyRemovesReferences()
{
    xpbd::XPBDWorld world;
    world.setGravity({0.0f, 0.0f, 0.0f});
    world.addConstraintSystem(xpbd::XPBDWorld::distanceConstraintSystem);

    const xpbd::Entity doomed = world.createParticle({0.0f, 0.0f, 0.0f}, 1.0f);
    const xpbd::Entity survivor = world.createParticle({2.0f, 0.0f, 0.0f}, 1.0f);
    const xpbd::Entity third = world.createParticle({3.0f, 0.0f, 0.0f}, 1.0f);

    const xpbd::Entity doomedCollider =
        world.createCollider(doomed, xpbd::Shape::makeSphere(0.25f));
    const xpbd::Entity survivorCollider =
        world.createCollider(survivor, xpbd::Shape::makeSphere(0.25f));
    const xpbd::Entity doomedConstraint =
        world.createDistanceConstraint(doomed, survivor, 2.0f);
    const xpbd::Entity survivorConstraint =
        world.createDistanceConstraint(survivor, third, 1.0f);

    world.step(1.0f / 120.0f);  // create broadphase proxies before destruction

    check(world.destroy(doomed), "particle destroyed");
    check(!world.alive(doomed), "destroyed particle is not alive");
    check(world.alive(doomedCollider), "referencing collider awaits lazy cleanup");
    check(world.alive(doomedConstraint), "referencing constraint awaits lazy cleanup");

    world.step(1.0f / 120.0f);  // resolves dead handles and cleans up references
    check(!world.alive(doomedCollider), "referencing collider destroyed");
    check(world.alive(survivorCollider), "unrelated collider survives");
    check(!world.alive(doomedConstraint), "referencing distance constraint destroyed");
    check(world.alive(survivorConstraint), "unrelated distance constraint survives");
    check(world.colliderCount() == 1, "only unrelated collider remains");
    check(world.distanceConstraintCount() == 1, "only unrelated distance constraint remains");

    world.step(1.0f / 120.0f);  // must not touch the removed collider proxy
    check(world.contactCount() == 0, "no contacts from destroyed particle references");
}

// Quaternion / Mat3 basics the rigid-body solver relies on.
void testRigidBodyMath()
{
    // Rotating a vector 90 deg about +Y maps +X to -Z (right-handed).
    const xpbd::Quat q = xpbd::Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, 3.14159265f * 0.5f);
    const xpbd::Vec3 r = xpbd::rotate(q, {1.0f, 0.0f, 0.0f});
    check(approx(r.x, 0.0f) && approx(r.y, 0.0f) && approx(r.z, -1.0f),
          "quaternion rotates +X about +Y to -Z");

    // toMat3 must agree with rotate().
    const xpbd::Vec3 m = xpbd::toMat3(q) * xpbd::Vec3{1.0f, 0.0f, 0.0f};
    check(approx(m.x, r.x) && approx(m.y, r.y) && approx(m.z, r.z),
          "rotation matrix matches quaternion rotate");

    // inverse(diagonal) is the reciprocal diagonal.
    const xpbd::Mat3 inv = xpbd::inverse(xpbd::Mat3::diagonal(2.0f, 4.0f, 8.0f));
    check(approx(inv.rows[0].x, 0.5f) && approx(inv.rows[1].y, 0.25f) && approx(inv.rows[2].z, 0.125f),
          "diagonal matrix inverse is reciprocal diagonal");
}

// A free rigid body under gravity falls like a point mass (no spurious spin).
void testRigidBodyFreeFall()
{
    xpbd::XPBDWorld world;
    world.setGravity({0.0f, -9.81f, 0.0f});
    world.setDamping(1.0f);  // undamped, so the drop matches 0.5 g t^2
    world.setSubsteps(4);
    world.addIntegrationSystem(xpbd::XPBDWorld::rigidBodyIntegrationSystem);

    const xpbd::Entity body = world.createRigidBody(
        {0.0f, 5.0f, 0.0f}, xpbd::Quat::identity(), 1.0f, xpbd::Mat3::identity());

    const float t = 1.0f;
    const int steps = 120;
    for (int i = 0; i < steps; ++i) {
        world.step(t / static_cast<float>(steps));
    }

    const xpbd::RigidBody* rb = world.rigidBody(body);
    check(rb != nullptr, "rigid body survived simulation");
    if (rb == nullptr) {
        return;
    }
    // Undamped Verlet free-fall over 1s ~ 0.5 g t^2 = 4.905. Allow a small margin.
    check(approx(rb->position.y, 5.0f - 4.905f, 0.1f), "rigid body falls under gravity ~0.5gt^2");
    const float spin = xpbd::length(rb->angularVelocity);
    check(spin < 1e-3f, "free fall introduces no spurious angular velocity");
}

// A tetrahedron rigid body dropped onto a static floor must come to rest above
// it (its lowest vertex sphere not tunneling through) rather than passing through.
void testTetrahedronRigidBodyRestsOnGround()
{
    xpbd::XPBDWorld world;
    world.setGravity({0.0f, -9.81f, 0.0f});
    world.setSubsteps(8);
    world.setSolverIterations(12);
    world.addIntegrationSystem(xpbd::XPBDWorld::rigidBodyIntegrationSystem);

    // Large static sphere as the ground (reuses the sphere pipeline).
    const float groundRadius = 20.0f;
    world.createStaticCollider(xpbd::Shape::makeSphere(groundRadius),
                               {0.0f, -groundRadius, 0.0f});

    const float vertexRadius = 0.12f;
    const xpbd::Vec3 verts[4] = {
        {0.0f, 1.6f, 0.0f},
        {0.5f, 1.0f, 0.0f},
        {-0.25f, 1.0f, 0.43f},
        {-0.25f, 1.0f, -0.43f},
    };
    const xpbd::Entity tet =
        world.createTetrahedronRigidBody(verts, 1.0f, vertexRadius);

    check(world.rigidBodyCount() == 1, "one rigid body created");
    check(world.colliderCount() == 5, "four vertex colliders + ground collider");

    for (int i = 0; i < 600; ++i) {
        world.step(1.0f / 120.0f);
    }

    const xpbd::RigidBody* rb = world.rigidBody(tet);
    check(rb != nullptr, "tetrahedron rigid body survived simulation");
    if (rb == nullptr) {
        return;
    }
    // Centroid must settle above the floor (y = 0) by at least a vertex radius
    // and must not have tunneled below it.
    check(rb->position.y > vertexRadius * 0.5f, "tetrahedron rests above the ground");
    check(rb->position.y < 1.5f, "tetrahedron actually fell from its start height");
    check(std::isfinite(rb->position.y), "tetrahedron pose stays finite");
}

}  // namespace

int main()
{
    testDynamicVsStaticContact();
    testDynamicPairAndFiltering();
    testSameBodyCollidersDoNotSelfCollide();
    testColliderLifecycle();
    testParticleDestroyLazilyRemovesReferences();
    testRigidBodyMath();
    testRigidBodyFreeFall();
    testTetrahedronRigidBodyRestsOnGround();

    if (gFailures == 0) {
        std::printf("All XPBD foundation tests passed.\n");
        return 0;
    }
    std::printf("%d test(s) failed.\n", gFailures);
    return 1;
}
