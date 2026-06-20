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

}  // namespace

int main()
{
    testDynamicVsStaticContact();
    testDynamicPairAndFiltering();
    testColliderLifecycle();

    if (gFailures == 0) {
        std::printf("All XPBD foundation tests passed.\n");
        return 0;
    }
    std::printf("%d test(s) failed.\n", gFailures);
    return 1;
}
