#include "raylib.h"
#include "raymath.h"
#include "xpbd/xpbd_world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

constexpr int kScreenWidth = 1280;
constexpr int kScreenHeight = 720;

Vector3 toRaylib(const xpbd::Vec3& value)
{
    return {value.x, value.y, value.z};
}

struct ClothDemo {
    int columns = 34;
    int rows = 22;
    float spacing = 0.13f;
    std::vector<xpbd::Entity> particles;
    std::vector<xpbd::Entity> constraints;

    xpbd::Entity at(int row, int column) const
    {
        return particles[static_cast<std::size_t>(row * columns + column)];
    }
};

void addDistanceConstraint(xpbd::XPBDWorld& world,
                           ClothDemo& cloth,
                           xpbd::Entity particleA,
                           xpbd::Entity particleB,
                           float compliance)
{
    const xpbd::Particle* a = world.particle(particleA);
    const xpbd::Particle* b = world.particle(particleB);
    if (a == nullptr || b == nullptr) {
        return;
    }

    const float restLength = xpbd::distance(a->position, b->position);
    cloth.constraints.push_back(world.createDistanceConstraint(particleA, particleB, restLength, compliance));
}

void buildCloth(xpbd::XPBDWorld& world, ClothDemo& cloth)
{
    world.clearEntities();
    cloth.particles.clear();
    cloth.constraints.clear();
    cloth.particles.reserve(static_cast<std::size_t>(cloth.columns * cloth.rows));
    cloth.constraints.reserve(static_cast<std::size_t>(cloth.columns * cloth.rows * 6));

    const float startX = -0.5f * static_cast<float>(cloth.columns - 1) * cloth.spacing;
    const float topY = 2.25f;

    for (int row = 0; row < cloth.rows; ++row) {
        for (int column = 0; column < cloth.columns; ++column) {
            const float x = startX + static_cast<float>(column) * cloth.spacing;
            const float y = topY - static_cast<float>(row) * cloth.spacing;
            const float z = 0.04f * std::sin(static_cast<float>(column) * 0.5f);
            const bool pinned = row == 0 && (column % 3 == 0 || column == cloth.columns - 1);
            const float mass = pinned ? 0.0f : 1.0f;
            cloth.particles.push_back(world.createParticle({x, y, z}, mass, 0.018f));
        }
    }

    constexpr float kStructuralCompliance = 1e-7f;
    constexpr float kShearCompliance = 2e-6f;
    constexpr float kLongRangeCompliance = 2e-5f;

    for (int row = 0; row < cloth.rows; ++row) {
        for (int column = 0; column < cloth.columns; ++column) {
            if (column + 1 < cloth.columns) {
                addDistanceConstraint(world, cloth, cloth.at(row, column), cloth.at(row, column + 1), kStructuralCompliance);
            }
            if (row + 1 < cloth.rows) {
                addDistanceConstraint(world, cloth, cloth.at(row, column), cloth.at(row + 1, column), kStructuralCompliance);
            }
            if (row + 1 < cloth.rows && column + 1 < cloth.columns) {
                addDistanceConstraint(world, cloth, cloth.at(row, column), cloth.at(row + 1, column + 1), kShearCompliance);
            }
            if (row + 1 < cloth.rows && column > 0) {
                addDistanceConstraint(world, cloth, cloth.at(row, column), cloth.at(row + 1, column - 1), kShearCompliance);
            }
            if (column + 2 < cloth.columns) {
                addDistanceConstraint(world, cloth, cloth.at(row, column), cloth.at(row, column + 2), kLongRangeCompliance);
            }
            if (row + 2 < cloth.rows) {
                addDistanceConstraint(world, cloth, cloth.at(row, column), cloth.at(row + 2, column), kLongRangeCompliance);
            }
        }
    }
}

void drawCloth(xpbd::XPBDWorld& world)
{
    world.forEachDistanceConstraint([&world](xpbd::Entity, const xpbd::DistanceConstraint& constraint) {
        const xpbd::Particle* a = world.particle(constraint.particleA);
        const xpbd::Particle* b = world.particle(constraint.particleB);
        if (a == nullptr || b == nullptr) {
            return;
        }

        const bool longRange = constraint.restLength > 0.19f;
        const Color color = longRange ? Color{80, 90, 110, 120} : Color{165, 190, 210, 190};
        DrawLine3D(toRaylib(a->position), toRaylib(b->position), color);
    });

    world.forEachParticle([](xpbd::Entity, const xpbd::Particle& particle) {
        const bool pinned = particle.inverseMass <= 0.0f;
        const float radius = pinned ? particle.radius * 1.9f : particle.radius;
        const Color color = pinned ? Color{255, 115, 76, 255} : Color{74, 188, 255, 255};
        DrawSphere(toRaylib(particle.position), radius, color);
    });
}

}  // namespace

int main()
{
    xpbd::initializeSimd();

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
    InitWindow(kScreenWidth, kScreenHeight, "XPBD Physics");
    SetWindowMinSize(640, 360);
    SetTargetFPS(144);

    Camera3D camera = {};
    camera.position = {0.0f, 1.15f, 5.2f};
    camera.target = {0.0f, 0.85f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    xpbd::XPBDWorld world;
    world.setGravity({0.0f, -9.81f, 0.0f});
    world.setDamping(0.997f);
    world.setSubsteps(4);
    world.setSolverIterations(10);
    world.addIntegrationSystem(xpbd::XPBDWorld::verletIntegrationSystem);
    world.addConstraintSystem(xpbd::XPBDWorld::distanceConstraintSystem);

    ClothDemo cloth;
    buildCloth(world, cloth);

    bool paused = false;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_R)) {
            buildCloth(world, cloth);
        }
        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
        }

        UpdateCamera(&camera, CAMERA_ORBITAL);

        const float dt = std::min(GetFrameTime(), 1.0f / 30.0f);
        if (!paused) {
            world.step(dt);
        }

        BeginDrawing();
        ClearBackground({28, 30, 34, 255});

        BeginMode3D(camera);
        DrawGrid(24, 0.25f);
        drawCloth(world);
        EndMode3D();

        DrawFPS(10, 10);
        char stats[160] = {};
        std::snprintf(stats,
                      sizeof(stats),
                      "particles: %zu  distance constraints: %zu  simd: %s  %s",
                      world.particleCount(),
                      world.distanceConstraintCount(),
                      world.simdBackendName(),
                      paused ? "paused" : "running");
        DrawText(stats, 10, 34, 18, Color{210, 218, 226, 255});

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
