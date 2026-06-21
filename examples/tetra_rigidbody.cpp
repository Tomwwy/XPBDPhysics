// Manual interactive test for tetrahedron rigid bodies, in the same spirit as
// xpbd_physics.exe (the cloth demo): build a world, drop bodies into it, and
// render with raylib so the contact response can be eyeballed.
//
// Scene: a field of tetrahedron rigid bodies tumbling onto a large static
// "ground" sphere (the ground reuses the sphere collision pipeline, so no new
// narrowphase is needed). Each tet is one RigidBody with four sphere colliders
// at its vertices (DESIGN.md 6.2, route 1).
//
// Controls:
//   Tab          toggle orbital / free-fly camera
//   W A S D Q E  fly (free-fly mode); hold right mouse to look; wheel = speed
//   Space        pause / resume
//   R            respawn the scene
//   C            toggle collisions
//   Up / Down    spawn more / fewer tetrahedra on respawn
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "camera_controller.hpp"
#include "xpbd/xpbd_world.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <random>
#include <vector>

namespace {

constexpr int kScreenWidth = 1280;
constexpr int kScreenHeight = 720;

constexpr float kGroundRadius = 60.0f;     // large sphere ~ a flat floor at y=0
constexpr float kVertexRadius = 0.14f;     // collision sphere at each tet vertex
constexpr float kTetScale = 0.55f;         // edge ~ 2 * scale
constexpr xpbd::CollisionLayerMask kTetLayer = xpbd::CollisionLayerMask{1u} << 0u;
constexpr xpbd::CollisionLayerMask kGroundLayer = xpbd::CollisionLayerMask{1u} << 1u;

Vector3 toRaylib(const xpbd::Vec3& v) { return {v.x, v.y, v.z}; }

// Canonical regular-tetrahedron vertices (centered at origin), scaled.
std::array<xpbd::Vec3, 4> tetLocalVertices(float scale)
{
    return {xpbd::Vec3{1, 1, 1} * scale,
            xpbd::Vec3{1, -1, -1} * scale,
            xpbd::Vec3{-1, 1, -1} * scale,
            xpbd::Vec3{-1, -1, 1} * scale};
}

// One tet instance: the body entity plus its rest-frame local vertices, so the
// renderer can transform them by the live pose each frame.
struct TetInstance {
    xpbd::Entity body;
    std::array<xpbd::Vec3, 4> localVertices;
    Color color;
};

struct Scene {
    int tetCount = 14;
    std::vector<TetInstance> tets;
    xpbd::Entity ground;
    std::mt19937 rng{0xC0FFEEu};
};

Color randomTetColor(std::mt19937& rng)
{
    static const std::array<Color, 6> palette = {
        Color{120, 200, 255, 255}, Color{255, 170, 90, 255}, Color{150, 230, 160, 255},
        Color{230, 140, 200, 255}, Color{255, 220, 110, 255}, Color{160, 175, 255, 255}};
    std::uniform_int_distribution<std::size_t> pick(0, palette.size() - 1);
    return palette[pick(rng)];
}

void buildScene(xpbd::XPBDWorld& world, Scene& scene)
{
    world.clearEntities();
    scene.tets.clear();

    scene.ground = world.createStaticCollider(
        xpbd::Shape::makeSphere(kGroundRadius),
        {0.0f, -kGroundRadius, 0.0f},
        kGroundLayer,
        kTetLayer);

    std::uniform_real_distribution<float> spread(-2.2f, 2.2f);
    std::uniform_real_distribution<float> height(2.5f, 7.5f);
    const std::array<xpbd::Vec3, 4> localVertices = tetLocalVertices(kTetScale);

    for (int i = 0; i < scene.tetCount; ++i) {
        const xpbd::Vec3 origin = {spread(scene.rng), height(scene.rng), spread(scene.rng)};
        // Place the canonical tet at this origin in world space; the builder
        // recenters on the centroid and stores vertex offsets as local space.
        xpbd::Vec3 worldVertices[4];
        for (int v = 0; v < 4; ++v) {
            worldVertices[v] = origin + localVertices[static_cast<std::size_t>(v)];
        }

        TetInstance instance;
        instance.body = world.createTetrahedronRigidBody(
            worldVertices, 1.0f, kVertexRadius, kTetLayer, kTetLayer | kGroundLayer);
        instance.localVertices = localVertices;  // centroid of canonical tet is origin
        instance.color = randomTetColor(scene.rng);
        scene.tets.push_back(instance);
    }
}

// World-space vertices of a tet for the current body pose.
std::array<xpbd::Vec3, 4> tetWorldVertices(const xpbd::XPBDWorld& world, const TetInstance& tet)
{
    std::array<xpbd::Vec3, 4> out{};
    const xpbd::RigidBody* rb = world.rigidBody(tet.body);
    if (rb == nullptr) {
        return out;
    }
    for (int v = 0; v < 4; ++v) {
        out[static_cast<std::size_t>(v)] =
            rb->position + xpbd::rotate(rb->orientation, tet.localVertices[static_cast<std::size_t>(v)]);
    }
    return out;
}

void drawTet(const std::array<xpbd::Vec3, 4>& v, Color fill, Color wire)
{
    static const int faces[4][3] = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    const Vector3 p[4] = {toRaylib(v[0]), toRaylib(v[1]), toRaylib(v[2]), toRaylib(v[3])};

    rlDisableBackfaceCulling();
    for (const auto& f : faces) {
        DrawTriangle3D(p[f[0]], p[f[1]], p[f[2]], fill);
    }
    rlEnableBackfaceCulling();

    // Edges.
    static const int edges[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    for (const auto& e : edges) {
        DrawLine3D(p[e[0]], p[e[1]], wire);
    }
    for (const Vector3& corner : p) {
        DrawSphereEx(corner, kVertexRadius, 6, 8, wire);
    }
}

void drawScene(const xpbd::XPBDWorld& world, const Scene& scene)
{
    for (const TetInstance& tet : scene.tets) {
        const std::array<xpbd::Vec3, 4> v = tetWorldVertices(world, tet);
        const Color wire = {static_cast<unsigned char>(std::min(255, tet.color.r + 40)),
                            static_cast<unsigned char>(std::min(255, tet.color.g + 40)),
                            static_cast<unsigned char>(std::min(255, tet.color.b + 40)),
                            255};
        drawTet(v, Color{tet.color.r, tet.color.g, tet.color.b, 150}, wire);
    }
}

}  // namespace

int main()
{
    xpbd::initializeSimd();

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(kScreenWidth, kScreenHeight, "XPBD Tetrahedron Rigid Bodies");
    SetWindowMinSize(640, 360);
    SetTargetFPS(144);

    examples::CameraController cameraController({0.0f, 3.5f, 9.0f}, {0.0f, 1.0f, 0.0f});

    xpbd::XPBDWorld world;
    world.setGravity({0.0f, -9.81f, 0.0f});
    world.setDamping(0.999f);
    world.setSubsteps(8);
    world.setSolverIterations(12);
    world.setCollisionsEnabled(true);
    world.setContactCompliance(0.0f);
    world.addIntegrationSystem(xpbd::XPBDWorld::rigidBodyIntegrationSystem);

    Scene scene;
    buildScene(world, scene);

    bool paused = false;
    double simMs = 0.0;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_R)) {
            buildScene(world, scene);
        }
        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
        }
        if (IsKeyPressed(KEY_C)) {
            world.setCollisionsEnabled(!world.collisionsEnabled());
        }
        if (IsKeyPressed(KEY_UP)) {
            scene.tetCount = std::min(scene.tetCount + 2, 80);
        }
        if (IsKeyPressed(KEY_DOWN)) {
            scene.tetCount = std::max(scene.tetCount - 2, 1);
        }

        const float frameDt = GetFrameTime();
        cameraController.update(frameDt);

        const float dt = std::min(frameDt, 1.0f / 30.0f);
        const double simStart = GetTime();
        if (!paused) {
            world.step(dt);
        }
        simMs = (GetTime() - simStart) * 1000.0;

        BeginDrawing();
        ClearBackground({26, 28, 33, 255});

        BeginMode3D(cameraController.camera());
        DrawGrid(40, 0.5f);
        drawScene(world, scene);
        EndMode3D();

        DrawFPS(10, 10);
        char stats[200] = {};
        std::snprintf(stats, sizeof(stats),
                      "tetrahedra: %zu  rigid bodies: %zu  colliders: %zu  contacts: %zu  simd: %s",
                      scene.tets.size(), world.rigidBodyCount(), world.colliderCount(),
                      world.contactCount(), world.simdBackendName());
        DrawText(stats, 10, 34, 18, Color{210, 218, 226, 255});
        char timings[200] = {};
        std::snprintf(timings, sizeof(timings),
                      "sim: %.2f ms  %s  collisions: %s  camera: %s",
                      simMs, paused ? "paused" : "running",
                      world.collisionsEnabled() ? "on" : "off",
                      cameraController.modeName());
        DrawText(timings, 10, 56, 18, Color{210, 218, 226, 255});
        DrawText("Tab camera  |  WASD/QE fly + RMB look  |  Space pause  |  R respawn  |  C collisions  |  Up/Down count",
                 10, 78, 18, Color{170, 180, 190, 255});

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
