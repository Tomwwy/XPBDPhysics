#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "xpbd/xpbd_world.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

constexpr int kScreenWidth = 1280;
constexpr int kScreenHeight = 720;
constexpr int kMeshVertexBufferPosition = 0;
constexpr int kMeshVertexBufferNormal = 2;

Vector3 toRaylib(const xpbd::Vec3& value)
{
    return {value.x, value.y, value.z};
}

struct ClothDemo {
    int columns = 34;
    int rows = 22;
    float spacing = 0.13f;
    std::vector<xpbd::Entity> particles;
    std::vector<xpbd::Entity> pinnedParticles;
    std::vector<xpbd::Entity> constraints;

    xpbd::Entity at(int row, int column) const
    {
        return particles[static_cast<std::size_t>(row * columns + column)];
    }
};

struct ClothRenderer {
    Mesh mesh = {};
    Material clothMaterial = {};
    Material wireMaterial = {};
    bool ready = false;
    int columns = 0;
    int rows = 0;
};

Color lerpColor(Color from, Color to, float amount)
{
    const auto lerpChannel = [amount](unsigned char a, unsigned char b) {
        const float value = static_cast<float>(a) + (static_cast<float>(b) - static_cast<float>(a)) * amount;
        return static_cast<unsigned char>(std::max(0.0f, std::min(255.0f, value)));
    };

    return {lerpChannel(from.r, to.r),
            lerpChannel(from.g, to.g),
            lerpChannel(from.b, to.b),
            lerpChannel(from.a, to.a)};
}

void setVertexColor(Mesh& mesh, int vertexIndex, Color color)
{
    unsigned char* colorPtr = mesh.colors + vertexIndex * 4;
    colorPtr[0] = color.r;
    colorPtr[1] = color.g;
    colorPtr[2] = color.b;
    colorPtr[3] = color.a;
}

xpbd::Vec3 meshVertex(const Mesh& mesh, int vertexIndex)
{
    const float* vertex = mesh.vertices + vertexIndex * 3;
    return {vertex[0], vertex[1], vertex[2]};
}

xpbd::Vec3 meshNormal(const Mesh& mesh, int vertexIndex)
{
    const float* normal = mesh.normals + vertexIndex * 3;
    return {normal[0], normal[1], normal[2]};
}

void addNormal(Mesh& mesh, int vertexIndex, const xpbd::Vec3& normal)
{
    float* normalPtr = mesh.normals + vertexIndex * 3;
    normalPtr[0] += normal.x;
    normalPtr[1] += normal.y;
    normalPtr[2] += normal.z;
}

void setNormal(Mesh& mesh, int vertexIndex, const xpbd::Vec3& normal)
{
    float* normalPtr = mesh.normals + vertexIndex * 3;
    normalPtr[0] = normal.x;
    normalPtr[1] = normal.y;
    normalPtr[2] = normal.z;
}

void updateClothMesh(ClothRenderer& renderer,
                     const xpbd::XPBDWorld& world,
                     const ClothDemo& cloth,
                     bool uploadToGpu)
{
    if (renderer.mesh.vertices == nullptr || renderer.mesh.normals == nullptr) {
        return;
    }

    const int vertexCount = renderer.columns * renderer.rows;
    const int vertexDataSize = vertexCount * 3 * static_cast<int>(sizeof(float));

    for (int index = 0; index < vertexCount; ++index) {
        const xpbd::Particle* particle = index < static_cast<int>(cloth.particles.size())
            ? world.particle(cloth.particles[static_cast<std::size_t>(index)])
            : nullptr;
        const xpbd::Vec3 position = particle != nullptr ? particle->position : xpbd::Vec3{};

        float* vertex = renderer.mesh.vertices + index * 3;
        vertex[0] = position.x;
        vertex[1] = position.y;
        vertex[2] = position.z;
    }

    std::memset(renderer.mesh.normals, 0, static_cast<std::size_t>(vertexDataSize));

    for (int triangle = 0; triangle < renderer.mesh.triangleCount; ++triangle) {
        const unsigned short indexA = renderer.mesh.indices[triangle * 3 + 0];
        const unsigned short indexB = renderer.mesh.indices[triangle * 3 + 1];
        const unsigned short indexC = renderer.mesh.indices[triangle * 3 + 2];
        const xpbd::Vec3 a = meshVertex(renderer.mesh, indexA);
        const xpbd::Vec3 b = meshVertex(renderer.mesh, indexB);
        const xpbd::Vec3 c = meshVertex(renderer.mesh, indexC);
        const xpbd::Vec3 faceNormal = xpbd::cross(b - a, c - a);

        addNormal(renderer.mesh, indexA, faceNormal);
        addNormal(renderer.mesh, indexB, faceNormal);
        addNormal(renderer.mesh, indexC, faceNormal);
    }

    for (int index = 0; index < vertexCount; ++index) {
        const xpbd::Vec3 normal = meshNormal(renderer.mesh, index);
        const float normalLength = xpbd::length(normal);
        setNormal(renderer.mesh, index, normalLength > 1e-5f ? normal / normalLength : xpbd::Vec3{0.0f, 0.0f, 1.0f});
    }

    if (uploadToGpu && renderer.ready) {
        UpdateMeshBuffer(renderer.mesh, kMeshVertexBufferPosition, renderer.mesh.vertices, vertexDataSize, 0);
        UpdateMeshBuffer(renderer.mesh, kMeshVertexBufferNormal, renderer.mesh.normals, vertexDataSize, 0);
    }
}

void unloadClothRenderer(ClothRenderer& renderer)
{
    if (renderer.mesh.vertices != nullptr || renderer.mesh.vboId != nullptr) {
        UnloadMesh(renderer.mesh);
    }
    if (renderer.clothMaterial.maps != nullptr) {
        UnloadMaterial(renderer.clothMaterial);
    }
    if (renderer.wireMaterial.maps != nullptr) {
        UnloadMaterial(renderer.wireMaterial);
    }

    renderer = {};
}

bool initializeClothRenderer(ClothRenderer& renderer, const xpbd::XPBDWorld& world, const ClothDemo& cloth)
{
    unloadClothRenderer(renderer);

    renderer.columns = cloth.columns;
    renderer.rows = cloth.rows;

    const int vertexCount = renderer.columns * renderer.rows;
    const int triangleCount = (renderer.columns - 1) * (renderer.rows - 1) * 2;
    const int vertexDataSize = vertexCount * 3 * static_cast<int>(sizeof(float));
    const int colorDataSize = vertexCount * 4 * static_cast<int>(sizeof(unsigned char));
    const int indexDataSize = triangleCount * 3 * static_cast<int>(sizeof(unsigned short));

    renderer.mesh.vertexCount = vertexCount;
    renderer.mesh.triangleCount = triangleCount;
    renderer.mesh.vertices = static_cast<float*>(MemAlloc(static_cast<unsigned int>(vertexDataSize)));
    renderer.mesh.normals = static_cast<float*>(MemAlloc(static_cast<unsigned int>(vertexDataSize)));
    renderer.mesh.colors = static_cast<unsigned char*>(MemAlloc(static_cast<unsigned int>(colorDataSize)));
    renderer.mesh.indices = static_cast<unsigned short*>(MemAlloc(static_cast<unsigned int>(indexDataSize)));

    if (renderer.mesh.vertices == nullptr ||
        renderer.mesh.normals == nullptr ||
        renderer.mesh.colors == nullptr ||
        renderer.mesh.indices == nullptr) {
        unloadClothRenderer(renderer);
        return false;
    }

    int triangle = 0;
    for (int row = 0; row + 1 < renderer.rows; ++row) {
        for (int column = 0; column + 1 < renderer.columns; ++column) {
            const unsigned short topLeft = static_cast<unsigned short>(row * renderer.columns + column);
            const unsigned short topRight = static_cast<unsigned short>(row * renderer.columns + column + 1);
            const unsigned short bottomLeft = static_cast<unsigned short>((row + 1) * renderer.columns + column);
            const unsigned short bottomRight = static_cast<unsigned short>((row + 1) * renderer.columns + column + 1);

            renderer.mesh.indices[triangle * 3 + 0] = topLeft;
            renderer.mesh.indices[triangle * 3 + 1] = bottomLeft;
            renderer.mesh.indices[triangle * 3 + 2] = topRight;
            ++triangle;

            renderer.mesh.indices[triangle * 3 + 0] = topRight;
            renderer.mesh.indices[triangle * 3 + 1] = bottomLeft;
            renderer.mesh.indices[triangle * 3 + 2] = bottomRight;
            ++triangle;
        }
    }

    const Color topColor = {120, 218, 255, 255};
    const Color bottomColor = {39, 134, 202, 255};
    for (int row = 0; row < renderer.rows; ++row) {
        const float amount = renderer.rows > 1 ? static_cast<float>(row) / static_cast<float>(renderer.rows - 1) : 0.0f;
        const Color rowColor = lerpColor(topColor, bottomColor, amount);
        for (int column = 0; column < renderer.columns; ++column) {
            setVertexColor(renderer.mesh, row * renderer.columns + column, rowColor);
        }
    }

    updateClothMesh(renderer, world, cloth, false);
    UploadMesh(&renderer.mesh, true);

    renderer.clothMaterial = LoadMaterialDefault();
    renderer.clothMaterial.maps[MATERIAL_MAP_DIFFUSE].color = {255, 255, 255, 255};
    renderer.wireMaterial = LoadMaterialDefault();
    renderer.wireMaterial.maps[MATERIAL_MAP_DIFFUSE].color = {42, 58, 70, 210};
    renderer.ready = renderer.mesh.vboId != nullptr;
    return renderer.ready;
}

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
    cloth.pinnedParticles.clear();
    cloth.constraints.clear();
    cloth.particles.reserve(static_cast<std::size_t>(cloth.columns * cloth.rows));
    cloth.pinnedParticles.reserve(static_cast<std::size_t>(cloth.columns));
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
            const xpbd::Entity particle = world.createParticle({x, y, z}, mass, 0.018f);
            cloth.particles.push_back(particle);
            if (pinned) {
                cloth.pinnedParticles.push_back(particle);
            }
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

void drawClothDebug(const xpbd::XPBDWorld& world)
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

void drawPinnedParticles(const xpbd::XPBDWorld& world, const ClothDemo& cloth)
{
    for (xpbd::Entity entity : cloth.pinnedParticles) {
        const xpbd::Particle* particle = world.particle(entity);
        if (particle == nullptr) {
            continue;
        }

        DrawSphereEx(toRaylib(particle->position), particle->radius * 1.9f, 5, 8, Color{255, 115, 76, 255});
    }
}

void drawClothMesh(ClothRenderer& renderer, const xpbd::XPBDWorld& world, const ClothDemo& cloth)
{
    if (!renderer.ready) {
        drawClothDebug(world);
        return;
    }

    rlDisableBackfaceCulling();
    DrawMesh(renderer.mesh, renderer.clothMaterial, MatrixIdentity());

    int meshMaterial = 0;
    Model wireModel = {};
    wireModel.transform = MatrixIdentity();
    wireModel.meshCount = 1;
    wireModel.materialCount = 1;
    wireModel.meshes = &renderer.mesh;
    wireModel.materials = &renderer.wireMaterial;
    wireModel.meshMaterial = &meshMaterial;
    DrawModelWires(wireModel, Vector3{0.0f, 0.0f, 0.0f}, 1.0f, Color{255, 255, 255, 255});
    rlEnableBackfaceCulling();

    drawPinnedParticles(world, cloth);
}

}  // namespace

int main()
{
    xpbd::initializeSimd();

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
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
    ClothRenderer renderer;
    const bool meshRendererReady = initializeClothRenderer(renderer, world, cloth);

    bool paused = false;
    bool debugRender = !meshRendererReady;
    double simMs = 0.0;
    double renderMs = 0.0;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_R)) {
            buildCloth(world, cloth);
            updateClothMesh(renderer, world, cloth, true);
        }
        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
        }
        if (IsKeyPressed(KEY_D)) {
            debugRender = !debugRender;
        }

        UpdateCamera(&camera, CAMERA_ORBITAL);

        const float dt = std::min(GetFrameTime(), 1.0f / 30.0f);
        const double simStart = GetTime();
        if (!paused) {
            world.step(dt);
            updateClothMesh(renderer, world, cloth, true);
        }
        simMs = (GetTime() - simStart) * 1000.0;

        const double renderStart = GetTime();
        BeginDrawing();
        ClearBackground({28, 30, 34, 255});

        BeginMode3D(camera);
        DrawGrid(24, 0.25f);
        if (debugRender) {
            drawClothDebug(world);
        } else {
            drawClothMesh(renderer, world, cloth);
        }
        EndMode3D();

        DrawFPS(10, 10);
        char stats[160] = {};
        std::snprintf(stats,
                      sizeof(stats),
                      "particles: %zu  distance constraints: %zu  simd: %s  render: %s",
                      world.particleCount(),
                      world.distanceConstraintCount(),
                      world.simdBackendName(),
                      debugRender ? "debug" : "mesh");
        DrawText(stats, 10, 34, 18, Color{210, 218, 226, 255});
        char timings[120] = {};
        std::snprintf(timings,
                      sizeof(timings),
                      "sim: %.2f ms  frame: %.2f ms  %s",
                      simMs,
                      renderMs,
                      paused ? "paused" : "running");
        DrawText(timings, 10, 56, 18, Color{210, 218, 226, 255});

        EndDrawing();
        renderMs = (GetTime() - renderStart) * 1000.0;
    }

    unloadClothRenderer(renderer);
    CloseWindow();
    return 0;
}
