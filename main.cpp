#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "xpbd/xpbd_world.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

namespace {

constexpr int kScreenWidth = 1280;
constexpr int kScreenHeight = 720;
constexpr int kMeshVertexBufferPosition = 0;
constexpr int kMeshVertexBufferNormal = 2;
constexpr xpbd::CollisionLayerMask kClothCollisionLayer = xpbd::CollisionLayerMask{1u} << 0u;
constexpr xpbd::CollisionLayerMask kColliderCollisionLayer = xpbd::CollisionLayerMask{1u} << 1u;
constexpr float kClothParticleRadius = 0.018f;
constexpr float kCollisionSphereRadius = 0.52f;

Vector3 toRaylib(const xpbd::Vec3& value)
{
    return {value.x, value.y, value.z};
}

struct ClothDemo {
    int columns = 34;
    int rows = 22;
    float spacing = 0.13f;
    xpbd::Entity collisionSphere;        // static collider entity
    xpbd::Vec3 collisionSphereCenter = {0.0f, 0.72f, 0.05f};
    float collisionSphereRadius = kCollisionSphereRadius;
    bool selfCollisionEnabled = false;
    std::vector<xpbd::Entity> particles;
    std::vector<xpbd::Entity> colliders;  // parallel to particles
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

struct RandomWindowForceSettings {
    bool enabled = true;
    xpbd::Vec3 center = {0.0f, 0.78f, 0.0f};
    float width = 2.8f;
    float height = 1.65f;
    float depth = 1.2f;
    float baseStrength = 18.0f;
    float randomStrength = 16.0f;
    float changeInterval = 0.42f;
    float timeUntilChange = 0.0f;
    xpbd::Vec3 currentAcceleration = {0.0f, 0.0f, 18.0f};
    std::mt19937 rng{0x51f15eedu};
};

float clampFloat(float value, float low, float high)
{
    return std::max(low, std::min(high, value));
}

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

xpbd::CollisionLayerMask clothParticleCollisionMask(const ClothDemo& cloth)
{
    xpbd::CollisionLayerMask mask = kColliderCollisionLayer;
    if (cloth.selfCollisionEnabled) {
        mask |= kClothCollisionLayer;
    }
    return mask;
}

void applyClothCollisionFilters(xpbd::XPBDWorld& world, const ClothDemo& cloth)
{
    const xpbd::CollisionLayerMask clothMask = clothParticleCollisionMask(cloth);
    for (xpbd::Entity colliderEntity : cloth.colliders) {
        world.setColliderFilter(colliderEntity, kClothCollisionLayer, clothMask);
    }
    world.setColliderFilter(cloth.collisionSphere, kColliderCollisionLayer, kClothCollisionLayer);
}

xpbd::Vec3 sampleWindowAcceleration(RandomWindowForceSettings& settings)
{
    std::uniform_real_distribution<float> lateral(-0.45f, 0.45f);
    std::uniform_real_distribution<float> vertical(-0.18f, 0.32f);
    std::uniform_real_distribution<float> strength(0.0f, 1.0f);

    const xpbd::Vec3 direction = xpbd::normalized({lateral(settings.rng), vertical(settings.rng), 1.0f});
    return direction * (settings.baseStrength + settings.randomStrength * strength(settings.rng));
}

void applyRandomWindowForce(xpbd::XPBDWorld& world, RandomWindowForceSettings& settings, float dt)
{
    if (!settings.enabled || !(dt > 0.0f)) {
        return;
    }

    settings.timeUntilChange -= dt;
    if (settings.timeUntilChange <= 0.0f) {
        settings.currentAcceleration = sampleWindowAcceleration(settings);
        settings.timeUntilChange = std::max(0.02f, settings.changeInterval);
    }

    const float halfWidth = std::max(0.01f, settings.width * 0.5f);
    const float halfHeight = std::max(0.01f, settings.height * 0.5f);
    const float halfDepth = std::max(0.01f, settings.depth * 0.5f);

    world.forEachParticle([&settings, halfWidth, halfHeight, halfDepth](xpbd::Entity, xpbd::Particle& particle) {
        if (particle.inverseMass <= 0.0f) {
            return;
        }

        const xpbd::Vec3 offset = particle.position - settings.center;
        const float xAmount = std::fabs(offset.x) / halfWidth;
        const float yAmount = std::fabs(offset.y) / halfHeight;
        const float zAmount = std::fabs(offset.z) / halfDepth;
        if (xAmount > 1.0f || yAmount > 1.0f || zAmount > 1.0f) {
            return;
        }

        const float edgeAmount = std::max(xAmount, std::max(yAmount, zAmount));
        const float falloff = clampFloat((1.0f - edgeAmount) * 4.0f, 0.0f, 1.0f);
        particle.externalAcceleration += settings.currentAcceleration * falloff;
    });
}

void updateRuntimeSettings(xpbd::XPBDWorld& world, RandomWindowForceSettings& settings, ClothDemo& cloth)
{
    const float strengthStep = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 8.0f : 2.0f;
    const float sizeStep = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 0.4f : 0.15f;

    if (IsKeyPressed(KEY_C)) {
        world.setCollisionsEnabled(!world.collisionsEnabled());
    }
    if (IsKeyPressed(KEY_S)) {
        cloth.selfCollisionEnabled = !cloth.selfCollisionEnabled;
        applyClothCollisionFilters(world, cloth);
    }
    if (IsKeyPressed(KEY_F)) {
        settings.enabled = !settings.enabled;
    }
    if (IsKeyPressed(KEY_UP)) {
        settings.baseStrength = clampFloat(settings.baseStrength + strengthStep, 0.0f, 90.0f);
    }
    if (IsKeyPressed(KEY_DOWN)) {
        settings.baseStrength = clampFloat(settings.baseStrength - strengthStep, 0.0f, 90.0f);
    }
    if (IsKeyPressed(KEY_RIGHT)) {
        settings.width = clampFloat(settings.width + sizeStep, 0.4f, 5.0f);
    }
    if (IsKeyPressed(KEY_LEFT)) {
        settings.width = clampFloat(settings.width - sizeStep, 0.4f, 5.0f);
    }
    if (IsKeyPressed(KEY_PAGE_UP)) {
        settings.height = clampFloat(settings.height + sizeStep, 0.4f, 3.5f);
    }
    if (IsKeyPressed(KEY_PAGE_DOWN)) {
        settings.height = clampFloat(settings.height - sizeStep, 0.4f, 3.5f);
    }
}

void buildCloth(xpbd::XPBDWorld& world, ClothDemo& cloth)
{
    world.clearEntities();
    cloth.particles.clear();
    cloth.colliders.clear();
    cloth.pinnedParticles.clear();
    cloth.constraints.clear();
    cloth.particles.reserve(static_cast<std::size_t>(cloth.columns * cloth.rows));
    cloth.colliders.reserve(static_cast<std::size_t>(cloth.columns * cloth.rows));
    cloth.pinnedParticles.reserve(static_cast<std::size_t>(cloth.columns));
    cloth.constraints.reserve(static_cast<std::size_t>(cloth.columns * cloth.rows * 6));

    const float startX = -0.5f * static_cast<float>(cloth.columns - 1) * cloth.spacing;
    const float topY = 2.25f;

    const xpbd::CollisionLayerMask clothMask = clothParticleCollisionMask(cloth);
    for (int row = 0; row < cloth.rows; ++row) {
        for (int column = 0; column < cloth.columns; ++column) {
            const float x = startX + static_cast<float>(column) * cloth.spacing;
            const float y = topY - static_cast<float>(row) * cloth.spacing;
            const float z = 0.04f * std::sin(static_cast<float>(column) * 0.5f);
            const bool pinned = row == 0 && (column % 3 == 0 || column == cloth.columns - 1);
            const float mass = pinned ? 0.0f : 1.0f;
            const xpbd::Entity particle = world.createParticle({x, y, z}, mass);
            const xpbd::Entity collider = world.createCollider(
                particle,
                xpbd::Shape::makeSphere(kClothParticleRadius),
                kClothCollisionLayer,
                clothMask,
                xpbd::BroadphasePartition::Dynamic);
            cloth.particles.push_back(particle);
            cloth.colliders.push_back(collider);
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

    cloth.collisionSphereCenter = {0.0f, 0.72f, 0.05f};
    cloth.collisionSphereRadius = kCollisionSphereRadius;
    cloth.collisionSphere = world.createStaticCollider(
        xpbd::Shape::makeSphere(cloth.collisionSphereRadius),
        cloth.collisionSphereCenter,
        kColliderCollisionLayer,
        kClothCollisionLayer);
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
        const float radius = pinned ? kClothParticleRadius * 1.9f : kClothParticleRadius;
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

        DrawSphereEx(toRaylib(particle->position), kClothParticleRadius * 1.9f, 5, 8, Color{255, 115, 76, 255});
    }
}

void drawCollisionSpheres(const xpbd::XPBDWorld& world, const ClothDemo& cloth)
{
    const bool enabled = world.collisionsEnabled();
    const Color fillColor = enabled ? Color{255, 183, 77, 48} : Color{120, 128, 138, 36};
    const Color wireColor = enabled ? Color{255, 204, 118, 230} : Color{145, 152, 160, 180};

    const Vector3 center = toRaylib(cloth.collisionSphereCenter);
    DrawSphere(center, cloth.collisionSphereRadius, fillColor);
    DrawSphereWires(center, cloth.collisionSphereRadius, 24, 16, wireColor);
}

void drawRandomWindowForce(const RandomWindowForceSettings& settings)
{
    const float halfWidth = settings.width * 0.5f;
    const float halfHeight = settings.height * 0.5f;
    const float halfDepth = settings.depth * 0.5f;
    const xpbd::Vec3 c = settings.center;
    const std::array<xpbd::Vec3, 8> corners = {
        xpbd::Vec3{c.x - halfWidth, c.y - halfHeight, c.z - halfDepth},
        xpbd::Vec3{c.x + halfWidth, c.y - halfHeight, c.z - halfDepth},
        xpbd::Vec3{c.x + halfWidth, c.y + halfHeight, c.z - halfDepth},
        xpbd::Vec3{c.x - halfWidth, c.y + halfHeight, c.z - halfDepth},
        xpbd::Vec3{c.x - halfWidth, c.y - halfHeight, c.z + halfDepth},
        xpbd::Vec3{c.x + halfWidth, c.y - halfHeight, c.z + halfDepth},
        xpbd::Vec3{c.x + halfWidth, c.y + halfHeight, c.z + halfDepth},
        xpbd::Vec3{c.x - halfWidth, c.y + halfHeight, c.z + halfDepth},
    };

    const Color color = settings.enabled ? Color{84, 214, 167, 220} : Color{118, 128, 136, 160};
    const auto drawEdge = [&corners, color](int a, int b) {
        DrawLine3D(toRaylib(corners[static_cast<std::size_t>(a)]),
                   toRaylib(corners[static_cast<std::size_t>(b)]),
                   color);
    };

    drawEdge(0, 1);
    drawEdge(1, 2);
    drawEdge(2, 3);
    drawEdge(3, 0);
    drawEdge(4, 5);
    drawEdge(5, 6);
    drawEdge(6, 7);
    drawEdge(7, 4);
    drawEdge(0, 4);
    drawEdge(1, 5);
    drawEdge(2, 6);
    drawEdge(3, 7);

    const float accelLength = xpbd::length(settings.currentAcceleration);
    if (accelLength > 1e-5f) {
        const xpbd::Vec3 direction = settings.currentAcceleration / accelLength;
        const xpbd::Vec3 arrowEnd = settings.center + direction * 0.55f;
        DrawLine3D(toRaylib(settings.center), toRaylib(arrowEnd), color);
        DrawSphereEx(toRaylib(arrowEnd), 0.035f, 8, 8, color);
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
    world.setCollisionsEnabled(true);
    world.setContactCompliance(0.0f);

    RandomWindowForceSettings windowForce;
    world.addIntegrationSystem([&windowForce](xpbd::XPBDWorld& forceWorld, float subDt) {
        applyRandomWindowForce(forceWorld, windowForce, subDt);
    });
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
        updateRuntimeSettings(world, windowForce, cloth);

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
        drawCollisionSpheres(world, cloth);
        drawRandomWindowForce(windowForce);
        if (debugRender) {
            drawClothDebug(world);
        } else {
            drawClothMesh(renderer, world, cloth);
        }
        EndMode3D();
        renderMs = (GetTime() - renderStart) * 1000.0;

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
        char timings[160] = {};
        std::snprintf(timings,
                      sizeof(timings),
                      "sim: %.2f ms  render: %.2f ms  %s  sphere collision: %s  cloth self: %s",
                      simMs,
                      renderMs,
                      paused ? "paused" : "running",
                      world.collisionsEnabled() ? "on" : "off",
                      cloth.selfCollisionEnabled ? "on" : "off");
        DrawText(timings, 10, 56, 18, Color{210, 218, 226, 255});
        char collisionStats[180] = {};
        std::snprintf(collisionStats,
                      sizeof(collisionStats),
                      "bvh nodes: %zu  broadphase pairs: %zu  contacts: %zu  colliders: %zu",
                      world.broadphaseNodeCount(),
                      world.broadphasePairCount(),
                      world.contactCount(),
                      world.colliderCount());
        DrawText(collisionStats, 10, 78, 18, Color{210, 218, 226, 255});
        char forceStats[220] = {};
        std::snprintf(forceStats,
                      sizeof(forceStats),
                      "window force: %s  strength: %.1f + %.1f random  size: %.2fx%.2fx%.2f  C/S/F toggle, arrows/page resize",
                      windowForce.enabled ? "on" : "off",
                      windowForce.baseStrength,
                      windowForce.randomStrength,
                      windowForce.width,
                      windowForce.height,
                      windowForce.depth);
        DrawText(forceStats, 10, 100, 18, Color{210, 218, 226, 255});

        EndDrawing();
    }

    unloadClothRenderer(renderer);
    CloseWindow();
    return 0;
}
