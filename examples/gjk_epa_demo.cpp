// Manual interactive test for the GJK/EPA convex-convex narrowphase cell.
//
// Two movable convex shapes (cycle each through sphere / box / capsule). Move
// and rotate the active shape; the demo runs gjkEpaContact() every frame and
// visualizes the result: a sphere at the contact point, an arrow along the
// contact normal whose length is the penetration depth, and an on-screen
// readout of depth + normal. This is the visual counterpart to tests/gjk_tests.
//
// Controls:
//   Tab            toggle orbital / free-fly camera
//   W A S D Q E    fly (free-fly mode); hold RMB to look; wheel = speed
//   Arrows / PgUp/PgDn   move the ACTIVE shape (camera-relative XZ, vertical)
//   I K J L U O    rotate the ACTIVE shape (pitch / yaw / roll)
//   1              select shape A as active   2  select shape B
//   Z / X          cycle the active shape type (sphere / box / capsule)
//   Space          reset the two shapes to their start poses
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include "camera_controller.hpp"
#include "xpbd/narrowphase/gjk.hpp"
#include "xpbd/quat.hpp"
#include "xpbd/xpbd_simd.hpp"

#include <array>
#include <cmath>
#include <cstdio>

namespace {

constexpr int kScreenWidth = 1280;
constexpr int kScreenHeight = 720;

using xpbd::Quat;
using xpbd::Vec3;
namespace np = xpbd::narrowphase;

Vector3 toRaylib(const Vec3& v) { return {v.x, v.y, v.z}; }

enum class ShapeKind { Sphere, Box, Capsule };

const char* shapeKindName(ShapeKind k)
{
    switch (k) {
    case ShapeKind::Sphere:  return "sphere";
    case ShapeKind::Box:     return "box";
    case ShapeKind::Capsule: return "capsule";
    }
    return "?";
}

// One movable convex body: a kind, a world pose, and a half-size. The local
// vertex set (for box) and segment (for capsule) are derived from `halfExtent`.
struct Body {
    ShapeKind kind = ShapeKind::Box;
    Vec3 position{};
    Quat orientation = Quat::identity();
    float halfExtent = 0.8f;   // box half-side, sphere radius proxy, capsule half-length
    float radius = 0.5f;       // sphere/capsule rounding radius
    Color color{200, 200, 200, 255};

    // World-space box corners (used by both the hull adapter and the renderer).
    std::array<Vec3, 8> boxCorners() const
    {
        const float h = halfExtent;
        std::array<Vec3, 8> local = {
            Vec3{-h, -h, -h}, Vec3{h, -h, -h}, Vec3{h, h, -h}, Vec3{-h, h, -h},
            Vec3{-h, -h, h},  Vec3{h, -h, h},  Vec3{h, h, h},  Vec3{-h, h, h}};
        std::array<Vec3, 8> world{};
        for (std::size_t i = 0; i < 8; ++i) {
            world[i] = position + xpbd::rotate(orientation, local[i]);
        }
        return world;
    }

    // World-space capsule segment endpoints (axis along the body's local Y).
    void capsuleSegment(Vec3& a, Vec3& b) const
    {
        const Vec3 axis = xpbd::rotate(orientation, Vec3{0.0f, halfExtent, 0.0f});
        a = position - axis;
        b = position + axis;
    }
};

// Run the active shape pair through GJK/EPA. The hull adapters need stable
// storage for the box corners, so they are kept alive by the caller.
xpbd::Contact evaluateContact(const Body& A, const Body& B,
                            std::array<Vec3, 8>& cornersA, std::array<Vec3, 8>& cornersB)
{
    cornersA = A.boxCorners();
    cornersB = B.boxCorners();

    // Dispatch each body to its adapter, then call the single generic cell.
    // The two template parameters resolve at compile time per kind pair.
    auto withShape = [&](const Body& body, std::array<Vec3, 8>& corners, auto&& fn) {
        switch (body.kind) {
        case ShapeKind::Sphere:
            fn(np::GjkSphere{body.position, body.radius});
            break;
        case ShapeKind::Box:
            fn(np::GjkHull{corners.data(), 8, 0.0f});
            break;
        case ShapeKind::Capsule: {
            Vec3 a, b;
            body.capsuleSegment(a, b);
            // Segment endpoints must outlive the call; capture by value below.
            fn(np::GjkCapsule{a, b, body.radius});
            break;
        }
        }
    };

    xpbd::Contact result;
    withShape(A, cornersA, [&](auto shapeA) {
        withShape(B, cornersB, [&](auto shapeB) {
            result = np::gjkEpaContact(shapeA, shapeB);
        });
    });
    return result;
}

// --- Rendering --------------------------------------------------------------

void drawBox(const std::array<Vec3, 8>& v, Color fill, Color wire)
{
    static const int faces[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4},
                                     {2, 3, 7, 6}, {1, 2, 6, 5}, {0, 3, 7, 4}};
    static const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                                     {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    Vector3 p[8];
    for (int i = 0; i < 8; ++i) p[i] = toRaylib(v[static_cast<std::size_t>(i)]);

    rlDisableBackfaceCulling();
    for (const auto& f : faces) {
        DrawTriangle3D(p[f[0]], p[f[1]], p[f[2]], fill);
        DrawTriangle3D(p[f[0]], p[f[2]], p[f[3]], fill);
    }
    rlEnableBackfaceCulling();
    for (const auto& e : edges) DrawLine3D(p[e[0]], p[e[1]], wire);
}

void drawCapsule(const Body& body, Color fill, Color wire)
{
    Vec3 a, b;
    body.capsuleSegment(a, b);
    DrawCylinderEx(toRaylib(a), toRaylib(b), body.radius, body.radius, 16, fill);
    DrawSphere(toRaylib(a), body.radius, fill);
    DrawSphere(toRaylib(b), body.radius, fill);
    DrawCylinderWiresEx(toRaylib(a), toRaylib(b), body.radius, body.radius, 16, wire);
}

void drawBody(const Body& body, const std::array<Vec3, 8>& corners)
{
    const Color fill = {body.color.r, body.color.g, body.color.b, 110};
    const Color wire = body.color;
    switch (body.kind) {
    case ShapeKind::Sphere:
        DrawSphere(toRaylib(body.position), body.radius, fill);
        DrawSphereWires(toRaylib(body.position), body.radius, 12, 12, wire);
        break;
    case ShapeKind::Box:
        drawBox(corners, fill, wire);
        break;
    case ShapeKind::Capsule:
        drawCapsule(body, fill, wire);
        break;
    }
}

// An arrow from `from` to `from + dir`, with a conical head. Used for the
// contact normal (length scaled to the penetration depth).
void drawArrow(const Vec3& from, const Vec3& dir, float length, Color color)
{
    if (length < 1e-5f) return;
    const Vec3 unit = xpbd::normalized(dir);
    const Vec3 tip = from + unit * length;
    const float headLen = std::fmin(length * 0.35f, 0.25f);
    const Vec3 headBase = tip - unit * headLen;
    DrawLine3D(toRaylib(from), toRaylib(headBase), color);
    DrawCylinderEx(toRaylib(headBase), toRaylib(tip), headLen * 0.45f, 0.0f, 12, color);
}

// Reset both bodies to a known overlapping-ish start so the demo opens with a
// contact already visible.
void resetBodies(Body& A, Body& B)
{
    A = Body{};
    A.kind = ShapeKind::Box;
    A.position = {-0.7f, 1.2f, 0.0f};
    A.orientation = Quat::identity();
    A.halfExtent = 0.8f;
    A.radius = 0.5f;
    A.color = Color{120, 200, 255, 255};

    B = Body{};
    B.kind = ShapeKind::Sphere;
    B.position = {0.7f, 1.2f, 0.0f};
    B.orientation = Quat::identity();
    B.halfExtent = 0.8f;
    B.radius = 0.6f;
    B.color = Color{255, 170, 90, 255};
}

// Apply a small incremental rotation about a world axis to a body.
void spin(Body& body, const Vec3& axis, float radians)
{
    body.orientation = xpbd::normalized(Quat::fromAxisAngle(axis, radians) * body.orientation);
}

}  // namespace

int main()
{
    xpbd::initializeSimd();

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(kScreenWidth, kScreenHeight, "XPBD GJK / EPA Visual Test");
    SetWindowMinSize(640, 360);
    SetTargetFPS(144);

    examples::CameraController cameraController({4.0f, 4.0f, 7.0f}, {0.0f, 1.2f, 0.0f});

    Body bodyA, bodyB;
    resetBodies(bodyA, bodyB);
    int active = 0;  // 0 = A, 1 = B

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();
        cameraController.update(dt);

        if (IsKeyPressed(KEY_ONE)) active = 0;
        if (IsKeyPressed(KEY_TWO)) active = 1;
        Body& act = active == 0 ? bodyA : bodyB;

        if (IsKeyPressed(KEY_Z) || IsKeyPressed(KEY_X)) {
            const int n = static_cast<int>(act.kind);
            act.kind = static_cast<ShapeKind>((n + (IsKeyPressed(KEY_Z) ? 1 : 2)) % 3);
        }
        if (IsKeyPressed(KEY_SPACE)) {
            resetBodies(bodyA, bodyB);
        }

        // Translation: arrows move in the camera's XZ plane, PgUp/PgDn vertical.
        const float moveStep = 2.5f * dt;
        const Camera3D& cam = cameraController.camera();
        Vec3 fwd = xpbd::normalized(Vec3{cam.target.x - cam.position.x, 0.0f, cam.target.z - cam.position.z});
        if (xpbd::lengthSq(fwd) < 1e-6f) fwd = Vec3{0.0f, 0.0f, -1.0f};
        const Vec3 right = xpbd::normalized(xpbd::cross(fwd, Vec3{0.0f, 1.0f, 0.0f}));
        if (IsKeyDown(KEY_UP))        act.position += fwd * moveStep;
        if (IsKeyDown(KEY_DOWN))      act.position -= fwd * moveStep;
        if (IsKeyDown(KEY_RIGHT))     act.position += right * moveStep;
        if (IsKeyDown(KEY_LEFT))      act.position -= right * moveStep;
        if (IsKeyDown(KEY_PAGE_UP))   act.position.y += moveStep;
        if (IsKeyDown(KEY_PAGE_DOWN)) act.position.y -= moveStep;

        // Rotation: I/K pitch, J/L yaw, U/O roll.
        const float spinStep = 1.5f * dt;
        if (IsKeyDown(KEY_I)) spin(act, {1.0f, 0.0f, 0.0f}, spinStep);
        if (IsKeyDown(KEY_K)) spin(act, {1.0f, 0.0f, 0.0f}, -spinStep);
        if (IsKeyDown(KEY_J)) spin(act, {0.0f, 1.0f, 0.0f}, spinStep);
        if (IsKeyDown(KEY_L)) spin(act, {0.0f, 1.0f, 0.0f}, -spinStep);
        if (IsKeyDown(KEY_U)) spin(act, {0.0f, 0.0f, 1.0f}, spinStep);
        if (IsKeyDown(KEY_O)) spin(act, {0.0f, 0.0f, 1.0f}, -spinStep);

        std::array<Vec3, 8> cornersA{}, cornersB{};
        const xpbd::Contact contact = evaluateContact(bodyA, bodyB, cornersA, cornersB);

        BeginDrawing();
        ClearBackground({26, 28, 33, 255});

        BeginMode3D(cameraController.camera());
        DrawGrid(40, 0.5f);

        // The bodies are translucent so the contact arrows underneath stay
        // visible. Translucent geometry must NOT write the depth buffer: if it
        // did, its own faces would z-reject each other (some faces vanish) and
        // it would occlude the arrows drawn afterwards. Disable depth writes for
        // the bodies, draw the arrows with depth writes back on so they read
        // crisp on top. (Depth *testing* stays on throughout.)
        rlDisableDepthMask();
        drawBody(bodyA, cornersA);
        drawBody(bodyB, cornersB);
        rlEnableDepthMask();

        if (contact.touching) {
            // Contact point.
            DrawSphere(toRaylib(contact.point), 0.06f, Color{255, 80, 80, 255});
            // Normal arrow (A -> B), drawn both ways from the contact point and
            // scaled by the penetration depth so deeper overlap = longer arrow.
            const float vis = std::fmax(contact.penetration, 0.08f) * 2.0f;
            drawArrow(contact.point, contact.normal, vis, Color{255, 80, 80, 255});
            drawArrow(contact.point, contact.normal * -1.0f, vis, Color{120, 120, 255, 255});
            // Surface witness markers (point pushed onto each body's surface).
            const Vec3 surfA = contact.point - contact.normal * (contact.penetration * 0.5f);
            const Vec3 surfB = contact.point + contact.normal * (contact.penetration * 0.5f);
            DrawSphere(toRaylib(surfA), 0.04f, Color{120, 200, 255, 255});
            DrawSphere(toRaylib(surfB), 0.04f, Color{255, 170, 90, 255});
        }
        EndMode3D();

        DrawFPS(10, 10);
        char line1[160] = {};
        std::snprintf(line1, sizeof(line1),
                      "active: shape %s (%s)   |   A: %s   B: %s",
                      active == 0 ? "A" : "B", shapeKindName(act.kind),
                      shapeKindName(bodyA.kind), shapeKindName(bodyB.kind));
        DrawText(line1, 10, 34, 18, Color{210, 218, 226, 255});

        char line2[160] = {};
        if (contact.touching) {
            std::snprintf(line2, sizeof(line2),
                          "CONTACT   depth: %.4f   normal (A->B): (%.3f, %.3f, %.3f)",
                          contact.penetration, contact.normal.x, contact.normal.y, contact.normal.z);
            DrawText(line2, 10, 56, 18, Color{255, 140, 140, 255});
        } else {
            DrawText("no contact (shapes are separated)", 10, 56, 18, Color{150, 220, 160, 255});
        }

        DrawText("1/2 select  |  Z/X cycle type  |  Arrows+PgUp/Dn move  |  IKJLUO rotate  |  Space reset  |  Tab camera",
                 10, 78, 18, Color{170, 180, 190, 255});

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
