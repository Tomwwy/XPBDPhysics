#ifndef XPBD_EXAMPLES_CAMERA_CONTROLLER_HPP
#define XPBD_EXAMPLES_CAMERA_CONTROLLER_HPP

#include "raylib.h"
#include "raymath.h"

#include <cmath>

namespace examples {

// A small camera controller for the manual demos. It owns a raylib Camera3D and
// switches between two modes:
//
//   Orbital  - raylib's built-in CAMERA_ORBITAL: the camera circles the target.
//   Free fly - WASD to move, Q/E down/up, right-mouse-drag (or hold to capture)
//              to look around, wheel to change move speed. Classic flythrough.
//
// Toggle with a key (default Tab). The demo calls update() once per frame and
// reads camera() for BeginMode3D.
class CameraController {
public:
    enum class Mode { Orbital, FreeFly };

    explicit CameraController(Vector3 position = {0.0f, 2.0f, 6.0f},
                              Vector3 target = {0.0f, 1.0f, 0.0f})
    {
        camera_.position = position;
        camera_.target = target;
        camera_.up = {0.0f, 1.0f, 0.0f};
        camera_.fovy = 45.0f;
        camera_.projection = CAMERA_PERSPECTIVE;
        syncAnglesFromCamera();
    }

    Camera3D& camera() { return camera_; }
    const Camera3D& camera() const { return camera_; }
    Mode mode() const { return mode_; }
    const char* modeName() const { return mode_ == Mode::FreeFly ? "free fly" : "orbital"; }

    void setToggleKey(int key) { toggleKey_ = key; }

    void update(float dt)
    {
        if (IsKeyPressed(toggleKey_)) {
            mode_ = (mode_ == Mode::Orbital) ? Mode::FreeFly : Mode::Orbital;
            if (mode_ == Mode::FreeFly) {
                syncAnglesFromCamera();
            }
        }

        if (mode_ == Mode::Orbital) {
            // Built-in orbit; ensures the demo behaves like xpbd_physics by default.
            UpdateCamera(&camera_, CAMERA_ORBITAL);
            return;
        }

        updateFreeFly(dt);
    }

private:
    void syncAnglesFromCamera()
    {
        const Vector3 forward = Vector3Normalize(Vector3Subtract(camera_.target, camera_.position));
        yaw_ = std::atan2(forward.x, forward.z);
        pitch_ = std::asin(forward.y);
    }

    void updateFreeFly(float dt)
    {
        // Mouse look while the right button is held (so the cursor stays usable
        // for the on-screen hints otherwise).
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            const Vector2 delta = GetMouseDelta();
            yaw_ -= delta.x * kLookSensitivity;
            pitch_ -= delta.y * kLookSensitivity;
            const float limit = (PI * 0.5f) - 0.01f;
            if (pitch_ > limit) pitch_ = limit;
            if (pitch_ < -limit) pitch_ = -limit;
        }

        // Wheel adjusts move speed.
        const float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) {
            moveSpeed_ = Clamp(moveSpeed_ * (1.0f + wheel * 0.1f), 0.5f, 60.0f);
        }

        const Vector3 forward = {std::cos(pitch_) * std::sin(yaw_),
                                 std::sin(pitch_),
                                 std::cos(pitch_) * std::cos(yaw_)};
        const Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera_.up));

        float speed = moveSpeed_;
        if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
            speed *= 3.0f;
        }
        const float step = speed * dt;

        Vector3 move = {0.0f, 0.0f, 0.0f};
        if (IsKeyDown(KEY_W)) move = Vector3Add(move, Vector3Scale(forward, step));
        if (IsKeyDown(KEY_S)) move = Vector3Subtract(move, Vector3Scale(forward, step));
        if (IsKeyDown(KEY_D)) move = Vector3Add(move, Vector3Scale(right, step));
        if (IsKeyDown(KEY_A)) move = Vector3Subtract(move, Vector3Scale(right, step));
        if (IsKeyDown(KEY_E)) move.y += step;
        if (IsKeyDown(KEY_Q)) move.y -= step;

        camera_.position = Vector3Add(camera_.position, move);
        camera_.target = Vector3Add(camera_.position, forward);
    }

    static constexpr float kLookSensitivity = 0.003f;

    Camera3D camera_ = {};
    Mode mode_ = Mode::Orbital;
    int toggleKey_ = KEY_TAB;
    float yaw_ = 0.0f;    // radians, around +Y
    float pitch_ = 0.0f;  // radians, around local right axis
    float moveSpeed_ = 6.0f;
};

}  // namespace examples

#endif  // XPBD_EXAMPLES_CAMERA_CONTROLLER_HPP
