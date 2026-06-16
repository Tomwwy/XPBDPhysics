#include "raylib.h"
#include "raymath.h"

namespace {

constexpr int kScreenWidth = 1280;
constexpr int kScreenHeight = 720;

} // namespace

int main()
{
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
    InitWindow(kScreenWidth, kScreenHeight, "XPBD Physics");
    SetWindowMinSize(640, 360);
    SetTargetFPS(144);

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground({ 30, 30, 35, 255 });
        DrawFPS(10, 10);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
