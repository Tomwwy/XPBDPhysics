cmake_minimum_required(VERSION 3.20)

project(xpbd_physics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(RAYLIB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../raylib_source/build-gl43/raylib")
set(RAYLIB_INCLUDE_DIR "${RAYLIB_ROOT}/include")
set(RAYLIB_DLL "${RAYLIB_ROOT}/Release/raylib.dll")
set(RAYLIB_IMPLIB "${RAYLIB_ROOT}/Release/raylib.lib")

if(NOT EXISTS "${RAYLIB_INCLUDE_DIR}/raylib.h")
    message(FATAL_ERROR "raylib headers not found under: ${RAYLIB_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${RAYLIB_DLL}")
    message(FATAL_ERROR "raylib DLL not found under: ${RAYLIB_DLL}")
endif()

if(NOT EXISTS "${RAYLIB_IMPLIB}")
    message(FATAL_ERROR "raylib import library not found under: ${RAYLIB_IMPLIB}")
endif()

add_library(raylib SHARED IMPORTED GLOBAL)
set_target_properties(raylib PROPERTIES
    IMPORTED_LOCATION "${RAYLIB_DLL}"
    IMPORTED_IMPLIB "${RAYLIB_IMPLIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${RAYLIB_INCLUDE_DIR}"
)

add_library(xpbd STATIC
    src/xpbd/systems/distance_constraint_system.cpp
    src/xpbd/systems/sphere_collision_system.cpp
    src/xpbd/systems/verlet_integration_system.cpp
    src/xpbd/xpbd_simd.cpp
    src/xpbd/xpbd_world.cpp
)

target_include_directories(xpbd PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

if(MSVC)
    target_compile_options(xpbd PRIVATE /W4)
else()
    target_compile_options(xpbd PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(xpbd_physics main.cpp)

if(MSVC)
    target_compile_options(xpbd_physics PRIVATE /W4)
else()
    target_compile_options(xpbd_physics PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(xpbd_physics PRIVATE xpbd raylib)

add_custom_command(TARGET xpbd_physics POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${RAYLIB_DLL}"
        "$<TARGET_FILE_DIR:xpbd_physics>/raylib.dll"
    VERBATIM
)
