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

set(XPBD_UTILS_MATH_SOURCES
    src/utils/math_dispatch.cpp
    src/utils/math_scalar.cpp
)

set(XPBD_X86_SIMD OFF)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "AMD64|x86_64|x86|i[3-6]86")
    set(XPBD_X86_SIMD ON)
    list(APPEND XPBD_UTILS_MATH_SOURCES
        src/utils/math_sse2.cpp
        src/utils/math_avx.cpp
        src/utils/math_avx2.cpp
    )
endif()

add_library(raylib SHARED IMPORTED GLOBAL)
set_target_properties(raylib PROPERTIES
    IMPORTED_LOCATION "${RAYLIB_DLL}"
    IMPORTED_IMPLIB "${RAYLIB_IMPLIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${RAYLIB_INCLUDE_DIR}"
)

add_library(xpbd STATIC
    src/xpbd/systems/distance_constraint_system.cpp
    src/xpbd/systems/rigid_body_integration_system.cpp
    src/xpbd/systems/verlet_integration_system.cpp
    src/xpbd/xpbd_simd.cpp
    src/xpbd/xpbd_world.cpp
    ${XPBD_UTILS_MATH_SOURCES}
)

target_include_directories(xpbd PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

if(XPBD_X86_SIMD)
    target_compile_definitions(xpbd PRIVATE UTILS_HAS_X86_SIMD=1)
endif()

if(MSVC)
    target_compile_options(xpbd PRIVATE /W4)
    if(XPBD_X86_SIMD)
        set_source_files_properties(src/utils/math_avx.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX")
        set_source_files_properties(src/utils/math_avx2.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX2")
    endif()
else()
    target_compile_options(xpbd PRIVATE -Wall -Wextra -Wpedantic)
    if(XPBD_X86_SIMD)
        set_source_files_properties(src/utils/math_sse2.cpp PROPERTIES COMPILE_OPTIONS "-msse2")
        set_source_files_properties(src/utils/math_avx.cpp PROPERTIES COMPILE_OPTIONS "-mavx")
        set_source_files_properties(src/utils/math_avx2.cpp PROPERTIES COMPILE_OPTIONS "-mavx2")
    endif()
endif()

add_executable(xpbd_physics main.cpp)
target_include_directories(xpbd_physics PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/examples")

if(MSVC)
    target_compile_options(xpbd_physics PRIVATE /W4)
else()
    target_compile_options(xpbd_physics PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(xpbd_physics PRIVATE xpbd raylib)

add_executable(xpbd_tetra_rigidbody examples/tetra_rigidbody.cpp)
target_include_directories(xpbd_tetra_rigidbody PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/examples")
if(MSVC)
    target_compile_options(xpbd_tetra_rigidbody PRIVATE /W4)
else()
    target_compile_options(xpbd_tetra_rigidbody PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_link_libraries(xpbd_tetra_rigidbody PRIVATE xpbd raylib)

add_executable(xpbd_tests tests/foundation_tests.cpp)
target_link_libraries(xpbd_tests PRIVATE xpbd)
if(MSVC)
    target_compile_options(xpbd_tests PRIVATE /W4)
else()
    target_compile_options(xpbd_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()
enable_testing()
add_test(NAME xpbd_foundation COMMAND xpbd_tests)

add_executable(xpbd_gjk_tests tests/gjk_tests.cpp)
target_link_libraries(xpbd_gjk_tests PRIVATE xpbd)
if(MSVC)
    target_compile_options(xpbd_gjk_tests PRIVATE /W4)
else()
    target_compile_options(xpbd_gjk_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()
add_test(NAME xpbd_gjk_epa COMMAND xpbd_gjk_tests)

add_executable(xpbd_gjk_demo examples/gjk_epa_demo.cpp)
target_include_directories(xpbd_gjk_demo PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/examples")
if(MSVC)
    target_compile_options(xpbd_gjk_demo PRIVATE /W4)
else()
    target_compile_options(xpbd_gjk_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_link_libraries(xpbd_gjk_demo PRIVATE xpbd raylib)

add_custom_command(TARGET xpbd_physics POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${RAYLIB_DLL}"
        "$<TARGET_FILE_DIR:xpbd_physics>/raylib.dll"
    VERBATIM
)

add_custom_command(TARGET xpbd_tetra_rigidbody POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${RAYLIB_DLL}"
        "$<TARGET_FILE_DIR:xpbd_tetra_rigidbody>/raylib.dll"
    VERBATIM
)

add_custom_command(TARGET xpbd_gjk_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${RAYLIB_DLL}"
        "$<TARGET_FILE_DIR:xpbd_gjk_demo>/raylib.dll"
    VERBATIM
)
