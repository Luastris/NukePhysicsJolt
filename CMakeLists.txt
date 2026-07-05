cmake_minimum_required(VERSION 3.20)
project(NukePhysicsJolt CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Match the rest of the toolchain: dynamic debug CRT (/MDd in Debug).
cmake_policy(SET CMP0091 NEW)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

set(NUKE_ROOT       "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(VCPKG_INSTALLED "$ENV{VCPKG_ROOT}/installed/x64-windows")

# --- Jolt Physics (vendored release v5.5.0, static lib; only the library target) ---
set(TARGET_UNIT_TESTS       OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD      OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES          OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER           OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS     OFF CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
# Jolt defaults to the STATIC MSVC runtime — the whole toolchain here is /MDd (dynamic).
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/deps/JoltPhysics/Build Jolt)

add_library(NukePhysicsJolt SHARED
    src/NukePhysicsJolt.cpp
)

# SERVICE plugin implementing iPhysics behind the POD seam. Plugins EXTEND the engine:
# NukeEngine.lib here is the IMPORT library of NukeEngine.dll (no static linking), used
# for engine API such as nuke::Jobs (solver runs on the shared worker pool, 2.4).
target_compile_definitions(NukePhysicsJolt PRIVATE
    WIN32 _WINDOWS NOMINMAX _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS
    GLM_ENABLE_EXPERIMENTAL
    BOOST_ALL_DYN_LINK
)

target_include_directories(NukePhysicsJolt PRIVATE
    ${NUKE_ROOT}/NukeEngine
    ${NUKE_ROOT}/NukeEngine/include
    ${NUKE_ROOT}/NukeEngine/deps
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/JoltPhysics
    ${VCPKG_INSTALLED}/include                    # boost (NUKEModule header chain)
)

target_link_directories(NukePhysicsJolt PRIVATE
    ${VCPKG_INSTALLED}/$<$<CONFIG:Debug>:debug/>lib   # boost auto-link (header chain)
    ${NUKE_ROOT}/NukeEngine/x64/$<CONFIG>             # NukeEngine.dll import lib
)

target_link_libraries(NukePhysicsJolt PRIVATE
    Jolt
    NukeEngine
)

# Deploy the plugin DLL into the editor's modules/ folder.
set(MODULES_DIR "${NUKE_ROOT}/NukeEngine/x64/$<CONFIG>/modules")
add_custom_command(TARGET NukePhysicsJolt POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${MODULES_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:NukePhysicsJolt>" "${MODULES_DIR}/"
    COMMENT "Deploying NukePhysicsJolt.dll to modules/"
)
