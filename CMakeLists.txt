cmake_minimum_required(VERSION 3.5...3.31)
project(3DPhysicsEngine)
set(CMAKE_CXX_STANDARD 20)

include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
)
FetchContent_MakeAvailable(raylib)

file(GLOB_RECURSE PHYSICS_SOURCES src/*.cpp)

add_library(physics STATIC ${PHYSICS_SOURCES})

target_include_directories(physics
    PUBLIC src  # expose headers to consumers
)

file(GLOB_RECURSE DEMO_SOURCES demo/*.cpp)

add_executable(demo ${DEMO_SOURCES})

target_link_libraries(demo
    PRIVATE
        physics
        raylib
)