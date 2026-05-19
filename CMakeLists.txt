cmake_minimum_required(VERSION 3.15)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5) 

project(PhysicsEngine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        master 
)
FetchContent_MakeAvailable(raylib)

# This tells CMake where to find your .h files (so you don't list them in SOURCES)
include_directories(src)
include_directories(inc)

# ONLY .cpp files go here!
set(SOURCES 
    src/main.cpp 
    src/entities.cpp
    src/quadtree.cpp
    src/physics.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE raylib)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /fp:fast)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -ffast-math -mavx -msse4.2)
endif()