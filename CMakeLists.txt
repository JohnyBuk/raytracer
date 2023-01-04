cmake_minimum_required(VERSION 3.14)
project(raytracer LANGUAGES CXX)
add_compile_definitions(NDEBUG) # for PhysX

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(submodules/PhysX/physx)
add_subdirectory(submodules/bullet3)

add_executable(${PROJECT_NAME} src/main.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(${PROJECT_NAME} PRIVATE
    submodules/bullet3/src
    ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE 
    PhysX::PhysXCommon 
    PhysX::PhysXExtensions
    BulletCollision 
    Bullet3Common
    LinearMath 
    Threads::Threads
    OpenMP::OpenMP_CXX
    ${OpenCV_LIBS})