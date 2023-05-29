# On Ubuntu 18.04, get the latest CMake from https://apt.kitware.com/.
cmake_minimum_required(VERSION 3.18)

project(EPFL LANGUAGES C CXX)
include_directories(external)
add_executable(${PROJECT_NAME}
                main.cpp
                src/PointCloudProcessor.cpp
                src/PointCloudProcessor.h
        )

#open3d
find_package(Open3D REQUIRED PATHS /home/guli/CLionProjects/EPFL/external/Open3D/build/)
target_include_directories(${PROJECT_NAME} PRIVATE external/Open3D/build/include)

#cilantro
find_package(cilantro REQUIRED PATHS /home/guli/CLionProjects/EPFL/external/cilantro/build)

#cgal
target_include_directories(${PROJECT_NAME} PRIVATE external/cgal/build/include)


# linking
target_link_libraries(${PROJECT_NAME} PUBLIC Open3D::Open3D cilantro ${cilantro_LIBRARIES})