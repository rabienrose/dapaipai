cmake_minimum_required(VERSION 3.1)
project(bag_tool)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(--std=c++11 -Wno-unknown-pragmas)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/vslam)
cs_add_library(${PROJECT_NAME}
    src/extract_bag.cpp
    src/stitch_algo.cpp
    src/vslam/Initializer.cc
    src/vslam/KeyFrame.cc
    src/vslam/Optimizer.cc
    src/vslam/ORBextractor.cc
    src/vslam/ORBmatcher.cc
    src/vslam/PanoMap.cc
    src/vslam/Random.cpp
    src/vslam/RayPoint.cc
    src/vslam/so3.cpp
    src/vslam/Timestamp.cpp
)

cs_add_executable(${PROJECT_NAME}_exe src/extract_bag_exe.cpp)
target_link_libraries(${PROJECT_NAME}_exe ${PROJECT_NAME})

cs_install()
cs_export()
