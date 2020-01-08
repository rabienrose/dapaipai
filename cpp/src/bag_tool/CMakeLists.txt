cmake_minimum_required(VERSION 3.1)
project(bag_tool)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
set(OpenCV_DIR /usr/local/share/opencv4)
find_package(OpenCV 4 REQUIRED)
add_definitions(--std=c++11 -Wno-unknown-pragmas)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/vslam)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/g2o/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/g2o/include/g2o/types)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/g2o/include/g2o/stuff)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/g2o/include/g2o/solvers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/g2o/include/g2o/core)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/stitching/include)
include_directories( ${OpenCV_INCLUDE_DIRS} )
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
    src/stitching/src/autocalib.cpp
    src/stitching/src/blenders.cpp
    src/stitching/src/camera.cpp
    src/stitching/src/exposure_compensate.cpp
    src/stitching/src/matchers.cpp
    src/stitching/src/motion_estimators.cpp
    src/stitching/src/precomp.hpp
    src/stitching/src/seam_finders.cpp
    src/stitching/src/timelapsers.cpp
    src/stitching/src/util.cpp
    src/stitching/src/warpers.cpp
    src/g2o/src/types/types_sba.cpp
    src/g2o/src/types/types_seven_dof_expmap.cpp
    src/g2o/src/types/types_six_dof_expmap.cpp
    src/g2o/src/stuff/os_specific.c
    src/g2o/src/stuff/property.cpp
    src/g2o/src/stuff/string_tools.cpp
    src/g2o/src/stuff/timeutil.cpp
    src/g2o/src/core/batch_stats.cpp
    src/g2o/src/core/cache.cpp
    src/g2o/src/core/estimate_propagator.cpp
    src/g2o/src/core/factory.cpp
    src/g2o/src/core/hyper_dijkstra.cpp
    src/g2o/src/core/hyper_graph_action.cpp
    src/g2o/src/core/hyper_graph.cpp
    src/g2o/src/core/jacobian_workspace.cpp
    src/g2o/src/core/marginal_covariance_cholesky.cpp
    src/g2o/src/core/matrix_structure.cpp
    src/g2o/src/core/optimizable_graph.cpp
    src/g2o/src/core/optimization_algorithm_dogleg.cpp
    src/g2o/src/core/optimization_algorithm_factory.cpp
    src/g2o/src/core/optimization_algorithm_gauss_newton.cpp
    src/g2o/src/core/optimization_algorithm_levenberg.cpp
    src/g2o/src/core/optimization_algorithm_with_hessian.cpp
    src/g2o/src/core/optimization_algorithm.cpp
    src/g2o/src/core/parameter_container.cpp
    src/g2o/src/core/parameter.cpp
    src/g2o/src/core/robust_kernel_factory.cpp
    src/g2o/src/core/robust_kernel_impl.cpp
    src/g2o/src/core/robust_kernel.cpp
    src/g2o/src/core/solver.cpp
    src/g2o/src/core/sparse_optimizer.cpp
)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})
cs_add_executable(${PROJECT_NAME}_exe src/extract_bag_exe.cpp)
target_link_libraries(${PROJECT_NAME}_exe ${PROJECT_NAME})

cs_install()
cs_export()
