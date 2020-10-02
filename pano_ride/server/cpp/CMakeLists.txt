cmake_minimum_required(VERSION 2.8)

project(MapScale)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3 -march=native -fopenmp")
set(CMAKE_BUIDL_TYPE Release)

find_package(OpenCV REQUIRED)
find_package(g2o REQUIRED
             COMPONENTS
             g2o::core
             g2o::stuff
             g2o::types_sba
             g2o::types_sim3
             g2o::solver_dense
             g2o::solver_eigen
             g2o::solver_csparse
             g2o::csparse_extension)
find_package(Eigen3 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    /usr/local/include/opencv4
    /usr/local/include/eigen3
    /usr/local/include/
)
set(SRC
${PROJECT_SOURCE_DIR}/src/convert_to_visual_map.cc
${PROJECT_SOURCE_DIR}/src/g2otypes.cpp
${PROJECT_SOURCE_DIR}/src/imu_tools.cpp
${PROJECT_SOURCE_DIR}/src/IMUConverter.cc
${PROJECT_SOURCE_DIR}/src/imudata.cpp
${PROJECT_SOURCE_DIR}/src/IMUPreintegrator.cpp
${PROJECT_SOURCE_DIR}/src/NavState.cpp
${PROJECT_SOURCE_DIR}/src/so3.cpp
)

set(LIBS
    ${OpenCV_LIBS}
    g2o::core
    g2o::stuff
    g2o::types_sba
    g2o::types_sim3
    g2o::solver_dense
    g2o::solver_eigen
    g2o::solver_csparse
    g2o::csparse_extension
)


#for exe:
add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${LIBS})

