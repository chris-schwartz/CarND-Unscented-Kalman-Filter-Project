project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/ukf.cpp
    src/main.cpp
    src/sigma_point_operations.cpp
    src/tools.cpp
    src/radar_measurement_handler.cpp
    src/measurement_handler.cpp
    src/lidar_measurment_handler.cpp)

set(test_sources
    test/all_tests.cpp
    test/test_sigma_point_operations.cpp
    test/test_ukf.cpp
    test/test_radar_measurement_handler.cpp
    test/test_lidar_measurement_handler.cpp
    test/test_measurement_handler.cpp)

include_directories(src)
add_library(UnscentedKFLib ${sources})

add_executable(UnscentedKF src/main.cpp)
target_link_libraries(UnscentedKF UnscentedKFLib)

add_executable(Tests ${test_sources})
target_link_libraries(Tests UnscentedKFLib)

