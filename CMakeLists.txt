project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   src/ukf.cpp
   src/main.cpp
   src/sigma_point_operations.cpp
   src/tools.cpp)

set(test_sources
    test/all_tests.cpp
    test/)

include_directories(src)
add_library(UnscentedKFLib ${sources})

add_executable(UnscentedKF src/main.cpp)
target_link_libraries(UnscentedKF UnscentedKFLib)

add_executable(Tests test/tests.cpp)
target_link_libraries(Tests UnscentedKFLib)

