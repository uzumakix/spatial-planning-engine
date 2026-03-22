cmake_minimum_required(VERSION 3.16)
project(spatial-planning-engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(LIB_SOURCES
    src/grid.cpp
    src/astar.cpp
    src/rrt.cpp
    src/smoother.cpp
)

add_library(planning_core STATIC ${LIB_SOURCES})
target_include_directories(planning_core PUBLIC src)

add_executable(planner src/main.cpp)
target_link_libraries(planner planning_core)

# tests
add_executable(test_grid tests/test_grid.cpp)
target_link_libraries(test_grid planning_core)

add_executable(test_astar tests/test_astar.cpp)
target_link_libraries(test_astar planning_core)

add_executable(test_rrt tests/test_rrt.cpp)
target_link_libraries(test_rrt planning_core)

add_executable(bench_planner bench/bench_planner.cpp)
target_link_libraries(bench_planner planning_core)

enable_testing()
add_test(NAME grid COMMAND test_grid)
add_test(NAME astar COMMAND test_astar)
add_test(NAME rrt COMMAND test_rrt)
