cmake_minimum_required(VERSION 3.16)
project(tilt_parallel_leg_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()
add_executable(test_parallel_leg test_parallel_leg.cpp)
target_include_directories(test_parallel_leg PRIVATE
    ../main
    ../../../components/tilt_config/include
)
add_test(NAME parallel_leg COMMAND test_parallel_leg)
