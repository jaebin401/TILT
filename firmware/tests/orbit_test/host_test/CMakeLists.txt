cmake_minimum_required(VERSION 3.16)
project(tilt_orbit_model_test CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_testing()
add_executable(test_orbit_model
    test_orbit_model.cpp
    ../main/GaitController.cpp
)
target_include_directories(test_orbit_model PRIVATE
    ../main
    ../../../components/tilt_config/include
    ../../../components/tilt_MPU6050/include
)
add_test(NAME orbit_model COMMAND test_orbit_model)
