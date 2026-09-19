cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tilt_core"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tilt_motion"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tilt_safety"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tilt_actuators"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/tilt_sts3215"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tilt_pose_test)
