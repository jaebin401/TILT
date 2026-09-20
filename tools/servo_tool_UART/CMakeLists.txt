cmake_minimum_required(VERSION 3.16)

# 공용 컴포넌트(tilt_config, tilt_sts3215)를 firmware/components 에서 가져온다.
# 이 파일이 firmware/tools/servo_tool/CMakeLists.txt 에 있다는 전제.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tilt_servo_tool)