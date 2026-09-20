idf_component_register(
    SRCS "main.cpp" "JointMapper.cpp" "Interpolator.cpp"
    INCLUDE_DIRS "."
    REQUIRES tilt_config tilt_kinematics tilt_sts3215
             esp_driver_usb_serial_jtag freertos
)
