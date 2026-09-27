idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_uart
        esp_driver_usb_serial_jtag
        tilt_actuators
        tilt_core
        tilt_motion
        tilt_safety
        tilt_sts3215
)
