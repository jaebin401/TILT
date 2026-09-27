idf_component_register(
    SRCS "PacketCodec.cpp" "Sts3215Bus.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_uart tilt_actuators
)
