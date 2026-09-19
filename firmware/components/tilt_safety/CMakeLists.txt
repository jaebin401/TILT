idf_component_register(
    SRCS
        "PositionLimitValidator.cpp"
        "SafetyController.cpp"
        "VelocityLimitValidator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES tilt_core
)
