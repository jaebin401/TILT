#pragma once

#include <cstdint>

namespace tilt {

inline constexpr float DEG2RAD = 0.01745329251994329577f;

inline constexpr float THIGH_LENGTH_MM = 50.0f;
inline constexpr float CALF_LENGTH_MM = 50.0f;
inline constexpr float FOOT_LENGTH_MM = 10.0f;
inline constexpr float Y_HIP_MM = 36.2f;

// The ankle bracket keeps the foot at this fixed angle relative to the calf.
inline constexpr float ANKLE_FIXED_RAD = -20.0f * DEG2RAD;

// The knee bracket contributes this fixed mechanical offset to the calf.
inline constexpr float KNEE_OFFSET_RAD = +20.0f * DEG2RAD;

inline constexpr float YAW_SINGULARITY_EPS_MM = 1.0f;

inline constexpr int IMU_I2C_PORT = 0;
inline constexpr int IMU_SDA_PIN = 8;
inline constexpr int IMU_SCL_PIN = 9;
inline constexpr std::uint8_t IMU_I2C_ADDR = 0x68;
inline constexpr float IMU_FILTER_ALPHA = 0.98f;
inline constexpr std::uint32_t IMU_SAMPLE_PERIOD_MS = 10;

struct JointLimit {
    float minimum_rad;
    float maximum_rad;
};

// Indexed as [Leg][Hip Yaw, Hip Pitch, Knee Pitch]. These are provisional
// software limits and remain separate from the raw-servo calibration table.
inline constexpr JointLimit JOINT_LIMIT[2][3] = {
    {
        {-10.0f * DEG2RAD, +10.0f * DEG2RAD},
        {-35.0f * DEG2RAD, +5.0f * DEG2RAD},
        {-5.0f * DEG2RAD, +40.0f * DEG2RAD},
    },
    {
        {-10.0f * DEG2RAD, +10.0f * DEG2RAD},
        {-35.0f * DEG2RAD, +5.0f * DEG2RAD},
        {-5.0f * DEG2RAD, +40.0f * DEG2RAD},
    },
};

}  // namespace tilt
