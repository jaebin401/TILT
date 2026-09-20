#pragma once

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
