#pragma once

#include <cmath>

#include "tilt_config.h"

namespace tilt_rock_test {

struct ParallelLegPose {
    float hip_pitch_rad = 0.0f;
    float knee_pitch_rad = 0.0f;
    float thigh_angle_rad = 0.0f;
    float foot_x_mm = 0.0f;  // Relative to hip, at zero torso lean.
    bool reachable = false;
};

inline float calfVerticalMm() {
    return tilt::CALF_LENGTH_MM * std::cos(-tilt::ANKLE_FIXED_RAD) +
           tilt::FOOT_LENGTH_MM;
}

inline float calfHorizontalMm() {
    return -tilt::CALF_LENGTH_MM * std::sin(-tilt::ANKLE_FIXED_RAD);
}

inline ParallelLegPose parallelLeg(float height_mm, float lean_rad) {
    ParallelLegPose pose{};
    if (!std::isfinite(height_mm) || !std::isfinite(lean_rad)) return pose;

    const float cosine =
        (height_mm - calfVerticalMm()) / tilt::THIGH_LENGTH_MM;
    if (cosine < -1.0f || cosine > 1.0f) return pose;

    const float a = std::acos(cosine);
    pose.hip_pitch_rad = -a + lean_rad;
    pose.knee_pitch_rad =
        -tilt::ANKLE_FIXED_RAD - tilt::KNEE_OFFSET_RAD + a;
    pose.thigh_angle_rad = a;
    pose.foot_x_mm = tilt::THIGH_LENGTH_MM * std::sin(a) +
                     calfHorizontalMm();
    pose.reachable = true;
    return pose;
}

inline bool parallelStance(float left_height_mm, float right_height_mm,
                           float lean_rad,
                           float out_theta[tilt::NUM_JOINTS],
                           float* out_left_foot_x_mm = nullptr,
                           float* out_right_foot_x_mm = nullptr) {
    if (out_theta == nullptr) return false;
    const ParallelLegPose left = parallelLeg(left_height_mm, lean_rad);
    const ParallelLegPose right = parallelLeg(right_height_mm, lean_rad);
    if (!left.reachable || !right.reachable) return false;

    const float theta[tilt::NUM_JOINTS] = {
        0.0f, left.hip_pitch_rad, left.knee_pitch_rad,
        0.0f, right.hip_pitch_rad, right.knee_pitch_rad,
    };
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const tilt::JointLimit& limit =
            tilt::JOINT_LIMIT[joint / 3][joint % 3];
        if (!std::isfinite(theta[joint]) || theta[joint] < limit.minimum_rad ||
            theta[joint] > limit.maximum_rad) {
            return false;
        }
    }

    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        out_theta[joint] = theta[joint];
    }
    if (out_left_foot_x_mm) *out_left_foot_x_mm = left.foot_x_mm;
    if (out_right_foot_x_mm) *out_right_foot_x_mm = right.foot_x_mm;
    return true;
}

}  // namespace tilt_rock_test
