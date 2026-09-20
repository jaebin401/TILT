#include "FlatPose.h"

#include <cmath>

#include "tilt_config.h"

namespace tilt_pose_test {

bool flatPoseForHeight(float leg_height_mm, float out[3]) {
    if (out == nullptr || !std::isfinite(leg_height_mm)) {
        return false;
    }

    const float fixed_lower_height_mm =
        tilt::CALF_LENGTH_MM * std::cos(tilt::KNEE_OFFSET_RAD) +
        tilt::FOOT_LENGTH_MM;
    const float raw_cosine =
        (leg_height_mm - fixed_lower_height_mm) / tilt::THIGH_LENGTH_MM;
    if (raw_cosine < -1.0f || raw_cosine > 1.0f) {
        return false;
    }

    // Clamp only after the workspace check to absorb floating-point edge noise.
    const float cosine = std::fmax(-1.0f, std::fmin(1.0f, raw_cosine));
    const float angle_rad = std::acos(cosine);
    out[0] = 0.0f;
    out[1] = -angle_rad;
    out[2] = +angle_rad;
    return std::isfinite(angle_rad);
}

bool flatPoseForHeightAndBodyX(float leg_height_mm, float body_x_mm,
                               float out[3]) {
    if (!flatPoseForHeight(leg_height_mm, out) || !std::isfinite(body_x_mm)) {
        return false;
    }

    // With theta2 + theta3 fixed at zero, the thigh angle is the remaining
    // fore/aft degree of freedom. Positive body_x requires negative foot x.
    const float nominal_angle_rad = out[2];
    const float shifted_sine = std::sin(nominal_angle_rad) -
                               body_x_mm / tilt::THIGH_LENGTH_MM;
    if (shifted_sine < -1.0f || shifted_sine > 1.0f) {
        return false;
    }
    const float angle_rad = std::asin(
        std::fmax(-1.0f, std::fmin(1.0f, shifted_sine)));
    out[1] = -angle_rad;
    out[2] = +angle_rad;
    return std::isfinite(angle_rad);
}

}  // namespace tilt_pose_test
