#include <tilt/motion/LegKinematics.h>

#include <algorithm>
#include <cmath>

namespace tilt {
namespace motion {

namespace {

constexpr float kPlaneToleranceMm = 1.0e-4f;
constexpr float kDistanceToleranceMm = 1.0e-4f;

constexpr LegGeometry kTiltLegGeometry{
    36.2f,
    50.0f,
    50.0f,
};

bool isValidGeometry(const LegGeometry& geometry) {
    return std::isfinite(geometry.hip_offset_y_mm) &&
           std::isfinite(geometry.thigh_length_mm) &&
           std::isfinite(geometry.shin_length_mm) &&
           geometry.hip_offset_y_mm >= 0.0f &&
           geometry.thigh_length_mm > 0.0f &&
           geometry.shin_length_mm > 0.0f;
}

bool isFiniteAngles(const LegJointAngles& angles) {
    return std::isfinite(angles.hip_yaw_rad) &&
           std::isfinite(angles.hip_pitch_rad) &&
           std::isfinite(angles.knee_pitch_rad);
}

float hipOffsetForSide(const LegGeometry& geometry, LegSide side) {
    return side == LegSide::Left ? geometry.hip_offset_y_mm
                                 : -geometry.hip_offset_y_mm;
}

}  // namespace

const LegGeometry& tiltLegGeometry() {
    return kTiltLegGeometry;
}

std::optional<CartesianPoint> forwardAnkleKinematics(
    const LegGeometry& geometry,
    LegSide side,
    const LegJointAngles& angles) {
    if (!isValidGeometry(geometry) || !isFiniteAngles(angles)) {
        return std::nullopt;
    }

    const float lower_leg_angle =
        angles.hip_pitch_rad + angles.knee_pitch_rad;
    const float radial_mm =
        -geometry.thigh_length_mm * std::sin(angles.hip_pitch_rad) -
        geometry.shin_length_mm * std::sin(lower_leg_angle);
    const float z_mm =
        -geometry.thigh_length_mm * std::cos(angles.hip_pitch_rad) -
        geometry.shin_length_mm * std::cos(lower_leg_angle);

    CartesianPoint result{
        radial_mm * std::cos(angles.hip_yaw_rad),
        hipOffsetForSide(geometry, side) +
            radial_mm * std::sin(angles.hip_yaw_rad),
        z_mm,
    };
    if (!isFiniteCartesianPoint(result)) {
        return std::nullopt;
    }
    return result;
}

std::optional<LegJointAngles> inverseAnkleKinematics(
    const LegGeometry& geometry,
    LegSide side,
    const CartesianPoint& ankle_target,
    float hip_yaw_rad) {
    if (!isValidGeometry(geometry) ||
        !isFiniteCartesianPoint(ankle_target) ||
        !std::isfinite(hip_yaw_rad)) {
        return std::nullopt;
    }

    const float relative_x = ankle_target.x_mm;
    const float relative_y =
        ankle_target.y_mm - hipOffsetForSide(geometry, side);
    const float yaw_cos = std::cos(hip_yaw_rad);
    const float yaw_sin = std::sin(hip_yaw_rad);

    const float radial_mm = relative_x * yaw_cos + relative_y * yaw_sin;
    const float lateral_residual_mm =
        -relative_x * yaw_sin + relative_y * yaw_cos;
    if (std::fabs(lateral_residual_mm) > kPlaneToleranceMm) {
        return std::nullopt;
    }

    const float down_mm = -ankle_target.z_mm;
    const float backward_mm = -radial_mm;
    const float distance_squared =
        down_mm * down_mm + backward_mm * backward_mm;
    const float maximum_reach =
        geometry.thigh_length_mm + geometry.shin_length_mm;
    const float minimum_reach = std::fabs(
        geometry.thigh_length_mm - geometry.shin_length_mm);
    const float distance = std::sqrt(distance_squared);
    if (!std::isfinite(distance) || distance < kDistanceToleranceMm ||
        distance > maximum_reach + kDistanceToleranceMm ||
        distance < minimum_reach - kDistanceToleranceMm) {
        return std::nullopt;
    }

    const float cosine_knee = std::clamp(
        (distance_squared -
         geometry.thigh_length_mm * geometry.thigh_length_mm -
         geometry.shin_length_mm * geometry.shin_length_mm) /
            (2.0f * geometry.thigh_length_mm * geometry.shin_length_mm),
        -1.0f,
        1.0f);
    const float knee_pitch_rad = std::acos(cosine_knee);
    const float hip_pitch_rad =
        std::atan2(backward_mm, down_mm) -
        std::atan2(geometry.shin_length_mm * std::sin(knee_pitch_rad),
                   geometry.thigh_length_mm +
                       geometry.shin_length_mm * std::cos(knee_pitch_rad));

    LegJointAngles result{
        hip_yaw_rad,
        hip_pitch_rad,
        knee_pitch_rad,
    };
    if (!isFiniteAngles(result)) {
        return std::nullopt;
    }
    return result;
}

}  // namespace motion
}  // namespace tilt
