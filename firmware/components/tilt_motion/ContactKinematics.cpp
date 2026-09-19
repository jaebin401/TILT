#include <tilt/motion/ContactKinematics.h>

#include <algorithm>
#include <cmath>

namespace tilt {
namespace motion {

namespace {

constexpr float kPlaneToleranceMm = 1.0e-4f;
constexpr float kDistanceToleranceMm = 1.0e-4f;

// Home Pose의 O4=(17.101007, ±36.2, -96.984634)와
// E=(0, ±36.2, -104)를 만족하도록 역산했다.
constexpr FootContactGeometry kProvisionalFootContactGeometry{
    -17.101007f,
    -7.015366f,
};

bool isValidLegGeometry(const LegGeometry& geometry) {
    return std::isfinite(geometry.hip_offset_y_mm) &&
           std::isfinite(geometry.thigh_length_mm) &&
           std::isfinite(geometry.shin_length_mm) &&
           geometry.hip_offset_y_mm >= 0.0f &&
           geometry.thigh_length_mm > 0.0f &&
           geometry.shin_length_mm > 0.0f;
}

bool isValidContactGeometry(const FootContactGeometry& geometry) {
    return std::isfinite(geometry.forward_mm) &&
           std::isfinite(geometry.up_mm);
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

std::size_t jointIndex(JointIndex joint) {
    return static_cast<std::size_t>(joint);
}

LegJointAngles legAngles(const JointPose& pose,
                         JointIndex hip_yaw,
                         JointIndex hip_pitch,
                         JointIndex knee_pitch) {
    return {
        pose.angle_rad[jointIndex(hip_yaw)],
        pose.angle_rad[jointIndex(hip_pitch)],
        pose.angle_rad[jointIndex(knee_pitch)],
    };
}

void setLegAngles(JointPose& pose,
                  JointIndex hip_yaw,
                  JointIndex hip_pitch,
                  JointIndex knee_pitch,
                  const LegJointAngles& angles) {
    pose.angle_rad[jointIndex(hip_yaw)] = angles.hip_yaw_rad;
    pose.angle_rad[jointIndex(hip_pitch)] = angles.hip_pitch_rad;
    pose.angle_rad[jointIndex(knee_pitch)] = angles.knee_pitch_rad;
}

}  // namespace

const FootContactGeometry& provisionalFootContactGeometry() {
    return kProvisionalFootContactGeometry;
}

std::optional<CartesianPoint> forwardContactKinematics(
    const LegGeometry& leg_geometry,
    const FootContactGeometry& contact_geometry,
    LegSide side,
    const LegJointAngles& angles) {
    if (!isValidLegGeometry(leg_geometry) ||
        !isValidContactGeometry(contact_geometry) ||
        !isFiniteAngles(angles)) {
        return std::nullopt;
    }

    const auto ankle = forwardAnkleKinematics(leg_geometry, side, angles);
    if (!ankle.has_value()) {
        return std::nullopt;
    }

    const float lower_leg_angle =
        angles.hip_pitch_rad + angles.knee_pitch_rad;
    const float contact_radial =
        contact_geometry.forward_mm * std::cos(lower_leg_angle) +
        contact_geometry.up_mm * std::sin(lower_leg_angle);
    const float contact_z =
        -contact_geometry.forward_mm * std::sin(lower_leg_angle) +
        contact_geometry.up_mm * std::cos(lower_leg_angle);

    CartesianPoint result{
        ankle->x_mm + contact_radial * std::cos(angles.hip_yaw_rad),
        ankle->y_mm + contact_radial * std::sin(angles.hip_yaw_rad),
        ankle->z_mm + contact_z,
    };
    if (!isFiniteCartesianPoint(result)) {
        return std::nullopt;
    }
    return result;
}

std::optional<LegJointAngles> inverseContactKinematics(
    const LegGeometry& leg_geometry,
    const FootContactGeometry& contact_geometry,
    LegSide side,
    const CartesianPoint& contact_target,
    float hip_yaw_rad) {
    if (!isValidLegGeometry(leg_geometry) ||
        !isValidContactGeometry(contact_geometry) ||
        !isFiniteCartesianPoint(contact_target) ||
        !std::isfinite(hip_yaw_rad)) {
        return std::nullopt;
    }

    const float relative_x = contact_target.x_mm;
    const float relative_y =
        contact_target.y_mm - hipOffsetForSide(leg_geometry, side);
    const float yaw_cos = std::cos(hip_yaw_rad);
    const float yaw_sin = std::sin(hip_yaw_rad);
    const float radial_mm = relative_x * yaw_cos + relative_y * yaw_sin;
    const float lateral_residual_mm =
        -relative_x * yaw_sin + relative_y * yaw_cos;
    if (std::fabs(lateral_residual_mm) > kPlaneToleranceMm) {
        return std::nullopt;
    }

    // Knee부터 접지점까지의 고정 벡터를 하나의 유효 링크로 합친다.
    const float distal_forward_mm = contact_geometry.forward_mm;
    const float distal_up_mm =
        -leg_geometry.shin_length_mm + contact_geometry.up_mm;
    const float distal_length_mm = std::hypot(
        distal_forward_mm, distal_up_mm);
    if (!std::isfinite(distal_length_mm) ||
        distal_length_mm <= kDistanceToleranceMm) {
        return std::nullopt;
    }
    const float distal_angle_offset_rad =
        std::atan2(-distal_forward_mm, -distal_up_mm);

    const float down_mm = -contact_target.z_mm;
    const float backward_mm = -radial_mm;
    const float distance_squared =
        down_mm * down_mm + backward_mm * backward_mm;
    const float distance = std::sqrt(distance_squared);
    const float maximum_reach =
        leg_geometry.thigh_length_mm + distal_length_mm;
    const float minimum_reach = std::fabs(
        leg_geometry.thigh_length_mm - distal_length_mm);
    if (!std::isfinite(distance) || distance < kDistanceToleranceMm ||
        distance > maximum_reach + kDistanceToleranceMm ||
        distance < minimum_reach - kDistanceToleranceMm) {
        return std::nullopt;
    }

    const float cosine_effective_knee = std::clamp(
        (distance_squared -
         leg_geometry.thigh_length_mm * leg_geometry.thigh_length_mm -
         distal_length_mm * distal_length_mm) /
            (2.0f * leg_geometry.thigh_length_mm * distal_length_mm),
        -1.0f,
        1.0f);
    const float effective_knee_rad = std::acos(cosine_effective_knee);
    const float hip_pitch_rad =
        std::atan2(backward_mm, down_mm) -
        std::atan2(distal_length_mm * std::sin(effective_knee_rad),
                   leg_geometry.thigh_length_mm +
                       distal_length_mm * std::cos(effective_knee_rad));
    const float knee_pitch_rad =
        effective_knee_rad - distal_angle_offset_rad;

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

std::optional<RobotContactPositions> forwardRobotContactKinematics(
    const JointPose& pose) {
    if (!isFiniteJointPose(pose)) {
        return std::nullopt;
    }

    const LegJointAngles left_angles = legAngles(
        pose,
        JointIndex::LeftHipYaw,
        JointIndex::LeftHipPitch,
        JointIndex::LeftKneePitch);
    const LegJointAngles right_angles = legAngles(
        pose,
        JointIndex::RightHipYaw,
        JointIndex::RightHipPitch,
        JointIndex::RightKneePitch);
    const auto left = forwardContactKinematics(
        tiltLegGeometry(),
        provisionalFootContactGeometry(),
        LegSide::Left,
        left_angles);
    const auto right = forwardContactKinematics(
        tiltLegGeometry(),
        provisionalFootContactGeometry(),
        LegSide::Right,
        right_angles);
    if (!left.has_value() || !right.has_value()) {
        return std::nullopt;
    }
    return RobotContactPositions{*left, *right};
}

std::optional<JointPose> inverseRobotContactKinematics(
    const RobotContactPositions& targets,
    float left_hip_yaw_rad,
    float right_hip_yaw_rad) {
    const auto left = inverseContactKinematics(
        tiltLegGeometry(),
        provisionalFootContactGeometry(),
        LegSide::Left,
        targets.left,
        left_hip_yaw_rad);
    const auto right = inverseContactKinematics(
        tiltLegGeometry(),
        provisionalFootContactGeometry(),
        LegSide::Right,
        targets.right,
        right_hip_yaw_rad);
    if (!left.has_value() || !right.has_value()) {
        return std::nullopt;
    }

    JointPose result{};
    setLegAngles(
        result,
        JointIndex::LeftHipYaw,
        JointIndex::LeftHipPitch,
        JointIndex::LeftKneePitch,
        *left);
    setLegAngles(
        result,
        JointIndex::RightHipYaw,
        JointIndex::RightHipPitch,
        JointIndex::RightKneePitch,
        *right);
    return result;
}

}  // namespace motion
}  // namespace tilt
