#include <tilt/motion/RobotKinematics.h>

#include <tilt/motion/LegKinematics.h>

namespace tilt {
namespace motion {

namespace {

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

std::optional<RobotAnklePositions> forwardRobotAnkleKinematics(
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
    const auto left = forwardAnkleKinematics(
        tiltLegGeometry(), LegSide::Left, left_angles);
    const auto right = forwardAnkleKinematics(
        tiltLegGeometry(), LegSide::Right, right_angles);
    if (!left.has_value() || !right.has_value()) {
        return std::nullopt;
    }
    return RobotAnklePositions{*left, *right};
}

std::optional<JointPose> inverseRobotAnkleKinematics(
    const RobotAnklePositions& targets,
    float left_hip_yaw_rad,
    float right_hip_yaw_rad) {
    const auto left = inverseAnkleKinematics(
        tiltLegGeometry(),
        LegSide::Left,
        targets.left,
        left_hip_yaw_rad);
    const auto right = inverseAnkleKinematics(
        tiltLegGeometry(),
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
