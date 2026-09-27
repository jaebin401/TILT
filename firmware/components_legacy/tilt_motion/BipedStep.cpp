#include <tilt/motion/BipedStep.h>

#include <tilt/motion/FootTrajectory.h>

namespace tilt {
namespace motion {

std::optional<RobotAnkleTrajectorySample> sampleBipedStep(
    const RobotAnklePositions& start,
    LegSide swing_leg,
    const CartesianPoint& swing_target,
    float clearance_mm,
    float elapsed_seconds,
    float duration_seconds) {
    if (!isFiniteCartesianPoint(start.left) ||
        !isFiniteCartesianPoint(start.right)) {
        return std::nullopt;
    }

    const CartesianPoint& swing_start =
        swing_leg == LegSide::Left ? start.left : start.right;
    const auto swing = sampleFootTrajectory(
        swing_start,
        swing_target,
        clearance_mm,
        elapsed_seconds,
        duration_seconds);
    if (!swing.has_value()) {
        return std::nullopt;
    }

    RobotAnkleTrajectorySample result{};
    result.position = start;
    if (swing_leg == LegSide::Left) {
        result.position.left = swing->position;
        result.velocity_mm_per_sec.left = swing->velocity_mm_per_sec;
        result.acceleration_mm_per_sec2.left =
            swing->acceleration_mm_per_sec2;
    } else {
        result.position.right = swing->position;
        result.velocity_mm_per_sec.right = swing->velocity_mm_per_sec;
        result.acceleration_mm_per_sec2.right =
            swing->acceleration_mm_per_sec2;
    }
    result.finished = swing->finished;
    return result;
}

std::optional<BipedStepPoseSample> sampleBipedStepPose(
    const RobotAnklePositions& start,
    LegSide swing_leg,
    const CartesianPoint& swing_target,
    float left_hip_yaw_rad,
    float right_hip_yaw_rad,
    float clearance_mm,
    float elapsed_seconds,
    float duration_seconds) {
    const auto ankle_sample = sampleBipedStep(
        start,
        swing_leg,
        swing_target,
        clearance_mm,
        elapsed_seconds,
        duration_seconds);
    if (!ankle_sample.has_value()) {
        return std::nullopt;
    }

    const auto pose = inverseRobotAnkleKinematics(
        ankle_sample->position,
        left_hip_yaw_rad,
        right_hip_yaw_rad);
    if (!pose.has_value()) {
        return std::nullopt;
    }
    return BipedStepPoseSample{*pose, ankle_sample->finished};
}

}  // namespace motion
}  // namespace tilt
