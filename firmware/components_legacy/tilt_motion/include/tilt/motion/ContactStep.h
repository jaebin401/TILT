#pragma once

#include <optional>

#include <tilt/motion/ContactKinematics.h>

namespace tilt {
namespace motion {

struct RobotContactTrajectorySample {
    RobotContactPositions position;
    RobotContactPositions velocity_mm_per_sec;
    RobotContactPositions acceleration_mm_per_sec2;
    bool finished = false;
};

struct ContactStepPoseSample {
    JointPose pose;
    bool finished = false;
};

std::optional<RobotContactTrajectorySample> sampleContactStep(
    const RobotContactPositions& start,
    LegSide swing_leg,
    const CartesianPoint& swing_target,
    float clearance_mm,
    float elapsed_seconds,
    float duration_seconds);

std::optional<ContactStepPoseSample> sampleContactStepPose(
    const RobotContactPositions& start,
    LegSide swing_leg,
    const CartesianPoint& swing_target,
    float left_hip_yaw_rad,
    float right_hip_yaw_rad,
    float clearance_mm,
    float elapsed_seconds,
    float duration_seconds);

}  // namespace motion
}  // namespace tilt
