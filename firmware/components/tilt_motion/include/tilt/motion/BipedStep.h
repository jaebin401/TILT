#pragma once

#include <optional>

#include <tilt/motion/LegKinematics.h>
#include <tilt/motion/RobotKinematics.h>

namespace tilt {
namespace motion {

struct RobotAnkleTrajectorySample {
    RobotAnklePositions position;
    RobotAnklePositions velocity_mm_per_sec;
    RobotAnklePositions acceleration_mm_per_sec2;
    bool finished = false;
};

struct BipedStepPoseSample {
    JointPose pose;
    bool finished = false;
};

// support Ankle은 고정하고 swing Ankle만 목표점까지 이동시키는 한 스텝이다.
std::optional<RobotAnkleTrajectorySample> sampleBipedStep(
    const RobotAnklePositions& start,
    LegSide swing_leg,
    const CartesianPoint& swing_target,
    float clearance_mm,
    float elapsed_seconds,
    float duration_seconds);

// Cartesian Step을 IK로 변환해 여섯 논리 관절의 Pose를 반환한다.
std::optional<BipedStepPoseSample> sampleBipedStepPose(
    const RobotAnklePositions& start,
    LegSide swing_leg,
    const CartesianPoint& swing_target,
    float left_hip_yaw_rad,
    float right_hip_yaw_rad,
    float clearance_mm,
    float elapsed_seconds,
    float duration_seconds);

}  // namespace motion
}  // namespace tilt
