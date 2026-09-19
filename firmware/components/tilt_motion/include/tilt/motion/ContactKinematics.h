#pragma once

#include <optional>

#include <tilt/motion/CartesianPoint.h>
#include <tilt/motion/JointPose.h>
#include <tilt/motion/LegKinematics.h>

namespace tilt {
namespace motion {

// lower-leg angle이 0일 때 Ankle O4에서 접지점 E까지의 고정 벡터다.
struct FootContactGeometry {
    float forward_mm;
    float up_mm;
};

struct RobotContactPositions {
    CartesianPoint left;
    CartesianPoint right;
};

// ADR-008의 Home Pose와 접지점 좌표로 역산한 임시 기하값이다.
// CAD 실측 후 이 값만 교체한다.
const FootContactGeometry& provisionalFootContactGeometry();

std::optional<CartesianPoint> forwardContactKinematics(
    const LegGeometry& leg_geometry,
    const FootContactGeometry& contact_geometry,
    LegSide side,
    const LegJointAngles& angles);

std::optional<LegJointAngles> inverseContactKinematics(
    const LegGeometry& leg_geometry,
    const FootContactGeometry& contact_geometry,
    LegSide side,
    const CartesianPoint& contact_target,
    float hip_yaw_rad);

std::optional<RobotContactPositions> forwardRobotContactKinematics(
    const JointPose& pose);

std::optional<JointPose> inverseRobotContactKinematics(
    const RobotContactPositions& targets,
    float left_hip_yaw_rad,
    float right_hip_yaw_rad);

}  // namespace motion
}  // namespace tilt
