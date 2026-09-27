#pragma once

#include <optional>

#include <tilt/motion/CartesianPoint.h>
#include <tilt/motion/JointPose.h>

namespace tilt {
namespace motion {

struct RobotAnklePositions {
    CartesianPoint left;
    CartesianPoint right;
};

std::optional<RobotAnklePositions> forwardRobotAnkleKinematics(
    const JointPose& pose);

std::optional<JointPose> inverseRobotAnkleKinematics(
    const RobotAnklePositions& targets,
    float left_hip_yaw_rad,
    float right_hip_yaw_rad);

}  // namespace motion
}  // namespace tilt
