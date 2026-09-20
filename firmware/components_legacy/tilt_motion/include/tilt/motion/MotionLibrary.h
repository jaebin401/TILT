#pragma once

#include <optional>

#include <tilt/motion/JointPose.h>
#include <tilt/motion/PoseSample.h>

namespace tilt {
namespace motion {

// 임의의 시작자세에서 ADR-008 Home Pose로 복귀하는 첫 표준 Motion이다.
std::optional<PoseSample> sampleReturnHomeMotion(const JointPose& initial_pose,
                                                  float elapsed_seconds,
                                                  float duration_seconds);

}  // namespace motion
}  // namespace tilt
