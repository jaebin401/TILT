#pragma once

#include <optional>

#include <tilt/motion/JointPose.h>
#include <tilt/motion/PoseSample.h>

namespace tilt {
namespace motion {

// 무상태 quintic smootherstep 궤적이다. 위치, 속도, 가속도를 함께 반환한다.
std::optional<PoseSample> samplePoseTransition(const JointPose& start,
                                                const JointPose& target,
                                                float elapsed_seconds,
                                                float duration_seconds);

// 위치만 필요한 호출자를 위한 편의 함수다.
std::optional<JointPose> interpolatePose(const JointPose& start,
                                         const JointPose& target,
                                         float elapsed_seconds,
                                         float duration_seconds);

}  // namespace motion
}  // namespace tilt
