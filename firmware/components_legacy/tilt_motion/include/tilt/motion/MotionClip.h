#pragma once

#include <cstddef>
#include <optional>

#include <tilt/motion/JointPose.h>
#include <tilt/motion/PoseSample.h>

namespace tilt {
namespace motion {

// 이전 자세에서 target까지 이동하는 한 구간이다.
struct PoseKeyframe {
    JointPose target;
    float duration_seconds;
};

// 동적 메모리 없이 기존 키프레임 배열을 바라보는 뷰다.
struct MotionClipView {
    const PoseKeyframe* keyframes = nullptr;
    std::size_t keyframe_count = 0;
};

std::optional<float> motionDurationSeconds(const MotionClipView& clip);

// 전체 동작의 임의 시각에서 위치, 속도, 가속도를 샘플링한다.
std::optional<PoseSample> sampleMotionState(const JointPose& initial_pose,
                                             const MotionClipView& clip,
                                             float elapsed_seconds);

// 위치만 필요한 호출자를 위한 편의 함수다.
std::optional<JointPose> sampleMotion(const JointPose& initial_pose,
                                      const MotionClipView& clip,
                                      float elapsed_seconds);

}  // namespace motion
}  // namespace tilt
