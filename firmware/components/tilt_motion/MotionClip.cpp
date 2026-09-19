#include <tilt/motion/MotionClip.h>

#include <cmath>

#include <tilt/motion/PoseInterpolator.h>

namespace tilt {
namespace motion {

std::optional<float> motionDurationSeconds(const MotionClipView& clip) {
    if (clip.keyframe_count > 0 && clip.keyframes == nullptr) {
        return std::nullopt;
    }

    float total_seconds = 0.0f;
    for (std::size_t index = 0; index < clip.keyframe_count; ++index) {
        const PoseKeyframe& keyframe = clip.keyframes[index];
        if (!isFiniteJointPose(keyframe.target) ||
            !std::isfinite(keyframe.duration_seconds) ||
            keyframe.duration_seconds <= 0.0f) {
            return std::nullopt;
        }

        total_seconds += keyframe.duration_seconds;
        if (!std::isfinite(total_seconds)) {
            return std::nullopt;
        }
    }
    return total_seconds;
}

std::optional<PoseSample> sampleMotionState(const JointPose& initial_pose,
                                             const MotionClipView& clip,
                                             float elapsed_seconds) {
    const auto total_duration = motionDurationSeconds(clip);
    if (!isFiniteJointPose(initial_pose) || !std::isfinite(elapsed_seconds) ||
        !total_duration.has_value()) {
        return std::nullopt;
    }

    if (clip.keyframe_count == 0) {
        PoseSample result{};
        result.pose = initial_pose;
        result.finished = true;
        return result;
    }

    if (elapsed_seconds >= *total_duration) {
        PoseSample result{};
        result.pose = clip.keyframes[clip.keyframe_count - 1].target;
        result.finished = true;
        return result;
    }

    float remaining_seconds = elapsed_seconds;
    JointPose segment_start = initial_pose;

    for (std::size_t index = 0; index < clip.keyframe_count; ++index) {
        const PoseKeyframe& keyframe = clip.keyframes[index];
        if (remaining_seconds <= keyframe.duration_seconds) {
            auto result = samplePoseTransition(segment_start,
                                               keyframe.target,
                                               remaining_seconds,
                                               keyframe.duration_seconds);
            if (result.has_value()) {
                result->finished = false;
            }
            return result;
        }

        remaining_seconds -= keyframe.duration_seconds;
        segment_start = keyframe.target;
    }

    return std::nullopt;
}

std::optional<JointPose> sampleMotion(const JointPose& initial_pose,
                                      const MotionClipView& clip,
                                      float elapsed_seconds) {
    const auto sample = sampleMotionState(initial_pose, clip, elapsed_seconds);
    if (!sample.has_value()) {
        return std::nullopt;
    }
    return sample->pose;
}

}  // namespace motion
}  // namespace tilt
