#include <tilt/motion/PoseInterpolator.h>

#include <cmath>

namespace tilt {
namespace motion {

namespace {

PoseSample stationarySample(const JointPose& pose, bool finished) {
    PoseSample sample{};
    sample.pose = pose;
    sample.finished = finished;
    return sample;
}

}  // namespace

std::optional<PoseSample> samplePoseTransition(const JointPose& start,
                                                const JointPose& target,
                                                float elapsed_seconds,
                                                float duration_seconds) {
    if (!isFiniteJointPose(start) || !isFiniteJointPose(target) ||
        !std::isfinite(elapsed_seconds) || !std::isfinite(duration_seconds) ||
        duration_seconds <= 0.0f) {
        return std::nullopt;
    }

    if (elapsed_seconds <= 0.0f) {
        return stationarySample(start, false);
    }
    if (elapsed_seconds >= duration_seconds) {
        return stationarySample(target, true);
    }

    const float normalized_time = elapsed_seconds / duration_seconds;
    const float u2 = normalized_time * normalized_time;
    const float u3 = u2 * normalized_time;
    const float u4 = u3 * normalized_time;
    const float u5 = u4 * normalized_time;

    // 6u^5 - 15u^4 + 10u^3: 양 끝의 속도와 가속도가 모두 0이다.
    const float position_scale = 6.0f * u5 - 15.0f * u4 + 10.0f * u3;
    const float velocity_scale =
        (30.0f * u4 - 60.0f * u3 + 30.0f * u2) / duration_seconds;
    const float acceleration_scale =
        (120.0f * u3 - 180.0f * u2 + 60.0f * normalized_time) /
        (duration_seconds * duration_seconds);

    PoseSample result{};
    for (std::size_t joint = 0; joint < kLegJointCount; ++joint) {
        const float delta = target.angle_rad[joint] - start.angle_rad[joint];
        result.pose.angle_rad[joint] = start.angle_rad[joint] + delta * position_scale;
        result.velocity_rad_per_sec[joint] = delta * velocity_scale;
        result.acceleration_rad_per_sec2[joint] = delta * acceleration_scale;
    }

    if (!isFinitePoseSample(result)) {
        return std::nullopt;
    }
    return result;
}

std::optional<JointPose> interpolatePose(const JointPose& start,
                                         const JointPose& target,
                                         float elapsed_seconds,
                                         float duration_seconds) {
    const auto sample =
        samplePoseTransition(start, target, elapsed_seconds, duration_seconds);
    if (!sample.has_value()) {
        return std::nullopt;
    }
    return sample->pose;
}

}  // namespace motion
}  // namespace tilt
