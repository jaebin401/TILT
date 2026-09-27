#include "Interpolator.h"

#include <algorithm>

namespace tilt_pose_test {

void Interpolator::start(const float from[tilt::NUM_JOINTS],
                         const float to[tilt::NUM_JOINTS],
                         std::uint32_t duration_ms,
                         std::uint32_t now_ms) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        from_[joint] = from[joint];
        to_[joint] = to[joint];
    }
    started_ms_ = now_ms;
    duration_ms_ = duration_ms;
    busy_ = duration_ms > 0;
}

bool Interpolator::update(std::uint32_t now_ms, float out[tilt::NUM_JOINTS]) {
    if (out == nullptr) {
        return false;
    }

    const std::uint32_t elapsed_ms = now_ms - started_ms_;
    const float progress = duration_ms_ == 0
                               ? 1.0f
                               : std::min(1.0f, static_cast<float>(elapsed_ms) /
                                                   static_cast<float>(duration_ms_));
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        out[joint] = from_[joint] + (to_[joint] - from_[joint]) * progress;
    }
    busy_ = progress < 1.0f;
    return busy_;
}

bool Interpolator::isBusy() const {
    return busy_;
}

void Interpolator::abort() {
    busy_ = false;
}

}  // namespace tilt_pose_test
