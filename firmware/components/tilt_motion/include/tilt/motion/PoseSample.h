#pragma once

#include <array>
#include <cmath>

#include <tilt/motion/JointPose.h>

namespace tilt {
namespace motion {

// 한 시각의 목표 위치와 그 해석적 미분값이다.
struct PoseSample {
    JointPose pose;
    std::array<float, kLegJointCount> velocity_rad_per_sec{};
    std::array<float, kLegJointCount> acceleration_rad_per_sec2{};
    bool finished = false;
};

inline bool isFinitePoseSample(const PoseSample& sample) {
    if (!isFiniteJointPose(sample.pose)) {
        return false;
    }

    for (std::size_t joint = 0; joint < kLegJointCount; ++joint) {
        if (!std::isfinite(sample.velocity_rad_per_sec[joint]) ||
            !std::isfinite(sample.acceleration_rad_per_sec2[joint])) {
            return false;
        }
    }
    return true;
}

}  // namespace motion
}  // namespace tilt
