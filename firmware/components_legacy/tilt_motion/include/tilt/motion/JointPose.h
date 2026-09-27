#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace tilt {
namespace motion {

inline constexpr std::size_t kLegJointCount = 6;

// Safety의 JointTargetBatch와 같은 논리 관절 순서다.
enum class JointIndex : std::size_t {
    LeftHipYaw = 0,
    LeftHipPitch,
    LeftKneePitch,
    RightHipYaw,
    RightHipPitch,
    RightKneePitch,
};

// 하드웨어 표현과 분리된 여섯 다리 관절의 논리 각도다.
struct JointPose {
    std::array<float, kLegJointCount> angle_rad{};
};

inline bool isFiniteJointPose(const JointPose& pose) {
    for (const float angle : pose.angle_rad) {
        if (!std::isfinite(angle)) {
            return false;
        }
    }
    return true;
}

}  // namespace motion
}  // namespace tilt
