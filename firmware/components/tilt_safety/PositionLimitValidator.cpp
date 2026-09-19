#include <tilt/safety/PositionLimitValidator.h>

#include <cmath>

namespace tilt {
namespace safety {

namespace {

bool validateAngles(const float (&angles)[6],
                    const core::JointPositionLimits& limits) {
    for (unsigned int joint = 0; joint < 6; ++joint) {
        const float target = angles[joint];
        const float minimum = limits.min_rad[joint];
        const float maximum = limits.max_rad[joint];

        if (!std::isfinite(target) || !std::isfinite(minimum) ||
            !std::isfinite(maximum) || minimum > maximum ||
            target < minimum || target > maximum) {
            return false;  // One invalid joint rejects the whole batch.
        }
    }
    return true;
}

}  // namespace

bool validatePositionLimits(const core::JointTargetBatch& batch,
                            const core::JointPositionLimits& limits) {
    return validateAngles(batch.angle_rad, limits);
}

bool validatePositionLimits(const core::JointPositionReference& reference,
                            const core::JointPositionLimits& limits) {
    return validateAngles(reference.angle_rad, limits);
}

}  // namespace safety
}  // namespace tilt
