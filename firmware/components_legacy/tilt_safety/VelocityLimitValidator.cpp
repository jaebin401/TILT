#include <tilt/safety/VelocityLimitValidator.h>

#include <cmath>

namespace tilt {
namespace safety {

bool validateVelocityLimits(const core::JointTargetBatch& batch,
                            const core::JointPositionReference& reference,
                            const core::JointVelocityLimits& limits) {
    if (batch.duration_ms == 0) {
        return false;  // Avoid division by zero, even when no joint moves.
    }

    const float duration_seconds = static_cast<float>(batch.duration_ms) / 1000.0f;
    for (unsigned int joint = 0; joint < 6; ++joint) {
        const float target = batch.angle_rad[joint];
        const float start = reference.angle_rad[joint];
        const float maximum = limits.max_rad_per_sec[joint];

        if (!std::isfinite(target) || !std::isfinite(start) ||
            !std::isfinite(maximum) || maximum < 0.0f) {
            return false;
        }

        const float delta_angle = std::fabs(target - start);
        const float required_average_velocity = delta_angle / duration_seconds;
        if (!std::isfinite(required_average_velocity) ||
            required_average_velocity > maximum) {
            return false;  // One invalid joint rejects the whole batch.
        }
    }
    return true;
}

}  // namespace safety
}  // namespace tilt
