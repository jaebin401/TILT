#pragma once

#include <tilt/core/JointPositionReference.h>
#include <tilt/core/JointTargetBatch.h>
#include <tilt/core/JointVelocityLimits.h>

namespace tilt {
namespace safety {

// Checks required average velocity for the entire batch; does not authorize motion.
bool validateVelocityLimits(const core::JointTargetBatch& batch,
                            const core::JointPositionReference& reference,
                            const core::JointVelocityLimits& limits);

}  // namespace safety
}  // namespace tilt
