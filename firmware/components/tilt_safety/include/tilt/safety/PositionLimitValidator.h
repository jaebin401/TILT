#pragma once

#include <tilt/core/JointPositionLimits.h>
#include <tilt/core/JointPositionReference.h>
#include <tilt/core/JointTargetBatch.h>

namespace tilt {
namespace safety {

// Checks the entire six-joint batch without changing it or authorizing motion.
bool validatePositionLimits(const core::JointTargetBatch& batch,
                            const core::JointPositionLimits& limits);

// Uses the same six-joint position rules for a supplied logical reference.
bool validatePositionLimits(const core::JointPositionReference& reference,
                            const core::JointPositionLimits& limits);

}  // namespace safety
}  // namespace tilt
