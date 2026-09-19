#pragma once

namespace tilt {
namespace core {

// Logical joint position bounds in radians, using JointTargetBatch index order.
// Values must be supplied by the caller; no robot limits are defined here.
struct JointPositionLimits {
    float min_rad[6];
    float max_rad[6];
};

}  // namespace core
}  // namespace tilt
