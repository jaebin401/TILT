#pragma once

namespace tilt {
namespace core {

// Six logical joint positions in radians, supplied for velocity validation.
// This is not a complete actuator telemetry or joint-state schema.
struct JointPositionReference {
    float angle_rad[6];
};

}  // namespace core
}  // namespace tilt
