#pragma once

#include <stdint.h>

namespace tilt {
namespace core {

// Shared six-leg-joint command. Angles are logical robot joint angles in radians.
struct JointTargetBatch {
    uint32_t seq;
    uint32_t timestamp_ms;
    uint8_t source;       // GAIT, INIT, or FAULT_RECOVERY; numeric values are not defined yet.
    uint16_t duration_ms;
    float angle_rad[6];   // Logical joint indices below; no servo IDs here.
};

// angle_rad indices:
// 0 LEFT_HIP_YAW,   1 LEFT_HIP_PITCH,   2 LEFT_KNEE_PITCH
// 3 RIGHT_HIP_YAW,  4 RIGHT_HIP_PITCH,  5 RIGHT_KNEE_PITCH

}  // namespace core
}  // namespace tilt
