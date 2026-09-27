#pragma once

#include <stdint.h>

namespace tilt {
namespace core {

// Internal, single-bit fault reasons. These are not JointTargetBatch source values.
enum class FaultReason : uint32_t {
    EmergencyStop = 1u << 0,
    ServoCommunication = 1u << 1,  // Reported externally; detection is not implemented here.
};

}  // namespace core
}  // namespace tilt
