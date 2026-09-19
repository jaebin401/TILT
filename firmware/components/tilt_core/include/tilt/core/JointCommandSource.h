#pragma once

#include <stdint.h>

namespace tilt {
namespace core {

// Safety 로그와 명령 추적에 사용하는 논리 명령 출처다.
enum class JointCommandSource : uint8_t {
    Gait = 1,
    Initialization = 2,
    FaultRecovery = 3,
    PoseTest = 4,
};

}  // namespace core
}  // namespace tilt
