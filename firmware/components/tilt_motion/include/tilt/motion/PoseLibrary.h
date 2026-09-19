#pragma once

#include <optional>
#include <stddef.h>

#include <tilt/motion/JointPose.h>

namespace tilt {
namespace motion {

// ADR-008에서 확정한 [0°, -20°, +20°] 기본자세를 반환한다.
const JointPose& homePose();

// Home 외 자세는 하드웨어 검증을 위한 보수적인 초안이다.
enum class PoseId {
    Home,
    Tall,
    Crouch,
    YawLeft,
    YawRight,
};

struct NamedPose {
    PoseId id;
    const char* name;
    const JointPose* pose;
};

const NamedPose* poseLibrary();
std::size_t poseLibrarySize();
const JointPose& poseFor(PoseId id);
std::optional<PoseId> poseIdForName(const char* name);

}  // namespace motion
}  // namespace tilt
