#include <tilt/motion/PoseLibrary.h>

#include <cstring>

namespace tilt {
namespace motion {

namespace {

constexpr float kTwentyDegreesInRadians = 0.3490658504f;
constexpr float kFifteenDegreesInRadians = 0.2617993878f;
constexpr float kThirtyDegreesInRadians = 0.5235987756f;
constexpr float kThirtyFiveDegreesInRadians = 0.6108652382f;
constexpr float kFiveDegreesInRadians = 0.0872664626f;

constexpr JointPose kHomePose{{
    0.0f,
    -kTwentyDegreesInRadians,
    kTwentyDegreesInRadians,
    0.0f,
    -kTwentyDegreesInRadians,
    kTwentyDegreesInRadians,
}};

constexpr JointPose kTallPose{{
    0.0f, -kFifteenDegreesInRadians, kFifteenDegreesInRadians,
    0.0f, -kFifteenDegreesInRadians, kFifteenDegreesInRadians,
}};

constexpr JointPose kCrouchPose{{
    0.0f, -kThirtyDegreesInRadians, kThirtyFiveDegreesInRadians,
    0.0f, -kThirtyDegreesInRadians, kThirtyFiveDegreesInRadians,
}};

constexpr JointPose kYawLeftPose{{
    kFiveDegreesInRadians, -kTwentyDegreesInRadians, kTwentyDegreesInRadians,
    kFiveDegreesInRadians, -kTwentyDegreesInRadians, kTwentyDegreesInRadians,
}};

constexpr JointPose kYawRightPose{{
    -kFiveDegreesInRadians, -kTwentyDegreesInRadians, kTwentyDegreesInRadians,
    -kFiveDegreesInRadians, -kTwentyDegreesInRadians, kTwentyDegreesInRadians,
}};

constexpr NamedPose kPoseLibrary[] = {
    {PoseId::Home, "home", &kHomePose},
    {PoseId::Tall, "tall", &kTallPose},
    {PoseId::Crouch, "crouch", &kCrouchPose},
    {PoseId::YawLeft, "yaw-left", &kYawLeftPose},
    {PoseId::YawRight, "yaw-right", &kYawRightPose},
};

}  // namespace

const JointPose& homePose() {
    return kHomePose;
}

const NamedPose* poseLibrary() {
    return kPoseLibrary;
}

std::size_t poseLibrarySize() {
    return sizeof(kPoseLibrary) / sizeof(kPoseLibrary[0]);
}

const JointPose& poseFor(PoseId id) {
    for (const auto& named_pose : kPoseLibrary) {
        if (named_pose.id == id) {
            return *named_pose.pose;
        }
    }
    return kHomePose;
}

std::optional<PoseId> poseIdForName(const char* name) {
    if (name == nullptr) {
        return std::nullopt;
    }
    for (const auto& named_pose : kPoseLibrary) {
        if (std::strcmp(named_pose.name, name) == 0) {
            return named_pose.id;
        }
    }
    return std::nullopt;
}

}  // namespace motion
}  // namespace tilt
