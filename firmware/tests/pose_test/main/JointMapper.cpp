#include "JointMapper.h"

#include <algorithm>
#include <cmath>

namespace tilt_pose_test {
namespace {

bool validJointIndex(int joint_index) {
    return joint_index >= 0 && joint_index < tilt::NUM_JOINTS;
}

}  // namespace

std::uint16_t radToTick(int joint_index, float rad) {
    if (!validJointIndex(joint_index) || !std::isfinite(rad)) {
        return static_cast<std::uint16_t>(tilt::SERVO_POS_MIN);
    }

    const float signed_ticks = rad / tilt::RAD_PER_TICK *
                               static_cast<float>(tilt::JOINT_SIGN[joint_index]);
    const int raw_tick = static_cast<int>(std::lround(signed_ticks)) +
                         static_cast<int>(tilt::ZERO_TICK[joint_index]);
    return static_cast<std::uint16_t>(std::clamp(
        raw_tick, tilt::SERVO_POS_MIN, tilt::SERVO_POS_MAX));
}

float tickToRad(int joint_index, std::uint16_t tick) {
    if (!validJointIndex(joint_index)) {
        return 0.0f;
    }
    return static_cast<float>(static_cast<int>(tick) -
                              static_cast<int>(tilt::ZERO_TICK[joint_index])) *
           tilt::RAD_PER_TICK * static_cast<float>(tilt::JOINT_SIGN[joint_index]);
}

void radArrayToTicks(const float rad[tilt::NUM_JOINTS],
                     std::uint16_t out_ticks[tilt::NUM_JOINTS]) {
    if (rad == nullptr || out_ticks == nullptr) {
        return;
    }
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        out_ticks[joint] = radToTick(joint, rad[joint]);
    }
}

}  // namespace tilt_pose_test
