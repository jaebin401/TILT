#pragma once

#include <cstdint>

#include "tilt_config.h"

namespace tilt_pose_test {

// The only logical-radian <-> STS3215-raw conversion in this test app.
std::uint16_t radToTick(int joint_index, float rad);
float tickToRad(int joint_index, std::uint16_t tick);
void radArrayToTicks(const float rad[tilt::NUM_JOINTS],
                     std::uint16_t out_ticks[tilt::NUM_JOINTS]);

}  // namespace tilt_pose_test
