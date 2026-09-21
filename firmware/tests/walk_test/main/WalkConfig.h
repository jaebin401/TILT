#pragma once

#include <cstdint>

#include "tilt_config.h"

namespace tilt_walk_test {

// Control loop: 100 Hz, matching the reference gait loop.
constexpr std::uint32_t kLoopPeriodMs = 10;

// Leg lengths scaled from the Bruton robot's 82--88% extension range.
constexpr float kLongLegDefaultMm = 96.5f;
constexpr float kShortLegDefaultMm = 90.0f;
constexpr float kLegLengthMinMm = 80.0f;
constexpr float kLegLengthMaxMm = 106.0f;
constexpr float kLegLengthStepMm = 0.5f;

constexpr float kLegxDefaultMm = 0.0f;
constexpr float kLegxMaxMm = 15.0f;
constexpr float kLegxStepMm = 1.0f;
constexpr float kStanceBiasRatio = 0.25f;

constexpr std::uint32_t kStepTimeDefaultMs = 250;
constexpr std::uint32_t kLegTimeDefaultMs = 140;
constexpr std::uint32_t kTimeMinMs = 60;
constexpr std::uint32_t kTimeMaxMs = 600;
constexpr std::uint32_t kTimeStepMs = 10;

// Optional IMU rhythm modulation. It is OFF at startup.
constexpr float kStepTimePitchGain = 3.0f;
constexpr float kLegTimePitchGain = 1.0f;
constexpr std::uint32_t kStepTimeModMinMs = 240;
constexpr std::uint32_t kStepTimeModMaxMs = 260;
constexpr std::uint32_t kLegTimeModMinMs = 130;
constexpr std::uint32_t kLegTimeModMaxMs = 150;

// Raw zero means that the STS3215's own speed/acceleration limits are disabled.
constexpr std::uint16_t kServoSpeedRaw = 0;
constexpr std::uint8_t kServoAcceleration = 0;

// Final software barrier, deliberately much looser than normal gait motion.
constexpr float kMaxJointVelocityDegS = 300.0f;
constexpr float kMaxJointStepDeg =
    kMaxJointVelocityDegS * (kLoopPeriodMs / 1000.0f);
constexpr float kMaxJointStepRad = kMaxJointStepDeg * tilt::DEG2RAD;

constexpr std::uint32_t kPoseDurationMs = 2000;
constexpr float kMaxJogStepDeg = 5.0f;

// tilt_kinematics' effective two-link maximum reach (50 + 59.4953 mm).
constexpr float kMaximumReachMm = 109.4953159567f;

}  // namespace tilt_walk_test
