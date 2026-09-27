#pragma once

#include <cstdint>

#include "tilt_config.h"

namespace tilt_rock_test {

constexpr std::uint32_t kLoopPeriodMs = 10;

constexpr float kBaseDefaultMm = 96.5f;
constexpr float kShiftDefaultMm = 3.0f;
constexpr float kLiftDefaultMm = 0.0f;
constexpr float kHeightStepMm = 0.5f;
constexpr float kBaseMinMm = 85.0f;
constexpr float kBaseMaxMm = 104.0f;
constexpr float kShiftMaxMm = 10.0f;
constexpr float kLiftMaxMm = 15.0f;

constexpr float kLeanDefaultDeg = 0.0f;
constexpr float kLeanStepDeg = 0.5f;
constexpr float kLeanMinDeg = -10.0f;
constexpr float kLeanMaxDeg = +25.0f;

constexpr std::uint32_t kShiftDurationDefaultMs = 150;
constexpr std::uint32_t kLiftDurationDefaultMs = 140;
constexpr std::uint32_t kPlantDurationDefaultMs = 140;
constexpr std::uint32_t kLiftHoldDefaultMs = 140;
constexpr std::uint32_t kPlantHoldDefaultMs = 0;
constexpr std::uint32_t kHoldMaxMs = 1000;
constexpr std::uint32_t kDurationStepMs = 10;
constexpr std::uint32_t kDurationMinMs = 40;
constexpr std::uint32_t kDurationMaxMs = 1500;

constexpr float kBrutonLiftDefaultMm = 6.5f;
constexpr std::uint32_t kBrutonStepTimeDefaultMs = 250;
constexpr std::uint32_t kBrutonLegTimeDefaultMs = 140;
constexpr std::uint32_t kBrutonStepTimeMinMs = 60;
constexpr std::uint32_t kBrutonStepTimeMaxMs = 600;
constexpr std::uint32_t kBrutonLegTimeMinMs = 40;
constexpr std::uint32_t kBrutonLegTimeMaxMs = 1500;
constexpr float kBrutonStepPitchGain = 3.0f;
constexpr float kBrutonLegPitchGain = 1.0f;

constexpr std::uint16_t kServoSpeedRaw = 0;
constexpr std::uint8_t kServoAcceleration = 0;
constexpr float kMaxJointVelocityDegS = 400.0f;
constexpr float kMaxJointStepRad =
    kMaxJointVelocityDegS * (kLoopPeriodMs / 1000.0f) * tilt::DEG2RAD;

constexpr std::uint32_t kPoseDurationMs = 2000;

}  // namespace tilt_rock_test
