#pragma once

#include <cstdint>

namespace tilt_orbit {

constexpr float kMassG = 610.8f;
constexpr float kCoMHeightMm = 109.8f;
constexpr float kIrollGmm2 = 2.926e6f;
constexpr float kFootWidthMm = 36.0f;
constexpr float kNominalHeightMm = 96.5f;
constexpr float kTargetLambdaHalfT = 1.28f;
constexpr float kGravityMmS2 = 9810.0f;

// Change these after the suspended check if physical sides or IMU polarity differ.
constexpr bool kSwapLegSides = false;
constexpr std::int8_t kRollSign = +1;

constexpr std::uint32_t kLoopPeriodMs = 10;
constexpr float kLiftZmaxDefaultMm = 6.5f;
constexpr float kLiftZnegMm = -1.5f;
constexpr float kLiftMaxMm = 14.0f;
constexpr float kStartupShiftMm = 3.0f;
constexpr int kStartupMaxCycles = 12;
constexpr float kPushWindow = 0.3f;
constexpr float kPushDefaultMm = 0.0f;
constexpr float kPushMaxMm = 4.0f;
constexpr float kEnergyGainMmPerDegS = 0.02f;
constexpr float kCrossMinRateDegS = 15.0f;
constexpr float kLeanDefaultDeg = 0.0f;
constexpr float kTimeStepMs = 10.0f;
constexpr float kTMinS = 0.20f;
constexpr float kTMaxS = 0.50f;
constexpr std::uint16_t kServoSpeedRaw = 0;
constexpr std::uint8_t kServoAcceleration = 0;
constexpr float kMaxJointVelocityDegS = 400.0f;
constexpr std::uint32_t kStandDurationMs = 2000;
constexpr float kHeightMinMm = 80.0f;
constexpr float kHeightMaxMm = 104.0f;
constexpr float kLeanMinDeg = -10.0f;
constexpr float kLeanMaxDeg = 25.0f;

}  // namespace tilt_orbit
