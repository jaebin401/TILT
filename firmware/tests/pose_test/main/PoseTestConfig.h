#pragma once

#include <cstdint>

#include "tilt_config.h"

namespace tilt_pose_test {

constexpr float kBodyHeightMinMm = 92.0f;
constexpr float kBodyHeightMaxMm = 107.0f;
constexpr float kBodyXMinMm = -20.0f;
constexpr float kBodyXMaxMm = +20.0f;
constexpr float kIkFineStepMm = 0.5f;
constexpr float kIkCoarseStepMm = 2.0f;
constexpr std::uint32_t kIkStepDurationMs = 150;

// Rocking test: differential leg length while body_x remains zero.
constexpr float kRockDeltaMaxMm = 16.0f;
constexpr float kRockFineStepMm = 0.5f;
constexpr float kRockCoarseStepMm = 2.0f;
constexpr std::uint32_t kRockStepDurationMs = 300;
constexpr float kRockSweepStartMm = 0.0f;
constexpr float kRockSweepEndMm = 14.0f;
constexpr float kRockSweepStepMm = 1.0f;
constexpr std::uint32_t kRockSweepHoldMs = 1500;
constexpr std::uint32_t kRockAlternatePeriodMs = 1000;
constexpr std::uint32_t kRockAlternatePeriodMinMs = 300;
constexpr std::uint32_t kRockAlternatePeriodMaxMs = 2000;
constexpr std::uint32_t kRockAlternatePeriodStepMs = 100;
constexpr float kRockRollAbortDeg = 20.0f;
constexpr float kRockRecommendedMarginMm = 1.5f;

constexpr std::uint32_t kPoseDurationMs = 3000;
constexpr std::uint32_t kJointDurationMs = 1000;

constexpr float kMaxJointStepDeg = 35.0f *
                                   (tilt::CONTROL_PERIOD_MS / 1000.0f);
constexpr float kMaxJointStepRad = kMaxJointStepDeg * tilt::DEG2RAD;
constexpr float kMaxJogStepDeg = 5.0f;
constexpr std::uint16_t kServoSpeedRaw = 300;
constexpr std::uint8_t kServoAcceleration = 10;

struct PosePreset {
    const char* name;
    float deg[tilt::NUM_JOINTS];
};

// Only home is an ADR-008-confirmed pose. The others are hardware-test drafts.
constexpr PosePreset kPoses[] = {
    {"home",      {0, -20, +20,  0, -20, +20}},
    {"tall",      {0, -15, +15,  0, -15, +15}},
    {"crouch",    {0, -30, +35,  0, -30, +35}},
    {"yaw-left",  {+5, -20, +20, +5, -20, +20}},
    {"yaw-right", {-5, -20, +20, -5, -20, +20}},
};

}  // namespace tilt_pose_test
