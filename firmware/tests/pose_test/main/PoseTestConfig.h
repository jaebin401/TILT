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
constexpr float kRockDeltaMaxMm = 24.0f;
constexpr float kRockFineStepMm = 0.5f;
constexpr float kRockCoarseStepMm = 2.0f;
constexpr std::uint32_t kRockStepDurationMs = 300;
constexpr float kRockSweepStartMm = 0.0f;
constexpr float kRockSweepEndMm = 14.0f;
constexpr float kRockSweepStepMm = 1.0f;
constexpr std::uint32_t kRockSweepHoldMs = 1500;
constexpr std::uint32_t kRockAlternatePeriodMs = 1000;
constexpr std::uint32_t kRockAlternatePeriodMinMs = 150;
constexpr std::uint32_t kRockAlternatePeriodMaxMs = 2000;
constexpr std::uint32_t kRockAlternatePeriodStepMs = 50;
constexpr float kRockRollAbortDeg = 20.0f;
constexpr float kRockRecommendedMarginMm = 1.5f;

constexpr std::uint32_t kPoseDurationMs = 3000;
constexpr std::uint32_t kJointDurationMs = 1000;

constexpr float kMaxJointVelocityDegS = 180.0f;
constexpr float kDurationSafetyFactor = 1.3f;
constexpr std::uint32_t kMinMoveDurationMs = 60;
constexpr float kMaxJointStepDeg = kMaxJointVelocityDegS *
                                   (tilt::CONTROL_PERIOD_MS / 1000.0f);
constexpr float kMaxJointStepRad = kMaxJointStepDeg * tilt::DEG2RAD;
constexpr float kMaxJogStepDeg = 5.0f;
constexpr std::uint16_t kServoSpeedRaw = 0;
constexpr std::uint8_t kServoAcceleration = 0;

// IMU compensation shared by stand and rocking modes.
constexpr float kCompKpPitchDefault = 2.0f;
constexpr float kCompKpPitchMax = 10.0f;
constexpr float kCompKpPitchStep = 0.5f;
constexpr float kCompBodyXMaxMm = 15.0f;
constexpr float kCompLpfAlpha = 0.30f;
constexpr float kCompLpfAlphaStep = 0.05f;

// Slow, cycle-boundary roll compensation for rocking mode.
constexpr float kCompKrCenterDefault = 0.3f;
constexpr float kCompDeltaBiasMaxMm = 4.0f;
constexpr float kCompRollTargetAmpDeg = 4.0f;
constexpr float kCompKaAmpDefault = 0.05f;
constexpr float kCompDeltaScaleMax = 1.8f;

constexpr float kCompAbortPitchDeg = 20.0f;
constexpr float kCompAbortRollDeg = 20.0f;

constexpr float kStepTestBodyXMm = 8.0f;
constexpr std::uint32_t kStepTestHoldMs = 500;
constexpr std::uint32_t kStepTestRecordMs = 2000;
constexpr float kStepTestSettleDeg = 1.0f;

constexpr std::uint32_t kStatusPrintPeriodMs = 200;

// Forward walk experiment.
constexpr float kWalkLegxDefaultMm = 5.0f;
constexpr float kWalkLegxMaxMm = 20.0f;
constexpr float kWalkLegxStepMm = 1.0f;
constexpr float kWalkDeltaDefaultMm = 14.0f;
constexpr std::uint32_t kWalkLegTimeDefaultMs = 150;
constexpr std::uint32_t kWalkLegTimeMinMs = 60;
constexpr std::uint32_t kWalkLegTimeMaxMs = 500;
constexpr std::uint32_t kWalkLegTimeStepMs = 10;
constexpr std::uint32_t kWalkMaxConsecutiveRejects = 3;

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
