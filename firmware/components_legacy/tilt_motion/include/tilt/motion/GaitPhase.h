#pragma once

#include <optional>

#include <tilt/motion/LegKinematics.h>

namespace tilt {
namespace motion {

enum class SupportMode {
    Double,
    Left,
    Right,
};

struct GaitTiming {
    float double_support_seconds;
    float swing_seconds;
};

struct GaitPhaseSample {
    SupportMode support_mode;
    std::optional<LegSide> swing_leg;
    LegSide next_swing_leg;
    float phase_progress;
};

std::optional<float> gaitCycleDurationSeconds(const GaitTiming& timing);

// 한 cycle은 Double Support → 첫 Swing → Double Support → 반대 Swing 순서다.
std::optional<GaitPhaseSample> sampleGaitPhase(
    const GaitTiming& timing,
    LegSide first_swing_leg,
    float elapsed_seconds);

}  // namespace motion
}  // namespace tilt
