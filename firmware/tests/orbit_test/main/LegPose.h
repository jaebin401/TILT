#pragma once

#include <cmath>

#include "tilt_config.h"

namespace tilt_orbit {

struct LegAngles {
    float yaw = 0.0f;
    float hip = 0.0f;
    float knee = 0.0f;
    bool reachable = false;
};

inline float calfVerticalMm() {
    return tilt::CALF_LENGTH_MM * std::cos(-tilt::ANKLE_FIXED_RAD) +
           tilt::FOOT_LENGTH_MM;
}

inline LegAngles legForHeight(float height_mm, float lean_rad) {
    if (!std::isfinite(height_mm) || !std::isfinite(lean_rad)) return {};
    const float cosine = (height_mm - calfVerticalMm()) /
                         tilt::THIGH_LENGTH_MM;
    if (cosine < -1.0f || cosine > 1.0f) return {};
    const float a = std::acos(cosine);
    const float hip = -a + lean_rad;
    const float knee = -tilt::ANKLE_FIXED_RAD -
                       tilt::KNEE_OFFSET_RAD + a;
    return {0.0f, hip, knee, true};
}

inline float heightFromKnee(float knee_rad) {
    const float a = knee_rad + tilt::ANKLE_FIXED_RAD +
                    tilt::KNEE_OFFSET_RAD;
    return calfVerticalMm() + tilt::THIGH_LENGTH_MM * std::cos(a);
}

inline float footXForHeight(float height_mm) {
    const float cosine = (height_mm - calfVerticalMm()) /
                         tilt::THIGH_LENGTH_MM;
    if (!std::isfinite(cosine) || cosine < -1.0f || cosine > 1.0f) return NAN;
    const float a = std::acos(cosine);
    return tilt::THIGH_LENGTH_MM * std::sin(a) -
           tilt::CALF_LENGTH_MM * std::sin(-tilt::ANKLE_FIXED_RAD);
}

}  // namespace tilt_orbit
