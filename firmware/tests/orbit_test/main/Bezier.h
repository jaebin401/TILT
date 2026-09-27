#pragma once

#include <algorithm>

namespace tilt_orbit {

inline float swingBezier(float phase, float z_max_mm, float z_neg_mm) {
    const float s = std::clamp(phase, 0.0f, 1.0f);
    const float u = 1.0f - s;
    return z_max_mm * (6.0f * s * u * u * u * u * u +
                       15.0f * s * s * u * u * u * u +
                       20.0f * s * s * s * u * u * u +
                       15.0f * s * s * s * s * u * u) +
           z_neg_mm * s * s * s * s * s * s;
}

// The positive lobe peaks near this fraction for the specified control points.
constexpr float kApproxPeakFraction = 0.875f;

}  // namespace tilt_orbit
