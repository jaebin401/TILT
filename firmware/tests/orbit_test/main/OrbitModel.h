#pragma once

#include <cmath>

#include "OrbitConfig.h"
#include "tilt_config.h"

namespace tilt_orbit {

constexpr float kRadToDeg = 1.0f / tilt::DEG2RAD;

struct OrbitParams {
    float k_mm, z_eff_mm, lambda, t_star_s;
    float u_mm, d_mm;
};

struct OrbitPrediction {
    float t_s, lambda_half_t;
    float sigma2, p_star_mm, v_star_mm_s, p_mid_mm;
    float roll_amp_deg, roll_rate_target_deg_s, hip_rise_mm;
};

inline OrbitParams computeParams() {
    const float k = std::sqrt(kIrollGmm2 / kMassG);
    const float z_eff = kCoMHeightMm + k * k / kCoMHeightMm;
    const float lambda = std::sqrt(kGravityMmS2 / z_eff);
    return {k, z_eff, lambda, 2.0f * kTargetLambdaHalfT / lambda,
            2.0f * (tilt::Y_HIP_MM - kFootWidthMm / 2.0f),
            2.0f * tilt::Y_HIP_MM - kFootWidthMm / 2.0f};
}

inline OrbitPrediction predict(float t_s) {
    const OrbitParams p = computeParams();
    const float half = p.lambda * t_s / 2.0f;
    const float sigma2 = p.lambda * std::tanh(half);
    const float p_star = p.u_mm / 2.0f;
    const float v_star = sigma2 * p_star;
    const float p_mid = p_star / std::cosh(half);
    const float delta_p = p_star - p_mid;
    return {t_s, half, sigma2, p_star, v_star, p_mid,
            std::atan(delta_p / kCoMHeightMm) * kRadToDeg,
            v_star / kCoMHeightMm * kRadToDeg,
            p.d_mm * delta_p / kCoMHeightMm};
}

}  // namespace tilt_orbit
