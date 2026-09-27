#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "Bezier.h"
#include "LegPose.h"
#include "OrbitModel.h"

namespace {

void near(float actual, float expected, float relative_tolerance = 0.01f) {
    if (!(std::fabs(actual - expected) <=
          relative_tolerance * std::max(1.0f, std::fabs(expected)))) {
        std::fprintf(stderr, "expected %.6f, got %.6f\n", expected, actual);
        std::abort();
    }
}

}  // namespace

int main() {
    using namespace tilt_orbit;
    const OrbitParams p = computeParams();
    near(p.k_mm, 69.2f);
    near(p.z_eff_mm, 153.4f);
    near(p.lambda, 8.00f);
    near(p.t_star_s, 0.320f);
    near(p.u_mm, 36.4f);
    near(p.d_mm, 54.4f);

    const OrbitPrediction orbit = predict(0.32f);
    near(orbit.roll_amp_deg, 4.6f);
    near(orbit.roll_rate_target_deg_s, 65.0f);
    near(orbit.hip_rise_mm, 4.4f);

    const LegAngles nominal = legForHeight(96.5f, 0.0f);
    assert(nominal.reachable);
    near(-nominal.hip * kRadToDeg, 37.8f, 0.003f);
    near(nominal.hip + nominal.knee + tilt::KNEE_OFFSET_RAD +
         tilt::ANKLE_FIXED_RAD, 0.0f);

    const LegAngles zero = legForHeight(103.97f, 0.0f);
    assert(zero.reachable);
    near(zero.yaw, 0.0f);
    near(zero.hip * kRadToDeg, -20.0f, 0.003f);
    near(zero.knee * kRadToDeg, 20.0f, 0.003f);
    near(footXForHeight(103.97f), 0.0f, 0.03f);
    assert(!legForHeight(200.0f, 0.0f).reachable);

    constexpr float z_max = 6.5f;
    constexpr float z_neg = -1.5f;
    near(swingBezier(0.0f, z_max, z_neg), 0.0f);
    near(swingBezier(1.0f, z_max, z_neg), z_neg);
    near(swingBezier(0.5f, z_max, z_neg),
         0.875f * z_max + z_neg / 64.0f);
    std::puts("orbit model checks passed");
}
