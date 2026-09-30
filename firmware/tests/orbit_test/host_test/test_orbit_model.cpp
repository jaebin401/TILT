#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "Bezier.h"
#include "GaitController.h"
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

    const LegAngles maximum_push = legForHeight(
        kNominalHeightMm + kPushMaxMm, 0.0f);
    assert(maximum_push.reachable);
    const float pushed_joints[3]{
        maximum_push.yaw, maximum_push.hip, maximum_push.knee};
    for (int leg = 0; leg < 2; ++leg) {
        for (int axis = 0; axis < 3; ++axis) {
            assert(pushed_joints[axis] >=
                   tilt::JOINT_LIMIT[leg][axis].minimum_rad);
            assert(pushed_joints[axis] <=
                   tilt::JOINT_LIMIT[leg][axis].maximum_rad);
        }
    }

    constexpr float z_max = kLiftZmaxDefaultMm;
    constexpr float z_neg = -1.5f;
    near(swingBezier(0.0f, z_max, z_neg), 0.0f);
    near(swingBezier(1.0f, z_max, z_neg), z_neg);
    near(swingBezier(0.5f, z_max, z_neg),
         0.875f * z_max + z_neg / 64.0f);

    GaitController gait;
    assert(gait.selectMode(GaitMode::OPEN));
    gait.setPeriodS(0.32f);
    gait.setPushMm(kPushMaxMm);
    assert(gait.start(0));
    RollState roll{};
    roll.valid = true;
    roll.roll_deg = 5.0f;
    roll.rate_deg_s = 20.0f;
    gait.update(0, roll);
    gait.update(320, roll);
    roll.roll_deg = -5.0f;
    roll.rate_deg_s = -20.0f;
    gait.update(640, roll);
    assert(gait.state() == GaitState::SSP_RIGHT);
    const GaitOutput before_switch = gait.update(950, roll);
    const GaitOutput at_switch = gait.update(960, roll);
    assert(gait.state() == GaitState::SSP_LEFT);
    near(at_switch.left_mm, before_switch.left_mm, 0.00001f);
    near(at_switch.right_mm, before_switch.right_mm, 0.00001f);
    const GaitOutput stance_blend_tick = gait.update(970, roll);
    const float blend_phase = 10.0f / kStanceBlendMs;
    const float current_push = kPushMaxMm * 10.0f / 320.0f;
    const float expected_stance_height = at_switch.left_mm +
        (kNominalHeightMm + current_push - at_switch.left_mm) * blend_phase;
    near(stance_blend_tick.left_mm, expected_stance_height, 0.00001f);

    GaitController orbit_gait;
    assert(orbit_gait.selectMode(GaitMode::ORBIT));
    orbit_gait.setPeriodS(0.32f);
    assert(orbit_gait.start(0));
    roll.roll_deg = 5.0f;
    roll.rate_deg_s = 20.0f;
    orbit_gait.update(0, roll);
    orbit_gait.update(320, roll);
    roll.roll_deg = -5.0f;
    roll.rate_deg_s = -20.0f;
    orbit_gait.update(640, roll);
    assert(orbit_gait.state() == GaitState::SSP_RIGHT);
    roll.rate_deg_s = -8.1f;
    roll.roll_deg = 1.0f;
    orbit_gait.update(700, roll);
    roll.roll_deg = 0.5f;
    orbit_gait.update(710, roll);
    roll.roll_deg = -0.5f;
    const GaitOutput before_orbit_crossing = orbit_gait.update(720, roll);
    roll.roll_deg = -1.0f;
    const GaitOutput quarter_period_crossing = orbit_gait.update(730, roll);
    assert(quarter_period_crossing.event.kind ==
           GaitEventKind::HALF_CYCLE);
    assert(!quarter_period_crossing.event.watchdog);
    assert(quarter_period_crossing.event.elapsed_ms == 90);
    near(quarter_period_crossing.left_mm,
         before_orbit_crossing.left_mm, 0.00001f);
    near(quarter_period_crossing.right_mm,
         before_orbit_crossing.right_mm, 0.00001f);
    std::puts("orbit model checks passed");
}
