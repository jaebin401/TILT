#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "ParallelLeg.h"

namespace {

constexpr float kRadToDeg = 57.2957795131f;

bool near(float actual, float expected, float tolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

}  // namespace

int main() {
    using tilt_rock_test::parallelLeg;
    using tilt_rock_test::parallelStance;

    assert(near(tilt_rock_test::calfVerticalMm(), 56.985f, 0.002f));
    assert(near(tilt_rock_test::calfHorizontalMm(), -17.101f, 0.002f));

    struct Expected {
        float height_mm;
        float angle_deg;
        float foot_x_mm;
    };
    const Expected expected[] = {
        {tilt::ZERO_POSE_HEIGHT_MM, 20.0f, 0.0f},
        {100.0f, 30.7f, 8.4f},
        {96.5f, 37.8f, 13.5f},
        {93.0f, 43.9f, 17.6f},
        {90.0f, 48.7f, 20.5f},
    };

    for (const Expected& row : expected) {
        const auto pose = parallelLeg(row.height_mm, 0.0f);
        assert(pose.reachable);
        assert(near(pose.thigh_angle_rad * kRadToDeg,
                    row.angle_deg, 0.15f));
        assert(near(pose.foot_x_mm, row.foot_x_mm, 0.15f));
        assert(near(pose.hip_pitch_rad + pose.knee_pitch_rad +
                        tilt::KNEE_OFFSET_RAD + tilt::ANKLE_FIXED_RAD,
                    0.0f, 0.00001f));

        float joints[tilt::NUM_JOINTS]{};
        assert(parallelStance(row.height_mm, row.height_mm, 0.0f, joints));
        assert(near(joints[tilt::L_HIP_YAW], 0.0f, 0.00001f));
        assert(near(joints[tilt::R_HIP_YAW], 0.0f, 0.00001f));
    }

    // Torso lean changes hip pitch. Subtracting that torso rotation keeps the
    // foot horizontal in world coordinates.
    for (float height_mm : {90.0f, 93.0f, 96.5f, 100.0f,
                            tilt::ZERO_POSE_HEIGHT_MM}) {
        const float lean = 4.0f * tilt::DEG2RAD;
        const auto pose = parallelLeg(height_mm, lean);
        assert(pose.reachable);
        assert(near(pose.hip_pitch_rad + pose.knee_pitch_rad +
                        tilt::KNEE_OFFSET_RAD + tilt::ANKLE_FIXED_RAD - lean,
                    0.0f, 0.00001f));
    }

    assert(!parallelLeg(120.0f, 0.0f).reachable);
    assert(!parallelLeg(-100.0f, 0.0f).reachable);
    float joints[tilt::NUM_JOINTS]{};
    assert(!parallelStance(120.0f, 90.0f, 0.0f, joints));
    std::puts("parallel leg geometry: OK");
}
