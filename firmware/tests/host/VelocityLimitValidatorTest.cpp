#include <tilt/safety/SafetyController.h>
#include <tilt/safety/VelocityLimitValidator.h>

#include <cstdio>
#include <limits>

namespace {

// Synthetic values for host tests only; these are not TILT hardware limits.
tilt::core::JointVelocityLimits makeTestVelocityLimits() {
    tilt::core::JointVelocityLimits limits{};
    for (unsigned int joint = 0; joint < 6; ++joint) {
        limits.max_rad_per_sec[joint] = 2.0f;
    }
    return limits;
}

tilt::core::JointPositionLimits makeTestPositionLimits() {
    tilt::core::JointPositionLimits limits{};
    for (unsigned int joint = 0; joint < 6; ++joint) {
        limits.min_rad[joint] = -2.0f;
        limits.max_rad[joint] = 2.0f;
    }
    return limits;
}

tilt::core::JointTargetBatch makeTestBatch() {
    tilt::core::JointTargetBatch batch{};
    batch.duration_ms = 1000;
    for (float& angle : batch.angle_rad) {
        angle = 1.0f;
    }
    return batch;
}

bool check(bool condition, const char* case_name) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", case_name);
    }
    return condition;
}

}  // namespace

int main() {
    auto batch = makeTestBatch();
    tilt::core::JointPositionReference reference{};
    auto limits = makeTestVelocityLimits();

    if (!check(tilt::safety::validateVelocityLimits(batch, reference, limits),
               "all six joints within velocity limits")) return 1;

    for (float& maximum : limits.max_rad_per_sec) {
        maximum = 1.0f;
    }
    if (!check(tilt::safety::validateVelocityLimits(batch, reference, limits),
               "required velocity equals maximum")) return 1;

    limits = makeTestVelocityLimits();
    limits.max_rad_per_sec[0] = 0.9999f;
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "slightly above maximum")) return 1;

    limits = makeTestVelocityLimits();
    batch.angle_rad[0] = 100.0f;
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "far above maximum")) return 1;

    batch = makeTestBatch();
    batch.duration_ms = 0;
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "zero duration with movement")) return 1;
    for (float& angle : batch.angle_rad) {
        angle = 0.0f;
    }
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "zero duration without movement")) return 1;

    batch = makeTestBatch();
    batch.duration_ms = 1;
    for (float& angle : batch.angle_rad) {
        angle = 0.001f;
    }
    if (!check(tilt::safety::validateVelocityLimits(batch, reference, limits),
               "one millisecond duration within limits")) return 1;
    batch.angle_rad[0] = 0.01f;
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "one millisecond duration above limits")) return 1;

    batch = makeTestBatch();
    batch.angle_rad[0] = std::numeric_limits<float>::quiet_NaN();
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "NaN target")) return 1;
    batch.angle_rad[0] = std::numeric_limits<float>::infinity();
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "positive infinity target")) return 1;
    batch.angle_rad[0] = -std::numeric_limits<float>::infinity();
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "negative infinity target")) return 1;

    batch = makeTestBatch();
    reference.angle_rad[0] = std::numeric_limits<float>::quiet_NaN();
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "NaN reference")) return 1;
    reference.angle_rad[0] = std::numeric_limits<float>::infinity();
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "positive infinity reference")) return 1;
    reference.angle_rad[0] = -std::numeric_limits<float>::infinity();
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "negative infinity reference")) return 1;

    reference = {};
    limits.max_rad_per_sec[0] = std::numeric_limits<float>::quiet_NaN();
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "NaN velocity limit")) return 1;
    limits.max_rad_per_sec[0] = std::numeric_limits<float>::infinity();
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "positive infinity velocity limit")) return 1;
    limits.max_rad_per_sec[0] = -std::numeric_limits<float>::infinity();
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "negative infinity velocity limit")) return 1;
    limits.max_rad_per_sec[0] = -1.0f;
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "negative velocity limit")) return 1;

    limits = makeTestVelocityLimits();
    limits.max_rad_per_sec[0] = 0.0f;
    batch.angle_rad[0] = 0.0f;
    if (!check(tilt::safety::validateVelocityLimits(batch, reference, limits),
               "zero maximum with zero movement")) return 1;
    batch.angle_rad[0] = 1.0f;
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "zero maximum with movement")) return 1;

    batch = makeTestBatch();
    for (float& angle : batch.angle_rad) {
        angle = 0.0f;
    }
    limits = makeTestVelocityLimits();
    if (!check(tilt::safety::validateVelocityLimits(batch, reference, limits),
               "zero movement with positive duration")) return 1;

    batch = makeTestBatch();
    limits.max_rad_per_sec[5] = 0.5f;
    if (!check(!tilt::safety::validateVelocityLimits(batch, reference, limits),
               "one failing joint rejects the batch")) return 1;

    limits = makeTestVelocityLimits();
    const auto position_limits = makeTestPositionLimits();
    tilt::safety::SafetyController controller(position_limits, limits);
    const auto result = controller.evaluate(batch, reference);
    if (!check(controller.state() == tilt::safety::SafetyState::Disarmed &&
                   result.status == tilt::safety::SafetyResult::Status::Rejected &&
                   !result.approved_batch.has_value(),
               "valid positions and velocities remain rejected while Disarmed")) return 1;

    const auto no_reference = controller.evaluate(batch);
    if (!check(no_reference.status == tilt::safety::SafetyResult::Status::Rejected &&
                   !no_reference.approved_batch.has_value(),
               "missing reference remains rejected")) return 1;

    tilt::safety::SafetyController no_velocity_config(position_limits);
    const auto no_velocity = no_velocity_config.evaluate(batch, reference);
    if (!check(no_velocity.status == tilt::safety::SafetyResult::Status::Rejected &&
                   !no_velocity.approved_batch.has_value(),
               "missing velocity configuration remains rejected")) return 1;

    std::puts("Safety velocity-limit host tests passed");
    return 0;
}
