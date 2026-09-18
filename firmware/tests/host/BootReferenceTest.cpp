#include <tilt/safety/SafetyController.h>

#include <cmath>
#include <cstdio>
#include <limits>

namespace {

// Synthetic host-test limits in radians; these are not TILT hardware values.
tilt::core::JointPositionLimits makeTestLimits() {
    tilt::core::JointPositionLimits limits{};
    for (unsigned int joint = 0; joint < 6; ++joint) {
        limits.min_rad[joint] = -1.0f;
        limits.max_rad[joint] = 1.0f;
    }
    return limits;
}

tilt::core::JointVelocityLimits makeTestVelocityLimits() {
    tilt::core::JointVelocityLimits limits{};
    for (unsigned int joint = 0; joint < 6; ++joint) {
        limits.max_rad_per_sec[joint] = 1.0f;
    }
    return limits;
}

bool check(bool condition, const char* case_name) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", case_name);
    }
    return condition;
}

bool rejectedWithoutBatch(const tilt::safety::SafetyResult& result) {
    return result.status == tilt::safety::SafetyResult::Status::Rejected &&
           !result.approved_batch.has_value();
}

}  // namespace

int main() {
    using tilt::core::FaultReason;
    using tilt::safety::RecoveryResult;
    using tilt::safety::SafetyState;

    const auto limits = makeTestLimits();
    tilt::core::JointPositionReference valid{};
    tilt::safety::SafetyController controller(limits, makeTestVelocityLimits());

    if (!check(controller.state() == SafetyState::Disarmed &&
                   !controller.hasBootReference(),
               "new controller is Disarmed without a boot reference")) return 1;
    if (!check(controller.initializeBootReference(valid) &&
                   controller.hasBootReference() &&
                   controller.state() == SafetyState::Disarmed,
               "valid six-joint reference is saved without arming")) return 1;

    tilt::safety::SafetyController no_limits;
    if (!check(!no_limits.initializeBootReference(valid) &&
                   !no_limits.hasBootReference() &&
                   no_limits.state() == SafetyState::Disarmed,
               "missing position limits reject initialization")) return 1;

    const float invalid_values[] = {
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
    };
    for (float value : invalid_values) {
        tilt::core::JointPositionReference invalid{};
        invalid.angle_rad[2] = value;
        tilt::safety::SafetyController fresh(limits);
        if (!check(!fresh.initializeBootReference(invalid) &&
                       !fresh.hasBootReference() &&
                       fresh.state() == SafetyState::Disarmed,
                   "NaN and either infinity reject a reference")) return 1;
        if (!check(!controller.initializeBootReference(invalid) &&
                       controller.hasBootReference(),
                   "invalid input does not replace an existing reference")) return 1;
    }

    auto invalid_limits = limits;
    invalid_limits.min_rad[1] = std::numeric_limits<float>::quiet_NaN();
    if (!check(!tilt::safety::SafetyController(invalid_limits)
                    .initializeBootReference(valid),
               "NaN limit rejects initialization")) return 1;
    invalid_limits = limits;
    invalid_limits.max_rad[1] = std::numeric_limits<float>::infinity();
    if (!check(!tilt::safety::SafetyController(invalid_limits)
                    .initializeBootReference(valid),
               "infinite limit rejects initialization")) return 1;
    invalid_limits = limits;
    invalid_limits.min_rad[1] = 2.0f;
    if (!check(!tilt::safety::SafetyController(invalid_limits)
                    .initializeBootReference(valid),
               "min above max rejects initialization")) return 1;

    tilt::core::JointPositionReference outside{};
    outside.angle_rad[4] = 1.001f;
    tilt::safety::SafetyController out_of_range(limits);
    if (!check(!out_of_range.initializeBootReference(outside) &&
                   !out_of_range.hasBootReference() &&
                   out_of_range.state() == SafetyState::Disarmed &&
                   !out_of_range.hasFault(FaultReason::ServoCommunication),
               "out-of-range reference fails without creating Fault")) return 1;
    if (!check(!controller.initializeBootReference(outside) &&
                   controller.hasBootReference(),
               "out-of-range input does not replace an existing reference")) return 1;

    tilt::core::JointPositionReference boundary{};
    for (unsigned int joint = 0; joint < 6; ++joint) {
        boundary.angle_rad[joint] = limits.min_rad[joint];
    }
    if (!check(tilt::safety::SafetyController(limits)
                   .initializeBootReference(boundary),
               "exact minimum boundary is valid")) return 1;
    for (unsigned int joint = 0; joint < 6; ++joint) {
        boundary.angle_rad[joint] = limits.max_rad[joint];
    }
    if (!check(tilt::safety::SafetyController(limits)
                   .initializeBootReference(boundary),
               "exact maximum boundary is valid")) return 1;

    tilt::core::JointTargetBatch batch{};
    batch.duration_ms = 1000;
    if (!check(rejectedWithoutBatch(controller.evaluate(batch)) &&
                   rejectedWithoutBatch(controller.evaluate(batch, valid)) &&
                   controller.state() == SafetyState::Disarmed,
               "boot reference never authorizes motion or replaces runtime reference")) return 1;

    controller.reportFault(FaultReason::EmergencyStop);
    if (!check(controller.state() == SafetyState::Fault &&
                   !controller.hasBootReference() &&
                   !controller.initializeBootReference(valid),
               "any reported Fault invalidates reference and blocks reinitialization")) return 1;
    controller.reportFaultConditionCleared(FaultReason::EmergencyStop);
    if (!check(controller.requestFaultRecovery() == RecoveryResult::RecoveredToDisarmed &&
                   controller.state() == SafetyState::Disarmed &&
                   !controller.hasBootReference(),
               "recovery does not restore old boot snapshot")) return 1;
    if (!check(controller.initializeBootReference(valid) &&
                   controller.hasBootReference(),
               "fresh reference can be initialized after recovery")) return 1;

    controller.reportFault(FaultReason::ServoCommunication);
    if (!check(!controller.hasBootReference(),
               "non-E-STOP Fault also invalidates reference")) return 1;

    std::puts("Safety boot-reference host tests passed");
    return 0;
}
