#include <tilt/safety/SafetyController.h>

#include <cstdio>
#include <stdint.h>

namespace {

// Synthetic host-test values, not measured TILT hardware limits.
tilt::core::JointPositionLimits makeTestPositionLimits() {
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

tilt::core::JointTargetBatch makeTestBatch() {
    tilt::core::JointTargetBatch batch{};
    batch.duration_ms = 1000;
    return batch;
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
    using tilt::safety::SafetyState;

    tilt::safety::SafetyController controller;
    if (!check(controller.state() == SafetyState::Disarmed &&
                   !controller.hasFault(FaultReason::EmergencyStop) &&
                   !controller.hasFault(FaultReason::ServoCommunication),
               "initial state has no Fault reasons")) return 1;

    const auto zero_reason = static_cast<FaultReason>(0u);
    const auto composite_reason = static_cast<FaultReason>(
        static_cast<uint32_t>(FaultReason::EmergencyStop) |
        static_cast<uint32_t>(FaultReason::ServoCommunication));
    const auto undefined_reason = static_cast<FaultReason>(uint32_t{1} << 2);
    const FaultReason invalid_reasons[] = {
        zero_reason, composite_reason, undefined_reason,
    };

    for (FaultReason invalid : invalid_reasons) {
        controller.reportFault(invalid);
        if (!check(controller.state() == SafetyState::Disarmed &&
                       !controller.hasFault(invalid) &&
                       !controller.hasFault(FaultReason::EmergencyStop) &&
                       !controller.hasFault(FaultReason::ServoCommunication),
                   "zero, composite, and undefined Fault reasons are ignored")) return 1;
    }

    tilt::safety::SafetyController with_boot_reference(makeTestPositionLimits());
    tilt::core::JointPositionReference boot_reference{};
    if (!check(with_boot_reference.initializeBootReference(boot_reference) &&
                   with_boot_reference.hasBootReference(),
               "valid boot reference is present before invalid Fault")) return 1;
    with_boot_reference.reportFault(zero_reason);
    if (!check(with_boot_reference.state() == SafetyState::Disarmed &&
                   with_boot_reference.hasBootReference(),
               "zero Fault reason does not invalidate boot reference")) return 1;

    tilt::safety::SafetyController uncleared;
    uncleared.reportFault(FaultReason::EmergencyStop);
    for (FaultReason invalid : invalid_reasons) {
        uncleared.reportFaultConditionCleared(invalid);
        if (!check(uncleared.requestFaultRecovery() ==
                       tilt::safety::RecoveryResult::NoConditionCleared &&
                       uncleared.state() == SafetyState::Fault &&
                       uncleared.hasFault(FaultReason::EmergencyStop),
                   "invalid clearance cannot recover a valid Fault")) return 1;
    }

    controller.reportFault(FaultReason::EmergencyStop);
    if (!check(controller.state() == SafetyState::Fault &&
                   controller.hasFault(FaultReason::EmergencyStop) &&
                   controller.emergencyStopLatched() &&
                   !controller.hasFault(FaultReason::ServoCommunication),
               "first Fault is latched")) return 1;

    controller.reportFault(FaultReason::EmergencyStop);
    if (!check(controller.state() == SafetyState::Fault &&
                   controller.hasFault(FaultReason::EmergencyStop) &&
                   !controller.hasFault(FaultReason::ServoCommunication),
               "reporting the same Fault is idempotent")) return 1;

    controller.reportFault(FaultReason::ServoCommunication);
    if (!check(controller.state() == SafetyState::Fault &&
                   controller.hasFault(FaultReason::EmergencyStop) &&
                   controller.hasFault(FaultReason::ServoCommunication),
               "different Fault reasons accumulate")) return 1;

    auto batch = makeTestBatch();
    if (!check(rejectedWithoutBatch(controller.evaluate(batch)),
               "Fault blocks the basic evaluate overload")) return 1;

    tilt::safety::SafetyController configured(makeTestPositionLimits(),
                                              makeTestVelocityLimits());
    tilt::core::JointPositionReference reference{};
    configured.reportFault(FaultReason::ServoCommunication);
    if (!check(rejectedWithoutBatch(configured.evaluate(batch, reference)),
               "Fault blocks valid position and velocity command")) return 1;

    for (unsigned int attempt = 0; attempt < 3; ++attempt) {
        batch.seq = attempt;
        if (!check(rejectedWithoutBatch(configured.evaluate(batch)) &&
                       rejectedWithoutBatch(configured.evaluate(batch, reference)) &&
                       configured.state() == SafetyState::Fault &&
                       configured.hasFault(FaultReason::ServoCommunication),
                   "later commands do not clear Fault")) return 1;
    }

    tilt::safety::SafetyController estop_controller;
    estop_controller.requestEmergencyStop();
    if (!check(estop_controller.state() == SafetyState::Fault &&
                   estop_controller.hasFault(FaultReason::EmergencyStop) &&
                   estop_controller.emergencyStopLatched(),
               "E-STOP uses the common Fault path")) return 1;

    tilt::safety::SafetyController invalid_position(makeTestPositionLimits(),
                                                    makeTestVelocityLimits());
    batch = makeTestBatch();
    batch.angle_rad[0] = 2.0f;
    if (!check(rejectedWithoutBatch(invalid_position.evaluate(batch, reference)) &&
                   invalid_position.state() == SafetyState::Disarmed &&
                   !invalid_position.hasFault(FaultReason::EmergencyStop) &&
                   !invalid_position.hasFault(FaultReason::ServoCommunication),
               "position rejection does not latch Fault")) return 1;

    tilt::safety::SafetyController invalid_velocity(makeTestPositionLimits(),
                                                    makeTestVelocityLimits());
    batch = makeTestBatch();
    batch.angle_rad[0] = 0.5f;
    batch.duration_ms = 100;
    if (!check(rejectedWithoutBatch(invalid_velocity.evaluate(batch, reference)) &&
                   invalid_velocity.state() == SafetyState::Disarmed &&
                   !invalid_velocity.hasFault(FaultReason::EmergencyStop) &&
                   !invalid_velocity.hasFault(FaultReason::ServoCommunication),
               "velocity rejection does not latch Fault")) return 1;

    tilt::safety::SafetyController missing_reference(makeTestPositionLimits(),
                                                     makeTestVelocityLimits());
    batch = makeTestBatch();
    if (!check(rejectedWithoutBatch(missing_reference.evaluate(batch)) &&
                   missing_reference.state() == SafetyState::Disarmed &&
                   !missing_reference.hasFault(FaultReason::EmergencyStop) &&
                   !missing_reference.hasFault(FaultReason::ServoCommunication),
               "missing reference does not latch Fault")) return 1;

    tilt::safety::SafetyController missing_velocity_config(makeTestPositionLimits());
    if (!check(rejectedWithoutBatch(missing_velocity_config.evaluate(batch, reference)) &&
                   missing_velocity_config.state() == SafetyState::Disarmed &&
                   !missing_velocity_config.hasFault(FaultReason::EmergencyStop) &&
                   !missing_velocity_config.hasFault(FaultReason::ServoCommunication),
               "missing velocity configuration does not latch Fault")) return 1;

    std::puts("Safety Fault-state host tests passed");
    return 0;
}
