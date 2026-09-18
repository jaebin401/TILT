#include <tilt/safety/SafetyController.h>

#include <cstdio>

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

    tilt::safety::SafetyController controller(makeTestPositionLimits(),
                                             makeTestVelocityLimits());
    tilt::core::JointTargetBatch batch{};
    batch.duration_ms = 1000;
    tilt::core::JointPositionReference reference{};

    if (!check(controller.state() == SafetyState::Disarmed &&
                   controller.requestFaultRecovery() == RecoveryResult::NoFault &&
                   !controller.emergencyStopLatched(),
               "recovery with no Fault is a no-op")) return 1;

    controller.requestEmergencyStop();
    if (!check(controller.state() == SafetyState::Fault &&
                   controller.emergencyStopLatched() &&
                   controller.requestFaultRecovery() == RecoveryResult::NoConditionCleared &&
                   controller.emergencyStopLatched(),
               "recovery request alone cannot clear E-STOP")) return 1;

    // In this host test, the caller claims the external E-STOP source is inactive.
    // This does not verify a physical switch or actuator condition.
    controller.reportFaultConditionCleared(FaultReason::EmergencyStop);
    if (!check(controller.state() == SafetyState::Fault &&
                   controller.emergencyStopLatched() &&
                   rejectedWithoutBatch(controller.evaluate(batch, reference)),
               "condition-cleared report alone does not recover")) return 1;

    if (!check(controller.requestFaultRecovery() == RecoveryResult::RecoveredToDisarmed &&
                   controller.state() == SafetyState::Disarmed &&
                   !controller.emergencyStopLatched(),
               "explicit recovery clears the final Fault to Disarmed")) return 1;
    if (!check(rejectedWithoutBatch(controller.evaluate(batch)) &&
                   rejectedWithoutBatch(controller.evaluate(batch, reference)),
               "recovery does not approve motion")) return 1;
    if (!check(controller.requestFaultRecovery() == RecoveryResult::NoFault,
               "repeating recovery after success is a no-op")) return 1;

    controller.requestEmergencyStop();
    if (!check(controller.requestFaultRecovery() == RecoveryResult::NoConditionCleared &&
                   controller.emergencyStopLatched(),
               "consumed clearance cannot recover a future E-STOP")) return 1;

    tilt::safety::SafetyController multiple;
    multiple.requestEmergencyStop();
    multiple.reportFault(FaultReason::ServoCommunication);
    multiple.reportFaultConditionCleared(FaultReason::EmergencyStop);
    if (!check(multiple.requestFaultRecovery() == RecoveryResult::PartiallyRecovered &&
                   !multiple.hasFault(FaultReason::EmergencyStop) &&
                   multiple.hasFault(FaultReason::ServoCommunication) &&
                   multiple.state() == SafetyState::Fault,
               "partial recovery keeps unresolved Fault and Fault state")) return 1;
    if (!check(multiple.requestFaultRecovery() == RecoveryResult::NoConditionCleared &&
                   multiple.hasFault(FaultReason::ServoCommunication),
               "repeated request cannot clear an unresolved Fault")) return 1;
    multiple.reportFaultConditionCleared(FaultReason::ServoCommunication);
    if (!check(multiple.requestFaultRecovery() == RecoveryResult::RecoveredToDisarmed &&
                   !multiple.hasFault(FaultReason::ServoCommunication) &&
                   multiple.state() == SafetyState::Disarmed,
               "last cleared Fault returns to Disarmed")) return 1;

    tilt::safety::SafetyController recurrence;
    recurrence.reportFault(FaultReason::ServoCommunication);
    recurrence.reportFaultConditionCleared(FaultReason::ServoCommunication);
    recurrence.reportFault(FaultReason::ServoCommunication);
    if (!check(recurrence.requestFaultRecovery() == RecoveryResult::NoConditionCleared &&
                   recurrence.hasFault(FaultReason::ServoCommunication) &&
                   recurrence.state() == SafetyState::Fault,
               "new Fault report invalidates earlier clearance")) return 1;

    tilt::safety::SafetyController stale_clearance;
    stale_clearance.reportFaultConditionCleared(FaultReason::ServoCommunication);
    stale_clearance.reportFault(FaultReason::ServoCommunication);
    if (!check(stale_clearance.requestFaultRecovery() == RecoveryResult::NoConditionCleared &&
                   stale_clearance.hasFault(FaultReason::ServoCommunication),
               "clearance for an unlatched reason is not saved")) return 1;

    // No FAULT_RECOVERY source value is defined yet; evaluate never requests recovery.
    if (!check(rejectedWithoutBatch(stale_clearance.evaluate(batch)) &&
                   stale_clearance.hasFault(FaultReason::ServoCommunication),
               "ordinary batch cannot clear Fault")) return 1;

    std::puts("Safety Fault-recovery host tests passed");
    return 0;
}
