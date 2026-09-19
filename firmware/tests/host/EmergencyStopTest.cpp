#include <tilt/safety/SafetyController.h>

#include <cstdio>

namespace {

// Synthetic values for host tests only; these are not TILT hardware limits.
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
    tilt::safety::SafetyController controller;
    if (!check(controller.state() == tilt::safety::SafetyState::Disarmed &&
                   !controller.emergencyStopLatched(),
               "new controller is Disarmed with no E-STOP latch")) return 1;

    controller.requestEmergencyStop();
    if (!check(controller.state() == tilt::safety::SafetyState::Fault &&
                   controller.emergencyStopLatched(),
               "E-STOP latches and enters Fault")) return 1;

    tilt::core::JointTargetBatch batch{};
    batch.duration_ms = 1000;
    if (!check(rejectedWithoutBatch(controller.evaluate(batch)),
               "ordinary command is rejected with no approved batch")) return 1;

    tilt::safety::SafetyController configured(makeTestPositionLimits(),
                                              makeTestVelocityLimits());
    tilt::core::JointPositionReference reference{};
    configured.requestEmergencyStop();
    if (!check(rejectedWithoutBatch(configured.evaluate(batch, reference)),
               "reference overload rejects after E-STOP")) return 1;

    for (unsigned int attempt = 0; attempt < 3; ++attempt) {
        batch.seq = attempt;
        if (!check(rejectedWithoutBatch(configured.evaluate(batch)) &&
                       rejectedWithoutBatch(configured.evaluate(batch, reference)) &&
                       configured.state() == tilt::safety::SafetyState::Fault &&
                       configured.emergencyStopLatched(),
                   "later commands cannot clear E-STOP")) return 1;
    }

    configured.requestEmergencyStop();
    configured.requestEmergencyStop();
    if (!check(configured.state() == tilt::safety::SafetyState::Fault &&
                   configured.emergencyStopLatched() &&
                   rejectedWithoutBatch(configured.evaluate(batch, reference)),
               "repeated E-STOP requests are idempotent")) return 1;

    std::puts("Safety E-STOP host tests passed");
    return 0;
}
