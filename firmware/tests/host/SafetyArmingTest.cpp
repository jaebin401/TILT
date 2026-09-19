#include <tilt/safety/SafetyController.h>

#include <cstdio>

namespace {

tilt::core::JointPositionLimits positionLimits() {
    tilt::core::JointPositionLimits limits{};
    for (unsigned int joint = 0; joint < 6; ++joint) {
        limits.min_rad[joint] = -1.0f;
        limits.max_rad[joint] = +1.0f;
    }
    return limits;
}

tilt::core::JointVelocityLimits velocityLimits() {
    tilt::core::JointVelocityLimits limits{};
    for (float& maximum : limits.max_rad_per_sec) {
        maximum = 1.0f;
    }
    return limits;
}

bool check(bool condition, const char* name) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", name);
    }
    return condition;
}

}  // namespace

int main() {
    using tilt::safety::SafetyResult;
    using tilt::safety::SafetyState;

    tilt::core::JointPositionReference reference{};
    tilt::core::JointTargetBatch batch{};
    batch.duration_ms = 1000;
    batch.angle_rad[0] = 0.5f;

    tilt::safety::SafetyController controller(positionLimits(), velocityLimits());
    if (!check(!controller.arm(), "cannot arm before boot reference")) return 1;
    if (!check(controller.initializeBootReference(reference) && controller.arm() &&
                   controller.state() == SafetyState::Armed,
               "validated boot reference enables explicit arm")) return 1;

    const auto accepted = controller.evaluate(batch, reference);
    if (!check(accepted.status == SafetyResult::Status::Accepted &&
                   accepted.approved_batch.has_value() &&
                   accepted.approved_batch->seq == batch.seq,
               "armed valid command is accepted atomically")) return 1;

    if (!check(controller.evaluate(batch).status == SafetyResult::Status::Rejected,
               "missing runtime reference still rejects while armed")) return 1;

    batch.angle_rad[0] = 2.0f;
    if (!check(controller.evaluate(batch, reference).status ==
                   SafetyResult::Status::Rejected &&
                   controller.state() == SafetyState::Armed,
               "invalid command rejects without implicit state transition")) return 1;

    batch.angle_rad[0] = 0.0f;
    reference.angle_rad[0] = 1.1f;
    if (!check(controller.evaluate(batch, reference).status ==
                   SafetyResult::Status::Rejected,
               "out-of-range runtime feedback is rejected")) return 1;
    reference.angle_rad[0] = 0.0f;

    controller.disarm();
    if (!check(controller.state() == SafetyState::Disarmed &&
                   controller.evaluate(batch, reference).status ==
                       SafetyResult::Status::Rejected,
               "disarm revokes execution permission")) return 1;

    if (!check(controller.arm(), "existing valid boot reference can re-arm")) return 1;
    controller.requestEmergencyStop();
    controller.disarm();
    if (!check(controller.state() == SafetyState::Fault && !controller.arm(),
               "disarm and arm cannot bypass Fault")) return 1;

    std::puts("Safety arming host tests passed");
    return 0;
}
