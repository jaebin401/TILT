#include <tilt/safety/SafetyController.h>
#include <tilt/safety/PositionLimitValidator.h>

#include <cstdio>
#include <limits>

namespace {

// Synthetic radians used only by this host test, not TILT hardware limits.
tilt::core::JointPositionLimits makeTestLimits() {
    tilt::core::JointPositionLimits limits{};
    for (unsigned int joint = 0; joint < 6; ++joint) {
        limits.min_rad[joint] = -1.0f;
        limits.max_rad[joint] = 1.0f;
    }
    return limits;
}

tilt::core::JointTargetBatch makeTestBatch() {
    tilt::core::JointTargetBatch batch{};
    batch.seq = 42;
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
    auto limits = makeTestLimits();
    auto batch = makeTestBatch();

    if (!check(tilt::safety::validatePositionLimits(batch, limits),
               "all six targets inside limits")) return 1;

    for (unsigned int joint = 0; joint < 6; ++joint) {
        batch.angle_rad[joint] = limits.min_rad[joint];
    }
    if (!check(tilt::safety::validatePositionLimits(batch, limits),
               "exact minimum boundaries")) return 1;

    for (unsigned int joint = 0; joint < 6; ++joint) {
        batch.angle_rad[joint] = limits.max_rad[joint];
    }
    if (!check(tilt::safety::validatePositionLimits(batch, limits),
               "exact maximum boundaries")) return 1;

    batch = makeTestBatch();
    batch.angle_rad[0] = -1.0001f;
    if (!check(!tilt::safety::validatePositionLimits(batch, limits),
               "slightly below minimum")) return 1;

    batch = makeTestBatch();
    batch.angle_rad[0] = 1.0001f;
    if (!check(!tilt::safety::validatePositionLimits(batch, limits),
               "slightly above maximum")) return 1;

    batch = makeTestBatch();
    batch.angle_rad[0] = 100.0f;
    if (!check(!tilt::safety::validatePositionLimits(batch, limits),
               "far outside limits")) return 1;

    batch = makeTestBatch();
    batch.angle_rad[0] = std::numeric_limits<float>::quiet_NaN();
    if (!check(!tilt::safety::validatePositionLimits(batch, limits),
               "NaN target")) return 1;

    batch = makeTestBatch();
    batch.angle_rad[0] = std::numeric_limits<float>::infinity();
    if (!check(!tilt::safety::validatePositionLimits(batch, limits),
               "positive infinity target")) return 1;

    batch = makeTestBatch();
    batch.angle_rad[0] = -std::numeric_limits<float>::infinity();
    if (!check(!tilt::safety::validatePositionLimits(batch, limits),
               "negative infinity target")) return 1;

    limits = makeTestLimits();
    limits.min_rad[0] = std::numeric_limits<float>::quiet_NaN();
    if (!check(!tilt::safety::validatePositionLimits(makeTestBatch(), limits),
               "NaN limit")) return 1;

    limits = makeTestLimits();
    limits.max_rad[0] = std::numeric_limits<float>::infinity();
    if (!check(!tilt::safety::validatePositionLimits(makeTestBatch(), limits),
               "infinite limit")) return 1;

    limits = makeTestLimits();
    limits.min_rad[0] = 2.0f;
    if (!check(!tilt::safety::validatePositionLimits(makeTestBatch(), limits),
               "minimum greater than maximum")) return 1;

    batch = makeTestBatch();
    limits = makeTestLimits();
    batch.angle_rad[5] = 2.0f;
    if (!check(!tilt::safety::validatePositionLimits(batch, limits),
               "one invalid joint rejects entire batch")) return 1;

    tilt::safety::SafetyController controller;
    if (!check(controller.state() == tilt::safety::SafetyState::Disarmed,
               "default controller is Disarmed")) return 1;
    const auto unconfigured = controller.evaluate(makeTestBatch());
    if (!check(unconfigured.status == tilt::safety::SafetyResult::Status::Rejected &&
                   !unconfigured.approved_batch.has_value(),
               "unconfigured controller rejects with no batch")) return 1;

    tilt::safety::SafetyController configured(limits);
    if (!check(configured.state() == tilt::safety::SafetyState::Disarmed,
               "configured controller is Disarmed")) return 1;
    const auto result = configured.evaluate(makeTestBatch());
    if (!check(result.status == tilt::safety::SafetyResult::Status::Rejected &&
                   !result.approved_batch.has_value(),
               "valid position still has no execution permission")) return 1;
    if (!check(configured.state() == tilt::safety::SafetyState::Disarmed,
               "evaluation does not change state")) return 1;

    std::puts("Safety position-limit host tests passed");
    return 0;
}
