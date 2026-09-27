#include <tilt/safety/SafetyController.h>
#include <tilt/safety/PositionLimitValidator.h>
#include <tilt/safety/VelocityLimitValidator.h>

namespace tilt {
namespace safety {

namespace {

bool isValidFaultReason(core::FaultReason reason) {
    return reason == core::FaultReason::EmergencyStop ||
           reason == core::FaultReason::ServoCommunication;
}

}  // namespace

SafetyController::SafetyController(const core::JointPositionLimits& limits)
    : position_limits_(limits) {}

SafetyController::SafetyController(
    const core::JointPositionLimits& position_limits,
    const core::JointVelocityLimits& velocity_limits)
    : position_limits_(position_limits), velocity_limits_(velocity_limits) {}

SafetyState SafetyController::state() const {
    return state_;
}

void SafetyController::reportFault(core::FaultReason reason) {
    if (!isValidFaultReason(reason)) {
        return;
    }

    const uint32_t reason_bit = static_cast<uint32_t>(reason);
    fault_mask_ |= reason_bit;
    condition_cleared_mask_ &= ~reason_bit;
    state_ = SafetyState::Fault;
    boot_reference_.reset();
}

bool SafetyController::hasFault(core::FaultReason reason) const {
    if (!isValidFaultReason(reason)) {
        return false;
    }

    return (fault_mask_ & static_cast<uint32_t>(reason)) != 0;
}

void SafetyController::reportFaultConditionCleared(core::FaultReason reason) {
    if (!isValidFaultReason(reason)) {
        return;
    }

    const uint32_t reason_bit = static_cast<uint32_t>(reason);
    if ((fault_mask_ & reason_bit) != 0) {
        condition_cleared_mask_ |= reason_bit;
    }
}

RecoveryResult SafetyController::requestFaultRecovery() {
    if (fault_mask_ == 0) {
        return RecoveryResult::NoFault;
    }

    const uint32_t clearable = fault_mask_ & condition_cleared_mask_;
    if (clearable == 0) {
        return RecoveryResult::NoConditionCleared;
    }

    fault_mask_ &= ~clearable;
    condition_cleared_mask_ &= fault_mask_;
    if (fault_mask_ != 0) {
        return RecoveryResult::PartiallyRecovered;
    }

    state_ = SafetyState::Disarmed;
    return RecoveryResult::RecoveredToDisarmed;
}

void SafetyController::requestEmergencyStop() {
    reportFault(core::FaultReason::EmergencyStop);
}

bool SafetyController::emergencyStopLatched() const {
    return hasFault(core::FaultReason::EmergencyStop);
}

bool SafetyController::initializeBootReference(
    const core::JointPositionReference& reference) {
    if (state_ != SafetyState::Disarmed || !position_limits_.has_value() ||
        !validatePositionLimits(reference, *position_limits_)) {
        return false;
    }

    boot_reference_ = reference;
    return true;
}

bool SafetyController::hasBootReference() const {
    return boot_reference_.has_value();
}

bool SafetyController::arm() {
    if (state_ != SafetyState::Disarmed || fault_mask_ != 0 ||
        !position_limits_.has_value() || !velocity_limits_.has_value() ||
        !boot_reference_.has_value()) {
        return false;
    }

    state_ = SafetyState::Armed;
    return true;
}

void SafetyController::disarm() {
    if (state_ != SafetyState::Fault) {
        state_ = SafetyState::Disarmed;
    }
}

SafetyResult SafetyController::evaluate(const core::JointTargetBatch& batch) {
    if (fault_mask_ != 0) {
        return {SafetyResult::Status::Rejected, std::nullopt};
    }
    if (!position_limits_.has_value() ||
        !validatePositionLimits(batch, *position_limits_)) {
        return {SafetyResult::Status::Rejected, std::nullopt};
    }

    // Without a reference, velocity validation cannot run.
    return {SafetyResult::Status::Rejected, std::nullopt};
}

SafetyResult SafetyController::evaluate(
    const core::JointTargetBatch& batch,
    const core::JointPositionReference& runtime_reference) {
    if (state_ != SafetyState::Armed || fault_mask_ != 0) {
        return {SafetyResult::Status::Rejected, std::nullopt};
    }
    if (!position_limits_.has_value() ||
        !validatePositionLimits(batch, *position_limits_) ||
        !validatePositionLimits(runtime_reference, *position_limits_)) {
        return {SafetyResult::Status::Rejected, std::nullopt};
    }
    if (!velocity_limits_.has_value() ||
        !validateVelocityLimits(batch, runtime_reference, *velocity_limits_)) {
        return {SafetyResult::Status::Rejected, std::nullopt};
    }

    return {SafetyResult::Status::Accepted, batch};
}

}  // namespace safety
}  // namespace tilt
