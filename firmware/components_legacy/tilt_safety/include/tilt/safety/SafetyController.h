#pragma once

#include <optional>
#include <stdint.h>

#include <tilt/core/FaultReason.h>
#include <tilt/core/JointPositionLimits.h>
#include <tilt/core/JointPositionReference.h>
#include <tilt/core/JointTargetBatch.h>
#include <tilt/core/JointVelocityLimits.h>

namespace tilt {
namespace safety {

enum class SafetyState {
    Disarmed,
    Armed,
    Fault,
};

struct SafetyResult {
    enum class Status {
        Accepted,
        Modified,
        Rejected,
    };

    Status status;
    // Empty for Rejected; only an approved batch may be sent onward.
    std::optional<core::JointTargetBatch> approved_batch;
};

enum class RecoveryResult {
    NoFault,
    NoConditionCleared,
    PartiallyRecovered,
    RecoveredToDisarmed,
};

class SafetyController {
public:
    SafetyController() = default;
    explicit SafetyController(const core::JointPositionLimits& limits);
    SafetyController(const core::JointPositionLimits& position_limits,
                     const core::JointVelocityLimits& velocity_limits);

    // Fault means the application must arrange a safe stop through the actuator layer.
    SafetyState state() const;
    void reportFault(core::FaultReason reason);
    bool hasFault(core::FaultReason reason) const;
    void reportFaultConditionCleared(core::FaultReason reason);
    RecoveryResult requestFaultRecovery();
    // Latches E-STOP and enters Fault; does not issue a hardware command.
    void requestEmergencyStop();
    bool emergencyStopLatched() const;
    // Stores a validated logical boot-time snapshot, not a runtime reference.
    bool initializeBootReference(const core::JointPositionReference& reference);
    bool hasBootReference() const;
    // 부팅 기준값과 모든 제한값이 준비된 Disarmed 상태에서만 Armed로 전환한다.
    bool arm();
    // Fault는 해제하지 않고, 정상 상태의 출력 권한만 회수한다.
    void disarm();
    SafetyResult evaluate(const core::JointTargetBatch& batch);
    // runtime_reference is the current logical joint position, not the boot snapshot.
    SafetyResult evaluate(const core::JointTargetBatch& batch,
                          const core::JointPositionReference& runtime_reference);

private:
    SafetyState state_ = SafetyState::Disarmed;
    uint32_t fault_mask_ = 0;
    uint32_t condition_cleared_mask_ = 0;
    std::optional<core::JointPositionLimits> position_limits_;
    std::optional<core::JointVelocityLimits> velocity_limits_;
    std::optional<core::JointPositionReference> boot_reference_;
};

}  // namespace safety
}  // namespace tilt
