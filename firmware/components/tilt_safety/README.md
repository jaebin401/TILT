# TILT Safety

## Responsibility and coordinates

`tilt_safety` validates six-joint commands before actuator execution and maintains Safety/Fault state. `JointTargetBatch.angle_rad` contains Robot/Kinematics logical joint angles in radians:

| Index | Logical joint |
| --- | --- |
| 0 | LEFT_HIP_YAW |
| 1 | LEFT_HIP_PITCH |
| 2 | LEFT_KNEE_PITCH |
| 3 | RIGHT_HIP_YAW |
| 4 | RIGHT_HIP_PITCH |
| 5 | RIGHT_KNEE_PITCH |

Motor Zero and Robot Zero are different coordinates. STS3215 raw position 2047 represents Motor Zero, not a logical robot target. Servo ID mapping, raw-position conversion, calibration offsets, servo direction/mirroring, UART, and STS3215 protocol belong below Safety. Safety does not generate trajectories or execute actuator commands. The actual TILT joint position and velocity limits have not been measured/configured here; host tests use synthetic values only.

## Implemented behavior

- Position and required-average-velocity validators check all six joints atomically. One invalid joint fails the entire validation. Targets are neither clamped nor automatically modified.
- The velocity validator checks `abs(target - runtime_reference) / (duration_ms / 1000)` in rad/s. This is the required **average** velocity, not measured instantaneous servo velocity. Time-based firmware calculations use seconds; `JointTargetBatch` retains its agreed millisecond fields.
- E-STOP and `FaultReason` (`EmergencyStop`, `ServoCommunication`) use a latched bitmask. Undefined, zero, and combined reason values are ignored by the public Fault APIs.
- A reported Fault enters `SafetyState::Fault`, invalidates the boot reference, and blocks new commands. A condition-cleared report alone does not remove the Fault. Explicit recovery is also required; clearing the final Fault returns to `Disarmed`, never directly to `Armed`.
- `initializeBootReference()` stores a validated logical joint-position snapshot. A boot snapshot indicates initialization readiness; it is not the current-position reference supplied to velocity validation during motion. Fault recovery does not restore it. A fresh physical read and initialization are needed after recovery.

There is no Armed transition yet. **Every `evaluate()` path currently returns `Rejected` with an empty `approved_batch`.** `Accepted` and `Modified` exist in `SafetyResult` for later policy work but are not produced. This component currently authorizes no actuator command.

`SafetyState::Fault` does not mean a servo has physically stopped. Torque-off, position hold, zero-velocity commands, and STS3215 stop packets are not implemented here. The actuator/driver integration must determine and execute a hardware stop strategy.

## Pending work

| Area | Status and missing contract |
| --- | --- |
| Joint acceleration limit | **BLOCKED:** signed current velocity, velocity trajectory, acceleration/deceleration ramp, and execution guarantee |
| Actual boot position read | **BLOCKED:** STS3215 read, actuator state, and raw/motor-to-logical calibration integration |
| Safe initial pose | **BLOCKED:** INIT source value, actual target pose, actuator execution and completion feedback, and acceleration/execution safety; Safety-side INIT policy is designed but does not approve commands |
| Servo communication watchdog | **BLOCKED:** driver health results, six-axis aggregation, thresholds, and hardware validation; existing `ServoCommunication` Fault API is ready for integration |
| Stale command rejection | **CONTRACT PENDING:** `timestamp_ms` clock domain, stamping point, current-time provider, and maximum age |
| Command stream timeout | **BLOCKED:** Armed/active-motion and INIT execution semantics, stream requirement, and threshold |
| Over-temperature and overload | **BLOCKED:** telemetry fields and units, thresholds, sampling, and hardware validation; existing Fault mechanism can be reused once reasons are defined |

## Testing and concurrency

`firmware/tests/host/` tests pure position/velocity validation, E-STOP and Fault state, recovery, boot-reference state, and invalid FaultReason handling without hardware. **Host test success does not demonstrate physical robot safety or actuator stop behavior.** ESP32/STS3215 integration requires separate verification in `firmware/tests/hardware/`.

`SafetyController` is not thread-safe. If command, E-STOP, actuator-health, or watchdog tasks call it concurrently, the application must serialize calls or provide synchronization.
