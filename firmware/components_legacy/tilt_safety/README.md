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
- `arm()` succeeds only from `Disarmed` when position limits, velocity limits, and a validated boot reference are all present. `disarm()` revokes execution permission but never clears a Fault.
- While `Armed`, the runtime-reference overload returns `Accepted` only after the target and current runtime reference are both inside all six position limits and the required-average-velocity check passes. The overload without a runtime reference always rejects.

`SafetyState::Fault` does not itself mean a servo has physically stopped. The integrated pose test performs best-effort torque-off through the actuator driver; a physical power cutoff remains necessary when communication has failed.

## Pending work

| Area | Status and missing contract |
| --- | --- |
| Joint acceleration limit | **BLOCKED:** signed current velocity, velocity trajectory, acceleration/deceleration ramp, and execution guarantee |
| Actual boot position read | **INTEGRATED FOR TEST:** `esp32_pose_test` reads all six STS3215 positions and maps raw feedback to logical radians before arming; physical calibration remains pending |
| Safe initial pose | **INTEGRATED FOR TEST:** current-position hold precedes torque-on, then Home is reached through the normal Motion/Safety/Actuator path; completion tolerance and physical validation remain pending |
| Servo communication watchdog | **PARTIAL:** pose execution faults and requests torque-off on a failed read/write; a periodic background watchdog and measured thresholds remain pending |
| Stale command rejection | **CONTRACT PENDING:** `timestamp_ms` clock domain, stamping point, current-time provider, and maximum age |
| Command stream timeout | **BLOCKED:** Armed/active-motion and INIT execution semantics, stream requirement, and threshold |
| Over-temperature and overload | **BLOCKED:** telemetry fields and units, thresholds, sampling, and hardware validation; existing Fault mechanism can be reused once reasons are defined |

## Testing and concurrency

`firmware/tests/host/` tests pure position/velocity validation, arming, E-STOP and Fault state, recovery, boot-reference state, mapping, and packet encoding without hardware. `firmware/tests/esp32_pose_test/` exercises the integrated hardware path. **Host test success does not demonstrate physical robot safety or actuator stop behavior.**

`SafetyController` is not thread-safe. If command, E-STOP, actuator-health, or watchdog tasks call it concurrently, the application must serialize calls or provide synchronization.
