# TILT ESP32-S3 pose test

`pose_test` is a new, standalone hardware test for the current components:

`tilt_kinematics → JointMapper → Interpolator → tilt_sts3215 UART`

It deliberately does **not** depend on the old `tilt::core` or
`tilt::actuators` APIs. `../esp32_pose_test` is historical reference only and
is not built or modified by this project.

## Wiring and safety

- This adapter connection is **straight**, not crossed: `GPIO17 TX → Adapter
  TXD`, `GPIO18 RX → Adapter RXD`.
- Set the Waveshare Bus Servo Adapter (A) UART jumper to **A**.
- Use servo IDs `11, 12, 13, 21, 22, 23` at **1 Mbps**.
- Use a separate servo supply, share GND with the ESP32, and keep a physical
  servo-power disconnect within reach.
- Start with servo power disconnected. The `!` command requests torque OFF,
  but it cannot protect against a lost UART/power connection.

## Build and flash

```bash
source /Users/jae/esp/esp-idf/export.sh
cd firmware/tests/pose_test
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

## Required first-run sequence

`ZERO_TICK` and `JOINT_SIGN` in `components/tilt_config/include/tilt_config.h`
are currently placeholders. Measure each joint with `tools/servo_tool` and
update those shared values before treating any IK result as physically valid.

1. Keep the robot suspended in a stand; do not begin on the floor.
2. Run `status` and confirm all six IDs respond.
3. Run `check` and confirm the measured zero pose.
4. Run `arm`; it reads all present positions and writes them as the hold goal
   before enabling torque.
5. Use `joint <name> <deg>` in increments no larger than 5° to verify each
   direction.
6. Try `home`, then `pose <name>`, then `ik`.

`body_x` causes a real forward/back shift. For floor testing, keep it at zero
and verify height changes first.

## Commands

| Command | Purpose |
| --- | --- |
| `help` | Show the command list. |
| `status` | Ping each servo and print raw/logical positions. |
| `check` | Compare encoder angles with ADR-008 zero pose. |
| `arm` / `disarm` | Enable torque after hold-goal capture / request torque OFF. |
| `!` | Immediate E-STOP request in every mode. |
| `recover` | ESTOP → DISARMED after all six servos respond; run `arm` again. |
| `home`, `pose <name>` | Interpolated move to a preset. |
| `joint <name> <deg>` | Relative single-joint jog (maximum ±5°). |
| `fk` | Calculate both feet from current encoder readings. |
| `ik` | Enter the torso-motion IK demonstration after interpolating home. |
| `stand` | Enter the floor-based stationary pitch-compensation test. |
| `rock` | Enter the floor-based left/right weight-shift test after confirmation. |
| `walk` | Enter the confirmed, IMU roll-triggered lift-and-step walk test. |

In IK mode: arrow keys or `W/A/S/D` move the torso, `m` toggles fine/coarse
steps, `0` returns to IK home, `v` compares encoder FK with the requested feet,
`q` exits back to home, and `!` E-STOPS. Unreachable or joint-limit-clamped IK
targets are rejected rather than sent to the servos.

## Stand compensation test

`stand` returns to home and starts with compensation **OFF**. Use it only on
the floor with both hands ready to catch the robot. The control loop calculates
pitch compensation every 30 ms while console status is limited to one line per
200 ms.

- `c`: toggle pitch compensation; `+` / `-`: adjust Kp by 0.5 mm/degree.
- `[` / `]`: adjust the low-pass coefficient by 0.05.
- Up/down arrows: change base height; `r`: reset roll and pitch zero.
- `0`: disable compensation and return `body_x` to zero.
- `t`: apply a +8 mm, 500 ms disturbance and record the following 2-second
  response, including peak pitch, settling time, residual, and sign reversals.
- `v`: print the current state; `q`: return to home; `!`: E-STOP.

## Rocking test

Unlike the earlier calibration steps, `rock` is meaningful only with both feet
on the floor. Keep both hands ready to catch the robot before confirming the
entry prompt. The test changes the two leg lengths in opposite directions while
keeping `body_x=0`; MPU roll is display/abort information only and is never fed
back into motion control. If the MPU6050 is unavailable, the mode remains usable
and displays `roll=--`, `pitch=--`.

- Arrow keys or `W/A/S/D`: change differential leg length or base height.
- `m`: toggle 0.5 mm / 2 mm steps; `0`: return differential length to zero.
- `f`: toggle the leg target generator. `FLAT` (the default) directly uses
  `[0, -a, +a]` to keep each foot horizontal; `IK` preserves the original
  free-foot-angle behavior and displays each target foot tilt.
- `1`: sweep from 0 to 14 mm with a 1.5 s hold at each 1 mm step.
- `2`: alternate the current non-zero magnitude left/right; `[` and `]` adjust
  its 150–2000 ms period in 50 ms steps. If the requested period is shorter
  than the velocity-limited move time, the period is expanded automatically.
- `k`: record the visually observed left/right foot-lift point.
- `r`: use the current IMU roll and pitch as the new zero; `v`: print current
  delta, leg lengths, target foot geometry, and absolute/relative attitude.
- `c`: toggle pitch/body-x compensation; `+` / `-`: adjust its Kp.
- `x`: toggle cycle-boundary roll recentering; `z`: toggle cycle-boundary roll
  amplitude tracking. Both roll compensators default to OFF.
- `q`: print the result and return home; `!`: immediate E-STOP.

Any key stops an automatic sequence and returns `delta` to zero. A target is
rejected without transmission if IK is unreachable or any logical joint limit
would be clamped. Automatic motion also stops if measured roll exceeds 20°.
Move duration is automatically extended from the largest joint-angle change so
the 180°/s speed guard remains a final fault barrier rather than a normal motion
limiter. Stopping alternating rocking prints roll/pitch peak-to-peak motion and
the final pitch residual, compensation state, roll-center movement, final
delta bias/scale, and body-x usage. It warns for dominant pitch oscillation,
possible backward drift, or saturated body-x authority. Roll bias and amplitude
scale update only at commanded-delta sign changes, never every control tick.
After returning to `delta=0`, a relative-roll residual over 1° warns that the
curved foot may not have returned to its original contact point.

## Walk test

`walk` returns home after a safety confirmation, then waits stopped with
`delta=14 mm`, `lift=0`, and `legx=0`. Entry is rejected unless valid MPU6050
roll/pitch data is available. Press space to start only after confirming that
stationary rocking is safe. The delta sign flips on a fixed open-loop period,
while relative roll selects the unloaded swing leg using separate entry and
release thresholds. The swing foot receives the additional lift and forward x;
the support foot moves backward. Every target uses IK only.

- Space starts/stops; stopping returns `delta`, `lift`, and `legx` to zero.
- Left/right arrows or `A/D` change `legx` by 1 mm within ±20 mm.
- Up/down arrows or `W/S` change base height by 0.5 mm.
- `+` / `-` change swing-foot `lift` by 1 mm within 0–14 mm.
- `,` / `.` change rocking `delta` by 1 mm within 0–24 mm.
- `[` / `]` change the rocking period by 25 ms within 150–800 ms.
- `;` / `'` change the roll entry threshold by 0.25°; release remains 1.2°.
- `c` toggles common pitch/body-x compensation; `r` resets roll/pitch zero.
- `0` returns both `lift` and `legx` to zero; `v` prints all parameters;
  `q` prints the result and returns home; `!` E-STOPS.

Swing entry occurs at ±2.5° and is held until roll returns inside ±1.2°. The
80 ms minimum prevents noise from cutting a swing short, while the 400 ms
maximum forces release if roll does not return. Output is limited to swing-leg
changes, including return to double support. The exit summary reports per-leg
swing counts and average duration, forced releases, suppressed early releases,
and rejected targets. It intentionally does not estimate forward distance from
the 6-axis IMU.

All compensation is disabled if relative roll or pitch exceeds 20°, or after
five consecutive IMU read failures. Walk stops after five consecutive IMU
failures because swing timing is no longer available. Targets that exceed the
workspace or logical joint limits are rejected instead of clamped and sent;
three consecutive rejections stop the walk.
