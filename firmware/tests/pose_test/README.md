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
| `rock` | Enter the floor-based left/right weight-shift test after confirmation. |

In IK mode: arrow keys or `W/A/S/D` move the torso, `m` toggles fine/coarse
steps, `0` returns to IK home, `v` compares encoder FK with the requested feet,
`q` exits back to home, and `!` E-STOPS. Unreachable or joint-limit-clamped IK
targets are rejected rather than sent to the servos.

## Rocking test

Unlike the earlier calibration steps, `rock` is meaningful only with both feet
on the floor. Keep both hands ready to catch the robot before confirming the
entry prompt. The test changes the two leg lengths in opposite directions while
keeping `body_x=0`; MPU roll is display/abort information only and is never fed
back into motion control. If the MPU6050 is unavailable, the mode remains usable
and displays `roll=--`.

- Arrow keys or `W/A/S/D`: change differential leg length or base height.
- `m`: toggle 0.5 mm / 2 mm steps; `0`: return differential length to zero.
- `1`: sweep from 0 to 14 mm with a 1.5 s hold at each 1 mm step.
- `2`: alternate the current non-zero magnitude left/right; `[` and `]` adjust
  its 300–2000 ms period in 100 ms steps.
- `k`: record the visually observed left/right foot-lift point.
- `v`: print current delta, leg lengths, and IMU roll; `q`: print the result and
  return home; `!`: immediate E-STOP.

Any key stops an automatic sequence and returns `delta` to zero. A target is
rejected without transmission if IK is unreachable or any logical joint limit
would be clamped. Automatic motion also stops if measured roll exceeds 20°.
