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

In IK mode: arrow keys or `W/A/S/D` move the torso, `m` toggles fine/coarse
steps, `0` returns to IK home, `v` compares encoder FK with the requested feet,
`q` exits back to home, and `!` E-STOPS. Unreachable or joint-limit-clamped IK
targets are rejected rather than sent to the servos.
