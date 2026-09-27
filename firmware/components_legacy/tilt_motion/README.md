# TILT Motion

`tilt_motion`은 하드웨어와 독립적으로 여섯 다리 관절의 Pose와 시간 기반 Motion Clip을 계산합니다.

## Responsibility

- 논리 관절각 기반 `JointPose` 정의
- 확정된 기본자세 제공
- 두 Pose 사이의 부드러운 위치·속도·가속도 궤적 계산
- 여러 키프레임으로 구성된 Motion Clip 샘플링
- 임의의 시작자세에서 Home Pose로 복귀하는 표준 Motion 제공
- torso 좌표계에서 발의 위치·속도·가속도 궤적 계산
- Hip부터 Ankle 고정점까지의 FK·도달 가능 검사·closed-form IK
- Double Support와 좌우 Swing의 무상태 보행 phase 계산
- 지지 Ankle을 고정한 단일 Biped Step 궤적 계산

이 컴포넌트는 서보 ID, STS3215 raw position, 캘리브레이션 오프셋, 좌우 모터 방향 반전, UART 또는 실제 명령 실행을 다루지 않습니다. 이러한 변환과 실행은 actuator 및 driver 계층의 책임입니다.

## Joint order

모든 각도는 라디안이며 Robot/Kinematics 논리 좌표계를 사용합니다.

| Index | Joint |
| ---: | --- |
| 0 | LEFT_HIP_YAW |
| 1 | LEFT_HIP_PITCH |
| 2 | LEFT_KNEE_PITCH |
| 3 | RIGHT_HIP_YAW |
| 4 | RIGHT_HIP_PITCH |
| 5 | RIGHT_KNEE_PITCH |

## Home Pose

ADR-008에서 확정한 각 다리의 기본자세 `[0°, -20°, +20°]`를 사용합니다.

```text
[0, -0.349066, +0.349066, 0, -0.349066, +0.349066] rad
```

좌우 다리는 같은 논리 부호를 사용합니다. 실제 모터의 대칭 방향은 driver 계층에서 처리합니다.

## Interpolation

`samplePoseTransition()`은 quintic smootherstep `6u⁵ - 15u⁴ + 10u³`을 사용합니다. 시작과 끝에서 속도와 가속도가 모두 0이고 내부 상태가 없으므로, 호출 순서와 관계없이 임의 시각을 동일하게 샘플링할 수 있습니다.

반환되는 `PoseSample`에는 다음 값이 함께 들어 있습니다.

- 논리 관절 위치 `pose.angle_rad`
- 논리 관절 속도 `velocity_rad_per_sec`
- 논리 관절 가속도 `acceleration_rad_per_sec2`
- 전체 Motion 완료 여부 `finished`

`sampleMotionState()`는 첫 구간을 `initial_pose`에서 첫 키프레임까지 보간하고 이후 키프레임을 순서대로 연결합니다. 마지막 구간 이후에는 마지막 Pose와 0 속도·0 가속도를 반환합니다. 위치만 필요한 호출자는 `interpolatePose()`와 `sampleMotion()` 편의 함수를 사용할 수 있습니다.

`sampleReturnHomeMotion()`은 아직 실제 제한이 검증되지 않은 임의 Pose를 추가하지 않고, 문서로 확정된 Home Pose까지의 궤적만 생성하는 첫 표준 Motion입니다. 이동 시간은 호출자가 지정합니다.

## Foot trajectory

`sampleFootTrajectory()`는 torso 좌표계(`x=전진`, `y=왼쪽`, `z=위`)에서 시작점과 목표점 사이를 이동합니다. 수평·기준 높이 이동은 quintic smootherstep을 사용하고, `clearance_mm`만큼의 상승과 하강도 각각 quintic으로 계산해 시작·정점·착지에서 속도와 가속도가 0이 되도록 합니다.

보폭, 발을 드는 높이와 이동 시간은 하드웨어 검증 전이므로 라이브러리에 임의 상수로 고정하지 않습니다. 호출자가 게이트 파라미터로 전달해야 하며, `clearance_mm = 0`이면 지지발의 torso 기준 이동에도 사용할 수 있습니다. 이 출력은 다음 단계의 IK 입력이며 그 자체로 관절 명령이 아닙니다.

## Leg kinematics

`forwardAnkleKinematics()`와 `inverseAnkleKinematics()`는 ADR-008의 `Hip Yaw → Hip Pitch → Knee Pitch` 체인에서 Ankle 고정점 O4까지 계산합니다. `tiltLegGeometry()`는 확정된 Hip y 오프셋 `±36.2 mm`와 허벅지·정강이 길이 `50 mm`를 제공합니다.

IK는 호출자가 지정한 Hip Yaw 평면 안에서 양(+)의 Knee 굽힘 해를 계산합니다. 목표점이 해당 평면 밖에 있거나 두 링크의 작업공간 밖이면 `std::nullopt`를 반환합니다. 따라서 뒤쪽 목표도 Hip Yaw를 180°로 뒤집지 않고 지정된 다리 평면 안에서 처리할 수 있습니다.

Ankle FK/IK API는 O4까지만 계산해 확정된 링크 모델과 접지점 가정을 분리합니다. 접지점 E가 필요한 호출자는 아래의 별도 `ContactKinematics`를 사용합니다.

`forwardRobotAnkleKinematics()`와 `inverseRobotAnkleKinematics()`는 이 계산을 좌우 여섯 관절의 `JointPose` 단위로 묶습니다. 이를 통해 하드웨어 없이 `JointPose → 양쪽 O4 위치 → JointPose` 왕복 오차를 검증할 수 있습니다.

`ContactKinematics`는 다음 작업을 진행하기 위해 ADR-008의 Home Pose O4 위치와 문서상 접지점 E를 이용해 `O4→E=(-17.101007, 0, -7.015366) mm`를 임시로 역산합니다. 이 값은 CAD 실측값이 아니며 `provisionalFootContactGeometry()` 한 곳에 격리되어 있습니다. 현재 값으로 Home Pose 접지점과 `JointPose ↔ 양쪽 접지점` 왕복 계산을 재현하며, 실측 후 해당 상수만 교체합니다.

## Gait phase

`sampleGaitPhase()`는 `Double Support → 첫 번째 Swing → Double Support → 반대쪽 Swing`을 한 cycle로 반복합니다. 현재 지지 상태, 스윙 다리, 다음 스윙 다리와 phase 내부 진행률을 반환하며 내부 상태를 갖지 않습니다. Double Support와 Swing 지속 시간 및 첫 스윙 다리는 호출자가 전달하므로, 아직 검증되지 않은 주기나 좌우 시작 순서를 라이브러리에 고정하지 않습니다.

`sampleBipedStep()`은 전달받은 양쪽 O4 위치 중 지지 Ankle을 고정하고 스윙 Ankle에 `sampleFootTrajectory()`를 적용합니다. 위치·속도·가속도를 모두 반환하며, 결과 위치는 `inverseRobotAnkleKinematics()`로 관절 Pose 변환과 도달 가능 여부를 확인할 수 있습니다.

`sampleBipedStepPose()`는 이 경로를 한 번에 실행해 `발 궤적 → IK → JointPose`를 반환합니다. Cartesian 목표가 해당 Hip Yaw 평면이나 링크 작업공간을 벗어나면 관절각을 임의로 clamp하지 않고 실패합니다.

실제 보행 계획용 `sampleContactStep()`과 `sampleContactStepPose()`는 같은 동작을 접지점 E 기준으로 수행합니다. 지지 접지점을 고정하고 스윙 접지점만 이동한 뒤, 임시 접지점 기하를 포함한 IK로 `JointPose`를 계산합니다.

유효하지 않은 duration, NaN, 무한대 또는 잘못된 Clip은 `std::nullopt`로 거부합니다. 이 계층의 출력은 이후 Adapter가 `JointTargetBatch`로 변환하고 Safety 검증을 통과해야 실제 액추에이터로 전달될 수 있습니다.
