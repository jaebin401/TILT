# TILT rock test

`rock_test`는 좌우 무게 이동과 한 발이 지면에서 온전히 떨어지는지를
분리해서 확인하는 독립 ESP-IDF 실험이다. 전진 보행은 하지 않는다. 기존
`pose_test`·`walk_test`나 공용 컴포넌트를 수정하지 않는다.

다리를 평행하게 접기 위해 hip pitch와 knee pitch를 같은 크기, 반대 방향으로
움직인다. `tilt_kinematics`의 IK는 사용하지 않는다. 발 목표 높이(mm)를
10ms마다 선형 보간하고, [ParallelLeg.h](main/ParallelLeg.h)의 수식으로
6축 관절각을 만든 뒤 한 개의 `syncWritePositions` 패킷으로 전송한다.

## 배선과 물리적 안전

- ESP32-S3 `GPIO17 TX → Adapter TXD`, `GPIO18 RX → Adapter RXD`로 직결한다.
- Waveshare Bus Servo Adapter (A)의 UART 점퍼를 A에 둔다.
- 서보 ID는 `11, 12, 13, 21, 22, 23`, 통신 속도는 1 Mbps다.
- 서보는 별도 전원을 사용하고 ESP32와 GND를 공유한다.
- 로봇을 바닥에 세우고 두 손으로 즉시 받칠 준비를 한다.
- 서보 전원을 즉시 끊을 수단을 손 닿는 곳에 둔다. `!`는 UART 통신이나
  전원 자체가 끊어진 경우를 대신할 수 없다.
- 부팅 시 `DISARMED`, 토크 OFF, 자동 이동 없음이다. `arm`은 현재 여섯
  위치를 읽어 hold goal로 먼저 쓴 뒤 토크를 켠다.
- 이 테스트에는 **IMU 각도에 의한 자동 정지가 없다.** 넘어지려 하면
  사람이 space 또는 물리적 전원 차단으로 멈춰야 한다.

높이 공간 보간만 사용하므로 `arm` 당시 자세가 평행 다리 자세에서 약 2°
이상 벗어나면 `stand`/`rock`을 거부한다. 이 경우 `disarm`하고 로봇을
ADR-008 zero 평행 자세 근처에 배치한 뒤 다시 `arm`한다. 임의의 비평행
자세에서 관절을 갑자기 평행 목표로 당기지 않기 위한 제한이다.

## 빌드

```bash
source /Users/jae/esp/esp-idf/export.sh
cd firmware/tests/rock_test
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

기존 `firmware/tests/host_test`를 그대로 두기 위해 PC 검산은 이 프로젝트
안의 `host_test`에 독립 실행파일로 둔다.

```bash
cmake -S firmware/tests/rock_test/host_test -B /tmp/tilt-rock-host-build
cmake --build /tmp/tilt-rock-host-build
ctest --test-dir /tmp/tilt-rock-host-build --output-on-failure
```

PC 검산은 90–103.97mm 높이에서의 허벅지각·발 x 표, ADR-008 zero pose,
lean을 반영한 발판의 수평 각도를 확인한다.

## 명령

줄 단위 명령:

| 명령 | 동작 |
| --- | --- |
| `help` | 도움말 |
| `status` | 6축 ping, raw tick, 논리 각도 |
| `arm` / `disarm` | 현재 자세 hold 후 토크 ON / 토크 OFF |
| `recover` | E-STOP에서 DISARMED로 복구; 다시 `arm` 필요 |
| `!` | 모든 모드에서 E-STOP |
| `stand [h] [lean_deg]` | 양발을 같은 높이의 평행 자세로 2초 이동 |
| `rock` | 기준 stand 자세로 이동한 뒤 정지 상태의 rock 모드 진입 |

rock 모드에서는 Enter 없이 한 키씩 입력한다. 기본 패턴은 `PHASE6`이고,
정지 상태에서 `m`으로 `BRUTON`과 전환한다. 전환 시 해당 키 안내가 출력된다.

| 키 | 동작 |
| --- | --- |
| space | 연속 모드 시작/정지 |
| `n` | 다음 phase를 한 번 실행하고 정지 |
| ↑ / `W`, ↓ / `S` | base ±0.5mm |
| → / `D`, ← / `A` | shift ±0.5mm |
| `+` / `-` | lift ±0.5mm |
| `1` / `2` | shift duration ∓/±10ms |
| `3` / `4` | lift duration ∓/±10ms |
| `5` / `6` | plant duration ∓/±10ms |
| `[` / `]` | lift 정점 hold ∓/±10ms (0–1000ms) |
| `;` / `'` | plant hold ∓/±10ms (0–1000ms) |
| `m` | 정지 상태에서 PHASE6 ↔ BRUTON 전환 |
| `,` / `.` | lean ∓/±0.5° |
| `k` | 육안으로 확인한 발-뜸을 현재 phase·높이·roll과 함께 기록 |
| `r` | 표시용 IMU roll/pitch 기준 재설정 |
| `v` | 파라미터, phase 목표, 도달 가능 여부, x 결합, 기록 출력 |
| `0` | 정지 후 base 평행 stand로 복귀 |
| `q` | 종료 요약 후 base 평행 stand로 복귀, 줄 단위 콘솔 복귀 |
| `!` | 즉시 E-STOP |

BRUTON 패턴에서는 `space`가 별도의 양다리 dual-timer 동작을 시작/정지한다.
오른다리가 먼저 시작하고, 좌우 높이를 독립적으로 보간한다. `shift`는
사용하지 않는다. `n`, 좌우 방향키/A·D, `5`/`6`, hold 키는 동작하지 않고
안내만 출력한다. 위/아래는 base, `+`/`-`는 lift, `,`/`.`는 lean이며,
`1`/`2`는 step time, `3`/`4`는 leg time을 각각 10ms씩 조정한다.
`i`로 pitch 기반의 미세한 시간 변조를 켜거나 끌 수 있다(기본 OFF).
IMU는 자동 정지나 스윙 판정에 사용하지 않는다. BRUTON 정지 시 양다리는
2초 동안 base로 복귀한다. 다른 높이에서 시작하려 하면 먼저 `0`으로
base에 복귀해야 한다.

base는 85–104mm, shift는 0–10mm, lift는 0–15mm다. 개별 phase 목표가
기구학 범위 또는 관절 한계 밖이면 `v`에 `UNREACHABLE`로 표시된다. 그런
설정으로는 연속 모드를 시작할 수 없다. 동작 중에 새 설정으로 다음 phase가
거부되면 마지막 전송 목표를 유지하고, 연속 모드를 자동 정지하지 않는다.

## 6개 phase

기본값은 base=96.5mm, shift=3mm, lift=0mm,
shift/lift/plant duration=150/140/140ms, lift/plant hold=140/0ms다.
hold를 둘 다 0으로 설정하면 기존 6-phase 타이밍과 같다. `rock` 진입만으로 연속 모드가
시작되지는 않는다.

| # | phase | 왼쪽 높이 | 오른쪽 높이 | duration |
| --- | --- | ---: | ---: | --- |
| 0 | SHIFT_R | base+shift | base−shift | shift |
| 1 | LIFT_L | base+shift−lift | base−shift | lift |
| 2 | PLANT_L | base+shift | base−shift | plant |
| 3 | SHIFT_L | base−shift | base+shift | shift |
| 4 | LIFT_R | base−shift | base+shift−lift | lift |
| 5 | PLANT_R | base−shift | base+shift | plant |

각 phase는 **직전 실제 전송 높이**에서 새 목표 높이까지 독립 duration으로
보간한다. `n` 단계 모드에서는 완료 시 phase·발 x·직전 phase 대비 Δx를
한 줄 출력한다. 지지발 Δx가 크면 바닥에 붙은 발이 몸통을 앞뒤로 밀 수 있다.
연속 모드에서는 phase마다 출력하지 않고 여섯 phase가 끝날 때마다 한 줄의
roll/pitch 범위와 지지발 최대 Δx를 출력한다.

LIFT 정점 hold가 끝날 때마다 해당 다리의 knee 서보 하나를 읽어 명령한
다리 접힘량 대비 실제 접힘량을 출력한다. BRUTON은 다리별 state 1→2에서
동일하게 측정한다. `v`에는 최근 10회 추종률 평균·최소, 종료 요약에는
전체 평균이 나온다. 이 값은 **다리가 접힌 정도**이지 발의 지면
클리어런스나 실제 발-뜸 판정이 아니다. 읽기 실패 시 해당 측정만 건너뛴다.

BRUTON 기본값은 lift=6.5mm, step time=250ms, leg time=140ms이다.
각 다리는 접기 → 짧은 높이 유지 → 펴기 → 긴 높이 유지의 4-state를
독립적으로 수행한다. x 이동이나 IK는 추가하지 않았다.

평행 다리는 발바닥 각도를 유지하는 대신 높이에 따라 발 x가 바뀐다.
`foot_x = L1 sin(a) − L2 sin(−ANKLE_FIXED)`이며 base 96.5mm 부근에서는
높이 1mm당 약 1.29mm의 x 이동이 생긴다. `v`에 이 결합 비율을 표시한다.
lean의 양수는 상체를 앞으로 숙이는 방향이며, 사용 중 IMU pitch가 반대로
간다면 부호를 다시 확인해야 한다. PHASE6에서 IMU는 표시용이며,
BRUTON에서만 `i`를 켰을 때 pitch가 타이밍을 미세조정한다. 어느 패턴에서도
IMU로 자동 정지하거나 발-뜸을 판정하지 않는다.

속도 가드는 400°/s의 최후 방어선이다. 발동 시 목표를 마지막으로 성공한
전송값에 고정해 경고의 무한 반복을 막고 연속 모드를 정지한다. 서보 speed와
acceleration raw 설정은 둘 다 `0`이다.

## 권장 실험 순서

1. `status`로 여섯 서보의 응답과 각도를 확인하고, 로봇을 받친 채 `arm`한다.
2. `stand 96.5 0`으로 발 평행을 확인한다. 뒤로 기울면 rock 모드의 `.`로
   lean을 올리면서 `v`의 상대 pitch가 0 근처로 오는지 관찰한다.
3. `rock`에 들어간 뒤 lift=0, shift=3 상태로 `n`을 눌러 SHIFT_R/SHIFT_L의
   좌우 기울기를 확인한다.
4. space로 연속 rocking을 시작해 roll이 양쪽으로 진동하는지 확인한다.
5. 정지 후 `+`로 lift를 올리고 `n`으로 LIFT phase에서 발이 뜨는지 본다.
   눈으로 확인했을 때 `k`로 기록한다.
6. 뜨지 않으면 두 방향을 비교한다: shift를 0에 가깝게 줄이고 lift
   duration을 100–150ms로 줄이는 동적 방식, 또는 shift와 각 duration을
   늘리는 준정적 방식이다.
7. 출력의 지지발 Δx가 크면 base를 낮춰 앞뒤 결합을 줄인다.
