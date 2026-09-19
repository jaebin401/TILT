# ESP32-S3 통합 Pose Test

이 앱은 나중의 `main`과 같은 실행 경계를 미리 검증합니다.

`PoseInterpolator → SafetyController → ServoMapper → STS3215 UART → 6개 서보`

## 첫 실행 전

- 로봇을 스탠드에 고정해 두 발이 바닥에서 떨어진 상태로 시작합니다.
- 서보 전원 가까이에 물리 차단 수단을 둡니다. `!` 키의 토크 OFF는 통신이 끊기면 보장되지 않습니다.
- STS3215-C001에 맞는 별도 서보 전원을 사용하고 ESP32와 GND를 공통으로 연결합니다.
- Waveshare Bus Servo Adapter (A)는 UART 점퍼 A에 둡니다.
- 이 어댑터는 일반 UART 교차 배선이 아니라 `GPIO17 TX → Adapter TXD`, `GPIO18 RX → Adapter RXD`로 연결합니다.
- 서보 ID는 `11, 12, 13, 21, 22, 23`, 보레이트는 1 Mbps여야 합니다.

## 빌드와 실행

ESP-IDF 환경을 연 터미널에서:

```bash
source /Users/jae/esp/esp-idf/export.sh
cd firmware/tests/esp32_pose_test
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

현재 저장소를 확인한 환경에는 `idf.py`가 PATH에 없었으므로, ESP-IDF export가 먼저 필요합니다. 실제 포트명은 연결 후 확인합니다.

## 안전한 검증 순서

부팅하면 항상 전체 토크 OFF, `DISARMED`로 시작합니다. 자동으로 Home으로 움직이지 않습니다.

1. `status`로 6개 ID의 응답과 raw 위치가 2047 근처인지 확인합니다.
2. `directions`로 논리 방향을 확인합니다. 기본값은 전부 `+1`이며 실제 조립 방향은 아직 확정값이 아닙니다.
3. 필요하면 `direction LHP -`처럼 바꿉니다.
4. `confirm`을 입력합니다. 확인값은 재부팅하면 사라집니다.
5. `arm`을 입력합니다. 현재 위치를 Goal로 먼저 기록한 다음 토크를 켜므로 과거 Goal로 튀는 것을 막습니다.
6. `joint LHY 1`처럼 관절 하나를 1도씩 움직여 모든 방향을 먼저 확인합니다. 한 번에 최대 5도입니다.
7. 방향이 틀리면 `disarm`, `direction ...`, `confirm`, `arm` 순서로 다시 진행합니다.
8. 여섯 관절이 확인된 뒤 `home`, `pose tall`, `pose crouch`, `pose yaw-left`, `pose yaw-right`를 실행합니다.

어느 때든 `!`는 소프트웨어 E-STOP과 전체 토크 OFF를 요청합니다. 이후에는 통신 상태를 해결하고 `recover`, `confirm`, `arm`을 다시 해야 합니다.

## Pose 상태

- `home`: ADR-008에서 확정된 `[0°, -20°, +20°]` 양쪽 다리 자세입니다.
- `tall`: `[0°, -15°, +15°]`
- `crouch`: `[0°, -30°, +35°]`
- `yaw-left`: 양쪽 Hip Yaw `+5°`, 나머지는 Home
- `yaw-right`: 양쪽 Hip Yaw `-5°`, 나머지는 Home

Home만 팀 문서의 확정 자세입니다. 나머지는 작은 범위의 하드웨어 검증용 초안이며 실제 안정 자세로 확정된 값이 아닙니다.

## 현재 임시 제한값

테스트 앱 안의 `PoseTestConfig.h` 한 곳에 모았습니다.

- Hip Yaw: `-10° ... +10°`
- Hip Pitch: `-35° ... +5°`
- Knee Pitch: `-5° ... +40°`
- 최대 논리 속도: `35°/s`
- raw soft limit: `1500 ... 2600`
- pose 이동 시간: 3초, 제어 주기: 100 ms

이 값은 실측 하드웨어 한계가 아닙니다. 첫 테스트 결과로 방향, 기구 충돌 범위, 속도, 가속도를 확인한 뒤 좁혀야 합니다.
