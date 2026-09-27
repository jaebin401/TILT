# TILT orbit_test

H-LIP 관상면(P2, 순 좌우 이동 0) 궤도를 참고한 **제자리 발 들기 실험**이다.
전진 x 스텝, IK, IMU 자동 정지는 없다. 기존 테스트와 공용 컴포넌트는
수정하지 않는다. 실험은 물리적 서보 전원 차단 수단을 손 닿는 곳에 두고 한다.

## 빌드와 PC 검산

```bash
source /Users/jae/esp/esp-idf/export.sh
cd firmware/tests/orbit_test
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

```bash
cmake -S firmware/tests/orbit_test/host_test -B /tmp/tilt-orbit-host-build
cmake --build /tmp/tilt-orbit-host-build
ctest --test-dir /tmp/tilt-orbit-host-build --output-on-failure
```

`OrbitModel.h`, `LegPose.h`, `Bezier.h`는 ESP 의존성이 없는 헤더다.
Fusion 실측 상수로부터 roll 반경·유효 높이·고유 지지 시간·예측 진폭을
계산한다. 모델의 예상치는 약 T*=320ms, roll 진폭 ±4.6°,
교대 각속도 65°/s, 스윙쪽 hip 상승 4.4mm이다. 이는 검증 대상인
예측치이지 TILT의 실측 보행 결과가 아니다.

## 필수 안전 순서

1. 부팅은 `DISARMED`, 토크 OFF, 자동 이동 없음이다. `status`로 6축 응답을
   확인하고, 로봇을 받친 상태에서 `arm`한다. `arm`은 현재 서보 위치를
   hold goal로 전송한 뒤 토크를 켠다.
2. 로봇을 **공중에 매단 상태**로 `check`를 실행한다. ID 11–13 블록이
   5mm 접힌 후, 실제 올라간 다리가 로봇 기준 왼쪽이면 `l`, 오른쪽이면
   `r`을 누른다. 원위치가 끝나면 똑바로 잡고 `tilt` + Enter를 입력한 뒤
   2초 동안 로봇 기준 왼쪽으로 기울인다. 결과가 현재 부팅에서 즉시
   적용된다. 재부팅 뒤에도 유지하려면 결과에 나온 `kSwapLegSides`와
   `kRollSign`을 `main/OrbitConfig.h`에 반영한다.
3. 로봇을 바닥에 내리고 `stand` (기본 96.5mm), `gait`, `r` 순서로
   똑바로 정지한 상태의 roll 기준을 잡는다. `r`은 움직이는 동안 거부된다.
4. `1` ROCK으로 시작한다. `space`를 눌러 공진 흔들기의 실측 반주기와
   roll p-p를 모델 예측(약 320ms, 9.2°)과 비교한다. 다르면
   유효 높이 또는 발판 안쪽 지지점 가정을 재검토한다.
5. `space`로 멈춘 뒤 `2` OPEN에서 슬로모션으로 발이 실제로 뜨는지 확인한다.
   `3` ORBIT은 이번 부팅에서 `check` 통과 전에는 시작할 수 없다.
   ORBIT에서 `e`로 에너지 조절을 켜고 워치독 횟수와 교대 각속도를 본다.

좌우 서보 블록의 실제 다리 대응은 미검증이다. 잘못 매핑하면 지지발을
들어 넘어질 수 있으므로 `check` 없이 바닥에서 보행을 시도하지 않는다.
ROCK/OPEN은 미확인 상태에서도 경고 후 허용하지만 안전하다는 뜻이 아니다.
IMU 읽기가 실패해도 자동 정지하지 않는다. ORBIT은 1.5T 워치독으로
교대를 계속한다. 사람이 `space`, `!`, 또는 물리적 전원 차단으로 멈춘다.

## 명령과 키

줄 단위 명령: `help`, `status`, `model`, `arm`, `disarm`, `recover`,
`check`, `tilt`(check 안내가 나온 뒤), `stand [height_mm] [lean_deg]`,
`gait`. `!`는 어떤 모드에서든 긴급 토크 OFF를 요청한다.

`gait`에서는 Enter 없이 한 키씩 입력한다.

| 키 | 동작 |
| --- | --- |
| `1` / `2` / `3` | ROCK / OPEN / ORBIT 선택 (정지 중) |
| space | 시작 / 정지 (정지 시 두 다리 nominal stand 복귀) |
| `[` / `]` | 한 발 지지 시간 T ∓/±10ms (0.20–0.50s) |
| `+` / `-` | Bézier lift 제어점 z_max ±0.5mm |
| `,` / `.` | 밀어주기 δ ∓/±0.5mm |
| `e` | ORBIT 에너지 조절 ON/OFF |
| `s` / `S` | STARTUP 높이차 ∓/±0.5mm |
| `r` | 정지 중 roll 기준 재설정 |
| `c` | 100Hz CSV ON/OFF |
| `m` / `v` | 모델 예측 / 현재 상태·측정 요약 |
| `q` | 결과 출력, nominal stand 복귀 후 줄 단위 콘솔 |

ROCK은 타이머 T로 양발 높이차만 번갈아 주며 발을 들지 않는다.
OPEN·ORBIT은 먼저 STARTUP에서 roll 진폭이 예측 진폭의 70% 이상인
반주기가 연속 두 번 나와야 SSP를 시작한다. 12사이클 안에 충족하지 못하면
경고하고 흔들기만 계속한다. OPEN은 타이머, ORBIT은 roll 영점 교차와
자이로 각속도로 지지발을 교대한다. ORBIT의 `e`는 교대 속도 오차에 따라
다음 SSP의 δ를 0–4mm 범위에서 조절한다.

명령 lift 정점은 Bézier 제어점의 약 0.875배이다. SSP 중간에 스윙 다리의
무릎 서보 **하나만** 읽어 실제 다리 접힘량을 계산한다. 출력의
`estimated clearance = 실제 접힘량 + D·sin(roll 피크)`는 **모델 기반
추정값이며, 발과 바닥 사이 간격을 실측한 값이 아니다.** 밀어주기에 따른
지지발 앞뒤 밀림도 δ·cot(a)의 추정치다. CSV는
`t_ms,mode,state,roll_deg,rate_deg_s,h_L,h_R,delta`를 매 10ms 출력하며,
켜져 있는 동안 반주기 텍스트 출력은 숨긴다.
