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
2. 현재 설정은 `ID 11–13 = 오른다리`, `raw roll sign = -1`로
   `main/OrbitConfig.h`에 영속화되어 부팅 직후 ORBIT을 허용한다. 그래도
   로봇을 **공중에 매단 상태**로 `check`를 실행해 실제 배선과 장착 방향을
   다시 확인한다. ID 11–13 블록이
   5mm 접힌 후, 실제 올라간 다리가 로봇 기준 왼쪽이면 `l`, 오른쪽이면
   `r`을 누른다. 원위치가 끝나면 똑바로 잡고 `tilt` + Enter를 입력한 뒤
   2초 동안 로봇 기준 왼쪽으로 기울인다. 결과가 현재 부팅에서 즉시
   적용된다. 결과가 헤더와 다르면 경고가 나오므로, 확인된
   `kSwapLegSides`와 `kRollSign`을 `main/OrbitConfig.h`에 다시 반영한다.
3. 로봇을 바닥에 내리고 `stand` (기본 96.5mm), `gait`, `r` 순서로
   똑바로 정지한 상태의 roll 기준을 잡는다. `r`은 움직이는 동안 거부된다.
4. `1` ROCK으로 시작한다. `space`를 눌러 공진 흔들기의 실측 반주기와
   roll p-p를 모델 예측(약 320ms, 9.2°)과 비교한다. 다르면
   유효 높이 또는 발판 안쪽 지지점 가정을 재검토한다.
5. `space`로 멈춘 뒤 `2` OPEN에서 슬로모션으로 발이 실제로 뜨는지 확인한다.
   `3` ORBIT은 헤더의 영속 매핑 또는 이번 부팅의 `check` 결과가 있어야
   시작할 수 있다. ORBIT에서 `e`로 에너지 조절을 켜고 워치독 횟수와
   교대 각속도를 본다.

현재 좌우 매핑은 헤더에 저장되어 있지만, 하드웨어 변경 뒤 잘못된 매핑이면
지지발을 들어 넘어질 수 있다. 배선이나 IMU 장착을 바꿨다면 반드시 공중에서
`check`를 다시 수행한다. 헤더가 기본값이면 ROCK/OPEN은 경고 후 허용하지만
안전하다는 뜻이 아니며 ORBIT은 잠긴다.
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
| `k` | 비동기 스윙 knee 계측 ON/OFF (기본 OFF) |
| `m` / `v` | 모델 예측 / 현재 상태·측정 요약 |
| `q` | 결과 출력, nominal stand 복귀 후 줄 단위 콘솔 |

ROCK은 타이머 T로 양발 높이차만 번갈아 주며 발을 들지 않는다.
OPEN·ORBIT은 먼저 STARTUP에서 roll 진폭이 예측 진폭의 70% 이상인
반주기가 연속 두 번 나와야 SSP를 시작한다. 12사이클 안에 충족하지 못하면
경고하고 흔들기만 계속한다. OPEN은 타이머, ORBIT은 roll 영점 교차와
자이로 각속도로 지지발을 교대한다. 교차 판정은 SSP의 25% 이후부터
|rate| 8°/s 이상에서 허용하며, 1.5T 워치독은 그대로 유지한다. ORBIT의
`e`는 교대 속도 오차에 따라 다음 SSP의 δ를 0–7mm 범위에서 조절한다.
push는 SSP 전체에서 선형 증가한다. 교대 시 새 스윙발은 직전 높이에서
SSP 앞 20% 동안 기존 Bézier 궤적에 합류하고, 새 지지발은 직전 높이에서
50ms 동안 nominal+push 목표로 이동해 양쪽 높이 명령의 불연속을 없앤다.

명령 lift 정점은 Bézier 제어점의 약 0.875배이다. lift 계측은 기본 OFF라서
출력에 `--`로 표시된다. `k`로 켜면 낮은 우선순위의 별도 태스크가 SSP
중간에 스윙 다리의 무릎 서보 **하나만** 한 번 읽어 실제 다리 접힘량을
계산한다. 버스를 즉시 확보하지 못하거나 읽기가 실패해도 재시도하지 않으며,
연속 5회 실패하면 자동으로 OFF 된다. 따라서 100Hz 제어·CSV 루프는 계측
응답을 기다리지 않는다. 출력의
`estimated clearance = 실제 접힘량 + D·sin(roll 피크)`는 **모델 기반
추정값이며, 발과 바닥 사이 간격을 실측한 값이 아니다.** 밀어주기에 따른
지지발 앞뒤 밀림도 δ·cot(a)의 추정치다. CSV는
`t_ms,mode,state,roll_deg,rate_deg_s,h_L,h_R,delta`를 매 10ms 출력하며,
켜져 있는 동안 반주기 텍스트 출력은 숨긴다. 종료 요약에는 20ms를 넘은
루프 지연 횟수와 최대 지연, 비정상 dt 또는 과도한 rate 때문에 거부한 IMU
샘플 수가 함께 표시된다. ORBIT의 영점 교차는 최근 roll 3샘플의 중앙값으로
판정한다. 결과에는 push로 생긴 지지발 전후 이동의 최대 추정값(`max x drift`)
도 표시된다.
