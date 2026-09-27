# ESP32-S3 + Waveshare Bus Servo Adapter A (STS3215 x6) UART 제어 테스트

## 배선
| ESP32-S3        | Waveshare Bus Servo Adapter A |
|------------------|-------------------------------|
| GPIO17 (TX)      | TXD                            |
| GPIO18 (RX)      | RXD                            |
| GND              | GND                            |

**주의: 이 보드는 일반적인 UART 크로스 연결(TX↔RX)이 아니라 스트레이트(TX-TX, RX-RX)로 묶습니다.** Waveshare 공식 위키에 명시된 사항으로, 보드의 실크스크린 라벨이 호스트 기준으로 붙어 있기 때문입니다. 크로스로 연결하면 서보가 전혀 응답하지 않습니다.

- 보드의 점퍼는 **A 위치**(UART 모드)에 있어야 함
- 서보(STS3215)는 어댑터의 버스 서보 포트에 데이지체인으로 연결
- 서보 전원(6~12.6V)은 반드시 별도 파워서플라이에서 공급. ESP32의 5V/USB 전원으로 직접 구동하지 말 것 (ESP32는 USB로만 동작해도 되지만, 서보는 별도 전원 없이는 절대 응답도 동작도 하지 않음)
- 서보 통신은 UART1(GPIO17/18)로 분리. 콘솔(키보드 입력)은 네이티브 USB-Serial/JTAG 컨트롤러로 읽습니다 (이 보드는 별도 USB-UART 브릿지 칩 없이 네이티브 USB 하나로만 맥북과 연결되는 타입)

## 제어 방식
STS3215는 12bit 자기식 엔코더로 360도를 4096 step으로 표현하며, 최소 위치 분해능은 **0.088도(1 tick)** 입니다.
절대 각도로 바로 점프하지 않고, 키를 누를 때마다 정해진 스텝만큼만 이동합니다.

스텝 크기는 2단계이고 `m` 키로 토글합니다:

| 모드 | 스텝 | 각도 | 용도 |
|---|---|---|---|
| 정밀(FINE, 기본) | 1 tick | 0.088도 | 각도 정밀 캘리브레이션 |
| 빠름(COARSE) | 20 tick | 약 1.76도 | 구동 테스트 |

더 빠르게 하고 싶으면 `main.cpp`의 `COARSE_STEP` 값만 키우면 됩니다. 반대로 빠름 모드에서 움직임이 너무 급하면 `COARSE_GOAL_SPEED`를 1000 정도로 올려 서보 자체 속도를 제한할 수 있습니다.

부팅 시에도 목표 위치(GOAL_POSITION)를 먼저 쓰지 않고, 각 서보의 현재 위치를 `READ`로 먼저 확인한 뒤 그 값을 시작점으로 삼습니다. READ 응답을 못 받은 서보는 중앙값(2048, 약 180도)을 임시로 가정하니, 첫 몇 스텝은 실제 서보를 눈으로 보면서 확인하는 걸 권장합니다.

### 키 매핑 (Enter 불필요, 누르는 즉시 반응)
| 서보 | + | - |
|---|---|---|
| 11 | q | a |
| 12 | w | s |
| 13 | e | d |
| 21 | r | f |
| 22 | t | g |
| 23 | y | h |

- **m** : 스텝 모드 토글 (정밀 1 tick ↔ 빠름 20 tick)
- **p** : ID 스캔 (기본 0~253 범위, PING으로 실제 응답하는 서보 ID를 찾아서 출력)
- **b** : 보레이트 자동 스캔 (1M/500k/250k/128k/115200/76800/57600/38400/19200/9600을 순차적으로 바꿔가며 응답하는 값을 찾음. 찾으면 그 보레이트를 유지)
- **space** : 비상정지 (6개 서보 토크 전체 OFF)

`idf.py monitor`는 실행되는 동안 터미널을 raw 모드로 바꿔서, 키를 누르는 즉시 그 바이트를 타겟(USB-Serial/JTAG)으로 전달합니다. 그래서 별도로 Enter를 칠 필요 없이 코드가 1바이트씩 바로 읽어서 처리합니다.

## 사전 준비 (맥북)
1. ESP-IDF 설치 (v5.x 권장)
   ```bash
   git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
   cd ~/esp/esp-idf
   ./install.sh esp32s3
   . ./export.sh
   ```
   새 터미널을 열 때마다 `. ~/esp/esp-idf/export.sh` 로 환경을 다시 활성화해야 합니다.

## 빌드 및 실행
```bash
cd esp32_servo_uart_test
idf.py set-target esp32s3
idf.py build
```

ESP32-S3를 USB로 맥북에 연결한 뒤 포트 확인:
```bash
ls /dev/tty.usbmodem*   # 또는 /dev/tty.usbserial*
```

플래시 및 모니터 실행:
```bash
idf.py -p /dev/tty.usbmodemXXXX flash monitor
```

부팅 로그가 끝나면 바로 q/a/w/s/e/d/r/f/t/g/y/h 키로 서보를 움직여볼 수 있습니다.
서보가 반응이 없을 때는 우선 `b`로 보레이트부터 자동 스캔해보세요. 응답하는 보레이트를 찾으면 그 값을 유지한 채 멈추고, 그 자리에서 `p`로 ID도 바로 확인해볼 수 있습니다. 모든 보레이트에서도 응답이 없으면 보레이트 문제가 아니라 배선/전원/ID 설정 쪽을 의심해야 합니다.
`Ctrl + ]` 로 모니터를 종료합니다.

## 참고 / 주의사항
- 컨트롤 테이블 주소(Torque Enable=40, Goal Position=42, Goal Speed=46, Present Position=56)는 Feetech STS 계열 버스 서보에서 흔히 쓰이는 값을 기준으로 작성했습니다. STS3215(C001) 실제 동작이 다르면 데이터시트/메모리 맵으로 주소를 재확인하세요.
- Present Position READ 응답 파싱은 간단한 구현이라, 어댑터가 half-duplex라서 TX가 그대로 에코되어 돌아오는 구조라면 응답을 못 찾을 수 있습니다. 이 경우 로그에 READ 실패 경고가 뜨고 중앙값(2048)으로 대체되니, 실제 위치와 다를 수 있음을 감안하고 초기 스텝을 조심스럽게 확인하세요.
- 이 프로젝트는 콘솔 입력을 네이티브 USB-Serial/JTAG 컨트롤러로 읽습니다 (`console_input_init`). 만약 보드가 별도 USB-UART 브릿지 칩을 쓰는 구조라서 UART0(GPIO43/44)로 다시 바꾸고 싶다면, `console_input_init` 안을 이전의 `uart_driver_install(UART_NUM_0, ...)` 방식으로 되돌리면 됩니다.
- 서보 버스 보레이트는 STS3215 공장 출하 기본값인 1,000,000bps로 설정했습니다. 서보가 응답 없으면 이전에 보레이트를 바꾼 이력이 있는지 확인하세요.
