# ESP32-S3 + Waveshare Bus Servo Adapter A (STS3215 x6) UART 제어 테스트

## 배선
| ESP32-S3        | Waveshare Bus Servo Adapter A |
|------------------|-------------------------------|
| GPIO17 (TX)      | RXD                            |
| GPIO18 (RX)      | TXD                            |
| GND              | GND                            |

- 서보(STS3215)는 어댑터의 버스 서보 포트에 데이지체인으로 연결
- 서보 전원(6~12.6V)은 반드시 별도 파워서플라이에서 공급. ESP32의 5V/USB 전원으로 직접 구동하지 말 것
- 서보 통신은 UART1(GPIO17/18)로 분리. 콘솔(키보드 입력)은 맥북과 USB로 연결되는 UART0을 그대로 사용

## 제어 방식
STS3215는 12bit 자기식 엔코더로 360도를 4096 step으로 표현하며, 최소 위치 분해능은 **0.088도(1 tick)** 입니다.
절대 각도로 바로 점프하지 않고, **키를 한 번 누를 때마다 1 tick(0.088도)씩만** 이동합니다.

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

- **space** : 비상정지 (6개 서보 토크 전체 OFF)

`idf.py monitor`는 실행되는 동안 터미널을 raw 모드로 바꿔서, 키를 누르는 즉시 그 바이트를 타겟(UART0)으로 전달합니다. 그래서 별도로 Enter를 칠 필요 없이 코드가 1바이트씩 바로 읽어서 처리합니다.

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
`Ctrl + ]` 로 모니터를 종료합니다.

## 참고 / 주의사항
- 컨트롤 테이블 주소(Torque Enable=40, Goal Position=42, Goal Speed=46, Present Position=56)는 Feetech STS 계열 버스 서보에서 흔히 쓰이는 값을 기준으로 작성했습니다. STS3215(C001) 실제 동작이 다르면 데이터시트/메모리 맵으로 주소를 재확인하세요.
- Present Position READ 응답 파싱은 간단한 구현이라, 어댑터가 half-duplex라서 TX가 그대로 에코되어 돌아오는 구조라면 응답을 못 찾을 수 있습니다. 이 경우 로그에 READ 실패 경고가 뜨고 중앙값(2048)으로 대체되니, 실제 위치와 다를 수 있음을 감안하고 초기 스텝을 조심스럽게 확인하세요.
- 콘솔 UART0을 코드에서 직접 드라이버로 열기 때문에(`console_uart_init`), 만약 프로젝트에서 `idf.py menuconfig`로 기본 콘솔 보레이트를 115200이 아닌 다른 값으로 바꿨다면 `CONSOLE_UART_BAUD`도 같이 맞춰줘야 합니다.
- 서보 버스 보레이트는 STS3215 공장 출하 기본값인 1,000,000bps로 설정했습니다. 서보가 응답 없으면 이전에 보레이트를 바꾼 이력이 있는지 확인하세요.
