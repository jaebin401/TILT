# ESP32-S3 + Waveshare Bus Servo Adapter A (STS3215 x6) UART 제어 테스트

## 배선
| ESP32-S3        | Waveshare Bus Servo Adapter A |
|------------------|-------------------------------|
| GPIO17 (TX)      | RXD                            |
| GPIO18 (RX)      | TXD                            |
| GND              | GND                            |

- 서보(STS3215)는 어댑터의 버스 서보 포트에 데이지체인으로 연결
- 서보 전원(6~12.6V)은 반드시 별도 파워서플라이에서 공급. ESP32의 5V/USB 전원으로 직접 구동하지 말 것
- 이 예제는 UART0(맥북과 USB로 연결되는 기본 콘솔 포트)는 그대로 `idf.py monitor` 콘솔/키보드 입력용으로 남겨두고, 서보 통신은 UART1(GPIO17/18)로 분리했습니다. USB-UART 브릿지가 없는 보드(네이티브 USB만 있는 S3 보드)를 쓴다면 콘솔 동작 방식이 다를 수 있으니 보드 스펙을 확인하세요.

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

## 사용법
모니터가 뜨면(부팅 로그 이후) 키보드로 아래 형식을 입력하고 Enter:
```
<서보 ID> <각도(0~360)>
```
예:
```
11 90
21 180
13 45
```
`Ctrl + ]` 로 모니터를 종료합니다.

## 참고 / 주의사항
- 코드의 컨트롤 테이블 주소(Torque Enable=40, Goal Position=42, Goal Speed=46)는 Feetech STS 계열 버스 서보에서 흔히 쓰이는 값을 기준으로 작성했습니다. STS3215(C001) 실제 동작이 다르면 데이터시트/메모리 맵으로 주소를 재확인하세요.
- 현재는 위치만 지령하는 단순 WRITE이며, 응답(READ)이나 에러 체크는 구현하지 않았습니다. 통신 확인이 필요하면 PING(0x01) 명령이나 Present Position(주소 56 부근) READ를 추가하면 됩니다.
- 보레이트는 STS3215 공장 출하 기본값인 1,000,000bps로 설정했습니다. 서보가 응답 없으면 이전에 보레이트를 바꾼 이력이 있는지 확인하세요.
