// ESP32-S3 + Waveshare Bus Servo Adapter A (STS3215 x6) UART 제어 테스트
//
// 배선:
//   ESP32-S3 GPIO17 (TX) -> Adapter RXD
//   ESP32-S3 GPIO18 (RX) -> Adapter TXD
//   GND 공통, 서보 전원은 별도 파워서플라이 사용 (ESP32 5V로 직접 구동 금지)
//
// 사용법:
//   idf.py -p /dev/tty.usbmodemXXXX flash monitor 로 접속한 뒤
//   시리얼 모니터(=맥북 키보드 입력)에서 "<id> <angle>" 형식으로 입력
//   예) 11 90   -> ID 11번 서보를 90도로 이동
//       21 180  -> ID 21번 서보를 180도로 이동

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "servo_test";

// ---------------- 서보 버스 UART 설정 ----------------
#define SERVO_UART_NUM      UART_NUM_1
#define SERVO_UART_TX_PIN   GPIO_NUM_17
#define SERVO_UART_RX_PIN   GPIO_NUM_18
#define SERVO_UART_BAUD     1000000   // STS3215 공장 출하 기본 보레이트(1Mbps)

// ---------------- STS/SCS 버스 서보 프로토콜 ----------------
// 패킷 구조: [0xFF][0xFF][ID][LEN][INSTR][PARAMS...][CHECKSUM]
// LEN = INSTR(1) + PARAMS(N) + CHECKSUM(1)
// CHECKSUM = ~(ID + LEN + INSTR + sum(PARAMS)) & 0xFF
//
// 주의: 아래 컨트롤 테이블 주소는 Feetech STS 계열에서 널리 쓰이는 값입니다.
// 실제 동작이 다르면 STS3215 데이터시트/메모리 맵으로 주소를 다시 확인하세요.
#define INST_WRITE            0x03
#define ADDR_TORQUE_ENABLE    40   // 0x28, 1 byte
#define ADDR_GOAL_POSITION    42   // 0x2A, 2 byte (little endian)
#define ADDR_GOAL_SPEED       46   // 0x2E, 2 byte (0 = 최대 속도)

static void servo_uart_init() {
    uart_config_t cfg = {};
    cfg.baud_rate = SERVO_UART_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(SERVO_UART_NUM, 512, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SERVO_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(SERVO_UART_NUM, SERVO_UART_TX_PIN, SERVO_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// 공통 WRITE 패킷 전송 (addr에서 시작하는 data_len 바이트를 씀)
static void servo_write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t data_len) {
    uint8_t param_len = data_len + 1; // addr(1) + data
    uint8_t len = param_len + 2;      // INSTR + PARAMS + CHECKSUM

    uint8_t pkt[6 + 8]; // params 최대 8바이트로 가정 (여유)
    int idx = 0;
    pkt[idx++] = 0xFF;
    pkt[idx++] = 0xFF;
    pkt[idx++] = id;
    pkt[idx++] = len;
    pkt[idx++] = INST_WRITE;
    pkt[idx++] = addr;

    uint32_t sum = id + len + INST_WRITE + addr;
    for (int i = 0; i < data_len; ++i) {
        pkt[idx++] = data[i];
        sum += data[i];
    }
    pkt[idx++] = (uint8_t)(~sum);

    uart_write_bytes(SERVO_UART_NUM, (const char *)pkt, idx);
}

static void servo_torque_enable(uint8_t id, bool on) {
    uint8_t v = on ? 1 : 0;
    servo_write(id, ADDR_TORQUE_ENABLE, &v, 1);
}

// position: 0~4095 (0~360도에 대응, 서보 스펙에 따라 실제 범위는 다를 수 있음)
// speed: 0~약 3400, 0이면 최대 속도
static void servo_write_pos(uint8_t id, uint16_t position, uint16_t speed) {
    uint8_t data[4] = {
        (uint8_t)(position & 0xFF),
        (uint8_t)((position >> 8) & 0xFF),
        (uint8_t)(speed & 0xFF),
        (uint8_t)((speed >> 8) & 0xFF),
    };
    servo_write(id, ADDR_GOAL_POSITION, data, sizeof(data));
}

// ---------------- 키보드(시리얼 콘솔) 명령 처리 ----------------
static void console_task(void *arg) {
    char line[64];
    printf("\n서보 제어 준비 완료. 입력 형식: <id> <angle 0~360>\n");
    printf("예) 11 90   -> ID 11번 서보를 90도로 이동\n\n");

    while (true) {
        if (fgets(line, sizeof(line), stdin) != NULL) {
            int id = 0, angle = 0;
            if (sscanf(line, "%d %d", &id, &angle) == 2) {
                if (angle < 0) angle = 0;
                if (angle > 360) angle = 360;
                uint16_t pos = (uint16_t)((angle / 360.0f) * 4095.0f);
                servo_write_pos((uint8_t)id, pos, 0);
                printf("-> servo %d : %d deg (pos=%u)\n", id, angle, pos);
            } else {
                printf("입력 형식 오류. 예) 11 90\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void) {
    servo_uart_init();

    // 사용할 서보 ID: 왼쪽 다리(11,12,13) / 오른쪽 다리(21,22,23)
    const uint8_t ids[6] = {11, 12, 13, 21, 22, 23};

    for (uint8_t id : ids) {
        servo_torque_enable(id, true);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "6개 서보 토크 ON 완료 (ID: 11,12,13,21,22,23)");

    xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);
}
