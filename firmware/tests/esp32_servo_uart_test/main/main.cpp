// ESP32-S3 + Waveshare Bus Servo Adapter A (STS3215 x6) UART 제어 테스트
// - 절대 각도로 바로 점프하지 않고, 키를 누를 때마다 최소 분해능(1 tick = 0.088도)만큼만 증분 이동
// - Enter 없이 키 1개 입력마다 즉시 반응 (idf.py monitor는 터미널을 raw 모드로 돌려서
//   키 입력을 누르는 즉시 바이트 단위로 전달해줌)
//
// 배선:
//   ESP32-S3 GPIO17 (TX) -> Adapter RXD
//   ESP32-S3 GPIO18 (RX) -> Adapter TXD
//   GND 공통, 서보 전원은 별도 파워서플라이 사용 (ESP32 5V로 직접 구동 금지)
//
// 키 매핑 (QWERTY 두 줄, 윗줄 +/아랫줄 -):
//   서보 11 : q(+) / a(-)
//   서보 12 : w(+) / s(-)
//   서보 13 : e(+) / d(-)
//   서보 21 : r(+) / f(-)
//   서보 22 : t(+) / g(-)
//   서보 23 : y(+) / h(-)
//   스페이스바 : 비상정지 (6개 서보 토크 전체 OFF)

#include <cstdio>
#include <cstring>
#include <cctype>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "servo_test";

// ---------------- 서보 버스 UART 설정 (STS3215 x6) ----------------
#define SERVO_UART_NUM      UART_NUM_1
#define SERVO_UART_TX_PIN   GPIO_NUM_17
#define SERVO_UART_RX_PIN   GPIO_NUM_18
#define SERVO_UART_BAUD     1000000   // STS3215 공장 출하 기본 보레이트(1Mbps)

// ---------------- 콘솔(키보드 입력) UART 설정 ----------------
// idf.py monitor가 붙는 포트 = 기본 콘솔 UART0. 핀은 보드 기본값 그대로 사용.
#define CONSOLE_UART_NUM     UART_NUM_0
#define CONSOLE_UART_BAUD    115200    // menuconfig 기본 콘솔 보레이트와 동일해야 함

// ---------------- STS/SCS 버스 서보 프로토콜 ----------------
// 패킷 구조: [0xFF][0xFF][ID][LEN][INSTR][PARAMS...][CHECKSUM]
// LEN = INSTR(1) + PARAMS(N) + CHECKSUM(1)
// CHECKSUM = ~(ID + LEN + INSTR + sum(PARAMS)) & 0xFF
//
// 주의: 아래 컨트롤 테이블 주소는 Feetech STS 계열에서 널리 쓰이는 값입니다.
// 실제 동작이 다르면 STS3215 데이터시트/메모리 맵으로 주소를 다시 확인하세요.
#define INST_READ             0x02
#define INST_WRITE            0x03
#define ADDR_TORQUE_ENABLE    40   // 0x28, 1 byte
#define ADDR_GOAL_POSITION    42   // 0x2A, 2 byte (little endian)
#define ADDR_GOAL_SPEED       46   // 0x2E, 2 byte (0 = 최대 속도)
#define ADDR_PRESENT_POSITION 56   // 0x38, 2 byte (little endian)

// 최소 분해능 1 tick = 0.088도 (4096 step / 360도, STS3215 12bit 엔코더 기준)
#define POS_MIN 0
#define POS_MAX 4095

// 사용할 서보 ID: 왼쪽 다리(11,12,13) / 오른쪽 다리(21,22,23)
static const uint8_t SERVO_IDS[6] = {11, 12, 13, 21, 22, 23};
static uint16_t s_cur_pos[6]; // 각 서보의 현재(로컬에서 추적하는) 목표 위치

// 키 -> (서보 인덱스, 방향) 매핑
struct KeyMap { char key; int servo_idx; int8_t dir; };
static const KeyMap KEY_MAP[12] = {
    {'q', 0, +1}, {'a', 0, -1}, // 서보 11
    {'w', 1, +1}, {'s', 1, -1}, // 서보 12
    {'e', 2, +1}, {'d', 2, -1}, // 서보 13
    {'r', 3, +1}, {'f', 3, -1}, // 서보 21
    {'t', 4, +1}, {'g', 4, -1}, // 서보 22
    {'y', 5, +1}, {'h', 5, -1}, // 서보 23
};

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

// 콘솔 UART0을 드라이버로 직접 열어서, printf/로그 출력은 그대로 두고
// 입력만 uart_read_bytes로 1바이트씩 받아온다 (fgets처럼 Enter를 기다리지 않음).
static void console_uart_init() {
    uart_config_t cfg = {};
    cfg.baud_rate = CONSOLE_UART_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONSOLE_UART_NUM, &cfg));
    // 핀은 보드 기본 USB-UART 브릿지 핀 그대로 사용 (uart_set_pin 호출 안 함)
}

// 공통 WRITE 패킷 전송 (addr에서 시작하는 data_len 바이트를 씀)
static void servo_write(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t data_len) {
    uint8_t param_len = data_len + 1; // addr(1) + data
    uint8_t len = param_len + 2;      // INSTR + PARAMS + CHECKSUM

    uint8_t pkt[6 + 8];
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

static void servo_write_pos(uint8_t id, uint16_t position, uint16_t speed) {
    uint8_t data[4] = {
        (uint8_t)(position & 0xFF),
        (uint8_t)((position >> 8) & 0xFF),
        (uint8_t)(speed & 0xFF),
        (uint8_t)((speed >> 8) & 0xFF),
    };
    servo_write(id, ADDR_GOAL_POSITION, data, sizeof(data));
}

// 현재 위치 READ. 응답을 받으면 true, 못 받으면 false.
// 응답 패킷: [0xFF][0xFF][ID][LEN][ERR][DATA...][CHECKSUM]
static bool servo_read_pos(uint8_t id, uint16_t *out_pos) {
    uint8_t param[2] = { ADDR_PRESENT_POSITION, 2 };
    uint8_t len = sizeof(param) + 2;
    uint8_t pkt[8];
    int idx = 0;
    pkt[idx++] = 0xFF;
    pkt[idx++] = 0xFF;
    pkt[idx++] = id;
    pkt[idx++] = len;
    pkt[idx++] = INST_READ;
    uint32_t sum = id + len + INST_READ;
    for (uint8_t b : param) { pkt[idx++] = b; sum += b; }
    pkt[idx++] = (uint8_t)(~sum);

    uart_flush_input(SERVO_UART_NUM);
    uart_write_bytes(SERVO_UART_NUM, (const char *)pkt, idx);

    uint8_t resp[16];
    int got = uart_read_bytes(SERVO_UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(50));
    if (got < 8) return false;

    for (int i = 0; i <= got - 8; ++i) {
        if (resp[i] == 0xFF && resp[i + 1] == 0xFF && resp[i + 2] == id) {
            uint8_t pos_l = resp[i + 5];
            uint8_t pos_h = resp[i + 6];
            *out_pos = (uint16_t)(pos_l | (pos_h << 8));
            return true;
        }
    }
    return false;
}

static uint16_t clamp_pos(int32_t v) {
    if (v < POS_MIN) return POS_MIN;
    if (v > POS_MAX) return POS_MAX;
    return (uint16_t)v;
}

static void print_help() {
    printf("\n서보 증분 제어 (키 1개 = 1 tick = 0.088도, Enter 불필요)\n");
    printf("  q/a : 서보 11  +/-\n");
    printf("  w/s : 서보 12  +/-\n");
    printf("  e/d : 서보 13  +/-\n");
    printf("  r/f : 서보 21  +/-\n");
    printf("  t/g : 서보 22  +/-\n");
    printf("  y/h : 서보 23  +/-\n");
    printf("  space : 비상정지 (전체 토크 OFF)\n\n");
}

// 모든 서보 토크 OFF (비상정지)
static void emergency_stop() {
    for (uint8_t id : SERVO_IDS) {
        servo_torque_enable(id, false);
    }
    printf("!! 비상정지: 전체 토크 OFF !!\n");
}

// ---------------- 키보드(시리얼 콘솔) 1바이트 입력 처리 ----------------
static void console_task(void *arg) {
    print_help();

    uint8_t ch;
    while (true) {
        int len = uart_read_bytes(CONSOLE_UART_NUM, &ch, 1, pdMS_TO_TICKS(50));
        if (len <= 0) {
            continue;
        }

        if (ch == ' ') {
            emergency_stop();
            continue;
        }

        char key = (char)tolower(ch);
        bool matched = false;
        for (const auto &km : KEY_MAP) {
            if (km.key == key) {
                matched = true;
                int idx = km.servo_idx;
                uint16_t new_pos = clamp_pos((int32_t)s_cur_pos[idx] + km.dir);
                s_cur_pos[idx] = new_pos;
                servo_write_pos(SERVO_IDS[idx], new_pos, 0);
                printf("servo %d : pos=%u (%.3f deg)\n",
                       SERVO_IDS[idx], new_pos, new_pos * 360.0f / 4096.0f);
                break;
            }
        }
        // 매핑 안 된 키(엔터, 방향키 등)는 무시
        (void)matched;
    }
}

extern "C" void app_main(void) {
    servo_uart_init();
    console_uart_init();

    // 부팅 시 절대 위치 명령은 보내지 않는다.
    // 각 서보의 실제 현재 위치를 READ로 확인해서 로컬 기준값으로만 저장하고,
    // 그 값에서부터 ±1 tick씩만 움직이도록 해서 초기 스냅(급격한 위치 점프)을 방지한다.
    for (int i = 0; i < 6; ++i) {
        uint16_t pos = 2048; // 읽기 실패 시 기본값(중앙, 약 180도)
        if (servo_read_pos(SERVO_IDS[i], &pos)) {
            ESP_LOGI(TAG, "servo %d 현재 위치 읽음: pos=%u", SERVO_IDS[i], pos);
        } else {
            ESP_LOGW(TAG, "servo %d 위치 READ 실패 -> 임시로 중앙값(2048) 가정. "
                           "첫 스텝 이동 시 실제 위치와 다를 수 있으니 확인 필요", SERVO_IDS[i]);
        }
        s_cur_pos[i] = pos;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    for (uint8_t id : SERVO_IDS) {
        servo_torque_enable(id, true);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "6개 서보 토크 ON 완료 (ID: 11,12,13,21,22,23)");

    xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);
}
