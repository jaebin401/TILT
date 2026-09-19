// ESP32-S3 + Waveshare Bus Servo Adapter A (STS3215 x6) UART 제어 테스트
// - 절대 각도로 바로 점프하지 않고, 키를 누를 때마다 정해진 스텝만큼만 증분 이동
// - 스텝 크기는 2단계: 정밀(1 tick = 0.088도) / 빠름(기본 20 tick 약 1.76도), 'm' 키로 토글
// - Enter 없이 키 1개 입력마다 즉시 반응 (idf.py monitor는 터미널을 raw 모드로 돌려서
//   키 입력을 누르는 즉시 바이트 단위로 전달해줌)
//
// 배선 (주의: 이 보드는 일반적인 UART 크로스 연결이 아니라 스트레이트로 묶는다):
//   ESP32-S3 GPIO17 (TX) -> Adapter TXD
//   ESP32-S3 GPIO18 (RX) -> Adapter RXD
//   보드의 점퍼는 A 위치(UART 모드)
//   GND 공통, 서보 전원은 별도 파워서플라이 사용 (ESP32 5V로 직접 구동 금지)
//
// 키 매핑 (QWERTY 두 줄, 윗줄 +/아랫줄 -):
//   서보 11 : q(+) / a(-)
//   서보 12 : w(+) / s(-)
//   서보 13 : e(+) / d(-)
//   서보 21 : r(+) / f(-)
//   서보 22 : t(+) / g(-)
//   서보 23 : y(+) / h(-)
//   m : 스텝 모드 토글 (정밀 <-> 빠름)
//   p : 서보 ID 스캔
//   b : 보레이트 자동 스캔
//   스페이스바 : 비상정지 (6개 서보 토크 전체 OFF)

#include <cstdio>
#include <cstring>
#include <cctype>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"

static const char *TAG = "servo_test";

// ---------------- 서보 버스 UART 설정 (STS3215 x6) ----------------
#define SERVO_UART_NUM      UART_NUM_1
#define SERVO_UART_TX_PIN   GPIO_NUM_17
#define SERVO_UART_RX_PIN   GPIO_NUM_18
#define SERVO_UART_BAUD     1000000   // STS3215 공장 출하 기본 보레이트(1Mbps)

// ---------------- 콘솔(키보드 입력) 설정 ----------------
// 이 보드는 별도 USB-UART 브릿지 칩 없이 ESP32-S3 내장 USB-Serial/JTAG 컨트롤러로
// 맥북과 통신한다 (esptool 연결 로그의 "USB mode: USB-Serial/JTAG"가 그 증거).
// 그래서 UART_NUM_0(GPIO43/44) 물리 핀이 아니라 usb_serial_jtag 드라이버로 입력을 읽어야 한다.

// ---------------- STS/SCS 버스 서보 프로토콜 ----------------
// 패킷 구조: [0xFF][0xFF][ID][LEN][INSTR][PARAMS...][CHECKSUM]
// LEN = INSTR(1) + PARAMS(N) + CHECKSUM(1)
// CHECKSUM = ~(ID + LEN + INSTR + sum(PARAMS)) & 0xFF
//
// 주의: 아래 컨트롤 테이블 주소는 Feetech STS 계열에서 널리 쓰이는 값입니다.
// 실제 동작이 다르면 STS3215 데이터시트/메모리 맵으로 주소를 다시 확인하세요.
#define INST_PING             0x01
#define INST_READ             0x02
#define INST_WRITE            0x03
#define ADDR_TORQUE_ENABLE    40   // 0x28, 1 byte
#define ADDR_GOAL_POSITION    42   // 0x2A, 2 byte (little endian)
#define ADDR_GOAL_SPEED       46   // 0x2E, 2 byte (0 = 최대 속도)
#define ADDR_PRESENT_POSITION 56   // 0x38, 2 byte (little endian)

// 최소 분해능 1 tick = 0.088도 (4096 step / 360도, STS3215 12bit 엔코더 기준)
#define POS_MIN 0
#define POS_MAX 4095

// ---------------- 스텝 모드 (2단계) ----------------
// FINE   : 각도 정밀 캘리브레이션용. 한 번에 1 tick(0.088도)만 움직임.
// COARSE : 구동 테스트용. 한 번에 COARSE_STEP tick씩 움직임.
#define FINE_STEP    1     // 0.088도
#define COARSE_STEP  20    // 약 1.76도 (더 빠르게 하려면 이 값만 키우면 됨)

// 서보에 실어 보내는 GOAL_SPEED (0 = 서보 최대 속도)
// 스텝이 작을 때는 속도를 낮춰도 체감이 거의 없어 정밀 모드는 0으로 둔다.
// 빠름 모드에서 움직임이 너무 급하면 COARSE_GOAL_SPEED를 1000 정도로 올려 제한할 수 있다.
#define FINE_GOAL_SPEED    0
#define COARSE_GOAL_SPEED  0

static bool s_coarse_mode = false; // false = 정밀 모드(기본), true = 빠름 모드

static int current_step()              { return s_coarse_mode ? COARSE_STEP : FINE_STEP; }
static uint16_t current_speed()        { return s_coarse_mode ? COARSE_GOAL_SPEED : FINE_GOAL_SPEED; }
static const char *current_mode_name() { return s_coarse_mode ? "빠름(COARSE)" : "정밀(FINE)"; }

// 'p' 키로 실행하는 ID 스캔 범위 (필요하면 넓혀도 됨, 최대 253)
#define SCAN_ID_MIN 0
#define SCAN_ID_MAX 253

// 'b' 키로 실행하는 보레이트 자동 스캔 후보 (STS 계열이 지원하는 흔한 값들)
static const uint32_t BAUD_CANDIDATES[] = {
    1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400, 19200, 9600
};

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

// 콘솔 입력을 USB-Serial/JTAG 드라이버에서 직접 읽는다 (fgets처럼 Enter를 기다리지 않음).
// IDF 콘솔이 부팅 시 이미 이 드라이버를 설치해뒀을 수 있으므로, 이미 설치된 상태(ESP_ERR_INVALID_STATE)는
// 에러로 취급하지 않고 그냥 기존 드라이버를 그대로 사용한다.
static void console_input_init() {
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }
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

// PING: 해당 ID의 서보가 버스에 붙어있으면 true.
// 응답 패킷: [0xFF][0xFF][ID][LEN][ERR][CHECKSUM] (파라미터 없음)
static bool servo_ping(uint8_t id) {
    uint8_t len = 2; // INSTR + CHECKSUM
    uint8_t pkt[6];
    int idx = 0;
    pkt[idx++] = 0xFF;
    pkt[idx++] = 0xFF;
    pkt[idx++] = id;
    pkt[idx++] = len;
    pkt[idx++] = INST_PING;
    uint32_t sum = id + len + INST_PING;
    pkt[idx++] = (uint8_t)(~sum);

    uart_flush_input(SERVO_UART_NUM);
    uart_write_bytes(SERVO_UART_NUM, (const char *)pkt, idx);

    uint8_t resp[8];
    int got = uart_read_bytes(SERVO_UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(15));
    if (got < 6) return false; // 최소 응답 길이: 0xFF 0xFF ID LEN ERR CHK

    for (int i = 0; i <= got - 6; ++i) {
        if (resp[i] == 0xFF && resp[i + 1] == 0xFF && resp[i + 2] == id) {
            return true;
        }
    }
    return false;
}

// SCAN_ID_MIN ~ SCAN_ID_MAX 범위를 순회하며 응답하는 서보 ID를 출력하고 개수를 반환한다.
static int scan_servo_ids_quiet() {
    int found = 0;
    for (int id = SCAN_ID_MIN; id <= SCAN_ID_MAX; ++id) {
        if (servo_ping((uint8_t)id)) {
            printf("    -> 응답 있음: ID %d\n", id);
            found++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return found;
}

// SCAN_ID_MIN ~ SCAN_ID_MAX 범위를 순회하며 실제로 응답하는 서보 ID를 출력한다.
static void scan_servo_ids() {
    printf("\n=== 서보 ID 스캔 시작 (ID %d~%d) ===\n", SCAN_ID_MIN, SCAN_ID_MAX);
    int found = scan_servo_ids_quiet();
    printf("=== 스캔 완료: %d개 서보 응답 ===\n\n", found);
    if (found == 0) {
        printf("응답한 서보가 하나도 없습니다. 서보 전원/배선/보레이트를 다시 확인하세요.\n\n");
    }
}

// 여러 보레이트를 순차적으로 바꾸며 각 보레이트에서 ID 스캔을 돌려본다.
// 응답이 있는 보레이트를 찾으면 그 보레이트를 유지한 채 멈추고, 못 찾으면 원래 설정으로 되돌린다.
static void scan_baud_rates() {
    printf("\n=== 보레이트 자동 스캔 시작 ===\n");
    for (uint32_t baud : BAUD_CANDIDATES) {
        printf("  [%u bps] 시도 중...\n", (unsigned)baud);
        uart_set_baudrate(SERVO_UART_NUM, baud);
        vTaskDelay(pdMS_TO_TICKS(20)); // 보레이트 전환 안정화 대기

        int found = scan_servo_ids_quiet();
        if (found > 0) {
            printf("=== 보레이트 %u bps에서 %d개 응답 발견! 이 보레이트로 계속 사용합니다. ===\n", (unsigned)baud, found);
            printf("    -> main.cpp의 SERVO_UART_BAUD를 %u로 고치면 다음 부팅부터 바로 적용됩니다.\n\n", (unsigned)baud);
            return;
        }
    }

    printf("=== 모든 후보 보레이트에서 응답 없음. 원래 설정(%u bps)으로 되돌립니다. ===\n", (unsigned)SERVO_UART_BAUD);
    printf("이 경우는 보레이트 문제가 아니라 배선/전원/ID 설정 쪽을 다시 보셔야 합니다.\n\n");
    uart_set_baudrate(SERVO_UART_NUM, SERVO_UART_BAUD);
}

static uint16_t clamp_pos(int32_t v) {
    if (v < POS_MIN) return POS_MIN;
    if (v > POS_MAX) return POS_MAX;
    return (uint16_t)v;
}

static void print_help() {
    printf("\n서보 증분 제어 (Enter 불필요, 키 누르는 즉시 반응)\n");
    printf("  q/a : 서보 11  +/-\n");
    printf("  w/s : 서보 12  +/-\n");
    printf("  e/d : 서보 13  +/-\n");
    printf("  r/f : 서보 21  +/-\n");
    printf("  t/g : 서보 22  +/-\n");
    printf("  y/h : 서보 23  +/-\n");
    printf("  m : 스텝 모드 토글 (정밀 %d tick / 빠름 %d tick)\n", FINE_STEP, COARSE_STEP);
    printf("  p : ID 스캔 (%d~%d 범위, 실제 응답하는 서보 ID 찾기)\n", SCAN_ID_MIN, SCAN_ID_MAX);
    printf("  b : 보레이트 자동 스캔 (여러 보레이트를 순회하며 서보가 응답하는 값 찾기)\n");
    printf("  space : 비상정지 (전체 토크 OFF)\n");
    printf("현재 모드: %s (1회 %d tick = %.3f deg)\n\n",
           current_mode_name(), current_step(), current_step() * 360.0f / 4096.0f);
}

// 스텝 모드 토글 (정밀 <-> 빠름)
static void toggle_step_mode() {
    s_coarse_mode = !s_coarse_mode;
    printf(">> 스텝 모드: %s (1회 %d tick = %.3f deg)\n",
           current_mode_name(), current_step(), current_step() * 360.0f / 4096.0f);
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
        int len = usb_serial_jtag_read_bytes(&ch, 1, pdMS_TO_TICKS(50));
        if (len <= 0) {
            continue;
        }

        if (ch == ' ') {
            emergency_stop();
            continue;
        }

        char key = (char)tolower(ch);

        if (key == 'm') {
            toggle_step_mode();
            continue;
        }

        if (key == 'p') {
            scan_servo_ids();
            continue;
        }

        if (key == 'b') {
            scan_baud_rates();
            continue;
        }

        for (const auto &km : KEY_MAP) {
            if (km.key == key) {
                int idx = km.servo_idx;
                int32_t delta = (int32_t)km.dir * current_step();
                uint16_t new_pos = clamp_pos((int32_t)s_cur_pos[idx] + delta);
                s_cur_pos[idx] = new_pos;
                servo_write_pos(SERVO_IDS[idx], new_pos, current_speed());
                printf("servo %d : pos=%u (%.3f deg) [%s]\n",
                       SERVO_IDS[idx], new_pos, new_pos * 360.0f / 4096.0f,
                       current_mode_name());
                break;
            }
        }
        // 매핑 안 된 키(엔터, 방향키 등)는 무시
    }
}

extern "C" void app_main(void) {
    servo_uart_init();
    console_input_init();

    // 부팅 시 절대 위치 명령은 보내지 않는다.
    // 각 서보의 실제 현재 위치를 READ로 확인해서 로컬 기준값으로만 저장하고,
    // 그 값에서부터 스텝 단위로만 움직이도록 해서 초기 스냅(급격한 위치 점프)을 방지한다.
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
