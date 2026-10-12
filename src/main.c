/**
 * @file main.c
 * @brief ESP32-C6 ESP-IDF pure C: Ping motor (ID = 1) over motor bus (Protocol
 * V3).
 *
 * Wiring (half-duplex, single wire, open-drain + pull-up):
 *   TX = RX = GPIO 10 (same pin)
 *   Baudrate = 1,000,000 bps (1 Mbps), 8N1
 *
 * Ping TX : FF FF 01 02 01 FB
 * Ping RX : FF FF 01 05 00 02 01 00 F6 (example from specification)
 */

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "hal/uart_ll.h"
#include "soc/uart_struct.h"
#include "uart_bridge.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "MOTOR_BUS";

// ---------------- Hardware Configuration ----------------
#define MOTOR_UART_NUM UART_NUM_1
#define MOTOR_TX_PIN GPIO_NUM_10
#define MOTOR_RX_PIN GPIO_NUM_10 // cùng chân với TX (half-duplex 1 dây)
#define MOTOR_BAUD_RATE 1000000
#define MOTOR_ID 1
#define PING_PERIOD_MS 1000

// 1 = Bridge: PC (USB Serial/JTAG) <-> motor bus (trong suốt, lọc echo)
// 0 = Ping test: gửi ping ID=MOTOR_ID mỗi giây và in phản hồi
#define BRIDGE_MODE 1

// 1 = TX push-pull khi đang truyền, truyền xong thì tắt output (high-Z) để motor trả lời
//     (mức cao khi rảnh do trở kéo lên nội/ngoài giữ)
// 0 = TX luôn open-drain (cần trở kéo lên, đã chạy ổn)
#define TX_PUSH_PULL_RELEASE 1

#define INST_PING 0x01
#define RX_BUF_SIZE 128
#define UART_RX_RING_SIZE 512

// ---------------- Packet helpers ----------------
static uint8_t calc_checksum(const uint8_t *data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return (uint8_t)(~sum);
}

// ---------------- Bus direction control (TX drive / release) ----------------
// Gọi trước khi ghi dữ liệu ra UART: bật output của chân TX (push-pull)
void motor_bus_tx_begin(void) {
  if (TX_PUSH_PULL_RELEASE) {
    gpio_ll_output_enable(&GPIO, MOTOR_TX_PIN);
  }
}

// Gọi sau khi ghi: chờ gửi xong byte cuối rồi nhả bus (tắt output)
void motor_bus_tx_end(void) {
  uart_wait_tx_done(MOTOR_UART_NUM, pdMS_TO_TICKS(100));
  if (TX_PUSH_PULL_RELEASE) {
    gpio_ll_output_disable(&GPIO, MOTOR_TX_PIN);
  }
}

// ---------------- Async Non-blocking TX Engine ----------------
#define MOTOR_TX_QUEUE_LEN 16
#define MOTOR_TX_MAX_PAYLOAD 256

typedef struct {
  uint8_t data[MOTOR_TX_MAX_PAYLOAD];
  size_t len;
} motor_tx_msg_t;

static QueueHandle_t s_motor_tx_queue = NULL;
static volatile bool s_tx_in_progress = false;

bool motor_bus_is_tx_idle(void) {
  return !s_tx_in_progress &&
         uart_ll_is_tx_idle(&UART1) &&
         (s_motor_tx_queue == NULL || uxQueueMessagesWaiting(s_motor_tx_queue) == 0);
}

esp_err_t motor_bus_tx_async(const uint8_t *data, size_t len) {
  if (data == NULL || len == 0 || len > MOTOR_TX_MAX_PAYLOAD) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_motor_tx_queue == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  motor_tx_msg_t msg;
  memcpy(msg.data, data, len);
  msg.len = len;

  if (xQueueSend(s_motor_tx_queue, &msg, 0) != pdTRUE) {
    ESP_LOGW(TAG, "motor_bus_tx_async: TX queue full, dropped packet");
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

static void motor_tx_worker_task(void *pvParameters) {
  motor_tx_msg_t msg;
  while (1) {
    if (xQueueReceive(s_motor_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
      s_tx_in_progress = true;
      motor_bus_tx_begin(); // Bật Push-Pull 3.3V
      uart_write_bytes(MOTOR_UART_NUM, (const char *)msg.data, msg.len);
      // Chờ ngắt phần cứng UART_INTR_TX_DONE (task ngủ, 0% CPU):
      esp_err_t err = uart_wait_tx_done(MOTOR_UART_NUM, pdMS_TO_TICKS(100));
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "TX timeout (100 ms), cuong buc nha bus");
      }
      motor_bus_tx_end();   // Nhả bus về High-Z (RX mode)
      s_tx_in_progress = false;
    }
  }
}

void motor_bus_async_init(void) {
  if (s_motor_tx_queue == NULL) {
    s_motor_tx_queue = xQueueCreate(MOTOR_TX_QUEUE_LEN, sizeof(motor_tx_msg_t));
    xTaskCreate(motor_tx_worker_task, "motor_tx_worker", 3072, NULL, 12, NULL);
    ESP_LOGI(TAG, "Async TX worker initialized (depth=%d, max_len=%d)",
             MOTOR_TX_QUEUE_LEN, MOTOR_TX_MAX_PAYLOAD);
  }
}

// Bus half-duplex (TX open-drain nối chung RX) => RX nhận lại echo của chính
// gói vừa gửi
static uint8_t last_tx_pkt[6];
static volatile bool expect_echo = false;

static void send_ping(uint8_t id) {
  // Length = N + 2 = 2 with N = 0 parameters
  uint8_t pkt[6] = {0xFF, 0xFF, id, 0x02, INST_PING, 0x00};
  pkt[5] = calc_checksum(&pkt[2], 3); // ID + Length + Instruction
  memcpy(last_tx_pkt, pkt, sizeof(pkt));
  expect_echo = true;
  motor_bus_tx_async(pkt, sizeof(pkt)); // Non-blocking! Return ngay lập tức
}

// ---------------- Status packet parser (state machine) ----------------
typedef enum {
  RX_STATE_HDR1,
  RX_STATE_HDR2,
  RX_STATE_ID,
  RX_STATE_LEN,
  RX_STATE_BODY
} rx_state_t;

static rx_state_t rx_state = RX_STATE_HDR1;
static uint8_t rx_buf[RX_BUF_SIZE];
static size_t rx_idx = 0;
static uint8_t rx_body_left = 0;
static int64_t ping_sent_us = 0;
static volatile bool waiting_reply = false;
static QueueHandle_t uart_queue = NULL;
static SemaphoreHandle_t reply_sem = NULL;

static void print_packet_hex(const uint8_t *p, size_t len) {
  char hex_str[3 * RX_BUF_SIZE + 16];
  int offset = snprintf(hex_str, sizeof(hex_str), "FF FF");
  for (size_t i = 0; i < len && offset < (int)(sizeof(hex_str) - 4); i++) {
    offset +=
        snprintf(hex_str + offset, sizeof(hex_str) - offset, " %02X", p[i]);
  }
  ESP_LOGI(TAG, "RAW Packet: %s", hex_str);
}

static void handle_packet(const uint8_t *p, size_t len) {
  // p[0]=ID, p[1]=LEN, p[2]=Status, ..., p[len-1]=Checksum
  // Bỏ qua echo của gói ping vừa gửi (ID LEN INST CS giống hệt gói TX)
  if (expect_echo && len == 4 && memcmp(p, &last_tx_pkt[2], 4) == 0) {
    expect_echo = false;
    return;
  }
  uint8_t id = p[0];
  uint8_t cs = calc_checksum(p, len - 1);
  if (cs != p[len - 1]) {
    ESP_LOGW(TAG, "[RX] Checksum error (calc=0x%02X, got=0x%02X)", cs,
             p[len - 1]);
    print_packet_hex(p, len);
    return;
  }

  uint8_t status = p[2];
  int64_t rtt_us = waiting_reply ? (esp_timer_get_time() - ping_sent_us) : 0;
  ESP_LOGI(TAG, "[RX] ID=%u | Status=0x%02X | RTT=%lld us", id, status, rtt_us);
  print_packet_hex(p, len);

  if (status != 0) {
    ESP_LOGW(TAG, "     Motor error flags: 0x%02X", status);
  }
  waiting_reply = false;
  xSemaphoreGive(reply_sem);
}

static void parse_rx_byte(uint8_t b) {
  switch (rx_state) {
  case RX_STATE_HDR1:
    if (b == 0xFF)
      rx_state = RX_STATE_HDR2;
    break;
  case RX_STATE_HDR2:
    rx_state = (b == 0xFF) ? RX_STATE_ID : RX_STATE_HDR1;
    break;
  case RX_STATE_ID:
    if (b == 0xFF)
      break; // Extra 0xFF preamble
    rx_buf[0] = b;
    rx_idx = 1;
    rx_state = RX_STATE_LEN;
    break;
  case RX_STATE_LEN:
    if (b < 2 || (b + 2) > RX_BUF_SIZE) {
      rx_state = RX_STATE_HDR1;
      break;
    }
    rx_buf[rx_idx++] = b;
    rx_body_left = b;
    rx_state = RX_STATE_BODY;
    break;
  case RX_STATE_BODY:
    rx_buf[rx_idx++] = b;
    if (--rx_body_left == 0) {
      handle_packet(rx_buf, rx_idx);
      rx_state = RX_STATE_HDR1;
    }
    break;
  }
}

// ---------------- UART Initialization (ESP-IDF) ----------------
static void init_motor_uart(void) {
  const uart_config_t uart_config = {
      .baud_rate = MOTOR_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
      .source_clk = UART_SCLK_DEFAULT,
  };

  // 1. Cài đặt driver với bộ đệm nhận (+ hàng đợi sự kiện khi chạy ping test)
  ESP_ERROR_CHECK(uart_driver_install(MOTOR_UART_NUM, UART_RX_RING_SIZE, 0,
                                      BRIDGE_MODE ? 0 : 20,
                                      BRIDGE_MODE ? NULL : &uart_queue, 0));

  // 2. Cấu hình các tham số truyền thông (baud, bit, parity)
  ESP_ERROR_CHECK(uart_param_config(MOTOR_UART_NUM, &uart_config));

  // 3. Khởi tạo và gán chân cẳng chi tiết: TX, RX, RTS, CTS
  ESP_ERROR_CHECK(uart_set_pin(MOTOR_UART_NUM, MOTOR_TX_PIN, MOTOR_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // 4. Chế độ chân TX
  if (TX_PUSH_PULL_RELEASE) {
    // Push-pull khi truyền; output do GPIO_ENABLE điều khiển để nhả được
    // (UART TX vẫn nối vào chân qua GPIO Matrix). Mặc định nhả bus.
    gpio_ll_od_disable(&GPIO, MOTOR_TX_PIN);
    gpio_ll_set_output_enable_ctrl(&GPIO, MOTOR_TX_PIN, false, false);
    gpio_ll_output_disable(&GPIO, MOTOR_TX_PIN);
  } else {
    // Open-Drain (cần điện trở kéo lên ngoài, ví dụ 1k-4.7k lên 3.3V)
    gpio_ll_od_enable(&GPIO, MOTOR_TX_PIN);
  }
  ESP_ERROR_CHECK(
      gpio_pullup_en(MOTOR_TX_PIN)); // trở kéo lên nội ~45k (giữ mức cao khi
                                     // bus rảnh, vẫn nên có trở ngoài 1k-2.2k)
  ESP_ERROR_CHECK(
      gpio_pullup_en(MOTOR_RX_PIN)); // RX luôn ở mức cao khi bus rảnh

  // 5. Ngắt RX: ping test báo ngay mỗi byte; bridge gom tối 96 byte (FIFO 128)
  // hoặc sau 2 ký tự rảnh
  ESP_ERROR_CHECK(
      uart_set_rx_full_threshold(MOTOR_UART_NUM, BRIDGE_MODE ? 96 : 1));
  ESP_ERROR_CHECK(uart_set_rx_timeout(MOTOR_UART_NUM, 2));

  ESP_LOGI(TAG, "RX GPIO%d idle level = %d (phai = 1)", MOTOR_RX_PIN,
           gpio_get_level(MOTOR_RX_PIN));

  ESP_LOGI(TAG, "UART%d initialized: TX=GPIO%d, RX=GPIO%d @ %d bps",
           MOTOR_UART_NUM, MOTOR_TX_PIN, MOTOR_RX_PIN, MOTOR_BAUD_RATE);
}

// ---------------- UART RX event task (interrupt driven) ----------------
static void uart_event_task(void *pvParameters) {
  uart_event_t event;
  uint8_t data[64];

  while (1) {
    if (xQueueReceive(uart_queue, &event, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    switch (event.type) {
    case UART_DATA: {
      size_t remain = event.size;
      while (remain > 0) {
        size_t want = remain < sizeof(data) ? remain : sizeof(data);
        int n = uart_read_bytes(MOTOR_UART_NUM, data, want, 0);
        if (n <= 0)
          break;
        for (int i = 0; i < n; i++) {
          parse_rx_byte(data[i]);
        }
        remain -= n;
      }
      break;
    }
    case UART_FRAME_ERR:
      ESP_LOGW(TAG, "[UART] FRAME ERROR (sai baud hoac nhieu duong truyen)");
      break;
    case UART_PARITY_ERR:
      ESP_LOGW(TAG, "[UART] PARITY ERROR");
      break;
    case UART_BREAK:
      ESP_LOGW(TAG, "[UART] BREAK detected (RX bi keo xuong thap)");
      break;
    case UART_FIFO_OVF:
    case UART_BUFFER_FULL:
      ESP_LOGW(TAG, "[UART] RX overflow, flush");
      uart_flush_input(MOTOR_UART_NUM);
      xQueueReset(uart_queue);
      break;
    default:
      ESP_LOGW(TAG, "[UART] event type=%d", event.type);
      break;
    }
  }
}

// ---------------- FreeRTOS Motor Task ----------------
static void motor_task(void *pvParameters) {
  TickType_t last_wake_time = xTaskGetTickCount();

  while (1) {
    // Xóa tín hiệu cũ, gửi gói tin Ping (việc nhận do uart_event_task xử lý qua
    // ngắt RX)
    xSemaphoreTake(reply_sem, 0);
    ping_sent_us = esp_timer_get_time();
    waiting_reply = true;
    send_ping(MOTOR_ID);

    // Chờ phản hồi tối đa 50 ms (semaphore được give khi parse xong gói hợp lệ)
    // Lưu ý: FreeRTOS tick = 100 Hz nên không dùng vòng vTaskDelay(5 ms) được
    if (xSemaphoreTake(reply_sem, pdMS_TO_TICKS(50)) != pdTRUE) {
      ESP_LOGW(TAG, "Ping ID=%d timeout (no reply received) | RX level=%d",
               MOTOR_ID, gpio_get_level(MOTOR_RX_PIN));
      waiting_reply = false;
    }

    // Chờ đến chu kỳ tiếp theo (1 giây)
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PING_PERIOD_MS));
  }
}

// ---------------- Application Entry Point ----------------
void app_main(void) {
  ESP_LOGI(TAG, "=== ESP32-C6 Motor Ping (ESP-IDF Pure C) ===");

  // Khởi tạo UART bus
  reply_sem = xSemaphoreCreateBinary();
  init_motor_uart();
  motor_bus_async_init();

  if (BRIDGE_MODE) {
    ESP_LOGI(TAG, "Mode: UART BRIDGE (USB Serial/JTAG <-> motor bus)");
    uart_bridge_start(MOTOR_UART_NUM);
    return;
  }

  // Task xử lý sự kiện ngắt RX (ưu tiên cao hơn motor_task)
  xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 10, NULL);

  // Tạo FreeRTOS task chu kỳ 1s để giao tiếp motor
  xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
}
