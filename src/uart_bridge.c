/**
 * @file uart_bridge.c
 * @brief Transparent bridge: USB Serial/JTAG (PC) <-> Motor bus UART.
 *
 * The motor bus is half-duplex (open-drain TX shorted to RX), so every byte we
 * transmit is also received on RX (echo). To give the PC only the motor's reply,
 * the transmitted bytes are remembered in a small FIFO and matching bytes coming
 * back on RX are dropped. If the echo does not match (or never arrives, e.g. on a
 * full-duplex wiring), the FIFO is flushed so real data is never swallowed.
 */

#include "uart_bridge.h"

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "BRIDGE";

#define BRIDGE_CHUNK        256
#define ECHO_RING_SIZE      512     // power of two
#define ECHO_MARGIN_US      1000    // extra time allowed for echo after last TX byte
#define USB_BUF_SIZE        1024
#define STATS_PERIOD_US     5000000

static uart_port_t s_motor_uart;

// ---- Echo FIFO (single producer: usb->motor task, single consumer: motor->usb task) ----
static uint8_t           s_echo_ring[ECHO_RING_SIZE];
static volatile uint32_t s_echo_head = 0;           // written by producer
static volatile uint32_t s_echo_tail = 0;           // written by consumer
static volatile int64_t  s_echo_deadline_us = 0;    // after this, remaining entries are stale

// ---- Statistics ----
static volatile uint32_t s_cnt_pc_to_motor = 0;
static volatile uint32_t s_cnt_motor_to_pc = 0;
static volatile uint32_t s_cnt_echo_dropped = 0;

static void echo_push(const uint8_t *data, size_t len) {
    uint32_t head = s_echo_head;
    uint32_t tail = __atomic_load_n(&s_echo_tail, __ATOMIC_ACQUIRE);
    for (size_t i = 0; i < len; i++) {
        if ((head - tail) >= ECHO_RING_SIZE) break; // full: stop remembering
        s_echo_ring[head % ECHO_RING_SIZE] = data[i];
        head++;
    }
    __atomic_store_n(&s_echo_head, head, __ATOMIC_RELEASE);

    // Echo should be complete about (len * 10 us @ 1 Mbps) + margin after now
    int64_t now = esp_timer_get_time();
    int64_t base = (s_echo_deadline_us > now) ? s_echo_deadline_us : now;
    s_echo_deadline_us = base + (int64_t)len * 12 + ECHO_MARGIN_US;
}

/** @return true if byte @p b is the echo of our own TX and must be dropped. */
static bool echo_filter(uint8_t b) {
    uint32_t tail = s_echo_tail;
    uint32_t head = __atomic_load_n(&s_echo_head, __ATOMIC_ACQUIRE);
    if (tail == head) {
        return false;
    }
    if (esp_timer_get_time() > s_echo_deadline_us) {
        // Stale entries (echo never came back) -> forget them
        __atomic_store_n(&s_echo_tail, head, __ATOMIC_RELEASE);
        return false;
    }
    if (s_echo_ring[tail % ECHO_RING_SIZE] == b) {
        __atomic_store_n(&s_echo_tail, tail + 1, __ATOMIC_RELEASE);
        return true;
    }
    // Mismatch -> no echo on this wiring, do not swallow real data
    __atomic_store_n(&s_echo_tail, head, __ATOMIC_RELEASE);
    return false;
}

// ---------------- PC -> Motor ----------------
static void pc_to_motor_task(void *arg) {
    uint8_t buf[BRIDGE_CHUNK];
    while (1) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n > 0) {
            echo_push(buf, (size_t)n);                       // remember BEFORE sending
            motor_bus_tx_async(buf, (size_t)n);              // non-blocking async TX via interrupt
            s_cnt_pc_to_motor += (uint32_t)n;
        }
    }
}

// ---------------- Motor -> PC ----------------
static void motor_to_pc_task(void *arg) {
    uint8_t in[BRIDGE_CHUNK];
    uint8_t out[BRIDGE_CHUNK];
    int64_t last_stats_us = esp_timer_get_time();
    uint32_t last_pc2m = 0, last_m2pc = 0, last_echo = 0;

    while (1) {
        // Block until at least 1 byte, then drain whatever is buffered (low latency)
        int n = uart_read_bytes(s_motor_uart, in, 1, pdMS_TO_TICKS(100));
        if (n == 1) {
            size_t avail = 0;
            uart_get_buffered_data_len(s_motor_uart, &avail);
            if (avail > sizeof(in) - 1) avail = sizeof(in) - 1;
            if (avail > 0) {
                int m = uart_read_bytes(s_motor_uart, in + 1, avail, 0);
                if (m > 0) n += m;
            }

            int out_n = 0;
            for (int i = 0; i < n; i++) {
                if (echo_filter(in[i])) {
                    s_cnt_echo_dropped++;
                } else {
                    out[out_n++] = in[i];
                }
            }
            if (out_n > 0) {
                usb_serial_jtag_write_bytes(out, (size_t)out_n, pdMS_TO_TICKS(100));
                s_cnt_motor_to_pc += (uint32_t)out_n;
            }
        }

        int64_t now = esp_timer_get_time();
        if (now - last_stats_us >= STATS_PERIOD_US) {
            last_stats_us = now;
            if (s_cnt_pc_to_motor != last_pc2m || s_cnt_motor_to_pc != last_m2pc ||
                s_cnt_echo_dropped != last_echo) {
                last_pc2m = s_cnt_pc_to_motor;
                last_m2pc = s_cnt_motor_to_pc;
                last_echo = s_cnt_echo_dropped;
                ESP_LOGI(TAG, "PC->Motor=%u B | Motor->PC=%u B | echo dropped=%u B",
                         (unsigned)last_pc2m, (unsigned)last_m2pc, (unsigned)last_echo);
            }
        }
    }
}

void uart_bridge_start(uart_port_t motor_uart) {
    s_motor_uart = motor_uart;

    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = USB_BUF_SIZE;
    cfg.tx_buffer_size = USB_BUF_SIZE;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));

    xTaskCreate(motor_to_pc_task, "br_motor2pc", 4096, NULL, 10, NULL);
    xTaskCreate(pc_to_motor_task, "br_pc2motor", 4096, NULL, 10, NULL);

    ESP_LOGI(TAG, "UART bridge started: USB Serial/JTAG <-> motor UART%d", (int)motor_uart);
}
