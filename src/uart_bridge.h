/**
 * @file uart_bridge.h
 * @brief Transparent bridge: USB Serial/JTAG (PC)  <->  Motor bus UART.
 */
#pragma once

#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the bridge tasks.
 *
 * Requirements:
 *  - The motor UART must already be installed/configured (driver + pins),
 *    without an event queue.
 *  - The console must NOT use USB Serial/JTAG (CONFIG_ESP_CONSOLE_SECONDARY_NONE=y),
 *    otherwise log text would be mixed into the bridged data.
 *
 * Data flow:
 *   PC -> USB Serial/JTAG -> motor UART TX   (echo of these bytes is remembered)
 *   motor UART RX -> (drop echo of own TX) -> USB Serial/JTAG -> PC
 */
void uart_bridge_start(uart_port_t motor_uart);

/**
 * @brief Bus direction hooks, implemented in main.c.
 *
 * motor_bus_tx_begin(): drive the TX pin (call before writing to the UART).
 * motor_bus_tx_end():   wait until the last byte is sent, then release the bus
 *                       so the motor can answer.
 */
void motor_bus_tx_begin(void);
void motor_bus_tx_end(void);

/**
 * @brief Initialize the asynchronous TX worker task and queue.
 */
void motor_bus_async_init(void);

/**
 * @brief Check if the motor bus UART transmitter is completely idle.
 * @return true if hardware TX FIFO is empty and no transmission is in progress.
 */
bool motor_bus_is_tx_idle(void);

/**
 * @brief Asynchronously transmit data to the motor bus (non-blocking).
 *
 * Pushes data into a background queue and returns immediately.
 * The dedicated TX worker enables Push-Pull mode, streams to UART FIFO,
 * waits for the hardware UART_INTR_TX_DONE interrupt, and immediately
 * releases the bus to Input High-Z (RX mode).
 *
 * @param data Pointer to buffer
 * @param len  Length in bytes (max 256)
 * @return ESP_OK if queued, ESP_ERR_NO_MEM if queue full
 */
esp_err_t motor_bus_tx_async(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
