/**
 * @file uart_i2c_bridge.h
 * @brief UART to I2C bridge interface for RK3588 Linux host
 * 
 * This module provides a simple synchronous API to perform I2C read/write
 * operations via a STM32 bridge connected over UART (USART1 at 921600 baud).
 * 
 * Frame format (matches STM32 side):
 *   [START(0xAA)][ADDR][LEN_LO][LEN_HI][PAYLOAD...][END(0xEE)]
 * 
 * Response format (from STM32):
 *   [START(0xAA)][STATUS][LEN_LO][LEN_HI][PAYLOAD...][END(0xEE)]
 */

#ifndef UART_I2C_BRIDGE_H
#define UART_I2C_BRIDGE_H

#include <stdint.h>
#include <stddef.h>

/* Serial port configuration - modify these macros as needed */
#define BRIDGE_UART_DEVICE      "/dev/ttyS6"
#define BRIDGE_UART_BAUDRATE    B921600         /* Matches STM32 USART1 */
#define BRIDGE_UART_TIMEOUT_MS  1000            /* Read timeout in milliseconds */

/* Frame protocol constants */
#define BRIDGE_FRAME_START      0xAA
#define BRIDGE_FRAME_END        0xEE
#define BRIDGE_MAX_PAYLOAD      512             /* Max I2C payload size */

/* Status codes returned by STM32 (matches BridgeStatus_t enum) */
#define BRIDGE_STATUS_OK            0
#define BRIDGE_STATUS_LEN_ERROR     2
#define BRIDGE_STATUS_BUSY          3
#define BRIDGE_STATUS_I2C_TX_FAIL   4
#define BRIDGE_STATUS_I2C_RX_FAIL   5
#define BRIDGE_STATUS_I2C_ERROR     6

/* Return codes for API functions */
#define BRIDGE_RET_OK               0
#define BRIDGE_RET_INVALID_PARAM    -1
#define BRIDGE_RET_UART_OPEN_FAIL   -2
#define BRIDGE_RET_UART_WRITE_FAIL  -3
#define BRIDGE_RET_UART_READ_FAIL   -4
#define BRIDGE_RET_TIMEOUT          -5
#define BRIDGE_RET_FRAME_ERROR      -6
#define BRIDGE_RET_STM32_ERROR      -7

/**
 * @brief Initialize the UART bridge (open serial port and configure)
 * @return 0 on success, negative error code on failure
 */
int uart_i2c_bridge_init(void);

/**
 * @brief Close the UART bridge and release resources
 */
void uart_i2c_bridge_close(void);

/**
 * @brief Write data to an I2C device via the UART bridge
 * @param i2c_addr 7-bit I2C device address (e.g., 0x50)
 * @param data Pointer to data buffer to write
 * @param len Number of bytes to write (must be <= BRIDGE_MAX_PAYLOAD)
 * @return 0 on success, negative error code on failure
 * 
 * This function blocks until the STM32 responds or timeout occurs.
 */
int uart_i2c_write(uint8_t i2c_addr, const uint8_t *data, uint16_t len);

/**
 * @brief Read data from an I2C device via the UART bridge
 * @param i2c_addr 7-bit I2C device address (e.g., 0x50)
 * @param data Pointer to buffer to store read data (must be >= len bytes)
 * @param len Number of bytes to read (must be <= BRIDGE_MAX_PAYLOAD)
 * @return Number of bytes actually read on success (should equal len),
 *         negative error code on failure
 * 
 * This function blocks until the STM32 responds or timeout occurs.
 */
int uart_i2c_read(uint8_t i2c_addr, uint8_t *data, uint16_t len);

/**
 * @brief Trigger a device reset via special frame (addr=0, len=0, no payload).
 *        STM32 will pulse SEC_CHIP_RESET low for 50ms then high and reply OK.
 * @return 0 on success, negative error code on failure
 */
int uart_i2c_reset(void);

#endif /* UART_I2C_BRIDGE_H */
