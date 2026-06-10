/**
 * @file uart_i2c_bridge.c
 * @brief UART to I2C bridge implementation for RK3588 Linux host
 */

#include "uart_i2c_bridge.h"

#ifndef __linux__
/*
 * This source is intended to be built on the RK3588 (Linux). To keep static
 * analyzers on non-Linux hosts quiet, provide no-op stubs here.
 */
int uart_i2c_bridge_init(void) { return BRIDGE_RET_UART_OPEN_FAIL; }
void uart_i2c_bridge_close(void) {}
int uart_i2c_write(uint8_t i2c_addr, const uint8_t *data, uint16_t len) { (void)i2c_addr; (void)data; (void)len; return BRIDGE_RET_UART_OPEN_FAIL; }
int uart_i2c_read(uint8_t i2c_addr, uint8_t *data, uint16_t len) { (void)i2c_addr; (void)data; (void)len; return BRIDGE_RET_UART_OPEN_FAIL; }
int uart_i2c_reset(void) { return BRIDGE_RET_UART_OPEN_FAIL; }

#else /* __linux__ */

/* Ensure POSIX prototypes (e.g., tcdrain) are declared */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/time.h>

/* Internal state */
static int uart_fd = -1;

/* Internal helpers */
static int uart_write_frame(const uint8_t *frame, size_t len);
static int uart_read_response(uint8_t *frame_buf, size_t max_len, uint16_t *payload_len, uint8_t *status);
static void print_error(const char *func, const char *msg);

/**
 * @brief Initialize UART port with settings matching STM32 USART1
 */
int uart_i2c_bridge_init(void)
{
    struct termios tty;
    
    if (uart_fd >= 0) {
        print_error(__func__, "Bridge already initialized");
        return BRIDGE_RET_OK; /* Already open */
    }
    
    /* Open serial port */
    uart_fd = open(BRIDGE_UART_DEVICE, O_RDWR | O_NOCTTY | O_SYNC);
    if (uart_fd < 0) {
        print_error(__func__, "Failed to open UART device");
        perror(BRIDGE_UART_DEVICE);
        return BRIDGE_RET_UART_OPEN_FAIL;
    }
    
    /* Get current settings */
    if (tcgetattr(uart_fd, &tty) != 0) {
        print_error(__func__, "tcgetattr failed");
        perror("tcgetattr");
        close(uart_fd);
        uart_fd = -1;
        return BRIDGE_RET_UART_OPEN_FAIL;
    }
    
    /* Configure: 921600 baud, 8N1, raw mode, no flow control */
    cfsetospeed(&tty, BRIDGE_UART_BAUDRATE);
    cfsetispeed(&tty, BRIDGE_UART_BAUDRATE);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     /* 8-bit chars */
    /* input flags: raw input, disable CR/LF translations and XON/XOFF */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tty.c_lflag = 0;                                 /* no signaling chars, no echo, no canonical processing */
    tty.c_oflag = 0;                                 /* no remapping, no delays */
    tty.c_cc[VMIN]  = 0;                             /* read doesn't block */
    tty.c_cc[VTIME] = 5;                             /* 0.5 seconds read timeout */
    
    tty.c_cflag |= (CLOCAL | CREAD);                 /* ignore modem controls, enable reading */
    tty.c_cflag &= ~(PARENB | PARODD);               /* shut off parity */
    tty.c_cflag &= ~CSTOPB;                          /* 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;                         /* no hardware flow control */
    
    /* Apply settings */
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        print_error(__func__, "tcsetattr failed");
        perror("tcsetattr");
        close(uart_fd);
        uart_fd = -1;
        return BRIDGE_RET_UART_OPEN_FAIL;
    }
    
    /* Flush any stale data */
    tcflush(uart_fd, TCIOFLUSH);
    
    return BRIDGE_RET_OK;
}

/**
 * @brief Close UART port
 */
void uart_i2c_bridge_close(void)
{
    if (uart_fd >= 0) {
        close(uart_fd);
        uart_fd = -1;
    }
}

/**
 * @brief Write data to I2C device via STM32 bridge
 */
int uart_i2c_write(uint8_t i2c_addr, const uint8_t *data, uint16_t len)
{
    uint8_t frame[BRIDGE_MAX_PAYLOAD + 16];
    uint8_t resp_status;
    uint16_t resp_len;
    int ret;
    
    /* Validate parameters */
    if (!data || len == 0 || len > BRIDGE_MAX_PAYLOAD) {
        print_error(__func__, "Invalid parameters");
        return BRIDGE_RET_INVALID_PARAM;
    }
    if (i2c_addr > 0x7F) {
        print_error(__func__, "I2C address must be 7-bit (0x00..0x7F)");
        return BRIDGE_RET_INVALID_PARAM;
    }
    
    if (uart_fd < 0) {
        print_error(__func__, "Bridge not initialized");
        return BRIDGE_RET_UART_OPEN_FAIL;
    }
    
    /* Build write frame: [START][ADDR(W)][LEN_LO][LEN_HI][PAYLOAD][END] */
    frame[0] = BRIDGE_FRAME_START;
    frame[1] = (i2c_addr << 1) | 0x00;  /* Write: R/W bit = 0 */
    frame[2] = (uint8_t)(len & 0xFF);
    frame[3] = (uint8_t)((len >> 8) & 0xFF);
    memcpy(&frame[4], data, len);
    frame[4 + len] = BRIDGE_FRAME_END;
    
    size_t frame_len = 4 + len + 1;
    
    /* Send frame */
    ret = uart_write_frame(frame, frame_len);
    if (ret < 0) {
        return ret;
    }
    
    /* Wait for response */
    ret = uart_read_response(frame, sizeof(frame), &resp_len, &resp_status);
    if (ret < 0) {
        return ret;
    }
    
    /* Check status */
    if (resp_status != BRIDGE_STATUS_OK) {
        fprintf(stderr, "[%s] STM32 returned error status=%u", __func__, resp_status);
        if (resp_len > 0 && resp_len < sizeof(frame)) {
            fprintf(stderr, ", payload: %.*s", (int)resp_len, &frame[4]);
        }
        fprintf(stderr, "\n");
        return BRIDGE_RET_STM32_ERROR;
    }
    
    return BRIDGE_RET_OK;
}

/**
 * @brief Read data from I2C device via STM32 bridge
 */
int uart_i2c_read(uint8_t i2c_addr, uint8_t *data, uint16_t len)
{
    uint8_t frame[BRIDGE_MAX_PAYLOAD + 16];
    uint8_t resp_status;
    uint16_t resp_len;
    int ret;
    
    /* Validate parameters */
    if (!data || len == 0 || len > BRIDGE_MAX_PAYLOAD) {
        print_error(__func__, "Invalid parameters");
        return BRIDGE_RET_INVALID_PARAM;
    }
    if (i2c_addr > 0x7F) {
        print_error(__func__, "I2C address must be 7-bit (0x00..0x7F)");
        return BRIDGE_RET_INVALID_PARAM;
    }
    
    if (uart_fd < 0) {
        print_error(__func__, "Bridge not initialized");
        return BRIDGE_RET_UART_OPEN_FAIL;
    }
    
    /* Build read frame: [START][ADDR(R)][LEN_LO][LEN_HI][END] */
    /* Note: for read, no payload in request; len indicates how many bytes to read */
    frame[0] = BRIDGE_FRAME_START;
    frame[1] = (i2c_addr << 1) | 0x01;  /* Read: R/W bit = 1 */
    frame[2] = (uint8_t)(len & 0xFF);
    frame[3] = (uint8_t)((len >> 8) & 0xFF);
    frame[4] = BRIDGE_FRAME_END;
    
    size_t frame_len = 5;
    
    /* Send frame */
    ret = uart_write_frame(frame, frame_len);
    if (ret < 0) {
        return ret;
    }
    
    /* Wait for response */
    ret = uart_read_response(frame, sizeof(frame), &resp_len, &resp_status);
    if (ret < 0) {
        return ret;
    }
    
    /* Check status */
    if (resp_status != BRIDGE_STATUS_OK) {
        fprintf(stderr, "[%s] STM32 returned error status=%u", __func__, resp_status);
        if (resp_len > 0 && resp_len < sizeof(frame)) {
            fprintf(stderr, ", payload: %.*s", (int)resp_len, &frame[4]);
        }
        fprintf(stderr, "\n");
        return BRIDGE_RET_STM32_ERROR;
    }
    
    /* Copy payload to user buffer */
    if (resp_len != len) {
        fprintf(stderr, "[%s] Warning: expected %u bytes, got %u bytes\n", 
                __func__, len, resp_len);
    }
    
    size_t copy_len = (resp_len < len) ? resp_len : len;
    memcpy(data, &frame[4], copy_len);
    
    return (int)copy_len;
}

/**
 * @brief Send special reset command (addr=0, len=0, no payload)
 */
int uart_i2c_reset(void)
{
    uint8_t frame[8];
    uint8_t resp_status = 0;
    uint16_t resp_len = 0;
    int ret;

    if (uart_fd < 0) {
        print_error(__func__, "Bridge not initialized");
        return BRIDGE_RET_UART_OPEN_FAIL;
    }

    /* Frame: [START][ADDR(0x00)][LEN_LO=0][LEN_HI=0][END] -> AA 00 00 00 EE */
    frame[0] = BRIDGE_FRAME_START;
    frame[1] = 0x00; /* addr=0, rw bit ignored */
    frame[2] = 0x00;
    frame[3] = 0x00;
    frame[4] = BRIDGE_FRAME_END;

    ret = uart_write_frame(frame, 5);
    if (ret < 0) return ret;

    ret = uart_read_response(frame, sizeof(frame), &resp_len, &resp_status);
    if (ret < 0) return ret;

    if (resp_status != BRIDGE_STATUS_OK) {
        fprintf(stderr, "[%s] STM32 returned error status=%u", __func__, resp_status);
        if (resp_len > 0 && resp_len < sizeof(frame)) {
            fprintf(stderr, ", payload: %.*s", (int)resp_len, &frame[4]);
        }
        fprintf(stderr, "\n");
        return BRIDGE_RET_STM32_ERROR;
    }

    return BRIDGE_RET_OK;
}

/**
 * @brief Write a complete frame to UART
 */
static int uart_write_frame(const uint8_t *frame, size_t len)
{
    ssize_t written = write(uart_fd, frame, len);
    if (written < 0) {
        print_error(__func__, "UART write failed");
        perror("write");
        return BRIDGE_RET_UART_WRITE_FAIL;
    }
    
    if ((size_t)written != len) {
        fprintf(stderr, "[%s] Partial write: %zd/%zu bytes\n", __func__, written, len);
        return BRIDGE_RET_UART_WRITE_FAIL;
    }
    
    /* Ensure data is sent before reading response */
    tcdrain(uart_fd);
    
    return BRIDGE_RET_OK;
}

/**
 * @brief Read and parse response frame from UART
 * @param frame_buf Buffer to store frame (including header/footer)
 * @param max_len Size of frame_buf
 * @param payload_len Output: length of payload received
 * @param status Output: status code from STM32
 * @return 0 on success, negative error code on failure
 */
static int uart_read_response(uint8_t *frame_buf, size_t max_len, 
                               uint16_t *payload_len, uint8_t *status)
{
    fd_set readfds;
    struct timeval timeout;
    uint8_t byte;
    size_t idx = 0;
    int ret;
    
    /* State machine for frame parsing */
    enum { WAIT_START, READ_STATUS, READ_LEN_LO, READ_LEN_HI, READ_PAYLOAD, READ_END } state = WAIT_START;
    uint16_t expected_payload_len = 0;
    uint16_t payload_idx = 0;
    
    timeout.tv_sec = BRIDGE_UART_TIMEOUT_MS / 1000;
    timeout.tv_usec = (BRIDGE_UART_TIMEOUT_MS % 1000) * 1000;
    
    while (1) {
        FD_ZERO(&readfds);
        FD_SET(uart_fd, &readfds);
        
        struct timeval tv = timeout; /* select modifies timeout, so reset each iteration */
        ret = select(uart_fd + 1, &readfds, NULL, NULL, &tv);
        
        if (ret < 0) {
            print_error(__func__, "select failed");
            perror("select");
            return BRIDGE_RET_UART_READ_FAIL;
        }
        
        if (ret == 0) {
            print_error(__func__, "Read timeout waiting for STM32 response");
            return BRIDGE_RET_TIMEOUT;
        }
        
        /* Read one byte */
        ssize_t n = read(uart_fd, &byte, 1);
        if (n <= 0) {
            if (n < 0) {
                print_error(__func__, "UART read failed");
                perror("read");
            }
            return BRIDGE_RET_UART_READ_FAIL;
        }
        
        /* Store byte */
        if (idx < max_len) {
            frame_buf[idx++] = byte;
        } else {
            print_error(__func__, "Frame buffer overflow");
            return BRIDGE_RET_FRAME_ERROR;
        }
        
        /* State machine */
        switch (state) {
            case WAIT_START:
                if (byte == BRIDGE_FRAME_START) {
                    state = READ_STATUS;
                } else {
                    /* Discard spurious byte and keep waiting */
                    idx = 0;
                }
                break;
                
            case READ_STATUS:
                *status = byte;
                state = READ_LEN_LO;
                break;
                
            case READ_LEN_LO:
                expected_payload_len = byte;
                state = READ_LEN_HI;
                break;
                
            case READ_LEN_HI:
                expected_payload_len |= ((uint16_t)byte << 8);
                *payload_len = expected_payload_len;
                
                if (expected_payload_len == 0) {
                    state = READ_END;
                } else if (expected_payload_len > BRIDGE_MAX_PAYLOAD) {
                    fprintf(stderr, "[%s] Invalid payload length: %u\n", 
                            __func__, expected_payload_len);
                    return BRIDGE_RET_FRAME_ERROR;
                } else {
                    state = READ_PAYLOAD;
                    payload_idx = 0;
                }
                break;
                
            case READ_PAYLOAD:
                payload_idx++;
                if (payload_idx >= expected_payload_len) {
                    state = READ_END;
                }
                break;
                
            case READ_END:
                if (byte == BRIDGE_FRAME_END) {
                    /* Frame complete */
                    return BRIDGE_RET_OK;
                } else {
                    fprintf(stderr, "[%s] Invalid frame end: expected 0x%02X, got 0x%02X\n",
                            __func__, BRIDGE_FRAME_END, byte);
                    return BRIDGE_RET_FRAME_ERROR;
                }
                break;
        }
    }
    
    return BRIDGE_RET_FRAME_ERROR; /* Should never reach here */
}

/**
 * @brief Print error message with function name
 */
static void print_error(const char *func, const char *msg)
{
    fprintf(stderr, "[%s] Error: %s\n", func, msg);
}

#endif /* __linux__ */
