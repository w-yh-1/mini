#ifndef UV_API_H
#define UV_API_H

#include <stdint.h>

// 串口默认设置（可在编译时覆盖）
#ifndef SERIAL_PORT
#define SERIAL_PORT "/dev/ttyS8"
#endif

#ifndef BAUDRATE
#include <termios.h>
#define BAUDRATE B115200
#endif

// 协议常量（公共，便于上层使用/记录）
#define COMMAND_HEAD_1 0xD5
#define COMMAND_HEAD_2 0x5D
#define COMMAND_TAIL_1 0x5D
#define COMMAND_TAIL_2 0xD5
#define CHECKSUM 0xED

// 统一返回码（公共 API）
#define UV_OK 0
#define UV_ERR_ARG -10      // 无效参数
#define UV_ERR_RANGE -11    // 参数越界
#define UV_ERR_BUILD -12    // 帧构建失败
#define UV_ERR_SEND -13     // 发送失败
#define UV_ERR_IO -14       // IO/系统调用错误
#define UV_ERR_TIMEOUT -15  // 超时
#define UV_ERR_FRAME -16    // 帧格式错误
#define UV_ERR_OVERFLOW -17 // 缓冲区不足或溢出
#define UV_ERR_PARSE -18    // 内容解析不匹配

#ifdef __cplusplus
extern "C"
{
#endif

    // 串口打开/配置
    int open_serial_port(const char *device);
    int set_serial_port(int fd);

    // 发送/接收（阻塞直到超时或拿到完整帧）
    int send_serial_command(int fd, uint8_t *command, size_t length);
    int receive_serial_response(int fd, uint8_t *response, size_t length);

    // 设备指令（同步）
    int set_high_voltage_gain(int fd, uint16_t gain);
    int read_high_voltage_gain(int fd, uint16_t *gain);
    int set_counting_frequency(int fd, uint16_t frequency);
    int read_photon_count(int fd, uint32_t *count);
    int enable_region_photon_counting(int fd, uint8_t enable);
    int set_region_x_start(int fd, uint16_t x);
    int set_region_width(int fd, uint16_t width);
    int set_region_y_start(int fd, uint16_t y);
    int set_region_height(int fd, uint16_t height);
    int set_photons_color(int fd, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif // UV_API_H