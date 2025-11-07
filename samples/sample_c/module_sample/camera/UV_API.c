#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>

#include "UV_API.h"

// 内部帧辅助宏（不对外暴露）
#define FRAME_HEAD_0 COMMAND_HEAD_1
#define FRAME_HEAD_1 COMMAND_HEAD_2
#define FRAME_TAIL_0 COMMAND_TAIL_1
#define FRAME_TAIL_1 COMMAND_TAIL_2

// 最小帧长度: 2(head)+1(nn)+4(mm mm kk checksum)+2(tails)=9 字节
#define MIN_FRAME_LEN 9

// 可被上层缩短的接收超时（异步层会用更小的值覆盖）
#ifndef SERIAL_RX_CHUNK_TIMEOUT_MS
#define SERIAL_RX_CHUNK_TIMEOUT_MS 200
#endif
#ifndef SERIAL_RX_TOTAL_TIMEOUT_MS
#define SERIAL_RX_TOTAL_TIMEOUT_MS 2000
#endif

// 内部工具：基础格式校验
static int uv_validate_frame_basic(const uint8_t *buf, size_t len)
{
    if (!buf || len < MIN_FRAME_LEN)
        return -1;
    if (buf[0] != FRAME_HEAD_0 || buf[1] != FRAME_HEAD_1)
        return -2;
    uint8_t nn = buf[2];
    if ((size_t)nn != (len - 5))
        return -3; // nn 应等于总长 - 5
    size_t checksum_index = 2 + nn;
    if (buf[checksum_index] != CHECKSUM)
        return -4;
    if (buf[len - 2] != FRAME_TAIL_0 || buf[len - 1] != FRAME_TAIL_1)
        return -5;
    return 0;
}

// 内部工具：解析帧
static int uv_parse_frame(const uint8_t *buf, size_t len,
                          uint8_t *out_mm1, uint8_t *out_mm2, uint8_t *out_kk,
                          const uint8_t **out_params, size_t *out_params_len)
{
    int rc = uv_validate_frame_basic(buf, len);
    if (rc != 0)
        return rc;
    uint8_t nn = buf[2];
    if (out_mm1)
        *out_mm1 = buf[3];
    if (out_mm2)
        *out_mm2 = buf[4];
    if (out_kk)
        *out_kk = buf[5];
    size_t params_len = (nn >= 4) ? (nn - 4) : 0;
    if (out_params)
        *out_params = (params_len > 0) ? &buf[6] : NULL;
    if (out_params_len)
        *out_params_len = params_len;
    return 0;
}

// 内部工具：构建帧
static int uv_build_frame(uint8_t *out, size_t out_sz,
                          uint8_t mm1, uint8_t mm2, uint8_t kk,
                          const uint8_t *params, size_t params_len)
{
    if (!out)
        return -1;
    size_t nn = 2 + 1 + params_len + 1; // 2(mm)+1(kk)+params+1(checksum)
    size_t total = 2 + 1 + nn + 2;      // head + nn + payload + tails
    if (out_sz < total)
        return -1;
    out[0] = FRAME_HEAD_0;
    out[1] = FRAME_HEAD_1;
    out[2] = (uint8_t)nn;
    out[3] = mm1;
    out[4] = mm2;
    out[5] = kk;
    for (size_t i = 0; i < params_len; ++i)
        out[6 + i] = params ? params[i] : 0;
    out[2 + nn] = CHECKSUM;
    out[2 + nn + 1] = FRAME_TAIL_0;
    out[2 + nn + 2] = FRAME_TAIL_1;
    return (int)total;
}

// 内部工具：从累积缓冲中提取第一帧
static int uv_extract_first_frame(uint8_t *acc, size_t *acc_len,
                                  uint8_t *out, size_t out_sz)
{
    if (!acc || !acc_len)
        return -1;
    size_t n = *acc_len;
    if (n < 3)
        return 0;

    size_t i = 0;
    while (i + 1 < n)
    {
        if (acc[i] == FRAME_HEAD_0 && acc[i + 1] == FRAME_HEAD_1)
            break;
        ++i;
    }
    if (i > 0)
    {
        memmove(acc, acc + i, n - i);
        n -= i;
    }
    if (n < 3)
    {
        *acc_len = n;
        return 0;
    }

    uint8_t nn = acc[2];
    size_t total = 2 + 1 + nn + 2;
    if (n < total)
    {
        *acc_len = n;
        return 0;
    }
    if (out && out_sz < total)
        return -3;

    int v = uv_validate_frame_basic(acc, total);
    if (v != 0)
    {
        memmove(acc, acc + 1, n - 1);
        *acc_len = n - 1;
        return 0;
    }

    if (out)
        memcpy(out, acc, total);
    memmove(acc, acc + total, n - total);
    *acc_len = n - total;
    return (int)total;
}

// ------- 公共 API 实现 -------
int open_serial_port(const char *device)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("Error opening serial port");
        return -1;
    }
    return fd;
}

int set_serial_port(int fd)
{
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("Error setting serial port");
        return -1;
    }
    return 0;
}

int send_serial_command(int fd, uint8_t *command, size_t length)
{
    int rc = uv_validate_frame_basic(command, length);
    if (rc != 0)
    {
        fprintf(stderr, "Invalid command frame, rc=%d\n", rc);
        return -2;
    }
    int ret = (int)write(fd, command, length);
    if (ret < 0)
    {
        perror("Error sending serial command");
        return -1;
    }
    return ret;
}

int receive_serial_response(int fd, uint8_t *response, size_t length)
{
    if (!response || length < MIN_FRAME_LEN)
        return -3;

    uint8_t acc[1024];
    size_t acc_len = 0;
    uint8_t tmp[256];

    struct timeval t_start, t_now;
    gettimeofday(&t_start, NULL);

    for (;;)
    {
        int fr = uv_extract_first_frame(acc, &acc_len, response, length);
        if (fr > 0)
            return fr;
        if (fr < 0)
            return fr;

        gettimeofday(&t_now, NULL);
        long elapsed_ms = (long)((t_now.tv_sec - t_start.tv_sec) * 1000L +
                                 (t_now.tv_usec - t_start.tv_usec) / 1000L);
        if (elapsed_ms >= SERIAL_RX_TOTAL_TIMEOUT_MS)
            return -4;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv;
        tv.tv_sec = SERIAL_RX_CHUNK_TIMEOUT_MS / 1000;
        tv.tv_usec = (SERIAL_RX_CHUNK_TIMEOUT_MS % 1000) * 1000;
        int sel = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0)
        {
            perror("select() failed");
            return -1;
        }
        else if (sel == 0)
        {
            continue;
        }

        ssize_t r = read(fd, tmp, sizeof(tmp));
        if (r < 0)
        {
            perror("Error receiving serial response");
            return -1;
        }
        if (r == 0)
        {
            continue;
        }

        if (acc_len + (size_t)r > sizeof(acc))
        {
            size_t need = acc_len + (size_t)r - sizeof(acc);
            if (need >= acc_len)
            {
                acc_len = 0;
            }
            else
            {
                memmove(acc, acc + need, acc_len - need);
                acc_len -= need;
            }
        }
        memcpy(acc + acc_len, tmp, (size_t)r);
        acc_len += (size_t)r;
    }
}

int set_high_voltage_gain(int fd, uint16_t gain)
{
    if (gain > 0x64)
        gain = 0x64;
    uint8_t params[1] = {(uint8_t)(gain & 0xFF)};
    uint8_t frame[16];
    int flen = uv_build_frame(frame, sizeof(frame), 0xC2, 0x03, 0x01, params, sizeof(params));
    if (flen < 0)
        return UV_ERR_BUILD;
    if (send_serial_command(fd, frame, (size_t)flen) < 0)
        return UV_ERR_SEND;

    uint8_t resp[16];
    int rlen = receive_serial_response(fd, resp, sizeof(resp));
    if (rlen < 0)
        return (rlen == -4 ? UV_ERR_TIMEOUT : (rlen == -2 ? UV_ERR_FRAME : (rlen == -3 ? UV_ERR_OVERFLOW : UV_ERR_IO)));
    uint8_t mm1, mm2, kk;
    const uint8_t *p = NULL;
    size_t plen = 0;
    if (uv_parse_frame(resp, (size_t)rlen, &mm1, &mm2, &kk, &p, &plen) != 0)
        return UV_ERR_FRAME;
    if (!(mm1 == 0xC2 && mm2 == 0x03 && kk == 0x01 && p && plen >= 1 && p[0] == (uint8_t)gain))
        return UV_ERR_PARSE;
    return UV_OK;
}

int read_high_voltage_gain(int fd, uint16_t *gain)
{
    uint8_t frame[16];
    if (!gain)
        return UV_ERR_ARG;
    int flen = uv_build_frame(frame, sizeof(frame), 0xC2, 0x03, 0x00, NULL, 0);
    if (flen < 0)
        return UV_ERR_BUILD;
    if (send_serial_command(fd, frame, (size_t)flen) < 0)
        return UV_ERR_SEND;

    uint8_t resp[32];
    int rlen = receive_serial_response(fd, resp, sizeof(resp));
    if (rlen < 0)
        return (rlen == -4 ? UV_ERR_TIMEOUT : (rlen == -2 ? UV_ERR_FRAME : (rlen == -3 ? UV_ERR_OVERFLOW : UV_ERR_IO)));

    uint8_t mm1, mm2, kk;
    const uint8_t *p = NULL;
    size_t plen = 0;
    if (uv_parse_frame(resp, (size_t)rlen, &mm1, &mm2, &kk, &p, &plen) != 0)
        return UV_ERR_FRAME;
    if (mm1 == 0xC2 && mm2 == 0x03 && kk == 0x00 && p && plen >= 1)
    {
        *gain = (uint16_t)p[0];
        return UV_OK;
    }
    return UV_ERR_PARSE;
}

int set_counting_frequency(int fd, uint16_t frequency)
{
    if (frequency == 0)
        frequency = 1;
    if (frequency > 0xFF)
        frequency = 0xFF;
    uint8_t params[1] = {(uint8_t)(frequency & 0xFF)};
    uint8_t frame[16];
    int flen = uv_build_frame(frame, sizeof(frame), 0xC2, 0x01, 0x01, params, sizeof(params));
    if (flen < 0)
        return UV_ERR_BUILD;
    if (send_serial_command(fd, frame, (size_t)flen) < 0)
        return UV_ERR_SEND;

    uint8_t resp[16];
    int rlen = receive_serial_response(fd, resp, sizeof(resp));
    if (rlen < 0)
        return (rlen == -4 ? UV_ERR_TIMEOUT : (rlen == -2 ? UV_ERR_FRAME : (rlen == -3 ? UV_ERR_OVERFLOW : UV_ERR_IO)));
    uint8_t mm1, mm2, kk;
    const uint8_t *p = NULL;
    size_t plen = 0;
    if (uv_parse_frame(resp, (size_t)rlen, &mm1, &mm2, &kk, &p, &plen) != 0)
        return UV_ERR_FRAME;
    if (!(mm1 == 0xC2 && mm2 == 0x01 && kk == 0x01 && p && plen >= 1 && p[0] == (uint8_t)frequency))
        return UV_ERR_PARSE;
    return UV_OK;
}

int read_photon_count(int fd, uint32_t *count)
{
    uint8_t frame[16];
    if (!count)
        return UV_ERR_ARG;
    int flen = uv_build_frame(frame, sizeof(frame), 0xC2, 0x01, 0x02, NULL, 0);
    if (flen < 0)
        return UV_ERR_BUILD;
    if (send_serial_command(fd, frame, (size_t)flen) < 0)
        return UV_ERR_SEND;

    uint8_t resp[32];
    int rlen = receive_serial_response(fd, resp, sizeof(resp));
    if (rlen < 0)
        return (rlen == -4 ? UV_ERR_TIMEOUT : (rlen == -2 ? UV_ERR_FRAME : (rlen == -3 ? UV_ERR_OVERFLOW : UV_ERR_IO)));

    uint8_t mm1, mm2, kk;
    const uint8_t *p = NULL;
    size_t plen = 0;
    if (uv_parse_frame(resp, (size_t)rlen, &mm1, &mm2, &kk, &p, &plen) != 0)
        return UV_ERR_FRAME;
    if (mm1 == 0xC2 && mm2 == 0x01 && kk == 0x02 && p && plen >= 4)
    {
        // uint32_t val = ((uint32_t)p[0]) |
        //                ((uint32_t)p[1] << 8) |
        //                ((uint32_t)p[2] << 16) |
        //                ((uint32_t)p[3] << 24);
        uint32_t val = ((uint32_t)p[0] << 24) |
                       ((uint32_t)p[1] << 16) |
                       ((uint32_t)p[2] << 8) |
                       ((uint32_t)p[3]);
        *count = val;
        return UV_OK;
    }
    return UV_ERR_PARSE;
}

int enable_region_photon_counting(int fd, uint8_t enable)
{
    uint8_t val = enable ? 0x01 : 0x00;
    uint8_t frame[16];
    int flen = uv_build_frame(frame, sizeof(frame), 0xC2, 0x02, 0x01, &val, 1);
    if (flen < 0)
        return UV_ERR_BUILD;
    if (send_serial_command(fd, frame, (size_t)flen) < 0)
        return UV_ERR_SEND;

    uint8_t resp[16];
    int rlen = receive_serial_response(fd, resp, sizeof(resp));
    if (rlen < 0)
        return (rlen == -4 ? UV_ERR_TIMEOUT : (rlen == -2 ? UV_ERR_FRAME : (rlen == -3 ? UV_ERR_OVERFLOW : UV_ERR_IO)));
    uint8_t mm1, mm2, kk;
    const uint8_t *p = NULL;
    size_t plen = 0;
    if (uv_parse_frame(resp, (size_t)rlen, &mm1, &mm2, &kk, &p, &plen) != 0)
        return UV_ERR_FRAME;
    if (!(mm1 == 0xC2 && mm2 == 0x02 && kk == 0x01 && p && plen >= 1 && p[0] == val))
        return UV_ERR_PARSE;
    return UV_OK;
}

int set_region_x_start(int fd, uint16_t x)
{
    if (x > 0x0500)
        x = 0x0500;
    uint8_t params[2] = {(uint8_t)(x & 0xFF), (uint8_t)((x >> 8) & 0xFF)};
    uint8_t frame[16];
    int flen = uv_build_frame(frame, sizeof(frame), 0xC2, 0x02, 0x03, params, sizeof(params));
    if (flen < 0)
        return UV_ERR_BUILD;
    if (send_serial_command(fd, frame, (size_t)flen) < 0)
        return UV_ERR_SEND;

    uint8_t resp[16];
    int rlen = receive_serial_response(fd, resp, sizeof(resp));
    if (rlen < 0)
        return (rlen == -4 ? UV_ERR_TIMEOUT : (rlen == -2 ? UV_ERR_FRAME : (rlen == -3 ? UV_ERR_OVERFLOW : UV_ERR_IO)));
    uint8_t mm1, mm2, kk;
    const uint8_t *p = NULL;
    size_t plen = 0;
    if (uv_parse_frame(resp, (size_t)rlen, &mm1, &mm2, &kk, &p, &plen) != 0)
        return UV_ERR_FRAME;
    if (!(mm1 == 0xC2 && mm2 == 0x02 && kk == 0x03 && p && plen >= 2 && p[0] == (uint8_t)(x & 0xFF) && p[1] == (uint8_t)((x >> 8) & 0xFF)))
        return UV_ERR_PARSE;
    return UV_OK;
}

int set_region_width(int fd, uint16_t width)
{
    if (width < 0x0001)
        width = 0x0001;
    if (width > 0x0500)
        width = 0x0500;
    uint8_t params[2] = {(uint8_t)(width & 0xFF), (uint8_t)((width >> 8) & 0xFF)};
    uint8_t frame[16];
    int flen = uv_build_frame(frame, sizeof(frame), 0xC2, 0x02, 0x05, params, sizeof(params));
    if (flen < 0)
        return UV_ERR_BUILD;
    if (send_serial_command(fd, frame, (size_t)flen) < 0)
        return UV_ERR_SEND;
    uint8_t resp[16];
    int rlen = receive_serial_response(fd, resp, sizeof(resp));
    if (rlen < 0)
        return (rlen == -4 ? UV_ERR_TIMEOUT : (rlen == -2 ? UV_ERR_FRAME : (rlen == -3 ? UV_ERR_OVERFLOW : UV_ERR_IO)));
    uint8_t mm1, mm2, kk;
    const uint8_t *p = NULL;
    size_t plen = 0;
    if (uv_parse_frame(resp, (size_t)rlen, &mm1, &mm2, &kk, &p, &plen) != 0)
        return UV_ERR_FRAME;
    if (!(mm1 == 0xC2 && mm2 == 0x02 && kk == 0x05 && p && plen >= 2 && p[0] == (uint8_t)(width & 0xFF) && p[1] == (uint8_t)((width >> 8) & 0xFF)))
        return UV_ERR_PARSE;
    return UV_OK;
}

int set_region_y_start(int fd, uint16_t y)
{
    if (y > 0x0400)
        y = 0x0400;
    uint8_t params[2] = {(uint8_t)(y & 0xFF), (uint8_t)((y >> 8) & 0xFF)};
    uint8_t frame[16];
    int flen = uv_build_frame(frame, sizeof(frame), 0xC2, 0x02, 0x07, params, sizeof(params));
    if (flen < 0)
        return UV_ERR_BUILD;
    if (send_serial_command(fd, frame, (size_t)flen) < 0)
        return UV_ERR_SEND;
    uint8_t resp[16];
    int rlen = receive_serial_response(fd, resp, sizeof(resp));
    if (rlen < 0)
        return (rlen == -4 ? UV_ERR_TIMEOUT : (rlen == -2 ? UV_ERR_FRAME : (rlen == -3 ? UV_ERR_OVERFLOW : UV_ERR_IO)));
    uint8_t mm1, mm2, kk;
    const uint8_t *p = NULL;
    size_t plen = 0;
    if (uv_parse_frame(resp, (size_t)rlen, &mm1, &mm2, &kk, &p, &plen) != 0)
        return UV_ERR_FRAME;
    if (!(mm1 == 0xC2 && mm2 == 0x02 && kk == 0x07 && p && plen >= 2 && p[0] == (uint8_t)(y & 0xFF) && p[1] == (uint8_t)((y >> 8) & 0xFF)))
        return UV_ERR_PARSE;
    return UV_OK;
}

int set_region_height(int fd, uint16_t height)
{
    if (height < 0x0001)
        height = 0x0001;
    if (height > 0x0400)
        height = 0x0400;
    uint8_t params[2] = {(uint8_t)(height & 0xFF), (uint8_t)((height >> 8) & 0xFF)};
    uint8_t frame[16];
    int flen = uv_build_frame(frame, sizeof(frame), 0xC2, 0x02, 0x09, params, sizeof(params));
    if (flen < 0)
        return UV_ERR_BUILD;
    if (send_serial_command(fd, frame, (size_t)flen) < 0)
        return UV_ERR_SEND;
    uint8_t resp[16];
    int rlen = receive_serial_response(fd, resp, sizeof(resp));
    if (rlen < 0)
        return (rlen == -4 ? UV_ERR_TIMEOUT : (rlen == -2 ? UV_ERR_FRAME : (rlen == -3 ? UV_ERR_OVERFLOW : UV_ERR_IO)));
    uint8_t mm1, mm2, kk;
    const uint8_t *p = NULL;
    size_t plen = 0;
    if (uv_parse_frame(resp, (size_t)rlen, &mm1, &mm2, &kk, &p, &plen) != 0)
        return UV_ERR_FRAME;
    if (!(mm1 == 0xC2 && mm2 == 0x02 && kk == 0x09 && p && plen >= 2 && p[0] == (uint8_t)(height & 0xFF) && p[1] == (uint8_t)((height >> 8) & 0xFF)))
        return UV_ERR_PARSE;
    return UV_OK;
}

// 设置颜色指令：
// D5 5D 05 C2 04 01 XX ED 5D D5
// 指令返回：
// D5 5D 05 C2 04 01 XX ED 5D D5
// XX = 00	==>> 不二值化
// XX = 01	==>> 白色
// XX = 02	==>> 红色
// XX = 03	==>> 绿色
// XX = 04	==>> 蓝色
// XX = 05	==>> 黄色
// XX = 06	==>> 青色
// XX = 07	==>> 洋红
// XX = 08	==>> 银色
// XX = 09	==>> 灰色
// XX = 0A	==>> 栗色
// XX = 0B	==>> 橄榄色
// XX = 0C	==>> 紫色
// XX = 0D	==>> 蓝绿色
// XX = 0E	==>> 海军蓝
// XX = 0F	==>> 橙色
// XX = 10	==>> 粉红色
// XX = 11 ==>>  棕色
// XX = 00	==>> 不二值化
// XX = 01	==>> 白色
// XX = 02	==>> 红色
// XX = 03	==>> 绿色
// XX = 04	==>> 蓝色
// XX = 05	==>> 黄色
// XX = 06	==>> 青色
// XX = 07	==>> 洋红
// XX = 08	==>> 银色
// XX = 09	==>> 灰色
// XX = 0A	==>> 栗色
// XX = 0B	==>> 橄榄色
// XX = 0C	==>> 紫色
// XX = 0D	==>> 蓝绿色
// XX = 0E	==>> 海军蓝
// XX = 0F	==>> 橙色
// XX = 10	==>> 粉红色
// XX = 11 ==>>  棕色
int set_photons_color(int fd, uint8_t value)
{
    uint8_t frame[16];
    int flen = uv_build_frame(frame, sizeof(frame), 0xC2, 0x04, 0x01, &value, 1);
    if (flen < 0)
        return UV_ERR_BUILD;
    if (send_serial_command(fd, frame, (size_t)flen) < 0)
        return UV_ERR_SEND;

    uint8_t resp[16];
    int rlen = receive_serial_response(fd, resp, sizeof(resp));
    if (rlen < 0)
        return (rlen == -4 ? UV_ERR_TIMEOUT : (rlen == -2 ? UV_ERR_FRAME : (rlen == -3 ? UV_ERR_OVERFLOW : UV_ERR_IO)));
    uint8_t mm1, mm2, kk;
    const uint8_t *p = NULL;
    size_t plen = 0;
    if (uv_parse_frame(resp, (size_t)rlen, &mm1, &mm2, &kk, &p, &plen) != 0)
        return UV_ERR_FRAME;
    if (!(mm1 == 0xC2 && mm2 == 0x04 && kk == 0x01 && p && plen >= 1 && p[0] == value))
        return UV_ERR_PARSE;
    return UV_OK;
}