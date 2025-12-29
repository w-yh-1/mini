// 原打算用ffmpeg实现
//  extern "C"
//  {
//  #include <libavcodec/avcodec.h>
//  #include <libavformat/avformat.h>
//  #include <libswscale/swscale.h>
//  }
#include "encoder.h"
#include "font_array.h"
#include "rockchip/rk_venc_cmd.h"
#include "camera_source.h"
#include "unistd.h"
// #include "Log.h"
#include <string.h>
#include <stdlib.h>
#include <cerrno>

#define msleep(x) usleep((x) * 1000)
#define MPP_FRAME_FMT_IS_FBC(fmt) (fmt & MPP_FRAME_FBC_MASK)
#define SZ_4K (4096)
#define VIDEO_FRAME_AUD_LEN 6


static void fill_half_char(char *dst, unsigned char (*src)[32], unsigned int index, unsigned int total_size, unsigned int color = 1);
static unsigned int fill_int_string(char *dst, unsigned int value, unsigned int index, unsigned int total_size);
static void fill_one_char(char *dst, const unsigned char (*src)[32], unsigned int index, unsigned int total_size, unsigned int color = 1);
static unsigned int fill_float_string(char *dst, unsigned int value, unsigned int index, unsigned int total_size);
bool Encoder::isCN = true;

RK_S32 os_get_env_u32(const char *name, RK_U32 *value, RK_U32 default_value);

MPP_RET mpi_enc_gen_smart_gop_ref_cfg(MppEncRefCfg ref, RK_S32 gop_len, RK_S32 vi_len)
{
    MppEncRefLtFrmCfg lt_ref[4];
    MppEncRefStFrmCfg st_ref[16];
    RK_S32 lt_cnt = 1;
    RK_S32 st_cnt = 8;
    RK_S32 pos = 0;
    MPP_RET ret = MPP_OK;
    memset(&lt_ref, 0, sizeof(lt_ref));
    memset(&st_ref, 0, sizeof(st_ref));

    ret = mpp_enc_ref_cfg_set_cfg_cnt(ref, lt_cnt, st_cnt);

    /* set 8 frame lt-ref gap */
    lt_ref[0].lt_idx = 0;
    lt_ref[0].temporal_id = 0;
    lt_ref[0].ref_mode = REF_TO_PREV_LT_REF;
    lt_ref[0].lt_gap = gop_len;
    lt_ref[0].lt_delay = 0;

    ret = mpp_enc_ref_cfg_add_lt_cfg(ref, 1, lt_ref);

    /* st 0 layer 0 - ref */
    st_ref[pos].is_non_ref = 0;
    st_ref[pos].temporal_id = 0;
    st_ref[pos].ref_mode = REF_TO_PREV_INTRA;
    st_ref[pos].ref_arg = 0;
    st_ref[pos].repeat = 0;
    pos++;

    /* st 1 layer 1 - non-ref */
    if (vi_len > 1)
    {
        st_ref[pos].is_non_ref = 0;
        st_ref[pos].temporal_id = 1;
        st_ref[pos].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[pos].ref_arg = 0;
        st_ref[pos].repeat = vi_len - 2;
        pos++;
    }

    st_ref[pos].is_non_ref = 0;
    st_ref[pos].temporal_id = 0;
    st_ref[pos].ref_mode = REF_TO_PREV_INTRA;
    st_ref[pos].ref_arg = 0;
    st_ref[pos].repeat = 0;
    pos++;

    ret = mpp_enc_ref_cfg_add_st_cfg(ref, pos, st_ref);

    /* check and get dpb size */
    ret = mpp_enc_ref_cfg_check(ref);

    return ret;
}
MPP_RET mpi_enc_gen_ref_cfg(MppEncRefCfg ref, RK_S32 gop_mode)
{
    MppEncRefLtFrmCfg lt_ref[4];
    MppEncRefStFrmCfg st_ref[16];
    RK_S32 lt_cnt = 0;
    RK_S32 st_cnt = 0;
    MPP_RET ret = MPP_OK;

    memset(&lt_ref, 0, sizeof(lt_ref));
    memset(&st_ref, 0, sizeof(st_ref));

    switch (gop_mode)
    {
    case 3:
    {
        // tsvc4
        //      /-> P1      /-> P3        /-> P5      /-> P7
        //     /           /             /           /
        //    //--------> P2            //--------> P6
        //   //                        //
        //  ///---------------------> P4
        // ///
        // P0 ------------------------------------------------> P8
        lt_cnt = 1;

        /* set 8 frame lt-ref gap */
        lt_ref[0].lt_idx = 0;
        lt_ref[0].temporal_id = 0;
        lt_ref[0].ref_mode = REF_TO_PREV_LT_REF;
        lt_ref[0].lt_gap = 8;
        lt_ref[0].lt_delay = 0;

        st_cnt = 9;
        /* set tsvc4 st-ref struct */
        /* st 0 layer 0 - ref */
        st_ref[0].is_non_ref = 0;
        st_ref[0].temporal_id = 0;
        st_ref[0].ref_mode = REF_TO_TEMPORAL_LAYER;
        st_ref[0].ref_arg = 0;
        st_ref[0].repeat = 0;
        /* st 1 layer 3 - non-ref */
        st_ref[1].is_non_ref = 1;
        st_ref[1].temporal_id = 3;
        st_ref[1].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[1].ref_arg = 0;
        st_ref[1].repeat = 0;
        /* st 2 layer 2 - ref */
        st_ref[2].is_non_ref = 0;
        st_ref[2].temporal_id = 2;
        st_ref[2].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[2].ref_arg = 0;
        st_ref[2].repeat = 0;
        /* st 3 layer 3 - non-ref */
        st_ref[3].is_non_ref = 1;
        st_ref[3].temporal_id = 3;
        st_ref[3].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[3].ref_arg = 0;
        st_ref[3].repeat = 0;
        /* st 4 layer 1 - ref */
        st_ref[4].is_non_ref = 0;
        st_ref[4].temporal_id = 1;
        st_ref[4].ref_mode = REF_TO_PREV_LT_REF;
        st_ref[4].ref_arg = 0;
        st_ref[4].repeat = 0;
        /* st 5 layer 3 - non-ref */
        st_ref[5].is_non_ref = 1;
        st_ref[5].temporal_id = 3;
        st_ref[5].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[5].ref_arg = 0;
        st_ref[5].repeat = 0;
        /* st 6 layer 2 - ref */
        st_ref[6].is_non_ref = 0;
        st_ref[6].temporal_id = 2;
        st_ref[6].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[6].ref_arg = 0;
        st_ref[6].repeat = 0;
        /* st 7 layer 3 - non-ref */
        st_ref[7].is_non_ref = 1;
        st_ref[7].temporal_id = 3;
        st_ref[7].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[7].ref_arg = 0;
        st_ref[7].repeat = 0;
        /* st 8 layer 0 - ref */
        st_ref[8].is_non_ref = 0;
        st_ref[8].temporal_id = 0;
        st_ref[8].ref_mode = REF_TO_TEMPORAL_LAYER;
        st_ref[8].ref_arg = 0;
        st_ref[8].repeat = 0;
    }
    break;
    case 2:
    {
        // tsvc3
        //     /-> P1      /-> P3
        //    /           /
        //   //--------> P2
        //  //
        // P0/---------------------> P4
        lt_cnt = 0;

        st_cnt = 5;
        /* set tsvc4 st-ref struct */
        /* st 0 layer 0 - ref */
        st_ref[0].is_non_ref = 0;
        st_ref[0].temporal_id = 0;
        st_ref[0].ref_mode = REF_TO_TEMPORAL_LAYER;
        st_ref[0].ref_arg = 0;
        st_ref[0].repeat = 0;
        /* st 1 layer 2 - non-ref */
        st_ref[1].is_non_ref = 1;
        st_ref[1].temporal_id = 2;
        st_ref[1].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[1].ref_arg = 0;
        st_ref[1].repeat = 0;
        /* st 2 layer 1 - ref */
        st_ref[2].is_non_ref = 0;
        st_ref[2].temporal_id = 1;
        st_ref[2].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[2].ref_arg = 0;
        st_ref[2].repeat = 0;
        /* st 3 layer 2 - non-ref */
        st_ref[3].is_non_ref = 1;
        st_ref[3].temporal_id = 2;
        st_ref[3].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[3].ref_arg = 0;
        st_ref[3].repeat = 0;
        /* st 4 layer 0 - ref */
        st_ref[4].is_non_ref = 0;
        st_ref[4].temporal_id = 0;
        st_ref[4].ref_mode = REF_TO_TEMPORAL_LAYER;
        st_ref[4].ref_arg = 0;
        st_ref[4].repeat = 0;
    }
    break;
    case 1:
    {
        // tsvc2
        //   /-> P1
        //  /
        // P0--------> P2
        lt_cnt = 0;

        st_cnt = 3;
        /* set tsvc4 st-ref struct */
        /* st 0 layer 0 - ref */
        st_ref[0].is_non_ref = 0;
        st_ref[0].temporal_id = 0;
        st_ref[0].ref_mode = REF_TO_TEMPORAL_LAYER;
        st_ref[0].ref_arg = 0;
        st_ref[0].repeat = 0;
        /* st 1 layer 2 - non-ref */
        st_ref[1].is_non_ref = 1;
        st_ref[1].temporal_id = 1;
        st_ref[1].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[1].ref_arg = 0;
        st_ref[1].repeat = 0;
        /* st 2 layer 1 - ref */
        st_ref[2].is_non_ref = 0;
        st_ref[2].temporal_id = 0;
        st_ref[2].ref_mode = REF_TO_PREV_REF_FRM;
        st_ref[2].ref_arg = 0;
        st_ref[2].repeat = 0;
    }
    break;
    default:
    {
        USER_LOG_ERROR("unsupport gop mode %d\n", gop_mode);
    }
    break;
    }

    if (lt_cnt || st_cnt)
    {
        ret = mpp_enc_ref_cfg_set_cfg_cnt(ref, lt_cnt, st_cnt);

        if (lt_cnt)
            ret = mpp_enc_ref_cfg_add_lt_cfg(ref, lt_cnt, lt_ref);

        if (st_cnt)
            ret = mpp_enc_ref_cfg_add_st_cfg(ref, st_cnt, st_ref);

        /* check and get dpb size */
        ret = mpp_enc_ref_cfg_check(ref);
    }

    return ret;
}
MPP_RET mpi_enc_gen_osd_plt(MppEncOSDPlt *osd_plt, RK_U32 frame_cnt)
{
    /*
     * osd idx size range from 16x16 bytes(pixels) to hor_stride*ver_stride(bytes).
     * for general use, 1/8 Y buffer is enough.
     */
    static int plt_table[8] = {
        MPP_ENC_OSD_PLT_TRANS,
        MPP_ENC_OSD_PLT_WHITE,
        MPP_ENC_OSD_PLT_GREEN,
        MPP_ENC_OSD_PLT_BLUE,
        MPP_ENC_OSD_PLT_YELLOW,
        MPP_ENC_OSD_PLT_RED,
        MPP_ENC_OSD_PLT_CYAN,
        MPP_ENC_OSD_PLT_BLACK,
    };

    if (osd_plt)
    {
        RK_U32 k = 0;
        RK_U32 base = frame_cnt & 7;

        for (k = 0; k < 256; k++)
            osd_plt->data[k].val = plt_table[(base + k) % 8];
    }
    return MPP_OK;
}
static void fill_one_char(char *dst, const unsigned char (*src)[32], unsigned int index, unsigned int total_size, unsigned int color)
{
    char (*buf)[total_size * 32] = (char (*)[total_size * 32]) dst;
    for (int m = 0; m < 4; m++)
    {
        for (int n = 0; n < 32; n++)
        {
            buf[n][index] = src[m][n] >> 7 & 0x01 ? color : 0;
            buf[n][index + 1] = src[m][n] >> 6 & 0x01 ? color : 0;
            buf[n][index + 2] = src[m][n] >> 5 & 0x01 ? color : 0;
            buf[n][index + 3] = src[m][n] >> 4 & 0x01 ? color : 0;
            buf[n][index + 4] = src[m][n] >> 3 & 0x01 ? color : 0;
            buf[n][index + 5] = src[m][n] >> 2 & 0x01 ? color : 0;
            buf[n][index + 6] = src[m][n] >> 1 & 0x01 ? color : 0;
            buf[n][index + 7] = src[m][n] & 0x01 ? color : 0;
        }
        index += 8;
    }
}
// total_size 一行有几个dw。
static void fill_half_char(char *dst, unsigned char (*src)[32], unsigned int index, unsigned int total_size, unsigned int color)
{
    char (*buf)[total_size * 32] = (char (*)[total_size * 32]) dst;
    for (int m = 0; m < 2; m++)
    {
        for (int n = 0; n < 32; n++)
        {
            buf[n][index] = src[m][n] >> 7 & 0x01 ? color : 0;
            buf[n][index + 1] = src[m][n] >> 6 & 0x01 ? color : 0;
            buf[n][index + 2] = src[m][n] >> 5 & 0x01 ? color : 0;
            buf[n][index + 3] = src[m][n] >> 4 & 0x01 ? color : 0;
            buf[n][index + 4] = src[m][n] >> 3 & 0x01 ? color : 0;
            buf[n][index + 5] = src[m][n] >> 2 & 0x01 ? color : 0;
            buf[n][index + 6] = src[m][n] >> 1 & 0x01 ? color : 0;
            buf[n][index + 7] = src[m][n] & 0x01 ? color : 0;
        }
        index += 8;
    }
}
static void fill_string(char *dst, const char *str, unsigned int index, unsigned int line_width = 8, unsigned int color = 1)
{
    unsigned int str_len = strlen(str);
    for (int i = 0; i < str_len; i++, index += 16)
    {
        fill_half_char(dst, &Alpha_bet[(str[i] - 'a') * 2], index, line_width, color);
    }
}
/// @brief 在某行中指定位置填充整数数字字符串,返回数字的位数
/// @param dst osd缓冲区
/// @param value 数字
/// @param index 指定填充位置
/// @param total_size 缓冲区当前行宽
static unsigned int fill_int_string(char *dst, unsigned int value, unsigned int index, unsigned int total_size)
{
    unsigned int ret;
    unsigned int every_num[6] = {0};
    int i = 0;
    for (i = 0; value > 9; i++)
    {
        every_num[i] = value % 10;
        value /= 10;
    }
    ret = i + 1;
    every_num[i] = value;
    for (; i >= 0; i--, index += 16)
    {
        fill_half_char(dst, (unsigned char (*)[32]) & NUM[2 * every_num[i]][0], index, total_size);
    }
    return ret;
}

/// @brief 在某行中指定位置填充一位小数浮点数字符串,返回数字的位数（小数点算一位）
/// @param dst osd缓冲区
/// @param value 数字*10
/// @param index 指定填充位置（从开头起第几个像素）
/// @param total_size 缓冲区当前行宽
static unsigned int fill_float_string(char *dst, unsigned int value, unsigned int index, unsigned int total_size)
{
    unsigned int every_num[6] = {0};
    int ret;
    int i;
    for (i = 0; value > 9; i++)
    {
        every_num[i] = value % 10;
        value /= 10;
    }
    every_num[i] = value;

    if (!i)
    {
        fill_half_char(dst, (unsigned char (*)[32]) & NUM[0][0], index, total_size); // 补0
        index += 16;
    }

    ret = i + 3;
    for (; i >= 0; i--, index += 16)
    {
        if (i)
            fill_half_char(dst, (unsigned char (*)[32]) & NUM[2 * every_num[i]][0], index, total_size); // 整数
        else
        {
            fill_half_char(dst, (unsigned char (*)[32]) & NUM[2 * 10][0], index, total_size); //.
            index += 16;
            fill_half_char(dst, (unsigned char (*)[32]) & NUM[2 * every_num[i]][0], index, total_size); // 小数
        }
    }
    return ret;
}
void Encoder::draw_en_osd(char *ptr, int DW_per_line)
{
    int i, count;
    // fill_string(ptr, "uvirsys{", 0, DW_per_line);
    ptr += DW_per_line * 1024; // 行宽mb*字高mb*256

    // fill_string(ptr, "mode", 0, DW_per_line);      // MODE
    // fill_one_char(ptr, &模式[8], 64, DW_per_line); // MODE:
    // fill_string(ptr, "vl", 32 * 3, DW_per_line);   // MODE: VL
    ptr += DW_per_line * 1024;                     // 下一行

    fill_string(ptr, "gain", 0, DW_per_line); // GAIN
    for (i = 2; i < 5; i++)
        fill_one_char(ptr, (unsigned char (*)[32]) & 增益[i * 4][0], i * 32, DW_per_line); // GAIN:   %
    ptr += DW_per_line * 1024;                                                             // 下一行

    fill_string(ptr, "cont", 0, DW_per_line);      // UVPT
    fill_one_char(ptr, &计数[8], 64, DW_per_line); // UVPT:
    count = fill_int_string(ptr, 0, 3 * 32, DW_per_line);
    fill_one_char(ptr, (unsigned char (*)[32])分计数, 3 * 32 + count * 16, DW_per_line, 6); // UVPT:0%
    ptr += DW_per_line * 1024;                                                              // 下一行

    // fill_string(ptr, "dist", 0, DW_per_line);       // DIST
    // fill_one_char(ptr, &距离[8], 64, DW_per_line);  // DIST:
    // fill_float_string(ptr, 0, 3 * 32, DW_per_line); // DIST: 0.0
    // ptr += DW_per_line * 1024;

    // fill_string(ptr, "flaw", 0, DW_per_line);      // FLAW
    // fill_one_char(ptr, &缺陷[8], 64, DW_per_line); // FLAW:
    // fill_string(ptr, "none", 3 * 32, DW_per_line); // FLAW:NONE
}
void Encoder::draw_time_osd(MppEncOSDData *osd_data)
{
    int index = 0;
    MppBuffer buf = osd_data->buf;
    MppEncOSDRegion *region = osd_data->region;
    RK_U32 mb_w = region->num_mb_x;
    char *ptr = (char *)mpp_buffer_get_ptr(buf);

    time_t t = time(NULL);
    struct tm now_time;
    localtime_r(&t, &now_time);         // 填充 now_time 结构体
    int year = now_time.tm_year + 1900; // 年份需 +1900
    int month = now_time.tm_mon + 1;    // 月份从 0 开始，需 +1
    int day = now_time.tm_mday;
    int hour = now_time.tm_hour;
    int minute = now_time.tm_min;
    int second = now_time.tm_sec;
    // year
    fill_int_string(ptr, year, index, mb_w / 2);
    index += 16 * 4;
    fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 12][0], index, mb_w / 2);
    index += 16;
    // month
    if (month < 10)
    {
        fill_int_string(ptr, 0, index, mb_w / 2);
        index += 16;
        fill_int_string(ptr, month, index, mb_w / 2);
        index += 16;
    }
    else
    {
        fill_int_string(ptr, month, index, mb_w / 2);
        index += 16 * 2;
    }
    fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 12][0], index, mb_w / 2);
    index += 16;
    // day
    if (day < 10)
    {
        fill_int_string(ptr, 0, index, mb_w / 2);
        index += 16;
        fill_int_string(ptr, day, index, mb_w / 2);
        index += 16;
    }
    else
    {
        fill_int_string(ptr, day, index, mb_w / 2);
        index += 16 * 2;
    }
    fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 11][0], index, mb_w / 2);
    index += 16;
    // hour
    if (hour < 10)
    {
        fill_int_string(ptr, 0, index, mb_w / 2);
        index += 16;
        fill_int_string(ptr, hour, index, mb_w / 2);
        index += 16;
    }
    else
    {
        fill_int_string(ptr, hour, index, mb_w / 2);
        index += 16 * 2;
    }
    fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 13][0], index, mb_w / 2);
    index += 16;
    // minute
    if (minute < 10)
    {
        fill_int_string(ptr, 0, index, mb_w / 2);
        index += 16;
        fill_int_string(ptr, minute, index, mb_w / 2);
        index += 16;
    }
    else
    {
        fill_int_string(ptr, minute, index, mb_w / 2);
        index += 16 * 2;
    }
    fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 13][0], index, mb_w / 2);
    index += 16;
    // second
    if (second < 10)
    {
        fill_int_string(ptr, 0, index, mb_w / 2);
        index += 16;
        fill_int_string(ptr, second, index, mb_w / 2);
        index += 16;
    }
    else
    {
        fill_int_string(ptr, second, index, mb_w / 2);
        index += 16 * 2;
    }
}

void Encoder::draw_cn_osd(char *ptr, int DW_per_line)
{
    int i;
    int count;
    int gain = 0;
    // for (i = 0; i < 4; i++) // i<字符串长度
    // {
    //     fill_one_char(ptr, (unsigned char(*)[32]) & 紫红光电[i * 4][0], i * 32, DW_per_line);
    // }
    ptr += DW_per_line * 1024; // 行宽mb*字高mb*256
    // for (i = 0; i < 3; i++)
    // {
    //     fill_one_char(ptr, (unsigned char (*)[32]) & 模式[i * 4][0], i * 32, DW_per_line);
    // }
    // for (i = 3; i < 5; i++)
    // {
    //     fill_one_char(ptr, (unsigned char (*)[32]) & 可见[(i - 3) * 4][0], i * 32, DW_per_line);
    // }
    ptr += DW_per_line * 1024; // 下一行
    for (i = 0; i < 5; i++)
    {
        fill_one_char(ptr, (unsigned char (*)[32]) & 增益[i * 4][0], i * 32, DW_per_line);
    }
    if (gain < 100)
        fill_int_string(ptr, gain, 6 * 16, DW_per_line);
    else
        fill_int_string(ptr, gain, 5 * 16, DW_per_line);
    ptr += DW_per_line * 1024; // 下一行
    for (i = 0; i < 3; i++)
    {
        fill_one_char(ptr, (unsigned char (*)[32]) & 计数[i * 4][0], i * 32, DW_per_line);
    }

    // count = fill_int_string(ptr, 0, i * 32, DW_per_line);
    // fill_one_char(ptr, (unsigned char (*)[32])分计数, i * 32 + count * 16, DW_per_line, 6);
    // ptr += DW_per_line * 1024;
    // for (i = 0; i < 3; i++)
    // {
    //     fill_one_char(ptr, (unsigned char (*)[32]) & 距离[i * 4][0], i * 32, DW_per_line);
    // }

    // 缺陷类型
    //  fill_float_string(ptr, 0, 3 * 32, DW_per_line);
    //  ptr += DW_per_line * 1024;
    //  for (i = 0; i < 3; i++)
    //  {
    //      fill_one_char(ptr, (unsigned char(*)[32]) & 缺陷[i * 4][0], i * 32, DW_per_line);
    //  }
    //  for (i = 6; i < 13; i++)
    //  {
    //      // fill_one_char(ptr, (unsigned char(*)[32]) & 暂无[(i - 3) * 4][0], i * 32, DW_per_line, 2);
    //      fill_half_char(ptr, (unsigned char(*)[32]) & NUM[2 * 11][0], i * 16, DW_per_line);
    //  }
}
RK_S32 mpi_enc_width_default_stride(RK_S32 width, MppFrameFormat fmt)
{
    RK_S32 stride = 0;

    switch (fmt & MPP_FRAME_FMT_MASK)
    {
    case MPP_FMT_YUV420SP:
    case MPP_FMT_YUV420SP_VU:
    {
        stride = MPP_ALIGN(width, 8);
    }
    break;
    case MPP_FMT_YUV420P:
    {
        /* NOTE: 420P need to align to 16 so chroma can align to 8 */
        stride = MPP_ALIGN(width, 16);
    }
    break;
    case MPP_FMT_YUV422P:
    case MPP_FMT_YUV422SP:
    case MPP_FMT_YUV422SP_VU:
    {
        /* NOTE: 422 need to align to 8 so chroma can align to 16 */
        stride = MPP_ALIGN(width, 8);
    }
    break;
    case MPP_FMT_YUV444SP:
    case MPP_FMT_YUV444P:
    {
        stride = MPP_ALIGN(width, 8);
    }
    break;
    case MPP_FMT_RGB565:
    case MPP_FMT_BGR565:
    case MPP_FMT_RGB555:
    case MPP_FMT_BGR555:
    case MPP_FMT_RGB444:
    case MPP_FMT_BGR444:
    case MPP_FMT_YUV422_YUYV:
    case MPP_FMT_YUV422_YVYU:
    case MPP_FMT_YUV422_UYVY:
    case MPP_FMT_YUV422_VYUY:
    {
        /* NOTE: for vepu limitation */
        stride = MPP_ALIGN(width, 8) * 2;
    }
    break;
    case MPP_FMT_RGB888:
    case MPP_FMT_BGR888:
    {
        /* NOTE: for vepu limitation */
        stride = MPP_ALIGN(width, 8) * 3;
    }
    break;
    case MPP_FMT_RGB101010:
    case MPP_FMT_BGR101010:
    case MPP_FMT_ARGB8888:
    case MPP_FMT_ABGR8888:
    case MPP_FMT_BGRA8888:
    case MPP_FMT_RGBA8888:
    {
        /* NOTE: for vepu limitation */
        stride = MPP_ALIGN(width, 8) * 4;
    }
    break;
    default:
    {
        USER_LOG_ERROR("do not support type %d\n", fmt);
    }
    break;
    }

    return stride;
}


MPP_RET Encoder::gen_osd_data(MppEncOSDData *osd_data, MppBufferGroup group)
{
    MppEncOSDRegion *region = osd_data->region;
    MppBuffer buf = osd_data->buf;
    int DW_per_line = 8;
    RK_U32 mb_w = DW_per_line * 2; // 每行8个字，每字占2mb（32byte）
    RK_U32 mb_h = 2 * 6;           // 每个字高32p 模式，光子数，距离，增益，缺陷,共5行
    RK_U32 buf_size = 0;
    osd_data->num_region = 1;
    int count = 0;

    if (buf)
        buf_size = mpp_buffer_get_size(buf);
    // 公司名称，模式，光子数，距离，增益，缺陷
    region[0].inverse = 0;
    region[0].start_mb_x = 7;
    region[0].start_mb_y = 0;
    region[0].num_mb_x = mb_w;
    region[0].num_mb_y = mb_h;
    region[0].buf_offset = 0;
    region[0].enable = 1;
    /* create buffer and write osd index data */
    if (buf_size < mb_w * mb_h * 256)
    {
        buf_size = mb_w * mb_h * 256 ; // 
        if (buf)
            mpp_buffer_put(buf);

        mpp_buffer_get(group, &buf, buf_size);
        if (NULL == buf)
            USER_LOG_ERROR("failed to create osd buffer size %d\n", buf_size);
    }

    if (buf)
    {
        char *ptr = (char *)mpp_buffer_get_ptr(buf);
        if (isCN)
            draw_cn_osd(ptr, DW_per_line);
        else
            draw_en_osd(ptr, DW_per_line);
    }

    osd_data->buf = buf;
    return MPP_OK;
}

void Encoder::switch_language(MppEncOSDData *osd_data, bool is_CN)
{

    Encoder::isCN = is_CN;
    MppBuffer buf = osd_data->buf;

    if (buf)
    {
        char *ptr = (char *)mpp_buffer_get_ptr(buf);
        if (isCN)
            draw_cn_osd(ptr);
        else
            draw_en_osd(ptr);
    }
}

static uint32_t calculate_nal_len(char *ptr, int len)
{

    int head_index = -1;
    int end_index = -1;
    char nal_head[] = {0x00, 0x00, 0x00, 0x01};
    uint32_t i = 0, j = 0;

    for (;;)
    {
        if (ptr[i + j] != nal_head[j])
        {
            i++;
            j = 0;
            continue;
        }
        else
            j++;

        if (j == 4)
        {

            if (head_index < 0)
            {
                head_index = i;
                i += 4;
            }
            else if (end_index < 0)
            {
                end_index = i;
                break;
            }
            j = 0;
        }
    }

    return end_index - head_index;
}
Encoder::Encoder(RK_U32 width, RK_U32 height, MppFrameFormat frame_fmt, MppCodingType type, MppEncRcMode rc_mode) : args{0}, osd_data{0}, m_filepath{0}, m_record_state(record_idle)
{
    args.width = width;
    args.height = height;
    args.fmt = frame_fmt;
    args.hor_stride = mpi_enc_width_default_stride(width, args.fmt);
    args.ver_stride = MPP_ALIGN(height, 16);
    args.rc_mode = rc_mode;
    args.type = type;
    args.frame_size = calc_buf_size(frame_fmt);
    if (MPP_FRAME_FMT_IS_FBC(args.fmt))
    {
        if ((args.fmt & MPP_FRAME_FBC_MASK) == MPP_FRAME_FBC_AFBC_V1)
            args.header_size = MPP_ALIGN(MPP_ALIGN(args.width, 16) * MPP_ALIGN(args.height, 16) / 16, SZ_4K);
        else
            args.header_size = MPP_ALIGN(args.width, 16) * MPP_ALIGN(args.height, 16) / 16;
    }
    else
    {
        args.header_size = 0;
    }
}

Encoder::~Encoder()
{
    deinit();
}
MPP_RET Encoder::init(pOutput_func output_func)
{
    MPP_RET ret;
    ret = mpp_buffer_group_get_internal(&buf_grp, MPP_BUFFER_TYPE_DRM);
    if (ret)
    {
        USER_LOG_ERROR("failed to get mpp buffer group ret %d\n", ret);
        goto OUT;
    }
    for (int i = 0; i < 2; i++)
    {
        ret = mpp_buffer_get(buf_grp, &mEnc_buf[i].mpp_buf, args.frame_size + args.header_size);
        if (ret)
        {
            USER_LOG_ERROR("failed to get buffer for input frame ret %d\n", ret);
            goto OUT;
        }
        mEnc_buf[i].ptr = mpp_buffer_get_ptr(mEnc_buf[i].mpp_buf);
        mEnc_buf[i].fd = mpp_buffer_get_fd(mEnc_buf[i].mpp_buf);
        mEnc_buf[i].rga_buf_handle = importbuffer_fd(mEnc_buf[i].fd, (args.frame_size + args.header_size));
        mEnc_buf[i].rga_buf = wrapbuffer_handle(
            mEnc_buf[i].rga_buf_handle,
            args.width,
            args.height,
            change_MppFmt2RkFmt(args.fmt)); // 删除了, args.hor_stride, args.ver_stride
    }
    mCurrent_Enc_buf = &mEnc_buf[0];

    ret = mpp_buffer_get(buf_grp, &pkt_buf, args.frame_size + args.header_size);
    if (ret)
    {
        USER_LOG_ERROR("failed to get buffer for output packet ret %d\n", ret);
        goto OUT;
    }

    ret = mpp_create(&ctx, &mpi);
    if (ret)
    {
        USER_LOG_ERROR("mpp_create failed ret %d\n", ret);
        goto OUT;
    }
    ret = mpi->control(ctx, MPP_SET_INPUT_TIMEOUT, &timeout);
    if (ret)
    {
        USER_LOG_ERROR("mpi control set input timeout %d ret %d\n", timeout, ret);
        return ret;
    }

    timeout = MPP_POLL_BLOCK;
    ret = mpi->control(ctx, MPP_SET_OUTPUT_TIMEOUT, &timeout);
    if (MPP_OK != ret)
    {
        USER_LOG_ERROR("mpi control set output timeout %d ret %d\n", timeout, ret);
        goto OUT;
    }
    ret = mpp_init(ctx, MPP_CTX_ENC, args.type);
    if (ret)
    {
        USER_LOG_ERROR("mpp_init failed ret %d\n", ret);
        goto OUT;
    }
    ret = mpp_enc_cfg_init(&cfg);
    if (ret)
    {
        USER_LOG_ERROR("mpp_enc_cfg_init failed ret %d\n", ret);
        goto OUT;
    }
    ret = mpp_frame_init(&frame);
    if (ret)
    {
        USER_LOG_ERROR("mpp_frame_init failed\n");
        goto OUT;
    }
    if (!osd_data.num_region)
        gen_osd_data(&osd_data, buf_grp);
    mpp_frame_set_width(frame, args.width);
    mpp_frame_set_height(frame, args.height);
    mpp_frame_set_hor_stride(frame, args.hor_stride);
    mpp_frame_set_ver_stride(frame, args.ver_stride);
    mpp_frame_set_fmt(frame, args.fmt);

    // cfg_setup();
    ret = cfg_setup();
    if (ret)
    {
        USER_LOG_ERROR("mpp_enc_cfg_setup failed ret %d\n", ret);
        goto OUT;
    }
    // set_output_func()
    if (set_output_func(output_func) != true)
    {
        USER_LOG_ERROR("set_output_func failed \n");
        goto OUT;
    }
    USER_LOG_INFO("Encoder init success\n");
    return ret;
OUT:
    deinit();
    return MPP_NOK;
}

MPP_RET Encoder::cfg_setup()
{
    MPP_RET ret;
    if (cfg == NULL || ctx == NULL)
        USER_LOG_ERROR("未初始化\n");
    if (args.fps_in_den == 0)
        args.fps_in_den = 1;
    if (args.fps_in_num == 0)
        args.fps_in_num = 30;
    if (args.fps_out_den == 0)
        args.fps_out_den = 1;
    if (args.fps_out_num == 0)
        args.fps_out_num = 30;
    if (!args.bps)
    {
        // args.bps = args.width * args.height / 8 * (args.fps_out_num / args.fps_out_den);
        // args.bps = 4800000;
        args.bps = 6400000; // 8M
    }
    args.gop_len = 30;

    ret = mpp_enc_cfg_set_s32(cfg, "prep:width", args.width);
    ret = mpp_enc_cfg_set_s32(cfg, "prep:height", args.height);
    ret = mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", args.hor_stride);
    ret = mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", args.ver_stride);
    ret = mpp_enc_cfg_set_s32(cfg, "prep:format", args.fmt);
    ret = mpp_enc_cfg_set_s32(cfg, "rc:mode", args.rc_mode);
    /* fix input / output frame rate */
    ret = mpp_enc_cfg_set_s32(cfg, "rc:fps_in_flex", args.fps_in_flex);
    ret = mpp_enc_cfg_set_s32(cfg, "rc:fps_in_num", args.fps_in_num);
    ret = mpp_enc_cfg_set_s32(cfg, "rc:fps_in_denorm", args.fps_in_den);
    ret = mpp_enc_cfg_set_s32(cfg, "rc:fps_out_flex", args.fps_out_flex);
    ret = mpp_enc_cfg_set_s32(cfg, "rc:fps_out_num", args.fps_out_num);
    ret = mpp_enc_cfg_set_s32(cfg, "rc:fps_out_denorm", args.fps_out_den);
    ret = mpp_enc_cfg_set_s32(cfg, "rc:gop", args.gop_len ? args.gop_len : args.fps_out_num);

    /* drop frame or not when bitrate overflow */
    mpp_enc_cfg_set_u32(cfg, "rc:drop_mode", MPP_ENC_RC_DROP_FRM_DISABLED);
    mpp_enc_cfg_set_u32(cfg, "rc:drop_thd", 20); /* 20% of max bps */
    mpp_enc_cfg_set_u32(cfg, "rc:drop_gap", 1);  /* Do not continuous drop frame */
    /* setup bitrate for different rc_mode */
    ret = mpp_enc_cfg_set_s32(cfg, "rc:bps_target", args.bps);
    switch (args.rc_mode)
    {
    case MPP_ENC_RC_MODE_FIXQP:
    {
        /* do not setup bitrate on FIXQP mode */
    }
    break;
    case MPP_ENC_RC_MODE_CBR:
    {
        /* CBR mode has narrow bound */
        ret = mpp_enc_cfg_set_s32(cfg, "rc:bps_max", args.bps_max ? args.bps_max : args.bps * 17 / 16);
        ret = mpp_enc_cfg_set_s32(cfg, "rc:bps_min", args.bps_min ? args.bps_min : args.bps * 15 / 16);
    }
    break;
    case MPP_ENC_RC_MODE_VBR:
    case MPP_ENC_RC_MODE_AVBR:
    {
        /* VBR mode has wide bound */
        ret = mpp_enc_cfg_set_s32(cfg, "rc:bps_max", args.bps_max ? args.bps_max : args.bps * 17 / 16);
        ret = mpp_enc_cfg_set_s32(cfg, "rc:bps_min", args.bps_min ? args.bps_min : args.bps * 3 / 4);
    }
    break;
    default:
    {
        /* default use CBR mode */
        ret = mpp_enc_cfg_set_s32(cfg, "rc:bps_max", args.bps_max ? args.bps_max : args.bps * 17 / 16);
        ret = mpp_enc_cfg_set_s32(cfg, "rc:bps_min", args.bps_min ? args.bps_min : args.bps * 15 / 16);
    }
    break;
    }
    /* setup qp for different codec and rc_mode */
    switch (args.type)
    {
    case MPP_VIDEO_CodingAVC:
    case MPP_VIDEO_CodingHEVC:
    {
        switch (args.rc_mode)
        {
        case MPP_ENC_RC_MODE_FIXQP:
        {
            RK_S32 fix_qp = args.qp_init;

            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_init", fix_qp);
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_max", fix_qp);
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_min", fix_qp);
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", fix_qp);
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", fix_qp);
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 0);
        }
        break;
        case MPP_ENC_RC_MODE_CBR:
        case MPP_ENC_RC_MODE_VBR:
        case MPP_ENC_RC_MODE_AVBR:
        {
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_init", -1);
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 50);
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 10);
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 50);
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 10);
            ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 2);
        }
        break;
        default:
        {
            USER_LOG_ERROR("unsupport encoder rc mode %d\n", args.rc_mode);
        }
        break;
        }
    }
    break;
    case MPP_VIDEO_CodingVP8:
    {
        /* vp8 only setup base qp range */
        ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_init", 40);
        ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 127);
        ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 0);
        ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 127);
        ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 0);
        ret = mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 6);
    }
    break;
    case MPP_VIDEO_CodingMJPEG:
    {
        /* jpeg use special codec config to control qtable */
        ret = mpp_enc_cfg_set_s32(cfg, "jpeg:q_factor", 80);
        ret = mpp_enc_cfg_set_s32(cfg, "jpeg:qf_max", 99);
        ret = mpp_enc_cfg_set_s32(cfg, "jpeg:qf_min", 1);
    }
    break;
    default:
    {
    }
    break;
    }

    ret = mpp_enc_cfg_set_s32(cfg, "codec:type", args.type);
    switch (args.type)
    {
    case MPP_VIDEO_CodingAVC:
    {
        RK_U32 constraint_set;

        /*
         * H.264 profile_idc parameter
         * 66  - Baseline profile
         * 77  - Main profile
         * 100 - High profile
         */
        ret = mpp_enc_cfg_set_s32(cfg, "h264:profile", 100);
        /*
         * H.264 level_idc parameter
         * 10 / 11 / 12 / 13    - qcif@15fps / cif@7.5fps / cif@15fps / cif@30fps
         * 20 / 21 / 22         - cif@30fps / half-D1@@25fps / D1@12.5fps
         * 30 / 31 / 32         - D1@25fps / 720p@30fps / 720p@60fps
         * 40 / 41 / 42         - 1080p@30fps / 1080p@30fps / 1080p@60fps
         * 50 / 51 / 52         - 4K@30fps
         */
        ret = mpp_enc_cfg_set_s32(cfg, "h264:level", 41);
        ret = mpp_enc_cfg_set_s32(cfg, "h264:cabac_en", 1);
        ret = mpp_enc_cfg_set_s32(cfg, "h264:cabac_idc", 0);
        ret = mpp_enc_cfg_set_s32(cfg, "h264:trans8x8", 1);
        os_get_env_u32("constraint_set", &constraint_set, 0);
        if (constraint_set & 0x3f0000)
            ret = mpp_enc_cfg_set_s32(cfg, "h264:constraint_set", constraint_set);
    }
    break;
    case MPP_VIDEO_CodingHEVC:
    case MPP_VIDEO_CodingMJPEG:
    case MPP_VIDEO_CodingVP8:
    {
    }
    break;
    default:
    {
        USER_LOG_ERROR("unsupport encoder coding type %d\n", args.type);
    }
    break;
    }

    mpp_env_get_u32("split_mode", &args.split_mode, MPP_ENC_SPLIT_NONE);
    mpp_env_get_u32("split_arg", &args.split_arg, 0);
    mpp_env_get_u32("split_out", &args.split_out, 0);
    args.split_mode = MPP_ENC_SPLIT_BY_BYTE;
    args.split_arg = 40000;
    args.split_out = MPP_ENC_SPLIT_OUT_LOWDELAY;
    if (args.split_mode)
    {
        mpp_enc_cfg_set_s32(cfg, "split:mode", args.split_mode);
        mpp_enc_cfg_set_s32(cfg, "split:arg", args.split_arg);
        mpp_enc_cfg_set_s32(cfg, "split:out", args.split_out);
    }

    /* setup intra refresh 实现GDR刷新配置*/
    // args.refresh_en = true;
    // args.refresh_mode = MPP_ENC_RC_INTRA_REFRESH_ROW; // 逐行刷新
    // args.refresh_num = 6;
    // if (args.refresh_en)
    // {
    //     mpp_enc_cfg_set_u32(cfg, "rc:refresh_en", args.refresh_en);
    //     mpp_enc_cfg_set_u32(cfg, "rc:refresh_mode", args.refresh_mode);
    //     mpp_enc_cfg_set_u32(cfg, "rc:refresh_num", args.refresh_num);
    // }

    ret = mpi->control(ctx, MPP_ENC_SET_CFG, cfg);
    if (ret)
    {
        USER_LOG_ERROR("mpi control enc set cfg failed ret %d\n", ret);
        goto RET;
    }
    {
        RK_U32 sei_mode;
        mpp_env_get_u32("sei_mode", &sei_mode, MPP_ENC_SEI_MODE_DISABLE);
        args.sei_mode = (MppEncSeiMode)sei_mode;
        ret = mpi->control(ctx, MPP_ENC_SET_SEI_CFG, &args.sei_mode);
        if (ret)
        {
            USER_LOG_ERROR("mpi control enc set sei cfg failed ret %d\n", ret);
            goto RET;
        }
    }
    if (args.type == MPP_VIDEO_CodingAVC || args.type == MPP_VIDEO_CodingHEVC)
    {
        args.header_mode = MPP_ENC_HEADER_MODE_EACH_IDR;
        ret = mpi->control(ctx, MPP_ENC_SET_HEADER_MODE, &args.header_mode);
        if (ret)
        {
            USER_LOG_ERROR("mpi control enc set header mode failed ret %d\n", ret);
            goto RET;
        }
    }
    {
        RK_U32 gop_mode = (RK_U32)args.gop_mode;
        mpp_env_get_u32("gop_mode", &gop_mode, gop_mode);
        if (gop_mode)
        {
            MppEncRefCfg ref;
            mpp_enc_ref_cfg_init(&ref);

            if (args.gop_mode < 4)
                mpi_enc_gen_ref_cfg(ref, gop_mode);
            else
                mpi_enc_gen_smart_gop_ref_cfg(ref, args.gop_len, args.vi_len);

            ret = mpi->control(ctx, MPP_ENC_SET_REF_CFG, ref);
            if (ret)
            {
                USER_LOG_ERROR("mpi control enc set ref cfg failed ret %d\n", ret);
                goto RET;
            }
            mpp_enc_ref_cfg_deinit(&ref);
        }
    }
    MppEncOSDPltCfg osd_plt_cfg;
    MppEncOSDPlt osd_plt;
    mpi_enc_gen_osd_plt(&osd_plt, 0);
    osd_plt_cfg.change = MPP_ENC_OSD_PLT_CFG_CHANGE_ALL;
    osd_plt_cfg.type = MPP_ENC_OSD_PLT_TYPE_USERDEF;
    osd_plt_cfg.plt = &osd_plt;
    ret = mpi->control(ctx, MPP_ENC_SET_OSD_PLT_CFG, &osd_plt_cfg);
    if (ret)
    {
        USER_LOG_ERROR("mpi control enc set osd plt failed ret %d\n", ret);
        goto RET;
    }
    USER_LOG_INFO("cfg setup done\n");
RET:
    return ret;
}

void Encoder::enque_input_buf(rga_buffer_t *rga_buf)
{

    mCurrent_Enc_buf = &mEnc_buf[mCurrentIndex];
    mCurrentIndex = (mCurrentIndex + 1) % 2;
    // USER_LOG_INFO("input_tid = %d", input_tid);确认可以运行到这儿
    encode_input();
    // {
    //     pthread_mutex_lock(&frame_lock);
    //     // 如果rga_buf不为空，将rga_buf中图像复制到frame缓冲区中
    //     if (rga_buf)
    //     {
    //         imcopy(*rga_buf, mCurrent_Enc_buf->rga_buf);
    //     }
    //     pthread_cond_signal(&frame_cond);
    //     pthread_mutex_unlock(&frame_lock);
    // }
    return;
}

rga_buffer_t *Encoder::deque_input_buf()
{
    return &mEnc_buf[mCurrentIndex].rga_buf;
}

void Encoder::deinit()
{
    if (ctx)
    {
        mpp_destroy(ctx);
        ctx = NULL;
    }
    if (cfg)
    {
        mpp_enc_cfg_deinit(cfg);
        cfg = NULL;
    }
    if (frame)
    {
        mpp_frame_deinit(&frame);
        frame = NULL;
    }
    if (output_tid >= 0)
    {
        pthread_cancel(output_tid);
    }

    for (int i = 0; i < 2; i++)
    {
        if (mEnc_buf[i].mpp_buf)
        {
            releasebuffer_handle(mEnc_buf[i].rga_buf_handle);
            mpp_buffer_put(mEnc_buf[i].mpp_buf);
        }
    }

    if (pkt_buf)
    {
        mpp_buffer_put(pkt_buf);
        pkt_buf = NULL;
    }
    if (buf_grp)
    {
        mpp_buffer_group_put(buf_grp);
        buf_grp = NULL;
    }
}

void *encode_input_thread(void *arg)
{
    // 提交frame,修改OSD信息等
    // 条件变量
    static int count = 0;
    Encoder *encodec = (Encoder *)arg;
    for (;;)
    {
        // 修改OSD信息
        if (encodec->osd_infos_ptr.cam_mode_ptr)
        {

            if (encodec->isCN)
            {
                Encoder::modify_osd_cn_mode(&encodec->osd_data, *encodec->osd_infos_ptr.cam_mode_ptr);
            }
            else
            {
                Encoder::modify_osd_en_mode(&encodec->osd_data, *encodec->osd_infos_ptr.cam_mode_ptr);
            }
            Encoder::modify_osd_gain_data(&encodec->osd_data, *encodec->osd_infos_ptr.gain_val_ptr);
            Encoder::modify_osd_count_data(&encodec->osd_data, *encodec->osd_infos_ptr.count_val_ptr, *encodec->osd_infos_ptr.count_period_ptr);
            Encoder::modify_osd_distance_data(&encodec->osd_data, *encodec->osd_infos_ptr.distance_val_ptr);
        }
        // 每10次循环，调用一次draw_time_osd
        if (count++ % 10 == 0)
        {
            Encoder::draw_time_osd(&encodec->osd_data);
        }

        pthread_testcancel();
        pthread_mutex_lock(&encodec->frame_lock);
        pthread_cond_wait(&encodec->frame_cond, &encodec->frame_lock);
        encodec->encode_input();
        pthread_mutex_unlock(&encodec->frame_lock);
    }
    return nullptr;
}

void *encode_output_thread(void *arg)
{
    Encoder *encodec = (Encoder *)arg;
    MppPacket &packet = encodec->packet;
    char *ptr;
    size_t len;
    RK_U32 eoi = 0;
    RK_U32 soi = 0;
    int header_len = 0;
    static uint8_t frameAudInfo[] = {0x00, 0x00, 0x00, 0x01, 0x09, 0x10};
    for (;;)
    {
        pthread_testcancel();
        pthread_mutex_lock(&encodec->packet_lock);
        while (!packet)
        {
            pthread_cond_wait(&encodec->packet_cond, &encodec->packet_lock);
        }

        do
        {
            if (encodec->mpi->encode_get_packet(encodec->ctx, &packet) < 0)
            {
                USER_LOG_ERROR("encode get packet faied\n");
                msleep(1);
                continue;
            }
            // write packet to file here
            ptr = (char *)mpp_packet_get_pos(packet);
            len = mpp_packet_get_length(packet);

            if (mpp_packet_is_partition(packet))
            {
                eoi = mpp_packet_is_eoi(packet);
                soi = mpp_packet_is_soi(packet);
            }
            else
            {
                eoi = 1;
                soi = 1;
            }
            if (len > 0)
            {
                if (eoi)
                {
                    memcpy(ptr + len, frameAudInfo, VIDEO_FRAME_AUD_LEN);
                    encodec->write_func(ptr, len + VIDEO_FRAME_AUD_LEN,eoi);
                }
                else
                {
                    encodec->write_func(ptr, len,eoi);
                }
                if (encodec->m_record_state == record_start && soi)
                {
                    if (ptr[4] == 0x67)
                    {
                        if (header_len == 0)
                        {
                            header_len += calculate_nal_len(ptr, len);              // sps
                            header_len += calculate_nal_len(ptr + header_len, len); // pps
                        }
                        if (ptr[header_len + 4] == 0x65)
                        {
                            USER_LOG_INFO("开始录像\n");
                            encodec->m_record_state = record_doing;
                            fwrite(ptr, 1, len, encodec->m_fp);
                        }
                    }
                }
                else if (encodec->m_record_state == record_doing || encodec->m_record_state == record_end)
                {
                    fwrite(ptr, 1, len, encodec->m_fp);
                }
            }
        } while (!eoi);
        eoi = 0;
        ptr = NULL;
        len = 0;
        mpp_packet_deinit(&packet);
        if (encodec->m_record_state == record_end)
        {
            encodec->m_record_state = record_idle;
            fclose(encodec->m_fp);
            USER_LOG_ERROR("停止录像\n");
            pthread_cond_signal(&encodec->transcode_cond);
        }
        pthread_mutex_unlock(&encodec->packet_lock);
    }
    return NULL;
}
void *transcode_thread(void *arg)
{
    Encoder *encodec = (Encoder *)arg;
    char outPath[256] = {0};
    char ffmpegCmdStr[512];
    FILE *fpCommand = NULL;
    for (;;)
    {
        pthread_mutex_lock(&encodec->packet_lock);
        pthread_cond_wait(&encodec->transcode_cond, &encodec->packet_lock);
        // sprintf(outPath, "/mnt/sdcard/IMG/%s", encodec->m_filepath + 17);
        strcpy(outPath, encodec->m_filepath);
        strcpy(outPath + strlen(outPath) - 4, "mp4");
        pthread_mutex_unlock(&encodec->packet_lock);
        snprintf(ffmpegCmdStr, 512,
                 "echo \"y\" | ffmpeg -i \"%s\" -codec copy -f \"mp4\" \"%s\" 1>/dev/null 2>&1", encodec->m_filepath, outPath);
        fpCommand = popen(ffmpegCmdStr, "r");
        if (fpCommand == NULL)
        {
            USER_LOG_ERROR("execute transcode command fail.");
        }
        pclose(fpCommand);
    }
}

MPP_RET Encoder::encode_input()
{
    MPP_RET ret = MPP_NOK;
    MppMeta meta = NULL;

    if (pthread_mutex_trylock(&packet_lock))
    {
        USER_LOG_ERROR("encode input lock failed\n");
        return ret;
    }
    // pthread_mutex_lock(&packet_lock);
    if (packet)
    {
        USER_LOG_ERROR("last packet has not processed!\n");
        pthread_mutex_unlock(&packet_lock);
        return ret;
    }
    if (!frame)
    {
        USER_LOG_ERROR("frame wasn\'t initialized correctly!\n");
        pthread_mutex_unlock(&packet_lock);
        return ret;
    }

    // mpp_frame_set_buffer(frame, mEnc_buf.mpp_buf);
    mpp_frame_set_buffer(frame, mCurrent_Enc_buf->mpp_buf);
    mpp_frame_set_eos(frame, eos_flag);
    meta = mpp_frame_get_meta(frame);
    mpp_packet_init_with_buffer(&packet, pkt_buf);
    mpp_packet_set_length(packet, 0);
    mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, packet);
    mpp_meta_set_ptr(meta, KEY_OSD_DATA, (void *)&osd_data);
    ret = mpi->encode_put_frame(ctx, frame);
    if (ret)
    {
        USER_LOG_ERROR("encode put frame failed\n");
        pthread_mutex_unlock(&packet_lock);
        return ret;
    }
    pthread_cond_signal(&packet_cond);
    pthread_mutex_unlock(&packet_lock);
    return ret;
}

MPP_RET Encoder::encode(HyBirdBuffer *input_CamBuffer)
{
    MPP_RET ret = MPP_OK;
    RK_U32 eoi = 1;
    MppMeta meta = NULL;

    if (!frame)
    {
        USER_LOG_ERROR("frame wasn\'t initialized correctly!\n");
        return MPP_NOK;
    }

    // mpp_frame_set_buffer(frame, mEnc_buf.mpp_buf);
    mpp_frame_set_buffer(frame, mCurrent_Enc_buf->mpp_buf);
    mpp_frame_set_eos(frame, eos_flag);
    meta = mpp_frame_get_meta(frame);
    mpp_packet_init_with_buffer(&packet, pkt_buf);
    mpp_packet_set_length(packet, 0);
    mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, packet);
    ret = mpi->encode_put_frame(ctx, frame);
    if (ret)
    {
        USER_LOG_ERROR("encode put frame failed\n");
        goto RET;
    }

    do
    {
        ret = mpi->encode_get_packet(ctx, &packet);
        if (ret)
        {
            USER_LOG_ERROR("encode get packet faied\n");
            goto RET;
        }
        if (packet)
        {
            // write packet to file here
            void *ptr = mpp_packet_get_pos(packet);
            size_t len = mpp_packet_get_length(packet);
            // write_func(ptr, len, *((uint8_t *)ptr + 4));
        }
        if (mpp_packet_is_partition(packet))
        {
            eoi = mpp_packet_is_eoi(packet);
            // LOGI("成功进入\n");
        }
    } while (!eoi);
    // write_func(frameAudInfo,VIDEO_FRAME_AUD_LEN);
RET:
    if (packet)
    {
        mpp_packet_deinit(&packet);
    }
    return ret;
}
size_t Encoder::calc_buf_size(MppFrameFormat fmt)
{
    size_t size;
    RK_S32 hor_stride = mpi_enc_width_default_stride(args.width, fmt);
    switch (fmt & MPP_FRAME_FMT_MASK)
    {
    case MPP_FMT_YUV420SP:
    case MPP_FMT_YUV420P:
    {
        size = MPP_ALIGN(hor_stride, 64) * MPP_ALIGN(args.ver_stride, 64) * 3 / 2;
    }
    break;

    case MPP_FMT_YUV422_YUYV:
    case MPP_FMT_YUV422_YVYU:
    case MPP_FMT_YUV422_UYVY:
    case MPP_FMT_YUV422_VYUY:
    case MPP_FMT_YUV422P:
    case MPP_FMT_YUV422SP:
    {
        size = MPP_ALIGN(hor_stride, 64) * MPP_ALIGN(args.ver_stride, 64) * 2;
    }
    break;
    case MPP_FMT_RGB444:
    case MPP_FMT_BGR444:
    case MPP_FMT_RGB555:
    case MPP_FMT_BGR555:
    case MPP_FMT_RGB565:
    case MPP_FMT_BGR565:
    case MPP_FMT_RGB888:
    case MPP_FMT_BGR888:
    case MPP_FMT_RGB101010:
    case MPP_FMT_BGR101010:
    case MPP_FMT_ARGB8888:
    case MPP_FMT_ABGR8888:
    case MPP_FMT_BGRA8888:
    case MPP_FMT_RGBA8888:
    {
        size = MPP_ALIGN(hor_stride, 64) * MPP_ALIGN(args.ver_stride, 64);
    }
    break;

    default:
    {
        size = MPP_ALIGN(hor_stride, 64) * MPP_ALIGN(args.ver_stride, 64) * 4;
    }
    break;
    }
    return size;
}

bool Encoder::set_output_func(pOutput_func output_func)
{
    int ret;
    if (write_func)
        return false;
    write_func = output_func;
    ret = pthread_create(&output_tid, NULL, encode_output_thread, this);
    if (ret)
    {
        USER_LOG_ERROR("create encode output thread failed\n");
        return false;
    }
    pthread_detach(output_tid);
    ret = pthread_create(&transcode_tid, NULL, transcode_thread, this);
    if (ret)
    {
        USER_LOG_ERROR("create encode transcode thread failed\n");
        return false;
    }
    pthread_detach(transcode_tid);
    USER_LOG_INFO("set output func success\n");
    return true;
}

bool Encoder::start_record(char *save_path)
{
    if (m_record_state != record_idle)
        return false;
    m_fp = fopen(save_path, "w+b");
    if (m_fp == NULL)
        return false;
    memcpy(m_filepath, save_path, 256);
    pthread_mutex_lock(&packet_lock);
    mpi->control(ctx, MPP_ENC_SET_IDR_FRAME, NULL);
    m_record_state = record_start;
    pthread_mutex_unlock(&packet_lock);
    return true;
}
void Encoder::stop_record()
{
    if (m_record_state == record_doing)
    {
        pthread_mutex_lock(&packet_lock);
        m_record_state = record_end;
        pthread_mutex_unlock(&packet_lock);
    }
}

RK_S32 os_get_env_u32(const char *name, RK_U32 *value, RK_U32 default_value)
{
    char *ptr = getenv(name);
    if (NULL == ptr)
    {
        *value = default_value;
    }
    else
    {
        char *endptr;
        int base = (ptr[0] == '0' && ptr[1] == 'x') ? (16) : (10);
        errno = 0;
        *value = strtoul(ptr, &endptr, base);
        if (errno || (ptr == endptr))
        {
            errno = 0;
            *value = default_value;
        }
    }
    return 0;
}

static int os_malloc(void **memptr, size_t alignment, size_t size)
{
    return posix_memalign(memptr, alignment, size);
}

void *mpp_osal_malloc(size_t size)
{
    size_t size_align = MEM_ALIGNED(size);
    size_t size_real = size_align;
    void *ptr;

    os_malloc(&ptr, MEM_ALIGN, size_real);
    return ptr;
}

void *mpp_osal_calloc(size_t size)
{
    void *ptr = mpp_osal_malloc(size);
    if (ptr)
        memset(ptr, 0, size);
    return ptr;
}

void Encoder::modify_osd_cn_mode(MppEncOSDData *osd_data, unsigned int mode)
{
    MppBuffer buf = osd_data->buf;
    MppEncOSDRegion *region = osd_data->region;
    RK_U32 mb_w = region->num_mb_x;
    unsigned int index = 3 * 32;
    char *ptr = (char *)mpp_buffer_get_ptr(buf);
    ptr += mb_w * 2 * 256;
    int i = 0;
    switch (mode)
    {
    case VL_Mode:
        for (i = 0; i < 2; i++)
        {
            fill_one_char(ptr, (unsigned char (*)[32]) & 可见[i * 4][0], index + i * 32, mb_w / 2);
        }
        break;
    // case UV_Mode:
    //     fill_one_char(ptr, (unsigned char(*)[32]) & 紫红光电[0 * 4][0], index + i++ * 32, mb_w / 2);
    //     fill_one_char(ptr, (unsigned char(*)[32]) & 紫红光电[4 * 4][0], index + i * 32, mb_w / 2);
    //     break;
    case VL_UV_blendMode:
        for (i = 0; i < 2; i++)
        {
            fill_one_char(ptr, (unsigned char (*)[32]) & 融合[i * 4][0], index + i * 32, mb_w / 2);
        }
        break;
    default:
        break;
    }
}

void Encoder::modify_osd_en_mode(MppEncOSDData *osd_data, unsigned int mode)
{
    MppBuffer buf = osd_data->buf;
    MppEncOSDRegion *region = osd_data->region;
    RK_U32 mb_w = region->num_mb_x;
    unsigned int index = 3 * 32;
    char *ptr = (char *)mpp_buffer_get_ptr(buf);
    ptr += mb_w * 2 * 256;
    int i = 0;
    switch (mode)
    {
    case VL_Mode:
        fill_string(ptr, "vl", index, mb_w / 2);
        for (i = 0; i < 2; i++) // 清除上一帧残余数据
        {
            fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 11][0], 4 * 32 + i * 16, mb_w / 2);
        }
        break;
    // case UV_Mode:
    //     fill_string(ptr, "uv", index, mb_w / 2);
    //     for (i = 0; i < 2; i++) // 清除上一帧残余数据
    //     {
    //         fill_half_char(ptr, (unsigned char(*)[32]) & NUM[2 * 11][0], 4 * 32 + i * 16, mb_w / 2);
    //     }
    //     break;
    case VL_UV_blendMode:
        fill_string(ptr, "fuse", index, mb_w / 2);
        break;
    default:
        break;
    }
}
void Encoder::modify_osd_gain_data(MppEncOSDData *osd_data, unsigned int value)
{
    MppBuffer buf = osd_data->buf;
    MppEncOSDRegion *region = osd_data->region;
    RK_U32 mb_w = region->num_mb_x;
    char *ptr = (char *)mpp_buffer_get_ptr(buf);
    ptr += mb_w * 2 * 256 * 2; // 光子计数行
    if (value < 100)
    {
        if (value > 9)
        {
            fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 11][0], 5 * 16, mb_w / 2);
            fill_int_string(ptr, value, 6 * 16, mb_w / 2);
        }
        else
        {
            fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 11][0], 5 * 16, mb_w / 2);
            fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 11][0], 6 * 16, mb_w / 2);
            fill_int_string(ptr, value, 7 * 16, mb_w / 2);
        }
    }
    else
        fill_int_string(ptr, value, 5 * 16, mb_w / 2);
}
void Encoder::modify_osd_count_data(MppEncOSDData *osd_data, unsigned int value, bool period)
{
    MppBuffer buf = osd_data->buf;
    MppEncOSDRegion *region = osd_data->region;
    RK_U32 mb_w = region->num_mb_x;
    static int count_unint = 0; //
    int c;
    unsigned int index = 3 * 32;
    char *ptr = (char *)mpp_buffer_get_ptr(buf);
    ptr += mb_w * 2 * 256 * 3; // 光子计数行
    if (value > 1000000)
        return;
    c = fill_int_string(ptr, value, index, mb_w / 2);
    index += 16 * c;
    if (!period)
    {
        fill_one_char(ptr, (unsigned char (*)[32])秒计数, index, mb_w / 2);
    }
    else
    {
        fill_one_char(ptr, (unsigned char (*)[32])分计数, index, mb_w / 2);
    }
    index += 32;
    for (int i = c + 2; i < 10; i++, index += 16) // 清除上一帧残余数据
    {
        fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 11][0], index, mb_w / 2);
    }
}
void Encoder::modify_osd_distance_data(MppEncOSDData *osd_data, unsigned int value)
{
    MppBuffer buf = osd_data->buf;
    MppEncOSDRegion *region = osd_data->region;
    RK_U32 mb_w = region->num_mb_x;
    char *ptr = (char *)mpp_buffer_get_ptr(buf);
    unsigned int index = 3 * 32;
    int c;

    ptr += mb_w * 2 * 256 * 4;
    c = fill_float_string(ptr, value, index, mb_w / 2);
    index += 16 * c;
    for (int i = c; i < 5; i++, index += 16)
    {
        fill_half_char(ptr, (unsigned char (*)[32]) & NUM[2 * 11][0], index, mb_w / 2);
    }
}
