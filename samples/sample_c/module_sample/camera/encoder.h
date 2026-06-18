#ifndef __ENCODER_H__
#define __ENCODER_H__
#include <stdio.h>
#include "rockchip/rk_mpi.h"
#include "rockchip/rk_venc_rc.h"
#include "rockchip/mpp_rc_api.h"
#include "camera_source.h"
#include "dji_logger.h"
#include <pthread.h>
#include <vector>
#include "free_type_osd.h"
#define CODEC_TYPE MPP_VIDEO_CodingAVC
#define MPP_ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))
#define mpp_env_get_u32(name, value, default_value) os_get_env_u32(name, value, default_value)
enum CamMode
{
    VL_Mode = 0,
    // UV_Mode,
    IR_Mode,
    VL_UV_blendMode,
    // IR_UV_blendMode,
    VL_IR_UV_blendMode,
    UNINITIALIZED
};
typedef struct osd_infos_t
{
    unsigned int *cam_mode_ptr;
    unsigned int *gain_val_ptr;
    unsigned int *count_val_ptr;
    unsigned int *first_val_ptr;
    unsigned int *second_val_ptr;
    unsigned int *third_val_ptr;
    unsigned int *fourth_val_ptr;
    unsigned int *count_period_ptr;
    unsigned int *distance_val_ptr;
} OsdInfos;

typedef struct HyBirdBuffer_t
{
    MppBuffer mpp_buf;
    void *ptr{NULL};
    int fd{-1};
    rga_buffer_handle_t rga_buf_handle{0};
    rga_buffer_t rga_buf{0};
} HyBirdBuffer;

typedef struct EncodeArgs_t
{
    RK_U32 width;
    RK_U32 height;
    RK_U32 hor_stride;
    RK_U32 ver_stride;
    RK_S32 fps_in_flex;
    RK_S32 fps_in_den;
    RK_S32 fps_in_num;
    RK_S32 fps_out_flex;
    RK_S32 fps_out_den;
    RK_S32 fps_out_num;

    RK_S32 bps;
    RK_S32 bps_max;
    RK_S32 bps_min;

    size_t frame_size;
    size_t header_size;
    MppFrameFormat fmt;
    MppCodingType type;
    MppEncRcMode rc_mode;
    RK_S32 qp_init;

    GopMode gop_mode;
    RK_S32 gop_len;
    RK_S32 vi_len;

    MppEncSeiMode sei_mode;
    MppEncHeaderMode header_mode;

    RK_U32 split_mode;
    RK_U32 split_arg;
    RK_U32 split_out;

    RK_U32 refresh_en;
    MppEncRcRefreshMode refresh_mode;
    RK_U32 refresh_num;

    RK_U32 frm_eos;
} EncodeArgs;

enum record_state
{
    record_start,
    record_doing,
    record_end,
    record_idle
};

typedef void (*pOutput_func)(void *, size_t, bool);

class Encoder
{
private:
    /* data */
    char m_filepath[256];
    bool bIsRunning = 0;
    int mCurrentIndex = 0;
    FILE *m_fp;
    MppBufferGroup buf_grp;
    MppPacket packet = NULL;
    MppBuffer pkt_buf;
    MppEncCfg cfg;
    MppCtx ctx;
    MppApi *mpi;
    EncodeArgs args;
    MppPollType timeout = MPP_POLL_NON_BLOCK;
    pOutput_func write_func = NULL;
    size_t ext_buf_size = 0;
    MppFrameFormat ext_buf_fmt;
    pthread_t input_tid = -1;
    pthread_t output_tid = -1;
    pthread_t transcode_tid = -1;
    RK_U32 eos_flag = 0;
    pthread_mutex_t frame_lock = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t frame_cond = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t packet_lock = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t packet_cond = PTHREAD_COND_INITIALIZER;
    pthread_cond_t transcode_cond = PTHREAD_COND_INITIALIZER;
    record_state m_record_state;
    std::vector<uint8_t> m_last_vps;
    std::vector<uint8_t> m_last_sps;
    std::vector<uint8_t> m_last_pps;

    //function
    size_t calc_buf_size(MppFrameFormat fmt);
    bool set_output_func(pOutput_func output_func);

public:
    OsdInfos osd_infos_ptr = {};
    Encoder(RK_U32 width, RK_U32 height, MppFrameFormat frame_fmt = MPP_FMT_YUV420SP, MppCodingType type = MPP_VIDEO_CodingAVC, MppEncRcMode rc_mode = MPP_ENC_RC_MODE_CBR);
    ~Encoder();
    MPP_RET encode(HyBirdBuffer *input_CamBuffer);
    MPP_RET init(pOutput_func output_func);
    MPP_RET cfg_setup();
    MPP_RET encode_input();
    // int

    void enque_input_buf(rga_buffer_t *rga_buf = NULL);
    rga_buffer_t *deque_input_buf();
    friend void *encode_input_thread(void *arg);
    friend void *encode_output_thread(void *arg);
    friend void *transcode_thread(void *arg);
    void deinit();
    void stop_record();
    void pause_record();
    bool start_record(char *save_path);
    HyBirdBuffer mEnc_buf[2];
    HyBirdBuffer *mCurrent_Enc_buf;
    MppFrame frame = NULL;
    MppEncOSDData osd_data;
    rga_buffer_t *input_buf;
    static bool isCN;
    static FreeTypeOSD s_ftosd;
    static bool s_ftosd_inited;

    static bool init_ftosd();
    
    static MPP_RET gen_osd_data(MppEncOSDData *osd_data, MppBufferGroup group);
    static void switch_language(MppEncOSDData *osd_data, bool is_CN);
    static void modify_osd_en_flaw_info(MppEncOSDData *osd_data, int level);
    static void modify_osd_cn_flaw_info(MppEncOSDData *osd_data, int level);
    static void modify_osd_gain_data(MppEncOSDData *osd_data, unsigned int value);
    static void modify_osd_count_data(MppEncOSDData *osd_data, unsigned int value, bool period);
    static void draw_cn_osd(char *ptr, int DW_per_line = 8);
    static void draw_en_osd(char *ptr, int DW_per_line = 8);
    static void draw_time_osd(MppEncOSDData *osd_data);
    static void (*modify_osd_flaw_info)(MppEncOSDData *osd_data, int level);
};

#endif
