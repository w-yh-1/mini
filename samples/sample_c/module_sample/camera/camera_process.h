#ifndef CAMERA_PROCESS_H
#define CAMERA_PROCESS_H
#include <cstddef>
#include "camera_source.h"
#include "dma_buf.h"
#include "encoder.h"
#include <string>
#include <atomic>
#include <pthread.h>
#include <sys/eventfd.h>
#include <time.h>

// 默认快照保存目录，可在编译时覆盖
#ifndef SNAPSHOT_DIR
#define SNAPSHOT_DIR "/mnt/sdcard/IMG/"
#endif


class camera_process
{
private:
    /* data */
    pthread_t mCamThread;
    static void *camera_task(void *arg);
    CamSource *mCamSource = NULL;
    CamSource *mUvCamSource = NULL;
    const char *mCam = "/dev/video11";
    const char *mUvCam = "/dev/video20";
    MppFrameFormat mCamFmt = MPP_FMT_YUV420SP;
    MppFrameFormat mUvCamFmt = MPP_FMT_BGR888;
    int mBufFmt = RK_FORMAT_BGRA_8888;
    RK_U32 mCamWidth = 4208;
    RK_U32 mCamHeight = 3120;
    RK_U32 mUvCamWidth = 1280;
    RK_U32 mUvCamHeight = 720;
    Encoder *mEncoder = NULL;
    MppFrameFormat mEncFmt = MPP_FMT_YUV420SP;
    int mEncodeWidth = 1920;
    int mEncodeHeight = 1080;
    RK_U32 mBufcnt = 4;
    DMA_Buffer mTempBuf;
    int horzDistCorrectionFactor = 0; // 水平距离矫正系数
    int horzAngCorrectionFactor = 0;  // 水平角度矫正系数
    int vertDistCorrectionFactor = 0; // 垂直距离矫正系数
    int vertAngCorrectionFactor = 0;  // 垂直角度矫正系数
    float scaleFactor = 1.0;          // 缩放系数
    float zoom_ratio = 1.0;           // 变倍

    // --- 拍照/快照子系统 ---
    // 双缓冲 DMA 缓冲，用于按需通过 RGA 从 mTempBuf 转换为 RGB888 保存 PNG
    DMA_Buffer mSnapBuf[2];
    // 双缓冲状态：0-Free,1-Filling,2-Ready,3-Saving（原子）
    std::atomic<uint8_t> mSnapState[2];
    // 每个缓冲对应的目标文件路径（定长，避免跨线程动态分配）
    char mSnapPath[2][512] = {{0}};
    // 每个缓冲对应的拍照时间戳，供文件名/OSD/PNG元数据共用
    time_t mSnapTimestamp[2] = {0, 0};
    // 后台保存线程
    pthread_t mSnapThread = 0;
    // 后台激光测距线程
    pthread_t mLidarThread = 0;
    std::atomic<bool> mSnapThreadRunning{false};
    int mSnapEventFd = -1; // eventfd 用于唤醒保存线程
    // 待抓取请求（只允许占位一个）
    std::atomic<bool> mPendingSnap{false};
    char mPendingPathBuf[512] = {0};
    time_t mPendingSnapTimestamp = 0;

public:
    camera_process(/* args */);
    ~camera_process(); 
    bool init(pOutput_func output_func);
    void update_rects(int distance);
    bool load_correction_factors(const char *config_file);
    bool save_correction_factors(const char *config_file);
    void update_correction_factors(int h_dist, int h_ang, int v_dist, int v_ang, float scale);
    // 新增: 单项 set/get 接口
    // 水平距离矫正系数
    void set_horz_dist_correction(int v); int get_horz_dist_correction() const;
    // 水平角度矫正系数
    void set_horz_ang_correction(int v); int get_horz_ang_correction() const;
    // 垂直距离矫正系数
    void set_vert_dist_correction(int v); int get_vert_dist_correction() const;
    // 垂直角度矫正系数
    void set_vert_ang_correction(int v); int get_vert_ang_correction() const;
    // 缩放系数
    void set_scale_factor(float v); float get_scale_factor() const;
    // zoom ratio
    void set_zoom_ratio(float v); float get_zoom_ratio() const;
    // 统一 OSD 语言切换（内部同时作用录像与拍照）
    void apply_language(bool use_cn);
    im_rect mUvSrcRect = {160, 90, 960, 540};
    im_rect mUvDstRect = {0, 0, 0, 0};
    im_rect mVlSrcRect = {0, 0, 0, 0};

    // 触发一次拍照：成功返回0；忙(队列/缓冲占满)返回-1
    int request_snapshot(const char* path);
    // 自动生成文件名并拍照：使用 SNAPSHOT_DIR 和本地时间，若重名则追加序号
    int request_snapshot_auto();
    int request_snapshot_with_time(const char *path, time_t snapshot_time);

    // 开始录像
    int start_recording();
    // 停止录像
    void stop_recording();

    // 快照后台线程入口（静态成员，便于访问私有成员）
    static void* snapshot_worker(void* arg);
    // 激光测距线程入口（静态成员，便于访问私有成员）
    static void* laser_ranging_worker(void* arg);
};

extern "C"
{
    bool CameraMediaStartService(void *output_func);
    int pause_cam_stream();
    int resume_cam_stream();
    int set_uv_color(uint value);
    int set_uv_gain(uint value);
    int set_uv_gain_raw(uint value);
    // C 接口：触发拍照
    int take_snapshot(const char* path);
    // C 接口：自动命名触发拍照
    int take_snapshot_auto();
    int start_recording();
    void stop_recording();
    // 统一语言切换：录像 OSD + 拍照 OSD
    void language_switch(bool use_cn);
    // C 接口：单项矫正参数 set/get 封装
    int camera_set_horz_dist_correction(int v);
    int camera_get_horz_dist_correction();
    int camera_set_horz_ang_correction(int v);
    int camera_get_horz_ang_correction();
    int camera_set_vert_dist_correction(int v);
    int camera_get_vert_dist_correction();
    int camera_set_vert_ang_correction(int v);
    int camera_get_vert_ang_correction();
    int camera_set_scale_factor(float v);
    float camera_get_scale_factor();
    bool camera_correction_factor_save();
    bool camera_correction_factor_load();
    void camera_set_zoom_ratio(float v);
    float camera_get_zoom_ratio();
}

camera_process::camera_process(/* args */)
{
}

camera_process::~camera_process()
{
}

#endif // CAMERA_PROCESS_H