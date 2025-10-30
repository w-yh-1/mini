#pragma once
#include <cstddef>
#include "camera_source.h"
#include "dma_buf.h"
#include "encoder.h"


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
    RK_U32 mCamWidth = 3840;
    RK_U32 mCamHeight = 2160;
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
    float zoom_ratio = 1.0;           // 缩放比例 

public:
    camera_process(/* args */);
    ~camera_process(); 
    bool init(pOutput_func output_func);
    void update_uv_rect(int distance);
    bool load_correction_factors(const char *config_file);
    bool save_correction_factors(const char *config_file);
    void update_correction_factors(int h_dist, int h_ang, int v_dist, int v_ang, float scale);
    im_rect mUvSrcRect = {160, 90, 960, 540};
    im_rect mUvDstRect = {0, 0, 0, 0};
};

extern "C"
{
    bool CameraMediaStartService(void *output_func);
    int pause_cam_stream();
    int resume_cam_stream();
}

camera_process::camera_process(/* args */)
{
}

camera_process::~camera_process()
{
}
