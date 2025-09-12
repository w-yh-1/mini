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
    const char *mUvCam = "/dev/video21";
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
    int mEncodeHeight = 1088;
    RK_U32 mBufcnt = 4;
    DMA_Buffer mTempBuf;

public:
    camera_process(/* args */);
    ~camera_process();
    bool init(pOutput_func output_func);

    im_rect mUvSrcRect = {160, 90, 960, 540};
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
