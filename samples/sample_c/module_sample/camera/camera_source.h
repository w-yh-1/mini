/*
 * Copyright 2015 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __CAMERA_SOURCE_H__
#define __CAMERA_SOURCE_H__

#include "im2d.hpp"
#include "rga.h"
#include "mpp_frame.h"
#define IQ_PATH "/etc/iqfiles"

#define CAMERA_VIDEO_FPS 20
#define CAMERA_FRAME_INTERVAL_US (1000000 / CAMERA_VIDEO_FPS)

extern void *mem_osal_calloc(size_t size);
#define MEM_ALIGN 32
#define MEM_ALIGN_MASK (MEM_ALIGN - 1)
#define MEM_ALIGNED(x) (((x) + MEM_ALIGN) & (~MEM_ALIGN_MASK))
#define MEM_CALLOC(type, count) (type *)mem_osal_calloc(sizeof(type) * (count))
#define mpp_free(ptr) \
    free(ptr)
#define MEM_FREE(ptr)      \
    do                     \
    {                      \
        if (ptr)           \
            mpp_free(ptr); \
        ptr = NULL;        \
    } while (0)
#define STRIDE_ALIGN(x, a) (((x) + (a)-1) & ~((a)-1))

typedef struct CamSource CamSource;

// typedef struct HyBirdBuffer_t
// {
//     MppBuffer mpp_buf;
//     void *ptr{NULL};
//     int fd{-1};
//     rga_buffer_handle_t rga_buf_handle{0};
//     rga_buffer_t rga_buf{0};
// } HyBirdBuffer;

struct CamFrame
{
    void *start;
    int bytesused;
    size_t length;
    RK_S32 export_fd;
    // RK_S32 sequence;
    rga_buffer_handle_t rga_buf_handle;
    rga_buffer_t rga_buf;
    MppBuffer mpp_buf;
};
typedef struct CamFrame CamFrame;
typedef struct IspCtx_ IspCtx;
extern "C"
{
    IspCtx *isp_init(const char *device, uint32_t width, uint32_t height);
    int isp_deinit(IspCtx *ctx);
    // Create a new context to capture frames from <fname>. Returns NULL on error.
    CamSource *isp_camera_source_init(const char *device, RK_U32 bufcnt, RK_U32 width, RK_U32 height, MppFrameFormat fmt);
    CamSource *camera_source_init(const char *device, RK_U32 bufcnt, RK_U32 width, RK_U32 height, MppFrameFormat fmt);
    CamSource *mjpg_camera_source_init(const char *device, RK_U32 bufcnt, RK_U32 width, RK_U32 height);
    // Stop capturing and free a context.
    MPP_RET camera_source_deinit(CamSource *ctx);
    MPP_RET mjpg_camera_source_deinit(CamSource *ctx);
    // Returns the next captured frame and its meta-data.
    RK_S32 camera_source_get_frame(CamSource *ctx);
    CamFrame* mjpg_camera_source_get_frame(CamSource *ctx);
    // Tells the kernel it's OK to overwrite a frame captured
    MPP_RET camera_source_put_frame(CamSource *ctx, RK_S32 idx);
    rga_buffer_t *camera_frame_to_RgaBuffer(CamSource *ctx, RK_S32 idx);
    MppBuffer camera_frame_to_Mpp_buf(CamSource *ctx, RK_S32 idx);
    int get_frame_size(CamSource *ctx, RK_S32 idx);
    void *camera_frame_to_buf(CamSource *ctx, RK_S32 idx);
    void zoom_in(CamSource *ctx);  // 拉近
    void zoom_out(CamSource *ctx); // 拉远
    void zoom_stop(CamSource *ctx);
    void set_zoom(CamSource *ctx, int ratio);
    void auto_focus(CamSource *ctx);
    void focus_near(CamSource *ctx);
    void focus_far(CamSource *ctx);
    RgaSURF_FORMAT change_MppFmt2RkFmt(MppFrameFormat fmt);
    void set_focus_automode(const CamSource *ctx);
    void set_focus_manualmode(const CamSource *ctx);
    void sample_set_focus_semiautomode(const CamSource *ctx);
    void oneshot_focus(const CamSource *ctx);
    void oneshot_focus_auto();
}

#endif 
