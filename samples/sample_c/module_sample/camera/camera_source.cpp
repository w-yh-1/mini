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

#define MODULE_TAG "camera_source"

#include <rk_mpi.h>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <sys/select.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "camera_source.h"
// #include "utils.h"







struct CamSource
{
    RK_S32 fd; // Device handle
    unsigned int buf_len;
    RK_U32 bufcnt; // # of buffers
    unsigned int mem_type;
    enum v4l2_buf_type type;
    MppFrameFormat fmt;
    CamFrame fbuf[10]; // frame buffers
    bool autoFocus = true;
};
class decoder
{
#define ESC_START "\033["
#define ESC_END "\033[0m"
#define COLOR_GREEN "32;40;1m"
#define COLOR_RED "31;40;1m"
#define MPP_DBG(format, args...) (printf( ESC_START COLOR_GREEN "[MPP DBG]-[%s]-[%05d]:" format ESC_END, __FUNCTION__, (int)__LINE__, ##args))
#define MPP_ERR(format, args...) (printf(ESC_START COLOR_RED "[MPP ERR]-[%s]-[%05d]:" format ESC_END, __FUNCTION__, (int)__LINE__, ##args))
#define TYPE MPP_VIDEO_CodingMJPEG
#define FMT MPP_FMT_RGB888;

#define BUF_COUNT 4
private:
    uint32_t m_width;
    uint32_t m_height;
    MppApi *m_mpi;
    MppCtx m_ctx;
    MppFrame m_frame;
    MppDecCfg m_cfg;
    MppBufferGroup m_buf_group;
    MppBuffer m_frm_buf;
    MppBuffer m_pkt_bufs[BUF_COUNT];
    void get_buf_array(MppBuffer buf_array[], int cnt);


public:
    decoder(uint32_t width, uint32_t height);
    ~decoder();
    void decode(int index, int bytesused);
    int buf_fd_array[BUF_COUNT];
    CamFrame frm_buf;
};

decoder::decoder(uint32_t width, uint32_t height) : m_width(width), m_height(height), m_frame(NULL), m_cfg(NULL), m_buf_group(NULL), m_frm_buf(NULL)
{
    uint32_t hor_stride = STRIDE_ALIGN(width, 16);
    uint32_t ver_stride = STRIDE_ALIGN(height, 16);
    MppFrameFormat fmt = FMT;
    MPP_RET ret = MPP_OK;
    mpp_create(&m_ctx, &m_mpi);
    mpp_init(m_ctx, MPP_CTX_DEC, TYPE);
    mpp_dec_cfg_init(&m_cfg);
    m_mpi->control(m_ctx, MPP_DEC_GET_CFG, m_cfg);
    // mpp_dec_cfg_set_u32(m_cfg, "base:split_parse", 1); // 开启自动分包
    m_mpi->control(m_ctx, MPP_DEC_SET_CFG, m_cfg);
    mpp_buffer_group_get_internal(&m_buf_group, MPP_BUFFER_TYPE_DRM);
    get_buf_array(m_pkt_bufs, BUF_COUNT);
    mpp_frame_init(&m_frame);
    mpp_buffer_get(m_buf_group, &frm_buf.mpp_buf, hor_stride * ver_stride * 3);
    mpp_frame_set_buffer(m_frame, frm_buf.mpp_buf);
    m_mpi->control(m_ctx, MPP_DEC_SET_OUTPUT_FORMAT, &fmt);
    frm_buf.start = mpp_buffer_get_ptr(frm_buf.mpp_buf);
    frm_buf.export_fd = mpp_buffer_get_fd(frm_buf.mpp_buf);
    frm_buf.rga_buf_handle = importbuffer_fd(frm_buf.export_fd, hor_stride * ver_stride * 3);
    frm_buf.rga_buf = wrapbuffer_handle(frm_buf.rga_buf_handle, m_width, m_height, RK_FORMAT_BGR_888);
}

decoder::~decoder()
{
    if (m_frame)
    {
        mpp_frame_deinit(&m_frame);
        m_frame = NULL;
    }
    if (m_ctx)
    {
        mpp_destroy(m_ctx);
        m_ctx = NULL;
    }
    if (m_frm_buf)
    {
        releasebuffer_handle(frm_buf.rga_buf_handle);
        mpp_buffer_put(m_frm_buf);
        m_frm_buf = NULL;
    }
    if (m_buf_group)
    {
        mpp_buffer_group_put(m_buf_group);
        m_buf_group = NULL;
    }
    if (m_cfg)
    {
        mpp_dec_cfg_init(&m_cfg);
        m_cfg = NULL;
    }
}

void decoder::decode(int index, int bytesused)
{

    MppTask task = NULL;
    MppMeta meta = NULL;
    MppPacket packet = NULL;
    MPP_RET ret = MPP_OK;
    uint16_t times = 2;
    if (index < 0 || index >= BUF_COUNT)
        return;

    mpp_packet_init_with_buffer(&packet, m_pkt_bufs[index]);
    mpp_packet_set_length(packet, bytesused);
    ret = m_mpi->poll(m_ctx, MPP_PORT_INPUT, MPP_POLL_BLOCK);

    if (ret)
    {
        MPP_ERR("mpp input poll failed\n");
        return;
    }

    ret = m_mpi->dequeue(m_ctx, MPP_PORT_INPUT, &task); /* input queue */
    if (ret)
    {
        MPP_ERR("mpp task input dequeue failed\n");
        return;
    }
    mpp_task_meta_set_packet(task, KEY_INPUT_PACKET, packet);
    mpp_task_meta_set_frame(task, KEY_OUTPUT_FRAME, m_frame);

    ret = m_mpi->enqueue(m_ctx, MPP_PORT_INPUT, task); /* input queue */
    if (ret)
    {
        MPP_ERR("mpp task input enqueue failed\n");
        return;
    }

    /* poll and wait here */
    ret = m_mpi->poll(m_ctx, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);
    if (ret)
    {
        MPP_ERR("mpp output poll failed\n");
        return;
    }

    ret = m_mpi->dequeue(m_ctx, MPP_PORT_OUTPUT, &task); /* output queue */
    if (ret)
    {
        MPP_ERR("mpp task output dequeue failed\n");
        return;
    }

    if (task)
    {
        ret = m_mpi->enqueue(m_ctx, MPP_PORT_OUTPUT, task);
        if (ret)
        {
            MPP_ERR("mpp task output enqueue failed\n");
        }
    }
    mpp_packet_deinit(&packet);
}


void decoder::get_buf_array(MppBuffer buf_array[], int cnt)
{
    for (int i = 0; i < cnt; i++)
    {
        mpp_buffer_get(m_buf_group, buf_array + i, m_width * m_height * 2);
        buf_fd_array[i] = mpp_buffer_get_fd(buf_array[i]);
    }
}

static decoder *decoder_ptr = NULL;
static RK_U32 V4L2_yuv_cfg[MPP_FMT_YUV_BUTT] = {
    V4L2_PIX_FMT_NV12,
    0,
    V4L2_PIX_FMT_NV16,
    0,
    V4L2_PIX_FMT_YVU420,
    V4L2_PIX_FMT_NV21,
    V4L2_PIX_FMT_YUV422P,
    V4L2_PIX_FMT_NV61,
    V4L2_PIX_FMT_YUYV,
    V4L2_PIX_FMT_YVYU,
    V4L2_PIX_FMT_UYVY,
    V4L2_PIX_FMT_VYUY,
    V4L2_PIX_FMT_GREY,
    0,
    0,
    0,
};

static RK_U32 V4L2_RGB_cfg[MPP_FMT_RGB_BUTT - MPP_FRAME_FMT_RGB] = {
    V4L2_PIX_FMT_RGB565,
    0,
    V4L2_PIX_FMT_RGB555,
    0,
    V4L2_PIX_FMT_RGB444,
    0,
    V4L2_PIX_FMT_RGB24,
    V4L2_PIX_FMT_BGR24,
    0,
    0,
    V4L2_PIX_FMT_RGB32,
    V4L2_PIX_FMT_BGR32,
    0,
    0,
};

#define FMT_NUM_PLANES 1

// Wrap ioctl() to spin on EINTR
static RK_S32 camera_source_ioctl(RK_S32 fd, RK_S32 req, void *arg)
{
    struct timespec poll_time;
    RK_S32 ret;

    while ((ret = ioctl(fd, req, arg)))
    {
        if (ret == -1 && (EINTR != errno && EAGAIN != errno))
        {
            // mpp_err("ret = %d, errno %d", ret, errno);
            break;
        }
        // 2 milliseconds
        poll_time.tv_sec = 0;
        poll_time.tv_nsec = 2000000;
        nanosleep(&poll_time, NULL);
    }

    return ret;
}

static int v4l2_set_format(CamSource *ctx, RK_U32 width, RK_U32 height, MppFrameFormat format)
{
    struct v4l2_format vfmt = {0};
    struct v4l2_capability cap;

    // Determine if fd is a V4L2 Device
    if (0 != camera_source_ioctl(ctx->fd, VIDIOC_QUERYCAP, &cap))
    {
        printf("Not v4l2 compatible\n");
        return -1;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) && !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE))
    {
        printf("Capture not supported\n");
        return -1;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        printf("Streaming IO Not Supported\n"); 
        return -1;
    }
    vfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)
        vfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    vfmt.fmt.pix.width = width;
    vfmt.fmt.pix.height = height;
    if (MPP_FRAME_FMT_IS_YUV(format))
    {
        vfmt.fmt.pix.pixelformat = V4L2_yuv_cfg[format - MPP_FRAME_FMT_YUV];
    }
    else if (MPP_FRAME_FMT_IS_RGB(format))
    {
        vfmt.fmt.pix.pixelformat = V4L2_RGB_cfg[format - MPP_FRAME_FMT_RGB];
    }
    else // MJPG格式
    {
        vfmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    }

    if (!vfmt.fmt.pix.pixelformat)
        vfmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;

    // vfmt.fmt.pix.field=V4L2_FIELD_SEQ_TB;
    ctx->type = (enum v4l2_buf_type)vfmt.type;

    return camera_source_ioctl(ctx->fd, VIDIOC_S_FMT, &vfmt);
}

static int v4l2_require_bufs(CamSource *ctx, int bufcnt)
{
    struct v4l2_requestbuffers req = {0};
    // Request memory-mapped buffers
    ctx->bufcnt = bufcnt;
    req.count = ctx->bufcnt;
    req.type = ctx->type;
    req.memory = ctx->mem_type;
    if (-1 == camera_source_ioctl(ctx->fd, VIDIOC_REQBUFS, &req))
    {
        printf("Device does not support mmap\n");
        return -1;
    }

    if (req.count != ctx->bufcnt)
    {
        printf("Device buffer count mismatch\n");
        return -1;
    }
    return 0;
}
static int v4l2_mmap_bufs(CamSource *ctx, int index)
{
    struct v4l2_buffer buf = {0};
    struct v4l2_plane planes[FMT_NUM_PLANES];
    unsigned int buf_len;
    buf.type = ctx->type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;

    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == ctx->type)
    {
        buf.m.planes = planes;
        buf.length = FMT_NUM_PLANES;
    }

    if (-1 == camera_source_ioctl(ctx->fd, VIDIOC_QUERYBUF, &buf))
    {
        printf("ERROR: VIDIOC_QUERYBUF\n");
        return -1;
    }

    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == buf.type)
    {
        // tmp_buffers[n_buffers].length = buf.m.planes[0].length;
        ctx->buf_len = buf.m.planes[0].length;
        ctx->fbuf[index].start =
            mmap(NULL /* start anywhere */,
                 buf.m.planes[0].length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 ctx->fd, buf.m.planes[0].m.mem_offset);
    }
    else
    {
        ctx->buf_len = buf.length;
        ctx->fbuf[index].start =
            mmap(NULL /* start anywhere */,
                 buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 ctx->fd, buf.m.offset);
    }

    if (MAP_FAILED == ctx->fbuf[index].start)
    {
        printf("ERROR: Failed to map device frame buffers\n");
        return -1;
    }
    return 0;
}
static int v4l2_export_bufs(CamSource *ctx, int index, int w, int h, int rk_format)
{
    struct v4l2_exportbuffer expbuf = {0};
    // xcam_mem_clear (expbuf);
    expbuf.type = ctx->type;
    expbuf.index = index;
    expbuf.flags = O_CLOEXEC;
    if (camera_source_ioctl(ctx->fd, VIDIOC_EXPBUF, &expbuf) < 0)
    {
        printf("get dma buf failed\n");
        return -1;
    }
    else
    {
        MppBufferInfo info;
        memset(&info, 0, sizeof(MppBufferInfo));
        info.type = MPP_BUFFER_TYPE_EXT_DMA;
        info.fd = expbuf.fd;
        info.size = ctx->buf_len & 0x07ffffff;
        info.index = (ctx->buf_len & 0xf8000000) >> 27;
        mpp_buffer_import(&ctx->fbuf[index].mpp_buf, &info);
        ctx->fbuf[index].rga_buf_handle = importbuffer_fd(expbuf.fd, ctx->buf_len & 0x07ffffff);
        ctx->fbuf[index].rga_buf = wrapbuffer_handle(ctx->fbuf[index].rga_buf_handle, w, h, rk_format);
    }
    ctx->fbuf[index].export_fd = expbuf.fd;
    return 0;
}
static int v4l2_queue_dmabuf(CamSource *ctx, int index, int fd)
{
    struct v4l2_plane planes[FMT_NUM_PLANES];
    struct v4l2_buffer buf = {0};
    buf.type = ctx->type;
    buf.index = index;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.m.fd = fd;
    ctx->fbuf[index].export_fd = fd;
    if (-1 == camera_source_ioctl(ctx->fd, VIDIOC_QBUF, &buf))
    {
        printf("ERROR: VIDIOC_QBUF %d\n", index);
        camera_source_deinit(ctx);
        return -1;
    }
    return 0;
}

// Create a new context to capture frames from <fname>.
// Returns NULL on error.
CamSource *camera_source_init(const char *device, RK_U32 bufcnt, RK_U32 width, RK_U32 height, MppFrameFormat format)
{
    int rk_format = change_MppFmt2RkFmt(format);
    RK_U32 i;
    RK_U32 buf_len = 0;
    CamSource *ctx;

    ctx = MEM_CALLOC(CamSource, 1);
    if (!ctx)
        return NULL;
    ctx->mem_type = V4L2_MEMORY_MMAP;
    ctx->fd = open(device, O_RDWR, 0);
    if (ctx->fd < 0)
    {
        printf("Cannot open %s\n",device);
        goto FAIL;
    }
    // Set device format
    if (-1 == v4l2_set_format(ctx, width, height, format))
    {
        printf("Failed to set format\n");
        goto FAIL;
    }

    // Request memory-mapped buffers
    if (-1 == v4l2_require_bufs(ctx, bufcnt))
        goto FAIL;

    // mmap() the buffers into userspace memory
    for (i = 0; i < ctx->bufcnt; i++)
    {
        if (-1 == v4l2_mmap_bufs(ctx, i))
        {
            printf("Faile to mmap buf%d\n", i);
            goto FAIL;
        }

        if (-1 == v4l2_export_bufs(ctx, i, width, height, rk_format))
        {
            printf("Failed to export buf%d\n", i);
            goto FAIL;
        }
    }

    for (i = 0; i < ctx->bufcnt; i++)
    {
        if (MPP_NOK == camera_source_put_frame(ctx, i))
        {
            printf("Failed to queue buf%d\n", i);
            goto FAIL;
        }
    }

    // Start capturing
    if (-1 == camera_source_ioctl(ctx->fd, VIDIOC_STREAMON, &ctx->type))
    {
        printf("ERROR: VIDIOC_STREAMON\n");
        camera_source_deinit(ctx);
        goto FAIL;
    }

    // skip some frames at start
    for (i = 0; i < ctx->bufcnt * 3; i++)
    {
        RK_S32 idx = camera_source_get_frame(ctx);
        if (idx >= 0)
            camera_source_put_frame(ctx, idx);
    }
    return ctx;

FAIL:
    camera_source_deinit(ctx);
    return NULL;
}
CamSource *mjpg_camera_source_init(const char *device, RK_U32 bufcnt, RK_U32 width, RK_U32 height)
{

    RK_U32 i;
    RK_U32 buf_len = 0;
    CamSource *ctx;
    decoder_ptr = new decoder(width, height);
    ctx = MEM_CALLOC(CamSource, 1);
    if (!ctx)
        return NULL;
    ctx->mem_type = V4L2_MEMORY_DMABUF;
    ctx->fd = open(device, O_RDWR, 0);
    if (ctx->fd < 0)
    {
        printf("Cannot open uv device\n");
        goto FAIL;
    }
    // Set device format
    if (-1 == v4l2_set_format(ctx, width, height, MPP_FMT_BUTT))
    {
        printf("Failed to set format\n");
        goto FAIL;
    }

    // Request memory-mapped buffers
    if (-1 == v4l2_require_bufs(ctx, bufcnt))
        goto FAIL;

    for (i = 0; i < ctx->bufcnt; i++)
    {
        ctx->fbuf[i].export_fd = decoder_ptr->buf_fd_array[i];
        if (MPP_NOK == camera_source_put_frame(ctx, i))
        {
            printf("Failed to queue buf%d\n", i);
            goto FAIL;
        }
    }

    // Start capturing
    if (-1 == camera_source_ioctl(ctx->fd, VIDIOC_STREAMON, &ctx->type))
    {
        printf("ERROR: VIDIOC_STREAMON\n");
        camera_source_deinit(ctx);
        goto FAIL;
    }

    // skip some frames at start
    for (i = 0; i < ctx->bufcnt * 3; i++)
    {
        RK_S32 idx = camera_source_get_frame(ctx);
        if (idx >= 0)
            camera_source_put_frame(ctx, idx);
    }
    printf("mjpe camera ok\n");
    return ctx;

FAIL:
    camera_source_deinit(ctx);
    return NULL;
}
// Free a context to capture frames from <fname>.
// Returns NULL on error.
MPP_RET camera_source_deinit(CamSource *ctx)
{
    struct v4l2_buffer buf;
    enum v4l2_buf_type type;
    RK_U32 i;

    if (NULL == ctx)
        return MPP_OK;

    if (ctx->fd < 0)
        return MPP_OK;

    // Stop capturing
    type = ctx->type;

    camera_source_ioctl(ctx->fd, VIDIOC_STREAMOFF, &type);

    // un-mmap() buffers
    for (i = 0; i < ctx->bufcnt; i++)
    {
        buf = (struct v4l2_buffer){0};
        buf.type = type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        camera_source_ioctl(ctx->fd, VIDIOC_QUERYBUF, &buf);
        releasebuffer_handle(ctx->fbuf[buf.index].rga_buf_handle);
        munmap(ctx->fbuf[buf.index].start, buf.length);
    }

    // Close v4l2 device
    close(ctx->fd);
    MEM_FREE(ctx);
    return MPP_OK;
}

MPP_RET mjpg_camera_source_deinit(CamSource *ctx)
{
    MPP_RET ret;
    ret = camera_source_deinit(ctx);
    if (ret != MPP_OK)
        return ret;
    delete decoder_ptr;
    return MPP_OK;
}

// Returns a pointer to a captured frame and its meta-data. NOT thread-safe.
RK_S32 camera_source_get_frame(CamSource *ctx)
{
    struct v4l2_buffer buf;
    enum v4l2_buf_type type;
    fd_set rfds;
    struct timeval tv;
    int retval;

    type = ctx->type;
    buf = (struct v4l2_buffer){0};
    buf.type = type;
    buf.memory = ctx->mem_type;

    struct v4l2_plane planes[FMT_NUM_PLANES];
    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == type)
    {
        buf.m.planes = planes;
        buf.length = FMT_NUM_PLANES;
    }
    // 1.select
    FD_ZERO(&rfds); 
    FD_SET(ctx->fd, &rfds);
    tv.tv_sec = 1; // 等待1秒
    tv.tv_usec = 0;
    retval = select(ctx->fd + 1, &rfds, NULL, NULL, &tv);
    if (-1 == retval)
    {
        printf("VIDIOC_DQBUF\n");
        return -1;
    }
    else if (retval)
    // 2.ioctl
    {
        retval = ioctl(ctx->fd, VIDIOC_DQBUF, &buf);
        if (retval == -1 && (EINTR != errno && EAGAIN != errno))
        {
            printf("ret = %d, errno %d", retval, errno);
            return -1;
        }
    }
    //  if (-1 == camera_source_ioctl(ctx->fd, VIDIOC_DQBUF, &buf))
    //  {
    //      printf("VIDIOC_DQBUF\n");
    //      return -1;
    //  }

    if (buf.index > ctx->bufcnt)
    {
        printf("buffer index out of bounds\n");
        return -1;
    }

    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == type)
        buf.bytesused = buf.m.planes[0].bytesused;
    ctx->fbuf[buf.index].bytesused = buf.bytesused;
    return buf.index;
}

CamFrame *mjpg_camera_source_get_frame(CamSource *ctx)
{
    RK_S32 idx = camera_source_get_frame(ctx);
    if (idx < 0)
        return NULL;
    decoder_ptr->decode(idx, ctx->fbuf[idx].bytesused);
    camera_source_put_frame(ctx, idx);
    return &decoder_ptr->frm_buf;
}

// It's OK to capture into this framebuffer now
MPP_RET camera_source_put_frame(CamSource *ctx, RK_S32 idx)
{
    struct v4l2_buffer buf;
    enum v4l2_buf_type type;

    if (idx < 0)
        return MPP_OK;

    type = ctx->type;
    buf = (struct v4l2_buffer){0};
    buf.type = type;
    buf.memory = ctx->mem_type;
    buf.index = idx;
    if (buf.memory == V4L2_MEMORY_DMABUF)
        buf.m.fd = ctx->fbuf[idx].export_fd;
    struct v4l2_plane planes[FMT_NUM_PLANES];
    if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == type)
    {
        buf.m.planes = planes;
        buf.length = FMT_NUM_PLANES;
    }

    // Tell kernel it's ok to overwrite this frame
    if (-1 == camera_source_ioctl(ctx->fd, VIDIOC_QBUF, &buf))
    {
        printf("VIDIOC_QBUF\n");
        return MPP_NOK;
    }
    return MPP_OK;
}

int get_frame_size(CamSource *ctx, RK_S32 idx)
{
    if (idx < 0)
        return 0;

    return ctx->fbuf[idx].bytesused;
}

void *camera_frame_to_buf(CamSource *ctx, RK_S32 idx)
{
    if (idx < 0)
        return NULL;

    return ctx->fbuf[idx].start;
}
rga_buffer_t *camera_frame_to_RgaBuffer(CamSource *ctx, RK_S32 idx)
{
    if (idx < 0)
        return NULL;
    return &ctx->fbuf[idx].rga_buf;
}

MppBuffer camera_frame_to_Mpp_buf(CamSource *ctx, RK_S32 idx)
{
    if (idx < 0)
        return NULL;
    return ctx->fbuf[idx].mpp_buf;
}

RgaSURF_FORMAT change_MppFmt2RkFmt(MppFrameFormat fmt)
{
    switch (fmt & MPP_FRAME_FMT_MASK)
    {
    case MPP_FMT_YUV420SP:
        return RK_FORMAT_YCbCr_420_SP;
    case MPP_FMT_YUV420P:
        return RK_FORMAT_YCbCr_420_P;
    case MPP_FMT_YUV422_YUYV:
        return RK_FORMAT_YUYV_422;
    case MPP_FMT_YUV422_YVYU:
        return RK_FORMAT_YVYU_422;
    case MPP_FMT_YUV422_UYVY:
        return RK_FORMAT_UYVY_422;
    case MPP_FMT_YUV422_VYUY:
        return RK_FORMAT_VYUY_422;
    case MPP_FMT_YUV422P:
        return RK_FORMAT_YCbCr_422_P;
    case MPP_FMT_YUV422SP:
        return RK_FORMAT_YCbCr_422_SP;
    case MPP_FMT_RGB444:
        return RK_FORMAT_ARGB_4444;
    case MPP_FMT_BGR444:
        return RK_FORMAT_ABGR_4444;
    case MPP_FMT_RGB555:
        return RK_FORMAT_ARGB_5551;
    case MPP_FMT_BGR555:
        return RK_FORMAT_ARGB_5551;
    case MPP_FMT_RGB565:
        return RK_FORMAT_RGB_565;
    case MPP_FMT_BGR565:
        return RK_FORMAT_BGR_565;
    case MPP_FMT_RGB888:
        return RK_FORMAT_RGB_888;
    case MPP_FMT_BGR888:
        return RK_FORMAT_BGR_888;
    case MPP_FMT_ARGB8888:
        return RK_FORMAT_ABGR_8888;
    case MPP_FMT_ABGR8888:
        return RK_FORMAT_ABGR_8888;
    case MPP_FMT_BGRA8888:
        return RK_FORMAT_BGRA_8888;
    case MPP_FMT_RGBA8888:
        return RK_FORMAT_RGBA_8888;
    default:
        return RK_FORMAT_UNKNOWN;
    }
}
void zoom_in(CamSource *ctx) // 拉近 Val:2
{

    v4l2_control control = {V4L2_CID_ZOOM_ABSOLUTE, 2};
    camera_source_ioctl(ctx->fd, VIDIOC_S_CTRL, &control);
}

void zoom_out(CamSource *ctx)
{

    v4l2_control control = {V4L2_CID_ZOOM_ABSOLUTE, 0};
    camera_source_ioctl(ctx->fd, VIDIOC_S_CTRL, &control);
}
void zoom_stop(CamSource *ctx)
{
    v4l2_control control = {V4L2_CID_ZOOM_ABSOLUTE, 1};
    camera_source_ioctl(ctx->fd, VIDIOC_S_CTRL, &control);
}

void set_zoom(CamSource *ctx, int ratio)
{
    ratio = ratio > 10 ? 10 : ratio;
    v4l2_control control = {V4L2_CID_TILT_ABSOLUTE, 100 + ratio};
    camera_source_ioctl(ctx->fd, VIDIOC_S_CTRL, &control);
}

void auto_focus(CamSource *ctx)
{
    ctx->autoFocus = true;
    v4l2_control control = {V4L2_CID_FOCUS_AUTO, true};
    camera_source_ioctl(ctx->fd, VIDIOC_S_CTRL, &control);
}

void focus_near(CamSource *ctx)
{
    if (ctx->autoFocus)
    {
        ctx->autoFocus = false;
        v4l2_control control = {V4L2_CID_FOCUS_AUTO, false};
        camera_source_ioctl(ctx->fd, VIDIOC_S_CTRL, &control);
    }
    v4l2_control control = {V4L2_CID_FOCUS_ABSOLUTE, 0};
    camera_source_ioctl(ctx->fd, VIDIOC_S_CTRL, &control);
}

void focus_far(CamSource *ctx)
{
    if (ctx->autoFocus)
    {
        ctx->autoFocus = false;
        v4l2_control control = {V4L2_CID_FOCUS_AUTO, false};
        camera_source_ioctl(ctx->fd, VIDIOC_S_CTRL, &control);
    }
    v4l2_control control = {V4L2_CID_FOCUS_ABSOLUTE, 2};
    camera_source_ioctl(ctx->fd, VIDIOC_S_CTRL, &control);
}


static int os_malloc(void **memptr, size_t alignment, size_t size)
{
    return posix_memalign(memptr, alignment, size);
}

void *mem_osal_malloc(size_t size)
{
    size_t size_align = MEM_ALIGNED(size);
    size_t size_real = size_align;
    void *ptr;

    os_malloc(&ptr, MEM_ALIGN, size_real);
    return ptr;
}

void *mem_osal_calloc(size_t size)
{
    void *ptr = mem_osal_malloc(size);
    if (ptr)
        memset(ptr, 0, size);
    return ptr;
}

