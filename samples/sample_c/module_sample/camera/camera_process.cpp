#include "camera_process.h"
#include "select_delay.h"

static camera_process *processor = NULL;
static bool cam_stream_on = false;
static IM_STATUS rga_improcess(rga_buffer_t &src, rga_buffer_t &dst, im_rect sRect = {0}, im_rect dRect = {0}, int usage = 0);
static IM_STATUS rga_imcolorkey(const rga_buffer_t src,
                                rga_buffer_t dst,
                                im_rect srect,
                                im_rect drect,
                                im_colorkey_range range,
                                int mode,
                                int sync = 1,
                                int *release_fence_fd = NULL);
void *camera_process::camera_task(void *arg)
{
    camera_process *processor = (camera_process *)arg;
    int frame_id;
    int uv_frame_id;
    rga_buffer_t *uv_buf = NULL;
    rga_buffer_t *enc_buf = NULL;
    im_colorkey_range white_range;
    im_colorkey_range black_range;
    white_range.max = 0xFFFFFFFF; // ABGR
    white_range.min = 0xFF000001;
    black_range.max = 0xFF000001;
    black_range.min = 0xFF000000;

    // Add your camera processing logic here
    // processor->mEncoder->start_record("/mnt/sdcard/record.h264");
    DELAY_INIT;
    for (;;)
    {
        DELAY_US(33 * 1000, 1000);
        // Process main camera
        if (cam_stream_on)
        {
            frame_id = camera_source_get_frame(processor->mCamSource);
            if (frame_id < 0)
            {
                USER_LOG_ERROR("camera_source_get_frame failed\n");
                continue;
            }
            enc_buf = processor->mEncoder->deque_input_buf();
            if (enc_buf == NULL)
            {
                USER_LOG_ERROR("deque_input_buf failed\n");
                continue;
            }
            rga_improcess(*camera_frame_to_RgaBuffer(processor->mCamSource, frame_id), *enc_buf);
            camera_source_put_frame(processor->mCamSource, frame_id);
            // // Process UV camera
            // uv_frame_id = camera_source_get_frame(processor->mUvCamSource);
            // if (uv_frame_id < 0)
            // {
            //     USER_LOG_ERROR("camera_source_get_frame failed\n");
            //     continue;
            // }
            // uv_buf = camera_frame_to_RgaBuffer(processor->mUvCamSource, uv_frame_id);
            // {
            //     rga_imcolorkey(*uv_buf,
            //                    *enc_buf, // processor->mTempBuf.rga_buf,
            //                    processor->mUvSrcRect,
            //                    {0},
            //                    black_range,
            //                    IM_ALPHA_COLORKEY_NORMAL);
            //     camera_source_put_frame(processor->mUvCamSource, uv_frame_id);
            // }
            processor->mEncoder->enque_input_buf();
        }
    }
    return NULL;
}

bool camera_process::init(pOutput_func output_func)
{
    int ret = 0;
    if (mCamSource == NULL)
    {
        mCamSource = camera_source_init(mCam, mBufcnt, mCamWidth, mCamHeight, mCamFmt);
        if (mCamSource == NULL)
        {
            USER_LOG_ERROR("camera_source_init failed\n");
            return false;
        }
    }
    // if (mUvCamSource == NULL)
    // {
    //     mUvCamSource = camera_source_init(mUvCam, mBufcnt, mUvCamWidth, mUvCamHeight, mUvCamFmt);
    //     if (mUvCamSource == NULL)
    //     {
    //         USER_LOG_ERROR("camera_source_init failed\n");
    //         return false;
    //     }
    // }
    if (mEncoder == NULL)
    {
        mEncoder = new Encoder(mEncodeWidth, mEncodeHeight, mEncFmt);
        if (mEncoder == NULL)
        {
            USER_LOG_ERROR("encoder init failed\n");
            return false;
        }
    }

    ret = mEncoder->init(output_func);
    if (ret)
    {
        USER_LOG_ERROR("encoder init failed\n");
        return false;
    }

    ret = get_dma_buf(&mTempBuf, mEncodeWidth, mEncodeHeight, mBufFmt);
    if (ret < 0)
    {
        USER_LOG_ERROR("get_dma_buf failed\n");
        return false;
    }

    ret = pthread_create(&mCamThread, NULL, camera_task, this);
    if (ret)
    {
        USER_LOG_ERROR("pthread_create failed\n");
        return false;
    }
    pthread_detach(mCamThread);
    return true;
}

IM_STATUS rga_improcess(rga_buffer_t &src, rga_buffer_t &dst, im_rect sRect, im_rect dRect, int usage)
{
    IM_STATUS ret = IM_STATUS_NOERROR;

    rga_buffer_t pat = {0};
    im_rect prect = {0};
    usage |= IM_SYNC;
    ret = improcess(src, dst, pat, sRect, dRect, prect, usage);

    return ret;
}

static void empty_structure(rga_buffer_t *src, rga_buffer_t *dst, rga_buffer_t *pat,
                            im_rect *srect, im_rect *drect, im_rect *prect, im_opt_t *opt)
{
    if (src != NULL)
        memset(src, 0, sizeof(*src));
    if (dst != NULL)
        memset(dst, 0, sizeof(*dst));
    if (pat != NULL)
        memset(pat, 0, sizeof(*pat));
    if (srect != NULL)
        memset(srect, 0, sizeof(*srect));
    if (drect != NULL)
        memset(drect, 0, sizeof(*drect));
    if (prect != NULL)
        memset(prect, 0, sizeof(*prect));
    if (opt != NULL)
        memset(opt, 0, sizeof(*opt));
}

IM_STATUS rga_imcolorkey(const rga_buffer_t src, rga_buffer_t dst, im_rect srect, im_rect drect, im_colorkey_range range, int mode, int sync, int *release_fence_fd)
{
    int usage = 0;
    IM_STATUS ret = IM_STATUS_NOERROR;

    im_opt_t opt;

    rga_buffer_t pat;

    im_rect prect;

    empty_structure(NULL, NULL, &pat, NULL, NULL, &prect, &opt);

    usage |= mode;

    opt.colorkey_range = range;

    if (sync == 0)
        usage |= IM_ASYNC;
    else if (sync == 1)
        usage |= IM_SYNC;

    ret = improcess(src, dst, pat, srect, drect, prect, -1, release_fence_fd, &opt, usage);

    return ret;
}

bool CameraMediaStartService(void *func_ptr)
{
    processor = new camera_process();

    if (processor == NULL)
    {
        USER_LOG_ERROR("camera_process init failed\n");
        return false;
    }

    if (processor->init((pOutput_func)func_ptr) == false)
    {
        USER_LOG_ERROR("camera_process init failed\n");
        delete processor;
        return false;
    }
    cam_stream_on = true;
    return true;
}

int pause_cam_stream()
{
    if (processor == NULL)
        return -1;
    
    cam_stream_on = false;
    return 0;
}

int resume_cam_stream()
{
    if (processor == NULL)
        return -1;
    cam_stream_on = true;
    return 0;
}
