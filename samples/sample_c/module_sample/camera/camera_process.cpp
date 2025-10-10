#include "camera_process.h"
#include "select_delay.h"
#include <png.h>
#include "json.h"
#include <string>
#include <fstream>

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
                USER_LOG_ERROR("camera_source_get_frame failed:vl\n");
                continue;
            }
            rga_improcess(*camera_frame_to_RgaBuffer(processor->mCamSource, frame_id), processor->mTempBuf.rga_buf);
            camera_source_put_frame(processor->mCamSource, frame_id);
            // Process UV camera
            uv_frame_id = camera_source_get_frame(processor->mUvCamSource);
            if (uv_frame_id < 0)
            {
                USER_LOG_ERROR("camera_source_get_frame failed:uv\n");
                continue;
            }
            uv_buf = camera_frame_to_RgaBuffer(processor->mUvCamSource, uv_frame_id);
            rga_imcolorkey(*uv_buf,
                           processor->mTempBuf.rga_buf,
                           processor->mUvSrcRect,
                           processor->mUvDstRect,//dstRect
                           black_range,
                           IM_ALPHA_COLORKEY_NORMAL);
            camera_source_put_frame(processor->mUvCamSource, uv_frame_id);
            // enc_buf上锁
            enc_buf = processor->mEncoder->deque_input_buf();
            if (enc_buf == NULL)
            {
                USER_LOG_ERROR("deque_input_buf failed\n");
                continue;
            }
            rga_improcess(processor->mTempBuf.rga_buf, *enc_buf);
            processor->mEncoder->enque_input_buf();
            // enc_buf解锁
        }
        processor->update_uv_rect(0);

        /*另一实现方法*/
        /*
            VL_idx = camera_source_get_frame(VL_CamCtx);
            UV_idx = camera_source_get_frame(UV_CamCtx);
            dec.decode(UV_idx, get_frame_size(UV_CamCtx, UV_idx));
            camera_source_put_frame(UV_CamCtx, UV_idx);
            RGA_improcess(*camera_frame_to_RgaBuffer(VL_CamCtx, VL_idx), Temp_buf.rga_buf, DefaultRect, DefaultRect);
            camera_source_put_frame(VL_CamCtx, VL_idx);
            RGA_improcess(dec.frm_buf.rga_buf, UV_buf.rga_buf, UV_SrcRect, {0, 0, vl_roi_w, vl_roi_h}); // UV_SrcRect是紫外光源区域
            roi = (*UV_bgr_img)(Rect(0, 0, vl_roi_w, vl_roi_h));
            split(roi, UV_bgr_channels); // 拆分通道
            if (enable_denoise)          // 滤波
            {
                // morphologyEx(UV_bgr_channels[0], UV_bgr_channels[1], MORPH_CLOSE, CloseKernel);
                // morphologyEx(UV_bgr_channels[1], UV_bgr_channels[2], MORPH_OPEN, OpenKernel);
                threshold(UV_bgr_channels[0], UV_bgr_channels[1], 208, 255, THRESH_BINARY);
                medianBlur(UV_bgr_channels[1], UV_bgr_channels[2], 7); // 中值滤波
            }
            else
            {
                threshold(UV_bgr_channels[0], UV_bgr_channels[2], 208, 255, THRESH_BINARY);
            }

            // 将混合紫外光子
            roi = (*Temp_bgr_img)(Rect(vl_roi_x, vl_roi_y, vl_roi_w, vl_roi_h));
            rectangle(*Temp_bgr_img, Rect(vl_roi_x, vl_roi_y, vl_roi_w, vl_roi_h), Scalar(0, 255, 0), 4);
            split(roi, VL_bgr_channels);
            switch (colorSel)
            {
            case 0: // 纯红
                VL_bgr_channels[1].setTo(0, UV_bgr_channels[2]);
                VL_bgr_channels[0].setTo(0, UV_bgr_channels[2]);
            case 1: // 半红
                VL_bgr_channels[2].setTo(255, UV_bgr_channels[2]);
                break;
            case 2: // 橙色
                VL_bgr_channels[2].setTo(255, UV_bgr_channels[2]);
                VL_bgr_channels[1].setTo(97, UV_bgr_channels[2]);
                VL_bgr_channels[0].setTo(0, UV_bgr_channels[2]);
                break;
            case 3: // 黄色
                VL_bgr_channels[2].setTo(255, UV_bgr_channels[2]);
                VL_bgr_channels[1].setTo(255, UV_bgr_channels[2]);
                VL_bgr_channels[0].setTo(0, UV_bgr_channels[2]);
                break;
            case 4: // 纯绿
                VL_bgr_channels[2].setTo(0, UV_bgr_channels[2]);
                VL_bgr_channels[0].setTo(0, UV_bgr_channels[2]);
            case 5: // 半绿
                VL_bgr_channels[1].setTo(255, UV_bgr_channels[2]);
                break;
            case 6: // 青色
                VL_bgr_channels[2].setTo(0, UV_bgr_channels[2]);
                VL_bgr_channels[1].setTo(255, UV_bgr_channels[2]);
                VL_bgr_channels[0].setTo(255, UV_bgr_channels[2]);
                break;
            case 7: // 纯蓝
                VL_bgr_channels[2].setTo(0, UV_bgr_channels[2]);
                VL_bgr_channels[1].setTo(0, UV_bgr_channels[2]);
            case 8: // 半蓝
                VL_bgr_channels[0].setTo(255, UV_bgr_channels[2]);
                break;
            case 9: // 紫色
                VL_bgr_channels[2].setTo(160, UV_bgr_channels[2]);
                VL_bgr_channels[1].setTo(32, UV_bgr_channels[2]);
                VL_bgr_channels[0].setTo(240, UV_bgr_channels[2]);
                break;
            case 10: // 白色
                VL_bgr_channels[2].setTo(255, UV_bgr_channels[2]);
                VL_bgr_channels[1].setTo(255, UV_bgr_channels[2]);
                VL_bgr_channels[0].setTo(255, UV_bgr_channels[2]);
                break;
            case 11: // 黑色
                VL_bgr_channels[2].setTo(0, UV_bgr_channels[2]);
                VL_bgr_channels[1].setTo(0, UV_bgr_channels[2]);
                VL_bgr_channels[0].setTo(0, UV_bgr_channels[2]);
                break;
            }
            // VL_bgr_channels[2].setTo(255, UV_bgr_channels[2]);
            merge(VL_bgr_channels, 3, roi);
            RGA_improcess(Temp_buf.rga_buf, enc->mEnc_buf.rga_buf, DefaultRect, DefaultRect);
            enc->encode_input(&enc->mEnc_buf);

            // 计算可见融合的偏移和缩放
            if (radar->distance > 0)
            {
                uvvl_x_offset = zoom_ratio * (VL_Hor_relative_factor / radar->distance + VL_Hor_absolute_factor);
                uvvl_y_offset = zoom_ratio * (VL_Ver_relative_factor / radar->distance + VL_Ver_absolute_factor);
            }
            else
            {
                uvvl_x_offset = zoom_ratio * VL_Hor_absolute_factor;
                uvvl_y_offset = zoom_ratio * VL_Ver_absolute_factor;
            }
            uvvl_y_offset += VL_Axis_VerOffset[(int)zoom_ratio - 1];
            uvvl_x_offset += VL_Axis_HorOffset[(int)zoom_ratio - 1];

            dstRectWithVL.width = UV_BASE_W * zoom_ratio;
            dstRectWithVL.height = UV_BASE_H * zoom_ratio;
            dstRectWithVL.x = (SCREEN_W - dstRectWithVL.width) / 2 + uvvl_x_offset;
            dstRectWithVL.y = (SCREEN_H - dstRectWithVL.height) / 2 + uvvl_y_offset;

            vl_roi_x = std::max(dstRectWithVL.x, UV_RECT_X);
            vl_roi_y = std::max(dstRectWithVL.y, UV_RECT_Y);
            vl_roi_w = std::min((UV_RECT_X + UV_RECT_W), (dstRectWithVL.x + dstRectWithVL.width)) - vl_roi_x;
            vl_roi_h = std::min((UV_RECT_Y + UV_RECT_H), (dstRectWithVL.y + dstRectWithVL.height)) - vl_roi_y;

            UV_SrcRect.x = dstRectWithVL.x < UV_RECT_X ? UV_W * (UV_RECT_X - dstRectWithVL.x) / dstRectWithVL.width : 0;
            UV_SrcRect.y = dstRectWithVL.y < UV_RECT_Y ? UV_H * (UV_RECT_Y - dstRectWithVL.y) / dstRectWithVL.height : 0;
            UV_SrcRect.width = UV_W * vl_roi_w / dstRectWithVL.width;
            UV_SrcRect.height = UV_H * vl_roi_h / dstRectWithVL.height;
            break;
        */
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
    if (mUvCamSource == NULL)
    {
        mUvCamSource = camera_source_init(mUvCam, mBufcnt, mUvCamWidth, mUvCamHeight, mUvCamFmt);
        if (mUvCamSource == NULL)
        {
            USER_LOG_ERROR("camera_source_init failed\n");
            return false;
        }
    }
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

void camera_process::update_uv_rect(int distance)
{
    mUvDstRect.x = zoom_ratio*(horzDistCorrectionFactor/distance + horzAngCorrectionFactor);
    mUvDstRect.y = zoom_ratio*(vertDistCorrectionFactor/distance + vertAngCorrectionFactor);
    mUvDstRect.width = 1920 * scaleFactor*zoom_ratio;
    mUvDstRect.height = 1080 * scaleFactor*zoom_ratio;
    return;
}

bool camera_process::load_correction_factors(const char *config_file)
{
    int ret = 0;
    std::string errs;
    std::ifstream ifs(config_file, std::ios::binary);

        if (!ifs.is_open())
    {
        USER_LOG_ERROR("failed to open json file");
        return false;
    }
    Json::CharReaderBuilder rbuilder;
    Json::Value root;
    ret = Json::parseFromStream(rbuilder, ifs, &root, &errs);
    if (!ret)
    {
        USER_LOG_ERROR("failed to parse json file");
        ifs.close();
        return false;
    }
    ifs.close();
    horzDistCorrectionFactor = root["horzDistCorrectionFactor"].asInt();
    horzAngCorrectionFactor = root["horzAngCorrectionFactor"].asInt();
    vertDistCorrectionFactor = root["vertDistCorrectionFactor"].asInt();
    vertAngCorrectionFactor = root["vertAngCorrectionFactor"].asInt();
    scaleFactor = root["scaleFactor"].asFloat();
    return true;
}

bool camera_process::save_correction_factors(const char *config_file)
{
    Json::Value root;
    root["horzDistCorrectionFactor"] = horzDistCorrectionFactor;
    root["horzAngCorrectionFactor"] = horzAngCorrectionFactor;
    root["vertDistCorrectionFactor"] = vertDistCorrectionFactor;
    root["vertAngCorrectionFactor"] = vertAngCorrectionFactor;
    root["scaleFactor"] = scaleFactor;

    std::ofstream ofs(config_file, std::ios::binary);
    if (!ofs.is_open())
    {
        USER_LOG_ERROR("failed to open json file");
        return false;
    }
    Json::StreamWriterBuilder wbuilder;
    std::unique_ptr<Json::StreamWriter> writer(wbuilder.newStreamWriter());
    writer->write(root, &ofs);
    ofs.close();
    return true;    
}

void camera_process::update_correction_factors(int h_dist, int h_ang, int v_dist, int v_ang, float scale)
{
    horzDistCorrectionFactor = h_dist;
    horzAngCorrectionFactor = h_ang;
    vertDistCorrectionFactor = v_dist;
    vertAngCorrectionFactor = v_ang;
    scaleFactor = scale;
    return ;
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

int save_rgb_image_as_png(const char *filename, const uint8_t *image_data, int width, int height)
{
    FILE *fp = fopen(filename, "wb");
    if (!fp)
        return -1;

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr)
    {
        fclose(fp);
        return -2;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        png_destroy_write_struct(&png_ptr, NULL);
        fclose(fp);
        return -3;
    }

    if (setjmp(png_jmpbuf(png_ptr)))
    {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        fclose(fp);
        return -4;
    }

    png_init_io(png_ptr, fp);

    png_set_IHDR(png_ptr, info_ptr, width, height,
                 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    png_write_info(png_ptr, info_ptr);

    // 行指针数组
    png_bytep row_pointers[height];
    for (int y = 0; y < height; ++y)
    {
        row_pointers[y] = (png_bytep)(image_data + y * width * 3);
    }

    png_write_image(png_ptr, row_pointers);
    png_write_end(png_ptr, NULL);

    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);
    return 0;
}