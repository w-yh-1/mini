#include "camera_process.h"
#include "select_delay.h"
#include "UV_API.h"
#include "UV_API_async.h"
#include <png.h>
#include "json.h"
#include <string>
#include <fstream>
#include <deque>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <sys/eventfd.h>
#include <poll.h>
#include "dji_camera_manager.h"
#include "dji_aircraft_info.h"
#include "free_type_osd.h"

#define CORRECTION_FACTOR_CONFIG_FILE "/usr/local/bin/correction.json"

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

static uv_async_t *uv_ctx = NULL;
static uint32_t uv_photons = 0;
static uint32_t uv_gain = 0;
// 用户设置的原始增益值（未换算），用于 OSD 显示
static std::atomic_uint32_t uv_gain_display_raw{0};
// FreeType OSD instance (used for snapshot overlay)
static FreeTypeOSD g_ftosd;
static bool g_ftosd_inited = false;
// OSD 语言开关：false=英文，true=中文（同时作用于录像 OSD 与快照 OSD）
static std::atomic_bool g_osd_use_chinese{true};
static void uv_cb_func(int et, int status, uint32_t value, void *user);
static bool init_UV_async(uv_async_t **uv_ctx);
static bool deinit_UV_async(uv_async_t *uv_ctx);
static int save_image_as_png(const char *filename, const uint8_t *image_data, int width, int height, time_t snapshot_time);
static int save_image_as_png_stride(const char *filename, const uint8_t *image_data, int width, int height, int stride_bytes,
                                    time_t snapshot_time);
static bool ensure_dir_exists(const char *dir);
static std::string build_unique_png_path(const char *dir, time_t snapshot_time);
static void ensure_process_timezone();
static constexpr int VL_BASE_CROP_WIDTH = 2560;
static constexpr int VL_BASE_CROP_HEIGHT = 1440;
static inline int align_even_floor(int value)
{
    return value & ~1;
}

static inline int align_even_ceil(int value)
{
    return (value + 1) & ~1;
}

static inline int clamp_even_to_range(int value, int min_value, int max_value)
{
    if (value < min_value)
        value = min_value;
    if (value > max_value)
        value = max_value;
    value = align_even_floor(value);
    if (value < min_value)
        value = align_even_ceil(min_value);
    return value;
}

static inline void eventfd_notify(int efd)
{
    if (efd < 0)
        return;
    uint64_t one = 1;
    ssize_t n = write(efd, &one, sizeof(one));
    // (void)n;
}
static inline void eventfd_drain(int efd)
{
    if (efd < 0)
        return;
    uint64_t val;
    ssize_t n = read(efd, &val, sizeof(val));
    (void)n;
}

static void ensure_process_timezone()
{
    static bool timezoneInitialized = false;

    if (timezoneInitialized)
        return;

    if (setenv("TZ", "CST-8", 1) != 0)
    {
        USER_LOG_ERROR("set TZ failed, errno=%d\n", errno);
        return;
    }

    tzset();
    timezoneInitialized = true;

    time_t now = time(NULL);
    struct tm tmv;
    char timebuf[64] = {0};
    localtime_r(&now, &tmv);
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S %z %Z", &tmv);
    USER_LOG_INFO("process timezone forced to CST-8, local time: %s\n", timebuf);
}
// --- 快照后台线程（静态成员） ---
void *camera_process::snapshot_worker(void *arg)
{
    camera_process *self = reinterpret_cast<camera_process *>(arg);
    self->mSnapThreadRunning.store(true, std::memory_order_release);
    while (self->mSnapThreadRunning.load(std::memory_order_acquire))
    {
        // 阻塞等待事件或定时检查退出
        struct pollfd pfd{self->mSnapEventFd, POLLIN, 0};
        int pr = poll(&pfd, 1, 500); // 500ms 超时，便于响应退出
        if (pr > 0 && (pfd.revents & POLLIN))
        {
            // 清空事件计数
            eventfd_drain(self->mSnapEventFd);
        }
        // 消费所有 Ready 缓冲
        for (int i = 0; i < 2; ++i)
        {
            uint8_t st = self->mSnapState[i].load(std::memory_order_acquire);
            if (st == 2 /*Ready*/)
            {
                uint8_t expected = 2;
                if (!self->mSnapState[i].compare_exchange_strong(expected, (uint8_t)3, std::memory_order_acq_rel))
                {
                    continue;
                }
                DMA_Buffer &buf = self->mSnapBuf[i];
                const char *path = self->mSnapPath[i];
                time_t snapshot_time = self->mSnapTimestamp[i];
                // 确保 CPU 可见
                dma_sync_device_to_cpu(buf.dma_fd);

                const int width = (int)buf.width;
                const int height = (int)self->mEncodeHeight; // 仅保存有效高度
                const int row_bytes = width * 3;
                uint8_t *base = (uint8_t *)buf.ptr;
                int stride_bytes = (int)buf.pitch;
                if (stride_bytes <= 0)
                {
                    USER_LOG_ERROR("snapshot invalid pitch=%d, fallback row_bytes=%d\n", stride_bytes, row_bytes);
                    stride_bytes = row_bytes;
                }
                // 在保存前叠加 OSD（时间、增益、光子数），如果 FreeType 初始化成功
                if (g_ftosd_inited)
                {
                    char tb[64];
                    struct tm tmv;
                    localtime_r(&snapshot_time, &tmv);
                    strftime(tb, sizeof(tb), "%Y-%m-%d %H:%M:%S", &tmv);
                    const bool use_cn = g_osd_use_chinese.load(std::memory_order_acquire);
                    char s0[128];
                    char s1[128];
                    char s2[64];

                    uint32_t gain_show = uv_gain_display_raw.load(std::memory_order_acquire);
                    if (use_cn)
                    {
                        snprintf(s0, sizeof(s0), "紫红光电");
                        snprintf(s1, sizeof(s1), "时间: %s", tb);
                        snprintf(s2, sizeof(s2), "增益: %u  光子数: %u", gain_show, (unsigned)uv_photons);
                    }
                    else
                    {
                        snprintf(s0, sizeof(s0), "UVIRSYS");
                        snprintf(s1, sizeof(s1), "Time: %s", tb);
                        snprintf(s2, sizeof(s2), "Gain: %u  Photons: %u", gain_show, (unsigned)uv_photons);
                    }
                    // 基线位置：左上角，留 10px 边距
                    int px = 10;
                    int py = 10 + 20; // baseline y，字体大小约24，取20作为近似
                    g_ftosd.render_text(s0, base, width, height, stride_bytes, px, py, 255, 255, 255);
                    // 第二行
                    py += 24 + 4;
                    g_ftosd.render_text(s1, base, width, height, stride_bytes, px, py, 255, 255, 255);
                    // 第三行
                    py += 24 + 4;
                    g_ftosd.render_text(s2, base, width, height, stride_bytes, px, py, 255, 255, 255);
                }
                int rc = 0;
                if (stride_bytes == row_bytes)
                {
                    rc = save_image_as_png(path, base, width, height, snapshot_time);
                }
                else
                {
                    rc = save_image_as_png_stride(path, base, width, height, stride_bytes, snapshot_time);
                }
                if (rc != 0)
                {
                    USER_LOG_ERROR("save_image_as_png failed(%d): %s\n", rc, path);
                }
                // 释放缓冲
                self->mSnapState[i].store((uint8_t)0, std::memory_order_release);
            }
        }
    }
    return NULL;
}

void *camera_process::laser_ranging_worker(void *arg)
{
    T_DjiReturnCode returnCode;
    camera_process *processor = (camera_process *)arg;

    returnCode = DjiCameraManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Init camera manager failed, error code: 0x%08X\r\n", returnCode);
        return NULL;
    }

    while (1)
    {
        T_DjiCameraManagerLaserRangingInfo rangingInfo = {0};
        returnCode = DjiCameraManager_GetLaserRangingInfo(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &rangingInfo);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("Get mounted position 1 laser ranging info failed, error code: 0x%08X.",
                           returnCode);
        }
        USER_LOG_INFO(
            "Receive lidar range info, lon:%.6f, lat:%.6f, alt:%.1f, dis:%d, enable:%d, exception:%d, x:%d, y:%d",
            rangingInfo.longitude,
            rangingInfo.latitude,
            (float)rangingInfo.altitude / 10,
            rangingInfo.distance,
            rangingInfo.enable_lidar,
            rangingInfo.exception,
            rangingInfo.screenX,
            rangingInfo.screenY);
    }

    return NULL;
}

void *camera_process::camera_task(void *arg)
{
    camera_process *processor = (camera_process *)arg;
    int frame_id;
    int uv_frame_id;
    int level = 0;
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
    while (processor->mEncoder->modify_osd_flaw_info == NULL)
    {
        usleep(100000);
        USER_LOG_INFO("waiting for encoder ready...");
    }
    DELAY_INIT;
    for (;;)
    {
        DELAY_US(CAMERA_FRAME_INTERVAL_US, 1000);
        // Process main camera
        if (cam_stream_on)
        {
            frame_id = camera_source_get_frame(processor->mCamSource);
            if (frame_id < 0)
            {
                USER_LOG_ERROR("camera_source_get_frame failed:vl\n");
                continue;
            }
            rga_improcess(*camera_frame_to_RgaBuffer(processor->mCamSource, frame_id), processor->mTempBuf.rga_buf, processor->mVlSrcRect);
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
                           processor->mUvDstRect, // dstRect
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
            rga_improcess(processor->mTempBuf.rga_buf, *enc_buf); // 此处变焦
            processor->mEncoder->enque_input_buf();

            // --- 拍照：仅在有请求时触发一次 RGA 拷贝/转换到 RGB888 的快照缓冲（无锁 SPSC 双缓冲） ---
            if (processor->mPendingSnap.load(std::memory_order_acquire))
            {
                // 查找 Free 槽并占用为 Filling
                int idx = -1;
                for (int i = 0; i < 2; ++i)
                {
                    uint8_t st = processor->mSnapState[i].load(std::memory_order_acquire);
                    if (st == 0 /*Free*/)
                    {
                        uint8_t expected = 0;
                        if (processor->mSnapState[i].compare_exchange_strong(expected, (uint8_t)1, std::memory_order_acq_rel))
                        {
                            idx = i;
                            break;
                        }
                    }
                }
                if (idx >= 0)
                {
                    // 拷贝路径并清除挂起标志
                    char local_path[512];
                    // 从挂起槽读取路径（发布-获取保证可见）
                    strncpy(local_path, processor->mPendingPathBuf, sizeof(local_path) - 1);
                    local_path[sizeof(local_path) - 1] = '\0';
                    processor->mPendingSnap.store(false, std::memory_order_release);

                    // RGA 硬件把 mTempBuf -> dst（RGB888），仅拍照时执行
                    DMA_Buffer &dst = processor->mSnapBuf[idx];
                    rga_improcess(processor->mTempBuf.rga_buf, dst.rga_buf);

                    // 在拷贝后，如果 pitch 仍为 0（某些路径没初始化），设置为紧凑行长，避免后续保存竖纹。
                    if (dst.pitch == 0)
                    {
                        dst.pitch = dst.width * 3; // RGB888
                    }

                    // 将路径写入对应槽
                    strncpy(processor->mSnapPath[idx], local_path, sizeof(processor->mSnapPath[idx]) - 1);
                    processor->mSnapPath[idx][sizeof(processor->mSnapPath[idx]) - 1] = '\0';
                    processor->mSnapTimestamp[idx] = processor->mPendingSnapTimestamp;

                    // 发布 Ready 并唤醒保存线程
                    processor->mSnapState[idx].store((uint8_t)2, std::memory_order_release);
                    eventfd_notify(processor->mSnapEventFd);
                }
            }
            // enc_buf解锁
        }
        level = (uv_photons > 8000) ? 3 : 
                (uv_photons > 1000) ? 2 : 
                (uv_photons > 200)  ? 1: 0;
        processor->mEncoder->modify_osd_flaw_info(&processor->mEncoder->osd_data, level);
        processor->mEncoder->modify_osd_count_data(&processor->mEncoder->osd_data, uv_photons, true);
        processor->mEncoder->modify_osd_gain_data(&processor->mEncoder->osd_data, uv_gain_display_raw.load(std::memory_order_acquire));
        processor->update_rects(0);
    }
    return NULL;
}

bool camera_process::init(pOutput_func output_func)
{
    int ret = 0;
    if (mCamSource == NULL)
    {
        mCamSource = isp_camera_source_init(mCam, mBufcnt, mCamWidth, mCamHeight, mCamFmt);
        if (mCamSource == NULL)
        {
            USER_LOG_ERROR("isp_camera_source_init failed\n");
            return false;
        }
    }
    sample_set_focus_semiautomode(mCamSource);

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

    // 使录像 OSD 语言与全局开关保持一致（避免服务启动前已切换语言）
    apply_language(g_osd_use_chinese.load(std::memory_order_acquire));

    // 尝试从移动端语言自动同步 OSD 语言
    T_DjiMobileAppInfo mobileAppInfo;
    if (DjiAircraftInfo_GetMobileAppInfo(&mobileAppInfo) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("get mobile app information error. keep previous OSD language");
    }
    else
    {
        bool app_cn = (mobileAppInfo.appLanguage == DJI_MOBILE_APP_LANGUAGE_CHINESE);
        apply_language(app_cn);
        USER_LOG_INFO("OSD language auto-set from app: %s\n", app_cn ? "Chinese" : "English");
    }

    ret = init_UV_async(&uv_ctx);
    if (!ret)
    {
        USER_LOG_ERROR("init_UV_async failed\n");
        return false;
    }

    // 尝试在指定目录下寻找常见字库并初始化 FreeType OSD（非致命）
    const char *font_dir = "/usr/share/fonts/source-han-sans-cn";
    const char *font_candidates[] = {
        "SourceHanSansCN-Regular.otf",
        NULL};
    char font_path[512] = {0};
    for (int i = 0; font_candidates[i] != NULL; ++i)
    {
        snprintf(font_path, sizeof(font_path), "%s/%s", font_dir, font_candidates[i]);
        if (access(font_path, F_OK) == 0)
        {
            if (g_ftosd.init(font_path, 24))
            {
                g_ftosd_inited = true;
                USER_LOG_INFO("FreeType OSD initialized with %s\n", font_path);
            }
            else
            {
                USER_LOG_ERROR("FreeType OSD init failed for %s\n", font_path);
            }
            break;
        }
    }

    set_uv_gain_raw(50); // 默认值（显示与用户一致）

    load_correction_factors(CORRECTION_FACTOR_CONFIG_FILE);
    update_rects(0);

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

    // --- 初始化快照双缓冲（RGB888，用于 PNG）与后台线程（无锁方案A） ---
    // 使用 RGB888 以便 save_image_as_png 按行 width*3 访问
    if (get_dma_buf(&mSnapBuf[0], mEncodeWidth, mEncodeHeight, RK_FORMAT_RGB_888) != 0 ||
        get_dma_buf(&mSnapBuf[1], mEncodeWidth, mEncodeHeight, RK_FORMAT_RGB_888) != 0)
    {
        USER_LOG_ERROR("get_dma_buf snapshot buffers failed\n");
        return false;
    }
    mSnapState[0].store(0, std::memory_order_relaxed);
    mSnapState[1].store(0, std::memory_order_relaxed);
    mPendingSnap.store(false, std::memory_order_relaxed);
    mSnapEventFd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (mSnapEventFd < 0)
    {
        USER_LOG_ERROR("eventfd create failed\n");
        return false;
    }
    int rc = pthread_create(&mSnapThread, NULL, &camera_process::snapshot_worker, this);
    if (rc != 0)
    {
        USER_LOG_ERROR("pthread_create snapshot worker failed\n");
        return false;
    }
    pthread_detach(mSnapThread);

    rc = pthread_create(&mLidarThread, NULL, &camera_process::laser_ranging_worker, this);
    if (rc != 0)
    {
        USER_LOG_ERROR("pthread_create laser ranging worker failed\n");
        return false;
    }
    pthread_detach(mLidarThread);
    return true;
}

void camera_process::update_rects(int distance)
{
    static int last_distance = 0;
    static float last_zoom_ratio = 1.0f;

    last_distance = distance;
    last_zoom_ratio = zoom_ratio;
    // 计算理想偏移：当前以固定偏移为主；距离补偿先保留占位。
    int x_offset = distance > 0 ? zoom_ratio * (horzDistCorrectionFactor / distance + horzAngCorrectionFactor) : zoom_ratio * horzAngCorrectionFactor;
    int y_offset = distance > 0 ? zoom_ratio * (vertDistCorrectionFactor / distance + vertAngCorrectionFactor) : zoom_ratio * vertAngCorrectionFactor;

    // 可见光主摄裁剪：优先利用 VL 在传感器上的裁切余量吸收平移，
    // 只有 VL 裁剪窗已经碰到边界时，才让 UV 叠加层补剩余残差。
    const int cam_w = (int)mCamWidth;
    const int cam_h = (int)mCamHeight;
    const int enc_w = mEncodeWidth;
    const int enc_h = mEncodeHeight;
    const float vl_zoom = zoom_ratio < 1.0f ? 1.0f : zoom_ratio;

    // 在最大 16:9 传感器裁剪区内，再取一个固定的 2560x1440 基准区，
    // 用于补偿 VL/UV 焦距差；zoom_ratio 则继续在这个基准区内部做电子变倍。
    int base_w = VL_BASE_CROP_WIDTH;
    int base_h = VL_BASE_CROP_HEIGHT;
    if (base_w > cam_w || base_h > cam_h)
    {
        // 若后续换成更小分辨率传感器，则回退到传感器所能容纳的最大 16:9 区域。
        if ((int64_t)cam_w * enc_h <= (int64_t)cam_h * enc_w)
        {
            base_w = align_even_floor(cam_w);
            base_h = align_even_ceil((int)((int64_t)base_w * enc_h / enc_w));
            base_h = clamp_even_to_range(base_h, 2, cam_h);
        }
        else
        {
            base_h = align_even_floor(cam_h);
            base_w = align_even_ceil((int)((int64_t)base_h * enc_w / enc_h));
            base_w = clamp_even_to_range(base_w, 2, cam_w);
        }
    }
    else
    {
        base_w = align_even_floor(base_w);
        base_h = align_even_floor(base_h);
    }

    const int vl_width = clamp_even_to_range((int)(base_w / vl_zoom), 2, base_w);
    const int vl_height = clamp_even_to_range((int)(base_h / vl_zoom), 2, base_h);
    const int vl_center_x = align_even_floor((cam_w - vl_width) / 2);
    const int vl_center_y = align_even_floor((cam_h - vl_height) / 2);

    // 期望把输出平移量优先折算到 VL 源裁剪窗中。
    const float vl_src_shift_x_f = (float)x_offset * (float)vl_width / (float)enc_w;
    const float vl_src_shift_y_f = (float)y_offset * (float)vl_height / (float)enc_h;
    const int vl_src_shift_x = align_even_floor((int)lroundf(vl_src_shift_x_f));
    const int vl_src_shift_y = align_even_floor((int)lroundf(vl_src_shift_y_f));

    const int vl_min_x = 0;
    const int vl_min_y = 0;
    const int vl_max_x = align_even_floor(cam_w - vl_width);
    const int vl_max_y = align_even_floor(cam_h - vl_height);

    mVlSrcRect.width = vl_width;
    mVlSrcRect.height = vl_height;
    mVlSrcRect.x = clamp_even_to_range(vl_center_x + vl_src_shift_x, vl_min_x, vl_max_x);
    mVlSrcRect.y = clamp_even_to_range(vl_center_y + vl_src_shift_y, vl_min_y, vl_max_y);

    const int actual_vl_src_shift_x = mVlSrcRect.x - vl_center_x;
    const int actual_vl_src_shift_y = mVlSrcRect.y - vl_center_y;
    const int residual_x_offset = x_offset - (int)lroundf((float)actual_vl_src_shift_x * (float)enc_w / (float)vl_width);
    const int residual_y_offset = y_offset - (int)lroundf((float)actual_vl_src_shift_y * (float)enc_h / (float)vl_height);

    // 理想 UV 目标区域：仅补 VL 没吃掉的残差。
    int temp_dst_width = 1920 * scaleFactor * zoom_ratio;
    int temp_dst_height = 1080 * scaleFactor * zoom_ratio;
    int temp_dst_x = (1920 - temp_dst_width) / 2 + residual_x_offset;
    int temp_dst_y = (1080 - temp_dst_height) / 2 + residual_y_offset;

    // 实际 UV 目标区域
    mUvDstRect.x = std::max(0, temp_dst_x);
    mUvDstRect.y = std::max(0, temp_dst_y);
    mUvDstRect.width = std::min(1920, temp_dst_width + temp_dst_x) - mUvDstRect.x;
    mUvDstRect.height = std::min(1080, temp_dst_height + temp_dst_y) - mUvDstRect.y;

    mUvSrcRect.x = temp_dst_x < 0 ? 1280 * (-temp_dst_x) / temp_dst_width : 0;
    mUvSrcRect.y = temp_dst_y < 0 ? 720 * (-temp_dst_y) / temp_dst_height : 0;
    mUvSrcRect.width = 1280 * mUvDstRect.width / temp_dst_width;
    mUvSrcRect.height = 720 * mUvDstRect.height / temp_dst_height;
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
    return;
}

// ------ 单项 set/get 接口实现 ------
void camera_process::set_horz_dist_correction(int v) { horzDistCorrectionFactor = v; }
int camera_process::get_horz_dist_correction() const { return horzDistCorrectionFactor; }

void camera_process::set_horz_ang_correction(int v) { horzAngCorrectionFactor = v; }
int camera_process::get_horz_ang_correction() const { return horzAngCorrectionFactor; }

void camera_process::set_vert_dist_correction(int v) { vertDistCorrectionFactor = v; }
int camera_process::get_vert_dist_correction() const { return vertDistCorrectionFactor; }

void camera_process::set_vert_ang_correction(int v) { vertAngCorrectionFactor = v; }
int camera_process::get_vert_ang_correction() const { return vertAngCorrectionFactor; }

void camera_process::set_scale_factor(float v) { scaleFactor = v; }
float camera_process::get_scale_factor() const { return scaleFactor; }

void camera_process::set_zoom_ratio(float v) { zoom_ratio = v < 1.0f ? 1.0f : v; }
float camera_process::get_zoom_ratio() const { return zoom_ratio; }

void camera_process::apply_language(bool use_cn)
{
    // 快照 OSD 标志
    g_osd_use_chinese.store(use_cn, std::memory_order_release);
    // 录像 OSD 切换
    if (mEncoder)
    {
        Encoder::switch_language(&mEncoder->osd_data, use_cn);
    }
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
static void uv_cb_func(int et, int status, uint32_t value, void *user)
{
    if (status != UV_OK)
    {
        USER_LOG_INFO("evt %d error %d\n", et, status);
        return;
    }
    switch (et)
    {
    case CMD_GET_PHOTON:
        uv_photons = value;
        break;
    case CMD_SET_HV_GAIN:
        uv_gain = value;
        break;
    default:
        USER_LOG_INFO("unknown evt %d\n", et);
        break;
    }
}

bool init_UV_async(uv_async_t **uv_ctx)
{
    int rc = uv_async_create("/dev/ttyS8", uv_ctx, uv_cb_func, NULL);
    if (rc != UV_OK)
    {
        USER_LOG_ERROR("uv_async_create failed: %d\n", rc);
        return false;
    }

    rc = uv_async_start(*uv_ctx);
    if (rc != UV_OK)
    {
        USER_LOG_ERROR("uv_async_start failed: %d\n", rc);
        uv_async_destroy(*uv_ctx);
        *uv_ctx = NULL;
        return false;
    }

    // 默认关闭 UV 区域计数功能，避免上电后立即启用区域光子统计。
    rc = uv_async_enable_region_photon_counting(*uv_ctx, 0);
    if (rc != UV_OK)
    {
        USER_LOG_ERROR("uv_async_enable_region_photon_counting(disable) failed: %d\n", rc);
        uv_async_stop(*uv_ctx);
        uv_async_destroy(*uv_ctx);
        *uv_ctx = NULL;
        return false;
    }
    
    return true;
}

static bool deinit_UV_async(uv_async_t *uv_ctx)
{
    if (uv_ctx == NULL)
        return false;

    int rc = uv_async_stop(uv_ctx);
    if (rc != UV_OK)
    {
        USER_LOG_ERROR("uv_async_stop failed: %d\n", rc);
        return false;
    }
    uv_async_destroy(uv_ctx);

    return true;
}

bool CameraMediaStartService(void *func_ptr)
{
    ensure_process_timezone();

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

static int save_image_as_png(const char *filename, const uint8_t *image_data, int width, int height, time_t snapshot_time)
{
    if (!filename || !image_data || width <= 0 || height <= 0)
    {
        USER_LOG_ERROR("png save invalid args: fname=%p, data=%p, w=%d, h=%d\n", filename, image_data, width, height);
        return -10;
    }

    // 写入到临时文件，成功后原子重命名，避免出现0字节或不完整文件
    char tmpname[1024];
    snprintf(tmpname, sizeof(tmpname), "%s.tmp", filename);
    FILE *fp = fopen(tmpname, "wb");
    if (!fp)
    {
        USER_LOG_ERROR("fopen failed: %s (errno=%d)\n", tmpname, errno);
        return -1;
    }

    // libpng 错误/警告处理（便于定位问题）
    auto png_error_fn = [](png_structp png_ptr, png_const_charp msg)
    {
        USER_LOG_ERROR("[libpng] error: %s\n", msg);
        longjmp(png_jmpbuf(png_ptr), 1);
    };
    auto png_warn_fn = [](png_structp, png_const_charp msg)
    {
        USER_LOG_ERROR("[libpng] warn: %s\n", msg);
    };

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, png_error_fn, png_warn_fn);
    if (!png_ptr)
    {
        USER_LOG_ERROR("png_create_write_struct failed\n");
        fclose(fp);
        unlink(tmpname);
        return -2;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        USER_LOG_ERROR("png_create_info_struct failed\n");
        png_destroy_write_struct(&png_ptr, NULL);
        fclose(fp);
        unlink(tmpname);
        return -3;
    }

    if (setjmp(png_jmpbuf(png_ptr)))
    {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        // 刷新并关闭临时文件并删除
        fclose(fp);
        unlink(tmpname);
        return -4;
    }

    png_init_io(png_ptr, fp);

    png_set_IHDR(png_ptr, info_ptr, width, height,
                 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    // 统一 PNG 文本元数据（与 stride 版一致：Time/Gain/Photons）
    png_text text_ptr[3];
    memset(text_ptr, 0, sizeof(text_ptr));
    char timebuf[64];
    struct tm tmv;
    localtime_r(&snapshot_time, &tmv);
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S %z", &tmv);
    char gainbuf[32];
    snprintf(gainbuf, sizeof(gainbuf), "%u", (unsigned)uv_gain_display_raw.load(std::memory_order_acquire));
    char photonsbuf[32];
    snprintf(photonsbuf, sizeof(photonsbuf), "%u", (unsigned)uv_photons);

    text_ptr[0].compression = PNG_TEXT_COMPRESSION_NONE;
    text_ptr[0].key = (char *)"Time";
    text_ptr[0].text = timebuf;
    text_ptr[1].compression = PNG_TEXT_COMPRESSION_NONE;
    text_ptr[1].key = (char *)"Gain";
    text_ptr[1].text = gainbuf;
    text_ptr[2].compression = PNG_TEXT_COMPRESSION_NONE;
    text_ptr[2].key = (char *)"Photons";
    text_ptr[2].text = photonsbuf;
    png_set_text(png_ptr, info_ptr, text_ptr, 3);

    png_write_info(png_ptr, info_ptr);

    // 行指针数组（紧凑 RGB，stride = width*3）
    std::vector<png_bytep> row_pointers;
    row_pointers.resize((size_t)height);
    for (int y = 0; y < height; ++y)
    {
        row_pointers[(size_t)y] = (png_bytep)(image_data + (size_t)y * width * 3);
    }

    png_write_image(png_ptr, row_pointers.data());
    png_write_end(png_ptr, NULL);

    // 强制落盘，降低掉电导致的空文件概率
    fflush(fp);
    int fd = fileno(fp);
    if (fd >= 0)
        fsync(fd);

    png_destroy_write_struct(&png_ptr, &info_ptr);
    int cerr = fclose(fp);
    if (cerr != 0)
    {
        USER_LOG_ERROR("fclose failed for %s (errno=%d)\n", tmpname, errno);
        unlink(tmpname);
        return -5;
    }

    // 原子重命名
    if (rename(tmpname, filename) != 0)
    {
        USER_LOG_ERROR("rename %s -> %s failed (errno=%d)\n", tmpname, filename, errno);
        unlink(tmpname);
        return -6;
    }
    return 0;
}

// 支持任意 stride 的 PNG 写入，避免额外 memcpy
static int save_image_as_png_stride(const char *filename, const uint8_t *image_data, int width, int height, int stride_bytes,
                                    time_t snapshot_time)
{
    if (!filename || !image_data || width <= 0 || height <= 0)
    {
        USER_LOG_ERROR("png(save_stride) invalid args: fname=%p, data=%p, w=%d, h=%d\n", filename, image_data, width, height);
        return -10;
    }
    const int row_bytes = width * 3;
    if (stride_bytes < row_bytes)
    {
        USER_LOG_ERROR("png(save_stride) bad stride: %d < row:%d\n", stride_bytes, row_bytes);
        return -11;
    }

    char tmpname[1024];
    snprintf(tmpname, sizeof(tmpname), "%s.tmp", filename);
    FILE *fp = fopen(tmpname, "wb");
    if (!fp)
    {
        USER_LOG_ERROR("fopen failed: %s (errno=%d)\n", tmpname, errno);
        return -1;
    }

    // libpng 错误/警告处理（便于定位问题）
    auto png_error_fn = [](png_structp png_ptr, png_const_charp msg)
    {
        USER_LOG_ERROR("[libpng] error: %s\n", msg);
        longjmp(png_jmpbuf(png_ptr), 1);
    };
    auto png_warn_fn = [](png_structp, png_const_charp msg)
    {
        USER_LOG_ERROR("[libpng] warn: %s\n", msg);
    };

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, png_error_fn, png_warn_fn);
    if (!png_ptr)
    {
        USER_LOG_ERROR("png_create_write_struct failed\n");
        fclose(fp);
        unlink(tmpname);
        return -2;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        USER_LOG_ERROR("png_create_info_struct failed\n");
        png_destroy_write_struct(&png_ptr, NULL);
        fclose(fp);
        unlink(tmpname);
        return -3;
    }

    if (setjmp(png_jmpbuf(png_ptr)))
    {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        fclose(fp);
        unlink(tmpname);
        return -4;
    }

    png_init_io(png_ptr, fp);

    png_set_IHDR(png_ptr, info_ptr, width, height,
                 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

    // 添加 PNG 文本元数据（UV 光子计数）
    png_text text_ptr[3];
    memset(text_ptr, 0, sizeof(text_ptr));
    char timebuf[64];
    struct tm tmv;
    localtime_r(&snapshot_time, &tmv);
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S %z", &tmv);
    char gainbuf[64];
    snprintf(gainbuf, sizeof(gainbuf), "%u", (unsigned)uv_gain_display_raw.load(std::memory_order_acquire));
    char photonsbuf[64];
    snprintf(photonsbuf, sizeof(photonsbuf), "%u", (unsigned)uv_photons);

    text_ptr[0].compression = PNG_TEXT_COMPRESSION_NONE;
    text_ptr[0].key = (char *)"Time";
    text_ptr[0].text = timebuf;
    text_ptr[1].compression = PNG_TEXT_COMPRESSION_NONE;
    text_ptr[1].key = (char *)"Gain";
    text_ptr[1].text = gainbuf;
    text_ptr[2].compression = PNG_TEXT_COMPRESSION_NONE;
    text_ptr[2].key = (char *)"Photons";
    text_ptr[2].text = photonsbuf;
    png_set_text(png_ptr, info_ptr, text_ptr, 3);

    png_write_info(png_ptr, info_ptr);

    // 行指针数组（带 stride）
    std::vector<png_bytep> row_pointers;
    row_pointers.resize((size_t)height);
    for (int y = 0; y < height; ++y)
    {
        row_pointers[(size_t)y] = (png_bytep)(image_data + (size_t)y * stride_bytes);
    }

    png_write_image(png_ptr, row_pointers.data());
    png_write_end(png_ptr, NULL);

    fflush(fp);
    int fd = fileno(fp);
    if (fd >= 0)
        fsync(fd);

    png_destroy_write_struct(&png_ptr, &info_ptr);
    int cerr = fclose(fp);
    if (cerr != 0)
    {
        USER_LOG_ERROR("fclose failed for %s (errno=%d)\n", tmpname, errno);
        unlink(tmpname);
        return -5;
    }
    if (rename(tmpname, filename) != 0)
    {
        USER_LOG_ERROR("rename %s -> %s failed (errno=%d)\n", tmpname, filename, errno);
        unlink(tmpname);
        return -6;
    }
    return 0;
}

int set_uv_color(uint value)
{
    int rc = uv_async_set_photons_color(uv_ctx, (uint8_t)value);
    if (rc != UV_OK)
    {
        USER_LOG_ERROR("uv_async_set_photons_color failed: %d\n", rc);
        return -1;
    }
    return 0;
}

int set_uv_gain(uint value)
{
    // value 为换算后的实际硬件增益，尝试反推用户值以便显示一致
    // 反算公式近似 raw = (value - 45) * 3.3
    if (value >= 45)
    {
        uint32_t raw = (uint32_t)((value - 45) * 3.3f + 0.5f);
        uv_gain_display_raw.store(raw, std::memory_order_release);
    }

    int rc = uv_async_set_high_voltage_gain(uv_ctx, (uint16_t)value);
    if (rc != UV_OK)
    {
        USER_LOG_ERROR("uv_async_set_uv_gain failed: %d\n", rc);
        return -1;
    }
    return 0;
}

// 新接口：传入未换算的用户设置值，内部完成换算并用于 OSD 显示
int set_uv_gain_raw(uint value)
{
    uv_gain_display_raw.store(value, std::memory_order_release);
    uint16_t hw = (uint16_t)((float)value / 3.3f + 45.0f + 0.5f);
    int rc = uv_async_set_high_voltage_gain(uv_ctx, hw);
    if (rc != UV_OK)
    {
        USER_LOG_ERROR("uv_async_set_uv_gain failed: %d\n", rc);
        return -1;
    }
    return 0;
}

// 统一语言切换：录像 OSD + 拍照 OSD
void language_switch(bool use_cn)
{
    if (processor)
    {
        processor->apply_language(use_cn);
    }
    else
    {
        // 服务未启动时仍更新全局标志，确保启动后快照 OSD 生效
        g_osd_use_chinese.store(use_cn, std::memory_order_release);
    }
    USER_LOG_INFO("OSD language switched to %s\n", use_cn ? "Chinese" : "English");
}

// C 包装：触发拍照
int take_snapshot(const char *path)
{
    if (!processor)
        return -1;
    return processor->request_snapshot(path);
}

int take_snapshot_auto()
{
    if (!processor)
        return -1;
    return processor->request_snapshot_auto();
}

int start_recording()
{
    return processor->start_recording();
}

void stop_recording()
{
    return processor->stop_recording();
}

// -------- C 封装: 单项矫正系数 set/get --------
int camera_set_horz_dist_correction(int v)
{
    if (!processor)
        return -1;
    processor->set_horz_dist_correction(v);
    return 0;
}
int camera_get_horz_dist_correction()
{
    if (!processor)
        return 0; // 若未初始化返回0
    return processor->get_horz_dist_correction();
}
int camera_set_horz_ang_correction(int v)
{
    if (!processor)
        return -1;
    processor->set_horz_ang_correction(v);
    return 0;
}
int camera_get_horz_ang_correction()
{
    if (!processor)
        return 0;
    return processor->get_horz_ang_correction();
}
int camera_set_vert_dist_correction(int v)
{
    if (!processor)
        return -1;
    processor->set_vert_dist_correction(v);
    return 0;
}
int camera_get_vert_dist_correction()
{
    if (!processor)
        return 0;
    return processor->get_vert_dist_correction();
}
int camera_set_vert_ang_correction(int v)
{
    if (!processor)
        return -1;
    processor->set_vert_ang_correction(v);
    return 0;
}
int camera_get_vert_ang_correction()
{
    if (!processor)
        return 0;
    return processor->get_vert_ang_correction();
}
int camera_set_scale_factor(float v)
{
    if (!processor)
        return -1;
    processor->set_scale_factor(v);
    return 0;
}
float camera_get_scale_factor()
{
    if (!processor)
        return 0.0f;
    return processor->get_scale_factor();
}

bool camera_correction_factor_save()
{
    if (!processor)
        return false;
    return processor->save_correction_factors(CORRECTION_FACTOR_CONFIG_FILE);
}

bool camera_correction_factor_load()
{
    if (!processor)
        return false;
    return processor->load_correction_factors(CORRECTION_FACTOR_CONFIG_FILE);
}

void camera_set_zoom_ratio(float v)
{
    if (!processor)
        return;
    processor->set_zoom_ratio(v);
    USER_LOG_INFO("set zoom ratio to %.2f\n", v);
}
float camera_get_zoom_ratio()
{
    if (!processor)
        return 1.0f;
    return processor->get_zoom_ratio();
}

// 成员方法：请求一次拍照
int camera_process::request_snapshot(const char *path)
{
    return request_snapshot_with_time(path, time(NULL));
}

int camera_process::request_snapshot_with_time(const char *path, time_t snapshot_time)
{
    if (!path || !*path)
        return -1;
    // 若已有待抓取请求，则返回忙
    if (mPendingSnap.load(std::memory_order_acquire))
        return -1;
    // 检查是否存在 Free 槽
    bool has_free = false;
    for (int i = 0; i < 2; ++i)
    {
        if (mSnapState[i].load(std::memory_order_acquire) == 0)
        {
            has_free = true;
            break;
        }
    }
    if (!has_free)
        return -1; // busy
    // 写入路径并发布请求
    strncpy(mPendingPathBuf, path, sizeof(mPendingPathBuf) - 1);
    mPendingPathBuf[sizeof(mPendingPathBuf) - 1] = '\0';
    mPendingSnapTimestamp = snapshot_time;
    mPendingSnap.store(true, std::memory_order_release);
    return 0;
}

int camera_process::request_snapshot_auto()
{
    // 确保保存目录存在
    if (!ensure_dir_exists(SNAPSHOT_DIR))
    {
        USER_LOG_ERROR("snapshot dir not available: %s\n", SNAPSHOT_DIR);
        return -1;
    }
    time_t snapshot_time = time(NULL);
    std::string path = build_unique_png_path(SNAPSHOT_DIR, snapshot_time);
    return request_snapshot_with_time(path.c_str(), snapshot_time);
}

static bool ensure_dir_exists(const char *dir)
{
    struct stat st;
    if (stat(dir, &st) == 0)
    {
        return S_ISDIR(st.st_mode);
    }
    if (mkdir(dir, 0775) == 0)
    {
        return true;
    }
    if (errno == EEXIST)
        return true;
    return false;
}

static std::string build_unique_png_path(const char *dir, time_t snapshot_time)
{
    struct tm tmv;
    localtime_r(&snapshot_time, &tmv);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);
    std::string base = std::string(dir) + ts;
    std::string path = base + ".png";
    int idx = 1;
    while (access(path.c_str(), F_OK) == 0 && idx < 1000)
    {
        path = base + "_" + std::to_string(idx) + ".png";
        ++idx;
    }
    return path;
}

static void Append_timeStr(char str[])
{
    time_t timep;
    time(&timep);
    char tempstr[128];
    strftime(tempstr, 128, "%Y.%m.%d-%H_%M_%S", localtime(&timep));
    strcat(str, tempstr);
}

int camera_process::start_recording()
{
    if (!processor)
        return -1;
    static int index = 1;
    size_t pos;
    char filename[256] = "/mnt/sdcard/H264/";
    Append_timeStr(filename);
    strcat(filename, ".h264");
    // 若文件已存在，加后缀
    if (access(filename, F_OK) == 0)
    {
        pos = strlen(filename) - 4;
        snprintf(&filename[pos], 10, "[%02d].h264", index++);
    }
    else
        index = 1;
    return processor->mEncoder->start_record(filename);
}

void camera_process::stop_recording()
{
    if (!processor)
        return;
    return processor->mEncoder->stop_record();
}

void oneshot_focus_auto()
{
    if (!processor)
        return;
    processor->do_oneshot_focus();
}

void camera_process::do_oneshot_focus()
{
    if (mCamSource)
        oneshot_focus(mCamSource);
}