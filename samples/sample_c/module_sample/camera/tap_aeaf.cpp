#include "tap_aeaf.h"
#include "camera_source.h"
#include "uAPI2/rk_aiq_user_api2_imgproc.h"
#include "uAPI2/rk_aiq_user_api2_ae.h"
#include <stdio.h>
#include <string.h>

#define TAP_AEAF_ERR(fmt, ...) printf("3A: " fmt, ##__VA_ARGS__)

static const rk_aiq_sys_ctx_t *tap_aiq(const CamSource *src)
{
    return cam_source_aiq(src);
}

/* ISP30 WinA: h_offs>=2, v_offs>=1, size multiple of 15,
 * and x+w < pic_w-2, y+h < pic_h-2. */
static int af_align_down(int v, int a)
{
    if (a <= 0 || v < 0)
        return 0;
    return v - (v % a);
}

static int af_clamp(int v, int lo, int hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static int isp_align_af_win(const CamSource *src, int *x, int *y, int *w, int *h)
{
    int sw;
    int sh;
    int x0;
    int y0;
    int x1;
    int y1;
    int usable_w;
    int usable_h;

    if (!src || !x || !y || !w || !h)
        return -1;
    if (cam_source_isp_size(src, &sw, &sh) != 0)
        return -1;

    x0 = 2;
    y0 = 1;
    x1 = sw - 2;
    y1 = sh - 2;
    usable_w = af_align_down(x1 - x0, 15);
    usable_h = af_align_down(y1 - y0, 15);
    if (usable_w < 15 || usable_h < 15)
        return -1;

    *w = af_align_down(*w, 15);
    *h = af_align_down(*h, 15);
    if (*w < 15)
        *w = 15;
    if (*h < 15)
        *h = 15;
    if (*w > usable_w)
        *w = usable_w;
    if (*h > usable_h)
        *h = usable_h;

    *x = af_align_down(*x, 2);
    *x = af_clamp(*x, x0, x1 - *w);
    *y = af_clamp(*y, y0, y1 - *h);
    return 0;
}

static int set_isp_focus_win(const CamSource *src, int *x, int *y, int *w, int *h)
{
    paRect_t rect;
    XCamReturn ret;
    const rk_aiq_sys_ctx_t *aiq = tap_aiq(src);

    if (!aiq || isp_align_af_win(src, x, y, w, h) != 0)
        return -1;

    memset(&rect, 0, sizeof(rect));
    rect.x = *x;
    rect.y = *y;
    rect.w = *w;
    rect.h = *h;
    ret = rk_aiq_uapi2_setFocusWin(aiq, &rect);
    if (ret != XCAM_RETURN_NO_ERROR)
    {
        TAP_AEAF_ERR("setFocusWin %d,%d %dx%d ret=%d\n",
                     rect.x, rect.y, rect.w, rect.h, (int)ret);
        return -1;
    }
    return 0;
}

static int trigger_oneshot_focus(const CamSource *src)
{
    XCamReturn ret;
    const rk_aiq_sys_ctx_t *aiq = tap_aiq(src);

    if (!aiq)
        return -1;
    ret = rk_aiq_uapi2_oneshotFocus(aiq);
    if (ret != XCAM_RETURN_NO_ERROR)
    {
        TAP_AEAF_ERR("oneshotFocus ret=%d\n", (int)ret);
        return -1;
    }
    return 0;
}

static int set_isp_ae_win(const CamSource *src, int x, int y, int w, int h)
{
    Uapi_ExpWin_t exp_win;
    XCamReturn ret;
    const rk_aiq_sys_ctx_t *aiq = tap_aiq(src);

    if (!aiq || isp_align_af_win(src, &x, &y, &w, &h) != 0)
        return -1;

    memset(&exp_win, 0, sizeof(exp_win));
    exp_win.sync.sync_mode = RK_AIQ_UAPI_MODE_SYNC;
    exp_win.Params.h_offs = (uint16_t)x;
    exp_win.Params.v_offs = (uint16_t)y;
    exp_win.Params.h_size = (uint16_t)w;
    exp_win.Params.v_size = (uint16_t)h;
    ret = rk_aiq_user_api2_ae_setExpWinAttr(aiq, exp_win);
    if (ret != XCAM_RETURN_NO_ERROR)
    {
        TAP_AEAF_ERR("setExpWin %u,%u %ux%u ret=%d\n",
                     exp_win.Params.h_offs, exp_win.Params.v_offs,
                     exp_win.Params.h_size, exp_win.Params.v_size, (int)ret);
        return -1;
    }
    return 0;
}

static int set_isp_ae_lock(const CamSource *src, int on)
{
    XCamReturn ret;
    const rk_aiq_sys_ctx_t *aiq = tap_aiq(src);

    if (!aiq)
        return -1;
    ret = rk_aiq_uapi2_setAeLock(aiq, on ? true : false);
    if (ret != XCAM_RETURN_NO_ERROR)
    {
        TAP_AEAF_ERR("setAeLock %d ret=%d\n", on ? 1 : 0, (int)ret);
        return -1;
    }
    return 0;
}

static int get_isp_ae_converged(const CamSource *src)
{
    Uapi_ExpQueryInfo_t info;
    XCamReturn ret;
    const rk_aiq_sys_ctx_t *aiq = tap_aiq(src);

    if (!aiq)
        return -1;
    memset(&info, 0, sizeof(info));
    ret = rk_aiq_user_api2_ae_queryExpResInfo(aiq, &info);
    if (ret != XCAM_RETURN_NO_ERROR)
        return -1;
    return info.IsConverged ? 1 : 0;
}

static int get_isp_af_search_stat(const CamSource *src)
{
    rk_aiq_af_result_t result;
    XCamReturn ret;
    const rk_aiq_sys_ctx_t *aiq = tap_aiq(src);

    if (!aiq)
        return -1;
    memset(&result, 0, sizeof(result));
    ret = rk_aiq_uapi2_getSearchResult(aiq, &result);
    if (ret != XCAM_RETURN_NO_ERROR)
        return -1;
    return (int)result.stat;
}

void TapAeAf::set_source(CamSource *src)
{
    std::lock_guard<std::mutex> lock(mtx_);
    src_ = src;
}

void TapAeAf::set_crop(int x, int y, int w, int h, int full_w, int full_h)
{
    std::lock_guard<std::mutex> lock(mtx_);
    crop_x_ = x;
    crop_y_ = y;
    crop_w_ = w;
    crop_h_ = h;
    if (full_w > 0)
        full_w_ = full_w;
    if (full_h > 0)
        full_h_ = full_h;
}

void TapAeAf::queue_tap(float nx, float ny)
{
    if (nx < 0.f)
        nx = 0.f;
    else if (nx > 1.f)
        nx = 1.f;
    if (ny < 0.f)
        ny = 0.f;
    else if (ny > 1.f)
        ny = 1.f;

    std::lock_guard<std::mutex> lock(mtx_);
    pending_nx_ = nx;
    pending_ny_ = ny;
    pending_ = true;
}

bool TapAeAf::compute_win(float nx, float ny, int *x, int *y, int *w, int *h) const
{
    static constexpr float kFrac = 1.0f / 6.0f;
    static constexpr int kMinW = 240;
    static constexpr int kMinH = 180;
    int crop_x = crop_x_;
    int crop_y = crop_y_;
    int crop_w = crop_w_;
    int crop_h = crop_h_;
    int win_w;
    int win_h;
    int cx;
    int cy;

    if (!x || !y || !w || !h)
        return false;
    if (crop_w <= 0 || crop_h <= 0)
    {
        crop_x = 0;
        crop_y = 0;
        crop_w = full_w_;
        crop_h = full_h_;
    }

    win_w = (int)(crop_w * kFrac);
    win_h = (int)(crop_h * kFrac);
    if (win_w < kMinW)
        win_w = crop_w < kMinW ? crop_w : kMinW;
    if (win_h < kMinH)
        win_h = crop_h < kMinH ? crop_h : kMinH;
    if (win_w > crop_w)
        win_w = crop_w;
    if (win_h > crop_h)
        win_h = crop_h;
    if (win_w < 2 || win_h < 2)
        return false;

    cx = crop_x + (int)(nx * (float)crop_w);
    cy = crop_y + (int)(ny * (float)crop_h);
    *w = win_w;
    *h = win_h;
    *x = cx - win_w / 2;
    *y = cy - win_h / 2;
    if (*x < crop_x)
        *x = crop_x;
    if (*y < crop_y)
        *y = crop_y;
    if (*x + *w > crop_x + crop_w)
        *x = crop_x + crop_w - *w;
    if (*y + *h > crop_y + crop_h)
        *y = crop_y + crop_h - *h;
    return true;
}

void TapAeAf::begin(float nx, float ny)
{
    static constexpr int kAfWaitStatsFrames = 2;
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
    CamSource *src = nullptr;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        src = src_;
        if (!compute_win(nx, ny, &x, &y, &w, &h))
        {
            TAP_AEAF_ERR("compute win failed\n");
            stage_ = Idle;
            return;
        }
    }
    if (!src)
        return;

    set_isp_ae_lock(src, 0);
    if (set_isp_focus_win(src, &x, &y, &w, &h) != 0)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        stage_ = Idle;
        return;
    }
    set_isp_ae_win(src, x, y, w, h);

    std::lock_guard<std::mutex> lock(mtx_);
    ae_frames_ = 0;
    wait_frames_ = kAfWaitStatsFrames;
    search_frames_ = 0;
    saw_running_ = false;
    af_retried_ = false;
    stage_ = AeMeterPre;
}

void TapAeAf::start_ae_post()
{
    std::lock_guard<std::mutex> lock(mtx_);
    ae_frames_ = 0;
    stage_ = AeMeterPost;
}

void TapAeAf::finish_af()
{
    bool pending;
    float pnx;
    float pny;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        pending = pending_;
        pnx = pending_nx_;
        pny = pending_ny_;
        if (pending)
            pending_ = false;
        stage_ = Idle;
    }

    if (pending)
        begin(pnx, pny);
    else
        start_ae_post();
}

void TapAeAf::poll()
{
    static constexpr int kAfWaitStatsFrames = 2;
    static constexpr int kAfRejectFrames = 8;
    static constexpr int kAfSearchTimeoutFrames = 40;
    static constexpr int kAePreMinFrames = 4;
    static constexpr int kAePreTimeoutFrames = 40;
    static constexpr int kAePostMinFrames = 2;
    static constexpr int kAePostTimeoutFrames = 20;
    float nx = 0.f;
    float ny = 0.f;
    bool start_pending = false;
    bool trigger = false;
    bool check_search = false;
    bool check_ae_pre = false;
    bool check_ae_post = false;
    CamSource *src = nullptr;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        src = src_;
        if (!src)
            return;
        if (pending_ && stage_ != AfSearching)
        {
            nx = pending_nx_;
            ny = pending_ny_;
            pending_ = false;
            start_pending = true;
        }
        else if (stage_ == AeMeterPre)
        {
            check_ae_pre = true;
        }
        else if (stage_ == AfWaitStats)
        {
            if (--wait_frames_ <= 0)
                trigger = true;
        }
        else if (stage_ == AfSearching)
        {
            check_search = true;
        }
        else if (stage_ == AeMeterPost)
        {
            check_ae_post = true;
        }
    }

    if (start_pending)
    {
        begin(nx, ny);
        return;
    }
    if (check_ae_pre)
    {
        int conv = get_isp_ae_converged(src);

        std::lock_guard<std::mutex> lock(mtx_);
        if (stage_ != AeMeterPre)
            return;
        ae_frames_++;
        if (ae_frames_ >= kAePreMinFrames &&
            (conv == 1 || ae_frames_ >= kAePreTimeoutFrames))
        {
            stage_ = AfWaitStats;
        }
        return;
    }
    if (trigger)
    {
        int oneshot_ret = trigger_oneshot_focus(src);
        bool give_up = false;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (oneshot_ret != 0)
            {
                if (!af_retried_)
                {
                    TAP_AEAF_ERR("oneshot rejected, retry once\n");
                    af_retried_ = true;
                    wait_frames_ = kAfWaitStatsFrames;
                    stage_ = AfWaitStats;
                }
                else
                {
                    TAP_AEAF_ERR("oneshot rejected after retry\n");
                    give_up = true;
                }
            }
            else
            {
                stage_ = AfSearching;
                search_frames_ = 0;
                saw_running_ = false;
            }
        }
        if (give_up)
            finish_af();
        return;
    }
    if (check_ae_post)
    {
        int conv = get_isp_ae_converged(src);
        bool done = false;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (stage_ != AeMeterPost)
                return;
            ae_frames_++;
            if (ae_frames_ >= kAePostMinFrames &&
                (conv == 1 || ae_frames_ >= kAePostTimeoutFrames))
            {
                stage_ = Idle;
                done = true;
            }
        }
        if (done)
            set_isp_ae_lock(src, 1);
        return;
    }
    if (!check_search)
        return;

    int stat = get_isp_af_search_stat(src);
    bool done = false;
    bool retry = false;
    int frames = 0;
    bool saw = false;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (stage_ != AfSearching)
            return;
        search_frames_++;
        frames = search_frames_;
        if (stat == 1)
            saw_running_ = true;
        saw = saw_running_;
        if (saw && stat == 2)
        {
            done = true;
        }
        else if ((!saw && frames >= kAfRejectFrames) ||
                 (saw && frames >= kAfSearchTimeoutFrames))
        {
            if (!af_retried_)
            {
                TAP_AEAF_ERR("AF internal error saw_running=%d frames=%d stat=%d, retry once\n",
                             (int)saw, frames, stat);
                retry = true;
            }
            else
            {
                TAP_AEAF_ERR("AF internal error after retry saw_running=%d frames=%d stat=%d\n",
                             (int)saw, frames, stat);
                done = true;
            }
        }
        if (retry)
        {
            af_retried_ = true;
            wait_frames_ = kAfWaitStatsFrames;
            search_frames_ = 0;
            saw_running_ = false;
            stage_ = AfWaitStats;
        }
    }

    if (done)
        finish_af();
}