#ifndef TAP_AEAF_H
#define TAP_AEAF_H

#include <mutex>

struct CamSource;

/*
 * Tap-to-AE then oneshot AF, then AE lock.
 * Non-blocking: queue_tap() from any thread; poll() once per frame.
 *
 * Does not judge whether the image is sharp. If oneshot is rejected or
 * never starts, retry the same oneshot once, then continue to AE lock.
 */
class TapAeAf
{
public:
    void set_source(CamSource *src);
    void set_crop(int x, int y, int w, int h, int full_w, int full_h);
    void queue_tap(float nx, float ny);
    void poll();

private:
    enum Stage
    {
        Idle,
        AeMeterPre,
        AfWaitStats,
        AfSearching,
        AeMeterPost,
    };

    bool compute_win(float nx, float ny, int *x, int *y, int *w, int *h) const;
    void begin(float nx, float ny);
    void start_ae_post();
    void finish_af();

    CamSource *src_ = nullptr;
    int crop_x_ = 0;
    int crop_y_ = 0;
    int crop_w_ = 0;
    int crop_h_ = 0;
    int full_w_ = 3840;
    int full_h_ = 2160;

    std::mutex mtx_;
    bool pending_ = false;
    float pending_nx_ = 0.5f;
    float pending_ny_ = 0.5f;
    Stage stage_ = Idle;
    int wait_frames_ = 0;
    int search_frames_ = 0;
    bool saw_running_ = false;
    bool af_retried_ = false;
    int ae_frames_ = 0;
};

#endif
