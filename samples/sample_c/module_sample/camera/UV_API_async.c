// Async/event-driven UART API built on top of UV_API.h
//
// Usage (no header provided; copy prototypes below as needed):
//   typedef struct uv_async uv_async_t;
//   typedef void (*uv_async_event_cb)(int event_type, int status, uint32_t value, void* user);
//   // lifecycle
//   int uv_async_create(const char* device, uv_async_t** out, uv_async_event_cb cb, void* user);
//   int uv_async_start(uv_async_t* ctx);
//   int uv_async_stop(uv_async_t* ctx);
//   void uv_async_destroy(uv_async_t* ctx);
//   // non-blocking enqueue APIs (return immediately)
//   int uv_async_set_high_voltage_gain(uv_async_t* ctx, uint16_t gain);
//   int uv_async_read_high_voltage_gain(uv_async_t* ctx);
//   int uv_async_set_counting_frequency(uv_async_t* ctx, uint16_t frequency);
//   int uv_async_request_photon_count(uv_async_t* ctx);
//   int uv_async_enable_region_photon_counting(uv_async_t* ctx, uint8_t enable);
//   int uv_async_set_region_x_start(uv_async_t* ctx, uint16_t x);
//   int uv_async_set_region_width(uv_async_t* ctx, uint16_t w);
//   int uv_async_set_region_y_start(uv_async_t* ctx, uint16_t y);
//   int uv_async_set_region_height(uv_async_t* ctx, uint16_t h);
//   // per-call callback variants (return immediately, out_req_id optional)
//   typedef void (*uv_async_once_cb)(uint32_t req_id, int event_type, int status, uint32_t value, void* user);
//   int uv_async_set_high_voltage_gain_cb(uv_async_t* ctx, uint16_t gain, uv_async_once_cb cb, void* user, uint32_t* out_req_id);
//   int uv_async_read_high_voltage_gain_cb(uv_async_t* ctx, uv_async_once_cb cb, void* user, uint32_t* out_req_id);
//   int uv_async_set_counting_frequency_cb(uv_async_t* ctx, uint16_t frequency, uv_async_once_cb cb, void* user, uint32_t* out_req_id);
//   int uv_async_request_photon_count_cb(uv_async_t* ctx, uv_async_once_cb cb, void* user, uint32_t* out_req_id);
//   int uv_async_enable_region_photon_counting_cb(uv_async_t* ctx, uint8_t enable, uv_async_once_cb cb, void* user, uint32_t* out_req_id);
//   int uv_async_set_region_x_start_cb(uv_async_t* ctx, uint16_t x, uv_async_once_cb cb, void* user, uint32_t* out_req_id);
//   int uv_async_set_region_width_cb(uv_async_t* ctx, uint16_t w, uv_async_once_cb cb, void* user, uint32_t* out_req_id);
//   int uv_async_set_region_y_start_cb(uv_async_t* ctx, uint16_t y, uv_async_once_cb cb, void* user, uint32_t* out_req_id);
//   int uv_async_set_region_height_cb(uv_async_t* ctx, uint16_t h, uv_async_once_cb cb, void* user, uint32_t* out_req_id);
//
// Events:
//   event_type: 1=ACK, 2=GAIN, 3=PHOTON_COUNT, 4=ERROR
//   status: UV_OK on success or UV_ERR_* on failure
//   value:  for GAIN -> gain value; for PHOTON_COUNT -> 32-bit count; for ACK -> echoed value if meaningful else 0
//
// Background behavior:
//   - A dedicated thread owns the serial fd, processes enqueued commands, and
//     when idle, polls photon count every 200ms and emits PHOTON_COUNT events.
//   - This file overrides UV_API receive timeouts to keep the loop responsive.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

// Shrink RX timeouts to keep async loop responsive
#ifndef SERIAL_RX_CHUNK_TIMEOUT_MS
#define SERIAL_RX_CHUNK_TIMEOUT_MS 50
#endif
#ifndef SERIAL_RX_TOTAL_TIMEOUT_MS
#define SERIAL_RX_TOTAL_TIMEOUT_MS 300
#endif

#include "UV_API.h"
#include "UV_API_async.h"



typedef struct
{
    uv_cmd_kind_t kind;
    uint16_t a;
    uint16_t b;
    uint8_t c;
    // per-call callback support
    uint32_t req_id;
    uv_async_once_cb once_cb;
    void *once_user;
} uv_cmd_t;

#define UV_CMD_QSIZE 32

struct uv_async
{
    int fd;
    volatile int running;
    pthread_t thr;
    pthread_mutex_t mtx;
    pthread_cond_t cv;
    uv_cmd_t q[UV_CMD_QSIZE];
    int q_head; // pop from head
    int q_tail; // push at tail
    uv_async_event_cb cb;
    void *user;
    // polling control
    uint32_t poll_interval_ms; // default 200
    struct timeval last_poll;
    // request id generator
    uint32_t next_req_id;
};

static uint64_t now_ms()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000ULL + (tv.tv_usec / 1000ULL);
}

static uint64_t tv_diff_ms(const struct timeval *a, const struct timeval *b)
{
    // return a - b in ms
    int64_t ds = (int64_t)a->tv_sec - (int64_t)b->tv_sec;
    int64_t du = (int64_t)a->tv_usec - (int64_t)b->tv_usec;
    return (uint64_t)(ds * 1000 + du / 1000);
}

static void uv_async_emit(uv_async_t *ctx, int event_type, int status, uint32_t value)
{
    if (ctx->cb)
        ctx->cb(event_type, status, value, ctx->user);
}

static void uv_emit_cmd(uv_async_t *ctx, const uv_cmd_t *cmd, int event_type, int status, uint32_t value)
{
    if (cmd && cmd->once_cb)
        cmd->once_cb(cmd->req_id, event_type, status, value, cmd->once_user);
    else
        uv_async_emit(ctx, event_type, status, value);
}

static int q_is_full(uv_async_t *ctx)
{
    int next = (ctx->q_tail + 1) % UV_CMD_QSIZE;
    return next == ctx->q_head;
}

static int q_is_empty(uv_async_t *ctx)
{
    return ctx->q_head == ctx->q_tail;
}

static int q_push(uv_async_t *ctx, uv_cmd_t cmd)
{
    pthread_mutex_lock(&ctx->mtx);
    if (q_is_full(ctx))
    {
        pthread_mutex_unlock(&ctx->mtx);
        return UV_ERR_OVERFLOW;
    }
    ctx->q[ctx->q_tail] = cmd;
    ctx->q_tail = (ctx->q_tail + 1) % UV_CMD_QSIZE;
    pthread_cond_signal(&ctx->cv);
    pthread_mutex_unlock(&ctx->mtx);
    return UV_OK;
}

static int q_pop(uv_async_t *ctx, uv_cmd_t *out)
{
    pthread_mutex_lock(&ctx->mtx);
    if (q_is_empty(ctx))
    {
        pthread_mutex_unlock(&ctx->mtx);
        return 0;
    }
    *out = ctx->q[ctx->q_head];
    ctx->q_head = (ctx->q_head + 1) % UV_CMD_QSIZE;
    pthread_mutex_unlock(&ctx->mtx);
    return 1;
}

static int wait_or_timeout(uv_async_t *ctx, uint32_t timeout_ms)
{
    struct timespec ts;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t ns = (uint64_t)tv.tv_sec * 1000000000ULL + (uint64_t)tv.tv_usec * 1000ULL + (uint64_t)timeout_ms * 1000000ULL;
    ts.tv_sec = (time_t)(ns / 1000000000ULL);
    ts.tv_nsec = (long)(ns % 1000000000ULL);
    pthread_mutex_lock(&ctx->mtx);
    int rc = 0;
    if (q_is_empty(ctx))
    {
        rc = pthread_cond_timedwait(&ctx->cv, &ctx->mtx, &ts);
    }
    pthread_mutex_unlock(&ctx->mtx);
    return rc; // 0 if signaled, ETIMEDOUT otherwise
}

static void *uv_async_thread(void *arg)
{
    uv_async_t *ctx = (uv_async_t *)arg;
    gettimeofday(&ctx->last_poll, NULL);

    while (ctx->running)
    {
        // Process one command if present
        uv_cmd_t cmd;
        int got = q_pop(ctx, &cmd);
        if (got == 1)
        {
            int rc = UV_OK;
            switch (cmd.kind)
            {
            case CMD_SET_HV_GAIN:
            {
                rc = set_high_voltage_gain(ctx->fd, cmd.a);
                uv_emit_cmd(ctx, &cmd, CMD_SET_HV_GAIN, rc, cmd.a);
            }
            break;
            case CMD_GET_HV_GAIN:
            {
                uint16_t gain = 0;
                rc = read_high_voltage_gain(ctx->fd, &gain);
                uv_emit_cmd(ctx, &cmd, CMD_GET_HV_GAIN, rc, gain);
            }
            break;
            case CMD_SET_FREQ:
            {
                rc = set_counting_frequency(ctx->fd, cmd.a);
                uv_emit_cmd(ctx, &cmd, CMD_SET_FREQ, rc, cmd.a);
            }
            break;
            case CMD_GET_PHOTON:
            {
                uint32_t cnt = 0;
                rc = read_photon_count(ctx->fd, &cnt);
                uv_emit_cmd(ctx, &cmd, CMD_GET_PHOTON, rc, cnt);
            }
            break;
            case CMD_ENABLE_REGION:
            {
                rc = enable_region_photon_counting(ctx->fd, (uint8_t)cmd.c);
                uv_emit_cmd(ctx, &cmd, CMD_ENABLE_REGION, rc, cmd.c);
            }
            break;
            case CMD_SET_X_START:
            {
                rc = set_region_x_start(ctx->fd, cmd.a);
                uv_emit_cmd(ctx, &cmd, CMD_SET_X_START, rc, cmd.a);
            }
            break;
            case CMD_SET_WIDTH:
            {
                rc = set_region_width(ctx->fd, cmd.a);
                uv_emit_cmd(ctx, &cmd, CMD_SET_WIDTH, rc, cmd.a);
            }
            break;
            case CMD_SET_Y_START:
            {
                rc = set_region_y_start(ctx->fd, cmd.a);
                uv_emit_cmd(ctx, &cmd, CMD_SET_Y_START, rc, cmd.a);
            }
            break;
            case CMD_SET_HEIGHT:
            {
                rc = set_region_height(ctx->fd, cmd.a);
                uv_emit_cmd(ctx, &cmd, CMD_SET_HEIGHT, rc, cmd.a);
            }
            break;
            case CMD_SET_UV_COLOR:
            {
                rc = set_photons_color(ctx->fd, cmd.c);
                uv_emit_cmd(ctx, &cmd, CMD_SET_UV_COLOR, rc, cmd.c);
            }
            break;
            default:
                break;
            }
            // After handling a command, continue to next loop without forced wait
            continue;
        }

        // Idle: perform periodic photon count poll every poll_interval_ms
        struct timeval now;
        gettimeofday(&now, NULL);
        uint64_t elapsed = tv_diff_ms(&now, &ctx->last_poll);
        if (elapsed >= ctx->poll_interval_ms)
        {
            uint32_t cnt = 0;
            int ret = UV_OK;
            ret = read_photon_count(ctx->fd, &cnt);
            // 注意：此处不是处理队列命令，不应引用未初始化的 cmd。
            // 直接走全局回调，避免解引用随机 once_cb 导致段错误。
            uv_async_emit(ctx, CMD_GET_PHOTON, ret, cnt);
            ctx->last_poll = now;
            continue;
        }

        // Wait for new command or until next poll time
        uint32_t wait_ms = (uint32_t)(ctx->poll_interval_ms - elapsed);
        if (wait_ms > 50)
            wait_ms = 50; // wake periodically even if far from poll time
        wait_or_timeout(ctx, wait_ms);
    }

    return NULL;
}

int uv_async_create(const char *device, uv_async_t **out, uv_async_event_cb cb, void *user)
{
    if (!out)
        return UV_ERR_ARG;
    uv_async_t *ctx = (uv_async_t *)calloc(1, sizeof(*ctx));
    if (!ctx)
        return UV_ERR_IO;

    ctx->fd = open_serial_port(device ? device : SERIAL_PORT);
    if (ctx->fd < 0)
    {
        free(ctx);
        return UV_ERR_IO;
    }
    if (set_serial_port(ctx->fd) != 0)
    {
        close(ctx->fd);
        free(ctx);
        return UV_ERR_IO;
    }

    pthread_mutex_init(&ctx->mtx, NULL);
    pthread_cond_init(&ctx->cv, NULL);
    ctx->q_head = ctx->q_tail = 0;
    ctx->cb = cb;
    ctx->user = user;
    ctx->poll_interval_ms = 200; // default 200ms
    gettimeofday(&ctx->last_poll, NULL);
    ctx->next_req_id = 1;

    *out = ctx;
    return UV_OK;
}

int uv_async_start(uv_async_t *ctx)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (ctx->running)
        return UV_OK;
    ctx->running = 1;
    int rc = pthread_create(&ctx->thr, NULL, uv_async_thread, ctx);
    if (rc != 0)
    {
        ctx->running = 0;
        return UV_ERR_IO;
    }
    return UV_OK;
}

int uv_async_stop(uv_async_t *ctx)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (!ctx->running)
        return UV_OK;
    ctx->running = 0;
    pthread_cond_broadcast(&ctx->cv);
    pthread_join(ctx->thr, NULL);
    return UV_OK;
}

void uv_async_destroy(uv_async_t *ctx)
{
    if (!ctx)
        return;
    uv_async_stop(ctx);
    if (ctx->fd >= 0)
        close(ctx->fd);
    pthread_mutex_destroy(&ctx->mtx);
    pthread_cond_destroy(&ctx->cv);
    free(ctx);
}

// Public non-blocking enqueue APIs
int uv_async_set_high_voltage_gain(uv_async_t *ctx, uint16_t gain)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (gain > 0x64)
        gain = 0x64;
    uv_cmd_t c = {.kind = CMD_SET_HV_GAIN, .a = gain};
    return q_push(ctx, c);
}

int uv_async_read_high_voltage_gain(uv_async_t *ctx)
{
    if (!ctx)
        return UV_ERR_ARG;
    uv_cmd_t c = {.kind = CMD_GET_HV_GAIN};
    return q_push(ctx, c);
}

int uv_async_set_counting_frequency(uv_async_t *ctx, uint16_t frequency)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (frequency == 0)
        frequency = 1;
    if (frequency > 0xFF)
        frequency = 0xFF;
    uv_cmd_t c = {.kind = CMD_SET_FREQ, .a = frequency};
    return q_push(ctx, c);
}

int uv_async_request_photon_count(uv_async_t *ctx)
{
    if (!ctx)
        return UV_ERR_ARG;
    uv_cmd_t c = {.kind = CMD_GET_PHOTON};
    return q_push(ctx, c);
}

int uv_async_enable_region_photon_counting(uv_async_t *ctx, uint8_t enable)
{
    if (!ctx)
        return UV_ERR_ARG;
    uv_cmd_t c = {.kind = CMD_ENABLE_REGION, .c = (enable ? 1 : 0)};
    return q_push(ctx, c);
}

int uv_async_set_region_x_start(uv_async_t *ctx, uint16_t x)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (x > 0x0500)
        x = 0x0500;
    uv_cmd_t c = {.kind = CMD_SET_X_START, .a = x};
    return q_push(ctx, c);
}

int uv_async_set_region_width(uv_async_t *ctx, uint16_t w)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (w < 0x0001)
        w = 0x0001;
    if (w > 0x0500)
        w = 0x0500;
    uv_cmd_t c = {.kind = CMD_SET_WIDTH, .a = w};
    return q_push(ctx, c);
}

int uv_async_set_region_y_start(uv_async_t *ctx, uint16_t y)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (y > 0x0400)
        y = 0x0400;
    uv_cmd_t c = {.kind = CMD_SET_Y_START, .a = y};
    return q_push(ctx, c);
}

int uv_async_set_region_height(uv_async_t *ctx, uint16_t h)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (h < 0x0001)
        h = 0x0001;
    if (h > 0x0400)
        h = 0x0400;
    uv_cmd_t c = {.kind = CMD_SET_HEIGHT, .a = h};
    return q_push(ctx, c);
}

int uv_async_set_photons_color(uv_async_t *ctx, uint8_t value)
{
    if (!ctx)
        return UV_ERR_ARG;
    uv_cmd_t c = {.kind = CMD_SET_UV_COLOR, .c = value};
    return q_push(ctx, c);
}

// ---------- Per-call callback variants ----------
static uint32_t uv_next_req_id(uv_async_t *ctx)
{
    pthread_mutex_lock(&ctx->mtx);
    uint32_t id = ++ctx->next_req_id;
    if (id == 0)
    {
        id = ++ctx->next_req_id;
    }
    pthread_mutex_unlock(&ctx->mtx);
    return id;
}

static int q_push_with_cb(uv_async_t *ctx, uv_cmd_t cmd, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (cb)
    {
        cmd.req_id = uv_next_req_id(ctx);
        cmd.once_cb = cb;
        cmd.once_user = user;
        if (out_req_id)
            *out_req_id = cmd.req_id;
    }
    return q_push(ctx, cmd);
}

int uv_async_set_high_voltage_gain_cb(uv_async_t *ctx, uint16_t gain, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (gain > 0x64)
        gain = 0x64;
    uv_cmd_t c = {.kind = CMD_SET_HV_GAIN, .a = gain};
    return q_push_with_cb(ctx, c, cb, user, out_req_id);
}

int uv_async_read_high_voltage_gain_cb(uv_async_t *ctx, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (!ctx)
        return UV_ERR_ARG;
    uv_cmd_t c = {.kind = CMD_GET_HV_GAIN};
    return q_push_with_cb(ctx, c, cb, user, out_req_id);
}

int uv_async_set_counting_frequency_cb(uv_async_t *ctx, uint16_t frequency, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (frequency == 0)
        frequency = 1;
    if (frequency > 0xFF)
        frequency = 0xFF;
    uv_cmd_t c = {.kind = CMD_SET_FREQ, .a = frequency};
    return q_push_with_cb(ctx, c, cb, user, out_req_id);
}

int uv_async_request_photon_count_cb(uv_async_t *ctx, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (!ctx)
        return UV_ERR_ARG;
    uv_cmd_t c = {.kind = CMD_GET_PHOTON};
    return q_push_with_cb(ctx, c, cb, user, out_req_id);
}

int uv_async_enable_region_photon_counting_cb(uv_async_t *ctx, uint8_t enable, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (!ctx)
        return UV_ERR_ARG;
    uv_cmd_t c = {.kind = CMD_ENABLE_REGION, .c = (enable ? 1 : 0)};
    return q_push_with_cb(ctx, c, cb, user, out_req_id);
}

int uv_async_set_region_x_start_cb(uv_async_t *ctx, uint16_t x, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (x > 0x0500)
        x = 0x0500;
    uv_cmd_t c = {.kind = CMD_SET_X_START, .a = x};
    return q_push_with_cb(ctx, c, cb, user, out_req_id);
}

int uv_async_set_region_width_cb(uv_async_t *ctx, uint16_t w, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (w < 0x0001)
        w = 0x0001;
    if (w > 0x0500)
        w = 0x0500;
    uv_cmd_t c = {.kind = CMD_SET_WIDTH, .a = w};
    return q_push_with_cb(ctx, c, cb, user, out_req_id);
}

int uv_async_set_region_y_start_cb(uv_async_t *ctx, uint16_t y, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (y > 0x0400)
        y = 0x0400;
    uv_cmd_t c = {.kind = CMD_SET_Y_START, .a = y};
    return q_push_with_cb(ctx, c, cb, user, out_req_id);
}

int uv_async_set_region_height_cb(uv_async_t *ctx, uint16_t h, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (!ctx)
        return UV_ERR_ARG;
    if (h < 0x0001)
        h = 0x0001;
    if (h > 0x0400)
        h = 0x0400;
    uv_cmd_t c = {.kind = CMD_SET_HEIGHT, .a = h};
    return q_push_with_cb(ctx, c, cb, user, out_req_id);
}

int uv_async_set_photons_color_cb(uv_async_t *ctx, uint8_t value, uv_async_once_cb cb, void *user, uint32_t *out_req_id)
{
    if (!ctx)
        return UV_ERR_ARG;
    uv_cmd_t c = {.kind = CMD_SET_UV_COLOR, .c = value};
    return q_push_with_cb(ctx, c, cb, user, out_req_id);
}
