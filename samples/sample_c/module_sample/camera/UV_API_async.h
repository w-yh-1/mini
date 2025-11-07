#ifndef UV_API_ASYNC_H
#define UV_API_ASYNC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	// 事件类型（同步到实现文件）
	enum
	{
		UV_EVENT_ACK = 1,
		UV_EVENT_GAIN = 2,
		UV_EVENT_PHOTON_COUNT = 3,
		UV_EVENT_ERROR = 4
	};

	// Opaque 句柄
	typedef struct uv_async uv_async_t;

	// 全局回调（持续事件/广播，如定时光子数）
	typedef void (*uv_async_event_cb)(int event_type, int status, uint32_t value, void *user);
	// 单次回调（只针对一次入队的命令，有 req_id）
	typedef void (*uv_async_once_cb)(uint32_t req_id, int event_type, int status, uint32_t value, void *user);

	// 生命周期
	int uv_async_create(const char *device, uv_async_t **out, uv_async_event_cb cb, void *user);
	int uv_async_start(uv_async_t *ctx);
	int uv_async_stop(uv_async_t *ctx);
	void uv_async_destroy(uv_async_t *ctx);

	// 非阻塞入队（无一次回调）
	int uv_async_set_high_voltage_gain(uv_async_t *ctx, uint16_t gain);
	int uv_async_read_high_voltage_gain(uv_async_t *ctx);
	int uv_async_set_counting_frequency(uv_async_t *ctx, uint16_t frequency);
	int uv_async_request_photon_count(uv_async_t *ctx);
	int uv_async_enable_region_photon_counting(uv_async_t *ctx, uint8_t enable);
	int uv_async_set_region_x_start(uv_async_t *ctx, uint16_t x);
	int uv_async_set_region_width(uv_async_t *ctx, uint16_t w);
	int uv_async_set_region_y_start(uv_async_t *ctx, uint16_t y);
	int uv_async_set_region_height(uv_async_t *ctx, uint16_t h);
	int uv_async_set_photons_color(uv_async_t *ctx, uint8_t value);

	// 非阻塞入队（带一次回调，可选返回 req_id）
	int uv_async_set_high_voltage_gain_cb(uv_async_t *ctx, uint16_t gain, uv_async_once_cb cb, void *user, uint32_t *out_req_id);
	int uv_async_read_high_voltage_gain_cb(uv_async_t *ctx, uv_async_once_cb cb, void *user, uint32_t *out_req_id);
	int uv_async_set_counting_frequency_cb(uv_async_t *ctx, uint16_t frequency, uv_async_once_cb cb, void *user, uint32_t *out_req_id);
	int uv_async_request_photon_count_cb(uv_async_t *ctx, uv_async_once_cb cb, void *user, uint32_t *out_req_id);
	int uv_async_enable_region_photon_counting_cb(uv_async_t *ctx, uint8_t enable, uv_async_once_cb cb, void *user, uint32_t *out_req_id);
	int uv_async_set_region_x_start_cb(uv_async_t *ctx, uint16_t x, uv_async_once_cb cb, void *user, uint32_t *out_req_id);
	int uv_async_set_region_width_cb(uv_async_t *ctx, uint16_t w, uv_async_once_cb cb, void *user, uint32_t *out_req_id);
	int uv_async_set_region_y_start_cb(uv_async_t *ctx, uint16_t y, uv_async_once_cb cb, void *user, uint32_t *out_req_id);
	int uv_async_set_region_height_cb(uv_async_t *ctx, uint16_t h, uv_async_once_cb cb, void *user, uint32_t *out_req_id);
	int uv_async_set_photons_color_cb(uv_async_t *ctx, uint8_t value, uv_async_once_cb cb, void *user, uint32_t *out_req_id);
#ifdef __cplusplus
}
#endif

#endif // UV_API_ASYNC_H