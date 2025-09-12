/*
 * Copyright (C) 2022 Rockchip Electronics Co., Ltd.
 * Authors:
 *  Cerf Yu <cerf.yu@rock-chips.com>
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


#ifndef __RGA_SAMPLES_ALLOCATOR_DMA_ALLOC_H__
#define __RGA_SAMPLES_ALLOCATOR_DMA_ALLOC_H__
#include <stddef.h>
#include <cstdint>
#include <im2d.hpp>

#define DMA_HEAP_UNCACHE_PATH           "/dev/dma_heap/system-uncached"
#define DMA_HEAP_PATH                   "/dev/dma_heap/system"
#define DMA_HEAP_DMA32_UNCACHED_PATH    "/dev/dma_heap/system-uncached-dma32"
#define DMA_HEAP_DMA32_PATH             "/dev/dma_heap/system-dma32"
#define CMA_HEAP_UNCACHED_PATH          "/dev/dma_heap/cma-uncached"
#define RV1106_CMA_HEAP_PATH	        "/dev/rk_dma_heap/rk-dma-heap-cma"

typedef struct DMA_Buffer_t
{
    int dma_fd;
    void *ptr;
    uint32_t width;
    uint32_t height;
    uint32_t bpp;//bits per pixel
    uint32_t pitch;
    size_t size;
    rga_buffer_handle_t rga_handle;
    rga_buffer_t rga_buf;
} DMA_Buffer;

int dma_sync_device_to_cpu(int fd);
int dma_sync_cpu_to_device(int fd);

int dma_buf_alloc(const char *path, size_t size, int *fd, void **va);
void dma_buf_free(size_t size, int *fd, void *va);

int get_dma_buf(DMA_Buffer* buf,uint32_t W, uint32_t H, int rk_fmt);
int release_dma_buf(DMA_Buffer* buf);
#endif /* #ifndef __RGA_SAMPLES_ALLOCATOR_DMA_ALLOC_H__ */