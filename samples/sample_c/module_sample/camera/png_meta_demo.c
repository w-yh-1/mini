#include "png_meta_extract.h"
#include <stdio.h>

/*
 * Demo函数：调用 png_extract_meta 解析并打印 PNG 文本元数据。
 * 不提供 main，避免与现有样例程序可执行入口冲突。
 */
void png_meta_demo_run(const char *path)
{
    PngMeta meta;
    int rc = png_extract_meta(path, &meta);
    if (rc != 0)
    {
        printf("[png_meta_demo] extract failed rc=%d\n", rc);
        return;
    }
    printf("[png_meta_demo] PNG: %s\n  Time: %s\n  Gain: %s\n  Photons: %s\n",
           path,
           meta.time[0] ? meta.time : "(none)",
           meta.gain[0] ? meta.gain : "(none)",
           meta.photons[0] ? meta.photons : "(none)");
}
