#ifndef PNG_META_EXTRACT_H
#define PNG_META_EXTRACT_H

#ifdef __cplusplus
extern "C"
{
#endif

    /*
     * 提供从 PNG 文件中提取在保存阶段写入的文本元数据：
     * 约定键: "Time" / "Gain" / "Photons"
     * 若某键缺失，对应输出置为空字符串或0。
     */

    typedef struct PngMeta_s
    {
        char time[64];    /* 时间戳: YYYY-MM-DD HH:MM:SS +zzzz */
        char gain[32];    /* UV增益数值的字符串形式 */
        char photons[32]; /* 光子计数的字符串形式 */
    } PngMeta;

    /*
     * 解析 PNG 文本元数据。
     * 返回: 0 成功; <0 失败。
     */
    int png_extract_meta(const char *filename, PngMeta *out);

    /* 便捷函数：直接获取 Gain (数值)，缺失时返回0，错误返回 -1 */
    int png_get_gain(const char *filename, unsigned *gain_out);

    /* 便捷函数：直接获取 Photons (数值)，缺失时返回0，错误返回 -1 */
    int png_get_photons(const char *filename, unsigned *photons_out);

#ifdef __cplusplus
}
#endif

#endif /* PNG_META_EXTRACT_H */
