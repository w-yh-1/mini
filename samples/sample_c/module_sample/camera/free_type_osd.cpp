#include"free_type_osd.h"
#include <iostream>
#include <vector>
#include <cstring>

// 简单 UTF-8 解码，仅支持合法 UTF-8，非法字节将被跳过。
static void utf8_to_codepoints(const std::string &text, std::vector<uint32_t> &out)
{
    out.clear();
    size_t i = 0;
    const size_t n = text.size();
    while (i < n)
    {
        unsigned char c = static_cast<unsigned char>(text[i]);
        if (c < 0x80)
        {
            out.push_back(c);
            ++i;
        }
        else if ((c >> 5) == 0x6 && i + 1 < n)
        {
            uint32_t cp = ((c & 0x1F) << 6) | (static_cast<unsigned char>(text[i + 1]) & 0x3F);
            out.push_back(cp);
            i += 2;
        }
        else if ((c >> 4) == 0xE && i + 2 < n)
        {
            uint32_t cp = ((c & 0x0F) << 12) |
                          ((static_cast<unsigned char>(text[i + 1]) & 0x3F) << 6) |
                          (static_cast<unsigned char>(text[i + 2]) & 0x3F);
            out.push_back(cp);
            i += 3;
        }
        else if ((c >> 3) == 0x1E && i + 3 < n)
        {
            uint32_t cp = ((c & 0x07) << 18) |
                          ((static_cast<unsigned char>(text[i + 1]) & 0x3F) << 12) |
                          ((static_cast<unsigned char>(text[i + 2]) & 0x3F) << 6) |
                          (static_cast<unsigned char>(text[i + 3]) & 0x3F);
            out.push_back(cp);
            i += 4;
        }
        else
        {
            // 跳过非法字节
            ++i;
        }
    }
}
FreeTypeOSD::FreeTypeOSD() : ft_library(nullptr), ft_face(nullptr), font_size(16)
{
}   
FreeTypeOSD::~FreeTypeOSD()
{
    deinit();
}
bool FreeTypeOSD::init(const char *font_path, int font_size)
{
    this->font_size = font_size;
    if (FT_Init_FreeType(&ft_library))
    {
        std::cerr << "Could not init FreeType library" << std::endl;
        return false;
    }
    if (FT_New_Face(ft_library, font_path, 0, &ft_face))
    {
        std::cerr << "Could not open font: " << font_path << std::endl;
        return false;
    }
    FT_Set_Pixel_Sizes(ft_face, 0, font_size);
    return true;
}
void FreeTypeOSD::deinit()
{
    glyph_cache.clear();
    if (ft_face)
    {
        FT_Done_Face(ft_face);
        ft_face = nullptr;
    }   
    if (ft_library)
    {
        FT_Done_FreeType(ft_library);
        ft_library = nullptr;
    }
}
bool FreeTypeOSD::render_text(const std::string &text, unsigned char *buffer, int buf_width, int buf_height, int buf_stride,
                             int pos_x, int pos_y, unsigned char r, unsigned char g, unsigned char b)
{
    if (!ft_face || !buffer)
        return false;
    int x = pos_x;
    int y = pos_y;

    std::vector<uint32_t> codepoints;
    utf8_to_codepoints(text, codepoints);

    FT_UInt prev_index = 0;
    const bool has_kerning = FT_HAS_KERNING(ft_face);

    for (uint32_t cp : codepoints)
    {
        FT_UInt glyph_index = FT_Get_Char_Index(ft_face, cp);
        if (has_kerning && prev_index && glyph_index)
        {
            FT_Vector delta;
            FT_Get_Kerning(ft_face, prev_index, glyph_index, FT_KERNING_DEFAULT, &delta);
            x += delta.x >> 6; // 26.6 固定小数转整数
        }

        const GlyphBitmap *gb = get_cached_glyph(cp);
        if (!gb)
        {
            prev_index = 0;
            continue;
        }
        prev_index = glyph_index;

        const int bmp_rows = gb->height;
        const int bmp_cols = gb->width;
        const int bmp_pitch = gb->pitch;
        const unsigned char *bmp_buf = gb->buf.data();

        for (int row = 0; row < bmp_rows; ++row)
        {
            for (int col = 0; col < bmp_cols; ++col)
            {
                int buf_x = x + gb->left + col;
                int buf_y = y - gb->top + row;
                if (buf_x < 0 || buf_x >= buf_width || buf_y < 0 || buf_y >= buf_height)
                    continue;
                unsigned char alpha = bmp_buf[row * bmp_pitch + col];
                if (alpha == 0)
                    continue;
                unsigned char *pixel = buffer + buf_y * buf_stride + buf_x * 3;
                pixel[0] = (alpha * r + (255 - alpha) * pixel[0]) / 255;
                pixel[1] = (alpha * g + (255 - alpha) * pixel[1]) / 255;
                pixel[2] = (alpha * b + (255 - alpha) * pixel[2]) / 255;
            }
        }
        x += gb->advance;
    }
    return true;
}

// 返回缓存中的 glyph；若未命中则渲染并插入缓存。
const FreeTypeOSD::GlyphBitmap *FreeTypeOSD::get_cached_glyph(uint32_t cp)
{
    if (!ft_face)
        return nullptr;

    FT_UInt glyph_index = FT_Get_Char_Index(ft_face, cp);
    if (!glyph_index)
        return nullptr;

    auto it = glyph_cache.find(glyph_index);
    if (it != glyph_cache.end())
        return &it->second;

    if (FT_Load_Char(ft_face, cp, FT_LOAD_RENDER))
    {
        return nullptr;
    }
    FT_GlyphSlot glyph = ft_face->glyph;

    GlyphBitmap gb;
    gb.width = glyph->bitmap.width;
    gb.height = glyph->bitmap.rows;
    gb.left = glyph->bitmap_left;
    gb.top = glyph->bitmap_top;
    gb.advance = (glyph->advance.x >> 6);
    gb.pitch = glyph->bitmap.pitch;

    if (gb.width > 0 && gb.height > 0 && glyph->bitmap.buffer)
    {
        gb.buf.resize(gb.height * gb.pitch);
        // pitch 可能大于 width，按 pitch 行拷贝
        for (int row = 0; row < gb.height; ++row)
        {
            const unsigned char *src = glyph->bitmap.buffer + row * gb.pitch;
            unsigned char *dst = gb.buf.data() + row * gb.pitch;
            memcpy(dst, src, gb.pitch);
        }
    }
    auto inserted = glyph_cache.emplace(glyph_index, std::move(gb));
    return inserted.first != glyph_cache.end() ? &inserted.first->second : nullptr;
}
