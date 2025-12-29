#ifndef FREE_TYPE_OSD_H
#define FREE_TYPE_OSD_H
#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_GLYPH_H
#include <string>
#include <vector>
#include <unordered_map>
class FreeTypeOSD
{
public:
    FreeTypeOSD();
    ~FreeTypeOSD();
    bool init(const char *font_path, int font_size);
    void deinit();
    bool render_text(const std::string &text, unsigned char *buffer, int buf_width, int buf_height, int buf_stride,
                     int pos_x, int pos_y, unsigned char r, unsigned char g, unsigned char b);
private:
    struct GlyphBitmap
    {
        std::vector<unsigned char> buf; // glyph bitmap data (rows * pitch)
        int width{0};
        int height{0};
        int left{0};
        int top{0};
        int advance{0};
        int pitch{0};
    };

    // key: glyph index (FT_UInt) at current face/size; value: rendered bitmap/metrics
    std::unordered_map<FT_UInt, GlyphBitmap> glyph_cache;

    // Load from cache if hit; otherwise render and store.
    const GlyphBitmap *get_cached_glyph(uint32_t cp);

    FT_Library ft_library;
    FT_Face ft_face;
    int font_size;
};
#endif // FREE_TYPE_OSD_H