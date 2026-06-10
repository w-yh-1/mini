#include "png_meta_extract.h"
#include <png.h>
#include <stdio.h>
#include <string.h>

static void _init_meta(PngMeta *m)
{
    if (!m)
        return;
    m->time[0] = '\0';
    m->gain[0] = '\0';
    m->photons[0] = '\0';
}

int png_extract_meta(const char *filename, PngMeta *out)
{
    if (!filename || !out)
        return -1;
    FILE *fp = fopen(filename, "rb");
    if (!fp)
        return -2;

    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr)
    {
        fclose(fp);
        return -3;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr)
    {
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        fclose(fp);
        return -4;
    }

    if (setjmp(png_jmpbuf(png_ptr)))
    {
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fp);
        return -5; /* libpng error */
    }

    png_init_io(png_ptr, fp);
    png_read_info(png_ptr, info_ptr);

    _init_meta(out);

    png_textp text_ptr = NULL;
    int num_text = 0;
    png_get_text(png_ptr, info_ptr, &text_ptr, &num_text);
    for (int i = 0; i < num_text; ++i)
    {
        if (!text_ptr[i].key || !text_ptr[i].text)
            continue;
        if (strcmp(text_ptr[i].key, "Time") == 0)
        {
            strncpy(out->time, text_ptr[i].text, sizeof(out->time) - 1);
            out->time[sizeof(out->time) - 1] = '\0';
        }
        else if (strcmp(text_ptr[i].key, "Gain") == 0)
        {
            strncpy(out->gain, text_ptr[i].text, sizeof(out->gain) - 1);
            out->gain[sizeof(out->gain) - 1] = '\0';
        }
        else if (strcmp(text_ptr[i].key, "Photons") == 0)
        {
            strncpy(out->photons, text_ptr[i].text, sizeof(out->photons) - 1);
            out->photons[sizeof(out->photons) - 1] = '\0';
        }
    }

    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    fclose(fp);
    return 0;
}

static int _parse_uint(const char *s, unsigned *val)
{
    if (!s || !*s)
    {
        if (val)
            *val = 0;
        return 0;
    }
    unsigned v = 0;
    for (const char *p = s; *p; ++p)
    {
        if (*p < '0' || *p > '9')
            return -1; /* 非纯数字，认为格式错误 */
        v = v * 10 + (unsigned)(*p - '0');
    }
    if (val)
        *val = v;
    return 0;
}

int png_get_gain(const char *filename, unsigned *gain_out)
{
    if (gain_out)
        *gain_out = 0;
    PngMeta meta;
    int rc = png_extract_meta(filename, &meta);
    if (rc != 0)
        return rc;
    if (_parse_uint(meta.gain, gain_out) != 0)
        return -6;
    return 0;
}

int png_get_photons(const char *filename, unsigned *photons_out)
{
    if (photons_out)
        *photons_out = 0;
    PngMeta meta;
    int rc = png_extract_meta(filename, &meta);
    if (rc != 0)
        return rc;
    if (_parse_uint(meta.photons, photons_out) != 0)
        return -6;
    return 0;
}
