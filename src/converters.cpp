#include "usb_cam/converters.h"
#include "usb_cam/types.h"
#include "usb_cam/util.h"
#include <linux/videodev2.h>

namespace usb_cam
{

namespace util
{

namespace converters
{

io_method_t io_method_from_string(const std::string & str)
{
    if (str == "mmap")
        return io_method_t::IO_METHOD_MMAP;
    else
        return io_method_t::IO_METHOD_UNKNOWN;
}

pixel_format_t pixel_format_from_string(const std::string & str)
{
    if (str == "uyvy")
        return pixel_format_t::PIXEL_FORMAT_UYVY;
    else
        return pixel_format_t::PIXEL_FORMAT_UNKNOWN;
}

color_format_t color_format_from_string(const std::string & str)
{
    if (str == "yuv422p")
        return color_format_t::COLOR_FORMAT_YUV422P;
    else
        return color_format_t::COLOR_FORMAT_UNKNOWN;
}

unsigned int v4l_pixel_format_from_pixel_format(const pixel_format_t& pixelformat)
{
    switch(pixelformat)
    {
    case PIXEL_FORMAT_UYVY:
        return V4L2_PIX_FMT_UYVY;
    default:
        return UINT_MAX;
    }
}

std::string pixel_format_to_string(const uint32_t & pixelformat)
{
    switch (pixelformat)
    {
    case pixel_format_t::PIXEL_FORMAT_UYVY:
        return "uyvy";
    default:
        return "unknown";
    }
}

} // namespace converters

} // namespace util

} // namespace usb_cam
