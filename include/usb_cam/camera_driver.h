#ifndef USB_CAM_CAMERA_DRIVER_H
#define USB_CAM_CAMERA_DRIVER_H

#include <string>
#include <iostream>
#include <algorithm>
#include <set>
extern "C"
{
// Legacy FFMPEG workaround
#include <libavcodec/version.h>
#if LIBAVCODEC_VERSION_MAJOR < 55
    #define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
#endif
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#define __STDC_CONSTANT_MACROS  // Required for libavutil
#include <libavutil/imgutils.h>
#include <libavutil/frame.h>
#include <libavutil/mem.h>
}

#include <fcntl.h>  // for O_* constants
#include <unistd.h>  // for getpagesize()
#include <malloc.h>  // for memalign
#include <sys/mman.h>  // for mmap
#include <sys/stat.h>  // for stat

#include <opencv2/imgproc.hpp>

#include "usb_cam/types.h"
#include "usb_cam/util.h"
#include "usb_cam/converters.h"

namespace usb_cam
{

class AbstractV4LUSBCam
{
protected:
    /* V4L/HARDWARE */
    static io_method_t io_method; // io_
    static pixel_format_t pixel_format;
    static color_format_t color_format;
    static int file_dev; // fd_
    static const time_t epoch_time_shift_us;

    /* FFMPEG */
    static buffer* buffers;
    static unsigned int buffers_count; // n_buffers_
    camera_image_t _image;
    static camera_image_t * image;
    static bool capturing;
    static std::vector<capture_format_t> supported_formats;

    /* V4L camera parameters */
    static bool streaming_status;
    static std::string video_device_name;
    static std::string io_method_name;
    static std::string pixel_format_name;
    static unsigned int v4l_pixel_format;
    static std::string color_format_name;
    static int image_width;
    static int image_height;
    static int framerate;
    static std::vector<camera_control_t>controls;
    static std::set<std::string> ignore_controls;

    /* Internal functions */
    static bool init();
    static bool start();
    static void run_grabber(unsigned int& buffer_size);
    static bool start_capture();
    static camera_image_t* read_frame();
    static bool suspend();
    static void release_device();
    static void close_handlers();
    AbstractV4LUSBCam();
    AbstractV4LUSBCam(const AbstractV4LUSBCam& root) = delete;

public:
    virtual ~AbstractV4LUSBCam();
    static std::vector<capture_format_t>& get_supported_formats();
};

} // namespace usb_cam

#endif // USB_CAM_CAMERA_DRIVER_H
