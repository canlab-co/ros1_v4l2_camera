#include <cstdio>
#include <linux/videodev2.h>
#include <sstream>
#include <string>

#include "usb_cam/camera_driver.h"
#include "usb_cam/converters.h"
#include "usb_cam/types.h"

using namespace usb_cam;

/* STATIC DATA INITIALIZERS */
/* V4L/HARDWARE */
io_method_t AbstractV4LUSBCam::io_method = io_method_t::IO_METHOD_MMAP; // io_
pixel_format_t AbstractV4LUSBCam::pixel_format = PIXEL_FORMAT_UNKNOWN;
color_format_t AbstractV4LUSBCam::color_format = COLOR_FORMAT_UNKNOWN;
int AbstractV4LUSBCam::file_dev = -1;
const time_t AbstractV4LUSBCam::epoch_time_shift_us = util::get_epoch_time_shift_us();

/* FFMPEG */
buffer* AbstractV4LUSBCam::buffers = nullptr;
unsigned int AbstractV4LUSBCam::buffers_count = 0; // n_buffers_
camera_image_t* AbstractV4LUSBCam::image = nullptr;
bool AbstractV4LUSBCam::capturing = false;
std::vector<capture_format_t> AbstractV4LUSBCam::supported_formats = std::vector<capture_format_t>();

/* V4L camera parameters */
bool AbstractV4LUSBCam::streaming_status = false;
std::string AbstractV4LUSBCam::video_device_name = "/dev/video0";
std::string AbstractV4LUSBCam::io_method_name = "mmap";
std::string AbstractV4LUSBCam::pixel_format_name = "uyvy";
unsigned int AbstractV4LUSBCam::v4l_pixel_format = V4L2_PIX_FMT_UYVY;
std::string AbstractV4LUSBCam::color_format_name = "yuv422p";
int AbstractV4LUSBCam::image_width = 1920;
int AbstractV4LUSBCam::image_height = 1080;
int AbstractV4LUSBCam::framerate = 30;
std::vector<camera_control_t> AbstractV4LUSBCam::controls = std::vector<camera_control_t>();
std::set<std::string> AbstractV4LUSBCam::ignore_controls = std::set<std::string>();


bool AbstractV4LUSBCam::init()
{
    // Resolving I/O method name tables
    io_method = util::converters::io_method_from_string(io_method_name);
    if(io_method == IO_METHOD_UNKNOWN)
    {
        printf("Unknown IO method '%s'\n", io_method_name.c_str());
        return false;
    }
    pixel_format = util::converters::pixel_format_from_string(pixel_format_name);
    if(pixel_format == PIXEL_FORMAT_UNKNOWN)
    {
        printf("Unknown pixel format '%s'\n", pixel_format_name.c_str());
        return false;
    }
    color_format = util::converters::color_format_from_string(color_format_name);
    if(color_format == COLOR_FORMAT_UNKNOWN)
    {
        printf("Unknown color format '%s'\n", color_format_name.c_str());
        return false;
    }
    v4l_pixel_format = util::converters::v4l_pixel_format_from_pixel_format(pixel_format);
    if(v4l_pixel_format == UINT_MAX)
    {
        printf("Error in conversion between FFMPEG and Video4Linux pixel format constant '%s'\n", pixel_format_name.c_str());
        return false;
    }

    return true;
}

bool AbstractV4LUSBCam::start()
{
    // V4L initilaization data
    struct stat st;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    unsigned int min;

    /* Creating filesystem handler for streaming device */
    printf("Opening streaming device %s\n", video_device_name.c_str());
    if(stat(video_device_name.c_str(), &st) < 0)
    {
        printf("Cannot identify device by name '%s' (%i)\n", video_device_name.c_str(), errno);
        return false;
    }
    if(!S_ISCHR(st.st_mode))
    {
        printf("'%s' is not a proper V4L device (%i)\n", video_device_name.c_str(), errno);
        return false;
    }
    file_dev = open(video_device_name.c_str(),
                    O_RDWR|O_NONBLOCK,
                    0);
    if(file_dev < 0)
    {
        printf("Cannot create a file handler for V4L device '%s' (%i)\n", video_device_name.c_str(), errno);
        return false;
    }

    /* Initializing V4L capture pipeline */
    if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QUERYCAP), &cap) < 0)
    {
        if(errno == EINVAL)
            printf("File handler created for V4L-incompatible device '%s' (%i)\n", video_device_name.c_str(), errno);
        else
            printf("Cannot query capabilities from V4L device '%s' (%i)\n", video_device_name.c_str(), errno);
        return false;
    }
    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        printf("V4L device '%s' does not support capture mode (%i)\n", video_device_name.c_str(), errno);
        return false;
    }
    if (io_method == io_method_t::IO_METHOD_MMAP)
    {
        if(!(cap.capabilities & V4L2_CAP_STREAMING))
        {
            printf("Device '%s' does not support '%s' access method (streaming error)\n", video_device_name.c_str(), io_method_name.c_str());
            return false;
        }
    }
    else
    {
        printf("Cannot parse access mode for device '%s': '%s', system malfunction expected\n", video_device_name.c_str(), io_method_name.c_str());
    }
    
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = image_width;
    fmt.fmt.pix.height = image_height;
    fmt.fmt.pix.pixelformat = v4l_pixel_format;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_S_FMT), &fmt) < 0)
    {
        printf("Cannot set pixel format '%s' (%u)\n", pixel_format_name.c_str(), v4l_pixel_format);
        return false;
    }
    // Buggy driver prevention
    min = fmt.fmt.pix.width * 2;
    if(fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if(fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;
    image_width = fmt.fmt.pix.width;
    image_height = fmt.fmt.pix.height;

    /* Final frame grabber setup */
    run_grabber(fmt.fmt.pix.sizeimage);

    image = reinterpret_cast<camera_image_t *>(calloc(1, sizeof(camera_image_t)));

    image->width = image_width;
    image->height = image_height;
    image->bytes_per_pixel = 2;  // corrected 11/10/15 (BYTES not BITS per pixel)

    image->image_size = image->width * image->height * image->bytes_per_pixel;
    image->is_new = 0;
    image->image = reinterpret_cast<char *>(calloc(image->image_size, sizeof(char)));
    memset(image->image, 0, image->image_size * sizeof(char));

    return true;
}

AbstractV4LUSBCam::~AbstractV4LUSBCam()
{
    suspend();
    release_device();
    close_handlers();

    if(image)
        free(image);
    image = nullptr;
}

bool AbstractV4LUSBCam::start_capture()
{
    if(streaming_status)
        return false;

    unsigned int i;
    enum v4l2_buf_type type;

    if (io_method == IO_METHOD_MMAP)
    {
        for (i = 0; i < buffers_count; ++i)
        {
            struct v4l2_buffer buf;
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QBUF), &buf) < 0)
            {
                printf("Video4linux: unable to configure stream (%i)\n", errno);
                return false;
            }
        }
    }
    else
    {
        printf("Video4linux: attempt to start stream with unknown I/O method. Dropping request\n");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (usb_cam::util::xioctl(file_dev, VIDIOC_STREAMON, &type) < 0)
    {
        printf("Video4linux: unable to start stream (%i)\n", errno);
        return false;
    }
    streaming_status = true;
    return true;
}

bool AbstractV4LUSBCam::suspend()
{
    if(!streaming_status)
        return false;
    enum v4l2_buf_type type;
    streaming_status = false;
    if (io_method == IO_METHOD_MMAP)
    {
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (usb_cam::util::xioctl(file_dev, VIDIOC_STREAMOFF, &type) < 0)
        {
            printf("Video4linux: cannot stop the device properly (%i)\n", errno);
            return false;
        }
        return true;
    }
    else
    {
        printf("Attempt to stop streaming over unknown I/O channel\n");
        return false;
    }
}

void AbstractV4LUSBCam::release_device()
{
    unsigned int i;
    if (io_method == IO_METHOD_MMAP)
    {
        for (i = 0; i < buffers_count; ++i)
        {
            if (munmap(buffers[i].start, buffers[i].length) < 0)
                printf("Video4linux: unable to deallocate frame buffers\n");
        }
    }
    else
    {
        printf("Attempt to free buffer for unknown I/O method\n");
    }
    free(buffers);
}

void AbstractV4LUSBCam::close_handlers()
{
    int res = close(file_dev);
    file_dev = -1;
    if(res < 0)
        printf("Unable to close device handler properly\n");
}

AbstractV4LUSBCam::AbstractV4LUSBCam()
{
}

camera_image_t *AbstractV4LUSBCam::read_frame()
{
    if((image->width == 0) || (image->height == 0))
        return nullptr;
    
    fd_set fds;
    struct timeval tv;
    int r;
    FD_ZERO(&fds);
    FD_SET(file_dev, &fds);
    // Timeout
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    r = select(file_dev + 1, &fds, nullptr, nullptr, &tv);
    /* if the v4l2_buffer timestamp isn't available use this time, though
     * it may be 10s of milliseconds after the frame acquisition.
     * image->stamp = clock->now(); */
    timespec_get(&image->stamp, TIME_UTC);
    if(r < 0)
    {
        if(errno == EINTR)
            return nullptr;
        printf("Video4linux: frame mapping operation failed (%i)\n", errno);
    }
    else if(r == 0)
    {
        printf("Video4linux: frame mapping timeout (%i)\n", errno);
        return nullptr;
    }

    // Reading the actual frame
    struct v4l2_buffer buf;
    unsigned int i;
    int len;
    struct timespec stamp;
    int64_t buffer_time_us;
    if (io_method == IO_METHOD_MMAP)
    {
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_DQBUF), &buf) < 0)
        {
            if(errno == EAGAIN)
                return nullptr;
            else if(errno == EIO){}
            else
            {
                printf("Memory mapping failure (%i)\n", errno);
                return nullptr;
            }
        }
        image->stamp = util::calc_img_timestamp(buf.timestamp, epoch_time_shift_us);
        timespec_get(&image->stamp, TIME_UTC);
        assert(buf.index < buffers_count);
        len = buf.bytesused;
        // Process image
        if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QBUF), &buf) < 0)
        {
            printf("Unable to exchange buffer with driver (%i)\n", errno);
            return nullptr;
        } 
    }
    else
    {
        printf("Attempt to grab the frame via unknown I/O method (%i)\n", errno);
    }
    bool processing_result = false;
    if(io_method == IO_METHOD_MMAP && v4l_pixel_format == V4L2_PIX_FMT_UYVY)
        memcpy(image->image, buffers[buf.index].start, len);
        processing_result = true;
    if(!processing_result)
    {
        printf("2D processing operation fault\n");
        return nullptr;
    }

    image->encoding = "yuv422";
    image->step = image->width * 2;    
    image->is_new = 1;
    return image;
}

void AbstractV4LUSBCam::run_grabber(unsigned int &buffer_size)
{
    if(io_method == IO_METHOD_MMAP)
    {
        struct v4l2_requestbuffers req;
        CLEAR(req);
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_REQBUFS), &req) < 0)
        {
            if(errno == EINVAL)
                printf("Video4Linux: device '%s' does not support memory mapping (%i)\n", video_device_name.c_str(),  errno);
            else
                printf("Video4Linux: unable to start memory mapping (%i)\n", errno);
            return;
        }
        if(req.count < 2)
        {
            printf("Video4Linux: insufficient memory buffers number (%i)\n", req.count);
            return;
        }
        buffers = reinterpret_cast<buffer *>(calloc(req.count, sizeof(*buffers)));
        if(!buffers)
        {
            printf("Out of memory\n");
            return;
        }
        for (buffers_count = 0; buffers_count < req.count; ++buffers_count)
        {
            struct v4l2_buffer buf;

            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = buffers_count;

            if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QUERYBUF), &buf) < 0)
            {
                printf("Video4Linux: unable to query buffer status (%i)\n", errno);
                return;
            }
            buffers[buffers_count].length = buf.length;
            buffers[buffers_count].start = mmap(NULL,
                                                buf.length,
                                                PROT_READ | PROT_WRITE,
                                                MAP_SHARED, file_dev,
                                                buf.m.offset);
            if (buffers[buffers_count].start == MAP_FAILED)
            {
                printf("Video4Linux: unable to allocate memory (%i)\n", errno);
                return;
            }
        }
    }
    else
    {
        printf("Cannot parse access mode for device '%s': '%s', system malfunction expected\n", video_device_name.c_str(), io_method_name.c_str());
        return;
    }
}

std::vector<capture_format_t> &AbstractV4LUSBCam::get_supported_formats()
{
    supported_formats.clear();
    struct v4l2_fmtdesc current_format;
    current_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    current_format.index = 0;
    for(current_format.index = 0;
         usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_ENUM_FMT), &current_format) == 0;
         current_format.index++)
    {
        struct v4l2_frmsizeenum current_size;
        current_size.index = 0;
        current_size.pixel_format = current_format.pixelformat;

        for(current_size.index = 0;
             usb_cam::util::xioctl(
                 file_dev, static_cast<int>(VIDIOC_ENUM_FRAMESIZES), &current_size) == 0;
             current_size.index++)
        {
            struct v4l2_frmivalenum current_interval;
            current_interval.index = 0;
            current_interval.pixel_format = current_size.pixel_format;
            current_interval.width = current_size.discrete.width;
            current_interval.height = current_size.discrete.height;
            for(current_interval.index = 0;
                 usb_cam::util::xioctl(
                     file_dev, static_cast<int>(VIDIOC_ENUM_FRAMEINTERVALS), &current_interval) == 0;
                 current_interval.index++)
            {
                if(current_interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
                    capture_format_t capture_format;
                    capture_format.format = current_format;
                    capture_format.size = current_size;
                    capture_format.interval = current_interval;
                    supported_formats.push_back(capture_format);
                }
            }  // interval loop
        }  // size loop
    }  // fmt loop
    return supported_formats;
}
