# v4l2_camera

A ROS 1 camera driver using Video4Linux2 For Canlab (V4L2).

### System Requirements

Requirements:
  * CANLAB CLEB-G-01A [(GUIDE)](https://can-lab.atlassian.net/wiki/spaces/CANLABGUID/pages/486768641/CLEB-G-01A+User+guide)
  * CANLAB CLV-G-Series [(GUIDE)](https://can-lab.atlassian.net/wiki/spaces/CANLABGUID/pages/459735068/CLV-G-Series+User+guide)
  * [ROS 1 Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

### Download Pacakage
If you need to modify the code or ensure you have the latest update you will need to clone this repo then build the package.

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ git clone --branch noetic https://github.com/canlab-co/ros1_v4l2_camera.git
    $ cd ~/catkin_ws
    $ catkin_make
    $ source devel/setup.bash

### Usage
Publish camera images, using the default parameters:

        # launch the usb_cam executable
        CLEB-G-01A : roslaunch usb_cam v4l2_camera_cleb.launch
        CLV-G-Series : roslaunch usb_cam v4l2_camera_clv.launch
        
        # run the executable with default settings:        
        1CH : rosrun usb_cam usb_cam_node (default : /dev/video0)
Preview the image (open another terminal):

        rosrun rqt_image_view rqt_image_view

## Nodes

### usb_cam_node

The `usb_cam_node` interfaces with standard V4L2 devices and
publishes images as `sensor_msgs/Image` messages.

#### Published Topics

* `/image_raw` - `sensor_msgs/Image`

    The image.

#### Parameters

* `video_device` - `string`, default: `"/dev/video0"`

    The device the camera is on.

* `pixel_format` - `string`, default: `"UYVY"`

    The pixel format to request from the camera. Must be a valid four
    character '[FOURCC](http://fourcc.org/)' code [supported by
    V4L2](https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/videodev.html)
    and by your camera. The node outputs the available formats
    supported by your camera when started.  
    Currently supported: `"UYVY"`

* `output_encoding` - `string`, default: `"yuv422"`

    The encoding to use for the output image.  
    Currently supported: `"yuv422"`.  
  
* `image_size` - `integer_array`, default: `[1920, 1080]`

    Width and height of the image.

* Camera Control Parameters

    Not Support
