# v4l2_camera

A ROS 1 camera driver using Video4Linux2 For Canlab (V4L2).

### System Requirements

Requirements:
  * CANLAB CLEB-G-Series [(GUIDE)](https://can-lab.atlassian.net/wiki/spaces/CANLABGUID/pages/485065636/CLEB-G-Series)
  * CANLAB CLV-G-Series [(GUIDE)](https://can-lab.atlassian.net/wiki/spaces/CANLABGUID/pages/453214214/CLV-G-Series)
  * CANLAB CLMU-G-Series [(GUIDE)](https://can-lab.atlassian.net/wiki/spaces/CANLABGUID/pages/484966555/CLMU-G-Series)
  * [ROS 1 Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

### Download Pacakage
If you need to modify the code or ensure you have the latest update you will need to clone this repo then build the package.

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ git clone --branch noetic https://github.com/canlab-co/ros1_v4l2_camera.git
    $ cd ~/catkin_ws
    $ catkin_make
    $ source ~/catkin_ws/devel/setup.bash

### Usage
Publish camera images, using the parameters:

        # launch the usb_cam executable
        CLV-G-Series : roslaunch usb_cam v4l2_camera_clv.launch

        /* CLEB-G-01A */
        # CLCC-G-01X
        roslaunch usb_cam v4l2_camera_cleb.launch image_size:="[1920, 1080]" cam:={x}
        # CLCC-G-02X
        roslaunch usb_cam v4l2_camera_cleb.launch image_size:="[2048, 1280]" cam:={x}

        /* CLMU-G-Series */
        # CLCC-G-01X
        roslaunch usb_cam v4l2_camera_clmu.launch image_size:="[1920, 1080]" cam:={x}
        # CLCC-G-02X
        roslaunch usb_cam v4l2_camera_clmu.launch image_size:="[2048, 1280]" cam:={x}

>Note: If the number of camera channels you want to use is 3, you can enter cam:=3.

        1CH camera (1 node)
        # run the executable with default settings:        
        rosrun usb_cam usb_cam_node (default : /dev/video0, [1920, 1080])

        # run the executable with customized settings:
        rosrun usb_cam usb_cam_node _video_device:=/dev/video{x} _image_width:={w} _image_height:={h}

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
    Currently supported:  
    CLMU-G-Series - `[1920, 1080]` `[2048, 1280]`  
    CLEB-G-01A - `[1920, 1080]` `[2048, 1280]`

* `cam` - `integer`, default: `6`

    The number of camera channels.  
    Currently supported:  
    CLMU-G-Series(default: `6`)  
    CLEB-G-01A(default: `4`)

* Camera Control Parameters

    Not Support
