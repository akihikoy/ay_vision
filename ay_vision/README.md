ay_vision
==================
Computer vision tools, including color detection, FingerVision processing programs, object segmentation, etc.

`ay_vision` is built on ROS, implemented with C++ using OpenCV.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Acknowledgment
==================
Ray tracing library (include/ay_vision/3rdparty/doncross) is implemented by Don Cross.
http://www.cosinekitty.com/raytrace/

PNG library LodePNG (include/ay_vision/3rdparty/lodepng) is implemented by Lode Vandevenne.


Requirements
==================
See `manifest.xml`.  OpenCV is used.

Previously this package depended on OpenCV 2.4.13, but currently this can be compiled with OpenCV 2.4.8.
You can install it from the Ubuntu repository if your Ubuntu version is 14.04 or 16.04.


Directories
==================

include/ay_vision
----------------------------
ROS-independent header files (C++).

src
----------------------------
ROS-independent source files (C++).

src_ros
----------------------------
ROS-dependent source files (C++).  Mainly, programs of ROS nodes are contained.


Build
==================
The repository directory should be in ROS workspace (e.g. ~/ros_ws/).
Build `ay_vision` with `rosmake`.

```
$ rosmake ay_vision
```

After `rosmake`, you will find some executables in `bin/` directory and shared objects in `lib/` directory.
There will be some directories made by `rosmake`.


Usage
==================

color_detector
---------------------------
Multiple color detector.  It can detect colored objects, and flow.

visual_skin
---------------------------
The processing programs of FingerVision are moved to a new package `fingervision`.
Use `fv_core_node` in `fingervision` instead of `visual_skin`.

The former launch and configuration files of `visual_skin` are moved to the `ay_fv_extra` package, which is a launcher of `fingervision`.

cv_usb
---------------------------
USB camera for ROS.

disp_rostime
---------------------------
Displaying ROS time.

segm_obj
---------------------------
Segmenting an object which should be placed on a white background.


Troubles
==================
Send e-mails to the author.

