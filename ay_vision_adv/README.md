ay_vision_adv
==================
Computer vision tools, including color detection, object segmentation, etc.

`ay_vision_adv` is built on ROS, implemented with C++ using OpenCV.

Originally the most of modules are contained in the `ay_vision` package, then moved here in order to make `ay_vision` simple.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Acknowledgment
==================
Ray tracing library (include/ay_vision_adv/3rdparty/doncross) is implemented by Don Cross.
http://www.cosinekitty.com/raytrace/

PNG library LodePNG (include/ay_vision_adv/3rdparty/lodepng) is implemented by Lode Vandevenne.


Requirements
==================
See `manifest.xml`.  OpenCV is used.

Previously this package used OpenCV 2.4.8, 2.4.13.
Currently the target platform is OpenCV 3.x on Ubuntu 18.04.


Note
=================
`flow_finder` is temporary excluded in the compile list as it uses legacy OpenCV functions (LK optical flow).  Accordingly, the programs using `flow_finder` are excluded: `color_detector_node`.



Directories
==================

include/ay_vision_adv
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
Build `ay_vision_adv` with `rosmake`.

```
$ rosmake ay_vision_adv
```

After `rosmake`, you will find some executables in `bin/` directory and shared objects in `lib/` directory.
There will be some directories made by `rosmake`.


Usage
==================

color_detector
---------------------------
Multiple color detector.  It can detect colored objects, and flow.

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

