ay_vision
==================
Computer vision utility.

`ay_vision` is built on ROS, implemented with C++ using OpenCV.

Note: Most of advanced tools were moved to the `ay_vision_adv` package and `ay_vision` contains only common functions.


Author
==================
Akihiko Yamaguchi, http://akihikoy.net/


Requirements
==================
See `manifest.xml`.  OpenCV is used.

Previously this package used OpenCV 2.4.8, 2.4.13.
Currently the target platform is OpenCV 3.x on Ubuntu 18.04.


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



Troubles
==================
Send e-mails to the author.

