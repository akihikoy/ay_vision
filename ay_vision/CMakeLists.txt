cmake_minimum_required(VERSION 3.0.2)
project(ay_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ay_vision_msgs
  ay_cpp
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

## System dependencies are found with CMake's conventions
find_package(OpenCV 3 REQUIRED)
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

find_package(Eigen3 REQUIRED)



## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ayv_usb_stereo ay_vision ayv_ros
  CATKIN_DEPENDS ay_vision_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


## Declare a C++ library
add_library(ayv_usb_stereo
  src/usb_stereo.cpp
)
target_link_libraries(ayv_usb_stereo
  ${OpenCV_LIBRARIES}
)

add_library(ay_vision
  src/vision_util.cpp
)
target_link_libraries(ay_vision
  ${OpenCV_LIBRARIES}
)

add_library(ayv_ros
  src_ros/ros_util.cpp
)
target_link_libraries(ayv_ros
  ${OpenCV_LIBRARIES}
)


## Install environment is not configured.

## Testing method is not configured.

