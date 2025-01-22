cmake_minimum_required(VERSION 3.0.2)
project(ay_vision_adv)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ay_vision_msgs
  ay_vision
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
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 REQUIRED)
endif()
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

find_package(Eigen3 REQUIRED)


## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ayv_color_detector ayv_edge_fit ayv_segm_obj ayv_viz_util ayv_rt_pose_estimator
  CATKIN_DEPENDS ay_vision ay_vision_msgs
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
add_library(ayv_color_detector
  src/color_detector.cpp
)
target_link_libraries(ayv_color_detector
  ${OpenCV_LIBRARIES}
)
add_library(ayv_edge_fit
  src/edge_fit.cpp
)
target_link_libraries(ayv_edge_fit
  ${OpenCV_LIBRARIES}
)
# NOTE: flow_finder is temporary removed as it uses legacy OpenCV functions (LK optical flow).
# add_library(ayv_flow_finder
#   src/flow_finder.cpp
# )
# target_link_libraries(ayv_flow_finder
#   ${OpenCV_LIBRARIES}
# )
add_library(ayv_segm_obj
  src/segm_obj.cpp
)
target_link_libraries(ayv_segm_obj
  ${OpenCV_LIBRARIES}
)

add_library(ayv_viz_util
  src_ros/viz_util.cpp
)
add_dependencies(ayv_viz_util
  ${ay_vision_msgs_EXPORTED_TARGETS}
)
target_link_libraries(ayv_viz_util
  ${OpenCV_LIBRARIES}
)

file(GLOB raytrace_srcs1 "include/ay_vision/3rdparty/doncross/*.cpp")
file(GLOB raytrace_srcs2 "include/ay_vision/3rdparty/lodepng/*.cpp")
add_library(ayv_rt_pose_estimator
  src/rt_pose_estimator.cpp
  src/rt_depthscene.cpp
  ${raytrace_srcs1}
  ${raytrace_srcs2}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
## Rename C++ executable without prefix
## Add cmake target dependencies of the executable
## Specify libraries to link a library or executable target against

# NOTE: Commented out since it uses flow_finder.
# add_executable(${PROJECT_NAME}_color_detector_node
#   src_ros/color_detector_node.cpp
# )
# add_dependencies(${PROJECT_NAME}_color_detector_node
#   ${ay_vision_msgs_EXPORTED_TARGETS}
# )
# set_target_properties(${PROJECT_NAME}_color_detector_node PROPERTIES OUTPUT_NAME color_detector_node PREFIX "")
# target_link_libraries(${PROJECT_NAME}_color_detector_node
#   ayv_color_detector
#   ayv_flow_finder
#   ay_vision
#   ay_cpp
#   ${OpenCV_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

add_executable(${PROJECT_NAME}_cv_usb_node
  src_ros/cv_usb_node.cpp
)
set_target_properties(${PROJECT_NAME}_cv_usb_node PROPERTIES OUTPUT_NAME cv_usb_node PREFIX "")
target_link_libraries(${PROJECT_NAME}_cv_usb_node
  ay_vision
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  boost_system
)

add_executable(${PROJECT_NAME}_disp_rostime
  src_ros/disp_rostime.cpp
)
set_target_properties(${PROJECT_NAME}_disp_rostime PROPERTIES OUTPUT_NAME disp_rostime PREFIX "")
target_link_libraries(${PROJECT_NAME}_disp_rostime
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_segm_obj_node
  src_ros/segm_obj_node.cpp
)
add_dependencies(${PROJECT_NAME}_segm_obj_node
  ${ay_vision_msgs_EXPORTED_TARGETS}
)
set_target_properties(${PROJECT_NAME}_segm_obj_node PROPERTIES OUTPUT_NAME segm_obj_node PREFIX "")
target_link_libraries(${PROJECT_NAME}_segm_obj_node
  ayv_segm_obj
  ayv_viz_util
  ay_vision
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_record_video_node
  src_ros/record_video_node.cpp
)
add_dependencies(${PROJECT_NAME}_record_video_node
  ${ay_vision_msgs_EXPORTED_TARGETS}
)
set_target_properties(${PROJECT_NAME}_record_video_node PROPERTIES OUTPUT_NAME record_video_node PREFIX "")
target_link_libraries(${PROJECT_NAME}_record_video_node
  ay_vision
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  boost_thread
  boost_system
)


## Install environment is not configured.

## Testing method is not configured.

