cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# ++++OPENCV-2.x++++
find_package(OpenCV 2 REQUIRED)
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
# ++++OPENCV-2.4.13++++
# # NOTE: Install OpenCV 2.4.13 on ~/.local from source which includes cv::fisheye
# # We don't use opencv found by any prior running of find_package
# unset(OpenCV_CONFIG_PATH CACHE)
# unset(OpenCV_DIR CACHE)
# set(TMP_PREFIX_PATH ${CMAKE_PREFIX_PATH})
# set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local")
# find_package(OpenCV 2.4.13 REQUIRED)
# # restore CMAKE_PREFIX_PATH and other cached variables
# # in order to avoid other packages find this opencv.
# set(CMAKE_PREFIX_PATH ${TMP_PREFIX_PATH})
# unset(OpenCV_CONFIG_PATH CACHE)
# unset(OpenCV_DIR CACHE)
# message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
# message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
# ----OPENCV----

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#Building libraries:

rosbuild_add_library(ayv_blob_tracker2
      src/blob_tracker2.cpp
      )
target_link_libraries(ayv_blob_tracker2
      ${OpenCV_LIBRARIES}
      )
rosbuild_add_library(ayv_color_detector
      src/color_detector.cpp
      )
target_link_libraries(ayv_color_detector
      ${OpenCV_LIBRARIES}
      )
rosbuild_add_library(ayv_edge_fit
      src/edge_fit.cpp
      )
target_link_libraries(ayv_edge_fit
      ${OpenCV_LIBRARIES}
      )
rosbuild_add_library(ayv_flow_finder
      src/flow_finder.cpp
      )
target_link_libraries(ayv_flow_finder
      ${OpenCV_LIBRARIES}
      )
rosbuild_add_library(ayv_prox_vision
      src/prox_vision.cpp
      )
target_link_libraries(ayv_prox_vision
      ${OpenCV_LIBRARIES}
      )
rosbuild_add_library(ayv_segm_obj
      src/segm_obj.cpp
      )
target_link_libraries(ayv_segm_obj
      ${OpenCV_LIBRARIES}
      )
rosbuild_add_library(ayv_usb_stereo
      src/usb_stereo.cpp
      )
target_link_libraries(ayv_usb_stereo
      ${OpenCV_LIBRARIES}
      )
target_link_libraries(ayv_prox_vision
      ${OpenCV_LIBRARIES}
      )
rosbuild_add_library(ay_vision
      src/vision_util.cpp
      )
target_link_libraries(ay_vision
      ${OpenCV_LIBRARIES}
      )

file(GLOB raytrace_srcs1 "include/ay_vision/3rdparty/doncross/*.cpp")
file(GLOB raytrace_srcs2 "include/ay_vision/3rdparty/lodepng/*.cpp")
rosbuild_add_library(ayv_rt_pose_estimator
      src/rt_pose_estimator.cpp
      src/rt_depthscene.cpp
      ${raytrace_srcs1}
      ${raytrace_srcs2})


#Building executables:

# NOTE: Adding libcv_bridge.so is a quick hack.
# In manifest.xml cv_bridge is omitted.
# If we put cv_bridge as a dependency in manifest.xml,
# both opencv 2.4.8 and 2.4.13 will be linked simultaneously,
# which causes a compile issue.

rosbuild_add_executable(color_detector_node
      src_ros/color_detector_node.cpp
      )
target_link_libraries(color_detector_node
      ayv_color_detector
      ayv_flow_finder
      ay_vision
      ay_cpp
      ${OpenCV_LIBRARIES}
      )

rosbuild_add_executable(cv_usb_node
      src_ros/cv_usb_node.cpp
      )
target_link_libraries(cv_usb_node
      ay_vision
      ${OpenCV_LIBRARIES}
      libcv_bridge.so
      boost_system
      )

rosbuild_add_executable(disp_rostime
      src_ros/disp_rostime.cpp
      )
target_link_libraries(disp_rostime
      ${OpenCV_LIBRARIES}
      )

rosbuild_add_executable(segm_obj_node
      src_ros/segm_obj_node.cpp
      src_ros/viz_util.cpp
      )
target_link_libraries(segm_obj_node
      ayv_segm_obj
      ay_vision
      ${OpenCV_LIBRARIES}
      libcv_bridge.so
      )

rosbuild_add_executable(visual_skin_node2
      src_ros/visual_skin_node2.cpp
      )
target_link_libraries(visual_skin_node2
      ayv_blob_tracker2
      ayv_prox_vision
      ay_vision
      ${OpenCV_LIBRARIES}
      boost_thread
      boost_system
      )
