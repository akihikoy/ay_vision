cmake_minimum_required(VERSION 3.0.2)
project(ay_vision_adv)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
find_package(OpenCV 3 REQUIRED)
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

find_package(Eigen3 REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ayv_color_detector ayv_edge_fit ayv_segm_obj ayv_viz_util ayv_rt_pose_estimator
  CATKIN_DEPENDS ay_vision ay_vision_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

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


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


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
# add_dependencies(ayv_viz_util
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
add_dependencies(ayv_viz_util
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
add_dependencies(ayv_viz_util
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



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ay_vision_adv.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
