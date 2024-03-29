//-------------------------------------------------------------------------------------------
/*! \file    ros_util.h
    \brief   Basic vision utilities related to ROS.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Oct.19, 2022
*/
//-------------------------------------------------------------------------------------------
#ifndef ros_util_h
#define ros_util_h
//-------------------------------------------------------------------------------------------
#include <cstring>
#include <ros/ros.h>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

// Get the image encoding of a ROS image topic.
// If convert_cv is true, the encoding is converted for OpenCV image conversion.
std::string GetImageEncoding(const std::string &img_topic, ros::NodeHandle &node, bool convert_cv=false, const double &time_out=5.0);
//-------------------------------------------------------------------------------------------

// Get camera projection matrix from ros topic.
void GetCameraProjectionMatrix(const std::string &cam_info_topic, std::string &frame_id, cv::Mat &proj_mat);
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // ros_util_h
//-------------------------------------------------------------------------------------------
