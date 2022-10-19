//-------------------------------------------------------------------------------------------
/*! \file    ros_util.cpp
    \brief   Basic vision utilities related to ROS.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Oct.19, 2022
*/
//-------------------------------------------------------------------------------------------
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//-------------------------------------------------------------------------------------------
#include "ay_vision/ros_util.h"
//-------------------------------------------------------------------------------------------
namespace trick
{


// Get the image encoding of a ROS image topic.
// If convert_cv is true, the encoding is converted for OpenCV image conversion.
std::string GetImageEncoding(const std::string &img_topic, ros::NodeHandle &node, bool convert_cv, const double &time_out)
{
  sensor_msgs::ImageConstPtr ptr_img_header;
  ptr_img_header= ros::topic::waitForMessage<sensor_msgs::Image>(img_topic, node, ros::Duration(time_out));
  if(ptr_img_header==NULL)
  {
    std::cerr<<"Failed to receive the image topic: "<<img_topic<<std::endl;
    return "";
  }
  const std::string &encoding(ptr_img_header->encoding);
  if(!convert_cv)  return encoding;
  if(encoding=="rgb8")  return "bgr8";
  if(encoding=="RGB8")  return "BGR8";
  // TODO: Add more conversion if necessary.
  return encoding;
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

