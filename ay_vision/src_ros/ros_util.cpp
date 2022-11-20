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
#include <sensor_msgs/CameraInfo.h>
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


// Get camera projection matrix from ros topic.
void GetCameraProjectionMatrix(const std::string &cam_info_topic, std::string &frame_id, cv::Mat &proj_mat)
{
  ros::NodeHandle node("~");
  boost::shared_ptr<sensor_msgs::CameraInfo const> ptr_cam_info;
  sensor_msgs::CameraInfo cam_info;
  ptr_cam_info= ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic, node);
  if(ptr_cam_info==NULL)
  {
    std::cerr<<"Failed to get camera info from the topic: "<<cam_info_topic<<std::endl;
    return;
  }
  cam_info= *ptr_cam_info;

  // std::cerr<<"cam_info: "<<cam_info<<std::endl;
  frame_id= cam_info.header.frame_id;
  // cv::Mat proj_mat(3,4, CV_64F, cam_info.P);
  proj_mat.create(3,4, CV_64F);
  for(int r(0),i(0);r<3;++r)
    for(int c(0);c<4;++c,++i)
      proj_mat.at<double>(r,c)= cam_info.P[i];
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

