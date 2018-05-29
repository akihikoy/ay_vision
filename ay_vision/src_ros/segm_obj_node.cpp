//-------------------------------------------------------------------------------------------
/*! \file    segm_obj_node.cpp
    \brief   certain c++ source file
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.17, 2017
*/
//-------------------------------------------------------------------------------------------
#include "ay_vision/segm_obj.h"
#include "ay_vision/vision_util.h"
#include "viz_util.h"
//-------------------------------------------------------------------------------------------
#include "ay_vision_msgs/SegmObj.h"
#include "ay_vision_msgs/ColDetViz.h"
#include "ay_vision_msgs/ColDetVizPrimitive.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/image_encodings.h>
//-------------------------------------------------------------------------------------------
namespace trick
{

bool Running(true);
bool ShowTrackbars(false);
TObjectDetector ObjDetector;
std::vector<ay_vision_msgs::ColDetVizPrimitive> VizObjs;  // External visualization request.
std::vector<ay_vision_msgs::ColDetVizPrimitive> MaskObjs;  // Mask request.
ros::Publisher SegmObjPub;

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

/*
  Right click: pause/resume
*/
void OnMouse(int event, int x, int y, int flags, void *data)
{
  if(event!=0)
  {
  }

  // if(flags!=0)  return;
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  if(event == cv::EVENT_LBUTTONDOWN)
  {
  }
}
//-------------------------------------------------------------------------------------------

// return if continue
bool HandleKeyEvent()
{
  // keyboard interface:
  char c(cv::waitKey(1));
  if(c=='\x1b'||c=='q') return false;
  else if(c==' ')
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  else if(c=='C')
  {
    ShowTrackbars= !ShowTrackbars;
    if(ShowTrackbars)
    {
      // For white detector:
      cv::createTrackbar("white_s_max", "segmented", &ObjDetector.Params().WhiteSMax, 255, NULL);
      cv::createTrackbar("white_v_min", "segmented", &ObjDetector.Params().WhiteVMin, 255, NULL);
      cv::createTrackbar("n_dilate1", "segmented", &ObjDetector.Params().NDilate1, 10, NULL);
      cv::createTrackbar("n_erode1", "segmented", &ObjDetector.Params().NErode1, 10, NULL);

      // For objects-on-white detector
      cv::createTrackbar("thresh_s", "segmented", &ObjDetector.Params().ThreshS, 255, NULL);
      cv::createTrackbar("thresh_v", "segmented", &ObjDetector.Params().ThreshV, 255, NULL);
      cv::createTrackbar("n_dilate2", "segmented", &ObjDetector.Params().NDilate2, 10, NULL);
      cv::createTrackbar("n_erode2", "segmented", &ObjDetector.Params().NErode2, 10, NULL);
      cv::createTrackbar("rect_len_min", "segmented", &ObjDetector.Params().RectLenMin, 600, NULL);
      cv::createTrackbar("rect_len_max", "segmented", &ObjDetector.Params().RectLenMax, 600, NULL);
    }
    else
    {
      cv::destroyWindow("segmented");
      cv::namedWindow("segmented",1);
      cv::setMouseCallback("segmented", OnMouse, NULL);
    }
  }

  return true;
}
//-------------------------------------------------------------------------------------------

bool Pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Paused..."<<std::endl;
  Running= false;
  return true;
}
//-------------------------------------------------------------------------------------------

bool Resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resumed..."<<std::endl;
  Running= true;
  return true;
}
//-------------------------------------------------------------------------------------------

void ColDetVizCallback(const ay_vision_msgs::ColDetVizConstPtr &msg)
{
  VizObjs= msg->objects;
}
//-------------------------------------------------------------------------------------------

void ColDetMaskCallback(const ay_vision_msgs::ColDetVizConstPtr &msg)
{
  MaskObjs= msg->objects;
}
//-------------------------------------------------------------------------------------------

void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(!Running)
  {
    if(!HandleKeyEvent())  ros::shutdown();
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat frame= cv_ptr->image;
  DrawColDetViz(frame, MaskObjs);


  ObjDetector.Step(frame);

  {
    ay_vision_msgs::SegmObj msg2;
    msg2.header= msg->header;
    msg2.height= msg->height;
    msg2.width= msg->width;
    msg2.num_points.resize(ObjDetector.Contours().size());
    int total_points(0);
    std::vector<int>::iterator m2npitr(msg2.num_points.begin());
    for(std::vector<std::vector<cv::Point> >::const_iterator citr(ObjDetector.Contours().begin()),
        citr_end(ObjDetector.Contours().end()); citr!=citr_end; ++citr,++m2npitr)
    {
      total_points+= citr->size();
      *m2npitr= citr->size();
    }
    msg2.contours.resize(total_points*2);
    std::vector<int>::iterator m2citr(msg2.contours.begin());
    for(std::vector<std::vector<cv::Point> >::const_iterator citr(ObjDetector.Contours().begin()),
        citr_end(ObjDetector.Contours().end()); citr!=citr_end; ++citr)
    {
      for(std::vector<cv::Point>::const_iterator pitr(citr->begin()),pitr_end(citr->end());
          pitr!=pitr_end; ++pitr)
      {
        *m2citr= pitr->x; ++m2citr;
        *m2citr= pitr->y; ++m2citr;
      }
    }
    SegmObjPub.publish(msg2);
  }

  cv::Mat img_disp;
  img_disp= 0.6*frame;
  ObjDetector.Draw(img_disp);

  DrawColDetViz(img_disp, VizObjs);

  // cv::imshow("camera", frame);
  cv::imshow("segmented", img_disp);

  if(!HandleKeyEvent())  ros::shutdown();
}
//-------------------------------------------------------------------------------------------


int main(int argc, char**argv)
{
  ros::init(argc, argv, "segm_obj_node");
  ros::NodeHandle node("~");
  std::string pkg_dir(".");
  std::string segm_obj_config("config/segm_obj1.yaml");
  std::string img_topic("/cameras/left_hand_camera/image");

  node.param("pkg_dir",pkg_dir,pkg_dir);
  node.param("segm_obj_config",segm_obj_config,segm_obj_config);
  node.param("img_topic",img_topic,img_topic);

  std::vector<TObjectDetectorParams> segm_obj_info;
  ReadFromYAML(segm_obj_info, pkg_dir+"/"+segm_obj_config);
  if(segm_obj_info.size()>0)  ObjDetector.Params()= segm_obj_info[0];

  // cv::namedWindow("camera", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("segmented", CV_WINDOW_AUTOSIZE);
  cv::setMouseCallback("segmented", OnMouse, NULL);

  ObjDetector.Init();

  SegmObjPub= node.advertise<ay_vision_msgs::SegmObj>(std::string("segm_obj"), 1);

  ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
  ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);

  // External visualization request:
  ros::Subscriber sub_viz= node.subscribe("viz", 1, &ColDetVizCallback);
  // Mask request:
  ros::Subscriber sub_mask= node.subscribe("mask", 1, &ColDetMaskCallback);
  ros::Subscriber sub_img= node.subscribe(img_topic, 1, &ImageCallback);

  ros::spin();

  return 0;
}
//-------------------------------------------------------------------------------------------