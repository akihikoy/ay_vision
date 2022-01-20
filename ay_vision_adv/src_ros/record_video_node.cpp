//-------------------------------------------------------------------------------------------
/*! \file    record_video.cpp
    \brief   Open a video (USB camera, or stream), display and record it into a file.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Jun.21, 2018
*/
//-------------------------------------------------------------------------------------------
#include "ay_vision/vision_util.h"
#include "ay_cpp/sys_util.h"
//-------------------------------------------------------------------------------------------
#include "ay_vision_msgs/SetInt32.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
bool Running(true), Shutdown(false), DoCalibrate(false);
std::string *CurrentWin(NULL);

/*FPS control parameters.
  FrameSkip: Video image is processed every 1+FrameSkip frames.
             FrameSkip=0: processing every frame.
  TargetFPS: Video image is processed at this FPS.
             TargetFPS=0: processing every frame.
  NOTE: Do not set FrameSkip and TargetFPS simultaneously, which would be confusing.
  Pseudo code:
    for each frame f:
      Capture frame;
      if(FrameSkip<=0 || f%(FrameSkip+1)==0)  Process;
    In Process:
      ros::Rate rate(TargetFPS)
      for each frame:
        rate.sleep()
  TODO:FIXME: Currently TargetFPS is not configurable during run time, but
    changing TargetFPS in execution is useful.
*/
int FrameSkip(0);  // 0: no skip
double TargetFPS(0);  // 0: no FPS control

std::vector<TCameraInfo> CamInfo;
std::vector<TCameraRectifier> SingleCamRectifier;
std::vector<boost::function<void(cv::Mat&)> > CamRectifier;  // Functions to rectify camera images.
void DummyRectify(cv::Mat&) {}  // Do nothing function

std::map<std::string, TEasyVideoOut> VideoOut;

std::vector<cv::Mat> Frame;
std::vector<int64_t> CapTime;
std::vector<boost::shared_ptr<boost::mutex> > MutCamCapture;  // Mutex for capture
std::vector<boost::shared_ptr<boost::mutex> > MutFrameCopy;  // Mutex for Frame
struct TIMShowStuff
{
  boost::shared_ptr<boost::mutex> Mutex;
  cv::Mat Frame;
};
std::map<std::string, TIMShowStuff> IMShowStuff;

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
    CurrentWin= reinterpret_cast<std::string*>(data);
    std::cerr<<"CurrentWin: "<<*CurrentWin<<std::endl;
  }

  // if(flags!=0)  return;
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
}
//-------------------------------------------------------------------------------------------

// return if continue
bool HandleKeyEvent()
{
  // keyboard interface:
  char c(cv::waitKey(1));
  if(c=='\x1b'||c=='q') return false;
  else if(c=='W')
  {
    for(std::map<std::string, TEasyVideoOut>::iterator itr(VideoOut.begin()),itr_end(VideoOut.end()); itr!=itr_end; ++itr)
      itr->second.Switch();
  }
  else if(c==' ')
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
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

bool SetFrameSkip(ay_vision_msgs::SetInt32::Request &req, ay_vision_msgs::SetInt32::Response &res)
{
  std::cerr<<"Setting frame skip as "<<req.data<<"..."<<std::endl;
  FrameSkip= req.data;
  res.result= true;
  return true;
}
//-------------------------------------------------------------------------------------------

void ExecRectifyImg(int i_cam)
{
  TCameraInfo &info(CamInfo[i_cam]);
  cv::Mat frame;
  int64_t t_cap(0);
  ros::Rate rate(TargetFPS>0.0?TargetFPS:1);
  while(!Shutdown)
  {
    if(Running)
    {
      if(CapTime[i_cam]==t_cap)
      {
        usleep(10*1000);
        continue;
      }
      if(TargetFPS>0.0)  rate.sleep();

      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
        t_cap= CapTime[i_cam];
      }
      CamRectifier[i_cam](frame);

      VideoOut[info.Name].Step(frame);
      VideoOut[info.Name].VizRec(frame);

      {
        // cv::imshow(info.Name, frame);
        boost::mutex::scoped_lock lock(*IMShowStuff[info.Name].Mutex);
        frame.copyTo(IMShowStuff[info.Name].Frame);
      }

      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

cv::Mat Capture(cv::VideoCapture &cap, int i_cam, bool rectify)
{
  cv::Mat frame;
  {
    boost::mutex::scoped_lock lock(*MutCamCapture[i_cam]);
    while(!cap.read(frame))
    {
      if(CapWaitReopen(CamInfo[i_cam],cap)) continue;
      else  return cv::Mat();
    }
  }
  if(CamInfo[i_cam].CapWidth!=CamInfo[i_cam].Width || CamInfo[i_cam].CapHeight!=CamInfo[i_cam].Height)
    cv::resize(frame,frame,cv::Size(CamInfo[i_cam].Width,CamInfo[i_cam].Height));
  Rotate90N(frame,frame,CamInfo[i_cam].NRotate90);
  if(rectify)  CamRectifier[i_cam](frame);
  return frame;
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "record_video_node");
  ros::NodeHandle node("~");
  std::string pkg_dir(".");
  std::string cam_config("config/usbcam4g1.yaml");
  std::string vout_base("/tmp/vout-");

  node.param("pkg_dir",pkg_dir,pkg_dir);
  node.param("cam_config",cam_config,cam_config);
  node.param("vout_base",vout_base,vout_base);
  node.param("frame_skip",FrameSkip,FrameSkip);
  node.param("target_fps",TargetFPS,TargetFPS);
  std::cerr<<"pkg_dir: "<<pkg_dir<<std::endl;
  std::cerr<<"cam_config: "<<cam_config<<std::endl;

  ReadFromYAML(CamInfo, pkg_dir+"/"+cam_config);

  std::vector<cv::VideoCapture> cap(CamInfo.size());
  SingleCamRectifier.resize(CamInfo.size());
  CamRectifier.resize(CamInfo.size());
  Frame.resize(CamInfo.size());
  CapTime.resize(CamInfo.size());
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    TCameraInfo &info(CamInfo[i_cam]);
    if(!CapOpen(info, cap[i_cam]))  return -1;
    MutCamCapture.push_back(boost::shared_ptr<boost::mutex>(new boost::mutex));
    MutFrameCopy.push_back(boost::shared_ptr<boost::mutex>(new boost::mutex));

    if(info.Rectification)
    {
      // Setup rectification
      // NOTE: The rectification of StereoInfo overwrites this rectification.
      cv::Size size_in(info.Width,info.Height), size_out(info.Width,info.Height);
      SingleCamRectifier[i_cam].Setup(info.K, info.D, info.R, size_in, info.Alpha, size_out);
      CamRectifier[i_cam]= boost::bind(&TCameraRectifier::Rectify, SingleCamRectifier[i_cam], _1, /*border=*/cv::Scalar(0,0,0));
    }
    else
    {
      CamRectifier[i_cam]= &DummyRectify;
    }
  }
  std::cerr<<"Opened camera(s)"<<std::endl;

  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  {
    cv::namedWindow(CamInfo[j].Name,1);
    cv::setMouseCallback(CamInfo[j].Name, OnMouse, &CamInfo[j].Name);
    IMShowStuff[CamInfo[j].Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
    VideoOut[CamInfo[j].Name].SetfilePrefix(vout_base+CamInfo[j].Name);
  }

  ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
  ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);
  ros::ServiceServer srv_set_frame_skip= node.advertiseService("set_frame_skip", &SetFrameSkip);

  int show_fps(0);

  // Dummy capture.
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    cap[i_cam] >> Frame[i_cam];
    CapTime[i_cam]= GetCurrentTimeL();
  }

  std::vector<boost::shared_ptr<boost::thread> > th_rectify;
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
    th_rectify.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecRectifyImg,j))));

  // ros::Rate loop_rate(5);  // 5 Hz
  for(int f(0);ros::ok();++f)
  {
    if(Running)
    {
      // Capture from cameras:
      for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
      {
        cv::Mat frame= Capture(cap[i_cam], i_cam, /*rectify=*/false);
        if(FrameSkip<=0 || f%(FrameSkip+1)==0)
        {
          boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
          Frame[i_cam]= frame;
          CapTime[i_cam]= GetCurrentTimeL();
        }
      }

      // Show windows
      for(std::map<std::string, TIMShowStuff>::iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
      {
        boost::mutex::scoped_lock lock(*itr->second.Mutex);
        if(itr->second.Frame.total()>0)
          cv::imshow(itr->first, itr->second.Frame);
      }

      // usleep(10*1000);
      if(show_fps==0)
      {
        std::cerr<<"FPS: "<<VideoOut.begin()->second.FPS()<<std::endl;
        show_fps=VideoOut.begin()->second.FPS()*4;
      }
      --show_fps;

    }  // Running
    else
    {
      usleep(200*1000);
    }

    if(!HandleKeyEvent())  break;

    ros::spinOnce();
  }
  Shutdown= true;
  for(int j(0),j_end(th_rectify.size());j<j_end;++j)
    th_rectify[j]->join();

  usleep(500*1000);

  return 0;
}
//-------------------------------------------------------------------------------------------
