//-------------------------------------------------------------------------------------------
/*! \file    segm_obj.cpp
    \brief   Segment objects on a plate of specific color (e.g. white).
             Segmentation is based on non-white color detection.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.17, 2017

Based on:
  testl/cv/segment_obj_simple1b.cpp
*/
//-------------------------------------------------------------------------------------------
#include "ay_vision_adv/segm_obj.h"
#include "ay_vision/vision_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/imgproc/imgproc.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;

/*Find contours of white areas.
  frame: Input image.
  frame_white: Detected white image.
  contours: Found contours.
  {h,s,v}_min, {h,s,v}_max: Thresholds of {h,s,v}-minimum and {h,s,v}-maximum of HSV.
  n_dilate, n_erode: dilate and erode filter parameters before detecting contours.
*/
void FindWhiteContours2(
    const cv::Mat &frame,
    cv::Mat &frame_white,
    std::vector<std::vector<cv::Point> > &contours,
    int h_min, int h_max, int s_min, int s_max, int v_min, int v_max,
    int n_dilate, int n_erode)
{
  cv::Mat frame_hsv;

  // White detection
  cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);
  cv::inRange(frame_hsv, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), frame_white);

  if(n_dilate>0)  cv::dilate(frame_white,frame_white,cv::Mat(),cv::Point(-1,-1), n_dilate);
  if(n_erode>0)   cv::erode(frame_white,frame_white,cv::Mat(),cv::Point(-1,-1), n_erode);

  // Contour detection
  cv::findContours(frame_white, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
}
//-------------------------------------------------------------------------------------------

// Make a mask from biggest contour.
void MakeBiggestContourMask(const std::vector<std::vector<cv::Point> > &contours,
    cv::Mat &mask, bool convex, int fill_value)
{
  if(contours.size()==0)  return;
  double a(0.0),a_max(0.0), i_max(0);
  for(int i(0),i_end(contours.size()); i<i_end; ++i)
  {
    a= cv::contourArea(contours[i],false);
    if(a>a_max)  {a_max= a;  i_max= i;}
  }
  if(!convex)
    cv::drawContours(mask, contours, i_max, fill_value, /*thickness=*/-1);
  else
  {
    std::vector<std::vector<cv::Point> > hull(1);
    cv::convexHull(contours[i_max], hull[0], /*clockwise=*/true);
    cv::drawContours(mask, hull, 0, fill_value, /*thickness=*/-1);
  }
}
//-------------------------------------------------------------------------------------------

TObjectDetectorParams::TObjectDetectorParams()
{
  // For white detector:
  WhiteHMin= 0;
  WhiteSMin= 0;
  WhiteVMin= 100;
  WhiteHMax= 255;
  WhiteSMax= 20;
  WhiteVMax= 255;
  NErode1= 1;
  NDilate1= 1;

  // For objects-on-white detector:
  ObjHMin= 0;
  ObjSMin= 30;
  ObjVMin= 0;
  ObjHMax= 255;
  ObjSMax= 255;
  ObjVMax= 224;
  NErode20= 0;
  NErode2= 1;
  NDilate2= 2;
  RectLenMin= 40;
  RectLenMax= 400;
  AreaMin= 250;
  AreaMax= 0;
  ContourApproxEps= 3.0;  // Contour approximation. If zero, no approximation.
}
//-------------------------------------------------------------------------------------------

void WriteToYAML(const std::vector<TObjectDetectorParams> &params, const std::string &file_name)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs<<"ObjectDetector"<<"[";
  for(std::vector<TObjectDetectorParams>::const_iterator itr(params.begin()),itr_end(params.end()); itr!=itr_end; ++itr)
  {
    fs<<"{";
    #define PROC_VAR(x)  fs<<#x<<itr->x;
    PROC_VAR(WhiteHMin  );
    PROC_VAR(WhiteSMin  );
    PROC_VAR(WhiteVMin  );
    PROC_VAR(WhiteHMax  );
    PROC_VAR(WhiteSMax  );
    PROC_VAR(WhiteVMax  );
    PROC_VAR(NErode1    );
    PROC_VAR(NDilate1   );
    PROC_VAR(ObjHMin    );
    PROC_VAR(ObjSMin    );
    PROC_VAR(ObjVMin    );
    PROC_VAR(ObjHMax    );
    PROC_VAR(ObjSMax    );
    PROC_VAR(ObjVMax    );
    PROC_VAR(NErode20   );
    PROC_VAR(NErode2    );
    PROC_VAR(NDilate2   );
    PROC_VAR(RectLenMin );
    PROC_VAR(RectLenMax );
    PROC_VAR(AreaMin    );
    PROC_VAR(AreaMax    );
    PROC_VAR(ContourApproxEps );
    fs<<"}";
    #undef PROC_VAR
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<TObjectDetectorParams> &params, const std::string &file_name)
{
  params.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs["ObjectDetector"];
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    TObjectDetectorParams cf;
    #define PROC_VAR(x)  if(!(*itr)[#x].empty())  (*itr)[#x]>>cf.x;
    PROC_VAR(WhiteHMin  );
    PROC_VAR(WhiteSMin  );
    PROC_VAR(WhiteVMin  );
    PROC_VAR(WhiteHMax  );
    PROC_VAR(WhiteSMax  );
    PROC_VAR(WhiteVMax  );
    PROC_VAR(NErode1    );
    PROC_VAR(NDilate1   );
    PROC_VAR(ObjHMin    );
    PROC_VAR(ObjSMin    );
    PROC_VAR(ObjVMin    );
    PROC_VAR(ObjHMax    );
    PROC_VAR(ObjSMax    );
    PROC_VAR(ObjVMax    );
    PROC_VAR(NErode20   );
    PROC_VAR(NErode2    );
    PROC_VAR(NDilate2   );
    PROC_VAR(RectLenMin );
    PROC_VAR(RectLenMax );
    PROC_VAR(AreaMin    );
    PROC_VAR(AreaMax    );
    PROC_VAR(ContourApproxEps );
    #undef PROC_VAR
    params.push_back(cf);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
// class TObjectDetector
//-------------------------------------------------------------------------------------------

void TObjectDetector::Init()
{
}
//-------------------------------------------------------------------------------------------

void TObjectDetector::Step(const cv::Mat &frame)
{
  cv::Mat mask_white;
  std::vector<std::vector<cv::Point> > contours_w;
  FindWhiteContours2(frame, mask_white, contours_w,
        params_.WhiteHMin, params_.WhiteHMax,
        params_.WhiteSMin, params_.WhiteSMax,
        params_.WhiteVMin, params_.WhiteVMax,
        /*n_dilate=*/params_.NDilate1, /*n_erode=*/params_.NErode1);

  // Make a mask of biggest contour:
  mask_white_biggest_.create(mask_white.size(), CV_8UC1);
  mask_white_biggest_.setTo(0);
  MakeBiggestContourMask(contours_w, mask_white_biggest_, /*convex=*/true);

  // Detect objects-on-white
  cv::Mat frame_white, frame_white_hsv;
  frame.copyTo(frame_white, mask_white_biggest_);

  // Non-white detection
  cv::cvtColor(frame_white, frame_white_hsv, cv::COLOR_BGR2HSV);
  cv::inRange(frame_white_hsv, cv::Scalar(params_.ObjHMin, params_.ObjSMin, params_.ObjVMin),
              cv::Scalar(params_.ObjHMax, params_.ObjSMax, params_.ObjVMax), mask_objects_);
  mask_objects_.setTo(0, 1-mask_white_biggest_);

  cv::erode(mask_objects_,mask_objects_,cv::Mat(),cv::Point(-1,-1), params_.NErode20);
  cv::dilate(mask_objects_,mask_objects_,cv::Mat(),cv::Point(-1,-1), params_.NDilate2);
  cv::erode(mask_objects_,mask_objects_,cv::Mat(),cv::Point(-1,-1), params_.NErode2);

  // Find object contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(mask_objects_, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  // Apply filters to object contours
  contours_obj_.clear();
  for(std::vector<std::vector<cv::Point> >::const_iterator
      citr(contours.begin()),citr_end(contours.end()); citr!=citr_end; ++citr)
  {
    cv::Rect bound= cv::boundingRect(*citr);
    if(bound.width<params_.RectLenMin || bound.width>params_.RectLenMax
      || bound.height<params_.RectLenMin || bound.height>params_.RectLenMax)  continue;
    double area= cv::contourArea(*citr,false);
    if(area<params_.AreaMin || (params_.AreaMax>0 && area>params_.AreaMax))  continue;
    if(params_.ContourApproxEps>0.0)
    {
      std::vector<cv::Point> capprox;
      cv::approxPolyDP(*citr, capprox, params_.ContourApproxEps, /*closed=*/true);
      contours_obj_.push_back(capprox);
    }
    else
      contours_obj_.push_back(*citr);
  }
}
//-------------------------------------------------------------------------------------------

void TObjectDetector::Draw(cv::Mat &frame)
{
  cv::Mat &img_disp(frame);
  cv::Mat mask_objectss[3]= {128.0*mask_white_biggest_,128.0*mask_white_biggest_,128.0*mask_white_biggest_+128.0*mask_objects_}, mask_objectsc;
  cv::merge(mask_objectss,3,mask_objectsc);
  img_disp+= mask_objectsc;

  if(contours_obj_.size()>0)
  {
    for(int ic(0),ic_end(contours_obj_.size()); ic<ic_end; ++ic)
    {
      cv::drawContours(img_disp, contours_obj_, ic, CV_RGB(255,0,255), /*thickness=*/2, /*linetype=*/8);
      // cv::Rect bound= cv::boundingRect(contours_obj_[ic]);
      // cv::rectangle(img_disp, bound, cv::Scalar(0,0,255), 2);
    }
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

