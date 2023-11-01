//-------------------------------------------------------------------------------------------
/*! \file    segm_obj.h
    \brief   Segment objects on a plate of specific color (e.g. white).
             Segmentation is based on non-white color detection.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    May.17, 2017

Based on:
  testl/cv/segment_obj_simple1b.cpp
*/
//-------------------------------------------------------------------------------------------
#ifndef segm_obj_h
#define segm_obj_h
//-------------------------------------------------------------------------------------------
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

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
    int h_min=0, int h_max=255,
    int s_min=0, int s_max=20,
    int v_min=100, int v_max=255,
    int n_dilate=1, int n_erode=1);

// Make a mask from biggest contour.
void MakeBiggestContourMask(const std::vector<std::vector<cv::Point> > &contours,
    cv::Mat &mask, bool convex=false, int fill_value=1);

//-------------------------------------------------------------------------------------------

struct TObjectDetectorParams
{
  // For white detector:
  int WhiteHMin;
  int WhiteSMin;
  int WhiteVMin;
  int WhiteHMax;
  int WhiteSMax;
  int WhiteVMax;
  int NErode1;
  int NDilate1;

  // For objects-on-white detector:
  int ObjHMin;
  int ObjSMin;
  int ObjVMin;
  int ObjHMax;
  int ObjSMax;
  int ObjVMax;
  int NErode20;
  int NErode2;
  int NDilate2;
  int RectLenMin;
  int RectLenMax;
  int AreaMin;
  int AreaMax;
  double ContourApproxEps;  // Contour approximation. If zero, no approximation.

  TObjectDetectorParams();
};
void WriteToYAML(const std::vector<TObjectDetectorParams> &blob_params, const std::string &file_name);
void ReadFromYAML(std::vector<TObjectDetectorParams> &blob_params, const std::string &file_name);
//-------------------------------------------------------------------------------------------

class TObjectDetector
{
public:
  void Init();
  void Step(const cv::Mat &frame);
  void Draw(cv::Mat &frame);

  TObjectDetectorParams& Params()  {return params_;}
  const TObjectDetectorParams& Params() const {return params_;}

  const cv::Mat& WhiteMask() const {return mask_white_biggest_;}
  const cv::Mat& ObjectMask() const {return mask_objects_;}
  const std::vector<std::vector<cv::Point> >& Contours() const {return contours_obj_;}

private:
  TObjectDetectorParams params_;

  cv::Mat mask_white_biggest_;
  cv::Mat mask_objects_;
  std::vector<std::vector<cv::Point> > contours_obj_;
};
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // segm_obj_h
//-------------------------------------------------------------------------------------------
