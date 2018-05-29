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

struct TObjectDetectorParams
{
  // For white detector:
  int WhiteSMax;
  int WhiteVMin;
  int NErode1;
  int NDilate1;

  // For objects-on-white detector:
  int ThreshS;
  int ThreshV;
  int NErode2;
  int NDilate2;
  int RectLenMin;
  int RectLenMax;
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
