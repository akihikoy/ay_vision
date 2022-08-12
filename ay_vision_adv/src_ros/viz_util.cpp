//-------------------------------------------------------------------------------------------
/*! \file    viz_util.cpp
    \brief   Visualization utility.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.20, 2018
*/
//-------------------------------------------------------------------------------------------
#include "ay_vision_adv/viz_util.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;

void DrawColDetViz(cv::Mat &img, const std::vector<ay_vision_msgs::ColDetVizPrimitive> &objects)
{
  for(std::vector<ay_vision_msgs::ColDetVizPrimitive>::const_iterator itr(objects.begin()),itr_end(objects.end());
      itr!=itr_end; ++itr)
  {
    cv::Scalar col= CV_RGB(itr->color.r,itr->color.g,itr->color.b);
    const double &lw= itr->line_width;
    std::vector<std::vector<cv::Point> >  points(1);
    cv::Mat mask;
    switch(itr->type)
    {
    case ay_vision_msgs::ColDetVizPrimitive::LINE :
      cv::line(img, cv::Point2d(itr->param[0],itr->param[1]), cv::Point2d(itr->param[2],itr->param[3]), col, lw);
      break;
    case ay_vision_msgs::ColDetVizPrimitive::CIRCLE :
      cv::circle(img, cv::Point2d(itr->param[0],itr->param[1]), itr->param[2], col, lw);
      break;
    case ay_vision_msgs::ColDetVizPrimitive::RECTANGLE :
      cv::rectangle(img, cv::Rect(itr->param[0],itr->param[1],itr->param[2],itr->param[3]), col, lw);
      break;
    case ay_vision_msgs::ColDetVizPrimitive::POLYGON :
      points[0].resize(itr->param.size()/2);
      for(int i(0),i_end(itr->param.size()/2); i<i_end; ++i)
        points[0][i]= cv::Point(itr->param[2*i],itr->param[2*i+1]);
      cv::polylines(img, points, /*isClosed=*/false, col, lw);
      break;
    case ay_vision_msgs::ColDetVizPrimitive::FILLED_POLY :
      points[0].resize(itr->param.size()/2);
      for(int i(0),i_end(itr->param.size()/2); i<i_end; ++i)
        points[0][i]= cv::Point(itr->param[2*i],itr->param[2*i+1]);
      cv::fillPoly(img, points, col);
      break;
    case ay_vision_msgs::ColDetVizPrimitive::REVERSED_POLY :
      mask.create(img.size(), CV_8UC1);
      mask.setTo(1);
      points[0].resize(itr->param.size()/2);
      for(int i(0),i_end(itr->param.size()/2); i<i_end; ++i)
        points[0][i]= cv::Point(itr->param[2*i],itr->param[2*i+1]);
      cv::fillPoly(mask, points, 0);
      img.setTo(col, mask);
      break;
    default:
      std::cerr<<"Unknown type:"<<itr->type<<std::endl;
      return;
    }
  }
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------

