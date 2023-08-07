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

void WriteToYAML(const std::vector<ay_vision_msgs::ColDetVizPrimitive> &viz_objs, const std::string &file_name, const std::string &section)
{
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  fs<<section<<"[";
  for(std::vector<ay_vision_msgs::ColDetVizPrimitive>::const_iterator itr(viz_objs.begin()),itr_end(viz_objs.end()); itr!=itr_end; ++itr)
  {
    fs<<"{";
    #define PROC_VAR(x)  fs<<#x<<itr->x;
    PROC_VAR(type       );
    PROC_VAR(color      );
    PROC_VAR(param      );
    PROC_VAR(line_width );
    fs<<"}";
    #undef PROC_VAR
  }
  fs<<"]";
  fs.release();
}
//-------------------------------------------------------------------------------------------

void ReadFromYAML(std::vector<ay_vision_msgs::ColDetVizPrimitive> &viz_objs, const std::string &file_name, const std::string &section)
{
  viz_objs.clear();
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  cv::FileNode data= fs[section];
  for(cv::FileNodeIterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr)
  {
    ay_vision_msgs::ColDetVizPrimitive y;
    #define PROC_VAR(x)  if(!(*itr)[#x].empty())  (*itr)[#x]>>y.x;
    PROC_VAR(type       );
    PROC_VAR(color      );
    PROC_VAR(param      );
    PROC_VAR(line_width );
    #undef PROC_VAR
    viz_objs.push_back(y);
  }
  fs.release();
}
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
namespace cv
{
//-------------------------------------------------------------------------------------------

void write(cv::FileStorage &fs, const cv::String&, const std_msgs::ColorRGBA &x)
{
  #define PROC_VAR(v)  fs<<#v<<x.v;
  fs<<"{";
  PROC_VAR(r);
  PROC_VAR(g);
  PROC_VAR(b);
  PROC_VAR(a);
  fs<<"}";
  #undef PROC_VAR
}
//-------------------------------------------------------------------------------------------
void read(const cv::FileNode &data, std_msgs::ColorRGBA &x, const std_msgs::ColorRGBA &default_value)
{
  #define PROC_VAR(v)  if(!data[#v].empty())  data[#v]>>x.v;
  PROC_VAR(r);
  PROC_VAR(g);
  PROC_VAR(b);
  PROC_VAR(a);
  #undef PROC_VAR
}
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // cv
//-------------------------------------------------------------------------------------------
