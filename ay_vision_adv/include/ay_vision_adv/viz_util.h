//-------------------------------------------------------------------------------------------
/*! \file    viz_util.h
    \brief   Visualization utility.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.20, 2018
*/
//-------------------------------------------------------------------------------------------
#ifndef viz_util_h
#define viz_util_h
//-------------------------------------------------------------------------------------------
#include <vector>
#include <opencv2/core/core.hpp>
#include "ay_vision_msgs/ColDetVizPrimitive.h"
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

void DrawColDetViz(cv::Mat &img, const std::vector<ay_vision_msgs::ColDetVizPrimitive> &objects);

void WriteToYAML(const std::vector<ay_vision_msgs::ColDetVizPrimitive> &viz_objs, const std::string &file_name, const std::string &section="VizPrimitive");
void ReadFromYAML(std::vector<ay_vision_msgs::ColDetVizPrimitive> &viz_objs, const std::string &file_name, const std::string &section="VizPrimitive");

//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // viz_util_h
//-------------------------------------------------------------------------------------------
