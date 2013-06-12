#include "filter_undistort.h"


/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_transformation::Undistort, vision_evaluator::Filter)

using namespace image_transformation;

Undistort::Undistort()
{
}

void Undistort::filter(cv::Mat &img, cv::Mat &mask)
{

}

void Undistort::insert(QBoxLayout *parent)
{

}

void Undistort::update()
{

}
