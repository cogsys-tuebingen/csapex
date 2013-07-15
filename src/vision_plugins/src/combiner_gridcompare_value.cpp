#include "combiner_gridcompare_value.h"

/// SYSTEM
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vision_evaluator::GridCompareValue, vision_evaluator::ImageCombiner)

using namespace vision_evaluator;
using namespace cv_grid;

GridCompareValue::GridCompareValue()
{
}

cv::Mat GridCompareValue::combine(const cv::Mat img1, const cv::Mat mask1, const cv::Mat img2, const cv::Mat mask2)
{
}

void GridCompareValue::fill(QBoxLayout *layout)
{
    GridCompare::fill(layout);
}
